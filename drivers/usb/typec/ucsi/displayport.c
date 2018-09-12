// SPDX-License-Identifier: GPL-2.0
/*
 * UCSI DisplayPort driver
 *
 * Copyright (C) 2018, Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/usb/typec_dp.h>
#include <linux/usb/pd_vdo.h>

#include "ucsi.h"

#define UCSI_CMD_SET_NEW_CAM(_con_num_, _enter_, _cam_, _am_)		\
	 UCSI_SET_NEW_CAM | _con_num_ << 16 | _enter_ << 23 |		\
	 _cam_ << 24 | (u64)_am_ << 32

struct ucsi_dp {
	struct typec_displayport_data data;
	struct ucsi_connector *con;
	struct typec_altmode *alt;
	struct work_struct work;

	bool override;
	bool initialized;

	u32 header;
	u32 *vdo_data;
	u8 vdo_size;
};

static int ucsi_displayport_offset(struct ucsi_connector *con)
{
	int i;

	for (i = 0; i < UCSI_MAX_ALTMODES; i++)
		if (con->port_altmode[i]->svid == USB_TYPEC_DP_SID &&
		    con->port_altmode[i]->mode == USB_TYPEC_DP_MODE)
			return i;

	return -ENOENT;
}

static int ucsi_displayport_enter(struct typec_altmode *alt)
{
	struct ucsi_dp *dp = typec_altmode_get_drvdata(alt);

	printk("%s\n", __func__);

	mutex_lock(&dp->con->lock);

	if (!dp->override && dp->initialized) {
		mutex_unlock(&dp->con->lock);
		return -EOPNOTSUPP;
	}

	/*
	 * We can't send the New CAM command yet to the PPM as it needs the
	 * configuration value as well. Pretending that we have now entered the
	 * mode, and letting the alt mode driver continue.
	 */

	dp->header = VDO(USB_TYPEC_DP_SID, 1, CMD_ENTER_MODE);
	dp->header |= VDO_OPOS(USB_TYPEC_DP_MODE);
	dp->header |= VDO_CMDT(CMDT_RSP_ACK);

	dp->vdo_data = NULL;
	dp->vdo_size = 1;

	schedule_work(&dp->work);

	mutex_unlock(&dp->con->lock);

	return 0;
}

static int ucsi_displayport_exit(struct typec_altmode *alt)
{
	struct ucsi_dp *dp = typec_altmode_get_drvdata(alt);
	struct ucsi_control ctrl;
	int ret = 0;
	int i;

	mutex_lock(&dp->con->lock);

	if (!dp->override) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}

	i = ucsi_displayport_offset(dp->con);
	if (i < 0)
		goto out_unlock;

	ctrl.raw_cmd = UCSI_CMD_SET_NEW_CAM(dp->con->num, 0, i, 0);
	ret = ucsi_send_command(dp->con->ucsi, &ctrl, NULL, 0);
	if (ret < 0)
		goto out_unlock;

	dp->header = VDO(USB_TYPEC_DP_SID, 1, CMD_EXIT_MODE);
	dp->header |= VDO_OPOS(USB_TYPEC_DP_MODE);
	dp->header |= VDO_CMDT(CMDT_RSP_ACK);

	dp->vdo_data = NULL;
	dp->vdo_size = 1;

	schedule_work(&dp->work);

out_unlock:
	mutex_unlock(&dp->con->lock);

	return ret;
}

/*
 * We do not actually have access to the Status Update VDO, so we have to guess
 * things.
 */
static int ucsi_displayport_status_update(struct ucsi_dp *dp)
{
	u32 cap = dp->alt->vdo;

	dp->data.status = DP_STATUS_ENABLED;

	/*
	 * If pin assignement D is supported, claiming always
	 * that Multi-function is preferred.
	 */
	if (DP_CAP_CAPABILITY(cap) & DP_CAP_UFP_D) {
		dp->data.status |= DP_STATUS_CON_UFP_D;

		if (DP_CAP_UFP_D_PIN_ASSIGN(cap) & BIT(DP_PIN_ASSIGN_D))
			dp->data.status |= DP_STATUS_PREFER_MULTI_FUNC;
	} else {
		dp->data.status |= DP_STATUS_CON_DFP_D;

		if (DP_CAP_DFP_D_PIN_ASSIGN(cap) & BIT(DP_PIN_ASSIGN_D))
			dp->data.status |= DP_STATUS_PREFER_MULTI_FUNC;
	}

	dp->vdo_data = &dp->data.status;
	dp->vdo_size = 2;

	return 0;
}

static int ucsi_displayport_configure(struct ucsi_dp *dp)
{
	struct ucsi_control ctrl;
	int i;

	i = ucsi_displayport_offset(dp->con);
	if (i < 0)
		return i;

	printk("%s: offset %d, conf 0x%x\n", __func__, i, dp->data.conf);

	if (!dp->override) {
		dp->initialized = true;
		return 0;
	}

	ctrl.raw_cmd = UCSI_CMD_SET_NEW_CAM(dp->con->num, 1, i, dp->data.conf);

	return ucsi_send_command(dp->con->ucsi, &ctrl, NULL, 0);
}

static int ucsi_displayport_vdm(struct typec_altmode *alt,
				u32 header, const u32 *data, int count)
{
	struct ucsi_dp *dp = typec_altmode_get_drvdata(alt);
	int cmd_type = PD_VDO_CMDT(header);
	int cmd = PD_VDO_CMD(header);
	struct typec_altmode *pdev;

	printk("%s header 0x%x, override %d, initialized %d\n", __func__,
		header, dp->override, dp->initialized);

	mutex_lock(&dp->con->lock);

	if (!dp->override && dp->initialized) {
		mutex_unlock(&dp->con->lock);
		return -EOPNOTSUPP;
	}

	pdev = typec_match_altmode(dp->con->partner_altmode, -1,
				   alt->svid, alt->mode);

	switch (cmd_type) {
	case CMDT_INIT:
		dp->header = VDO(USB_TYPEC_DP_SID, 1, cmd);
		dp->header |= VDO_OPOS(USB_TYPEC_DP_MODE);

		switch (cmd) {
		case DP_CMD_STATUS_UPDATE:
			if (ucsi_displayport_status_update(dp))
				dp->header |= VDO_CMDT(CMDT_RSP_NAK);
			else
				dp->header |= VDO_CMDT(CMDT_RSP_ACK);
			break;
		case DP_CMD_CONFIGURE:
			dp->data.conf = *data;
			if (ucsi_displayport_configure(dp)) {
				dp->header |= VDO_CMDT(CMDT_RSP_NAK);
			} else {
				dp->header |= VDO_CMDT(CMDT_RSP_ACK);
				typec_altmode_update_active(pdev, true);
			}
			break;
		default:
			dp->header |= VDO_CMDT(CMDT_RSP_ACK);
			break;
		}

		schedule_work(&dp->work);
		break;
	default:
		break;
	}

	mutex_unlock(&dp->con->lock);

	return 0;
}

static const struct typec_altmode_ops ucsi_displayport_ops = {
	.enter = ucsi_displayport_enter,
	.exit = ucsi_displayport_exit,
	.vdm = ucsi_displayport_vdm,
};

static void ucsi_displayport_work(struct work_struct *work)
{
	struct ucsi_dp *dp = container_of(work, struct ucsi_dp, work);
	int ret;

	mutex_lock(&dp->con->lock);

	printk("%s header 0x%x\n", __func__, dp->header);

	ret = typec_altmode_vdm(dp->alt, dp->header,
				dp->vdo_data, dp->vdo_size);
	if (ret)
		dev_err(&dp->alt->dev, "VDM 0x%x failed\n", dp->header);

	dp->vdo_data = NULL;
	dp->vdo_size = 0;
	dp->header = 0;

	mutex_unlock(&dp->con->lock);
}

struct typec_altmode *ucsi_register_displayport(struct ucsi_connector *con,
						bool override,
						struct typec_altmode_desc *desc)
{
	u8 all_assignments = BIT(DP_PIN_ASSIGN_C) | BIT(DP_PIN_ASSIGN_D) |
			     BIT(DP_PIN_ASSIGN_E);
	struct typec_altmode *alt;
	struct ucsi_dp *dp;

	/* We can't rely on the firmware with the capabilities. */
	desc->vdo |= DP_CAP_DP_SIGNALING | DP_CAP_RECEPTACLE;

	/* Claiming that we support all pin assignments */
	desc->vdo |= all_assignments << 8;
	desc->vdo |= all_assignments << 16;

	alt = typec_port_register_altmode(con->port, desc);
	if (IS_ERR(alt))
		return alt;

	dp = devm_kzalloc(&alt->dev, sizeof(*dp), GFP_KERNEL);
	if (!dp) {
		typec_unregister_altmode(alt);
		return ERR_PTR(-ENOMEM);
	}

	INIT_WORK(&dp->work, ucsi_displayport_work);
	dp->override = override;
	dp->con = con;
	dp->alt = alt;

	/*
	 * Note. UCSI allows the PPM to only supply the details about the
	 * alternate modes without allowing them to be overridden. It means the
	 * user can not configure, or even exit or enter the modes, when
	 * alternate mode overriding is not supported.
	 *
	 * In that situation, the driver will still display the supported pin
	 * assignments and configuration, but any changes the user attempts to
	 * do will lead into failure with return value of -EOPNOTSUPP.
	 */

	alt->ops = &ucsi_displayport_ops;
	typec_altmode_set_drvdata(alt, dp);

	return alt;
}
