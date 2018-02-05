// SPDX-License-Identifier: GPL-2.0
/**
 * Driver for Intel PMC USB mux control
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include <linux/platform_device.h>
#include <linux/usb/typec_mux.h>
#include <linux/usb/role.h>
#include <linux/property.h>
#include <linux/module.h>

#include <asm/intel_pmc_ipc.h>

#define PMC_USBC_CMD		0xa7

/* "Usage" OOB Message field values */
enum {
	PMC_USB_CONNECT,
	PMC_USB_DISCONNECT,
	PMC_USB_SAFE_MODE,
	PMC_USB_ALT_MODE,
	PMC_USB_ALT_DIRECT_MODE,
};

#define PMC_USB_MSG_USB2_PORT_SHIFT	0
#define PMC_USB_MSG_USB3_PORT_SHIFT	4
#define PMC_USB_MSG_UFP_SHIFT		4
#define PMC_USB_MSG_ORI_SHIFT		5

struct pmc_usb {
	struct typec_switch typec_sw;
	struct usb_role_switch *usb_sw;

	u8 usb2_port;
	u8 usb3_port;

	u8 msg[8]; /* OOB Message payload */
};

static int pmc_usb_command(struct pmc_usb *pmc, u8 usage, u32 len)
{
	u8 response[3];
	u8 status;

	pmc->msg[0] |= usage;
	pmc->msg[0] |= (pmc->usb3_port << PMC_USB_MSG_USB3_PORT_SHIFT);

	if (usage < PMC_USB_ALT_MODE)
		pmc->msg[1] |= (pmc->usb2_port << PMC_USB_MSG_USB2_PORT_SHIFT);

	/*
	 * Error bit will always be 0 with USBC command.
	 * Status has to be checked from the return message.
	 */
	intel_pmc_ipc_command(PMC_USBC_CMD, 0, pmc->msg, len,
			      (u32 *)response, sizeof(response));

	switch (usage) {
	case PMC_USB_CONNECT:
	case PMC_USB_DISCONNECT:
	case PMC_USB_ALT_DIRECT_MODE:
		status = response[2];
		break;
	default:
		status = response[1];
		break;
	}

	if (status) {
		if (status & BIT(1))
			return -EIO;
		return -EBUSY;
	}

	return 0;
}

static int pmc_usb_disconnect(struct pmc_usb *pmc)
{
	if (!pmc->msg[0])
		return 0;

	pmc->msg[1] = 0;

	pmc_usb_command(pmc, PMC_USB_DISCONNECT, 2);
	memset(pmc->msg, 0, sizeof(pmc->msg));

	return 0;
}

static int pmc_usb_set_orientation(struct typec_switch *sw,
				   enum typec_orientation orientation)
{
	struct pmc_usb *pmc = container_of(sw, struct pmc_usb, typec_sw);

	if (orientation == TYPEC_ORIENTATION_NONE)
		return pmc_usb_disconnect(pmc);

	pmc->msg[1] |= (orientation - 1) << PMC_USB_MSG_ORI_SHIFT;

	return 0;
}

static int pmc_usb_set_role(struct device *dev, enum usb_role role)
{
	struct pmc_usb *pmc = dev_get_drvdata(dev);
	u8 fields;
	int ret;

	if (role == USB_ROLE_NONE)
		return pmc_usb_disconnect(pmc);

	/* Always putting the state to disconnected before setting the role. */
	fields = pmc->msg[1];
	pmc->msg[1] = 0;
	ret = pmc_usb_command(pmc, PMC_USB_DISCONNECT, 2);
	if (ret)
		return ret;

	pmc->msg[1] = fields;
	pmc->msg[1] |= (role - 1) << PMC_USB_MSG_UFP_SHIFT;

	return pmc_usb_command(pmc, PMC_USB_CONNECT, 2);
}

static int pmc_usb_probe(struct platform_device *pdev)
{
	struct usb_role_switch_desc desc;
	struct pmc_usb *pmc;
	int ret;

	pmc = devm_kzalloc(&pdev->dev, sizeof(*pmc), GFP_KERNEL);
	if (!pmc)
		return -ENOMEM;

	pmc->typec_sw.dev = &pdev->dev;
	pmc->typec_sw.set = pmc_usb_set_orientation;

	ret = device_property_read_u8(&pdev->dev, "usb2_port", &pmc->usb2_port);
	if (ret)
		return ret;

	ret = device_property_read_u8(&pdev->dev, "usb3_port", &pmc->usb3_port);
	if (ret)
		return ret;

	ret = typec_switch_register(&pmc->typec_sw);
	if (ret)
		return ret;

	memset(&desc, 0, sizeof(desc));
	desc.set = pmc_usb_set_role;

	pmc->usb_sw = usb_role_switch_register(&pdev->dev, &desc);
	if (IS_ERR(pmc->usb_sw)) {
		typec_switch_unregister(&pmc->typec_sw);
		return PTR_ERR(pmc->usb_sw);
	}

	platform_set_drvdata(pdev, pmc);

	return 0;
}

static int pmc_usb_remove(struct platform_device *pdev)
{
	struct pmc_usb *pmc = platform_get_drvdata(pdev);

	usb_role_switch_unregister(pmc->usb_sw);
	typec_switch_unregister(&pmc->typec_sw);

	return 0;
}

static struct platform_driver pmc_usb_driver = {
	.driver = {
		.name		= "intel_pmc_usb",
	},
	.probe			= pmc_usb_probe,
	.remove			= pmc_usb_remove,
};

module_platform_driver(pmc_usb_driver);

MODULE_AUTHOR("Heikki Krogerus <heikki.krogerus@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Intel PMC USB mux driver");
