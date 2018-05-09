// SPDX-License-Identifier: GPL-2.0
/**
 * Bus for USB Type-C Alternate Modes
 *
 * Copyright (C) 2018 Intel Corporation
 * Author: Heikki Krogerus <heikki.krogerus@linux.intel.com>
 */

#include "bus.h"

/* -------------------------------------------------------------------------- */
/* Common API */

/**
 * typec_altmode_notify - Communication between the OS and alternate mode driver
 * @adev: Handle to the alternate mode
 * @conf: Alternate mode specific configuration value
 * @data: Alternate mode specific data
 *
 * The primary purpose for this function is to allow the alternate mode drivers
 * to tell which pin configuration has been negotiated with the partner. That
 * information will then be used for example to configure the muxes.
 * Communication to the other direction is also possible, and low level device
 * drivers can also send notifications to the alternate mode drivers. The actual
 * communication will be specific for every SVID.
 */
int typec_altmode_notify(struct typec_altmode *adev,
			 unsigned long conf, void *data)
{
	struct altmode *altmode;
	struct altmode *partner;
	int ret;

	/*
	 * All SVID specific configuration values must start from
	 * TYPEC_STATE_MODAL. The first values are reserved for the pin states
	 * defined in USB Type-C specification: TYPEC_STATE_USB and
	 * TYPEC_STATE_SAFE. We'll follow this rule even with modes that do not
	 * require pin reconfiguration for the sake of simplicity.
	 */
	if (conf < TYPEC_STATE_MODAL)
		return -EINVAL;

	if (!adev)
		return 0;

	altmode = to_altmode(adev);

	if (!altmode->partner)
		return -ENODEV;

	ret = typec_set_mode(typec_altmode2port(adev), (int)conf);
	if (ret)
		return ret;

	partner = altmode->partner;

	blocking_notifier_call_chain(is_typec_port(adev->dev.parent) ?
				     &altmode->nh : &partner->nh,
				     conf, data);

	if (partner->adev.ops && partner->adev.ops->notify)
		return partner->adev.ops->notify(&partner->adev, conf, data);

	return 0;
}
EXPORT_SYMBOL_GPL(typec_altmode_notify);

/**
 * typec_altmode_enter - Enter Mode
 * @adev: The alternate mode
 *
 * The alternate mode drivers use this function to enter mode. The port drivers
 * use this to inform the alternate mode drivers that their mode has been
 * entered successfully.
 */
int typec_altmode_enter(struct typec_altmode *adev)
{
	struct altmode *partner = to_altmode(adev)->partner;
	struct typec_altmode *pdev = &partner->adev;
	int ret;

	if (is_typec_port(adev->dev.parent)) {
		typec_altmode_update_active(pdev, pdev->mode, true);
		sysfs_notify(&pdev->dev.kobj, NULL, "active");
		goto enter_mode;
	}

	if (!pdev->active)
		return -EPERM;

	/* First moving to USB Safe State */
	ret = typec_set_mode(typec_altmode2port(adev), TYPEC_STATE_SAFE);
	if (ret)
		return ret;

	blocking_notifier_call_chain(&partner->nh, TYPEC_STATE_SAFE, NULL);

enter_mode:
	/* Enter Mode command */
	if (pdev->ops && pdev->ops->enter)
		pdev->ops->enter(pdev);

	return 0;
}
EXPORT_SYMBOL_GPL(typec_altmode_enter);

/**
 * typec_altmode_enter - Exit Mode
 * @adev: The alternate mode
 *
 * The alternate mode drivers use this function to exit mode. The port drivers
 * can also inform the alternate mode drivers with this function that the mode
 * was successfully exited.
 */
int typec_altmode_exit(struct typec_altmode *adev)
{
	struct altmode *partner = to_altmode(adev)->partner;
	struct typec_port *port = typec_altmode2port(adev);
	struct typec_altmode *pdev = &partner->adev;
	int ret;

	/* In case of port, just calling the driver and exiting */
	if (is_typec_port(adev->dev.parent)) {
		typec_altmode_update_active(pdev, pdev->mode, false);
		sysfs_notify(&pdev->dev.kobj, NULL, "active");

		if (pdev->ops && pdev->ops->exit)
			pdev->ops->exit(pdev);
		return 0;
	}

	/* Moving to USB Safe State */
	ret = typec_set_mode(port, TYPEC_STATE_SAFE);
	if (ret)
		return ret;

	blocking_notifier_call_chain(&partner->nh, TYPEC_STATE_SAFE, NULL);

	/* Exit Mode command */
	if (pdev->ops && pdev->ops->exit)
		pdev->ops->exit(pdev);

	/* Back to USB operation */
	ret = typec_set_mode(port, TYPEC_STATE_USB);
	if (ret)
		return ret;

	blocking_notifier_call_chain(&partner->nh, TYPEC_STATE_USB, NULL);

	return 0;
}
EXPORT_SYMBOL_GPL(typec_altmode_exit);

/**
 * typec_altmode_attention - Attention command
 * @adev: The alternate mode
 * @vdo: VDO for the Attention command
 *
 * Notifies the partner of @adev about Attention command.
 */
void typec_altmode_attention(struct typec_altmode *adev, const u32 vdo)
{
	struct typec_altmode *pdev = &to_altmode(adev)->partner->adev;

	if (pdev->ops && pdev->ops->attention)
		pdev->ops->attention(pdev, vdo);
}
EXPORT_SYMBOL_GPL(typec_altmode_attention);

/**
 * typec_altmode_vdm - Send Vendor Defined Messages (VDM) to the partner
 * @adev: Alternate mode handle
 * @header: VDM Header
 * @vdo: Array of Vendor Defined Data Objects
 * @count: Number of Data Objects
 *
 * The alternate mode drivers use this function for SVID specific communication
 * with the partner. The port drivers use it to deliver the Structured VDMs
 * received from the partners to the alternate mode drivers.
 */
int typec_altmode_vdm(struct typec_altmode *adev,
		      const u32 header, const u32 *vdo, int count)
{
	struct typec_altmode *pdev;
	struct altmode *altmode;

	if (!adev)
		return 0;

	altmode = to_altmode(adev);

	if (!altmode->partner)
		return -ENODEV;

	pdev = &altmode->partner->adev;

	if (pdev->ops && pdev->ops->vdm)
		return pdev->ops->vdm(pdev, header, vdo, count);

	return 0;
}
EXPORT_SYMBOL_GPL(typec_altmode_vdm);

void typec_altmode_register_ops(struct typec_altmode *adev,
				const struct typec_altmode_ops *ops)
{
	adev->ops = ops;
}
EXPORT_SYMBOL_GPL(typec_altmode_register_ops);

/* -------------------------------------------------------------------------- */
/* API for the alternate mode drivers */

/**
 * typec_altmode_get_plug - Find cable plug alternate mode
 * @adev: Handle to partner alternate mode
 * @index: Cable plug index
 *
 * Increment reference count for cable plug alternate mode device. Returns
 * handle to the cable plug alternate mode, or NULL if none is found.
 */
struct typec_altmode *typec_altmode_get_plug(struct typec_altmode *adev,
					     int index)
{
	struct altmode *port = to_altmode(adev)->partner;

	if (port->plug[index]) {
		get_device(&port->plug[index]->adev.dev);
		return &port->plug[index]->adev;
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(typec_altmode_get_plug);

/**
 * typec_altmode_get_plug - Decrement cable plug alternate mode reference count
 * @plug: Handle to the cable plug alternate mode
 */
void typec_altmode_put_plug(struct typec_altmode *plug)
{
	if (plug)
		put_device(&plug->dev);
}
EXPORT_SYMBOL_GPL(typec_altmode_put_plug);

int __typec_altmode_register_driver(struct typec_altmode_driver *drv,
				    struct module *module)
{
	if (!drv->probe)
		return -EINVAL;

	drv->driver.owner = module;
	drv->driver.bus = &typec_bus;

	return driver_register(&drv->driver);
}
EXPORT_SYMBOL_GPL(__typec_altmode_register_driver);

void typec_altmode_unregister_driver(struct typec_altmode_driver *drv)
{
	driver_unregister(&drv->driver);
}
EXPORT_SYMBOL_GPL(typec_altmode_unregister_driver);

/* -------------------------------------------------------------------------- */
/* API for the port drivers */

bool typec_altmode_ufp_capable(struct typec_altmode *adev)
{
	struct altmode *altmode = to_altmode(adev);

	if (!is_typec_port(adev->dev.parent))
		return false;

	return !(altmode->roles == TYPEC_PORT_DFP);
}
EXPORT_SYMBOL_GPL(typec_altmode_ufp_capable);

bool typec_altmode_dfp_capable(struct typec_altmode *adev)
{
	struct altmode *altmode = to_altmode(adev);

	if (!is_typec_port(adev->dev.parent))
		return false;

	return !(altmode->roles == TYPEC_PORT_UFP);
}
EXPORT_SYMBOL_GPL(typec_altmode_dfp_capable);

/**
 * typec_match_altmode - Match SVID to an array of alternate modes
 * @altmodes: Array of alternate modes
 * @n: Number of elements in the array, or -1 for NULL termiated arrays
 * @svid: Standard or Vendor ID to match with
 *
 * Return pointer to an alternate mode with SVID mathing @svid, or NULL when no
 * match is found.
 */
struct typec_altmode *typec_match_altmode(struct typec_altmode **altmodes,
					  size_t n, u16 svid, u8 mode)
{
	int i;

	for (i = 0; i < n; i++) {
		if (!altmodes[i])
			break;
		if (altmodes[i]->svid == svid && altmodes[i]->mode == mode)
			return altmodes[i];
	}

	return NULL;
}
EXPORT_SYMBOL_GPL(typec_match_altmode);

/* -------------------------------------------------------------------------- */

static ssize_t
description_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct typec_altmode *alt = to_typec_altmode(dev);

	return sprintf(buf, "%s\n", alt->desc ? alt->desc : "");
}
static DEVICE_ATTR_RO(description);

static struct attribute *typec_attrs[] = {
	&dev_attr_description.attr,
	NULL
};
ATTRIBUTE_GROUPS(typec);

static int typec_match(struct device *dev, struct device_driver *driver)
{
	struct typec_altmode_driver *drv = to_altmode_driver(driver);
	struct typec_altmode *altmode = to_typec_altmode(dev);
	const struct typec_device_id *id;

	for (id = drv->id_table; id->svid; id++)
		if ((id->svid == altmode->svid) &&
		    (id->mode == TYPEC_ANY_MODE || id->mode == altmode->mode))
			return 1;
	return 0;
}

static int typec_uevent(struct device *dev, struct kobj_uevent_env *env)
{
	struct typec_altmode *altmode = to_typec_altmode(dev);

	if (add_uevent_var(env, "SVID=%04X", altmode->svid))
		return -ENOMEM;

	if (add_uevent_var(env, "MODE=%u", altmode->mode))
		return -ENOMEM;

	return add_uevent_var(env, "MODALIAS=typec:id%04Xm%02X",
			      altmode->svid, altmode->mode);
}

static int typec_altmode_create_links(struct altmode *alt)
{
	struct device *port_dev = &alt->partner->adev.dev;
	struct device *dev = &alt->adev.dev;
	int err;

	err = sysfs_create_link(&dev->kobj, &port_dev->kobj, "port");
	if (err)
		return err;

	err = sysfs_create_link(&port_dev->kobj, &dev->kobj, "partner");
	if (err)
		sysfs_remove_link(&dev->kobj, "port");

	return err;
}

static void typec_altmode_remove_links(struct altmode *alt)
{
	sysfs_remove_link(&alt->partner->adev.dev.kobj, "partner");
	sysfs_remove_link(&alt->adev.dev.kobj, "port");
}

static int typec_probe(struct device *dev)
{
	struct typec_altmode_driver *drv = to_altmode_driver(dev->driver);
	struct typec_altmode *adev = to_typec_altmode(dev);
	struct altmode *altmode = to_altmode(adev);
	int ret;

	/* Fail if the port does not support the alternate mode */
	if (!altmode->partner)
		return -ENODEV;

	ret = typec_altmode_create_links(altmode);
	if (ret) {
		dev_warn(dev, "failed to create symlinks\n");
		return ret;
	}

	ret = drv->probe(adev, altmode->partner->adev.vdo);
	if (ret)
		typec_altmode_remove_links(altmode);

	return 0;
}

static int typec_remove(struct device *dev)
{
	struct typec_altmode_driver *drv = to_altmode_driver(dev->driver);
	struct typec_altmode *adev = to_typec_altmode(dev);
	struct altmode *altmode = to_altmode(adev);

	typec_altmode_remove_links(altmode);

	if (drv->remove)
		drv->remove(to_typec_altmode(dev));

	if (adev->active)
		typec_altmode_exit(adev);

	return 0;
}

struct bus_type typec_bus = {
	.name = "typec",
	.dev_groups = typec_groups,
	.match = typec_match,
	.uevent = typec_uevent,
	.probe = typec_probe,
	.remove = typec_remove,
};
