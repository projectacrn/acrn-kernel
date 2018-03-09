/* SPDX-License-Identifier: GPL-2.0 */

#ifndef __USB_TYPEC_ALTMODE_H
#define __USB_TYPEC_ALTMODE_H

#include <linux/device.h>
#include <linux/mod_devicetable.h>

#define MODE_DISCOVERY_MAX	6

struct typec_altmode_ops;

/**
 * struct typec_altmode - USB Type-C alternate mode device
 * @dev: Driver model's view of this device
 * @svid: Standard or Vendor ID (SVID) of the alternate mode
 * @mode: Index of the Mode
 * @vdo: VDO returned by Discover Modes USB PD command
 * @desc: Optional human readable description of the mode
 * @active: Tells has the mode been entered or not
 */
struct typec_altmode {
	struct device			dev;
	u16				svid;
	int				mode;
	u32				vdo;
	char				*desc;
	bool				active;
	const struct typec_altmode_ops	*ops;
} __packed;

#define to_typec_altmode(d) container_of(d, struct typec_altmode, dev)

static inline void typec_altmode_set_drvdata(struct typec_altmode *altmode,
					     void *data)
{
	dev_set_drvdata(&altmode->dev, data);
}

static inline void *typec_altmode_get_drvdata(struct typec_altmode *altmode)
{
	return dev_get_drvdata(&altmode->dev);
}

/**
 * struct typec_altmode_ops - Alternate mode specific operations vector
 * @enter: Operations to be executed with Enter Mode Command
 * @exit: Operations to be executed with Exit Mode Command
 * @attention: Callback for Attention Command
 * @vdm: Callback for SVID specific commands
 * @notify: Communication channel for platform and the alternate mode
 */
struct typec_altmode_ops {
	void (*enter)(struct typec_altmode *altmode);
	void (*exit)(struct typec_altmode *altmode);
	void (*attention)(struct typec_altmode *altmode, const u32 vdo);
	int (*vdm)(struct typec_altmode *altmode, const u32 hdr,
		   const u32 *vdo, int cnt);
	int (*notify)(struct typec_altmode *altmode, unsigned long conf,
		      void *data);
};

void typec_altmode_register_ops(struct typec_altmode *altmode,
				const struct typec_altmode_ops *ops);

int typec_altmode_enter(struct typec_altmode *altmode);
int typec_altmode_exit(struct typec_altmode *altmode);
void typec_altmode_attention(struct typec_altmode *altmode, const u32 vdo);
int typec_altmode_vdm(struct typec_altmode *altmode,
		      const u32 header, const u32 *vdo, int count);
int typec_altmode_notify(struct typec_altmode *altmode, unsigned long conf,
			 void *data);

/* Return values for type_altmode_vdm() */
#define VDM_DONE		0 /* Don't care */
#define VDM_OK			1 /* Suits me */

/*
 * These are the pin states (USB, Safe and Alt Mode) and accessory modes (Audio
 * and Debug) defined in USB Type-C Specification. SVID specific pin states are
 * expected to follow and start from the value TYPEC_STATE_MODAL.
 *
 * Port drivers should use TYPEC_STATE_AUDIO and TYPEC_STATE_DEBUG as the
 * operation value for typec_set_mode() when accessory modes are in use.
 *
 * NOTE: typec_altmode_notify() does not accept values smaller then
 * TYPEC_STATE_MODAL. USB Type-C bus will follow USB Type-C Specification with
 * TYPEC_STATE_USB and TYPEC_STATE_SAFE.
 */
enum {
	TYPEC_STATE_USB,	/* USB Operation */
	TYPEC_STATE_AUDIO,	/* Audio Accessory */
	TYPEC_STATE_DEBUG,	/* Debug Accessory */
	TYPEC_STATE_SAFE,	/* USB Safe State */
	TYPEC_STATE_MODAL,	/* Alternate Modes */
};

#define TYPEC_MODAL_STATE(_state_)	((_state_) + TYPEC_STATE_MODAL)

struct typec_altmode *typec_altmode_get_plug(struct typec_altmode *altmode,
					     int index);
void typec_altmode_put_plug(struct typec_altmode *plug);

bool typec_altmode_ufp_capable(struct typec_altmode *altmode);
bool typec_altmode_dfp_capable(struct typec_altmode *altmode);
struct typec_altmode *typec_match_altmode(struct typec_altmode **altmodes,
					  size_t n, u16 svid, u8 mode);

/**
 * struct typec_altmode_driver - USB Type-C alternate mode device driver
 * @id_table: Null terminated array of SVIDs
 * @probe: Callback for device binding
 * @remove: Callback for device unbinding
 * @driver: Device driver model driver
 *
 * These drivers will be bind to the partner alternate mode devices. They will
 * handle all SVID specific communication.
 */
struct typec_altmode_driver {
	const struct typec_device_id *id_table;
	int (*probe)(struct typec_altmode *altmode, u32 port_vdo);
	void (*remove)(struct typec_altmode *altmode);
	struct device_driver driver;
};

#define to_altmode_driver(d) container_of(d, struct typec_altmode_driver, \
					  driver)

#define typec_altmode_register_driver(drv) \
		__typec_altmode_register_driver(drv, THIS_MODULE)
int __typec_altmode_register_driver(struct typec_altmode_driver *drv,
				    struct module *module);
void typec_altmode_unregister_driver(struct typec_altmode_driver *drv);

#define module_typec_altmode_driver(__typec_altmode_driver) \
	module_driver(__typec_altmode_driver, typec_altmode_register_driver, \
		      typec_altmode_unregister_driver)

#endif /* __USB_TYPEC_ALTMODE_H */
