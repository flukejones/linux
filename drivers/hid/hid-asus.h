// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for ASUS ROG Ally X controller.
 *
 *  Copyright (c) 2024 Luke Jones <luke@ljones.dev>
 */

#include <linux/hid.h>
#include "hid-ids.h"

#define ALLY_X_INTERFACE_ADDRESS 0x87

extern int asus_dev_set_report(struct hid_device *hdev, const u8 *buf, size_t buf_size);

enum ROG_ALLY_TYPE {
	ROG_ALLY_TYPE_X,
};

static const struct hid_device_id rog_ally_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ASUSTEK, USB_DEVICE_ID_ASUSTEK_ROG_NKEY_ALLY_X),
	  .driver_data = ROG_ALLY_TYPE_X },
	{}
};

extern int asus_dev_set_report(struct hid_device *hdev, const u8 *buf, size_t buf_size)
{
	unsigned char *dmabuf;
	int ret;

	dmabuf = kmemdup(buf, buf_size, GFP_KERNEL);
	if (!dmabuf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, buf[0], dmabuf, buf_size,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	kfree(dmabuf);

	return ret;
}
