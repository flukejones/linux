// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for ASUS ROG Ally X controller.
 *
 *  Copyright (c) 2024 Luke Jones <luke@ljones.dev>
 */

#include <linux/hid.h>
#include "hid-ids.h"

#define FEATURE_KBD_REPORT_ID 0x5a
#define FEATURE_KBD_LED_REPORT_ID1 0x5d
#define FEATURE_KBD_LED_REPORT_ID2 0x5e

#define FEATURE_REPORT_ID 0x0d
#define ALLY_CFG_INTF_IN_ADDRESS 0x83
#define ALLY_CFG_INTF_OUT_ADDRESS 0x04
#define ALLY_X_INTERFACE_ADDRESS 0x87

extern int asus_dev_set_report(struct hid_device *hdev, const u8 *buf, size_t buf_size);
extern int asus_dev_get_report(struct hid_device *hdev, u8 *out_buf, size_t out_buf_size);

enum ROG_ALLY_TYPE {
	ROG_ALLY_TYPE,
	ROG_ALLY_TYPE_X,
};

static const struct hid_device_id rog_ally_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_ASUSTEK, USB_DEVICE_ID_ASUSTEK_ROG_NKEY_ALLY),
	  .driver_data = ROG_ALLY_TYPE },
	{ HID_USB_DEVICE(USB_VENDOR_ID_ASUSTEK, USB_DEVICE_ID_ASUSTEK_ROG_NKEY_ALLY_X),
	  .driver_data = ROG_ALLY_TYPE_X },
	{}
};

extern int asus_dev_get_report(struct hid_device *hdev, u8 *out_buf, size_t out_buf_size)
{
	return hid_hw_raw_request(hdev, FEATURE_REPORT_ID, out_buf,
				 out_buf_size, HID_FEATURE_REPORT,
				 HID_REQ_GET_REPORT);
}

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
