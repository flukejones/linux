/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */

#ifndef __UAPI_LINUX_USB_G_HID_H
#define __UAPI_LINUX_USB_G_HID_H

#include <linux/types.h>

#define HIDG_REPORT_SIZE_MAX 64

struct usb_hidg_report {
	__u16 length;
	__u8 data[HIDG_REPORT_SIZE_MAX];
};

/* The 'g' code is also used by gadgetfs and hid gadget ioctl requests.
 * Don't add any colliding codes to either driver, and keep
 * them in unique ranges (size 0x20 for now).
 */
#define GADGET_HID_READ_SET_REPORT	_IOR('g', 0x41, struct usb_hidg_report)
#define GADGET_HID_WRITE_GET_REPORT	_IOW('g', 0x42, struct usb_hidg_report)

#endif /* __UAPI_LINUX_USB_G_HID_H */
