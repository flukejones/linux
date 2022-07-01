/* SPDX-License-Identifier: GPL-2.0+ WITH Linux-syscall-note */
/*
 * g_hid.h -- Header file for USB HID gadget driver
 *
 * Copyright (C) 2022 Valve Software
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __UAPI_LINUX_USB_G_HID_H
#define __UAPI_LINUX_USB_G_HID_H

#include <linux/types.h>

struct usb_hidg_report {
	__u16 length;
	__u8 data[512];
};

/* The 'g' code is also used by gadgetfs and hid gadget ioctl requests.
 * Don't add any colliding codes to either driver, and keep
 * them in unique ranges (size 0x20 for now).
 */
#define GADGET_HID_WRITE_GET_REPORT	_IOW('g', 0x42, struct usb_hidg_report)

#endif /* __UAPI_LINUX_USB_G_HID_H */
