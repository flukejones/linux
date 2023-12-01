// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for Asus ROG laptops and Ally
 *
 *  Copyright (c) 2016 Yusuke Fujimaki <usk.fujimaki@gmail.com>
 */

#include <linux/hid.h>
#include <linux/types.h>
#include <linux/usb.h>

#include "hid-asus.h"

/* required so we can have nested attributes with same name but different functions */
#define ALLY_DEVICE_ATTR_RW(_name, _sysfs_name) \
	struct device_attribute dev_attr_##_name = __ATTR(_sysfs_name, 0644, _name##_show, _name##_store)

#define ALLY_DEVICE_ATTR_RO(_name, _sysfs_name) \
	struct device_attribute dev_attr_##_name = __ATTR(_sysfs_name, 0444, _name##_show, NULL)

enum ally_xpad_mode {
	ally_xpad_mode_game	= 0x01,
	ally_xpad_mode_wasd	= 0x02,
	ally_xpad_mode_mouse	= 0x03,
};

enum ally_xpad_cmd {
	ally_xpad_cmd_set_mode		= 0x01,
	ally_xpad_cmd_set_js_dz		= 0x04, /* deadzones */
	ally_xpad_cmd_set_tr_dz		= 0x05, /* deadzones */
	ally_xpad_cmd_check_ready	= 0x0A,
};

enum ally_xpad_axis {
	ally_xpad_axis_xy_left	= 0x01,
	ally_xpad_axis_xy_right	= 0x02,
	ally_xpad_axis_z_left	= 0x03,
	ally_xpad_axis_z_right	= 0x04,
};

/* ROG Ally has many settings related to the gamepad, all using the same n-key endpoint */
struct asus_rog_ally {
	enum ally_xpad_mode mode;
	/*
	 * index: [joysticks/triggers][left(2 bytes), right(2 bytes)]
	 * joysticks: 2 bytes: inner, outer
	 * triggers: 2 bytes: lower, upper
	 * min/max: 0-64
	 */
	u8 deadzones[2][4];
	/*
	 * index: left, right
	 * max: 64
	 */
	u8 vibration_intensity[2];
	/*
	 * index: [joysticks][2 byte stepping per point]
	 * - 4 points of 2 bytes each
	 * - byte 0 of pair = stick move %
	 * - byte 1 of pair = stick response %
	 * - min/max: 1-63
	 */
	bool supports_response_curves;
	u8 response_curve [2][8];
	/*
	 * left = byte 0, right = byte 1
	 */
	bool supports_anti_deadzones;
	u8 anti_deadzones[2];
};

/* ASUS ROG Ally device specific attributes */

static struct asus_rog_ally* __rog_ally_data(struct device *raw_dev) {
	struct hid_device *hdev = to_hid_device(raw_dev);
	return ((struct asus_drvdata*)hid_get_drvdata(hdev))->rog_ally_data;
}

/* This should be called before any attempts to set device functions */
static int __gamepad_check_ready(struct hid_device *hdev)
{
	u8 *hidbuf;
	int ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_KBD_REPORT_ID;
	hidbuf[1] = 0xD1;
	hidbuf[2] = ally_xpad_cmd_check_ready;
	hidbuf[3] = 01;
	ret = asus_kbd_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

	hidbuf[0] = hidbuf[1] = hidbuf[2] = hidbuf[3] = 0;
	ret = asus_kbd_get_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

	ret = hidbuf[2] == ally_xpad_cmd_check_ready;
	if (!ret) {
		hid_warn(hdev, "ROG Ally not ready\n");
		ret = -ENOMSG;
	}
	
	kfree(hidbuf);
	return ret;

report_fail:
	hid_dbg(hdev, "ROG Ally check failed get report: %d\n", ret);
	kfree(hidbuf);
	return ret;
}

// TODO: general purpose request function which checks the device is ready before setting
/* The gamepad mode also needs to be set on boot/mod-load and shutdown */
static ssize_t __gamepad_set_mode(struct device *raw_dev, int val) {
	struct hid_device *hdev = to_hid_device(raw_dev);
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_KBD_REPORT_ID;
	hidbuf[1] = 0xD1;
	hidbuf[2] = ally_xpad_cmd_set_mode;
	hidbuf[3] = 0x01;
	hidbuf[4] = val;

	ret = asus_kbd_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0) {
		kfree(hidbuf);
		return ret;
	}

	// TODO: only set on boot. Or if a default for a mode is retained
	hidbuf[2] = 0x02;
	hidbuf[3] = 0x08;
	hidbuf[4] = 0x2c;
	hidbuf[5] = 0x02;
	hidbuf[7] = 0x10; // M1
	hidbuf[27] = 0x02;
	hidbuf[29] = 0x10; // M2
	asus_kbd_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);

	kfree(hidbuf);
	return 0;
}

static ssize_t gamepad_mode_show(struct device *raw_dev, struct device_attribute *attr, char *buf) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	return sysfs_emit(buf, "%d\n", rog_ally->mode);
}

static ssize_t gamepad_mode_store(struct device *raw_dev, struct device_attribute *attr, const char *buf, size_t count) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	int ret, val;

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < ally_xpad_mode_game || val > ally_xpad_mode_mouse)
		return -EINVAL;

	rog_ally->mode = val;

	ret = __gamepad_set_mode(raw_dev, val);
	if (ret < 0)
		return ret;

	return count;
}

DEVICE_ATTR_RW(gamepad_mode);

/* ROG Ally deadzones */
static ssize_t __gamepad_set_deadzones(struct device *raw_dev, enum ally_xpad_axis axis, const char *buf) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	struct hid_device *hdev = to_hid_device(raw_dev);
	int ret, cmd, side, is_js;
	u32 inner, outer;
	u8 *hidbuf;

	if (sscanf(buf, "%d %d", &inner, &outer) != 2)
		return -EINVAL;

	if (inner > 64 || outer > 64 || inner > outer)
		return -EINVAL;

	is_js = !(axis <= ally_xpad_axis_xy_right);
	side = axis == ally_xpad_axis_xy_right
		|| axis == ally_xpad_axis_z_right ? 2 : 0;
	cmd = is_js ? ally_xpad_cmd_set_js_dz : ally_xpad_cmd_set_tr_dz;

	rog_ally->deadzones[is_js][side] = inner;
	rog_ally->deadzones[is_js][side+1] = outer;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_KBD_REPORT_ID;
	hidbuf[1] = 0xD1;
	hidbuf[2] = cmd;
	hidbuf[3] = 0x04; // length
	hidbuf[4] = rog_ally->deadzones[is_js][0];
	hidbuf[5] = rog_ally->deadzones[is_js][1];
	hidbuf[6] = rog_ally->deadzones[is_js][2];
	hidbuf[7] = rog_ally->deadzones[is_js][3];

	ret = asus_kbd_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	kfree(hidbuf);
	return ret;
}

static ssize_t axis_xyz_index_show(struct device *raw_dev, struct device_attribute *attr, char *buf) {
	return sysfs_emit(buf, "inner outer\n");
}

static ssize_t axis_xy_left_deadzone_show(struct device *raw_dev, struct device_attribute *attr, char *buf) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	return sysfs_emit(buf, "%d %d\n", rog_ally->deadzones[0][0], rog_ally->deadzones[0][1]);
}

static ssize_t axis_xy_left_deadzone_store(struct device *raw_dev, struct device_attribute *attr, const char *buf, size_t count) {
	int ret = __gamepad_set_deadzones(raw_dev, ally_xpad_axis_xy_left, buf);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t axis_xy_right_deadzone_show(struct device *raw_dev, struct device_attribute *attr, char *buf) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	return sysfs_emit(buf, "%d %d\n", rog_ally->deadzones[0][2], rog_ally->deadzones[0][3]);
}

static ssize_t axis_xy_right_deadzone_store(struct device *raw_dev, struct device_attribute *attr, const char *buf, size_t count) {
	int ret = __gamepad_set_deadzones(raw_dev, ally_xpad_axis_xy_right, buf);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t axis_z_left_deadzone_show(struct device *raw_dev, struct device_attribute *attr, char *buf) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	return sysfs_emit(buf, "%d %d\n", rog_ally->deadzones[1][0], rog_ally->deadzones[1][1]);
}

static ssize_t axis_z_left_deadzone_store(struct device *raw_dev, struct device_attribute *attr, const char *buf, size_t count) {
	int ret = __gamepad_set_deadzones(raw_dev, ally_xpad_axis_z_left, buf);
	if (ret < 0)
		return ret;
	return count;
}

static ssize_t axis_z_right_deadzone_show(struct device *raw_dev, struct device_attribute *attr, char *buf) {
	struct asus_rog_ally *rog_ally = __rog_ally_data(raw_dev);
	return sysfs_emit(buf, "%d %d\n", rog_ally->deadzones[1][2], rog_ally->deadzones[1][3]);
}

static ssize_t axis_z_right_deadzone_store(struct device *raw_dev, struct device_attribute *attr, const char *buf, size_t count) {
	int ret = __gamepad_set_deadzones(raw_dev, ally_xpad_axis_z_right, buf);
	if (ret < 0)
		return ret;
	return count;
}

ALLY_DEVICE_ATTR_RO(axis_xyz_index, index);
ALLY_DEVICE_ATTR_RW(axis_xy_left_deadzone, deadzone);
ALLY_DEVICE_ATTR_RW(axis_xy_right_deadzone, deadzone);
ALLY_DEVICE_ATTR_RW(axis_z_left_deadzone, deadzone);
ALLY_DEVICE_ATTR_RW(axis_z_right_deadzone, deadzone);

static struct attribute *gamepad_axis_xy_left_attrs[] = {
	&dev_attr_axis_xy_left_deadzone.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_xy_left_attr_group = {
	.name = "axis_xy_left",
	.attrs = gamepad_axis_xy_left_attrs,
};

static struct attribute *gamepad_axis_xy_right_attrs[] = {
	&dev_attr_axis_xy_right_deadzone.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_xy_right_attr_group = {
	.name = "axis_xy_right",
	.attrs = gamepad_axis_xy_right_attrs,
};

static struct attribute *gamepad_axis_z_left_attrs[] = {
	&dev_attr_axis_z_left_deadzone.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_z_left_attr_group = {
	.name = "axis_z_left",
	.attrs = gamepad_axis_z_left_attrs,
};

static struct attribute *gamepad_axis_z_right_attrs[] = {
	&dev_attr_axis_z_right_deadzone.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_z_right_attr_group = {
	.name = "axis_z_right",
	.attrs = gamepad_axis_z_right_attrs,
};

static struct attribute *gamepad_device_attrs[] = {
	&dev_attr_gamepad_mode.attr,
	NULL
};

static const struct attribute_group ally_controller_attr_group = {
	.attrs = gamepad_device_attrs,
};

static const struct attribute_group *gamepad_device_attr_groups[] = {
	&ally_controller_attr_group,
	&ally_controller_axis_xy_left_attr_group,
	&ally_controller_axis_xy_right_attr_group,
	&ally_controller_axis_z_left_attr_group,
	&ally_controller_axis_z_right_attr_group,
	NULL
};

static int asus_rog_ally_probe(struct hid_device *hdev, const struct rog_ops *ops)
{
	struct asus_drvdata *drvdata = hid_get_drvdata(hdev);
	int ret;

	/* all ROG devices have this HID interface but we will focus on Ally for now */
	if (drvdata->quirks & QUIRK_ROG_NKEY_KEYBOARD && hid_is_usb(hdev)) {
		struct usb_interface *intf = to_usb_interface(hdev->dev.parent);

		if (intf->altsetting->desc.bInterfaceNumber == 0) {
			hid_info(hdev, "Setting up ROG USB interface\n");
			/* initialise and set up USB, common to ROG */
			// TODO:

			/* initialise the Ally data */
			if (drvdata->quirks & QUIRK_ROG_ALLY_XPAD) {
				hid_info(hdev, "Setting up ROG Ally interface\n");
				
				drvdata->rog_ally_data = devm_kzalloc(&hdev->dev, sizeof(*drvdata->rog_ally_data), GFP_KERNEL);
				if (!drvdata->rog_ally_data) {
					hid_err(hdev, "Can't alloc Asus ROG USB interface\n");
					ret = -ENOMEM;
					goto err_stop_hw;
				}
				drvdata->rog_ally_data->mode = ally_xpad_mode_game;
				drvdata->rog_ally_data->deadzones[0][1] = 64;
				drvdata->rog_ally_data->deadzones[0][3] = 64;
				drvdata->rog_ally_data->deadzones[1][1] = 64;
				drvdata->rog_ally_data->deadzones[1][3] = 64;

				ret = __gamepad_set_mode(&hdev->dev, ally_xpad_mode_game);
				if (ret < 0)
					return ret;
			}

			if (sysfs_create_groups(&hdev->dev.kobj, gamepad_device_attr_groups))
				goto err_stop_hw;
		}
	}

	return 0;
err_stop_hw:
	hid_hw_stop(hdev);
	return ret;
}

void asus_rog_ally_remove(struct hid_device *hdev, const struct rog_ops *ops)
{
        struct asus_drvdata *drvdata = hid_get_drvdata(hdev);
        if (drvdata->rog_ally_data) {
		__gamepad_set_mode(&hdev->dev, ally_xpad_mode_mouse);
		sysfs_remove_groups(&hdev->dev.kobj, gamepad_device_attr_groups);
	}
}

const struct rog_ops rog_ally = {
        .probe = asus_rog_ally_probe,
        .remove = asus_rog_ally_remove,
};
