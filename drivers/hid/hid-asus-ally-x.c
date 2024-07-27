// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for ASUS ROG Ally X controller.
 *
 *  Copyright (c) 2024 Luke Jones <luke@ljones.dev>
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input-event-codes.h>
#include <linux/usb.h>
#include <linux/module.h>

#include "hid-asus.h"

#define ALLY_X_INPUT_REPORT_USB 0x0B
#define ALLY_X_INPUT_REPORT_USB_SIZE 16

/* The hatswitch outputs integers, we use them to index this X|Y pair */
static const int hat_values[][2] = {
	{0, 0},
	{0, 1},
	{1, 1},
	{1, 0},
	{1, -1},
	{0, -1},
	{-1, -1},
	{-1, 0},
	{-1, 1},
};

/* rumble packet structure */
struct ff_data {
	u8 enable;
	u8 magnitude_left;
	u8 magnitude_right;
	u8 magnitude_strong;
	u8 magnitude_weak;
	u8 pulse_sustain_10ms;
	u8 pulse_release_10ms;
	u8 loop_count;
} __packed;

struct ff_report {
	u8 report_id;
	struct ff_data ff;
} __packed;

struct ally_x_input_report {
	uint16_t x, y;
	uint16_t rx, ry;
	uint16_t z, rz;
	uint8_t buttons[4];
} __packed;

struct rog_ally_device {
	struct input_dev *gamepad;
	struct hid_device *hdev;
	spinlock_t lock;

	struct ff_report *ff_packet;
	struct work_struct output_worker;
	bool output_worker_initialized;
};

static int ally_x_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size)
{
	struct rog_ally_device *ally_x = hid_get_drvdata(hdev);
	struct ally_x_input_report *in_report;
	u8 byte;

	if (hdev->bus == BUS_USB && report->id == ALLY_X_INPUT_REPORT_USB &&
	    size == ALLY_X_INPUT_REPORT_USB_SIZE) {
		in_report = (struct ally_x_input_report *)&data[1];
	} else {
		hid_err(hdev,
			"Unhandled reportID=0x%02X, bus=0x%02X, size=%d\n",
			report->id, hdev->bus, size);
		return -1;
	}

	input_report_abs(ally_x->gamepad, ABS_X,	in_report->x);
	input_report_abs(ally_x->gamepad, ABS_Y,	in_report->y);
	input_report_abs(ally_x->gamepad, ABS_RX,	in_report->rx);
	input_report_abs(ally_x->gamepad, ABS_RY,	in_report->ry);
	input_report_abs(ally_x->gamepad, ABS_Z,	in_report->z);
	input_report_abs(ally_x->gamepad, ABS_RZ,	in_report->rz);

	byte = in_report->buttons[0];
	input_report_key(ally_x->gamepad, BTN_A,	byte & BIT(0));
	input_report_key(ally_x->gamepad, BTN_B,	byte & BIT(1));
	input_report_key(ally_x->gamepad, BTN_X,	byte & BIT(2));
	input_report_key(ally_x->gamepad, BTN_Y,	byte & BIT(3));
	input_report_key(ally_x->gamepad, BTN_TL,	byte & BIT(4));
	input_report_key(ally_x->gamepad, BTN_TR,	byte & BIT(5));
	input_report_key(ally_x->gamepad, BTN_SELECT,	byte & BIT(6));
	input_report_key(ally_x->gamepad, BTN_START,	byte & BIT(7));

	byte = in_report->buttons[1];
	input_report_key(ally_x->gamepad, BTN_THUMBL,	byte & BIT(0));
	input_report_key(ally_x->gamepad, BTN_THUMBR,	byte & BIT(1));
	input_report_key(ally_x->gamepad, BTN_MODE,	byte & BIT(2));

	byte = in_report->buttons[2];
	input_report_abs(ally_x->gamepad, ABS_HAT0X,	hat_values[byte][0]);
	input_report_abs(ally_x->gamepad, ABS_HAT0Y,	hat_values[byte][1]);

	input_sync(ally_x->gamepad);

	return 0;
}

static struct input_dev *ally_x_alloc_input_dev(struct hid_device *hdev,
						const char *name_suffix)
{
	struct input_dev *input_dev;

	input_dev = devm_input_allocate_device(&hdev->dev);
	if (!input_dev)
		return ERR_PTR(-ENOMEM);

	input_dev->id.bustype = hdev->bus;
	input_dev->id.vendor = hdev->vendor;
	input_dev->id.product = hdev->product;
	input_dev->id.version = hdev->version;
	input_dev->uniq = hdev->uniq;
	input_dev->name = "ASUS ROG Ally X Gamepad";

	input_set_drvdata(input_dev, hdev);

	return input_dev;
}

static int ally_x_play_effect(struct input_dev *idev, void *data,
			      struct ff_effect *effect)
{
	struct hid_device *hdev = input_get_drvdata(idev);
	struct rog_ally_device *ally_x = hid_get_drvdata(hdev);

	if (effect->type != FF_RUMBLE)
		return 0;

	ally_x->ff_packet->ff.magnitude_strong =
		effect->u.rumble.strong_magnitude / 512;
	ally_x->ff_packet->ff.magnitude_weak =
		effect->u.rumble.weak_magnitude / 512;
	if (ally_x->output_worker_initialized)
		schedule_work(&ally_x->output_worker);

	return 0;
}

static struct input_dev *setup_capabilities(struct hid_device *hdev)
{
	struct input_dev *gamepad;
	int ret, abs_min = 0, js_abs_max = 65535, tr_abs_max = 1023;

	gamepad = ally_x_alloc_input_dev(hdev, NULL);
	if (IS_ERR(gamepad))
		return ERR_CAST(gamepad);

	input_set_abs_params(gamepad, ABS_X, abs_min, js_abs_max, 0, 0);
	input_set_abs_params(gamepad, ABS_Y, abs_min, js_abs_max, 0, 0);
	input_set_abs_params(gamepad, ABS_RX, abs_min, js_abs_max, 0, 0);
	input_set_abs_params(gamepad, ABS_RY, abs_min, js_abs_max, 0, 0);
	input_set_abs_params(gamepad, ABS_Z, abs_min, tr_abs_max, 0, 0);
	input_set_abs_params(gamepad, ABS_RZ, abs_min, tr_abs_max, 0, 0);
	input_set_abs_params(gamepad, ABS_HAT0X, -1, 1, 0, 0);
	input_set_abs_params(gamepad, ABS_HAT0Y, -1, 1, 0, 0);
	input_set_capability(gamepad, EV_KEY, BTN_A);
	input_set_capability(gamepad, EV_KEY, BTN_B);
	input_set_capability(gamepad, EV_KEY, BTN_X);
	input_set_capability(gamepad, EV_KEY, BTN_Y);
	input_set_capability(gamepad, EV_KEY, BTN_TL);
	input_set_capability(gamepad, EV_KEY, BTN_TR);
	input_set_capability(gamepad, EV_KEY, BTN_SELECT);
	input_set_capability(gamepad, EV_KEY, BTN_START);
	input_set_capability(gamepad, EV_KEY, BTN_MODE);
	input_set_capability(gamepad, EV_KEY, BTN_THUMBL);
	input_set_capability(gamepad, EV_KEY, BTN_THUMBR);
	input_set_capability(gamepad, EV_FF, FF_RUMBLE);
	input_ff_create_memless(gamepad, NULL, ally_x_play_effect);

	ret = input_register_device(gamepad);
	if (ret)
		return ERR_PTR(ret);

	return gamepad;
}

static void ally_x_output_worker(struct work_struct *work)
{
	struct rog_ally_device *ally_x =
		container_of(work, struct rog_ally_device, output_worker);
	struct ff_report *report = ally_x->ff_packet;
	unsigned long flags;

	spin_lock_irqsave(&ally_x->lock, flags);
	report->ff.magnitude_left = report->ff.magnitude_strong;
	report->ff.magnitude_right = report->ff.magnitude_weak;
	asus_dev_set_report(ally_x->hdev, (u8 *)report, sizeof(*report));
	spin_unlock_irqrestore(&ally_x->lock, flags);
}

static int ally_x_create(struct hid_device *hdev)
{
	struct ff_report *report;
	struct rog_ally_device *ally_x;
	uint8_t max_output_report_size;

	ally_x = devm_kzalloc(&hdev->dev, sizeof(*ally_x), GFP_KERNEL);
	if (!ally_x)
		return -ENOMEM;

	ally_x->hdev = hdev;
	spin_lock_init(&ally_x->lock);
	INIT_WORK(&ally_x->output_worker, ally_x_output_worker);
	ally_x->output_worker_initialized = true;
	hid_set_drvdata(hdev, ally_x);

	max_output_report_size = sizeof(struct ally_x_input_report);
	report = devm_kzalloc(&hdev->dev, sizeof(report), GFP_KERNEL);
	if (!report)
		return -ENOMEM;

	// None of these bytes will change for the FF command for now
	report->report_id = 0x0D; // Report ID
	report->ff.enable = 0x0F; // Enable all by default for now
	report->ff.pulse_sustain_10ms = 0xFF; // Duration
	report->ff.pulse_release_10ms = 0x00; // Start Delay
	report->ff.loop_count = 0xEB; // Loop Count
	ally_x->ff_packet = report;

	ally_x->gamepad = setup_capabilities(hdev);
	if (IS_ERR(ally_x->gamepad)) {
		return PTR_ERR(ally_x->gamepad);
	}
	hid_info(hdev, "Registered Ally X controller using %s\n",
		 dev_name(&ally_x->gamepad->dev));

	return 0;
}

static int ally_x_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct usb_host_endpoint *ep = intf->cur_altsetting->endpoint;
	int ret;

	if (ep->desc.bEndpointAddress != ALLY_X_INTERFACE_ADDRESS)
		return -ENODEV;

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "Parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_HIDRAW);
	if (ret) {
		hid_err(hdev, "Failed to start HID device\n");
		return ret;
	}

	ret = hid_hw_open(hdev);
	if (ret) {
		hid_err(hdev, "Failed to open HID device\n");
		goto err_stop;
	}

	ret = ally_x_create(hdev);
	if (ret) {
		hid_err(hdev, "Failed to create Ally X controller.\n");
		goto err_close;
	}

	return ret;

err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ally_x_remove(struct hid_device *hdev)
{
	struct rog_ally_device *ally_x = hid_get_drvdata(hdev);
	unsigned long flags;

	spin_lock_irqsave(&ally_x->lock, flags);
	ally_x->output_worker_initialized = false;
	spin_unlock_irqrestore(&ally_x->lock, flags);

	cancel_work_sync(&ally_x->output_worker);

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

MODULE_DEVICE_TABLE(hid, rog_ally_devices);

static struct hid_driver rog_ally_driver = {
	.name = "asus_rog_ally_x",
	.id_table = rog_ally_devices,
	.probe = ally_x_probe,
	.remove = ally_x_remove,
	.raw_event = ally_x_raw_event,
};

static int __init rog_ally_init(void)
{
	return hid_register_driver(&rog_ally_driver);
}

static void __exit rog_ally_exit(void)
{
	hid_unregister_driver(&rog_ally_driver);
}

module_init(rog_ally_init);
module_exit(rog_ally_exit);

MODULE_AUTHOR("Luke D. Jones");
MODULE_DESCRIPTION("HID Driver for ASUS ROG Ally X.");
MODULE_LICENSE("GPL");
