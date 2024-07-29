// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for Asus ROG laptops and Ally
 *
 *  Copyright (c) 2023 Luke Jones <luke@ljones.dev>
 */

#include <linux/hid.h>
#include <linux/types.h>
#include <linux/usb.h>
#include <linux/leds.h>
#include <linux/led-class-multicolor.h>

#include "hid-asus.h"
#include "hid-asus-ally-cfg.h"

#define READY_MAX_TRIES 4
#define FEATURE_ROG_ALLY_REPORT_ID 0x5a
#define FEATURE_ROG_ALLY_CODE_PAGE 0xD1
#define FEATURE_ROG_ALLY_REPORT_SIZE 64

struct ally_gamepad_rgb_leds {
	struct hid_device *hdev;
	/* Need two dev here to enable the 3 step brightness */
	struct led_classdev led_bright_dev;
	struct led_classdev_mc led_rgb_dev;
	struct work_struct work;
	spinlock_t lock;

	bool removed;

	/* Update the main brightness 0-2 using a single raw write */
	bool update_bright;
	unsigned int brightness;

	/* Update the RGB only to keep write efficient */
	bool update_rgb;
	uint8_t gamepad_red;
	uint8_t gamepad_green;
	uint8_t gamepad_blue;
};

/* ROG Ally has many settings related to the gamepad, all using the same n-key endpoint */
struct ally_gamepad_cfg {
	struct ally_gamepad_rgb_leds *led_rgb;
	enum xpad_mode mode;
	/*
	 * index: [joysticks/triggers][left(2 bytes), right(2 bytes)]
	 * joysticks: 2 bytes: inner, outer
	 * triggers: 2 bytes: lower, upper
	 * min/max: 0-64
	 */
	u8 deadzones[xpad_mode_mouse][2][4];
	/*
	 * index: left, right
	 * max: 64
	 */
	u8 vibration_intensity[xpad_mode_mouse][2];
	/*
	 * index: [joysticks][2 byte stepping per point]
	 * - 4 points of 2 bytes each
	 * - byte 0 of pair = stick move %
	 * - byte 1 of pair = stick response %
	 * - min/max: 1-63
	 */
	bool supports_response_curves;
	u8 response_curve[xpad_mode_mouse][2][8];
	/*
	 * left = byte 0, right = byte 1
	 */
	bool supports_anti_deadzones;
	u8 anti_deadzones[xpad_mode_mouse][2];
	/*
	 * index: [mode][phys pair][b1, b1 secondary, b2, b2 secondary, blocks of 11]
	*/
	u8 key_mapping[xpad_mode_mouse][btn_pair_lt_rt][MAPPING_BLOCK_LEN];
	/*
	 *
	*/
	u8 turbo_btns[xpad_mode_mouse][TURBO_BLOCK_LEN];
	/*
	*/
	u32 js_calibrations[2][6];
	u32 tr_calibrations[2][2];
};

#define STR_TO_CODE_IF(_idx, _code, _label) \
	if (!strcmp(buf_copy, _label))      \
		out[_idx] = _code;

#define STR_TO_CODE_ELIF(_idx, _code, _label) \
	else if (!strcmp(buf_copy, _label)) out[_idx] = _code;

static int __gamepad_write_all_to_mcu(struct hid_device *hdev,
				      struct ally_gamepad_cfg *ally_cfg);

/* writes the bytes for a requested key/function in to the out buffer */
static int __string_to_key_code(const char *buf, u8 *out, int out_len)
{
	char buf_copy[32];
	u8 *save_buf;

	if (out_len != BTN_CODE_LEN)
		return -EINVAL;

	save_buf = kzalloc(out_len, GFP_KERNEL);
	if (!save_buf)
		return -ENOMEM;
	memcpy(save_buf, out, out_len);
	memset(out, 0, out_len); // always clear before adjusting

	// Allow clearing
	if (!strcmp(buf, " ") || !strcmp(buf, "\n") || !strcmp(buf, "\0"))
		goto success;

	strcpy(buf_copy, buf);
	buf_copy[strcspn(buf_copy, "\n")] = 0;

	// set group xpad
	out[0] = 0x01;
	STR_TO_CODE_IF(1, 0x01, PAD_A)
	STR_TO_CODE_ELIF(1, 0x02, PAD_B)
	STR_TO_CODE_ELIF(1, 0x03, PAD_X)
	STR_TO_CODE_ELIF(1, 0x04, PAD_Y)
	STR_TO_CODE_ELIF(1, 0x05, PAD_LB)
	STR_TO_CODE_ELIF(1, 0x06, PAD_RB)
	STR_TO_CODE_ELIF(1, 0x07, PAD_LS)
	STR_TO_CODE_ELIF(1, 0x08, PAD_RS)
	STR_TO_CODE_ELIF(1, 0x09, PAD_DPAD_UP)
	STR_TO_CODE_ELIF(1, 0x0a, PAD_DPAD_DOWN)
	STR_TO_CODE_ELIF(1, 0x0b, PAD_DPAD_LEFT)
	STR_TO_CODE_ELIF(1, 0x0c, PAD_DPAD_RIGHT)
	STR_TO_CODE_ELIF(1, 0x11, PAD_VIEW)
	STR_TO_CODE_ELIF(1, 0x12, PAD_MENU)
	STR_TO_CODE_ELIF(1, 0x13, PAD_XBOX)
	if (out[1])
		goto success;

	// set group keyboard
	out[0] = 0x02;
	STR_TO_CODE_IF(2, 0x8f, KB_M1)
	STR_TO_CODE_ELIF(2, 0x8e, KB_M2)

	STR_TO_CODE_ELIF(2, 0x76, KB_ESC)
	STR_TO_CODE_ELIF(2, 0x50, KB_F1)
	STR_TO_CODE_ELIF(2, 0x60, KB_F2)
	STR_TO_CODE_ELIF(2, 0x40, KB_F3)
	STR_TO_CODE_ELIF(2, 0x0c, KB_F4)
	STR_TO_CODE_ELIF(2, 0x03, KB_F5)
	STR_TO_CODE_ELIF(2, 0x0b, KB_F6)
	STR_TO_CODE_ELIF(2, 0x80, KB_F7)
	STR_TO_CODE_ELIF(2, 0x0a, KB_F8)
	STR_TO_CODE_ELIF(2, 0x01, KB_F9)
	STR_TO_CODE_ELIF(2, 0x09, KB_F10)
	STR_TO_CODE_ELIF(2, 0x78, KB_F11)
	STR_TO_CODE_ELIF(2, 0x07, KB_F12)
	STR_TO_CODE_ELIF(2, 0x10, KB_F14)
	STR_TO_CODE_ELIF(2, 0x18, KB_F15)

	STR_TO_CODE_ELIF(2, 0x0e, KB_BACKTICK)
	STR_TO_CODE_ELIF(2, 0x16, KB_1)
	STR_TO_CODE_ELIF(2, 0x1e, KB_2)
	STR_TO_CODE_ELIF(2, 0x26, KB_3)
	STR_TO_CODE_ELIF(2, 0x25, KB_4)
	STR_TO_CODE_ELIF(2, 0x2e, KB_5)
	STR_TO_CODE_ELIF(2, 0x36, KB_6)
	STR_TO_CODE_ELIF(2, 0x3d, KB_7)
	STR_TO_CODE_ELIF(2, 0x3e, KB_8)
	STR_TO_CODE_ELIF(2, 0x46, KB_9)
	STR_TO_CODE_ELIF(2, 0x45, KB_0)
	STR_TO_CODE_ELIF(2, 0x4e, KB_HYPHEN)
	STR_TO_CODE_ELIF(2, 0x55, KB_EQUALS)
	STR_TO_CODE_ELIF(2, 0x66, KB_BACKSPACE)

	STR_TO_CODE_ELIF(2, 0x0d, KB_TAB)
	STR_TO_CODE_ELIF(2, 0x15, KB_Q)
	STR_TO_CODE_ELIF(2, 0x1d, KB_W)
	STR_TO_CODE_ELIF(2, 0x24, KB_E)
	STR_TO_CODE_ELIF(2, 0x2d, KB_R)
	STR_TO_CODE_ELIF(2, 0x2d, KB_T)
	STR_TO_CODE_ELIF(2, 0x35, KB_Y)
	STR_TO_CODE_ELIF(2, 0x3c, KB_U)
	STR_TO_CODE_ELIF(2, 0x43, KB_I)
	STR_TO_CODE_ELIF(2, 0x44, KB_O)
	STR_TO_CODE_ELIF(2, 0x4d, KB_P)
	STR_TO_CODE_ELIF(2, 0x54, KB_LBRACKET)
	STR_TO_CODE_ELIF(2, 0x5b, KB_RBRACKET)
	STR_TO_CODE_ELIF(2, 0x5d, KB_BACKSLASH)

	STR_TO_CODE_ELIF(2, 0x58, KB_CAPS)
	STR_TO_CODE_ELIF(2, 0x1c, KB_A)
	STR_TO_CODE_ELIF(2, 0x1b, KB_S)
	STR_TO_CODE_ELIF(2, 0x23, KB_D)
	STR_TO_CODE_ELIF(2, 0x2b, KB_F)
	STR_TO_CODE_ELIF(2, 0x34, KB_G)
	STR_TO_CODE_ELIF(2, 0x33, KB_H)
	STR_TO_CODE_ELIF(2, 0x3b, KB_J)
	STR_TO_CODE_ELIF(2, 0x42, KB_K)
	STR_TO_CODE_ELIF(2, 0x4b, KB_L)
	STR_TO_CODE_ELIF(2, 0x4c, KB_SEMI)
	STR_TO_CODE_ELIF(2, 0x52, KB_QUOTE)
	STR_TO_CODE_ELIF(2, 0x5a, KB_RET)

	STR_TO_CODE_ELIF(2, 0x88, KB_LSHIFT)
	STR_TO_CODE_ELIF(2, 0x1a, KB_Z)
	STR_TO_CODE_ELIF(2, 0x22, KB_X)
	STR_TO_CODE_ELIF(2, 0x21, KB_C)
	STR_TO_CODE_ELIF(2, 0x2a, KB_V)
	STR_TO_CODE_ELIF(2, 0x32, KB_B)
	STR_TO_CODE_ELIF(2, 0x31, KB_N)
	STR_TO_CODE_ELIF(2, 0x3a, KB_M)
	STR_TO_CODE_ELIF(2, 0x41, KB_COMMA)
	STR_TO_CODE_ELIF(2, 0x49, KB_PERIOD)
	STR_TO_CODE_ELIF(2, 0x4a, KB_FWDSLASH)
	STR_TO_CODE_ELIF(2, 0x89, KB_RSHIFT)

	STR_TO_CODE_ELIF(2, 0x8c, KB_LCTL)
	STR_TO_CODE_ELIF(2, 0x82, KB_META)
	STR_TO_CODE_ELIF(2, 0xba, KB_LALT)
	STR_TO_CODE_ELIF(2, 0x29, KB_SPACE)
	STR_TO_CODE_ELIF(2, 0x8b, KB_RALT)
	STR_TO_CODE_ELIF(2, 0x84, KB_MENU)
	STR_TO_CODE_ELIF(2, 0x8d, KB_RCTL)

	STR_TO_CODE_ELIF(2, 0xc3, KB_PRNTSCN)
	STR_TO_CODE_ELIF(2, 0x7e, KB_SCRLCK)
	STR_TO_CODE_ELIF(2, 0x91, KB_PAUSE)
	STR_TO_CODE_ELIF(2, 0xc2, KB_INS)
	STR_TO_CODE_ELIF(2, 0x94, KB_HOME)
	STR_TO_CODE_ELIF(2, 0x96, KB_PGUP)
	STR_TO_CODE_ELIF(2, 0xc0, KB_DEL)
	STR_TO_CODE_ELIF(2, 0x95, KB_END)
	STR_TO_CODE_ELIF(2, 0x97, KB_PGDWN)

	STR_TO_CODE_ELIF(2, 0x99, KB_UP_ARROW)
	STR_TO_CODE_ELIF(2, 0x98, KB_DOWN_ARROW)
	STR_TO_CODE_ELIF(2, 0x91, KB_LEFT_ARROW)
	STR_TO_CODE_ELIF(2, 0x9b, KB_RIGHT_ARROW)

	STR_TO_CODE_ELIF(2, 0x77, NUMPAD_LOCK)
	STR_TO_CODE_ELIF(2, 0x90, NUMPAD_FWDSLASH)
	STR_TO_CODE_ELIF(2, 0x7c, NUMPAD_ASTERISK)
	STR_TO_CODE_ELIF(2, 0x7b, NUMPAD_HYPHEN)
	STR_TO_CODE_ELIF(2, 0x70, NUMPAD_0)
	STR_TO_CODE_ELIF(2, 0x69, NUMPAD_1)
	STR_TO_CODE_ELIF(2, 0x72, NUMPAD_2)
	STR_TO_CODE_ELIF(2, 0x7a, NUMPAD_3)
	STR_TO_CODE_ELIF(2, 0x6b, NUMPAD_4)
	STR_TO_CODE_ELIF(2, 0x73, NUMPAD_5)
	STR_TO_CODE_ELIF(2, 0x74, NUMPAD_6)
	STR_TO_CODE_ELIF(2, 0x6c, NUMPAD_7)
	STR_TO_CODE_ELIF(2, 0x75, NUMPAD_8)
	STR_TO_CODE_ELIF(2, 0x7d, NUMPAD_9)
	STR_TO_CODE_ELIF(2, 0x79, NUMPAD_PLUS)
	STR_TO_CODE_ELIF(2, 0x81, NUMPAD_ENTER)
	STR_TO_CODE_ELIF(2, 0x71, NUMPAD_PERIOD)
	if (out[2])
		goto success;

	out[0] = 0x03;
	STR_TO_CODE_IF(4, 0x01, MOUSE_LCLICK)
	STR_TO_CODE_ELIF(4, 0x02, MOUSE_RCLICK)
	STR_TO_CODE_ELIF(4, 0x03, MOUSE_MCLICK)
	STR_TO_CODE_ELIF(4, 0x04, MOUSE_WHEEL_UP)
	STR_TO_CODE_ELIF(4, 0x05, MOUSE_WHEEL_DOWN)
	if (out[4] != 0)
		goto success;

	out[0] = 0x05;
	STR_TO_CODE_IF(3, 0x16, MEDIA_SCREENSHOT)
	STR_TO_CODE_ELIF(3, 0x19, MEDIA_SHOW_KEYBOARD)
	STR_TO_CODE_ELIF(3, 0x1c, MEDIA_SHOW_DESKTOP)
	STR_TO_CODE_ELIF(3, 0x1e, MEDIA_START_RECORDING)
	STR_TO_CODE_ELIF(3, 0x01, MEDIA_MIC_OFF)
	STR_TO_CODE_ELIF(3, 0x02, MEDIA_VOL_DOWN)
	STR_TO_CODE_ELIF(3, 0x03, MEDIA_VOL_UP)
	if (out[3])
		goto success;

	// restore bytes if invalid input
	memcpy(out, save_buf, out_len);
	kfree(save_buf);
	return -EINVAL;

success:
	kfree(save_buf);
	return 0;
}

#define CODE_TO_STR_IF(_idx, _code, _label) \
	if (btn_block[_idx] == _code)       \
		return _label;

static const char *__btn_map_to_string(struct hid_device *hdev,
				       struct ally_gamepad_cfg *ally_cfg,
				       enum btn_pair pair,
				       enum btn_pair_side side, bool secondary)
{
	u8 *btn_block;
	int offs;

	// TODO: this little block is common
	offs = side ? MAPPING_BLOCK_LEN / 2 : 0;
	offs = secondary ? offs + BTN_CODE_LEN : offs;
	btn_block = ally_cfg->key_mapping[ally_cfg->mode - 1][pair - 1] + offs;

	if (btn_block[0] == 0x01) {
		CODE_TO_STR_IF(1, 0x01, PAD_A)
		CODE_TO_STR_IF(1, 0x02, PAD_B)
		CODE_TO_STR_IF(1, 0x03, PAD_X)
		CODE_TO_STR_IF(1, 0x04, PAD_Y)
		CODE_TO_STR_IF(1, 0x05, PAD_LB)
		CODE_TO_STR_IF(1, 0x06, PAD_RB)
		CODE_TO_STR_IF(1, 0x07, PAD_LS)
		CODE_TO_STR_IF(1, 0x08, PAD_RS)
		CODE_TO_STR_IF(1, 0x09, PAD_DPAD_UP)
		CODE_TO_STR_IF(1, 0x0a, PAD_DPAD_DOWN)
		CODE_TO_STR_IF(1, 0x0b, PAD_DPAD_LEFT)
		CODE_TO_STR_IF(1, 0x0c, PAD_DPAD_RIGHT)
		CODE_TO_STR_IF(1, 0x11, PAD_VIEW)
		CODE_TO_STR_IF(1, 0x12, PAD_MENU)
		CODE_TO_STR_IF(1, 0x13, PAD_XBOX)
	}

	if (btn_block[0] == 0x02) {
		CODE_TO_STR_IF(2, 0x8f, KB_M1)
		CODE_TO_STR_IF(2, 0x8e, KB_M2)
		CODE_TO_STR_IF(2, 0x76, KB_ESC)
		CODE_TO_STR_IF(2, 0x50, KB_F1)
		CODE_TO_STR_IF(2, 0x60, KB_F2)
		CODE_TO_STR_IF(2, 0x40, KB_F3)
		CODE_TO_STR_IF(2, 0x0c, KB_F4)
		CODE_TO_STR_IF(2, 0x03, KB_F5)
		CODE_TO_STR_IF(2, 0x0b, KB_F6)
		CODE_TO_STR_IF(2, 0x80, KB_F7)
		CODE_TO_STR_IF(2, 0x0a, KB_F8)
		CODE_TO_STR_IF(2, 0x01, KB_F9)
		CODE_TO_STR_IF(2, 0x09, KB_F10)
		CODE_TO_STR_IF(2, 0x78, KB_F11)
		CODE_TO_STR_IF(2, 0x07, KB_F12)
		CODE_TO_STR_IF(2, 0x10, KB_F14)
		CODE_TO_STR_IF(2, 0x18, KB_F15)

		CODE_TO_STR_IF(2, 0x0e, KB_BACKTICK)
		CODE_TO_STR_IF(2, 0x16, KB_1)
		CODE_TO_STR_IF(2, 0x1e, KB_2)
		CODE_TO_STR_IF(2, 0x26, KB_3)
		CODE_TO_STR_IF(2, 0x25, KB_4)
		CODE_TO_STR_IF(2, 0x2e, KB_5)
		CODE_TO_STR_IF(2, 0x36, KB_6)
		CODE_TO_STR_IF(2, 0x3d, KB_7)
		CODE_TO_STR_IF(2, 0x3e, KB_8)
		CODE_TO_STR_IF(2, 0x46, KB_9)
		CODE_TO_STR_IF(2, 0x45, KB_0)
		CODE_TO_STR_IF(2, 0x4e, KB_HYPHEN)
		CODE_TO_STR_IF(2, 0x55, KB_EQUALS)
		CODE_TO_STR_IF(2, 0x66, KB_BACKSPACE)

		CODE_TO_STR_IF(2, 0x0d, KB_TAB)
		CODE_TO_STR_IF(2, 0x15, KB_Q)
		CODE_TO_STR_IF(2, 0x1d, KB_W)
		CODE_TO_STR_IF(2, 0x24, KB_E)
		CODE_TO_STR_IF(2, 0x2d, KB_R)
		CODE_TO_STR_IF(2, 0x2d, KB_T)
		CODE_TO_STR_IF(2, 0x35, KB_Y)
		CODE_TO_STR_IF(2, 0x3c, KB_U)
		CODE_TO_STR_IF(2, 0x43, KB_I)
		CODE_TO_STR_IF(2, 0x44, KB_O)
		CODE_TO_STR_IF(2, 0x4d, KB_P)
		CODE_TO_STR_IF(2, 0x54, KB_LBRACKET)
		CODE_TO_STR_IF(2, 0x5b, KB_RBRACKET)
		CODE_TO_STR_IF(2, 0x5d, KB_BACKSLASH)

		CODE_TO_STR_IF(2, 0x58, KB_CAPS)
		CODE_TO_STR_IF(2, 0x1c, KB_A)
		CODE_TO_STR_IF(2, 0x1b, KB_S)
		CODE_TO_STR_IF(2, 0x23, KB_D)
		CODE_TO_STR_IF(2, 0x2b, KB_F)
		CODE_TO_STR_IF(2, 0x34, KB_G)
		CODE_TO_STR_IF(2, 0x33, KB_H)
		CODE_TO_STR_IF(2, 0x3b, KB_J)
		CODE_TO_STR_IF(2, 0x42, KB_K)
		CODE_TO_STR_IF(2, 0x4b, KB_L)
		CODE_TO_STR_IF(2, 0x4c, KB_SEMI)
		CODE_TO_STR_IF(2, 0x52, KB_QUOTE)
		CODE_TO_STR_IF(2, 0x5a, KB_RET)

		CODE_TO_STR_IF(2, 0x88, KB_LSHIFT)
		CODE_TO_STR_IF(2, 0x1a, KB_Z)
		CODE_TO_STR_IF(2, 0x22, KB_X)
		CODE_TO_STR_IF(2, 0x21, KB_C)
		CODE_TO_STR_IF(2, 0x2a, KB_V)
		CODE_TO_STR_IF(2, 0x32, KB_B)
		CODE_TO_STR_IF(2, 0x31, KB_N)
		CODE_TO_STR_IF(2, 0x3a, KB_M)
		CODE_TO_STR_IF(2, 0x41, KB_COMMA)
		CODE_TO_STR_IF(2, 0x49, KB_PERIOD)
		CODE_TO_STR_IF(2, 0x4a, KB_FWDSLASH)
		CODE_TO_STR_IF(2, 0x89, KB_RSHIFT)

		CODE_TO_STR_IF(2, 0x8c, KB_LCTL)
		CODE_TO_STR_IF(2, 0x82, KB_META)
		CODE_TO_STR_IF(2, 0xba, KB_LALT)
		CODE_TO_STR_IF(2, 0x29, KB_SPACE)
		CODE_TO_STR_IF(2, 0x8b, KB_RALT)
		CODE_TO_STR_IF(2, 0x84, KB_MENU)
		CODE_TO_STR_IF(2, 0x8d, KB_RCTL)

		CODE_TO_STR_IF(2, 0xc3, KB_PRNTSCN)
		CODE_TO_STR_IF(2, 0x7e, KB_SCRLCK)
		CODE_TO_STR_IF(2, 0x91, KB_PAUSE)
		CODE_TO_STR_IF(2, 0xc2, KB_INS)
		CODE_TO_STR_IF(2, 0x94, KB_HOME)
		CODE_TO_STR_IF(2, 0x96, KB_PGUP)
		CODE_TO_STR_IF(2, 0xc0, KB_DEL)
		CODE_TO_STR_IF(2, 0x95, KB_END)
		CODE_TO_STR_IF(2, 0x97, KB_PGDWN)

		CODE_TO_STR_IF(2, 0x99, KB_UP_ARROW)
		CODE_TO_STR_IF(2, 0x98, KB_DOWN_ARROW)
		CODE_TO_STR_IF(2, 0x91, KB_LEFT_ARROW)
		CODE_TO_STR_IF(2, 0x9b, KB_RIGHT_ARROW)

		CODE_TO_STR_IF(2, 0x77, NUMPAD_LOCK)
		CODE_TO_STR_IF(2, 0x90, NUMPAD_FWDSLASH)
		CODE_TO_STR_IF(2, 0x7c, NUMPAD_ASTERISK)
		CODE_TO_STR_IF(2, 0x7b, NUMPAD_HYPHEN)
		CODE_TO_STR_IF(2, 0x70, NUMPAD_0)
		CODE_TO_STR_IF(2, 0x69, NUMPAD_1)
		CODE_TO_STR_IF(2, 0x72, NUMPAD_2)
		CODE_TO_STR_IF(2, 0x7a, NUMPAD_3)
		CODE_TO_STR_IF(2, 0x6b, NUMPAD_4)
		CODE_TO_STR_IF(2, 0x73, NUMPAD_5)
		CODE_TO_STR_IF(2, 0x74, NUMPAD_6)
		CODE_TO_STR_IF(2, 0x6c, NUMPAD_7)
		CODE_TO_STR_IF(2, 0x75, NUMPAD_8)
		CODE_TO_STR_IF(2, 0x7d, NUMPAD_9)
		CODE_TO_STR_IF(2, 0x79, NUMPAD_PLUS)
		CODE_TO_STR_IF(2, 0x81, NUMPAD_ENTER)
		CODE_TO_STR_IF(2, 0x71, NUMPAD_PERIOD)
	}

	if (btn_block[0] == 0x03) {
		CODE_TO_STR_IF(4, 0x01, MOUSE_LCLICK)
		CODE_TO_STR_IF(4, 0x02, MOUSE_RCLICK)
		CODE_TO_STR_IF(4, 0x03, MOUSE_MCLICK)
		CODE_TO_STR_IF(4, 0x04, MOUSE_WHEEL_UP)
		CODE_TO_STR_IF(4, 0x05, MOUSE_WHEEL_DOWN)
	}

	if (btn_block[0] == 0x05) {
		CODE_TO_STR_IF(3, 0x16, MEDIA_SCREENSHOT)
		CODE_TO_STR_IF(3, 0x19, MEDIA_SHOW_KEYBOARD)
		CODE_TO_STR_IF(3, 0x1c, MEDIA_SHOW_DESKTOP)
		CODE_TO_STR_IF(3, 0x1e, MEDIA_START_RECORDING)
		CODE_TO_STR_IF(3, 0x01, MEDIA_MIC_OFF)
		CODE_TO_STR_IF(3, 0x02, MEDIA_VOL_DOWN)
		CODE_TO_STR_IF(3, 0x03, MEDIA_VOL_UP)
	}

	return "";
}

/* ASUS ROG Ally device specific attributes */

/* This should be called before any attempts to set device functions */
static int __gamepad_check_ready(struct hid_device *hdev)
{
	return 0;
	int ret, count;
	u8 *hidbuf;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	for (count = 0; count < READY_MAX_TRIES; count++) {
		hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
		hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
		hidbuf[2] = xpad_cmd_check_ready;
		hidbuf[3] = 01;
		ret = asus_dev_set_report(hdev, hidbuf,
					  FEATURE_ROG_ALLY_REPORT_SIZE);
		if (ret < 0)
			hid_warn(hdev, "ROG Ally check failed set report: %d\n",
				 ret);

		hidbuf[0] = hidbuf[1] = hidbuf[2] = hidbuf[3] = 0;
		ret = asus_dev_get_report(hdev, hidbuf,
					  FEATURE_ROG_ALLY_REPORT_SIZE);
		if (ret < 0)
			hid_warn(hdev, "ROG Ally check failed get report: %d\n",
				 ret);

		ret = hidbuf[2] == xpad_cmd_check_ready;
		if (ret)
			break;
		msleep(1); // don't spam the entire loop in less than USB response time
	}

	if (count == READY_MAX_TRIES)
		hid_warn(hdev, "ROG Ally never responded with a ready\n");

	kfree(hidbuf);
	return ret;
}

/********** BUTTON REMAPPING *********************************************************************/
static void __btn_pair_to_pkt(struct ally_gamepad_cfg *ally_cfg,
			      enum btn_pair pair, u8 *out, int out_len)
{
	out[0] = FEATURE_ROG_ALLY_REPORT_ID;
	out[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	out[2] = xpad_cmd_set_mapping;
	out[3] = pair;
	out[4] = xpad_cmd_len_mapping;
	memcpy(&out[5], &ally_cfg->key_mapping[ally_cfg->mode - 1][pair - 1],
	       MAPPING_BLOCK_LEN);
}

/* Store the button setting in driver data. Does not apply to device until __gamepad_set_mapping */
static int __gamepad_mapping_store(struct ally_gamepad_cfg *ally_cfg,
				   const char *buf, enum btn_pair pair,
				   int side, bool secondary)
{
	u8 *key_code;
	int offs;

	offs = side ? MAPPING_BLOCK_LEN / 2 : 0;
	offs = secondary ? offs + BTN_CODE_LEN : offs;
	key_code = ally_cfg->key_mapping[ally_cfg->mode - 1][pair - 1] + offs;

	return __string_to_key_code(buf, key_code, BTN_CODE_LEN);
}

/* Apply the mapping pair to the device */
static int __gamepad_set_mapping(struct hid_device *hdev,
				 struct ally_gamepad_cfg *ally_cfg,
				 enum btn_pair pair)
{
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	__btn_pair_to_pkt(ally_cfg, pair, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	kfree(hidbuf);

	return ret;
}

static ssize_t btn_mapping_apply_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	int ret;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	ret = __gamepad_write_all_to_mcu(hdev, ally_cfg);
	if (ret < 0)
		return ret;
	return count;
}
ALLY_DEVICE_ATTR_WO(btn_mapping_apply, apply_all);

/********** BUTTON TURBO *************************************************************************/
static int __btn_turbo_index(enum btn_pair pair, int side)
{
	return (pair - 1) * (2 * TURBO_BLOCK_STEP) + (side * TURBO_BLOCK_STEP);
};

static int __gamepad_turbo_show(struct device *dev, enum btn_pair pair,
				int side)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	return ally_cfg
		->turbo_btns[ally_cfg->mode - 1][__btn_turbo_index(pair, side)];
};

static int __gamepad_turbo_store(struct device *dev, const char *buf,
				 enum btn_pair pair, int side)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	int ret, val;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;
	if (val < 0 || val > 16)
		return -EINVAL;

	ally_cfg->turbo_btns[ally_cfg->mode - 1][__btn_turbo_index(pair, side)] =
		val;

	return 0;
};

/* button map attributes, regular and macro*/
ALLY_BTN_MAPPING(m2, btn_pair_m1_m2, btn_pair_side_left);
ALLY_BTN_MAPPING(m1, btn_pair_m1_m2, btn_pair_side_right);
ALLY_BTN_MAPPING(a, btn_pair_a_b, btn_pair_side_left);
ALLY_BTN_MAPPING(b, btn_pair_a_b, btn_pair_side_right);
ALLY_BTN_MAPPING(x, btn_pair_x_y, btn_pair_side_left);
ALLY_BTN_MAPPING(y, btn_pair_x_y, btn_pair_side_right);
ALLY_BTN_MAPPING(lb, btn_pair_lb_rb, btn_pair_side_left);
ALLY_BTN_MAPPING(rb, btn_pair_lb_rb, btn_pair_side_right);
ALLY_BTN_MAPPING(ls, btn_pair_ls_rs, btn_pair_side_left);
ALLY_BTN_MAPPING(rs, btn_pair_ls_rs, btn_pair_side_right);
ALLY_BTN_MAPPING(lt, btn_pair_lt_rt, btn_pair_side_left);
ALLY_BTN_MAPPING(rt, btn_pair_lt_rt, btn_pair_side_right);
ALLY_BTN_MAPPING(dpad_u, btn_pair_dpad_u_d, btn_pair_side_left);
ALLY_BTN_MAPPING(dpad_d, btn_pair_dpad_u_d, btn_pair_side_right);
ALLY_BTN_MAPPING(dpad_l, btn_pair_dpad_l_r, btn_pair_side_left);
ALLY_BTN_MAPPING(dpad_r, btn_pair_dpad_l_r, btn_pair_side_right);
ALLY_BTN_MAPPING(view, btn_pair_view_menu, btn_pair_side_left);
ALLY_BTN_MAPPING(menu, btn_pair_view_menu, btn_pair_side_right);

static void __gamepad_mapping_xpad_default(struct ally_gamepad_cfg *ally_cfg)
{
	memcpy(&ally_cfg->key_mapping[0][0], &XPAD_DEF1, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][1], &XPAD_DEF2, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][2], &XPAD_DEF3, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][3], &XPAD_DEF4, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][4], &XPAD_DEF5, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][5], &XPAD_DEF6, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][6], &XPAD_DEF7, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][7], &XPAD_DEF8, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[0][8], &XPAD_DEF9, MAPPING_BLOCK_LEN);
}

static void __gamepad_mapping_wasd_default(struct ally_gamepad_cfg *ally_cfg)
{
	memcpy(&ally_cfg->key_mapping[1][0], &WASD_DEF1, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][1], &WASD_DEF2, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][2], &WASD_DEF3, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][3], &WASD_DEF4, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][4], &WASD_DEF5, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][5], &WASD_DEF6, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][6], &WASD_DEF7, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][7], &WASD_DEF8, MAPPING_BLOCK_LEN);
	memcpy(&ally_cfg->key_mapping[1][8], &WASD_DEF9, MAPPING_BLOCK_LEN);
}

static ssize_t btn_mapping_reset_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	switch (ally_cfg->mode) {
	case xpad_mode_game:
		__gamepad_mapping_xpad_default(ally_cfg);
		break;
	case xpad_mode_wasd:
		__gamepad_mapping_wasd_default(ally_cfg);
		break;
	default:
		__gamepad_mapping_xpad_default(ally_cfg);
		break;
	}

	return count;
}

ALLY_DEVICE_ATTR_WO(btn_mapping_reset, reset_btn_mapping);

/********** GAMEPAD MODE *************************************************************************/
static ssize_t __gamepad_set_mode(struct hid_device *hdev,
				  struct ally_gamepad_cfg *ally_cfg, int val)
{
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_mode;
	hidbuf[3] = xpad_cmd_len_mode;
	hidbuf[4] = val;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		goto report_fail;

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

	ret = __gamepad_write_all_to_mcu(hdev, ally_cfg);
	if (ret < 0)
		goto report_fail;

report_fail:
	kfree(hidbuf);
	return ret;
}

static ssize_t gamepad_mode_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	return sysfs_emit(buf, "%d\n", ally_cfg->mode);
}

static ssize_t gamepad_mode_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	int ret, val;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < xpad_mode_game || val > xpad_mode_mouse)
		return -EINVAL;

	ally_cfg->mode = val;

	ret = __gamepad_set_mode(hdev, ally_cfg, val);
	if (ret < 0)
		return ret;

	return count;
}

DEVICE_ATTR_RW(gamepad_mode);

/********** VIBRATION INTENSITY ******************************************************************/
static ssize_t
gamepad_vibration_intensity_index_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "left right\n");
}

ALLY_DEVICE_ATTR_RO(gamepad_vibration_intensity_index,
		    vibration_intensity_index);

static ssize_t
__gamepad_write_vibe_intensity_to_mcu(struct hid_device *hdev,
				      struct ally_gamepad_cfg *ally_cfg)
{
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_vibe_intensity;
	hidbuf[3] = xpad_cmd_len_vibe_intensity;
	hidbuf[4] = ally_cfg->vibration_intensity[ally_cfg->mode - 1]
						 [btn_pair_side_left];
	hidbuf[5] = ally_cfg->vibration_intensity[ally_cfg->mode - 1]
						 [btn_pair_side_right];

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		goto report_fail;

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

report_fail:
	kfree(hidbuf);
	return ret;
}

static ssize_t gamepad_vibration_intensity_show(struct device *dev,
						struct device_attribute *attr,
						char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	return sysfs_emit(buf, "%d %d\n",
			  ally_cfg->vibration_intensity[ally_cfg->mode - 1]
						       [btn_pair_side_left],
			  ally_cfg->vibration_intensity[ally_cfg->mode - 1]
						       [btn_pair_side_right]);
}

static ssize_t gamepad_vibration_intensity_store(struct device *dev,
						 struct device_attribute *attr,
						 const char *buf, size_t count)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	u32 left, right;
	int ret;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d %d", &left, &right) != 2)
		return -EINVAL;

	if (left > 64 || right > 64)
		return -EINVAL;

	ally_cfg->vibration_intensity[ally_cfg->mode - 1][btn_pair_side_left] =
		left;
	ally_cfg->vibration_intensity[ally_cfg->mode - 1][btn_pair_side_right] =
		right;

	ret = __gamepad_write_vibe_intensity_to_mcu(hdev, ally_cfg);
	if (ret < 0)
		return ret;

	return count;
}

ALLY_DEVICE_ATTR_RW(gamepad_vibration_intensity, vibration_intensity);

/********** ROOT LEVEL ATTRS **********************************************************************/
static struct attribute *gamepad_device_attrs[] = {
	&dev_attr_gamepad_mode.attr,
	&dev_attr_btn_mapping_reset.attr,
	&dev_attr_btn_mapping_apply.attr,
	&dev_attr_gamepad_vibration_intensity.attr,
	&dev_attr_gamepad_vibration_intensity_index.attr,
	NULL
};

static const struct attribute_group ally_controller_attr_group = {
	.attrs = gamepad_device_attrs,
};

/********** ANALOGUE DEADZONES ********************************************************************/
static ssize_t __gamepad_set_deadzones(struct hid_device *hdev,
				       struct ally_gamepad_cfg *ally_cfg)
{
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_js_dz;
	hidbuf[3] = xpad_cmd_len_deadzone;
	hidbuf[4] = ally_cfg->deadzones[ally_cfg->mode - 1][0][0];
	hidbuf[5] = ally_cfg->deadzones[ally_cfg->mode - 1][0][1];
	hidbuf[6] = ally_cfg->deadzones[ally_cfg->mode - 1][0][2];
	hidbuf[7] = ally_cfg->deadzones[ally_cfg->mode - 1][0][3];

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto end;

	hidbuf[2] = xpad_cmd_set_tr_dz;
	hidbuf[4] = ally_cfg->deadzones[ally_cfg->mode - 1][1][0];
	hidbuf[5] = ally_cfg->deadzones[ally_cfg->mode - 1][1][1];
	hidbuf[6] = ally_cfg->deadzones[ally_cfg->mode - 1][1][2];
	hidbuf[7] = ally_cfg->deadzones[ally_cfg->mode - 1][1][3];

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto end;

end:
	kfree(hidbuf);
	return ret;
}

static ssize_t __gamepad_store_deadzones(struct hid_device *hdev,
					 struct ally_gamepad_cfg *ally_cfg,
					 enum xpad_axis axis, const char *buf)
{
	int cmd, side, is_tr;
	u32 inner, outer;

	if (sscanf(buf, "%d %d", &inner, &outer) != 2)
		return -EINVAL;

	if (inner > 64 || outer > 64 || inner > outer)
		return -EINVAL;

	is_tr = axis > xpad_axis_xy_right;
	side = axis == xpad_axis_xy_right || axis == xpad_axis_z_right ? 2 : 0;
	cmd = is_tr ? xpad_cmd_set_js_dz : xpad_cmd_set_tr_dz;

	ally_cfg->deadzones[ally_cfg->mode - 1][is_tr][side] = inner;
	ally_cfg->deadzones[ally_cfg->mode - 1][is_tr][side + 1] = outer;

	return 0;
}

static ssize_t axis_xyz_deadzone_index_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	return sysfs_emit(buf, "inner outer\n");
}

ALLY_DEVICE_ATTR_RO(axis_xyz_deadzone_index, deadzone_index);

ALLY_AXIS_DEADZONE(xpad_axis_xy_left, deadzone);
ALLY_AXIS_DEADZONE(xpad_axis_xy_right, deadzone);
ALLY_AXIS_DEADZONE(xpad_axis_z_left, deadzone);
ALLY_AXIS_DEADZONE(xpad_axis_z_right, deadzone);

/********** ANTI-DEADZONES ***********************************************************************/
static ssize_t __gamepad_write_js_ADZ_to_mcu(struct hid_device *hdev,
					     struct ally_gamepad_cfg *ally_cfg)
{
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_adz;
	hidbuf[3] = xpad_cmd_len_adz;
	hidbuf[4] =
		ally_cfg->anti_deadzones[ally_cfg->mode - 1][btn_pair_side_left];
	hidbuf[5] = ally_cfg->anti_deadzones[ally_cfg->mode - 1]
					    [btn_pair_side_right];

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		goto report_fail;

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

report_fail:
	kfree(hidbuf);
	return ret;
}

static ssize_t __gamepad_js_ADZ_store(struct device *dev, const char *buf,
				      enum btn_pair_side side)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	int ret, val;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	ret = kstrtoint(buf, 0, &val);
	if (ret)
		return ret;

	if (val < 0 || val > 32)
		return -EINVAL;

	ally_cfg->anti_deadzones[ally_cfg->mode - 1][side] = val;

	return ret;
}

static ssize_t xpad_axis_xy_left_ADZ_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	return sysfs_emit(buf, "%d\n",
			  ally_cfg->anti_deadzones[ally_cfg->mode - 1]
						  [btn_pair_side_left]);
}

static ssize_t xpad_axis_xy_left_ADZ_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	int ret = __gamepad_js_ADZ_store(dev, buf, btn_pair_side_left);
	if (ret)
		return ret;

	return count;
}

ALLY_DEVICE_ATTR_RW(xpad_axis_xy_left_ADZ, anti_deadzone);

static ssize_t xpad_axis_xy_right_ADZ_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	return sysfs_emit(buf, "%d\n",
			  ally_cfg->anti_deadzones[ally_cfg->mode - 1]
						  [btn_pair_side_right]);
}

static ssize_t xpad_axis_xy_right_ADZ_store(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	int ret = __gamepad_js_ADZ_store(dev, buf, btn_pair_side_right);
	if (ret)
		return ret;

	return count;
}

ALLY_DEVICE_ATTR_RW(xpad_axis_xy_right_ADZ, anti_deadzone);

/********** JS RESPONSE CURVES *******************************************************************/
static ssize_t rc_point_index_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	return sysfs_emit(buf, "move response\n");
}

ALLY_DEVICE_ATTR_RO(rc_point_index, rc_point_index);

static ssize_t
__gamepad_write_response_curves_to_mcu(struct hid_device *hdev,
				       struct ally_gamepad_cfg *ally_cfg)
{
	u8 *hidbuf;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_response_curve;
	hidbuf[3] = xpad_cmd_len_response_curve;
	hidbuf[4] = 0x01;
	memcpy(&hidbuf[5],
	       &ally_cfg->response_curve[ally_cfg->mode - 1][btn_pair_side_left],
	       8);

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		goto report_fail;

	hidbuf[4] = 0x02;
	memcpy(&hidbuf[5],
	       &ally_cfg->response_curve[ally_cfg->mode - 1]
					[btn_pair_side_right],
	       8);

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		goto report_fail;

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

report_fail:
	kfree(hidbuf);
	return ret;
}

static ssize_t __gamepad_store_response_curve(struct device *dev,
					      const char *buf,
					      enum btn_pair_side side,
					      int point)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	int idx = (point - 1) * 2;
	u32 move, response;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	if (sscanf(buf, "%d %d", &move, &response) != 2)
		return -EINVAL;

	if (move > 64 || response > 64)
		return -EINVAL;

	ally_cfg->response_curve[ally_cfg->mode - 1][side][idx] = move;
	ally_cfg->response_curve[ally_cfg->mode - 1][side][idx + 1] = response;

	return 0;
}

ALLY_JS_RC_POINT(left, 1, rc_point_);
ALLY_JS_RC_POINT(left, 2, rc_point_);
ALLY_JS_RC_POINT(left, 3, rc_point_);
ALLY_JS_RC_POINT(left, 4, rc_point_);

ALLY_JS_RC_POINT(right, 1, rc_point_);
ALLY_JS_RC_POINT(right, 2, rc_point_);
ALLY_JS_RC_POINT(right, 3, rc_point_);
ALLY_JS_RC_POINT(right, 4, rc_point_);

/********** CALIBRATIONS *************************************************************************/
static ssize_t __gamepad_write_cal_to_mcu(struct device *dev,
					  enum xpad_axis axis)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	u8 *hidbuf;
	u8 *c, side, pkt_len, data_len;
	int ret, cal, checksum = 0;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	side = axis == xpad_axis_xy_right || axis == xpad_axis_z_right ? 1 : 0;
	pkt_len = axis > xpad_axis_xy_right ? 0x06 : 0x0E;
	data_len = axis > xpad_axis_xy_right ? 2 : 6;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_calibration;
	hidbuf[3] = pkt_len;
	hidbuf[4] = 0x01; // second command (set)
	hidbuf[5] = axis;
	c = &hidbuf[6]; // pointer

	for (size_t i = 0; i < data_len; i++) {
		cal = ally_cfg->js_calibrations[side][i];
		*c = (u8)((cal & 0xff00) >> 8);
		checksum += *c;
		c += 1;
		*c = (u8)(cal & 0xff);
		checksum += *c;
		c += 1;
	}

	hidbuf[6 + data_len * 2] = checksum;

	// TODO: debug if
	printk("CAL: ");
	for (size_t i = 0; i < 19; i++) {
		printk(KERN_CONT "%02x,", hidbuf[i]);
	}

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

	memset(hidbuf, 0, FEATURE_ROG_ALLY_REPORT_SIZE);
	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_calibration;
	hidbuf[3] = xpad_cmd_len_calibration3;
	hidbuf[4] = 0x03; // second command (set)

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

report_fail:
	kfree(hidbuf);
	return ret;
}

static ssize_t __gamepad_cal_store(struct device *dev, const char *buf,
				   enum xpad_axis axis)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	u32 x_stable, x_min, x_max, y_stable, y_min, y_max, side;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	if (axis == xpad_axis_xy_left || axis == xpad_axis_xy_right) {
		if (sscanf(buf, "%d %d %d %d %d %d", &x_stable, &x_min, &x_max,
			   &y_stable, &y_min, &y_max) != 6)
			return -EINVAL;
		//TODO: validate input

		side = axis == xpad_axis_xy_right || axis == xpad_axis_z_right ?
			       1 :
			       0;
		/* stored in reverse order for easy copy to packet */
		ally_cfg->js_calibrations[side][0] = y_stable;
		ally_cfg->js_calibrations[side][1] = y_min;
		ally_cfg->js_calibrations[side][2] = y_max;
		ally_cfg->js_calibrations[side][3] = x_stable;
		ally_cfg->js_calibrations[side][4] = x_min;
		ally_cfg->js_calibrations[side][5] = x_max;

		return __gamepad_write_cal_to_mcu(dev, axis);
	} else {
		if (sscanf(buf, "%d %d", &x_stable, &x_max) != 2)
			return -EINVAL;
		//TODO: validate input

		side = axis == xpad_axis_xy_right || axis == xpad_axis_z_right ?
			       1 :
			       0;
		/* stored in reverse order for easy copy to packet */
		ally_cfg->tr_calibrations[side][0] = x_stable;
		ally_cfg->tr_calibrations[side][1] = x_max;

		return __gamepad_write_cal_to_mcu(dev, axis);
	}
}

static ssize_t __gamepad_cal_show(struct device *dev, char *buf,
				  enum xpad_axis axis)
{
	struct hid_device *hdev = to_hid_device(dev);
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	int side = axis == xpad_axis_xy_right || axis == xpad_axis_z_right ? 1 :
									     0;

	if (!ally_cfg) {
		hid_err(hdev, "Private driver data not found!\n");
		return -EINVAL;
	}

	if (axis == xpad_axis_xy_left || axis == xpad_axis_xy_right) {
		return sysfs_emit(buf, "%d %d %d %d %d %d\n",
				  ally_cfg->js_calibrations[side][3],
				  ally_cfg->js_calibrations[side][4],
				  ally_cfg->js_calibrations[side][5],
				  ally_cfg->js_calibrations[side][0],
				  ally_cfg->js_calibrations[side][1],
				  ally_cfg->js_calibrations[side][2]);
	} else {
		return sysfs_emit(buf, "%d %d\n",
				  ally_cfg->tr_calibrations[side][0],
				  ally_cfg->tr_calibrations[side][1]);
	}
}

ALLY_CAL_ATTR(xpad_axis_xy_left_cal, xpad_axis_xy_left, calibration);
ALLY_CAL_ATTR(xpad_axis_xy_right_cal, xpad_axis_xy_right, calibration);
ALLY_CAL_ATTR(xpad_axis_z_left_cal, xpad_axis_z_left, calibration);
ALLY_CAL_ATTR(xpad_axis_z_right_cal, xpad_axis_z_right, calibration);

static ssize_t xpad_axis_xy_cal_index_show(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	return sysfs_emit(buf, "x_stable x_min x_max y_stable y_min y_max\n");
}

ALLY_DEVICE_ATTR_RO(xpad_axis_xy_cal_index, calibration_index);

static ssize_t xpad_axis_z_cal_index_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	return sysfs_emit(buf, "z_stable z_max\n");
}

ALLY_DEVICE_ATTR_RO(xpad_axis_z_cal_index, calibration_index);

static ssize_t __gamepad_cal_reset(struct device *dev, const char *buf,
				   enum xpad_axis axis)
{
	struct hid_device *hdev = to_hid_device(dev);
	u8 *hidbuf;
	u8 side;
	int ret;

	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;

	side = axis == xpad_axis_xy_right || axis == xpad_axis_z_right ? 1 : 0;

	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_calibration;
	hidbuf[3] = xpad_cmd_len_calibration2;
	hidbuf[4] = 0x02; // second command (reset)
	hidbuf[5] = axis;

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

	memset(hidbuf, 0, FEATURE_ROG_ALLY_REPORT_SIZE);
	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_calibration;
	hidbuf[3] = xpad_cmd_len_calibration3;
	hidbuf[4] = 0x03; // second command (set)

	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);
	if (ret < 0)
		goto report_fail;

report_fail:
	kfree(hidbuf);
	return ret;
}

ALLY_CAL_RESET_ATTR(xpad_axis_xy_left_cal_reset, xpad_axis_xy_left,
		    calibration_reset);
ALLY_CAL_RESET_ATTR(xpad_axis_xy_right_cal_reset, xpad_axis_xy_right,
		    calibration_reset);
ALLY_CAL_RESET_ATTR(xpad_axis_z_left_cal_reset, xpad_axis_z_left,
		    calibration_reset);
ALLY_CAL_RESET_ATTR(xpad_axis_z_right_cal_reset, xpad_axis_z_right,
		    calibration_reset);

static struct attribute *gamepad_axis_xy_left_attrs[] = {
	&dev_attr_xpad_axis_xy_left_deadzone.attr,
	&dev_attr_axis_xyz_deadzone_index.attr,
	&dev_attr_xpad_axis_xy_left_ADZ.attr,
	&dev_attr_xpad_axis_xy_left_cal_reset.attr,
	&dev_attr_xpad_axis_xy_left_cal.attr,
	&dev_attr_xpad_axis_xy_cal_index.attr,
	&dev_attr_rc_point_left_1.attr,
	&dev_attr_rc_point_left_2.attr,
	&dev_attr_rc_point_left_3.attr,
	&dev_attr_rc_point_left_4.attr,
	&dev_attr_rc_point_index.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_xy_left_attr_group = {
	.name = "axis_xy_left",
	.attrs = gamepad_axis_xy_left_attrs,
};

static struct attribute *gamepad_axis_xy_right_attrs[] = {
	&dev_attr_xpad_axis_xy_right_deadzone.attr,
	&dev_attr_axis_xyz_deadzone_index.attr,
	&dev_attr_xpad_axis_xy_right_ADZ.attr,
	&dev_attr_xpad_axis_xy_right_cal_reset.attr,
	&dev_attr_xpad_axis_xy_right_cal.attr,
	&dev_attr_xpad_axis_xy_cal_index.attr,
	&dev_attr_rc_point_right_1.attr,
	&dev_attr_rc_point_right_2.attr,
	&dev_attr_rc_point_right_3.attr,
	&dev_attr_rc_point_right_4.attr,
	&dev_attr_rc_point_index.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_xy_right_attr_group = {
	.name = "axis_xy_right",
	.attrs = gamepad_axis_xy_right_attrs,
};

static struct attribute *gamepad_axis_z_left_attrs[] = {
	&dev_attr_xpad_axis_z_left_deadzone.attr,
	&dev_attr_axis_xyz_deadzone_index.attr,
	&dev_attr_xpad_axis_z_left_cal.attr,
	&dev_attr_xpad_axis_z_cal_index.attr,
	&dev_attr_xpad_axis_z_left_cal_reset.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_z_left_attr_group = {
	.name = "axis_z_left",
	.attrs = gamepad_axis_z_left_attrs,
};

static struct attribute *gamepad_axis_z_right_attrs[] = {
	&dev_attr_xpad_axis_z_right_deadzone.attr,
	&dev_attr_axis_xyz_deadzone_index.attr,
	&dev_attr_xpad_axis_z_right_cal.attr,
	&dev_attr_xpad_axis_z_cal_index.attr,
	&dev_attr_xpad_axis_z_right_cal_reset.attr,
	NULL
};
static const struct attribute_group ally_controller_axis_z_right_attr_group = {
	.name = "axis_z_right",
	.attrs = gamepad_axis_z_right_attrs,
};

static const struct attribute_group *gamepad_device_attr_groups[] = {
	&ally_controller_attr_group,
	&ally_controller_axis_xy_left_attr_group,
	&ally_controller_axis_xy_right_attr_group,
	&ally_controller_axis_z_left_attr_group,
	&ally_controller_axis_z_right_attr_group,
	&btn_mapping_m1_attr_group,
	&btn_mapping_m2_attr_group,
	&btn_mapping_a_attr_group,
	&btn_mapping_b_attr_group,
	&btn_mapping_x_attr_group,
	&btn_mapping_y_attr_group,
	&btn_mapping_lb_attr_group,
	&btn_mapping_rb_attr_group,
	&btn_mapping_ls_attr_group,
	&btn_mapping_rs_attr_group,
	&btn_mapping_dpad_u_attr_group,
	&btn_mapping_dpad_d_attr_group,
	&btn_mapping_dpad_l_attr_group,
	&btn_mapping_dpad_r_attr_group,
	&btn_mapping_view_attr_group,
	&btn_mapping_menu_attr_group,
	NULL
};

static int __gamepad_write_all_to_mcu(struct hid_device *hdev,
				      struct ally_gamepad_cfg *ally_cfg)
{
	u8 *hidbuf;
	int ret = 0;

	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_dpad_u_d);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_dpad_l_r);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_ls_rs);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_lb_rb);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_a_b);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_x_y);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_view_menu);
	if (ret < 0)
		return ret;
	ret = __gamepad_set_mapping(hdev, ally_cfg, btn_pair_m1_m2);
	if (ret < 0)
		return ret;
	__gamepad_set_mapping(hdev, ally_cfg, btn_pair_lt_rt);
	if (ret < 0)
		return ret;
	__gamepad_set_deadzones(hdev, ally_cfg);
	if (ret < 0)
		return ret;
	__gamepad_write_js_ADZ_to_mcu(hdev, ally_cfg);
	if (ret < 0)
		return ret;
	__gamepad_write_vibe_intensity_to_mcu(hdev, ally_cfg);
	if (ret < 0)
		return ret;
	__gamepad_write_response_curves_to_mcu(hdev, ally_cfg);
	if (ret < 0)
		return ret;
	ret = __gamepad_check_ready(hdev);
	if (ret < 0)
		return ret;

	/* set turbo */
	hidbuf = kzalloc(FEATURE_ROG_ALLY_REPORT_SIZE, GFP_KERNEL);
	if (!hidbuf)
		return -ENOMEM;
	hidbuf[0] = FEATURE_ROG_ALLY_REPORT_ID;
	hidbuf[1] = FEATURE_ROG_ALLY_CODE_PAGE;
	hidbuf[2] = xpad_cmd_set_turbo;
	hidbuf[3] = xpad_cmd_len_turbo;
	memcpy(&hidbuf[4], ally_cfg->turbo_btns[ally_cfg->mode - 1],
	       TURBO_BLOCK_LEN);
	ret = asus_dev_set_report(hdev, hidbuf, FEATURE_ROG_ALLY_REPORT_SIZE);

	kfree(hidbuf);
	return ret;
}

static void ally_schedule_work(struct ally_gamepad_rgb_leds *led)
{
	unsigned long flags;

	spin_lock_irqsave(&led->lock, flags);
	if (!led->removed)
		schedule_work(&led->work);
	spin_unlock_irqrestore(&led->lock, flags);
}

static void ally_led_do_brightness(struct work_struct *work)
{
	struct ally_gamepad_rgb_leds *led =
		container_of(work, struct ally_gamepad_rgb_leds, work);
	u8 buf[] = { FEATURE_KBD_REPORT_ID, 0xba, 0xc5, 0xc4, 0x00 };
	unsigned long flags;

	spin_lock_irqsave(&led->lock, flags);
	led->update_bright = false;
	buf[4] = led->brightness;
	spin_unlock_irqrestore(&led->lock, flags);

	if (asus_dev_set_report(led->hdev, buf, sizeof(buf)) < 0)
		hid_err(led->hdev, "Ally failed to set gamepad backlight\n");
}

static void ally_led_do_rgb(struct work_struct *work)
{
	struct ally_gamepad_rgb_leds *led =
		container_of(work, struct ally_gamepad_rgb_leds, work);
	unsigned long flags;
	int ret;

	/*           HID,  Code, Cmd,  len,  Data... */
	u8 buf[] = { FEATURE_ROG_ALLY_REPORT_ID,
		     FEATURE_ROG_ALLY_CODE_PAGE,
		     xpad_cmd_set_leds,
		     xpad_cmd_len_leds,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00,
		     0x00 };

	spin_lock_irqsave(&led->lock, flags);
	buf[4] = led->gamepad_red;
	buf[5] = led->gamepad_green;
	buf[6] = led->gamepad_blue;

	buf[7] = led->gamepad_red;
	buf[8] = led->gamepad_green;
	buf[9] = led->gamepad_blue;

	buf[10] = led->gamepad_red;
	buf[11] = led->gamepad_green;
	buf[12] = led->gamepad_blue;

	buf[13] = led->gamepad_red;
	buf[14] = led->gamepad_green;
	buf[15] = led->gamepad_blue;

	led->update_rgb = false;
	spin_unlock_irqrestore(&led->lock, flags);

	ret = asus_dev_set_report(led->hdev, buf, sizeof(buf));
	if (ret < 0)
		hid_err(led->hdev, "Ally failed to set gamepad backlight: %d\n",
			ret);
}

static void ally_led_work(struct work_struct *work)
{
	struct ally_gamepad_rgb_leds *led =
		container_of(work, struct ally_gamepad_rgb_leds, work);
	bool update_bright, update_rgb;
	unsigned long flags;

	spin_lock_irqsave(&led->lock, flags);
	update_bright = led->update_bright;
	update_rgb = led->update_rgb;
	spin_unlock_irqrestore(&led->lock, flags);

	if (update_bright)
		ally_led_do_brightness(work);

	if (update_rgb)
		ally_led_do_rgb(work);
}

static void ally_backlight_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct ally_gamepad_rgb_leds *led = container_of(
		led_cdev, struct ally_gamepad_rgb_leds, led_bright_dev);
	unsigned long flags;

	spin_lock_irqsave(&led->lock, flags);
	led->update_bright = true;
	led->brightness = brightness;
	spin_unlock_irqrestore(&led->lock, flags);

	ally_schedule_work(led);
}

static enum led_brightness ally_backlight_get(struct led_classdev *led_cdev)
{
	struct ally_gamepad_rgb_leds *led = container_of(
		led_cdev, struct ally_gamepad_rgb_leds, led_bright_dev);
	enum led_brightness brightness;
	unsigned long flags;

	spin_lock_irqsave(&led->lock, flags);
	brightness = led->brightness;
	spin_unlock_irqrestore(&led->lock, flags);

	return brightness;
}

static int ally_gamepad_set_rgb_brightness(struct led_classdev *cdev,
					   enum led_brightness brightness)
{
	struct led_classdev_mc *mc_cdev = lcdev_to_mccdev(cdev);
	struct ally_gamepad_rgb_leds *led = container_of(
		mc_cdev, struct ally_gamepad_rgb_leds, led_rgb_dev);
	unsigned long flags;

	led_mc_calc_color_components(mc_cdev, brightness);
	spin_lock_irqsave(&led->lock, flags);
	led->update_rgb = true;
	led->gamepad_red = mc_cdev->subled_info[0].brightness;
	led->gamepad_green = mc_cdev->subled_info[1].brightness;
	led->gamepad_blue = mc_cdev->subled_info[2].brightness;
	spin_unlock_irqrestore(&led->lock, flags);

	ally_schedule_work(led);

	return 0;
}

static int ally_gamepad_register_bright_leds(struct hid_device *hdev,
					     struct ally_gamepad_cfg *ally_cfg)
{
	struct led_classdev *led_cdev;

	ally_cfg->led_rgb->brightness = 0;
	ally_cfg->led_rgb->hdev = hdev;
	ally_cfg->led_rgb->removed = false;

	led_cdev = &ally_cfg->led_rgb->led_bright_dev;
	led_cdev->name = "ally:kbd_backlight";
	led_cdev->max_brightness = 3;
	led_cdev->brightness_set = ally_backlight_set;
	led_cdev->brightness_get = ally_backlight_get;

	return devm_led_classdev_register(&hdev->dev, led_cdev);
}

static int ally_gamepad_register_rgb_leds(struct hid_device *hdev,
					  struct ally_gamepad_cfg *ally_cfg)
{
	struct mc_subled *mc_led_info;
	struct led_classdev *led_cdev;

	mc_led_info = devm_kmalloc_array(&hdev->dev, 3, sizeof(*mc_led_info),
					 GFP_KERNEL | __GFP_ZERO);
	if (!mc_led_info)
		return -ENOMEM;

	mc_led_info[0].color_index = LED_COLOR_ID_RED;
	mc_led_info[1].color_index = LED_COLOR_ID_GREEN;
	mc_led_info[2].color_index = LED_COLOR_ID_BLUE;

	ally_cfg->led_rgb->led_rgb_dev.subled_info = mc_led_info;
	ally_cfg->led_rgb->led_rgb_dev.num_colors = 3;

	led_cdev = &ally_cfg->led_rgb->led_rgb_dev.led_cdev;
	led_cdev->name = "ally:rgb:gamepad";
	led_cdev->brightness = 128;
	led_cdev->max_brightness = 255;
	led_cdev->brightness_set_blocking = ally_gamepad_set_rgb_brightness;

	return devm_led_classdev_multicolor_register(
		&hdev->dev, &ally_cfg->led_rgb->led_rgb_dev);
}

static int ally_gamepad_register_leds(struct hid_device *hdev,
				      struct ally_gamepad_cfg *ally_cfg)
{
	int ret;

	ally_cfg->led_rgb = devm_kzalloc(
		&hdev->dev, sizeof(struct ally_gamepad_rgb_leds), GFP_KERNEL);
	if (!ally_cfg->led_rgb)
		return -ENOMEM;

	ret = ally_gamepad_register_bright_leds(hdev, ally_cfg);
	if (ret < 0) {
		cancel_work_sync(&ally_cfg->led_rgb->work);
		devm_kfree(&hdev->dev, ally_cfg->led_rgb);
		return ret;
	}

	ret = ally_gamepad_register_rgb_leds(hdev, ally_cfg);
	if (ret < 0) {
		cancel_work_sync(&ally_cfg->led_rgb->work);
		devm_kfree(&hdev->dev, ally_cfg->led_rgb);
		return ret;
	}

	ally_cfg->led_rgb->hdev = hdev;
	INIT_WORK(&ally_cfg->led_rgb->work, ally_led_work);
	spin_lock_init(&ally_cfg->led_rgb->lock);

	return 0;
}

static int ally_gamepad_init(struct hid_device *hdev, u8 report_id)
{
	const u8 buf[] = { report_id, 0x41, 0x53, 0x55, 0x53, 0x20, 0x54, 0x65,
			   0x63,      0x68, 0x2e, 0x49, 0x6e, 0x63, 0x2e, 0x00 };
	int ret;

	ret = asus_dev_set_report(hdev, buf, sizeof(buf));
	if (ret < 0)
		hid_err(hdev, "Ally failed to send init command: %d\n", ret);

	return ret;
}

static int ally_gamepad_cfg_create(struct hid_device *hdev)
{
	struct ally_gamepad_cfg *ally_cfg;
	int i, err;

	/* Initialize MCU even before alloc */
	err = ally_gamepad_init(hdev, FEATURE_KBD_REPORT_ID);
	if (err < 0)
		return err;

	err = ally_gamepad_init(hdev, FEATURE_KBD_LED_REPORT_ID1);
	if (err < 0)
		return err;

	err = ally_gamepad_init(hdev, FEATURE_KBD_LED_REPORT_ID2);
	if (err < 0)
		return err;

	ally_cfg = devm_kzalloc(&hdev->dev, sizeof(*ally_cfg), GFP_KERNEL);
	if (!ally_cfg)
		return -ENOMEM;

	hid_set_drvdata(hdev, (void *)ally_cfg);

	ally_cfg->mode = xpad_mode_game;
	for (i = 0; i < xpad_mode_mouse; i++) {
		ally_cfg->deadzones[i][0][1] = 64;
		ally_cfg->deadzones[i][0][3] = 64;
		ally_cfg->deadzones[i][1][1] = 64;
		ally_cfg->deadzones[i][1][3] = 64;
		ally_cfg->response_curve[i][0][0] = 0x14;
		ally_cfg->response_curve[i][0][1] = 0x14;
		ally_cfg->response_curve[i][0][2] = 0x28;
		ally_cfg->response_curve[i][0][3] = 0x28;
		ally_cfg->response_curve[i][0][4] = 0x3c;
		ally_cfg->response_curve[i][0][5] = 0x3c;
		ally_cfg->response_curve[i][0][6] = 0x50;
		ally_cfg->response_curve[i][0][7] = 0x50;
		ally_cfg->response_curve[i][1][0] = 0x14;
		ally_cfg->response_curve[i][1][1] = 0x14;
		ally_cfg->response_curve[i][1][2] = 0x28;
		ally_cfg->response_curve[i][1][3] = 0x28;
		ally_cfg->response_curve[i][1][4] = 0x3c;
		ally_cfg->response_curve[i][1][5] = 0x3c;
		ally_cfg->response_curve[i][1][6] = 0x50;
		ally_cfg->response_curve[i][1][7] = 0x50;
		ally_cfg->vibration_intensity[i][0] = 64;
		ally_cfg->vibration_intensity[i][1] = 64;
	}

	/* ignore all errors for this as they are related to USB HID I/O */
	__gamepad_mapping_xpad_default(ally_cfg);
	__gamepad_mapping_wasd_default(ally_cfg);
	// these calls will never error so ignore the return
	__gamepad_mapping_store(ally_cfg, PAD_XBOX, btn_pair_m1_m2,
				btn_pair_side_left, false); // M2
	__gamepad_mapping_store(ally_cfg, KB_M1, btn_pair_m1_m2,
				btn_pair_side_right, false); // M1
	__gamepad_set_mapping(hdev, ally_cfg, btn_pair_m1_m2);
	__gamepad_set_mode(hdev, ally_cfg, xpad_mode_game);

	if (sysfs_create_groups(&hdev->dev.kobj, gamepad_device_attr_groups))
		return -ENODEV;

	ally_gamepad_register_leds(hdev, ally_cfg);

	return 0;
}

static int ally_gamepad_cfg_probe(struct hid_device *hdev,
				  const struct hid_device_id *id)
{
	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
	struct usb_host_endpoint *ep = intf->cur_altsetting->endpoint;
	int ret;

	hid_warn(hdev, "CFG1 EP: %02X, %d\n", ep->desc.bEndpointAddress,
		 id->driver_data != ROG_ALLY_TYPE_X &&
			 id->driver_data != ROG_ALLY_TYPE);
	if (ep->desc.bEndpointAddress != ALLY_CFG_INTF_IN_ADDRESS)
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

	ret = ally_gamepad_cfg_create(hdev);
	if (ret) {
		hid_err(hdev, "Failed to create Ally gamepad attributes.\n");
		goto err_close;
	}

	return ret;

err_close:
	hid_hw_close(hdev);
err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void ally_gamepad_cfg_remove(struct hid_device *hdev)
{
	struct ally_gamepad_cfg *ally_cfg = hid_get_drvdata(hdev);
	unsigned long flags;

	if (ally_cfg) {
		if (ally_cfg->led_rgb) {
			spin_lock_irqsave(&ally_cfg->led_rgb->lock, flags);
			ally_cfg->led_rgb->removed = true;
			spin_unlock_irqrestore(&ally_cfg->led_rgb->lock, flags);
			cancel_work_sync(&ally_cfg->led_rgb->work);
		}

		__gamepad_set_mode(hdev, ally_cfg, xpad_mode_mouse);

		sysfs_remove_groups(&hdev->dev.kobj,
				    gamepad_device_attr_groups);

		hid_hw_close(hdev);
		hid_hw_stop(hdev);
	}
}

MODULE_DEVICE_TABLE(hid, rog_ally_devices);

static struct hid_driver rog_ally_cfg = {
	.name = "asus_rog_ally_cfg",
	.id_table = rog_ally_devices,
	.probe = ally_gamepad_cfg_probe,
	.remove = ally_gamepad_cfg_remove,
};

static int __init rog_ally_cfg_init(void)
{
	return hid_register_driver(&rog_ally_cfg);
}

static void __exit rog_ally_cfg_exit(void)
{
	hid_unregister_driver(&rog_ally_cfg);
}

module_init(rog_ally_cfg_init);
module_exit(rog_ally_cfg_exit);

MODULE_AUTHOR("Luke D. Jones");
MODULE_DESCRIPTION("HID Driver for ASUS ROG Ally gamepad configuration.");
MODULE_LICENSE("GPL");
