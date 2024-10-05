/* SPDX-License-Identifier: GPL-2.0-or-later
 *
 *  HID driver for Asus ROG laptops and Ally
 *
 *  Copyright (c) 2023 Luke Jones <luke@ljones.dev>
 */

#include <linux/hid.h>
#include <linux/types.h>

/*
 * the xpad_mode is used inside the mode setting packet and is used
 * for indexing (xpad_mode - 1)
 */
enum xpad_mode {
	xpad_mode_game = 0x01,
	xpad_mode_wasd = 0x02,
	xpad_mode_mouse = 0x03,
};

/* the xpad_cmd determines which feature is set or queried */
enum xpad_cmd {
	xpad_cmd_set_mapping = 0x02,
	xpad_cmd_set_leds = 0x08,
	xpad_cmd_check_ready = 0x0A,
};

/* the xpad_cmd determines which feature is set or queried */
enum xpad_cmd_len {
	xpad_cmd_len_mapping = 0x2c,
	xpad_cmd_len_leds = 0x0C,
};

/* Values correspond to the actual HID byte value required */
enum btn_pair_index {
	btn_pair_dpad_u_d = 0x01,
	btn_pair_dpad_l_r = 0x02,
	btn_pair_ls_rs = 0x03,
	btn_pair_lb_rb = 0x04,
	btn_pair_a_b = 0x05,
	btn_pair_x_y = 0x06,
	btn_pair_view_menu = 0x07,
	btn_pair_m1_m2 = 0x08,
	btn_pair_lt_rt = 0x09,
};

#define BTN_PAD_A             0x0101
#define BTN_PAD_B             0x0102
#define BTN_PAD_X             0x0103
#define BTN_PAD_Y             0x0104
#define BTN_PAD_LB            0x0105
#define BTN_PAD_RB            0x0106
#define BTN_PAD_LS            0x0107
#define BTN_PAD_RS            0x0108
#define BTN_PAD_DPAD_UP       0x0109
#define BTN_PAD_DPAD_DOWN     0x010A
#define BTN_PAD_DPAD_LEFT     0x010B
#define BTN_PAD_DPAD_RIGHT    0x010C
#define BTN_PAD_VIEW          0x0111
#define BTN_PAD_MENU          0x0112
#define BTN_PAD_XBOX          0x0113

#define BTN_KB_M2             0x02008E
#define BTN_KB_M1             0x02008F
#define BTN_KB_ESC            0x020076
#define BTN_KB_F1             0x020050
#define BTN_KB_F2             0x020060
#define BTN_KB_F3             0x020040
#define BTN_KB_F4             0x02000C
#define BTN_KB_F5             0x020003
#define BTN_KB_F6             0x02000B
#define BTN_KB_F7             0x020080
#define BTN_KB_F8             0x02000A
#define BTN_KB_F9             0x020001
#define BTN_KB_F10            0x020009
#define BTN_KB_F11            0x020078
#define BTN_KB_F12            0x020007
#define BTN_KB_BACKTICK       0x02000E
#define BTN_KB_1              0x020016
#define BTN_KB_2              0x02001E
#define BTN_KB_3              0x020026
#define BTN_KB_4              0x020025
#define BTN_KB_5              0x02002E
#define BTN_KB_6              0x020036
#define BTN_KB_7              0x02003D
#define BTN_KB_8              0x02003E
#define BTN_KB_9              0x020046
#define BTN_KB_0              0x020045
#define BTN_KB_HYPHEN         0x02004E
#define BTN_KB_EQUALS         0x020055
#define BTN_KB_BACKSPACE      0x020066
#define BTN_KB_TAB            0x02000D
#define BTN_KB_Q              0x020015
#define BTN_KB_W              0x02001D
#define BTN_KB_E              0x020024
#define BTN_KB_R              0x02002D
#define BTN_KB_T              0x02002C
#define BTN_KB_Y              0x020035
#define BTN_KB_U              0x02003C
#define BTN_KB_O              0x020044
#define BTN_KB_P              0x02004D
#define BTN_KB_LBRACKET       0x020054
#define BTN_KB_RBRACKET       0x02005B
#define BTN_KB_BACKSLASH      0x02005D
#define BTN_KB_CAPS           0x020058
#define BTN_KB_A              0x02001C
#define BTN_KB_S              0x02001B
#define BTN_KB_D              0x020023
#define BTN_KB_F              0x02002B
#define BTN_KB_G              0x020034
#define BTN_KB_H              0x020033
#define BTN_KB_J              0x02003B
#define BTN_KB_K              0x020042
#define BTN_KB_L              0x02004B
#define BTN_KB_SEMI           0x02004C
#define BTN_KB_QUOTE          0x020052
#define BTN_KB_RET            0x02005A
#define BTN_KB_LSHIFT         0x020088
#define BTN_KB_Z              0x02001A
#define BTN_KB_X              0x020022
#define BTN_KB_C              0x020021
#define BTN_KB_V              0x02002A
#define BTN_KB_B              0x020032
#define BTN_KB_N              0x020031
#define BTN_KB_M              0x02003A
#define BTN_KB_COMMA          0x020041
#define BTN_KB_PERIOD         0x020049
#define BTN_KB_RSHIFT         0x020089
#define BTN_KB_LCTL           0x02008C
#define BTN_KB_META           0x020082
#define BTN_KB_LALT           0x02008A
#define BTN_KB_SPACE          0x020029
#define BTN_KB_RALT           0x02008B
#define BTN_KB_MENU           0x020084
#define BTN_KB_RCTL           0x02008D
#define BTN_KB_PRNTSCN        0x0200C3
#define BTN_KB_SCRLCK         0x02007E
#define BTN_KB_PAUSE          0x020091
#define BTN_KB_INS            0x0200C2
#define BTN_KB_HOME           0x020094
#define BTN_KB_PGUP           0x020096
#define BTN_KB_DEL            0x0200C0
#define BTN_KB_END            0x020095
#define BTN_KB_PGDWN          0x020097
#define BTN_KB_UP_ARROW       0x020098
#define BTN_KB_DOWN_ARROW     0x020099
#define BTN_KB_LEFT_ARROW     0x020091
#define BTN_KB_RIGHT_ARROW    0x02009B

#define BTN_NUMPAD_LOCK       0x020077
#define BTN_NUMPAD_FWDSLASH   0x020090
#define BTN_NUMPAD_ASTERISK   0x02007C
#define BTN_NUMPAD_HYPHEN     0x02007B
#define BTN_NUMPAD_0          0x020070
#define BTN_NUMPAD_1          0x020069
#define BTN_NUMPAD_2          0x020072
#define BTN_NUMPAD_3          0x02007A
#define BTN_NUMPAD_4          0x02006B
#define BTN_NUMPAD_5          0x020073
#define BTN_NUMPAD_6          0x020074
#define BTN_NUMPAD_7          0x02006C
#define BTN_NUMPAD_8          0x020075
#define BTN_NUMPAD_9          0x02007D
#define BTN_NUMPAD_PLUS       0x020079
#define BTN_NUMPAD_ENTER      0x020081
#define BTN_NUMPAD_PERIOD     0x020071

#define BTN_MOUSE_LCLICK      0x0300000001
#define BTN_MOUSE_RCLICK      0x0300000002
#define BTN_MOUSE_MCLICK      0x0300000003
#define BTN_MOUSE_WHEEL_UP    0x0300000004
#define BTN_MOUSE_WHEEL_DOWN  0x0300000005

#define BTN_MEDIA_SCREENSHOT      0x05000016
#define BTN_MEDIA_SHOW_KEYBOARD   0x05000019
#define BTN_MEDIA_SHOW_DESKTOP    0x0500001C
#define BTN_MEDIA_START_RECORDING 0x0500001E
#define BTN_MEDIA_MIC_OFF         0x05000001
#define BTN_MEDIA_VOL_DOWN        0x05000002
#define BTN_MEDIA_VOL_UP          0x05000003

#define ALLY_DEVICE_ATTR_WO(_name, _sysfs_name)    \
	struct device_attribute dev_attr_##_name = \
		__ATTR(_sysfs_name, 0200, NULL, _name##_store)

/* required so we can have nested attributes with same name but different functions */
#define ALLY_DEVICE_ATTR_RW(_name, _sysfs_name)    \
	struct device_attribute dev_attr_##_name = \
		__ATTR(_sysfs_name, 0644, _name##_show, _name##_store)

/* button specific macros */
#define ALLY_BTN_SHOW(_fname, _btn_name, _secondary)                           \
	static ssize_t _fname##_show(struct device *dev,                       \
				     struct device_attribute *attr, char *buf) \
	{                                                                      \
		struct ally_gamepad_cfg *ally_cfg = drvdata.gamepad_cfg;       \
		struct btn_data *btn;                                          \
		const char* name;                                              \
		if (!drvdata.gamepad_cfg)                                      \
			return -ENODEV;                                        \
		btn = &ally_cfg->key_mapping[ally_cfg->mode - 1]._btn_name;   \
		name = btn_to_name(_secondary ? btn->macro : btn->button);     \
		return sysfs_emit(buf, "%s\n", name);                          \
	}

#define ALLY_BTN_STORE(_fname, _btn_name, _secondary)                          \
	static ssize_t _fname##_store(struct device *dev,                      \
				      struct device_attribute *attr,           \
				      const char *buf, size_t count)           \
	{                                                                      \
		struct ally_gamepad_cfg *ally_cfg = drvdata.gamepad_cfg;       \
		struct btn_data *btn;                                          \
		u64 code;                                                      \
		if (!drvdata.gamepad_cfg)                                      \
			return -ENODEV;                                        \
		btn = &ally_cfg->key_mapping[ally_cfg->mode - 1]._btn_name;   \
		code = name_to_btn(buf);                                       \
		if (_secondary)                                                \
			btn->macro = code;                                     \
		else                                                           \
			btn->button = code;                                    \
		return count;                                                  \
	}

#define ALLY_BTN_ATTRS_GROUP(_name, _fname)                               \
	static struct attribute *_fname##_attrs[] = {                     \
		&dev_attr_##_fname.attr,                                  \
		&dev_attr_##_fname##_macro.attr,                          \
	};                                                                \
	static const struct attribute_group _fname##_attr_group = {       \
		.name = __stringify(_name),                               \
		.attrs = _fname##_attrs,                                  \
	}

#define ALLY_BTN_MAPPING(_fname, _mname)                                \
	ALLY_BTN_SHOW(btn_mapping_##_fname, _mname, false);             \
	ALLY_BTN_STORE(btn_mapping_##_fname, _mname, false);            \
	ALLY_BTN_SHOW(btn_mapping_##_fname##_macro, _mname, true);      \
	ALLY_BTN_STORE(btn_mapping_##_fname##_macro, _mname, true);     \
	ALLY_DEVICE_ATTR_RW(btn_mapping_##_fname, remap);               \
	ALLY_DEVICE_ATTR_RW(btn_mapping_##_fname##_macro, macro_remap); \
	ALLY_BTN_ATTRS_GROUP(btn_##_fname, btn_mapping_##_fname)
