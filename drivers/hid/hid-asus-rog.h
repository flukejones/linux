enum ally_gamepad_cmd {
	ally_gamepad_cmd_set_mode		= 0x01,
	ally_gamepad_cmd_check_ready		= 0x0A,
};

enum ally_gamepad_mode {
	ally_gamepad_mode_game	= 0x01,
	ally_gamepad_mode_wasd	= 0x02,
	ally_gamepad_mode_mouse	= 0x03,
};

/* ROG Ally has many settings related to the gamepad, all using the same n-key endpoint */
struct asus_rog_ally {
	enum ally_gamepad_mode mode;
};