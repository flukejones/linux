// SPDX-License-Identifier: GPL-2.0-only
/*
 * 3-axis accelerometer driver supporting many Bosch-Sensortec chips
 * Copyright (c) 2014, Intel Corporation.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/acpi.h>
#include <linux/of_irq.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include "bmc150-accel.h"

#define BMC150_ACCEL_DRV_NAME			"bmc150_accel"
#define BMC150_ACCEL_IRQ_NAME			"bmc150_accel_event"

#define BMC150_ACCEL_REG_CHIP_ID		0x00

#define BMC150_ACCEL_REG_INT_STATUS_2		0x0B
#define BMC150_ACCEL_ANY_MOTION_MASK		0x07
#define BMC150_ACCEL_ANY_MOTION_BIT_X		BIT(0)
#define BMC150_ACCEL_ANY_MOTION_BIT_Y		BIT(1)
#define BMC150_ACCEL_ANY_MOTION_BIT_Z		BIT(2)
#define BMC150_ACCEL_ANY_MOTION_BIT_SIGN	BIT(3)

#define BMC150_ACCEL_REG_PMU_LPW		0x11
#define BMC150_ACCEL_PMU_MODE_MASK		0xE0
#define BMC150_ACCEL_PMU_MODE_SHIFT		5
#define BMC150_ACCEL_PMU_BIT_SLEEP_DUR_MASK	0x17
#define BMC150_ACCEL_PMU_BIT_SLEEP_DUR_SHIFT	1

#define BMC150_ACCEL_REG_PMU_RANGE		0x0F

#define BMC150_ACCEL_DEF_RANGE_2G		0x03
#define BMC150_ACCEL_DEF_RANGE_4G		0x05
#define BMC150_ACCEL_DEF_RANGE_8G		0x08
#define BMC150_ACCEL_DEF_RANGE_16G		0x0C

/* Default BW: 125Hz */
#define BMC150_ACCEL_REG_PMU_BW		0x10
#define BMC150_ACCEL_DEF_BW			125

#define BMC150_ACCEL_REG_RESET			0x14
#define BMC150_ACCEL_RESET_VAL			0xB6

#define BMC150_ACCEL_REG_INT_MAP_0		0x19
#define BMC150_ACCEL_INT_MAP_0_BIT_INT1_SLOPE	BIT(2)

#define BMC150_ACCEL_REG_INT_MAP_1		0x1A
#define BMC150_ACCEL_INT_MAP_1_BIT_INT1_DATA	BIT(0)
#define BMC150_ACCEL_INT_MAP_1_BIT_INT1_FWM	BIT(1)
#define BMC150_ACCEL_INT_MAP_1_BIT_INT1_FFULL	BIT(2)
#define BMC150_ACCEL_INT_MAP_1_BIT_INT2_FFULL	BIT(5)
#define BMC150_ACCEL_INT_MAP_1_BIT_INT2_FWM	BIT(6)
#define BMC150_ACCEL_INT_MAP_1_BIT_INT2_DATA	BIT(7)

#define BMC150_ACCEL_REG_INT_MAP_2		0x1B
#define BMC150_ACCEL_INT_MAP_2_BIT_INT2_SLOPE	BIT(2)

#define BMC150_ACCEL_REG_INT_RST_LATCH		0x21
#define BMC150_ACCEL_INT_MODE_LATCH_RESET	0x80
#define BMC150_ACCEL_INT_MODE_LATCH_INT	0x0F
#define BMC150_ACCEL_INT_MODE_NON_LATCH_INT	0x00

#define BMC150_ACCEL_REG_INT_EN_0		0x16
#define BMC150_ACCEL_INT_EN_BIT_SLP_X		BIT(0)
#define BMC150_ACCEL_INT_EN_BIT_SLP_Y		BIT(1)
#define BMC150_ACCEL_INT_EN_BIT_SLP_Z		BIT(2)

#define BMC150_ACCEL_REG_INT_EN_1		0x17
#define BMC150_ACCEL_INT_EN_BIT_DATA_EN		BIT(4)
#define BMC150_ACCEL_INT_EN_BIT_FFULL_EN	BIT(5)
#define BMC150_ACCEL_INT_EN_BIT_FWM_EN		BIT(6)

#define BMC150_ACCEL_REG_INT_OUT_CTRL		0x20
#define BMC150_ACCEL_INT_OUT_CTRL_INT1_LVL	BIT(0)
#define BMC150_ACCEL_INT_OUT_CTRL_INT2_LVL	BIT(2)

#define BMC150_ACCEL_REG_INT_5			0x27
#define BMC150_ACCEL_SLOPE_DUR_MASK		0x03

#define BMC150_ACCEL_REG_INT_6			0x28
#define BMC150_ACCEL_SLOPE_THRES_MASK		0xFF

/* Slope duration in terms of number of samples */
#define BMC150_ACCEL_DEF_SLOPE_DURATION		1
/* in terms of multiples of g's/LSB, based on range */
#define BMC150_ACCEL_DEF_SLOPE_THRESHOLD	1

#define BMC150_ACCEL_REG_XOUT_L		0x02

#define BMC150_ACCEL_MAX_STARTUP_TIME_MS	100

/* Sleep Duration values */
#define BMC150_ACCEL_SLEEP_500_MICRO		0x05
#define BMC150_ACCEL_SLEEP_1_MS		0x06
#define BMC150_ACCEL_SLEEP_2_MS		0x07
#define BMC150_ACCEL_SLEEP_4_MS		0x08
#define BMC150_ACCEL_SLEEP_6_MS		0x09
#define BMC150_ACCEL_SLEEP_10_MS		0x0A
#define BMC150_ACCEL_SLEEP_25_MS		0x0B
#define BMC150_ACCEL_SLEEP_50_MS		0x0C
#define BMC150_ACCEL_SLEEP_100_MS		0x0D
#define BMC150_ACCEL_SLEEP_500_MS		0x0E
#define BMC150_ACCEL_SLEEP_1_SEC		0x0F

#define BMC150_ACCEL_REG_TEMP			0x08
#define BMC150_ACCEL_TEMP_CENTER_VAL		23

#define BMC150_ACCEL_AXIS_TO_REG(axis)	(BMC150_ACCEL_REG_XOUT_L + (axis * 2))
#define BMC150_AUTO_SUSPEND_DELAY_MS		2000

#define BMC150_ACCEL_REG_FIFO_STATUS		0x0E
#define BMC150_ACCEL_REG_FIFO_CONFIG0		0x30
#define BMC150_ACCEL_REG_FIFO_CONFIG1		0x3E
#define BMC150_ACCEL_REG_FIFO_DATA		0x3F
#define BMC150_ACCEL_FIFO_LENGTH		32

#define BMC150_BMI323_TEMPER_CENTER_VAL 23
#define BMC150_BMI323_TEMPER_LSB_PER_KELVIN_VAL 512

#define BMC150_BMI323_AUTO_SUSPEND_DELAY_MS 2000

#define BMC150_BMI323_CHIP_ID_REG 0x00
#define BMC150_BMI323_SOFT_RESET_REG 0x7E
#define BMC150_BMI323_SOFT_RESET_VAL 0xDEAFU
#define BMC150_BMI323_DATA_BASE_REG 0x03
#define BMC150_BMI323_TEMPERATURE_DATA_REG 0x09
#define BMC150_BMI323_FIFO_FILL_LEVEL_REG 0x15
#define BMC150_BMI323_FIFO_DATA_REG 0x16
#define BMC150_BMI323_ACC_CONF_REG 0x20
#define BMC150_BMI323_GYR_CONF_REG 0x21
#define BMC150_BMI323_FIFO_CONF_REG 0x36

// these are bits [0:3] of ACC_CONF.acc_odr, sample rate in Hz for the accel part of the chip
#define BMC150_BMI323_ACCEL_ODR_0_78123_VAL 0x0001
#define BMC150_BMI323_ACCEL_ODR_1_5625_VAL 0x0002
#define BMC150_BMI323_ACCEL_ODR_3_125_VAL 0x0003
#define BMC150_BMI323_ACCEL_ODR_6_25_VAL 0x0004
#define BMC150_BMI323_ACCEL_ODR_12_5_VAL 0x0005
#define BMC150_BMI323_ACCEL_ODR_25_VAL 0x0006
#define BMC150_BMI323_ACCEL_ODR_50_VAL 0x0007
#define BMC150_BMI323_ACCEL_ODR_100_VAL 0x0008
#define BMC150_BMI323_ACCEL_ODR_200_VAL 0x0009
#define BMC150_BMI323_ACCEL_ODR_400_VAL 0x000A
#define BMC150_BMI323_ACCEL_ODR_800_VAL 0x000B
#define BMC150_BMI323_ACCEL_ODR_1600_VAL 0x000C
#define BMC150_BMI323_ACCEL_ODR_3200_VAL 0x000D
#define BMC150_BMI323_ACCEL_ODR_6400_VAL 0x000E

#define BMC150_BMI323_ACCEL_BW_ODR_2_VAL 0x0000
#define BMC150_BMI323_ACCEL_BW_ODR_4_VAL 0x0001

// these are bits [4:6] of ACC_CONF.acc_range, full scale resolution
#define BMC150_BMI323_ACCEL_RANGE_2_VAL 0x0000 // +/-2g, 16.38 LSB/mg
#define BMC150_BMI323_ACCEL_RANGE_4_VAL 0x0001 // +/-4g, 8.19 LSB/mg
#define BMC150_BMI323_ACCEL_RANGE_8_VAL 0x0002 // +/-8g, 4.10 LSB/mg
#define BMC150_BMI323_ACCEL_RANGE_16_VAL 0x0003 // +/-4g, 2.05 LSB/mg

// these are bits [0:3] of GYR_CONF.gyr_odr, sample rate in Hz for the gyro part of the chip
#define BMC150_BMI323_GYRO_ODR_0_78123_VAL 0x0001
#define BMC150_BMI323_GYRO_ODR_1_5625_VAL 0x0002
#define BMC150_BMI323_GYRO_ODR_3_125_VAL 0x0003
#define BMC150_BMI323_GYRO_ODR_6_25_VAL 0x0004
#define BMC150_BMI323_GYRO_ODR_12_5_VAL 0x0005
#define BMC150_BMI323_GYRO_ODR_25_VAL 0x0006
#define BMC150_BMI323_GYRO_ODR_50_VAL 0x0007
#define BMC150_BMI323_GYRO_ODR_100_VAL 0x0008
#define BMC150_BMI323_GYRO_ODR_200_VAL 0x0009
#define BMC150_BMI323_GYRO_ODR_400_VAL 0x000A
#define BMC150_BMI323_GYRO_ODR_800_VAL 0x000B
#define BMC150_BMI323_GYRO_ODR_1600_VAL 0x000C
#define BMC150_BMI323_GYRO_ODR_3200_VAL 0x000D
#define BMC150_BMI323_GYRO_ODR_6400_VAL 0x000E

#define BMC150_BMI323_GYRO_BW_ODR_2_VAL 0x0000
#define BMC150_BMI323_GYRO_BW_ODR_4_VAL 0x0001

// these are bits [4:6] of GYR_CONF.gyr_range, full scale resolution
#define BMC150_BMI323_GYRO_RANGE_125_VAL 0x0000 // +/-125°/s, 262.144 LSB/°/s
#define BMC150_BMI323_GYRO_RANGE_250_VAL 0x0001 // +/-250°/s,  131.2 LSB/°/s
#define BMC150_BMI323_GYRO_RANGE_500_VAL 0x0002 // +/-500°/s,  65.6 LSB/°/s
#define BMC150_BMI323_GYRO_RANGE_1000_VAL 0x0003 // +/-1000°/s, 32.8 LSB/°/s
#define BMC150_BMI323_GYRO_RANGE_2000_VAL 0x0004 // +/-2000°/s, 16.4 LSB/°/s

enum bmc150_accel_axis {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	AXIS_MAX,
};

enum bmc150_power_modes {
	BMC150_ACCEL_SLEEP_MODE_NORMAL,
	BMC150_ACCEL_SLEEP_MODE_DEEP_SUSPEND,
	BMC150_ACCEL_SLEEP_MODE_LPM,
	BMC150_ACCEL_SLEEP_MODE_SUSPEND = 0x04,
};

struct bmc150_scale_info {
	int scale;
	u8 reg_range;
};

/*
 * This enum MUST not be altered as there are parts in the code that
 * uses an int conversion to get the correct device register to read.
 */
enum bmi323_axis {
	BMI323_ACCEL_AXIS_X = 0,
	BMI323_ACCEL_AXIS_Y,
	BMI323_ACCEL_AXIS_Z,
	BMI323_GYRO_AXIS_X,
	BMI323_GYRO_AXIS_Y,
	BMI323_GYRO_AXIS_Z,
	BMI323_TEMP,
	BMI323_AXIS_MAX,
};

static const struct bmi323_scale_accel_info {
	u8 hw_val;
	int val;
	int val2;
	int ret_type;
} bmi323_accel_scale_map[] = {
	{
		.hw_val = (u16)BMC150_BMI323_ACCEL_RANGE_2_VAL << (u16)4,
		.val = 0,
		.val2 = 598,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_ACCEL_RANGE_4_VAL << (u16)4,
		.val = 0,
		.val2 = 1196,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_ACCEL_RANGE_8_VAL << (u16)4,
		.val = 0,
		.val2 = 2392,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_ACCEL_RANGE_16_VAL << (u16)4,
		.val = 0,
		.val2 = 4785,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
};

static const struct bmi323_scale_gyro_info {
	u8 hw_val;
	int val;
	int val2;
	int ret_type;
} bmi323_gyro_scale_map[] = {
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_125_VAL << (u16)4,
		.val = 0,
		.val2 = 66545,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_125_VAL << (u16)4,
		.val = 0,
		.val2 = 66,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_250_VAL << (u16)4,
		.val = 0,
		.val2 = 133090,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_250_VAL << (u16)4,
		.val = 0,
		.val2 = 133,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_500_VAL << (u16)4,
		.val = 0,
		.val2 = 266181,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_500_VAL << (u16)4,
		.val = 0,
		.val2 = 266,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_1000_VAL << (u16)4,
		.val = 0,
		.val2 = 532362,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_1000_VAL << (u16)4,
		.val = 0,
		.val2 = 532,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_2000_VAL << (u16)4,
		.val = 0,
		.val2 = 1064724,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		// this shouldn't be necessary, but iio seems to have a wrong rounding of this value...
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_2000_VAL << (u16)4,
		.val = 0,
		.val2 = 1064,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
	{
		.hw_val = (u16)BMC150_BMI323_GYRO_RANGE_2000_VAL << (u16)4,
		.val = 0,
		.val2 = 1065,
		.ret_type = IIO_VAL_INT_PLUS_NANO,
	},
};

/*
 * this reflects the frequency map that is following.
 * For each index i of that map index i*2 and i*2+1 of of this
 * holds ODR/2 and ODR/4
 */
static const struct bmi323_3db_freq_cutoff_accel_info {
	int val;
	int val2;
	int ret_type;
} bmi323_accel_3db_freq_cutoff[] = {
	{
		.val = 0,
		.val2 = 390615,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 195308,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 781300,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 390650,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1,
		.val2 = 562500,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 78125,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 3,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1,
		.val2 = 500000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 6,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 3,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 12,
		.val2 = 500000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 6,
		.val2 = 250000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 25,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 12,
		.val2 = 500000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 50,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 25,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 100,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 50,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 200,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 100,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 400,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 200,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 800,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 400,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1600,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 800,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1600,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 800,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 3200,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1600,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
};

static const struct bmi323_freq_accel_info {
	u8 hw_val;
	int val;
	int val2;
	s64 time_ns;
} bmi323_accel_odr_map[] = {
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_0_78123_VAL,
		.val = 0,
		.val2 = 781230,
		.time_ns = 1280032769,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_1_5625_VAL,
		.val = 1,
		.val2 = 562600,
		.time_ns = 886522247,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_3_125_VAL,
		.val = 3,
		.val2 = 125000,
		.time_ns = 320000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_6_25_VAL,
		.val = 6,
		.val2 = 250000,
		.time_ns = 160000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_12_5_VAL,
		.val = 12,
		.val2 = 500000,
		.time_ns = 80000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_25_VAL,
		.val = 25,
		.val2 = 0,
		.time_ns = 40000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_50_VAL,
		.val = 50,
		.val2 = 0,
		.time_ns = 20000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_100_VAL,
		.val = 100,
		.val2 = 0,
		.time_ns = 10000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_200_VAL,
		.val = 200,
		.val2 = 0,
		.time_ns = 5000000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_400_VAL,
		.val = 400,
		.val2 = 0,
		.time_ns = 2500000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_800_VAL,
		.val = 800,
		.val2 = 0,
		.time_ns = 1250000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_1600_VAL,
		.val = 1600,
		.val2 = 0,
		.time_ns = 625000,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_3200_VAL,
		.val = 3200,
		.val2 = 0,
		.time_ns = 312500,
	},
	{
		.hw_val = BMC150_BMI323_ACCEL_ODR_6400_VAL,
		.val = 6400,
		.val2 = 0,
		.time_ns = 156250,
	},
};

static const struct bmi323_freq_gyro_info {
	u8 hw_val;
	int val;
	int val2;
	s64 time_ns;
} bmi323_gyro_odr_map[] = {
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_0_78123_VAL,
		.val = 0,
		.val2 = 781230,
		.time_ns = 1280032769,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_1_5625_VAL,
		.val = 1,
		.val2 = 562600,
		.time_ns = 886522247,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_3_125_VAL,
		.val = 3,
		.val2 = 125000,
		.time_ns = 320000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_6_25_VAL,
		.val = 6,
		.val2 = 250000,
		.time_ns = 160000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_12_5_VAL,
		.val = 12,
		.val2 = 500000,
		.time_ns = 80000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_25_VAL,
		.val = 25,
		.val2 = 0,
		.time_ns = 40000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_50_VAL,
		.val = 50,
		.val2 = 0,
		.time_ns = 20000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_100_VAL,
		.val = 100,
		.val2 = 0,
		.time_ns = 10000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_200_VAL,
		.val = 200,
		.val2 = 0,
		.time_ns = 5000000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_400_VAL,
		.val = 400,
		.val2 = 0,
		.time_ns = 2500000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_800_VAL,
		.val = 800,
		.val2 = 0,
		.time_ns = 1250000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_1600_VAL,
		.val = 1600,
		.val2 = 0,
		.time_ns = 625000,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_3200_VAL,
		.val = 3200,
		.val2 = 0,
		.time_ns = 312500,
	},
	{
		.hw_val = BMC150_BMI323_GYRO_ODR_6400_VAL,
		.val = 6400,
		.val2 = 0,
		.time_ns = 156250,
	},
};

static const struct bmi323_3db_freq_cutoff_gyro_info {
	int val;
	int val2;
	int ret_type;
} bmi323_gyro_3db_freq_cutoff[] = {
	{
		.val = 0,
		.val2 = 390615,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 1953075, // TODO: check if this gets reported correctly...
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 781300,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 390650,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1,
		.val2 = 562500,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 0,
		.val2 = 78125,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 3,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1,
		.val2 = 500000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 6,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 3,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 12,
		.val2 = 500000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 6,
		.val2 = 250000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 25,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 12,
		.val2 = 500000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 50,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 25,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 100,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 50,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 200,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 100,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 400,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 200,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 800,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 400,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1600,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 800,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1600,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 800,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 3200,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
	{
		.val = 1600,
		.val2 = 000000,
		.ret_type = IIO_VAL_INT_PLUS_MICRO,
	},
};

static const int bmi323_accel_scales[] = {
	0, 598, 0, 1196, 0, 2392, 0, 4785,
};

static const int bmi323_gyro_scales[] = {
	0, 66545, 0, 133090, 0, 266181, 0, 532362, 0, 1064724,
};

static const int bmi323_sample_freqs[] = {
	0,   781230, 1,	   562600, 3,	 125000, 6,    250000, 12,  500000,
	25,  0,	     50,   0,	   100,	 0,	 200,  0,      400, 0,
	800, 0,	     1600, 0,	   3200, 0,	 6400, 0,
};

static const struct {
	int val;
	int val2; // IIO_VAL_INT_PLUS_MICRO
	u8 bw_bits;
} bmi323_samp_freq_table[] = { { 15, 620000, 0x08 }, { 31, 260000, 0x09 },
			       { 62, 500000, 0x0A }, { 125, 0, 0x0B },
			       { 250, 0, 0x0C },     { 500, 0, 0x0D },
			       { 1000, 0, 0x0E },    { 2000, 0, 0x0F } };

struct bmc150_accel_chip_info {
	const char *name;
	u8 chip_id;
	const struct iio_chan_spec *channels;
	int num_channels;
	const struct bmc150_scale_info scale_table[4];
};

static const struct {
	int val;
	int val2;
	u8 bw_bits;
} bmc150_accel_samp_freq_table[] = { {15, 620000, 0x08},
				     {31, 260000, 0x09},
				     {62, 500000, 0x0A},
				     {125, 0, 0x0B},
				     {250, 0, 0x0C},
				     {500, 0, 0x0D},
				     {1000, 0, 0x0E},
				     {2000, 0, 0x0F} };

static __maybe_unused const struct {
	int bw_bits;
	int msec;
} bmc150_accel_sample_upd_time[] = { {0x08, 64},
				     {0x09, 32},
				     {0x0A, 16},
				     {0x0B, 8},
				     {0x0C, 4},
				     {0x0D, 2},
				     {0x0E, 1},
				     {0x0F, 1} };

static const struct {
	int sleep_dur;
	u8 reg_value;
} bmc150_accel_sleep_value_table[] = { {0, 0},
				       {500, BMC150_ACCEL_SLEEP_500_MICRO},
				       {1000, BMC150_ACCEL_SLEEP_1_MS},
				       {2000, BMC150_ACCEL_SLEEP_2_MS},
				       {4000, BMC150_ACCEL_SLEEP_4_MS},
				       {6000, BMC150_ACCEL_SLEEP_6_MS},
				       {10000, BMC150_ACCEL_SLEEP_10_MS},
				       {25000, BMC150_ACCEL_SLEEP_25_MS},
				       {50000, BMC150_ACCEL_SLEEP_50_MS},
				       {100000, BMC150_ACCEL_SLEEP_100_MS},
				       {500000, BMC150_ACCEL_SLEEP_500_MS},
				       {1000000, BMC150_ACCEL_SLEEP_1_SEC} };

const struct regmap_config bmc150_regmap_conf = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x3f,
};
EXPORT_SYMBOL_NS_GPL(bmc150_regmap_conf, IIO_BMC150);

static int bmc150_accel_set_mode(struct bmc150_accel_data *data,
				 enum bmc150_power_modes mode,
				 int dur_us)
{
	struct device *dev = regmap_get_device(data->regmap);
	int i;
	int ret;
	u8 lpw_bits;
	int dur_val = -1;

	if (dur_us > 0) {
		for (i = 0; i < ARRAY_SIZE(bmc150_accel_sleep_value_table);
									 ++i) {
			if (bmc150_accel_sleep_value_table[i].sleep_dur ==
									dur_us)
				dur_val =
				bmc150_accel_sleep_value_table[i].reg_value;
		}
	} else {
		dur_val = 0;
	}

	if (dur_val < 0)
		return -EINVAL;

	lpw_bits = mode << BMC150_ACCEL_PMU_MODE_SHIFT;
	lpw_bits |= (dur_val << BMC150_ACCEL_PMU_BIT_SLEEP_DUR_SHIFT);

	dev_dbg(dev, "Set Mode bits %x\n", lpw_bits);

	ret = regmap_write(data->regmap, BMC150_ACCEL_REG_PMU_LPW, lpw_bits);
	if (ret < 0) {
		dev_err(dev, "Error writing reg_pmu_lpw\n");
		return ret;
	}

	return 0;
}

static int bmc150_accel_set_bw(struct bmc150_accel_data *data, int val,
			       int val2)
{
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(bmc150_accel_samp_freq_table); ++i) {
		if (bmc150_accel_samp_freq_table[i].val == val &&
		    bmc150_accel_samp_freq_table[i].val2 == val2) {
			ret = regmap_write(data->regmap,
				BMC150_ACCEL_REG_PMU_BW,
				bmc150_accel_samp_freq_table[i].bw_bits);
			if (ret < 0)
				return ret;

			data->bw_bits =
				bmc150_accel_samp_freq_table[i].bw_bits;
			return 0;
		}
	}

	return -EINVAL;
}

static int bmc150_accel_update_slope(struct bmc150_accel_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	ret = regmap_write(data->regmap, BMC150_ACCEL_REG_INT_6,
					data->slope_thres);
	if (ret < 0) {
		dev_err(dev, "Error writing reg_int_6\n");
		return ret;
	}

	ret = regmap_update_bits(data->regmap, BMC150_ACCEL_REG_INT_5,
				 BMC150_ACCEL_SLOPE_DUR_MASK, data->slope_dur);
	if (ret < 0) {
		dev_err(dev, "Error updating reg_int_5\n");
		return ret;
	}

	dev_dbg(dev, "%x %x\n", data->slope_thres, data->slope_dur);

	return ret;
}

static int bmc150_accel_any_motion_setup(struct bmc150_accel_trigger *t,
					 bool state)
{
	if (state)
		return bmc150_accel_update_slope(t->data);

	return 0;
}

static int bmc150_accel_get_bw(struct bmc150_accel_data *data, int *val,
			       int *val2)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bmc150_accel_samp_freq_table); ++i) {
		if (bmc150_accel_samp_freq_table[i].bw_bits == data->bw_bits) {
			*val = bmc150_accel_samp_freq_table[i].val;
			*val2 = bmc150_accel_samp_freq_table[i].val2;
			return IIO_VAL_INT_PLUS_MICRO;
		}
	}

	return -EINVAL;
}

#ifdef CONFIG_PM
static int bmc150_accel_get_startup_times(struct bmc150_accel_data *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bmc150_accel_sample_upd_time); ++i) {
		if (bmc150_accel_sample_upd_time[i].bw_bits == data->bw_bits)
			return bmc150_accel_sample_upd_time[i].msec;
	}

	return BMC150_ACCEL_MAX_STARTUP_TIME_MS;
}

static int bmc150_accel_set_power_state(struct bmc150_accel_data *data, bool on)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	if (on) {
		ret = pm_runtime_resume_and_get(dev);
	} else {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_autosuspend(dev);
	}

	if (ret < 0) {
		dev_err(dev,
			"Failed: %s for %d\n", __func__, on);
		return ret;
	}

	return 0;
}
#else
static int bmc150_accel_set_power_state(struct bmc150_accel_data *data, bool on)
{
	return 0;
}
#endif

#ifdef CONFIG_ACPI
/*
 * Support for getting accelerometer information from BOSC0200 ACPI nodes.
 *
 * There are 2 variants of the BOSC0200 ACPI node. Some 2-in-1s with 360 degree
 * hinges declare 2 I2C ACPI-resources for 2 accelerometers, 1 in the display
 * and 1 in the base of the 2-in-1. On these 2-in-1s the ROMS ACPI object
 * contains the mount-matrix for the sensor in the display and ROMK contains
 * the mount-matrix for the sensor in the base. On devices using a single
 * sensor there is a ROTM ACPI object which contains the mount-matrix.
 *
 * Here is an incomplete list of devices known to use 1 of these setups:
 *
 * Yoga devices with 2 accelerometers using ROMS + ROMK for the mount-matrices:
 * Lenovo Thinkpad Yoga 11e 3th gen
 * Lenovo Thinkpad Yoga 11e 4th gen
 *
 * Tablets using a single accelerometer using ROTM for the mount-matrix:
 * Chuwi Hi8 Pro (CWI513)
 * Chuwi Vi8 Plus (CWI519)
 * Chuwi Hi13
 * Irbis TW90
 * Jumper EZpad mini 3
 * Onda V80 plus
 * Predia Basic Tablet
 */
static bool bmc150_apply_bosc0200_acpi_orientation(struct device *dev,
						   struct iio_mount_matrix *orientation)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct acpi_device *adev = ACPI_COMPANION(dev);
	char *name, *alt_name, *label, *str;
	union acpi_object *obj, *elements;
	acpi_status status;
	int i, j, val[3];

	if (strcmp(dev_name(dev), "i2c-BOSC0200:base") == 0) {
		alt_name = "ROMK";
		label = "accel-base";
	} else {
		alt_name = "ROMS";
		label = "accel-display";
	}

	if (acpi_has_method(adev->handle, "ROTM")) {
		name = "ROTM";
	} else if (acpi_has_method(adev->handle, alt_name)) {
		name = alt_name;
		indio_dev->label = label;
	} else {
		return false;
	}

	status = acpi_evaluate_object(adev->handle, name, NULL, &buffer);
	if (ACPI_FAILURE(status)) {
		dev_warn(dev, "Failed to get ACPI mount matrix: %d\n", status);
		return false;
	}

	obj = buffer.pointer;
	if (obj->type != ACPI_TYPE_PACKAGE || obj->package.count != 3)
		goto unknown_format;

	elements = obj->package.elements;
	for (i = 0; i < 3; i++) {
		if (elements[i].type != ACPI_TYPE_STRING)
			goto unknown_format;

		str = elements[i].string.pointer;
		if (sscanf(str, "%d %d %d", &val[0], &val[1], &val[2]) != 3)
			goto unknown_format;

		for (j = 0; j < 3; j++) {
			switch (val[j]) {
			case -1: str = "-1"; break;
			case 0:  str = "0";  break;
			case 1:  str = "1";  break;
			default: goto unknown_format;
			}
			orientation->rotation[i * 3 + j] = str;
		}
	}

	kfree(buffer.pointer);
	return true;

unknown_format:
	dev_warn(dev, "Unknown ACPI mount matrix format, ignoring\n");
	kfree(buffer.pointer);
	return false;
}

static bool bmc150_apply_dual250e_acpi_orientation(struct device *dev,
						   struct iio_mount_matrix *orientation)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	if (strcmp(dev_name(dev), "i2c-DUAL250E:base") == 0)
		indio_dev->label = "accel-base";
	else
		indio_dev->label = "accel-display";

	return false; /* DUAL250E fwnodes have no mount matrix info */
}

static bool bmc150_apply_acpi_orientation(struct device *dev,
					  struct iio_mount_matrix *orientation)
{
	struct acpi_device *adev = ACPI_COMPANION(dev);

	if (adev && acpi_dev_hid_uid_match(adev, "BOSC0200", NULL))
		return bmc150_apply_bosc0200_acpi_orientation(dev, orientation);

	if (adev && acpi_dev_hid_uid_match(adev, "DUAL250E", NULL))
		return bmc150_apply_dual250e_acpi_orientation(dev, orientation);

	return false;
}
#else
static bool bmc150_apply_acpi_orientation(struct device *dev,
					  struct iio_mount_matrix *orientation)
{
	return false;
}
#endif

struct bmc150_accel_interrupt_info {
	u8 map_reg;
	u8 map_bitmask;
	u8 en_reg;
	u8 en_bitmask;
};

static const struct bmc150_accel_interrupt_info
bmc150_accel_interrupts_int1[BMC150_ACCEL_INTERRUPTS] = {
	{ /* data ready interrupt */
		.map_reg = BMC150_ACCEL_REG_INT_MAP_1,
		.map_bitmask = BMC150_ACCEL_INT_MAP_1_BIT_INT1_DATA,
		.en_reg = BMC150_ACCEL_REG_INT_EN_1,
		.en_bitmask = BMC150_ACCEL_INT_EN_BIT_DATA_EN,
	},
	{  /* motion interrupt */
		.map_reg = BMC150_ACCEL_REG_INT_MAP_0,
		.map_bitmask = BMC150_ACCEL_INT_MAP_0_BIT_INT1_SLOPE,
		.en_reg = BMC150_ACCEL_REG_INT_EN_0,
		.en_bitmask =  BMC150_ACCEL_INT_EN_BIT_SLP_X |
			BMC150_ACCEL_INT_EN_BIT_SLP_Y |
			BMC150_ACCEL_INT_EN_BIT_SLP_Z
	},
	{ /* fifo watermark interrupt */
		.map_reg = BMC150_ACCEL_REG_INT_MAP_1,
		.map_bitmask = BMC150_ACCEL_INT_MAP_1_BIT_INT1_FWM,
		.en_reg = BMC150_ACCEL_REG_INT_EN_1,
		.en_bitmask = BMC150_ACCEL_INT_EN_BIT_FWM_EN,
	},
};

static const struct bmc150_accel_interrupt_info
bmc150_accel_interrupts_int2[BMC150_ACCEL_INTERRUPTS] = {
	{ /* data ready interrupt */
		.map_reg = BMC150_ACCEL_REG_INT_MAP_1,
		.map_bitmask = BMC150_ACCEL_INT_MAP_1_BIT_INT2_DATA,
		.en_reg = BMC150_ACCEL_REG_INT_EN_1,
		.en_bitmask = BMC150_ACCEL_INT_EN_BIT_DATA_EN,
	},
	{  /* motion interrupt */
		.map_reg = BMC150_ACCEL_REG_INT_MAP_2,
		.map_bitmask = BMC150_ACCEL_INT_MAP_2_BIT_INT2_SLOPE,
		.en_reg = BMC150_ACCEL_REG_INT_EN_0,
		.en_bitmask =  BMC150_ACCEL_INT_EN_BIT_SLP_X |
			BMC150_ACCEL_INT_EN_BIT_SLP_Y |
			BMC150_ACCEL_INT_EN_BIT_SLP_Z
	},
	{ /* fifo watermark interrupt */
		.map_reg = BMC150_ACCEL_REG_INT_MAP_1,
		.map_bitmask = BMC150_ACCEL_INT_MAP_1_BIT_INT2_FWM,
		.en_reg = BMC150_ACCEL_REG_INT_EN_1,
		.en_bitmask = BMC150_ACCEL_INT_EN_BIT_FWM_EN,
	},
};

static void bmc150_accel_interrupts_setup(struct iio_dev *indio_dev,
					  struct bmc150_accel_data *data, int irq)
{
	const struct bmc150_accel_interrupt_info *irq_info = NULL;
	struct device *dev = regmap_get_device(data->regmap);
	int i;

	/*
	 * For now we map all interrupts to the same output pin.
	 * However, some boards may have just INT2 (and not INT1) connected,
	 * so we try to detect which IRQ it is based on the interrupt-names.
	 * Without interrupt-names, we assume the irq belongs to INT1.
	 */
	irq_info = bmc150_accel_interrupts_int1;
	if (data->type == BOSCH_BMC156 ||
	    irq == of_irq_get_byname(dev->of_node, "INT2"))
		irq_info = bmc150_accel_interrupts_int2;

	for (i = 0; i < BMC150_ACCEL_INTERRUPTS; i++)
		data->interrupts[i].info = &irq_info[i];
}

static int bmc150_accel_set_interrupt(struct bmc150_accel_data *data, int i,
				      bool state)
{
	struct device *dev = regmap_get_device(data->regmap);
	struct bmc150_accel_interrupt *intr = &data->interrupts[i];
	const struct bmc150_accel_interrupt_info *info = intr->info;
	int ret;

	if (state) {
		if (atomic_inc_return(&intr->users) > 1)
			return 0;
	} else {
		if (atomic_dec_return(&intr->users) > 0)
			return 0;
	}

	/*
	 * We will expect the enable and disable to do operation in reverse
	 * order. This will happen here anyway, as our resume operation uses
	 * sync mode runtime pm calls. The suspend operation will be delayed
	 * by autosuspend delay.
	 * So the disable operation will still happen in reverse order of
	 * enable operation. When runtime pm is disabled the mode is always on,
	 * so sequence doesn't matter.
	 */
	ret = bmc150_accel_set_power_state(data, state);
	if (ret < 0)
		return ret;

	/* map the interrupt to the appropriate pins */
	ret = regmap_update_bits(data->regmap, info->map_reg, info->map_bitmask,
				 (state ? info->map_bitmask : 0));
	if (ret < 0) {
		dev_err(dev, "Error updating reg_int_map\n");
		goto out_fix_power_state;
	}

	/* enable/disable the interrupt */
	ret = regmap_update_bits(data->regmap, info->en_reg, info->en_bitmask,
				 (state ? info->en_bitmask : 0));
	if (ret < 0) {
		dev_err(dev, "Error updating reg_int_en\n");
		goto out_fix_power_state;
	}

	return 0;

out_fix_power_state:
	bmc150_accel_set_power_state(data, false);
	return ret;
}

static int bmc150_accel_set_scale(struct bmc150_accel_data *data, int val)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(data->chip_info->scale_table); ++i) {
		if (data->chip_info->scale_table[i].scale == val) {
			ret = regmap_write(data->regmap,
				     BMC150_ACCEL_REG_PMU_RANGE,
				     data->chip_info->scale_table[i].reg_range);
			if (ret < 0) {
				dev_err(dev, "Error writing pmu_range\n");
				return ret;
			}

			data->range = data->chip_info->scale_table[i].reg_range;
			return 0;
		}
	}

	return -EINVAL;
}

static int bmc150_accel_get_temp(struct bmc150_accel_data *data, int *val)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;
	unsigned int value;

	mutex_lock(&data->mutex);

	ret = regmap_read(data->regmap, BMC150_ACCEL_REG_TEMP, &value);
	if (ret < 0) {
		dev_err(dev, "Error reading reg_temp\n");
		mutex_unlock(&data->mutex);
		return ret;
	}
	*val = sign_extend32(value, 7);

	mutex_unlock(&data->mutex);

	return IIO_VAL_INT;
}

static int bmc150_accel_get_axis(struct bmc150_accel_data *data,
				 struct iio_chan_spec const *chan,
				 int *val)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret;
	int axis = chan->scan_index;
	__le16 raw_val;

	mutex_lock(&data->mutex);
	ret = bmc150_accel_set_power_state(data, true);
	if (ret < 0) {
		mutex_unlock(&data->mutex);
		return ret;
	}

	ret = regmap_bulk_read(data->regmap, BMC150_ACCEL_AXIS_TO_REG(axis),
			       &raw_val, sizeof(raw_val));
	if (ret < 0) {
		dev_err(dev, "Error reading axis %d\n", axis);
		bmc150_accel_set_power_state(data, false);
		mutex_unlock(&data->mutex);
		return ret;
	}
	*val = sign_extend32(le16_to_cpu(raw_val) >> chan->scan_type.shift,
			     chan->scan_type.realbits - 1);
	ret = bmc150_accel_set_power_state(data, false);
	mutex_unlock(&data->mutex);
	if (ret < 0)
		return ret;

	return IIO_VAL_INT;
}

static int bmc150_accel_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_TEMP:
			return bmc150_accel_get_temp(data, val);
		case IIO_ACCEL:
			if (iio_buffer_enabled(indio_dev))
				return -EBUSY;
			else
				return bmc150_accel_get_axis(data, chan, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			*val = BMC150_ACCEL_TEMP_CENTER_VAL;
			return IIO_VAL_INT;
		} else {
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		switch (chan->type) {
		case IIO_TEMP:
			*val2 = 500000;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_ACCEL:
		{
			int i;
			const struct bmc150_scale_info *si;
			int st_size = ARRAY_SIZE(data->chip_info->scale_table);

			for (i = 0; i < st_size; ++i) {
				si = &data->chip_info->scale_table[i];
				if (si->reg_range == data->range) {
					*val2 = si->scale;
					return IIO_VAL_INT_PLUS_MICRO;
				}
			}
			return -EINVAL;
		}
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = bmc150_accel_get_bw(data, val, val2);
		mutex_unlock(&data->mutex);
		return ret;
	default:
		return -EINVAL;
	}
}

static int bmc150_accel_write_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  int val, int val2, long mask)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&data->mutex);
		ret = bmc150_accel_set_bw(data, val, val2);
		mutex_unlock(&data->mutex);
		break;
	case IIO_CHAN_INFO_SCALE:
		if (val)
			return -EINVAL;

		mutex_lock(&data->mutex);
		ret = bmc150_accel_set_scale(data, val2);
		mutex_unlock(&data->mutex);
		return ret;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int bmc150_accel_read_event(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info,
				   int *val, int *val2)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	*val2 = 0;
	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = data->slope_thres;
		break;
	case IIO_EV_INFO_PERIOD:
		*val = data->slope_dur;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int bmc150_accel_write_event(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info,
				    int val, int val2)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	if (data->ev_enable_state)
		return -EBUSY;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		data->slope_thres = val & BMC150_ACCEL_SLOPE_THRES_MASK;
		break;
	case IIO_EV_INFO_PERIOD:
		data->slope_dur = val & BMC150_ACCEL_SLOPE_DUR_MASK;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bmc150_accel_read_event_config(struct iio_dev *indio_dev,
					  const struct iio_chan_spec *chan,
					  enum iio_event_type type,
					  enum iio_event_direction dir)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	return data->ev_enable_state;
}

static int bmc150_accel_write_event_config(struct iio_dev *indio_dev,
					   const struct iio_chan_spec *chan,
					   enum iio_event_type type,
					   enum iio_event_direction dir,
					   int state)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	if (state == data->ev_enable_state)
		return 0;

	mutex_lock(&data->mutex);

	ret = bmc150_accel_set_interrupt(data, BMC150_ACCEL_INT_ANY_MOTION,
					 state);
	if (ret < 0) {
		mutex_unlock(&data->mutex);
		return ret;
	}

	data->ev_enable_state = state;
	mutex_unlock(&data->mutex);

	return 0;
}

static int bmc150_accel_validate_trigger(struct iio_dev *indio_dev,
					 struct iio_trigger *trig)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int i;

	for (i = 0; i < BMC150_ACCEL_TRIGGERS; i++) {
		if (data->triggers[i].indio_trig == trig)
			return 0;
	}

	return -EINVAL;
}

static ssize_t bmc150_accel_get_fifo_watermark(struct device *dev,
					       struct device_attribute *attr,
					       char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int wm;

	mutex_lock(&data->mutex);
	wm = data->watermark;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", wm);
}

static ssize_t bmc150_accel_get_fifo_state(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	bool state;

	mutex_lock(&data->mutex);
	state = data->fifo_mode;
	mutex_unlock(&data->mutex);

	return sprintf(buf, "%d\n", state);
}

static const struct iio_mount_matrix *
bmc150_accel_get_mount_matrix(const struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info bmc150_accel_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, bmc150_accel_get_mount_matrix),
	{ }
};

IIO_STATIC_CONST_DEVICE_ATTR(hwfifo_watermark_min, "1");
IIO_STATIC_CONST_DEVICE_ATTR(hwfifo_watermark_max,
			     __stringify(BMC150_ACCEL_FIFO_LENGTH));
static IIO_DEVICE_ATTR(hwfifo_enabled, S_IRUGO,
		       bmc150_accel_get_fifo_state, NULL, 0);
static IIO_DEVICE_ATTR(hwfifo_watermark, S_IRUGO,
		       bmc150_accel_get_fifo_watermark, NULL, 0);

static const struct iio_dev_attr *bmc150_accel_fifo_attributes[] = {
	&iio_dev_attr_hwfifo_watermark_min,
	&iio_dev_attr_hwfifo_watermark_max,
	&iio_dev_attr_hwfifo_watermark,
	&iio_dev_attr_hwfifo_enabled,
	NULL,
};

static int bmc150_accel_set_watermark(struct iio_dev *indio_dev, unsigned val)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	if (val > BMC150_ACCEL_FIFO_LENGTH)
		val = BMC150_ACCEL_FIFO_LENGTH;

	mutex_lock(&data->mutex);
	data->watermark = val;
	mutex_unlock(&data->mutex);

	return 0;
}

/*
 * We must read at least one full frame in one burst, otherwise the rest of the
 * frame data is discarded.
 */
static int bmc150_accel_fifo_transfer(struct bmc150_accel_data *data,
				      char *buffer, int samples)
{
	struct device *dev = regmap_get_device(data->regmap);
	int sample_length = 3 * 2;
	int ret;
	int total_length = samples * sample_length;

	ret = regmap_raw_read(data->regmap, BMC150_ACCEL_REG_FIFO_DATA,
			      buffer, total_length);
	if (ret)
		dev_err(dev,
			"Error transferring data from fifo: %d\n", ret);

	return ret;
}

static int __bmc150_accel_fifo_flush(struct iio_dev *indio_dev,
				     unsigned samples, bool irq)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	int ret, i;
	u8 count;
	u16 buffer[BMC150_ACCEL_FIFO_LENGTH * 3];
	int64_t tstamp;
	uint64_t sample_period;
	unsigned int val;

	ret = regmap_read(data->regmap, BMC150_ACCEL_REG_FIFO_STATUS, &val);
	if (ret < 0) {
		dev_err(dev, "Error reading reg_fifo_status\n");
		return ret;
	}

	count = val & 0x7F;

	if (!count)
		return 0;

	/*
	 * If we getting called from IRQ handler we know the stored timestamp is
	 * fairly accurate for the last stored sample. Otherwise, if we are
	 * called as a result of a read operation from userspace and hence
	 * before the watermark interrupt was triggered, take a timestamp
	 * now. We can fall anywhere in between two samples so the error in this
	 * case is at most one sample period.
	 */
	if (!irq) {
		data->old_timestamp = data->timestamp;
		data->timestamp = iio_get_time_ns(indio_dev);
	}

	/*
	 * Approximate timestamps for each of the sample based on the sampling
	 * frequency, timestamp for last sample and number of samples.
	 *
	 * Note that we can't use the current bandwidth settings to compute the
	 * sample period because the sample rate varies with the device
	 * (e.g. between 31.70ms to 32.20ms for a bandwidth of 15.63HZ). That
	 * small variation adds when we store a large number of samples and
	 * creates significant jitter between the last and first samples in
	 * different batches (e.g. 32ms vs 21ms).
	 *
	 * To avoid this issue we compute the actual sample period ourselves
	 * based on the timestamp delta between the last two flush operations.
	 */
	sample_period = (data->timestamp - data->old_timestamp);
	do_div(sample_period, count);
	tstamp = data->timestamp - (count - 1) * sample_period;

	if (samples && count > samples)
		count = samples;

	ret = bmc150_accel_fifo_transfer(data, (u8 *)buffer, count);
	if (ret)
		return ret;

	/*
	 * Ideally we want the IIO core to handle the demux when running in fifo
	 * mode but not when running in triggered buffer mode. Unfortunately
	 * this does not seem to be possible, so stick with driver demux for
	 * now.
	 */
	for (i = 0; i < count; i++) {
		int j, bit;

		j = 0;
		for_each_set_bit(bit, indio_dev->active_scan_mask,
				 indio_dev->masklength)
			memcpy(&data->scan.channels[j++], &buffer[i * 3 + bit],
			       sizeof(data->scan.channels[0]));

		iio_push_to_buffers_with_timestamp(indio_dev, &data->scan,
						   tstamp);

		tstamp += sample_period;
	}

	return count;
}

static int bmc150_accel_fifo_flush(struct iio_dev *indio_dev, unsigned samples)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = __bmc150_accel_fifo_flush(indio_dev, samples, false);
	mutex_unlock(&data->mutex);

	return ret;
}

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL(
		"15.620000 31.260000 62.50000 125 250 500 1000 2000");

static struct attribute *bmc150_accel_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group bmc150_accel_attrs_group = {
	.attrs = bmc150_accel_attributes,
};

static const struct iio_event_spec bmc150_accel_event = {
		.type = IIO_EV_TYPE_ROC,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_VALUE) |
				 BIT(IIO_EV_INFO_ENABLE) |
				 BIT(IIO_EV_INFO_PERIOD)
};

#define BMC150_ACCEL_CHANNEL(_axis, bits) {				\
	.type = IIO_ACCEL,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.scan_index = AXIS_##_axis,					\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = (bits),					\
		.storagebits = 16,					\
		.shift = 16 - (bits),					\
		.endianness = IIO_LE,					\
	},								\
	.ext_info = bmc150_accel_ext_info,				\
	.event_spec = &bmc150_accel_event,				\
	.num_event_specs = 1						\
}

#define BMI323_ACCEL_CHANNEL(_axis, bits)                                      \
	{                                                                      \
		.type = IIO_ACCEL, .modified = 1, .channel2 = IIO_MOD_##_axis, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),                  \
		.info_mask_shared_by_type =                                    \
			BIT(IIO_CHAN_INFO_SCALE) |                             \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                         \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),      \
		.info_mask_shared_by_type_available =                          \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                         \
			BIT(IIO_CHAN_INFO_SCALE),                              \
		.scan_index = BMI323_ACCEL_AXIS_##_axis,                       \
		.scan_type = {                                                 \
			.sign = 's',                                           \
			.realbits = (bits),                                    \
			.storagebits = 16,                                     \
			.shift = 16 - (bits),                                  \
			.endianness = IIO_LE,                                  \
		},                                                             \
	}

#define BMI323_GYRO_CHANNEL(_axis, bits)                                  \
	{                                                                 \
		.type = IIO_ANGL_VEL, .modified = 1,                      \
		.channel2 = IIO_MOD_##_axis,                              \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),             \
		.info_mask_shared_by_type =                               \
			BIT(IIO_CHAN_INFO_SCALE) |                        \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                    \
			BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY), \
		.info_mask_shared_by_type_available =                     \
			BIT(IIO_CHAN_INFO_SAMP_FREQ) |                    \
			BIT(IIO_CHAN_INFO_SCALE),                         \
		.scan_index = BMI323_GYRO_AXIS_##_axis,                   \
		.scan_type = {                                            \
			.sign = 's',                                      \
			.realbits = (bits),                               \
			.storagebits = 16,                                \
			.shift = 16 - (bits),                             \
			.endianness = IIO_LE,                             \
		},                                                        \
		/*.ext_info = bmi323_accel_ext_info,*/                    \
		/*.event_spec = &bmi323_accel_event,*/                    \
		/*.num_event_specs = 1*/                                  \
	}

#define BMC150_ACCEL_CHANNELS(bits) {					\
	{								\
		.type = IIO_TEMP,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				      BIT(IIO_CHAN_INFO_SCALE) |	\
				      BIT(IIO_CHAN_INFO_OFFSET),	\
		.scan_index = -1,					\
	},								\
	BMC150_ACCEL_CHANNEL(X, bits),					\
	BMC150_ACCEL_CHANNEL(Y, bits),					\
	BMC150_ACCEL_CHANNEL(Z, bits),					\
	IIO_CHAN_SOFT_TIMESTAMP(3),					\
}

static const struct iio_chan_spec bma222e_accel_channels[] =
	BMC150_ACCEL_CHANNELS(8);
static const struct iio_chan_spec bma250e_accel_channels[] =
	BMC150_ACCEL_CHANNELS(10);
static const struct iio_chan_spec bmc150_accel_channels[] =
	BMC150_ACCEL_CHANNELS(12);
static const struct iio_chan_spec bma280_accel_channels[] =
	BMC150_ACCEL_CHANNELS(14);

/*
 * The range for the Bosch sensors is typically +-2g/4g/8g/16g, distributed
 * over the amount of bits (see above). The scale table can be calculated using
 *     (range / 2^bits) * g = (range / 2^bits) * 9.80665 m/s^2
 * e.g. for +-2g and 12 bits: (4 / 2^12) * 9.80665 m/s^2 = 0.0095768... m/s^2
 * Multiply 10^6 and round to get the values listed below.
 */
static const struct bmc150_accel_chip_info bmc150_accel_chip_info_tbl[] = {
	{
		.name = "BMA222",
		.chip_id = 0x03,
		.channels = bma222e_accel_channels,
		.num_channels = ARRAY_SIZE(bma222e_accel_channels),
		.scale_table = { {153229, BMC150_ACCEL_DEF_RANGE_2G},
				 {306458, BMC150_ACCEL_DEF_RANGE_4G},
				 {612916, BMC150_ACCEL_DEF_RANGE_8G},
				 {1225831, BMC150_ACCEL_DEF_RANGE_16G} },
	},
	{
		.name = "BMA222E",
		.chip_id = 0xF8,
		.channels = bma222e_accel_channels,
		.num_channels = ARRAY_SIZE(bma222e_accel_channels),
		.scale_table = { {153229, BMC150_ACCEL_DEF_RANGE_2G},
				 {306458, BMC150_ACCEL_DEF_RANGE_4G},
				 {612916, BMC150_ACCEL_DEF_RANGE_8G},
				 {1225831, BMC150_ACCEL_DEF_RANGE_16G} },
	},
	{
		.name = "BMA250E",
		.chip_id = 0xF9,
		.channels = bma250e_accel_channels,
		.num_channels = ARRAY_SIZE(bma250e_accel_channels),
		.scale_table = { {38307, BMC150_ACCEL_DEF_RANGE_2G},
				 {76614, BMC150_ACCEL_DEF_RANGE_4G},
				 {153229, BMC150_ACCEL_DEF_RANGE_8G},
				 {306458, BMC150_ACCEL_DEF_RANGE_16G} },
	},
	{
		.name = "BMA253/BMA254/BMA255/BMC150/BMC156/BMI055",
		.chip_id = 0xFA,
		.channels = bmc150_accel_channels,
		.num_channels = ARRAY_SIZE(bmc150_accel_channels),
		.scale_table = { {9577, BMC150_ACCEL_DEF_RANGE_2G},
				 {19154, BMC150_ACCEL_DEF_RANGE_4G},
				 {38307, BMC150_ACCEL_DEF_RANGE_8G},
				 {76614, BMC150_ACCEL_DEF_RANGE_16G} },
	},
	{
		.name = "BMA280",
		.chip_id = 0xFB,
		.channels = bma280_accel_channels,
		.num_channels = ARRAY_SIZE(bma280_accel_channels),
		.scale_table = { {2394, BMC150_ACCEL_DEF_RANGE_2G},
				 {4788, BMC150_ACCEL_DEF_RANGE_4G},
				 {9577, BMC150_ACCEL_DEF_RANGE_8G},
				 {19154, BMC150_ACCEL_DEF_RANGE_16G} },
	},
};

static const struct iio_info bmc150_accel_info = {
	.attrs			= &bmc150_accel_attrs_group,
	.read_raw		= bmc150_accel_read_raw,
	.write_raw		= bmc150_accel_write_raw,
	.read_event_value	= bmc150_accel_read_event,
	.write_event_value	= bmc150_accel_write_event,
	.write_event_config	= bmc150_accel_write_event_config,
	.read_event_config	= bmc150_accel_read_event_config,
};

static const struct iio_info bmc150_accel_info_fifo = {
	.attrs			= &bmc150_accel_attrs_group,
	.read_raw		= bmc150_accel_read_raw,
	.write_raw		= bmc150_accel_write_raw,
	.read_event_value	= bmc150_accel_read_event,
	.write_event_value	= bmc150_accel_write_event,
	.write_event_config	= bmc150_accel_write_event_config,
	.read_event_config	= bmc150_accel_read_event_config,
	.validate_trigger	= bmc150_accel_validate_trigger,
	.hwfifo_set_watermark	= bmc150_accel_set_watermark,
	.hwfifo_flush_to_buffer	= bmc150_accel_fifo_flush,
};

static const unsigned long bmc150_accel_scan_masks[] = {
					BIT(AXIS_X) | BIT(AXIS_Y) | BIT(AXIS_Z),
					0};

static irqreturn_t bmc150_accel_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->mutex);
	ret = regmap_bulk_read(data->regmap, BMC150_ACCEL_REG_XOUT_L,
			       data->buffer, AXIS_MAX * 2);
	mutex_unlock(&data->mutex);
	if (ret < 0)
		goto err_read;

	iio_push_to_buffers_with_timestamp(indio_dev, data->buffer,
					   pf->timestamp);
err_read:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static void bmc150_accel_trig_reen(struct iio_trigger *trig)
{
	struct bmc150_accel_trigger *t = iio_trigger_get_drvdata(trig);
	struct bmc150_accel_data *data = t->data;
	struct device *dev = regmap_get_device(data->regmap);
	int ret;

	/* new data interrupts don't need ack */
	if (t == &t->data->triggers[BMC150_ACCEL_TRIGGER_DATA_READY])
		return;

	mutex_lock(&data->mutex);
	/* clear any latched interrupt */
	ret = regmap_write(data->regmap, BMC150_ACCEL_REG_INT_RST_LATCH,
			   BMC150_ACCEL_INT_MODE_LATCH_INT |
			   BMC150_ACCEL_INT_MODE_LATCH_RESET);
	mutex_unlock(&data->mutex);
	if (ret < 0)
		dev_err(dev, "Error writing reg_int_rst_latch\n");
}

static int bmc150_accel_trigger_set_state(struct iio_trigger *trig,
					  bool state)
{
	struct bmc150_accel_trigger *t = iio_trigger_get_drvdata(trig);
	struct bmc150_accel_data *data = t->data;
	int ret;

	mutex_lock(&data->mutex);

	if (t->enabled == state) {
		mutex_unlock(&data->mutex);
		return 0;
	}

	if (t->setup) {
		ret = t->setup(t, state);
		if (ret < 0) {
			mutex_unlock(&data->mutex);
			return ret;
		}
	}

	ret = bmc150_accel_set_interrupt(data, t->intr, state);
	if (ret < 0) {
		mutex_unlock(&data->mutex);
		return ret;
	}

	t->enabled = state;

	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_trigger_ops bmc150_accel_trigger_ops = {
	.set_trigger_state = bmc150_accel_trigger_set_state,
	.reenable = bmc150_accel_trig_reen,
};

static int bmc150_accel_handle_roc_event(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	int dir;
	int ret;
	unsigned int val;

	ret = regmap_read(data->regmap, BMC150_ACCEL_REG_INT_STATUS_2, &val);
	if (ret < 0) {
		dev_err(dev, "Error reading reg_int_status_2\n");
		return ret;
	}

	if (val & BMC150_ACCEL_ANY_MOTION_BIT_SIGN)
		dir = IIO_EV_DIR_FALLING;
	else
		dir = IIO_EV_DIR_RISING;

	if (val & BMC150_ACCEL_ANY_MOTION_BIT_X)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL,
						  0,
						  IIO_MOD_X,
						  IIO_EV_TYPE_ROC,
						  dir),
			       data->timestamp);

	if (val & BMC150_ACCEL_ANY_MOTION_BIT_Y)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL,
						  0,
						  IIO_MOD_Y,
						  IIO_EV_TYPE_ROC,
						  dir),
			       data->timestamp);

	if (val & BMC150_ACCEL_ANY_MOTION_BIT_Z)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_ACCEL,
						  0,
						  IIO_MOD_Z,
						  IIO_EV_TYPE_ROC,
						  dir),
			       data->timestamp);

	return ret;
}

static irqreturn_t bmc150_accel_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);
	bool ack = false;
	int ret;

	mutex_lock(&data->mutex);

	if (data->fifo_mode) {
		ret = __bmc150_accel_fifo_flush(indio_dev,
						BMC150_ACCEL_FIFO_LENGTH, true);
		if (ret > 0)
			ack = true;
	}

	if (data->ev_enable_state) {
		ret = bmc150_accel_handle_roc_event(indio_dev);
		if (ret > 0)
			ack = true;
	}

	if (ack) {
		ret = regmap_write(data->regmap, BMC150_ACCEL_REG_INT_RST_LATCH,
				   BMC150_ACCEL_INT_MODE_LATCH_INT |
				   BMC150_ACCEL_INT_MODE_LATCH_RESET);
		if (ret)
			dev_err(dev, "Error writing reg_int_rst_latch\n");

		ret = IRQ_HANDLED;
	} else {
		ret = IRQ_NONE;
	}

	mutex_unlock(&data->mutex);

	return ret;
}

static irqreturn_t bmc150_accel_irq_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	bool ack = false;
	int i;

	data->old_timestamp = data->timestamp;
	data->timestamp = iio_get_time_ns(indio_dev);

	for (i = 0; i < BMC150_ACCEL_TRIGGERS; i++) {
		if (data->triggers[i].enabled) {
			iio_trigger_poll(data->triggers[i].indio_trig);
			ack = true;
			break;
		}
	}

	if (data->ev_enable_state || data->fifo_mode)
		return IRQ_WAKE_THREAD;

	if (ack)
		return IRQ_HANDLED;

	return IRQ_NONE;
}

static const struct {
	int intr;
	const char *name;
	int (*setup)(struct bmc150_accel_trigger *t, bool state);
} bmc150_accel_triggers[BMC150_ACCEL_TRIGGERS] = {
	{
		.intr = 0,
		.name = "%s-dev%d",
	},
	{
		.intr = 1,
		.name = "%s-any-motion-dev%d",
		.setup = bmc150_accel_any_motion_setup,
	},
};

static void bmc150_accel_unregister_triggers(struct bmc150_accel_data *data,
					     int from)
{
	int i;

	for (i = from; i >= 0; i--) {
		if (data->triggers[i].indio_trig) {
			iio_trigger_unregister(data->triggers[i].indio_trig);
			data->triggers[i].indio_trig = NULL;
		}
	}
}

static int bmc150_accel_triggers_setup(struct iio_dev *indio_dev,
				       struct bmc150_accel_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int i, ret;

	for (i = 0; i < BMC150_ACCEL_TRIGGERS; i++) {
		struct bmc150_accel_trigger *t = &data->triggers[i];

		t->indio_trig = devm_iio_trigger_alloc(dev,
						       bmc150_accel_triggers[i].name,
						       indio_dev->name,
						       iio_device_id(indio_dev));
		if (!t->indio_trig) {
			ret = -ENOMEM;
			break;
		}

		t->indio_trig->ops = &bmc150_accel_trigger_ops;
		t->intr = bmc150_accel_triggers[i].intr;
		t->data = data;
		t->setup = bmc150_accel_triggers[i].setup;
		iio_trigger_set_drvdata(t->indio_trig, t);

		ret = iio_trigger_register(t->indio_trig);
		if (ret)
			break;
	}

	if (ret)
		bmc150_accel_unregister_triggers(data, i - 1);

	return ret;
}

#define BMC150_ACCEL_FIFO_MODE_STREAM          0x80
#define BMC150_ACCEL_FIFO_MODE_FIFO            0x40
#define BMC150_ACCEL_FIFO_MODE_BYPASS          0x00

static int bmc150_accel_fifo_set_mode(struct bmc150_accel_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	u8 reg = BMC150_ACCEL_REG_FIFO_CONFIG1;
	int ret;

	ret = regmap_write(data->regmap, reg, data->fifo_mode);
	if (ret < 0) {
		dev_err(dev, "Error writing reg_fifo_config1\n");
		return ret;
	}

	if (!data->fifo_mode)
		return 0;

	ret = regmap_write(data->regmap, BMC150_ACCEL_REG_FIFO_CONFIG0,
			   data->watermark);
	if (ret < 0)
		dev_err(dev, "Error writing reg_fifo_config0\n");

	return ret;
}

static int bmc150_accel_buffer_preenable(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	return bmc150_accel_set_power_state(data, true);
}

static int bmc150_accel_buffer_postenable(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret = 0;

	if (iio_device_get_current_mode(indio_dev) == INDIO_BUFFER_TRIGGERED)
		return 0;

	mutex_lock(&data->mutex);

	if (!data->watermark)
		goto out;

	ret = bmc150_accel_set_interrupt(data, BMC150_ACCEL_INT_WATERMARK,
					 true);
	if (ret)
		goto out;

	data->fifo_mode = BMC150_ACCEL_FIFO_MODE_FIFO;

	ret = bmc150_accel_fifo_set_mode(data);
	if (ret) {
		data->fifo_mode = 0;
		bmc150_accel_set_interrupt(data, BMC150_ACCEL_INT_WATERMARK,
					   false);
	}

out:
	mutex_unlock(&data->mutex);

	return ret;
}

static int bmc150_accel_buffer_predisable(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	if (iio_device_get_current_mode(indio_dev) == INDIO_BUFFER_TRIGGERED)
		return 0;

	mutex_lock(&data->mutex);

	if (!data->fifo_mode)
		goto out;

	bmc150_accel_set_interrupt(data, BMC150_ACCEL_INT_WATERMARK, false);
	__bmc150_accel_fifo_flush(indio_dev, BMC150_ACCEL_FIFO_LENGTH, false);
	data->fifo_mode = 0;
	bmc150_accel_fifo_set_mode(data);

out:
	mutex_unlock(&data->mutex);

	return 0;
}

static int bmc150_accel_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	return bmc150_accel_set_power_state(data, false);
}

static const struct iio_buffer_setup_ops bmc150_accel_buffer_ops = {
	.preenable = bmc150_accel_buffer_preenable,
	.postenable = bmc150_accel_buffer_postenable,
	.predisable = bmc150_accel_buffer_predisable,
	.postdisable = bmc150_accel_buffer_postdisable,
};

static int bmc150_accel_chip_init(struct bmc150_accel_data *data)
{
	struct device *dev = regmap_get_device(data->regmap);
	int ret, i;
	unsigned int val;
	
	/*
	 * Reset chip to get it in a known good state. A delay of 1.8ms after
	 * reset is required according to the data sheets of supported chips.
	 */
	regmap_write(data->regmap, BMC150_ACCEL_REG_RESET,
		     BMC150_ACCEL_RESET_VAL);
	usleep_range(1800, 2500);

	ret = regmap_read(data->regmap, BMC150_ACCEL_REG_CHIP_ID, &val);
	if (ret < 0) {
		dev_err(dev, "Error: Reading chip id\n");
		return ret;
	}

	dev_dbg(dev, "Chip Id %x\n", val);
	for (i = 0; i < ARRAY_SIZE(bmc150_accel_chip_info_tbl); i++) {
		if (bmc150_accel_chip_info_tbl[i].chip_id == val) {
			data->chip_info = &bmc150_accel_chip_info_tbl[i];
			break;
		}
	}

	if (!data->chip_info) {
		dev_err(dev, "Invalid chip %x\n", val);
		return -ENODEV;
	}

	ret = bmc150_accel_set_mode(data, BMC150_ACCEL_SLEEP_MODE_NORMAL, 0);
	if (ret < 0)
		return ret;

	/* Set Bandwidth */
	ret = bmc150_accel_set_bw(data, BMC150_ACCEL_DEF_BW, 0);
	if (ret < 0)
		return ret;

	/* Set Default Range */
	ret = regmap_write(data->regmap, BMC150_ACCEL_REG_PMU_RANGE,
			   BMC150_ACCEL_DEF_RANGE_4G);
	if (ret < 0) {
		dev_err(dev, "Error writing reg_pmu_range\n");
		return ret;
	}

	data->range = BMC150_ACCEL_DEF_RANGE_4G;

	/* Set default slope duration and thresholds */
	data->slope_thres = BMC150_ACCEL_DEF_SLOPE_THRESHOLD;
	data->slope_dur = BMC150_ACCEL_DEF_SLOPE_DURATION;
	ret = bmc150_accel_update_slope(data);
	if (ret < 0)
		return ret;

	/* Set default as latched interrupts */
	ret = regmap_write(data->regmap, BMC150_ACCEL_REG_INT_RST_LATCH,
			   BMC150_ACCEL_INT_MODE_LATCH_INT |
			   BMC150_ACCEL_INT_MODE_LATCH_RESET);
	if (ret < 0) {
		dev_err(dev, "Error writing reg_int_rst_latch\n");
		return ret;
	}

	return 0;
}

int bmc150_accel_core_probe(struct device *dev, struct regmap *regmap, int irq,
			    enum bmc150_type type, const char *name,
			    bool block_supported)
{
	const struct iio_dev_attr **fifo_attrs;
	struct bmc150_accel_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	/*
	 * Setting the dev_type here is necessary to avoid having it left uninitialized
	 * and therefore potentially executing bmi323 functions for the original bmc150 model.
	 */
	data->dev_type = BMC150;
	data->regmap = regmap;
	data->type = type;

	if (!bmc150_apply_acpi_orientation(dev, &data->orientation)) {
		ret = iio_read_mount_matrix(dev, &data->orientation);
		if (ret)
			return ret;
	}

	/*
	 * VDD   is the analog and digital domain voltage supply
	 * VDDIO is the digital I/O voltage supply
	 */
	data->regulators[0].supply = "vdd";
	data->regulators[1].supply = "vddio";
	ret = devm_regulator_bulk_get(dev,
				      ARRAY_SIZE(data->regulators),
				      data->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(data->regulators),
				    data->regulators);
	if (ret) {
		dev_err(dev, "failed to enable regulators: %d\n", ret);
		return ret;
	}
	/*
	 * 2ms or 3ms power-on time according to datasheets, let's better
	 * be safe than sorry and set this delay to 5ms.
	 */
	msleep(5);

	ret = bmc150_accel_chip_init(data);
	if (ret < 0)
		goto err_disable_regulators;

	mutex_init(&data->mutex);

	indio_dev->channels = data->chip_info->channels;
	indio_dev->num_channels = data->chip_info->num_channels;
	indio_dev->name = name ? name : data->chip_info->name;
	indio_dev->available_scan_masks = bmc150_accel_scan_masks;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bmc150_accel_info;

	if (block_supported) {
		indio_dev->modes |= INDIO_BUFFER_SOFTWARE;
		indio_dev->info = &bmc150_accel_info_fifo;
		fifo_attrs = bmc150_accel_fifo_attributes;
	} else {
		fifo_attrs = NULL;
	}

	ret = iio_triggered_buffer_setup_ext(indio_dev,
					     &iio_pollfunc_store_time,
					     bmc150_accel_trigger_handler,
					     IIO_BUFFER_DIRECTION_IN,
					     &bmc150_accel_buffer_ops,
					     fifo_attrs);
	if (ret < 0) {
		dev_err(dev, "Failed: iio triggered buffer setup\n");
		goto err_disable_regulators;
	}

	if (irq > 0) {
		ret = devm_request_threaded_irq(dev, irq,
						bmc150_accel_irq_handler,
						bmc150_accel_irq_thread_handler,
						IRQF_TRIGGER_RISING,
						BMC150_ACCEL_IRQ_NAME,
						indio_dev);
		if (ret)
			goto err_buffer_cleanup;

		/*
		 * Set latched mode interrupt. While certain interrupts are
		 * non-latched regardless of this settings (e.g. new data) we
		 * want to use latch mode when we can to prevent interrupt
		 * flooding.
		 */
		ret = regmap_write(data->regmap, BMC150_ACCEL_REG_INT_RST_LATCH,
				   BMC150_ACCEL_INT_MODE_LATCH_RESET);
		if (ret < 0) {
			dev_err(dev, "Error writing reg_int_rst_latch\n");
			goto err_buffer_cleanup;
		}

		bmc150_accel_interrupts_setup(indio_dev, data, irq);

		ret = bmc150_accel_triggers_setup(indio_dev, data);
		if (ret)
			goto err_buffer_cleanup;
	}

	ret = pm_runtime_set_active(dev);
	if (ret)
		goto err_trigger_unregister;

	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, BMC150_AUTO_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(dev, "Unable to register iio device\n");
		goto err_pm_cleanup;
	}

	return 0;

err_pm_cleanup:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
err_trigger_unregister:
	bmc150_accel_unregister_triggers(data, BMC150_ACCEL_TRIGGERS - 1);
err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
err_disable_regulators:
	regulator_bulk_disable(ARRAY_SIZE(data->regulators),
			       data->regulators);

	return ret;
}
EXPORT_SYMBOL_NS_GPL(bmc150_accel_core_probe, IIO_BMC150);

void bmc150_accel_core_remove(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);

	bmc150_accel_unregister_triggers(data, BMC150_ACCEL_TRIGGERS - 1);

	iio_triggered_buffer_cleanup(indio_dev);

	mutex_lock(&data->mutex);
	bmc150_accel_set_mode(data, BMC150_ACCEL_SLEEP_MODE_DEEP_SUSPEND, 0);
	mutex_unlock(&data->mutex);

	regulator_bulk_disable(ARRAY_SIZE(data->regulators),
			       data->regulators);
}
EXPORT_SYMBOL_NS_GPL(bmc150_accel_core_remove, IIO_BMC150);

struct device *bmi323_get_managed_device(struct bmi323_private_data *bmi323)
{
	if (bmi323->i2c_client != NULL)
		return &bmi323->i2c_client->dev;

	return &bmi323->spi_client->dev;
}

static int bmi323_set_power_state(struct bmi323_private_data *bmi323, bool on)
{
#ifdef CONFIG_PM
	struct device *dev = bmi323_get_managed_device(bmi323);
	int ret;

	if (on)
		ret = pm_runtime_get_sync(dev);
	else {
		pm_runtime_mark_last_busy(dev);
		ret = pm_runtime_put_autosuspend(dev);
	}

	if (ret < 0) {
		dev_err(dev, "bmi323_set_power_state failed with %d\n", on);

		if (on)
			pm_runtime_put_noidle(dev);

		return ret;
	}
#endif

	return 0;
}

int bmi323_write_u16(struct bmi323_private_data *bmi323, u8 in_reg,
		     u16 in_value)
{
	s32 ret;

	if (bmi323->i2c_client != NULL) {
		ret = i2c_smbus_write_i2c_block_data(bmi323->i2c_client, in_reg,
						     sizeof(in_value),
						     (u8 *)(&in_value));
		if (ret != 0) {
			return -2;
		}

		return 0;
	} else if (bmi323->spi_client != NULL) {
		/*
		 * To whoever may need this: implementing this should be straightforward:
		 * it's specular to the i2c part.
		 */

		return -EINVAL; // TODO: change with 0 once implemented
	}

	return -EINVAL;
}
EXPORT_SYMBOL_NS_GPL(bmi323_write_u16, IIO_BMC150);

int bmi323_read_u16(struct bmi323_private_data *bmi323, u8 in_reg,
		    u16 *out_value)
{
	s32 ret;
	u8 read_bytes[4];

	if (bmi323->i2c_client != NULL) {
		ret = i2c_smbus_read_i2c_block_data(bmi323->i2c_client, in_reg,
						    sizeof(read_bytes),
						    &read_bytes[0]);
		if (ret != 4) {
			return ret;
		}

		// DUMMY	= read_bytes[0]
		// DUMMY	= read_bytes[1]
		// LSB		= read_bytes[2]
		// MSB		= read_bytes[3]
		u8 *o = (u8 *)out_value;
		o[0] = read_bytes[2];
		o[1] = read_bytes[3];

		return 0;
	} else if (bmi323->spi_client != NULL) {
		printk(KERN_CRIT
		       "bmi323: SPI interface is not yet implemented.\n");

		/*
		 * To whoever may need this: implementing this should be straightforward:
		 * it's specular to the i2c part except that the dummy data is just 1 byte.
		 */

		return -EINVAL; // TODO: change with 0 once implemented
	}

	return -EINVAL;
}
EXPORT_SYMBOL_NS_GPL(bmi323_read_u16, IIO_BMC150);

int bmi323_chip_check(struct bmi323_private_data *bmi323)
{
	u16 chip_id;
	int ret;

	ret = bmi323_read_u16(bmi323, BMC150_BMI323_CHIP_ID_REG, &chip_id);
	if (ret != 0) {
		return ret;
	}

	if (((chip_id)&0x00FF) != cpu_to_le16((u16)0x0043U)) {
		dev_err(bmi323->dev,
			"bmi323_chip_check failed with: %d; chip_id = 0x%04x",
			ret, chip_id);

		return -EINVAL;
	}

	return 0;
}
EXPORT_SYMBOL_NS_GPL(bmi323_chip_check, IIO_BMC150);

static int bmi323_buffer_preenable(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	const int ret = bmi323_set_power_state(&data->bmi323, true);

	if (ret == 0) {
		mutex_lock(&data->bmi323.mutex);
		data->bmi323.fifo_frame_time_diff_ns =
			(data->bmi323.acc_odr_time_ns >=
			 data->bmi323.gyr_odr_time_ns) ?
				data->bmi323.acc_odr_time_ns :
				data->bmi323.gyr_odr_time_ns;
		mutex_unlock(&data->bmi323.mutex);
	}

	return ret;
}

static int bmi323_buffer_postenable(struct iio_dev *indio_dev)
{
	//struct bmc150_accel_data *data = iio_priv(indio_dev);

	/*
	 * This code is a placeholder until I can get a way to test it
	 */

	return 0;
}

static int bmi323_buffer_predisable(struct iio_dev *indio_dev)
{
	//struct bmc150_accel_data *data = iio_priv(indio_dev);

	/*
	 * This code is a placeholder until I can get a way to test it
	 */

	return 0;
}

static int bmi323_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	return bmi323_set_power_state(&data->bmi323, true);
}

static const struct iio_buffer_setup_ops bmi323_buffer_ops = {
	.preenable = bmi323_buffer_preenable,
	.postenable = bmi323_buffer_postenable,
	.predisable = bmi323_buffer_predisable,
	.postdisable = bmi323_buffer_postdisable,
};

int bmi323_chip_rst(struct bmi323_private_data *bmi323)
{
	u16 sensor_status = 0x0000, device_status = 0x0000;
	int ret;

	ret = bmi323_write_u16(bmi323, BMC150_BMI323_SOFT_RESET_REG,
			       cpu_to_le16((u16)BMC150_BMI323_SOFT_RESET_VAL));
	if (ret != 0) {
		dev_err(bmi323->dev,
			"bmi323: error while issuing the soft-reset command: %d",
			ret);
		return ret;
	}

	/* wait the specified amount of time... I agree with the bmc150 module: better safe than sorry. */
	msleep(5);

	// if the device is connected over SPI a dummy read is to be performed once after each reset
	if (bmi323->spi_client != NULL) {
		dev_info(bmi323->dev,
			"issuing the dummy read to switch mode to SPI");

		// do not even check the result of that... it's just a dummy read
		bmi323_chip_check(bmi323);
	}

	ret = bmi323_chip_check(bmi323);
	if (ret != 0) {
		return ret;
	}

	/* now check the correct initialization status as per datasheet */
	ret = bmi323_read_u16(bmi323, 0x01, &device_status);
	if (ret != 0) {
		return -EINVAL;
	}

	if ((device_status & cpu_to_le16((u16)0x00FFU)) !=
	    cpu_to_le16((u16)0x0000U)) {
		dev_err(bmi323->dev,
			"bmi323: device_status incorrect: %d; device_status = 0x%04x",
			ret, device_status);

		/* from the datasheet: power error */
		return -EINVAL;
	}

	/* from the datasheet: power ok */
	ret = bmi323_read_u16(bmi323, 0x02, &sensor_status);
	if (ret != 0) {
		return -EINVAL;
	}

	if ((sensor_status & cpu_to_le16((u16)0x00FFU)) !=
	    cpu_to_le16((u16)0x0001U)) {
		dev_err(bmi323->dev,
			"bmi323: sensor_status incorrect: %d; sensor_status = 0x%04x",
			ret, sensor_status);

		/* from the datasheet: initialization error */
		return -EINVAL;
	}

	/* from the datasheet: initialization ok */
	return 0;
}
EXPORT_SYMBOL_NS_GPL(bmi323_chip_rst, IIO_BMC150);

static const struct iio_chan_spec bmi323_channels[] = {
	BMI323_ACCEL_CHANNEL(X, 16),
	BMI323_ACCEL_CHANNEL(Y, 16),
	BMI323_ACCEL_CHANNEL(Z, 16),
	BMI323_GYRO_CHANNEL(X, 16),
	BMI323_GYRO_CHANNEL(Y, 16),
	BMI323_GYRO_CHANNEL(Z, 16),
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = BMI323_TEMP,
	},
	IIO_CHAN_SOFT_TIMESTAMP(BMI323_AXIS_MAX),
};

static int bmi323_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret = -EINVAL, was_sleep_modified = -1;
	u16 raw_read = 0x8000;

	mutex_lock(&data->bmi323.mutex);

	if ((data->bmi323.flags & BMI323_FLAGS_RESET_FAILED) != 0x00U) {
		dev_err(data->bmi323.dev,
			"bmi323 error: device has not being woken up correctly.");
		mutex_unlock(&data->bmi323.mutex);
		return -EBUSY;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW: {
		switch (chan->type) {
		case IIO_TEMP:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_read_raw_error;
			}

			was_sleep_modified =
				bmi323_set_power_state(&data->bmi323, true);
			if (was_sleep_modified != 0) {
				ret = was_sleep_modified;
				goto bmi323_read_raw_error_power;
			}

			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret != 0) {
				printk(KERN_CRIT
				       "bmc150 bmi323_read_raw IIO_TEMP iio_device_claim_direct_mode returned %d\n",
				       ret);
				goto bmi323_read_raw_error;
			}

			ret = bmi323_read_u16(
				&data->bmi323,
				BMC150_BMI323_TEMPERATURE_DATA_REG, &raw_read);
			iio_device_release_direct_mode(indio_dev);
			if (ret != 0) {
				printk(KERN_CRIT
				       "bmc150 bmi323_read_raw IIO_TEMP bmi323_read_u16 returned %d\n",
				       ret);
				goto bmi323_read_raw_error;
			}

			*val = sign_extend32(le16_to_cpu(raw_read), 15);
			bmi323_set_power_state(&data->bmi323, false);
			mutex_unlock(&data->bmi323.mutex);
			return IIO_VAL_INT;

		case IIO_ACCEL:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_read_raw_error;
			}

			was_sleep_modified =
				bmi323_set_power_state(&data->bmi323, true);
			if (was_sleep_modified != 0) {
				ret = was_sleep_modified;
				goto bmi323_read_raw_error_power;
			}

			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret != 0) {
				printk(KERN_CRIT
				       "bmc150 bmi323_read_raw IIO_ACCEL iio_device_claim_direct_mode returned %d\n",
				       ret);
				goto bmi323_read_raw_error;
			}

			ret = bmi323_read_u16(&data->bmi323,
					      BMC150_BMI323_DATA_BASE_REG +
						      (u8)(chan->scan_index),
					      &raw_read);
			iio_device_release_direct_mode(indio_dev);
			if (ret != 0) {
				printk(KERN_CRIT
				       "bmc150 bmi323_read_raw IIO_ACCEL bmi323_read_u16 returned %d\n",
				       ret);
				goto bmi323_read_raw_error;
			}
			*val = sign_extend32(le16_to_cpu(raw_read), 15);
			bmi323_set_power_state(&data->bmi323, false);
			mutex_unlock(&data->bmi323.mutex);
			return IIO_VAL_INT;

		case IIO_ANGL_VEL:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_read_raw_error;
			}

			was_sleep_modified =
				bmi323_set_power_state(&data->bmi323, true);
			if (was_sleep_modified != 0) {
				ret = was_sleep_modified;
				goto bmi323_read_raw_error_power;
			}

			ret = iio_device_claim_direct_mode(indio_dev);
			if (ret != 0) {
				printk(KERN_CRIT
				       "bmc150 bmi323_read_raw IIO_ANGL_VEL iio_device_claim_direct_mode returned %d\n",
				       ret);
				goto bmi323_read_raw_error;
			}

			ret = bmi323_read_u16(&data->bmi323,
					      BMC150_BMI323_DATA_BASE_REG +
						      (u8)(chan->scan_index),
					      &raw_read);
			iio_device_release_direct_mode(indio_dev);
			if (ret != 0) {
				printk(KERN_CRIT
				       "bmc150 bmi323_read_raw IIO_ANGL_VEL bmi323_read_u16 returned %d\n",
				       ret);
				goto bmi323_read_raw_error;
			}

			*val = sign_extend32(le16_to_cpu(raw_read), 15);
			bmi323_set_power_state(&data->bmi323, false);
			mutex_unlock(&data->bmi323.mutex);
			return IIO_VAL_INT;

		default:
			goto bmi323_read_raw_error;
		}
	}
	case IIO_CHAN_INFO_OFFSET: {
		switch (chan->type) {
		case IIO_TEMP:
			*val = BMC150_BMI323_TEMPER_CENTER_VAL;
			*val2 = 0;
			mutex_unlock(&data->bmi323.mutex);
			return IIO_VAL_INT;

		default:
			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
	}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_TEMP: {
			*val = 0;
			*val2 = BMC150_BMI323_TEMPER_LSB_PER_KELVIN_VAL;
			mutex_unlock(&data->bmi323.mutex);
			return IIO_VAL_FRACTIONAL;
		}
		case IIO_ACCEL: {
			u8 *le_raw_read =
				(u8 *)&data->bmi323.acc_conf_reg_value;
			for (int s = 0; s < ARRAY_SIZE(bmi323_accel_scale_map);
			     ++s) {
				if (((le_raw_read[0]) & ((u16)0b01110000U)) ==
				    (bmi323_accel_scale_map[s].hw_val)) {
					*val = bmi323_accel_scale_map[s].val;
					*val2 = bmi323_accel_scale_map[s].val2;

					mutex_unlock(&data->bmi323.mutex);
					return bmi323_accel_scale_map[s]
						.ret_type;
				}
			}

			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		case IIO_ANGL_VEL: {
			u8 *le_raw_read =
				(u8 *)&data->bmi323.gyr_conf_reg_value;
			for (int s = 0; s < ARRAY_SIZE(bmi323_gyro_scale_map);
			     ++s) {
				if (((le_raw_read[0]) & ((u16)0b01110000U)) ==
				    (bmi323_gyro_scale_map[s].hw_val)) {
					*val = bmi323_gyro_scale_map[s].val;
					*val2 = bmi323_gyro_scale_map[s].val2;

					mutex_unlock(&data->bmi323.mutex);
					return bmi323_gyro_scale_map[s].ret_type;
				}
			}

			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		default:
			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		switch (chan->type) {
		case IIO_ACCEL: {
			u8 *le_raw_read =
				(u8 *)&data->bmi323.acc_conf_reg_value;
			for (int s = 0; s < ARRAY_SIZE(bmi323_accel_odr_map);
			     ++s) {
				if (((le_raw_read[0]) & ((u16)0x0FU)) ==
				    (bmi323_accel_odr_map[s].hw_val)) {
					/*
							 * from tha datasheed: -3dB cut-off frequency can be configured with the bit 7 of GYR_confm,
							 * also called acc_bw that can either be 0 or 1, where 1 means odr/4 and 0 means odr/2
							 */
					int freq_adj_idx =
						(((le_raw_read[0]) &
						  ((u8)0x80U)) == (u8)0x00U) ?
							(s * 2) + 0 :
							(s * 2) + 1;
					*val = bmi323_accel_3db_freq_cutoff
						       [freq_adj_idx]
							       .val;
					*val2 = bmi323_accel_3db_freq_cutoff
							[freq_adj_idx]
								.val2;

					mutex_unlock(&data->bmi323.mutex);
					return IIO_VAL_INT_PLUS_MICRO;
				}
			}

			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		case IIO_ANGL_VEL: {
			u8 *le_raw_read =
				(u8 *)&data->bmi323.gyr_conf_reg_value;
			for (int s = 0; s < ARRAY_SIZE(bmi323_gyro_odr_map);
			     ++s) {
				if (((le_raw_read[0]) & ((u16)0x0FU)) ==
				    (bmi323_gyro_odr_map[s].hw_val)) {
					/*
							 * from tha datasheed: -3dB cut-off frequency can be configured with the bit 7 of GYR_confm,
							 * also called acc_bw that can either be 0 or 1, where 1 means odr/4 and 0 means odr/2
							 */
					int freq_adj_idx =
						(((le_raw_read[0]) &
						  ((u8)0x80U)) == (u8)0x0000U) ?
							(s * 2) + 0 :
							(s * 2) + 1;
					*val = bmi323_gyro_3db_freq_cutoff
						       [freq_adj_idx]
							       .val;
					*val2 = bmi323_gyro_3db_freq_cutoff
							[freq_adj_idx]
								.val2;

					mutex_unlock(&data->bmi323.mutex);
					return bmi323_gyro_3db_freq_cutoff
						[freq_adj_idx]
							.ret_type;
				}
			}

			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		default: {
			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->type) {
		case IIO_TEMP: {

			// while in normal or power mode the temperature sensur has a 50Hz sampling frequency
			*val = 50;
			*val2 = 0;

			mutex_unlock(&data->bmi323.mutex);
			return IIO_VAL_INT_PLUS_MICRO;
		}
		case IIO_ACCEL: {
			u8 *le_raw_read =
				(u8 *)&data->bmi323.acc_conf_reg_value;
			for (int s = 0; s < ARRAY_SIZE(bmi323_accel_odr_map);
			     ++s) {
				if (((le_raw_read[0]) & ((u16)0x0FU)) ==
				    (bmi323_accel_odr_map[s].hw_val)) {
					*val = bmi323_accel_odr_map[s].val;
					*val2 = bmi323_accel_odr_map[s].val2;

					mutex_unlock(&data->bmi323.mutex);
					return IIO_VAL_INT_PLUS_MICRO;
				}
			}

			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		case IIO_ANGL_VEL: {
			u8 *le_raw_read =
				(u8 *)&data->bmi323.gyr_conf_reg_value;
			for (int s = 0; s < ARRAY_SIZE(bmi323_gyro_odr_map);
			     ++s) {
				if (((le_raw_read[0]) & ((u16)0x0FU)) ==
				    (bmi323_gyro_odr_map[s].hw_val)) {
					*val = bmi323_gyro_odr_map[s].val;
					*val2 = bmi323_gyro_odr_map[s].val2;

					mutex_unlock(&data->bmi323.mutex);
					return IIO_VAL_INT_PLUS_MICRO;
				}
			}

			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
		default:
			ret = -EINVAL;
			goto bmi323_read_raw_error;
		}
	default:
		ret = -EINVAL;
		goto bmi323_read_raw_error;
	}

bmi323_read_raw_error:
	if (was_sleep_modified == 0) {
		bmi323_set_power_state(&data->bmi323, false);
	}

bmi323_read_raw_error_power:
	mutex_unlock(&data->bmi323.mutex);
	return ret;
}

static int bmi323_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret = -EINVAL, was_sleep_modified = -1;

	mutex_lock(&data->bmi323.mutex);

	if ((data->bmi323.flags & BMI323_FLAGS_RESET_FAILED) != 0x00U) {
		dev_err(data->bmi323.dev,
			"bmi323 error: device has not being woken up correctly.");
		mutex_unlock(&data->bmi323.mutex);
		return -EBUSY;
	}

	switch (mask) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		switch (chan->type) {
		default: {
			ret = -EINVAL;
			goto bmi323_write_raw_error;
		}
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->type) {
		case IIO_ACCEL:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_write_raw_error;
			}

			for (int s = 0; s < ARRAY_SIZE(bmi323_accel_odr_map);
			     ++s) {
				if ((bmi323_accel_odr_map[s].val == val) &&
				    (bmi323_accel_odr_map[s].val2 == val2)) {
					const u16 conf_backup =
						data->bmi323.acc_conf_reg_value;
					u8 *le_raw_read =
						(u8 *)&data->bmi323
							.acc_conf_reg_value;
					le_raw_read[0] &= (u8)0b11110000U;
					le_raw_read[0] |=
						((u8)bmi323_gyro_odr_map[s]
							 .hw_val);

					was_sleep_modified =
						bmi323_set_power_state(
							&data->bmi323, true);
					if (was_sleep_modified != 0) {
						ret = was_sleep_modified;
						data->bmi323.acc_conf_reg_value =
							conf_backup;
						goto bmi323_write_raw_error_power;
					}

					ret = bmi323_write_u16(
						&data->bmi323,
						BMC150_BMI323_ACC_CONF_REG,
						data->bmi323.acc_conf_reg_value);
					if (ret != 0) {
						data->bmi323.acc_conf_reg_value =
							conf_backup;
						goto bmi323_write_raw_error;
					}

					data->bmi323.acc_odr_time_ns =
						bmi323_accel_odr_map[s].time_ns;
					bmi323_set_power_state(&data->bmi323,
							       false);
					mutex_unlock(&data->bmi323.mutex);
					return 0;
				}
			}

			ret = -EINVAL;
			goto bmi323_write_raw_error;
		case IIO_ANGL_VEL:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_write_raw_error;
			}

			for (int s = 0; s < ARRAY_SIZE(bmi323_gyro_odr_map);
			     ++s) {
				if ((bmi323_gyro_odr_map[s].val == val) &&
				    (bmi323_gyro_odr_map[s].val2 == val2)) {
					const u16 conf_backup =
						data->bmi323.gyr_conf_reg_value;
					u8 *le_raw_read =
						(u8 *)&data->bmi323
							.gyr_conf_reg_value;
					le_raw_read[0] &= (u8)0b11110000U;
					le_raw_read[0] |=
						((u8)bmi323_gyro_odr_map[s]
							 .hw_val);

					was_sleep_modified =
						bmi323_set_power_state(
							&data->bmi323, true);
					if (was_sleep_modified != 0) {
						ret = was_sleep_modified;
						data->bmi323.gyr_conf_reg_value =
							conf_backup;
						goto bmi323_write_raw_error_power;
					}

					ret = bmi323_write_u16(
						&data->bmi323,
						BMC150_BMI323_GYR_CONF_REG,
						data->bmi323.gyr_conf_reg_value);
					if (ret != 0) {
						data->bmi323.gyr_conf_reg_value =
							conf_backup;
						goto bmi323_write_raw_error;
					}

					data->bmi323.gyr_odr_time_ns =
						bmi323_gyro_odr_map[s].time_ns;
					bmi323_set_power_state(&data->bmi323,
							       false);
					mutex_unlock(&data->bmi323.mutex);
					return 0;
				}
			}

			ret = -EINVAL;
			goto bmi323_write_raw_error;

		/* Termometer also ends up here: its sampling frequency depends on the chip configuration and cannot be changed */
		default:
			ret = -EINVAL;
			goto bmi323_write_raw_error;
		}

		break;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_write_raw_error;
			}

			for (int s = 0; s < ARRAY_SIZE(bmi323_accel_scale_map);
			     ++s) {
				if ((bmi323_accel_scale_map[s].val == val) &&
				    (bmi323_accel_scale_map[s].val2 == val2)) {
					u8 *le_raw_read =
						(u8 *)&data->bmi323
							.acc_conf_reg_value;
					le_raw_read[0] &= (u8)0b10001111U;
					le_raw_read[0] |=
						((u8)bmi323_accel_scale_map[s]
							 .hw_val);

					was_sleep_modified =
						bmi323_set_power_state(
							&data->bmi323, true);
					if (was_sleep_modified != 0) {
						ret = was_sleep_modified;
						goto bmi323_write_raw_error_power;
					}

					ret = bmi323_write_u16(
						&data->bmi323,
						BMC150_BMI323_ACC_CONF_REG,
						data->bmi323.acc_conf_reg_value);
					if (ret != 0) {
						goto bmi323_write_raw_error;
					}

					bmi323_set_power_state(&data->bmi323,
							       false);
					mutex_unlock(&data->bmi323.mutex);
					return 0;
				}
			}

			dev_warn(
				data->bmi323.dev,
				"bmi323 error: accel scale val=%d,val2=%d unavailable: ignoring.",
				val, val2);

			ret = -EINVAL;
			goto bmi323_write_raw_error;
		case IIO_ANGL_VEL:
			if (iio_buffer_enabled(indio_dev)) {
				ret = -EBUSY;
				goto bmi323_write_raw_error;
			}

			for (int s = 0; s < ARRAY_SIZE(bmi323_gyro_scale_map);
			     ++s) {
				if ((bmi323_gyro_scale_map[s].val == val) &&
				    (bmi323_gyro_scale_map[s].val2 == val2)) {
					u8 *le_raw_read =
						(u8 *)&data->bmi323
							.gyr_conf_reg_value;
					le_raw_read[0] &= (u8)0b10001111U;
					le_raw_read[0] |=
						((u8)bmi323_gyro_scale_map[s]
							 .hw_val);

					was_sleep_modified =
						bmi323_set_power_state(
							&data->bmi323, true);
					if (was_sleep_modified != 0) {
						ret = was_sleep_modified;
						goto bmi323_write_raw_error_power;
					}

					ret = bmi323_write_u16(
						&data->bmi323,
						BMC150_BMI323_GYR_CONF_REG,
						data->bmi323.acc_conf_reg_value);
					if (ret != 0) {
						goto bmi323_write_raw_error;
					}

					bmi323_set_power_state(&data->bmi323,
							       false);
					mutex_unlock(&data->bmi323.mutex);
					return 0;
				}
			}

			dev_warn(
				data->bmi323.dev,
				"bmi323 error: gyro scale val=%d,val2=%d unavailable: ignoring.",
				val, val2);

			ret = -EINVAL;
			goto bmi323_write_raw_error;

		default:
			ret = -EINVAL;
			goto bmi323_write_raw_error;
		}

	default:
		ret = -EINVAL;
		goto bmi323_write_raw_error;
	}

bmi323_write_raw_error:
	if (was_sleep_modified == 0) {
		bmi323_set_power_state(&data->bmi323, false);
	}

bmi323_write_raw_error_power:
	mutex_unlock(&data->bmi323.mutex);
	return ret;
}

static int bmi323_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ACCEL:
			*type = IIO_VAL_INT_PLUS_MICRO;
			*vals = bmi323_accel_scales;
			*length = ARRAY_SIZE(bmi323_accel_scales);
			return IIO_AVAIL_LIST;
		case IIO_ANGL_VEL:
			*type = IIO_VAL_INT_PLUS_NANO;
			*vals = bmi323_gyro_scales;
			*length = ARRAY_SIZE(bmi323_gyro_scales);
			return IIO_AVAIL_LIST;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT_PLUS_MICRO;
		*vals = bmi323_sample_freqs;
		*length = ARRAY_SIZE(bmi323_sample_freqs);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const struct iio_info bmi323_accel_info = {
	.read_raw = bmi323_read_raw,
	.write_raw = bmi323_write_raw,
	.read_avail = bmi323_read_avail,
	//.hwfifo_flush_to_buffer	= bmi323_fifo_flush,
};

static int bmi323_fifo_flush(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	ret = bmi323_write_u16(&data->bmi323, 0x37, cpu_to_le16(0x01));

	return ret;
}

static const u16 stub_value = 0x8000;

#define ADVANCE_AT_REQ_OR_AVAIL(req, avail, dst, dst_offset, src, src_offset) \
	if (req) {                                                            \
		if (gyr_avail) {                                              \
			memcpy((void *)(dst + dst_offset),                    \
			       (const void *)(src + src_offset), 2);          \
			src_offset += 2;                                      \
		} else {                                                      \
			memcpy((void *)(dst + dst_offset),                    \
			       (const void *)((const u8 *)(&stub_value)), 2); \
		}                                                             \
		dst_offset += 2;                                              \
	} else {                                                              \
		if (avail) {                                                  \
			src_offset += 2;                                      \
		}                                                             \
	}

static irqreturn_t iio_bmi323_trigger_h(int irq, void *p)
{
	printk(KERN_WARNING "bmi323 executed iio_bmi323_trigger_h");

	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct bmc150_accel_data *indio_data = iio_priv(indio_dev);

	mutex_lock(&indio_data->bmi323.mutex);

	const bool temp_avail = ((indio_data->bmi323.fifo_conf_reg_value &
				  (cpu_to_le16(0b0000100000000000))) != 0);
	const bool gyr_avail = ((indio_data->bmi323.fifo_conf_reg_value &
				 (cpu_to_le16(0b0000010000000000))) != 0);
	const bool acc_avail = ((indio_data->bmi323.fifo_conf_reg_value &
				 (cpu_to_le16(0b0000001000000000))) != 0);
	const bool time_avail = ((indio_data->bmi323.fifo_conf_reg_value &
				  (cpu_to_le16(0b0000000100000000))) != 0);

	/* Calculate the number of bytes for a frame */
	const u16 frames_aggregate_size_in_words =
		/* 2 * */ ((temp_avail ? 1 : 0) + (gyr_avail ? 3 : 0) +
			   (acc_avail ? 3 : 0) + (time_avail ? 1 : 0));

	u16 available_words = 0;
	const int available_words_read_res = bmi323_read_u16(
		&indio_data->bmi323, BMC150_BMI323_FIFO_FILL_LEVEL_REG,
		&available_words);
	if (available_words_read_res != 0) {
		goto bmi323_irq_done;
	}

	const u16 available_frame_aggregates = (le16_to_cpu(available_words)) /
					       (frames_aggregate_size_in_words);

	const s64 current_timestamp_ns = iio_get_time_ns(indio_dev);
	const s64 fifo_frame_time_ns =
		indio_data->bmi323.fifo_frame_time_diff_ns;
	const s64 first_sample_timestamp_ns =
		current_timestamp_ns -
		(fifo_frame_time_ns * (s64)(available_frame_aggregates));

	/* This can hold one full block */
	u8 temp_data[16];

	/* This is fifo data as read from the sensor */
	u8 fifo_data[32];

	/*
	| CHANNEL		|	scan_index
	|============================
	|				|			|
	| ACCEL_X		|		0	|
	| ACCEL_Y		|		1	|
	| ACCEL_Y		|		2	|
	| GYRO_X		|		3	|
	| GYRO_Y		|		4	|
	| GYRO_Z		|		5	|
	| TEMP			|		6	|
	| TIMESTAMP		|		?	|
	*/
	bool accel_x_requested = false;
	bool accel_y_requested = false;
	bool accel_z_requested = false;
	bool gyro_x_requested = false;
	bool gyro_y_requested = false;
	bool gyro_z_requested = false;
	bool temp_requested = false;

	int j = 0;
	for_each_set_bit(j, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		switch (j) {
		case 0:
			accel_x_requested = true;
			break;
		case 1:
			accel_y_requested = true;
			break;
		case 2:
			accel_z_requested = true;
			break;
		case 3:
			gyro_x_requested = true;
			break;
		case 4:
			gyro_y_requested = true;
			break;
		case 5:
			gyro_z_requested = true;
			break;
		case 6:
			temp_requested = true;
			break;
		default:
			break;
		}
	}

	u16 current_fifo_buffer_offset_bytes = 0;
	for (u16 f = 0; f < available_frame_aggregates; ++f) {
		u16 current_sample_buffer_offset = 0;

		/* Read data from the raw device */
		if (indio_data->bmi323.i2c_client != NULL) {
			const int bytes_to_read =
				2 + (2 * frames_aggregate_size_in_words);
			int read_block_ret = i2c_smbus_read_i2c_block_data(
				indio_data->bmi323.i2c_client,
				BMC150_BMI323_FIFO_DATA_REG, bytes_to_read,
				&fifo_data[0]);
			if (read_block_ret < bytes_to_read) {
				dev_warn(
					&indio_data->bmi323.i2c_client->dev,
					"bmi323: i2c_smbus_read_i2c_block_data wrong return: expected %d bytes, %d arrived. Doing what is possible with recovered data.\n",
					bytes_to_read, read_block_ret);

				/* at this point FIFO buffer must be flushed to avoid interpreting data incorrectly the next trigger */
				const int flush_res =
					bmi323_fifo_flush(indio_dev);
				if (flush_res != 0) {
					dev_err(&indio_data->bmi323.i2c_client
							 ->dev,
						"bmi323: Could not flush FIFO (%d). Following buffer data might be corrupted.\n",
						flush_res);
				}

				goto bmi323_irq_done;
			}

			/* Discard 2-bytes dummy data from I2C */
			current_fifo_buffer_offset_bytes = 2;
		} else if (indio_data->bmi323.spi_client != NULL) {
			printk(KERN_CRIT
			       "bmi323: SPI interface is not yet implemented.\n");

			/*
			* To whoever may need this: implementing this should be straightforward:
			* it's specular to the i2c part.
			*/

			/* Discard 1-byte dummy data from SPI */
			current_fifo_buffer_offset_bytes = 1;

			goto bmi323_irq_done;
		}

		ADVANCE_AT_REQ_OR_AVAIL(accel_x_requested, acc_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);
		ADVANCE_AT_REQ_OR_AVAIL(accel_y_requested, acc_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);
		ADVANCE_AT_REQ_OR_AVAIL(accel_z_requested, acc_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);
		ADVANCE_AT_REQ_OR_AVAIL(gyro_x_requested, gyr_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);
		ADVANCE_AT_REQ_OR_AVAIL(gyro_y_requested, gyr_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);
		ADVANCE_AT_REQ_OR_AVAIL(gyro_z_requested, gyr_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);
		ADVANCE_AT_REQ_OR_AVAIL(temp_requested, temp_avail,
					(u8 *)&temp_data[0],
					current_sample_buffer_offset,
					(u8 *)&fifo_data[0],
					current_fifo_buffer_offset_bytes);

#ifdef BMC150_BMI232_DEBUG_EN
		/* The following is code only used for debugging */
		u16 timestamp = 0;
		if (time_avail) {
			memcpy((u8 *)&timestamp,
			       (const u8
					*)(&fifo_data
						   [current_fifo_buffer_offset_bytes]),
			       2);
			current_fifo_buffer_offset_bytes += 2;
		}

		u16 *debg = (u16 *)&temp_data[0];
		if (!time_avail) {
			printk(KERN_WARNING
			       "bmi323 pushing to buffer %d/%d -- accel: %d %d %d gyro: %d %d %d",
			       (int)(f + 1), (int)available_frame_aggregates,
			       (int)(*((s16 *)&debg[0])),
			       (int)(*((s16 *)&debg[1])),
			       (int)(*((s16 *)&debg[2])),
			       (int)(*((s16 *)&debg[3])),
			       (int)(*((s16 *)&debg[4])),
			       (int)(*((s16 *)&debg[5])));
		} else {
			printk(KERN_WARNING
			       "bmi323 pushing to buffer %d/%d -- time: %d accel: %d %d %d gyro: %d %d %d",
			       (int)(f + 1), (int)available_frame_aggregates,
			       (int)timestamp, (int)(*((s16 *)&debg[0])),
			       (int)(*((s16 *)&debg[1])),
			       (int)(*((s16 *)&debg[2])),
			       (int)(*((s16 *)&debg[3])),
			       (int)(*((s16 *)&debg[4])),
			       (int)(*((s16 *)&debg[5])));
		}
#endif

		iio_push_to_buffers_with_timestamp(
			indio_dev, &temp_data[0],
			first_sample_timestamp_ns +
				(fifo_frame_time_ns * (s64)j));
	}

bmi323_irq_done:
	mutex_unlock(&indio_data->bmi323.mutex);

	/*
	 * Tell the core we are done with this trigger and ready for the
	 * next one.
	 */
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

int bmi323_set_trigger_state(struct iio_trigger *trig, bool state)
{
	return 0;
}

/*
// The following is meant to be used in a IRQ-enabled hardware
static  const struct iio_trigger_ops time_trigger_ops = {
    .set_trigger_state = &bmi323_set_trigger_state,
    //.reenable = NULL,
    .validate_device = &iio_trigger_validate_own_device,
};
*/

/*
 * A very basic scan mask: everything can work in conjunction with everything else so no need to worry about
 * managing conbinations of mutually exclusive data sources...
 */
static const unsigned long bmi323_accel_scan_masks[] = {
	BIT(BMI323_ACCEL_AXIS_X) | BIT(BMI323_ACCEL_AXIS_Y) |
		BIT(BMI323_ACCEL_AXIS_Z) | BIT(BMI323_GYRO_AXIS_X) |
		BIT(BMI323_GYRO_AXIS_Y) |
		BIT(BMI323_GYRO_AXIS_Z) /*| BIT(BMI323_TEMP)*/,
	0
};

int bmi323_iio_init(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	struct irq_data *irq_desc = NULL;

	if (data->bmi323.i2c_client != NULL) {
		data->bmi323.dev = &data->bmi323.i2c_client->dev;
	} else if (data->bmi323.spi_client != NULL) {
		data->bmi323.dev = &data->bmi323.spi_client->dev;
	} else {
		return -ENODEV;
	}

	int ret = 0;

	/* change to 8 for a default 200Hz sampling rate */
	const int gyr_odr_conf_idx = 7;
	const int acc_odr_conf_idx = 7;

	mutex_init(&data->bmi323.mutex);

	data->bmi323.acc_odr_time_ns =
		bmi323_accel_odr_map[acc_odr_conf_idx].time_ns;
	data->bmi323.gyr_odr_time_ns =
		bmi323_gyro_odr_map[gyr_odr_conf_idx].time_ns;

	// FIFO enabled for gyro, accel and temp. Overwrite older samples.
	data->bmi323.fifo_conf_reg_value = cpu_to_le16((u16)0x0F00U);
	//data->bmi323.fifo_conf_reg_value = cpu_to_le16((u16)0x0E00U);
	//data->bmi323.fifo_conf_reg_value = cpu_to_le16((u16)0x0600U); // working

	// now set the (default) normal mode...
	// normal mode: 0x4000
	// no averaging: 0x0000
	data->bmi323.acc_conf_reg_value = cpu_to_le16(
		0x4000 | ((u16)BMC150_BMI323_ACCEL_RANGE_2_VAL << (u16)4U) |
		((u16)bmi323_accel_odr_map[acc_odr_conf_idx].hw_val));

	// now set the (default) normal mode...
	// normal mode: 0x4000
	// no averaging: 0x0000
	// filtering to ODR/2: 0x0000
	data->bmi323.gyr_conf_reg_value = cpu_to_le16(
		0x4000 | ((u16)BMC150_BMI323_GYRO_RANGE_125_VAL << (u16)4U) |
		((u16)bmi323_gyro_odr_map[gyr_odr_conf_idx].hw_val));

	// the datasheet states that FIFO buffer MUST be enabled before enabling any sensor
	ret = bmi323_write_u16(&data->bmi323, BMC150_BMI323_FIFO_CONF_REG,
			       data->bmi323.fifo_conf_reg_value);
	if (ret != 0) {
		return -1;
	}

	ret = bmi323_write_u16(&data->bmi323, BMC150_BMI323_ACC_CONF_REG,
			       data->bmi323.acc_conf_reg_value);
	if (ret != 0) {
		return -1;
	}

	ret = bmi323_write_u16(&data->bmi323, BMC150_BMI323_GYR_CONF_REG,
			       data->bmi323.gyr_conf_reg_value);
	if (ret != 0) {
		return -2;
	}

	indio_dev->channels = bmi323_channels;
	indio_dev->num_channels = ARRAY_SIZE(bmi323_channels);
	indio_dev->name = "bmi323";
	indio_dev->available_scan_masks = bmi323_accel_scan_masks;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bmi323_accel_info;
	indio_dev->label = "bmi323-accel_base";

	if (data->bmi323.irq > 0) {
		dev_info(data->bmi323.dev, "IRQ pin reported as connected: %d",
			 data->bmi323.irq);

		irq_desc = irq_get_irq_data(data->bmi323.irq);
		if (!irq_desc) {
			dev_err(data->bmi323.dev,
				"Could not find IRQ %d. ignoring it.\n",
				data->bmi323.irq);
			goto bmi323_iio_init_missing_irq_pin;
		}

		//data->bmi323.trig[0] = devm_iio_trigger_alloc(data->bmi323.dev, "trig-fifo_full-%s-%d", indio_dev->name, iio_device_id(indio_dev));
		//if (data->bmi323.trig[0] == NULL) {
		//	ret = -ENOMEM;
		//	goto bmi323_iio_init_err_trigger_unregister;
		//}
		//
		//data->bmi323.trig[0]->ops = &time_trigger_ops;
		//iio_trigger_set_drvdata(data->bmi323.trig[0], indio_dev);
		//ret = devm_iio_trigger_register(data->bmi323.dev, data->bmi323.trig[0]);
		//if (ret) {
		//	dev_err(data->bmi323.dev, "iio trigger register failed\n");
		//	goto bmi323_iio_init_err_trigger_unregister;
		//}

		/*
		 * register triggers BEFORE buffer setup so that they are cleared
		 * on emergence exit by bmi323_iio_init_err_trigger_unregister.
		 *
		 * This is just a placeholder until I can get my hands on a bmi323
		 * device that has the IRQ pin actually connected to the CPU.
		 */

		/* here resume operation with the module part common to irq and non-irq enabled code. */
		goto bmi323_iio_init_common_irq_and_not_irq;
	}

bmi323_iio_init_missing_irq_pin:
	dev_info(
		data->bmi323.dev,
		"IRQ pin NOT connected (irq=%d). Will continue normally without triggers.",
		data->bmi323.irq);

bmi323_iio_init_common_irq_and_not_irq:

	/* Once orientation matrix is implemented switch this to iio_triggered_buffer_setup_ext. */
	ret = iio_triggered_buffer_setup(indio_dev, &iio_pollfunc_store_time,
					 iio_bmi323_trigger_h,
					 &bmi323_buffer_ops);
	if (ret < 0) {
		dev_err(data->bmi323.dev,
			"Failed: iio triggered buffer setup: %d\n", ret);
		goto bmi323_iio_init_err_trigger_unregister;
	}

	ret = pm_runtime_set_active(data->bmi323.dev);
	if (ret) {
		dev_err(data->bmi323.dev,
			"bmi323 unable to initialize runtime PD: pm_runtime_set_active returned %d\n",
			ret);
		goto bmi323_iio_init_err_buffer_cleanup;
	}

	pm_runtime_enable(data->bmi323.dev);
	pm_runtime_set_autosuspend_delay(data->bmi323.dev,
					 BMC150_BMI323_AUTO_SUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(data->bmi323.dev);

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(data->bmi323.dev,
			"bmi323 unable to register iio device: %d\n", ret);
		goto bmi323_iio_init_err_pm_cleanup;
	}

	return 0;

bmi323_iio_init_err_pm_cleanup:
	pm_runtime_dont_use_autosuspend(data->bmi323.dev);
	pm_runtime_disable(data->bmi323.dev);
bmi323_iio_init_err_buffer_cleanup:
	iio_triggered_buffer_cleanup(indio_dev);
bmi323_iio_init_err_trigger_unregister:
	/*
	 * unregister triggers if they have been setup already.
	 * iio_trigger_unregister shall be used in that regard.
	 *
	 * This is just a placeholder until I can get my hands on a bmi323
	 * device that has the IRQ pin actually connected to the CPU.
	 */
	//if (data->bmi323.trig[0] != NULL) {
	//	iio_trigger_unregister(data->bmi323.trig[0]);
	//}

	return ret;
}
EXPORT_SYMBOL_NS_GPL(bmi323_iio_init, IIO_BMC150);

void bmi323_iio_deinit(struct iio_dev *indio_dev)
{
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	struct device *dev = bmi323_get_managed_device(&data->bmi323);

	iio_device_unregister(indio_dev);

	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	pm_runtime_put_noidle(dev);

	iio_triggered_buffer_cleanup(indio_dev);

	//iio_device_free(indio_dev); // this isn't done in the bmg160 driver nor in other drivers so I guess I shouldn't do it too

	mutex_unlock(&data->bmi323.mutex);
	bmi323_chip_rst(&data->bmi323);
	mutex_unlock(&data->bmi323.mutex);
}
EXPORT_SYMBOL_NS_GPL(bmi323_iio_deinit, IIO_BMC150);

#ifdef CONFIG_PM_SLEEP
static int bmc150_accel_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	if (data->dev_type == BMI323) {
		int ret;

		//dev_warn(dev, "bmi323 suspending driver...");

		// here push the register GYRO & ACCEL configuration and issue a reset so that chip goes to sleep mode (the default one after a reset)
		mutex_unlock(&data->bmi323.mutex);

		ret = bmi323_chip_rst(&data->bmi323);
		mutex_unlock(&data->bmi323.mutex);
		if (ret != 0) {
			dev_err(dev,
				"bmi323 error in suspend on bmi323_chip_rst: %d\n",
				ret);
			data->bmi323.flags |= BMI323_FLAGS_RESET_FAILED;
			return -EAGAIN;
		}

		return 0;
	}

	mutex_lock(&data->mutex);
	bmc150_accel_set_mode(data, BMC150_ACCEL_SLEEP_MODE_SUSPEND, 0);
	mutex_unlock(&data->mutex);

	return 0;
}

static int bmc150_accel_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);

	if (data->dev_type == BMI323) {
		int ret;

		//dev_warn(dev, "bmi323 resuming driver...");

		// here pop the register GYRO & ACCEL configuration and issue a reset so that chip goes to sleep mode (the default one after a reset)
		mutex_lock(&data->bmi323.mutex);

		// this was done already in runtime_sleep function.
		if ((data->bmi323.flags & BMI323_FLAGS_RESET_FAILED) != 0x00U) {
			ret = bmi323_chip_rst(&data->bmi323);
			if (ret == 0) {
				data->bmi323.flags &=
					~BMI323_FLAGS_RESET_FAILED;
			} else {
				goto bmi323_bmc150_accel_resume_terminate;
			}
		}

		ret = bmi323_write_u16(&data->bmi323,
				       BMC150_BMI323_FIFO_CONF_REG,
				       data->bmi323.fifo_conf_reg_value);
		if (ret != 0) {
			goto bmi323_bmc150_accel_resume_terminate;
		}

		ret = bmi323_write_u16(&data->bmi323,
				       BMC150_BMI323_GYR_CONF_REG,
				       data->bmi323.gyr_conf_reg_value);
		if (ret != 0) {
			goto bmi323_bmc150_accel_resume_terminate;
		}

		ret = bmi323_write_u16(&data->bmi323,
				       BMC150_BMI323_ACC_CONF_REG,
				       data->bmi323.acc_conf_reg_value);
		if (ret != 0) {
			goto bmi323_bmc150_accel_resume_terminate;
		}

bmi323_bmc150_accel_resume_terminate:
		mutex_unlock(&data->bmi323.mutex);
		if (ret != 0) {
			return -EAGAIN;
		}

		/*
		 * datasheet says "Start-up time": suspend to high performance mode is tipically 30ms,
		 * however when setting this to 32 or even higher the first reading from the gyro (unlike accel part)
		 * is actually the (wrong) default value 0x8000 so it is better to sleep a bit longer
		 * to prevent issues and give time to the sensor to pick up first readings...
		 */
		msleep_interruptible(64);

		return 0;
	}

	mutex_lock(&data->mutex);
	bmc150_accel_set_mode(data, BMC150_ACCEL_SLEEP_MODE_NORMAL, 0);
	bmc150_accel_fifo_set_mode(data);
	mutex_unlock(&data->mutex);

	if (data->resume_callback)
		data->resume_callback(dev);

	return 0;
}
#endif

#ifdef CONFIG_PM
static int bmc150_accel_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;

	if (data->dev_type == BMI323) {
		//dev_warn(dev, "bmi323 suspending runtime...");

		/*
		 * Every operation requiring this function have the mutex locked already:
		 * with mutex_lock(&data->bmi323.mutex);
		 */
		ret = bmi323_chip_rst(&data->bmi323);
		if (ret != 0) {
			dev_err(dev,
				"bmi323 error in runtime_suspend on bmi323_chip_rst: %d\n",
				ret);
			data->bmi323.flags |= BMI323_FLAGS_RESET_FAILED;
			return -EAGAIN;
		}

		return 0;
	}

	ret = bmc150_accel_set_mode(data, BMC150_ACCEL_SLEEP_MODE_SUSPEND, 0);
	if (ret < 0)
		return -EAGAIN;

	return 0;
}

static int bmc150_accel_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct bmc150_accel_data *data = iio_priv(indio_dev);
	int ret;
	int sleep_val;

	if (data->dev_type == BMI323) {
		//dev_warn(dev, "bmi323 resuming runtime...");

		/*
		 * Every operation requiring this function have the mutex locked already:
		 * with mutex_lock(&data->bmi323.mutex);
		 */

		// recover from a bad state if it was left that way on reuntime_suspend
		if ((data->bmi323.flags & BMI323_FLAGS_RESET_FAILED) != 0x00U) {
			ret = bmi323_chip_rst(&data->bmi323);
			if (ret == 0) {
				data->bmi323.flags &=
					~BMI323_FLAGS_RESET_FAILED;
			} else {
				goto bmi323_bmc150_accel_runtime_resume_terminate;
			}
		}

		ret = bmi323_write_u16(&data->bmi323,
				       BMC150_BMI323_FIFO_CONF_REG,
				       data->bmi323.fifo_conf_reg_value);
		if (ret != 0) {
			dev_err(dev,
				"bmi323 writing to GYR_CONF register failed");
			goto bmi323_bmc150_accel_runtime_resume_terminate;
		}

		ret = bmi323_write_u16(&data->bmi323,
				       BMC150_BMI323_GYR_CONF_REG,
				       data->bmi323.gyr_conf_reg_value);
		if (ret != 0) {
			dev_err(dev,
				"bmi323 writing to GYR_CONF register failed");
			goto bmi323_bmc150_accel_runtime_resume_terminate;
		}

		ret = bmi323_write_u16(&data->bmi323,
				       BMC150_BMI323_ACC_CONF_REG,
				       data->bmi323.acc_conf_reg_value);
		if (ret != 0) {
			dev_err(dev,
				"bmi323 writing to ACC_CONF register failed");
			goto bmi323_bmc150_accel_runtime_resume_terminate;
		}

bmi323_bmc150_accel_runtime_resume_terminate:
		if (ret != 0) {
			dev_err(dev,
				"bmi323 bmc150_accel_runtime_resume -EAGAIN");
			return -EAGAIN;
		}

		/*
		 * datasheet says "Start-up time": suspend to high performance mode is tipically 30ms,
		 * however when setting this to 32 or even higher the first reading from the gyro (unlike accel part)
		 * is actually the (wrong) default value 0x8000 so it is better to sleep a bit longer
		 * to prevent issues and give time to the sensor to pick up first readings...
		 */
		msleep_interruptible(64);

		return 0;
	}

	ret = bmc150_accel_set_mode(data, BMC150_ACCEL_SLEEP_MODE_NORMAL, 0);
	if (ret < 0)
		return ret;
	ret = bmc150_accel_fifo_set_mode(data);
	if (ret < 0)
		return ret;

	sleep_val = bmc150_accel_get_startup_times(data);
	if (sleep_val < 20)
		usleep_range(sleep_val * 1000, 20000);
	else
		msleep_interruptible(sleep_val);

	return 0;
}
#endif

const struct dev_pm_ops bmc150_accel_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(bmc150_accel_suspend, bmc150_accel_resume)
	SET_RUNTIME_PM_OPS(bmc150_accel_runtime_suspend,
			   bmc150_accel_runtime_resume, NULL)
};
EXPORT_SYMBOL_NS_GPL(bmc150_accel_pm_ops, IIO_BMC150);

MODULE_AUTHOR("Srinivas Pandruvada <srinivas.pandruvada@linux.intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("BMC150 accelerometer driver");
