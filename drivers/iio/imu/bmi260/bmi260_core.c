// SPDX-License-Identifier: GPL-2.0
/*
 * IIO core driver for Bosch BMI260 6-Axis IMU.
 *
 * Copyright (C) 2023, Justin Weiss <justin@justinweiss.com>
 *
 * This driver is also based on the BMI160 driver, which is:
 * Copyright (c) 2016, Intel Corporation.
 * Copyright (c) 2019, Martin Kelly.
 */
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/acpi.h>

#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>

#include "bmi260.h"
#include "third_party/bmi260_config.h"

#define BMI260_REG_CHIP_ID	0x00
#define BMI260_CHIP_ID_VAL	0x27 /* 0x24 for BMI270 */

#define BMI260_REG_PMU_STATUS	0x03

/* X axis data low byte address, the rest can be obtained using axis offset */
#define BMI260_REG_DATA_AUX_XOUT_L	0x04
#define BMI260_REG_DATA_ACCEL_XOUT_L	0x0C
#define BMI260_REG_DATA_GYRO_XOUT_L	0x12

#define BMI260_REG_INTERNAL_STATUS	0x21
#define BMI260_STATUS_MESSAGE_MASK	GENMASK(3, 0)

#define BMI260_REG_ACCEL_CONFIG		0x40
#define BMI260_ACCEL_CONFIG_ODR_MASK	GENMASK(3, 0)
#define BMI260_ACCEL_CONFIG_BWP_MASK	GENMASK(6, 4)

#define BMI260_REG_ACCEL_RANGE		0x41
#define BMI260_ACCEL_RANGE_MASK		GENMASK(1, 0)
#define BMI260_ACCEL_RANGE_2G		0x00
#define BMI260_ACCEL_RANGE_4G		0x01
#define BMI260_ACCEL_RANGE_8G		0x02
#define BMI260_ACCEL_RANGE_16G		0x03

#define BMI260_REG_GYRO_CONFIG		0x42
#define BMI260_GYRO_CONFIG_ODR_MASK	GENMASK(3, 0)
#define BMI260_GYRO_CONFIG_BWP_MASK	GENMASK(5, 4)

#define BMI260_REG_GYRO_RANGE		0x43
#define BMI260_GYRO_RANGE_MASK		GENMASK(2, 0)
#define BMI260_GYRO_RANGE_2000DPS	0x00
#define BMI260_GYRO_RANGE_1000DPS	0x01
#define BMI260_GYRO_RANGE_500DPS	0x02
#define BMI260_GYRO_RANGE_250DPS	0x03
#define BMI260_GYRO_RANGE_125DPS	0x04

#define BMI260_REG_INIT_CTRL		0x59
#define BMI260_REG_INIT_DATA		0x5E

#define BMI260_REG_PWR_CONF		0x7C
#define BMI260_PWR_CONF_ADV_PWR_SAVE	BIT(0)
#define BMI260_PWR_CONF_FIFO_WAKE_UP	BIT(1)
#define BMI260_PWR_CONF_FUP_EN		BIT(2)

#define BMI260_REG_PWR_CTRL		0x7D
#define BMI260_PWR_CTRL_AUX_EN		BIT(0)
#define BMI260_PWR_CTRL_GYR_EN		BIT(1)
#define BMI260_PWR_CTRL_ACC_EN		BIT(2)
#define BMI260_PWR_CTRL_TEMP_EN		BIT(3)

#define BMI260_REG_CMD			0x7E
#define BMI260_CMD_SOFTRESET		0xB6

#define BMI260_REG_FIFO_CONFIG_1	0x49
#define BMI260_FIFO_TAG_INT1_LEVEL	BIT(0)
#define BMI260_FIFO_TAG_INT2_LEVEL	BIT(2)

#define BMI260_REG_INT1_IO_CTRL		0x53
#define BMI260_REG_INT2_IO_CTRL		0x54
#define BMI260_INT_IO_CTRL_MASK		GENMASK(4, 1)
#define BMI260_ACTIVE_HIGH		BIT(1)
#define BMI260_OPEN_DRAIN		BIT(2)
#define BMI260_OUTPUT_EN		BIT(3)
#define BMI260_INPUT_EN			BIT(4)

#define BMI260_REG_INT_MAP_DATA		0x58
#define BMI260_INT1_MAP_DRDY_EN		BIT(2)
#define BMI260_INT2_MAP_DRDY_EN		BIT(6)

#define BMI260_REG_DUMMY		0x7F

#define BMI260_NORMAL_WRITE_USLEEP	2
#define BMI260_SUSPENDED_WRITE_USLEEP	450
#define BMI260_SOFTRESET_USLEEP		2000
#define BMI260_INIT_USLEEP		22000

#define BMI260_CHANNEL(_type, _axis, _index) {			\
	.type = _type,						\
	.modified = 1,						\
	.channel2 = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |  \
		BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.scan_index = _index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = 16,					\
		.storagebits = 16,				\
		.endianness = IIO_LE,				\
	},							\
	.ext_info = bmi260_ext_info,				\
}

/* scan indexes follow DATA register order */
enum bmi260_scan_axis {
	BMI260_SCAN_AUX_X = 0,
	BMI260_SCAN_AUX_Y,
	BMI260_SCAN_AUX_Z,
	BMI260_SCAN_AUX_R,
	BMI260_SCAN_ACCEL_X,
	BMI260_SCAN_ACCEL_Y,
	BMI260_SCAN_ACCEL_Z,
	BMI260_SCAN_GYRO_X,
	BMI260_SCAN_GYRO_Y,
	BMI260_SCAN_GYRO_Z,
	BMI260_SCAN_TIMESTAMP,
};

enum bmi260_sensor_type {
	BMI260_ACCEL	= 0,
	BMI260_GYRO,
	BMI260_AUX,
	BMI260_NUM_SENSORS /* must be last */
};

const struct regmap_config bmi260_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};
EXPORT_SYMBOL_NS(bmi260_regmap_config, IIO_BMI260);

struct bmi260_regs {
	u8 data; /* LSB byte register for X-axis */
	u8 config;
	u8 config_odr_mask;
	u8 config_bwp_mask;
	u8 range;
};

static struct bmi260_regs bmi260_regs[] = {
	[BMI260_ACCEL] = {
		.data	= BMI260_REG_DATA_ACCEL_XOUT_L,
		.config	= BMI260_REG_ACCEL_CONFIG,
		.config_odr_mask = BMI260_ACCEL_CONFIG_ODR_MASK,
		.config_bwp_mask = BMI260_ACCEL_CONFIG_BWP_MASK,
		.range	= BMI260_REG_ACCEL_RANGE,
	},
	[BMI260_GYRO] = {
		.data	= BMI260_REG_DATA_GYRO_XOUT_L,
		.config	= BMI260_REG_GYRO_CONFIG,
		.config_odr_mask = BMI260_GYRO_CONFIG_ODR_MASK,
		.config_bwp_mask = BMI260_GYRO_CONFIG_BWP_MASK,
		.range	= BMI260_REG_GYRO_RANGE,
	},
};

struct bmi260_scale {
	u8 bits;
	int uscale;
};

struct bmi260_odr {
	u8 bits;
	int odr;
	int uodr;
};

static const struct bmi260_scale bmi260_accel_scale[] = {
	{ BMI260_ACCEL_RANGE_2G, 598},
	{ BMI260_ACCEL_RANGE_4G, 1197},
	{ BMI260_ACCEL_RANGE_8G, 2394},
	{ BMI260_ACCEL_RANGE_16G, 4788},
};

static const struct bmi260_scale bmi260_gyro_scale[] = {
	{ BMI260_GYRO_RANGE_2000DPS, 1065},
	{ BMI260_GYRO_RANGE_1000DPS, 532},
	{ BMI260_GYRO_RANGE_500DPS, 266},
	{ BMI260_GYRO_RANGE_250DPS, 133},
	{ BMI260_GYRO_RANGE_125DPS, 66},
};

struct bmi260_scale_item {
	const struct bmi260_scale *tbl;
	int num;
};

static const struct  bmi260_scale_item bmi260_scale_table[] = {
	[BMI260_ACCEL] = {
		.tbl	= bmi260_accel_scale,
		.num	= ARRAY_SIZE(bmi260_accel_scale),
	},
	[BMI260_GYRO] = {
		.tbl	= bmi260_gyro_scale,
		.num	= ARRAY_SIZE(bmi260_gyro_scale),
	},
};

static const struct bmi260_odr bmi260_accel_odr[] = {
	{0x01, 0, 781250},
	{0x02, 1, 562500},
	{0x03, 3, 125000},
	{0x04, 6, 250000},
	{0x05, 12, 500000},
	{0x06, 25, 0},
	{0x07, 50, 0},
	{0x08, 100, 0},
	{0x09, 200, 0},
	{0x0A, 400, 0},
	{0x0B, 800, 0},
	{0x0C, 1600, 0},
};

static const struct bmi260_odr bmi260_gyro_odr[] = {
	{0x06, 25, 0},
	{0x07, 50, 0},
	{0x08, 100, 0},
	{0x09, 200, 0},
	{0x0A, 400, 0},
	{0x0B, 800, 0},
	{0x0C, 1600, 0},
	{0x0D, 3200, 0},
};

struct bmi260_odr_item {
	const struct bmi260_odr *tbl;
	int num;
};

static const struct  bmi260_odr_item bmi260_odr_table[] = {
	[BMI260_ACCEL] = {
		.tbl	= bmi260_accel_odr,
		.num	= ARRAY_SIZE(bmi260_accel_odr),
	},
	[BMI260_GYRO] = {
		.tbl	= bmi260_gyro_odr,
		.num	= ARRAY_SIZE(bmi260_gyro_odr),
	},
};

#ifdef CONFIG_ACPI
/*
 * Support for getting accelerometer information from ACPI nodes.
 * Based off of the bmc150 implementation.
 */
static bool bmi260_apply_acpi_orientation(struct device *dev,
					  struct iio_mount_matrix *orientation)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct acpi_device *adev = ACPI_COMPANION(dev);
	char *name, *alt_name, *label, *str;
	union acpi_object *obj, *elements;
	acpi_status status;
	int i, j, val[3];

	if (!adev)
		return false;

	alt_name = "ROMS";
	label = "accel-display";

	if (acpi_has_method(adev->handle, alt_name)) {
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

#else
static bool bmi260_apply_acpi_orientation(struct device *dev,
					  struct iio_mount_matrix *orientation)
{
	return false;
}
#endif

static const struct iio_mount_matrix *
bmi260_get_mount_matrix(const struct iio_dev *indio_dev,
			const struct iio_chan_spec *chan)
{
	struct bmi260_data *data = iio_priv(indio_dev);

	return &data->orientation;
}

static const struct iio_chan_spec_ext_info bmi260_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, bmi260_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec bmi260_channels[] = {
	BMI260_CHANNEL(IIO_ACCEL, X, BMI260_SCAN_ACCEL_X),
	BMI260_CHANNEL(IIO_ACCEL, Y, BMI260_SCAN_ACCEL_Y),
	BMI260_CHANNEL(IIO_ACCEL, Z, BMI260_SCAN_ACCEL_Z),
	BMI260_CHANNEL(IIO_ANGL_VEL, X, BMI260_SCAN_GYRO_X),
	BMI260_CHANNEL(IIO_ANGL_VEL, Y, BMI260_SCAN_GYRO_Y),
	BMI260_CHANNEL(IIO_ANGL_VEL, Z, BMI260_SCAN_GYRO_Z),
	IIO_CHAN_SOFT_TIMESTAMP(BMI260_SCAN_TIMESTAMP),
};

static enum bmi260_sensor_type bmi260_to_sensor(enum iio_chan_type iio_type)
{
	switch (iio_type) {
	case IIO_ACCEL:
		return BMI260_ACCEL;
	case IIO_ANGL_VEL:
		return BMI260_GYRO;
	default:
		return -EINVAL;
	}
}

static
int bmi260_set_scale(struct bmi260_data *data, enum bmi260_sensor_type t,
		     int uscale)
{
	int i;

	for (i = 0; i < bmi260_scale_table[t].num; i++)
		if (bmi260_scale_table[t].tbl[i].uscale == uscale)
			break;

	if (i == bmi260_scale_table[t].num)
		return -EINVAL;

	return regmap_write(data->regmap, bmi260_regs[t].range,
			    bmi260_scale_table[t].tbl[i].bits);
}

static
int bmi260_get_scale(struct bmi260_data *data, enum bmi260_sensor_type t,
		     int *uscale)
{
	int i, ret, val;

	ret = regmap_read(data->regmap, bmi260_regs[t].range, &val);
	if (ret)
		return ret;

	for (i = 0; i < bmi260_scale_table[t].num; i++)
		if (bmi260_scale_table[t].tbl[i].bits == val) {
			*uscale = bmi260_scale_table[t].tbl[i].uscale;
			return 0;
		}

	return -EINVAL;
}

static int bmi260_get_data(struct bmi260_data *data, int chan_type,
			   int axis, int *val)
{
	u8 reg;
	int ret;
	__le16 sample;
	enum bmi260_sensor_type t = bmi260_to_sensor(chan_type);

	reg = bmi260_regs[t].data + (axis - IIO_MOD_X) * sizeof(sample);

	ret = regmap_bulk_read(data->regmap, reg, &sample, sizeof(sample));
	if (ret)
		return ret;

	*val = sign_extend32(le16_to_cpu(sample), 15);

	return 0;
}

static
int bmi260_set_odr(struct bmi260_data *data, enum bmi260_sensor_type t,
		   int odr, int uodr)
{
	int i;

	for (i = 0; i < bmi260_odr_table[t].num; i++)
		if (bmi260_odr_table[t].tbl[i].odr == odr &&
		    bmi260_odr_table[t].tbl[i].uodr == uodr)
			break;

	if (i >= bmi260_odr_table[t].num)
		return -EINVAL;

	return regmap_update_bits(data->regmap,
				  bmi260_regs[t].config,
				  bmi260_regs[t].config_odr_mask,
				  bmi260_odr_table[t].tbl[i].bits);
}

static int bmi260_get_odr(struct bmi260_data *data, enum bmi260_sensor_type t,
			  int *odr, int *uodr)
{
	int i, val, ret;

	ret = regmap_read(data->regmap, bmi260_regs[t].config, &val);
	if (ret)
		return ret;

	val &= bmi260_regs[t].config_odr_mask;

	for (i = 0; i < bmi260_odr_table[t].num; i++)
		if (val == bmi260_odr_table[t].tbl[i].bits)
			break;

	if (i >= bmi260_odr_table[t].num)
		return -EINVAL;

	*odr = bmi260_odr_table[t].tbl[i].odr;
	*uodr = bmi260_odr_table[t].tbl[i].uodr;

	return 0;
}

static irqreturn_t bmi260_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct bmi260_data *data = iio_priv(indio_dev);
	int i, ret, j = 0, base = BMI260_REG_DATA_AUX_XOUT_L;
	__le16 sample;

	for_each_set_bit(i, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		ret = regmap_bulk_read(data->regmap, base + i * sizeof(sample),
				       &sample, sizeof(sample));
		if (ret)
			goto done;
		data->buf[j++] = sample;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, data->buf, pf->timestamp);
done:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int bmi260_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret;
	struct bmi260_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = bmi260_get_data(data, chan->type, chan->channel2, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 0;
		ret = bmi260_get_scale(data,
				       bmi260_to_sensor(chan->type), val2);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = bmi260_get_odr(data, bmi260_to_sensor(chan->type),
				     val, val2);
		return ret ? ret : IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bmi260_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct bmi260_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return bmi260_set_scale(data,
					bmi260_to_sensor(chan->type), val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return bmi260_set_odr(data, bmi260_to_sensor(chan->type),
				      val, val2);
	default:
		return -EINVAL;
	}

	return 0;
}

static
IIO_CONST_ATTR(in_accel_sampling_frequency_available,
	       "0.78125 1.5625 3.125 6.25 12.5 25 50 100 200 400 800 1600");
static
IIO_CONST_ATTR(in_anglvel_sampling_frequency_available,
	       "25 50 100 200 400 800 1600 3200");
static
IIO_CONST_ATTR(in_accel_scale_available,
	       "0.000598 0.001197 0.002394 0.004788");
static
IIO_CONST_ATTR(in_anglvel_scale_available,
	       "0.001065 0.000532 0.000266 0.000133 0.000066");

static struct attribute *bmi260_attrs[] = {
	&iio_const_attr_in_accel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_sampling_frequency_available.dev_attr.attr,
	&iio_const_attr_in_accel_scale_available.dev_attr.attr,
	&iio_const_attr_in_anglvel_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group bmi260_attrs_group = {
	.attrs = bmi260_attrs,
};

static const struct iio_info bmi260_info = {
	.read_raw = bmi260_read_raw,
	.write_raw = bmi260_write_raw,
	.attrs = &bmi260_attrs_group,
};

static int bmi260_write_conf_reg(struct regmap *regmap, unsigned int reg,
				 unsigned int mask, unsigned int bits,
				 unsigned int write_usleep)
{
	int ret;
	unsigned int val;

	ret = regmap_read(regmap, reg, &val);
	if (ret)
		return ret;

	val = (val & ~mask) | bits;

	ret = regmap_write(regmap, reg, val);
	if (ret)
		return ret;

	/*
	 * We need to wait after writing before we can write again. See the
	 * datasheet, page 93.
	 */
	usleep_range(write_usleep, write_usleep + 1000);

	return 0;
}

static int bmi260_config_pin(struct regmap *regmap, enum bmi260_int_pin pin,
			     bool level_triggered, u8 irq_mask,
			     unsigned long write_usleep)
{
	int ret;
	struct device *dev = regmap_get_device(regmap);
	unsigned int ctrl_reg;
	unsigned int drdy_val;
	unsigned int level_val;
	u8 int_out_ctrl_bits;
	const char *pin_name;

	switch (pin) {
	case BMI260_PIN_INT1:
		ctrl_reg = BMI260_REG_INT1_IO_CTRL;
		drdy_val = BMI260_INT1_MAP_DRDY_EN;
		level_val = BMI260_FIFO_TAG_INT1_LEVEL;
		break;
	case BMI260_PIN_INT2:
		ctrl_reg = BMI260_REG_INT2_IO_CTRL;
		drdy_val = BMI260_INT2_MAP_DRDY_EN;
		level_val = BMI260_FIFO_TAG_INT2_LEVEL;
		break;
	}

	/*
	 * Enable the requested pin with the right settings:
	 * - Push-pull/open-drain
	 * - Active low/high
	 */
	int_out_ctrl_bits = BMI260_OUTPUT_EN | BMI260_INPUT_EN;
	int_out_ctrl_bits |= irq_mask;

	ret = bmi260_write_conf_reg(regmap, ctrl_reg,
				    BMI260_INT_IO_CTRL_MASK, int_out_ctrl_bits,
				    write_usleep);
	if (ret)
		return ret;

	/* Set level/edge triggered */
	if (level_triggered) {
		ret = bmi260_write_conf_reg(regmap, BMI260_REG_FIFO_CONFIG_1,
		                            level_val, level_val,
		                            write_usleep);
		if (ret)
			return ret;
	}

	/* Map interrupts to the requested pin. */
	ret = bmi260_write_conf_reg(regmap, BMI260_REG_INT_MAP_DATA,
				    drdy_val, drdy_val,
				    write_usleep);
	if (ret) {
		switch (pin) {
		case BMI260_PIN_INT1:
			pin_name = "INT1";
			break;
		case BMI260_PIN_INT2:
			pin_name = "INT2";
			break;
		}
		dev_err(dev, "Failed to configure %s IRQ pin", pin_name);
	}

	return ret;
}

int bmi260_enable_irq(struct regmap *regmap, enum bmi260_int_pin pin, bool enable)
{
	unsigned int enable_bit = 0;
	unsigned int mask = 0;

	switch (pin) {
	case BMI260_PIN_INT1:
		mask = BMI260_INT1_MAP_DRDY_EN;
		break;
	case BMI260_PIN_INT2:
		mask = BMI260_INT2_MAP_DRDY_EN;
		break;
	}

	if (enable)
		enable_bit = mask;

	return bmi260_write_conf_reg(regmap, BMI260_REG_INT_MAP_DATA,
				     mask, enable_bit,
				     BMI260_NORMAL_WRITE_USLEEP);
}
EXPORT_SYMBOL_NS(bmi260_enable_irq, IIO_BMI260);

static int bmi260_get_irq(struct fwnode_handle *fwnode, enum bmi260_int_pin *pin)
{
	int irq;

	/* Use INT1 if possible, otherwise fall back to INT2. */
	irq = fwnode_irq_get_byname(fwnode, "INT1");
	if (irq > 0) {
		*pin = BMI260_PIN_INT1;
		return irq;
	}

	irq = fwnode_irq_get_byname(fwnode, "INT2");
	if (irq > 0)
		*pin = BMI260_PIN_INT2;

	return irq;
}

static int bmi260_config_device_irq(struct iio_dev *indio_dev, int irq_type,
				    enum bmi260_int_pin pin)
{
	bool open_drain;
	u8 irq_mask;
	bool level_triggered = true;
	struct bmi260_data *data = iio_priv(indio_dev);
	struct device *dev = regmap_get_device(data->regmap);

	/* Edge-triggered, active-low is the default if we set all zeroes. */
	if (irq_type == IRQF_TRIGGER_RISING) {
		irq_mask = BMI260_ACTIVE_HIGH;
		level_triggered = false;
	} else if (irq_type == IRQF_TRIGGER_FALLING) {
		irq_mask = 0;
		level_triggered = false;
	} else if (irq_type == IRQF_TRIGGER_HIGH) {
		irq_mask = BMI260_ACTIVE_HIGH;
	} else if (irq_type == IRQF_TRIGGER_LOW) {
		irq_mask = 0;
	} else {
		dev_err(&indio_dev->dev,
			"Invalid interrupt type 0x%x specified\n", irq_type);
		return -EINVAL;
	}

	open_drain = device_property_read_bool(dev, "drive-open-drain");

	if (open_drain)
		irq_mask |= BMI260_OPEN_DRAIN;

	return bmi260_config_pin(data->regmap, pin, level_triggered, irq_mask,
				 BMI260_NORMAL_WRITE_USLEEP);
}

static int bmi260_setup_irq(struct iio_dev *indio_dev, int irq,
			    enum bmi260_int_pin pin)
{
	struct irq_data *desc;
	u32 irq_type;
	int ret;

	desc = irq_get_irq_data(irq);
	if (!desc) {
		dev_err(&indio_dev->dev, "Could not find IRQ %d\n", irq);
		return -EINVAL;
	}

	irq_type = irqd_get_trigger_type(desc);

	ret = bmi260_config_device_irq(indio_dev, irq_type, pin);
	if (ret)
		return ret;

	return bmi260_probe_trigger(indio_dev, irq, irq_type);
}

static int bmi260_chip_init(struct bmi260_data *data)
{
	int ret;
	unsigned int val;
	struct device *dev = regmap_get_device(data->regmap);

	ret = regulator_bulk_enable(ARRAY_SIZE(data->supplies), data->supplies);
	if (ret) {
		dev_err(dev, "Failed to enable regulators: %d\n", ret);
		return ret;
	}

	ret = regmap_write(data->regmap, BMI260_REG_CMD, BMI260_CMD_SOFTRESET);
	if (ret)
		goto disable_regulator;

	usleep_range(BMI260_SOFTRESET_USLEEP, BMI260_SOFTRESET_USLEEP + 1);

	ret = regmap_read(data->regmap, BMI260_REG_CHIP_ID, &val);
	if (ret) {
		dev_err(dev, "Error reading chip id\n");
		goto disable_regulator;
	}
	if (val != BMI260_CHIP_ID_VAL) {
		dev_err(dev, "Wrong chip id, got %x expected %x\n",
			val, BMI260_CHIP_ID_VAL);
		ret = -ENODEV;
		goto disable_regulator;
	}

	ret = bmi260_write_conf_reg(data->regmap, BMI260_REG_PWR_CONF,
				    BMI260_PWR_CONF_ADV_PWR_SAVE, false,
				    BMI260_SUSPENDED_WRITE_USLEEP);
	if (ret) {
		dev_err(dev, "Error disabling advanced power saving\n");
		goto disable_regulator;
	}

	/* Upload the config file */
	ret = regmap_write(data->regmap, BMI260_REG_INIT_CTRL, 0);
	if (ret) {
		dev_err(dev, "Error preparing for config upload\n");
		goto disable_regulator;
	}

	ret = regmap_raw_write(data->regmap, BMI260_REG_INIT_DATA, bmi260_config_file, ARRAY_SIZE(bmi260_config_file));
	if (ret) {
		dev_err(dev, "Error uploading config\n");
		goto disable_regulator;
	}

	ret = regmap_write(data->regmap, BMI260_REG_INIT_CTRL, 1);
	if (ret) {
		dev_err(dev, "Error finalizing config upload\n");
		goto disable_regulator;
	}

	usleep_range(BMI260_INIT_USLEEP, BMI260_INIT_USLEEP + 1);

	ret = regmap_read(data->regmap, BMI260_REG_INTERNAL_STATUS, &val);
	if (ret) {
		dev_err(dev, "Error reading chip status\n");
		goto disable_regulator;
	}
	if ((val & BMI260_STATUS_MESSAGE_MASK) != 0x01) {
		dev_err(dev, "Chip failed to init\n");
		ret = -ENODEV;
		goto disable_regulator;
	}

	/* Enable accel and gyro */
	ret = regmap_update_bits(data->regmap, BMI260_REG_PWR_CTRL,
				 BMI260_PWR_CTRL_ACC_EN | BMI260_PWR_CTRL_GYR_EN,
				 BMI260_PWR_CTRL_ACC_EN | BMI260_PWR_CTRL_GYR_EN);
	if (ret)
		goto disable_regulator;

	return 0;

disable_regulator:
	regulator_bulk_disable(ARRAY_SIZE(data->supplies), data->supplies);
	return ret;
}

static int bmi260_data_rdy_trigger_set_state(struct iio_trigger *trig,
					     bool enable)
{
	struct iio_dev *indio_dev = iio_trigger_get_drvdata(trig);
	struct bmi260_data *data = iio_priv(indio_dev);

	return bmi260_enable_irq(data->regmap, data->int_pin, enable);
}
static const struct iio_trigger_ops bmi260_trigger_ops = {
	.set_trigger_state = &bmi260_data_rdy_trigger_set_state,
};

int bmi260_probe_trigger(struct iio_dev *indio_dev, int irq, u32 irq_type)
{
	struct bmi260_data *data = iio_priv(indio_dev);
	int ret;

	data->trig = devm_iio_trigger_alloc(&indio_dev->dev, "%s-dev%d",
					    indio_dev->name,
					    iio_device_id(indio_dev));

	if (data->trig == NULL)
		return -ENOMEM;

	ret = devm_request_irq(&indio_dev->dev, irq,
			       &iio_trigger_generic_data_rdy_poll,
			       irq_type, "bmi260", data->trig);
	if (ret)
		return ret;

	data->trig->dev.parent = regmap_get_device(data->regmap);
	data->trig->ops = &bmi260_trigger_ops;
	iio_trigger_set_drvdata(data->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, data->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(data->trig);

	return 0;
}

static void bmi260_chip_uninit(void *data)
{
	struct bmi260_data *bmi_data = data;
	struct device *dev = regmap_get_device(bmi_data->regmap);
	int ret;

	/* Disable accel and gyro */
	regmap_update_bits(bmi_data->regmap, BMI260_REG_PWR_CTRL,
			   BMI260_PWR_CTRL_ACC_EN | BMI260_PWR_CTRL_GYR_EN,
			   0);

	ret = regulator_bulk_disable(ARRAY_SIZE(bmi_data->supplies),
				     bmi_data->supplies);
	if (ret)
		dev_err(dev, "Failed to disable regulators: %d\n", ret);
}

int bmi260_core_probe(struct device *dev, struct regmap *regmap,
		      int irq, const char *name)
{
	struct iio_dev *indio_dev;
	struct bmi260_data *data;
	enum bmi260_int_pin int_pin = BMI260_PIN_INT1;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);
	data->regmap = regmap;

	data->supplies[0].supply = "vdd";
	data->supplies[1].supply = "vddio";
	ret = devm_regulator_bulk_get(dev,
				      ARRAY_SIZE(data->supplies),
				      data->supplies);
	if (ret) {
		dev_err(dev, "Failed to get regulators: %d\n", ret);
		return ret;
	}

	if (!bmi260_apply_acpi_orientation(dev, &data->orientation)) {
		ret = iio_read_mount_matrix(dev, &data->orientation);
		if (ret)
			return ret;
	}

	ret = bmi260_chip_init(data);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, bmi260_chip_uninit, data);
	if (ret)
		return ret;

	indio_dev->channels = bmi260_channels;
	indio_dev->num_channels = ARRAY_SIZE(bmi260_channels);
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &bmi260_info;

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      bmi260_trigger_handler, NULL);
	if (ret)
		return ret;

	if (!irq) {
		irq = bmi260_get_irq(dev_fwnode(dev), &int_pin);
	}

	if (irq > 0) {
		data->int_pin = int_pin;
		ret = bmi260_setup_irq(indio_dev, irq, int_pin);
		if (ret)
			dev_err(&indio_dev->dev, "Failed to setup IRQ %d\n",
				irq);
	} else {
		dev_info(&indio_dev->dev, "Not setting up IRQ trigger\n");
	}

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_NS_GPL(bmi260_core_probe, IIO_BMI260);

MODULE_AUTHOR("Justin Weiss <justin@justinweiss.com>");
MODULE_DESCRIPTION("Bosch BMI260 driver");
MODULE_LICENSE("GPL v2");
