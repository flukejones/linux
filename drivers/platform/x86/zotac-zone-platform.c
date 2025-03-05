// SPDX-License-Identifier: GPL-2.0
/*
 * Zotac Handheld Platform Driver
 *
 * Copyright (C) 2025 Luke D. Jones
 */

#include "linux/mod_devicetable.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mutex.h>
#include <linux/dmi.h>
#include <linux/err.h>
#include <linux/acpi.h>
#include <linux/ioport.h>
#include <linux/jiffies.h>

#define DRIVER_NAME "zotac_zone_platform"

#define EC_COMMAND_PORT 0x4E
#define EC_DATA_PORT 0x4F

#define EC_FAN_CTRL_ADDR 0x44A
#define EC_FAN_DUTY_ADDR 0x44B
#define EC_FAN_SPEED_UPPER_ADDR 0x476
#define EC_FAN_SPEED_LOWER_ADDR 0x477
#define EC_CPU_TEMP_ADDR 0x462

#define EC_FAN_MODE_AUTO 0
#define EC_FAN_MODE_MANUAL 1
#define EC_FAN_MODE_CURVE 2

#define EC_FAN_VALUE_MIN 0
#define EC_FAN_VALUE_MAX 255

#define FAN_CURVE_POINTS 9 /* 9 points for 10-90°C like in the Zotac C# code */

static struct timer_list fan_curve_timer;

struct zotac_fan_data {
	struct device *hwmon_dev;
	struct mutex update_lock;
	unsigned int fan_rpm;
	unsigned int fan_duty;
	unsigned int fan_mode;
	unsigned int temp;
	unsigned long last_updated;
	bool valid;
	bool curve_enabled;
	/* Fan curve points */
	unsigned int curve_temp[FAN_CURVE_POINTS]; /* Temperature points */
	unsigned int curve_pwm[FAN_CURVE_POINTS]; /* PWM/duty points */
};

static struct platform_device *zotac_fan_device;
static DEFINE_MUTEX(ec_mutex);
static struct resource ec_io_ports[] = {
	{
		.start = EC_COMMAND_PORT,
		.end = EC_COMMAND_PORT,
		.name = "ec-command",
		.flags = IORESOURCE_IO,
	},
	{
		.start = EC_DATA_PORT,
		.end = EC_DATA_PORT,
		.name = "ec-data",
		.flags = IORESOURCE_IO,
	},
};

static u8 ec_read_byte(u16 addr)
{
	u8 addr_upper = (addr >> 8) & 0xFF;
	u8 addr_lower = addr & 0xFF;
	u8 value;

	mutex_lock(&ec_mutex);

	/* Select upper byte address */
	outb(0x2E, EC_COMMAND_PORT);
	outb(0x11, EC_DATA_PORT);
	outb(0x2F, EC_COMMAND_PORT);
	outb(addr_upper, EC_DATA_PORT);

	/* Select lower byte address */
	outb(0x2E, EC_COMMAND_PORT);
	outb(0x10, EC_DATA_PORT);
	outb(0x2F, EC_COMMAND_PORT);
	outb(addr_lower, EC_DATA_PORT);

	/* Read data */
	outb(0x2E, EC_COMMAND_PORT);
	outb(0x12, EC_DATA_PORT);
	outb(0x2F, EC_COMMAND_PORT);
	value = inb(EC_DATA_PORT);

	mutex_unlock(&ec_mutex);

	return value;
}

static int ec_write_byte(u16 addr, u8 value)
{
	u8 addr_upper = (addr >> 8) & 0xFF;
	u8 addr_lower = addr & 0xFF;

	mutex_lock(&ec_mutex);

	/* Select upper byte address */
	outb(0x2E, EC_COMMAND_PORT);
	outb(0x11, EC_DATA_PORT);
	outb(0x2F, EC_COMMAND_PORT);
	outb(addr_upper, EC_DATA_PORT);

	/* Select lower byte address */
	outb(0x2E, EC_COMMAND_PORT);
	outb(0x10, EC_DATA_PORT);
	outb(0x2F, EC_COMMAND_PORT);
	outb(addr_lower, EC_DATA_PORT);

	/* Write data */
	outb(0x2E, EC_COMMAND_PORT);
	outb(0x12, EC_DATA_PORT);
	outb(0x2F, EC_COMMAND_PORT);
	outb(value, EC_DATA_PORT);

	mutex_unlock(&ec_mutex);

	return 0;
}

static struct zotac_fan_data *zotac_fan_update_device(struct device *dev)
{
	struct zotac_fan_data *data = dev_get_drvdata(dev);
	unsigned long current_time = jiffies;

	if (time_after(current_time, data->last_updated + HZ) || !data->valid) {
		mutex_lock(&data->update_lock);

		data->fan_mode = ec_read_byte(EC_FAN_CTRL_ADDR);
		data->fan_duty = ec_read_byte(EC_FAN_DUTY_ADDR);

		u8 upper = ec_read_byte(EC_FAN_SPEED_UPPER_ADDR);
		u8 lower = ec_read_byte(EC_FAN_SPEED_LOWER_ADDR);
		data->fan_rpm = (upper << 8) | lower;

		data->temp = ec_read_byte(EC_CPU_TEMP_ADDR);

		data->last_updated = current_time;
		data->valid = true;

		mutex_unlock(&data->update_lock);
	}

	return data;
}

/* Internal version of set_fan_duty that doesn't acquire the lock */
static int set_fan_duty_internal(unsigned int duty_percent)
{
	u8 duty_val;

	if (duty_percent > 100)
		return -EINVAL;

	duty_val =
		(duty_percent * (EC_FAN_VALUE_MAX - EC_FAN_VALUE_MIN)) / 100 +
		EC_FAN_VALUE_MIN;
	return ec_write_byte(EC_FAN_DUTY_ADDR, duty_val);
}

static void fan_curve_function(struct timer_list *t)
{
	struct zotac_fan_data *data = platform_get_drvdata(zotac_fan_device);
	unsigned int current_temp;
	unsigned int pwm = 0;
	int i;

	if (!data || !data->curve_enabled) {
		if (data && data->curve_enabled)
			mod_timer(&fan_curve_timer, jiffies + HZ);
		return;
	}

	mutex_lock(&data->update_lock);

	current_temp = ec_read_byte(EC_CPU_TEMP_ADDR);
	data->temp = current_temp;

	pwm = data->curve_pwm[0];

	if (current_temp >= data->curve_temp[FAN_CURVE_POINTS - 1]) {
		/* Above highest temperature point - use maximum PWM */
		pwm = data->curve_pwm[FAN_CURVE_POINTS - 1];
	} else {
		/* Find the temperature range and interpolate */
		for (i = 0; i < FAN_CURVE_POINTS - 1; i++) {
			if (current_temp >= data->curve_temp[i] &&
			    current_temp < data->curve_temp[i + 1]) {
				/* Linear interpolation between points */
				int temp_range = data->curve_temp[i + 1] -
						 data->curve_temp[i];
				int pwm_range = data->curve_pwm[i + 1] -
						data->curve_pwm[i];
				int temp_offset =
					current_temp - data->curve_temp[i];

				if (temp_range > 0) {
					pwm = data->curve_pwm[i] +
					      (pwm_range * temp_offset) /
						      temp_range;
				} else {
					pwm = data->curve_pwm[i];
				}

				break;
			}
		}
	}

	set_fan_duty_internal(pwm);
	mutex_unlock(&data->update_lock);

	mod_timer(&fan_curve_timer, jiffies + HZ);
}

static int set_fan_duty(struct device *dev, u8 duty_percent)
{
	struct zotac_fan_data *data = dev_get_drvdata(dev);
	int err;

	if (duty_percent > 100)
		return -EINVAL;

	mutex_lock(&data->update_lock);
	err = set_fan_duty_internal(duty_percent);
	if (err == 0) {
		u8 duty_val =
			(duty_percent * (EC_FAN_VALUE_MAX - EC_FAN_VALUE_MIN)) /
				100 +
			EC_FAN_VALUE_MIN;
		data->fan_duty = duty_val;
	}
	mutex_unlock(&data->update_lock);

	return err;
}

static int set_fan_mode(struct device *dev, u8 mode)
{
	struct zotac_fan_data *data = dev_get_drvdata(dev);
	int err = 0;

	if (mode == EC_FAN_MODE_CURVE) {
		data->curve_enabled = true;

		err = ec_write_byte(EC_FAN_CTRL_ADDR, EC_FAN_MODE_MANUAL);
		if (err == 0) {
			data->fan_mode = EC_FAN_MODE_MANUAL;
			mod_timer(&fan_curve_timer, jiffies + HZ);
		}
	} else {
		if (data->curve_enabled) {
			data->curve_enabled = false;
			del_timer(&fan_curve_timer);
		}

		/* Set normal auto/manual mode to EC */
		err = ec_write_byte(EC_FAN_CTRL_ADDR, mode);
		if (err == 0)
			data->fan_mode = mode;
	}

	return err;
}

/* Fan speed RPM */
static ssize_t fan1_input_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct zotac_fan_data *data = zotac_fan_update_device(dev);
	return sprintf(buf, "%u\n", data->fan_rpm);
}
static DEVICE_ATTR_RO(fan1_input);

/* Fan mode */
static ssize_t fan1_mode_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct zotac_fan_data *data = zotac_fan_update_device(dev);

	if (data->curve_enabled)
		return sprintf(buf, "%u\n", EC_FAN_MODE_CURVE);
	else
		return sprintf(buf, "%u\n", data->fan_mode);
}

static ssize_t fan1_mode_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	unsigned long mode;
	int err;

	err = kstrtoul(buf, 10, &mode);
	if (err)
		return err;

	if (mode != EC_FAN_MODE_AUTO && mode != EC_FAN_MODE_MANUAL &&
	    mode != EC_FAN_MODE_CURVE)
		return -EINVAL;

	err = set_fan_mode(dev, mode);
	if (err)
		return err;

	return count;
}
static DEVICE_ATTR_RW(fan1_mode);

/* Fan duty cycle (percent) */
static ssize_t fan1_duty_show(struct device *dev, struct device_attribute *attr,
			      char *buf)
{
	struct zotac_fan_data *data = zotac_fan_update_device(dev);
	unsigned int duty_percent =
		((data->fan_duty - EC_FAN_VALUE_MIN) * 100) /
		(EC_FAN_VALUE_MAX - EC_FAN_VALUE_MIN);
	return sprintf(buf, "%u\n", duty_percent);
}

static ssize_t fan1_duty_store(struct device *dev,
			       struct device_attribute *attr, const char *buf,
			       size_t count)
{
	unsigned long duty_percent;
	int err;

	err = kstrtoul(buf, 10, &duty_percent);
	if (err)
		return err;

	if (duty_percent > 100)
		return -EINVAL;

	err = set_fan_duty(dev, duty_percent);
	if (err)
		return err;

	return count;
}
static DEVICE_ATTR_RW(fan1_duty);

/* Macro to generate temperature point attributes */
#define CURVE_TEMP_ATTR(index)                                                \
	static ssize_t pwm1_auto_point##index##_temp_show(                    \
		struct device *dev, struct device_attribute *attr, char *buf) \
	{                                                                     \
		struct zotac_fan_data *data = dev_get_drvdata(dev);           \
		return sprintf(buf, "%u\n", data->curve_temp[index - 1]);     \
	}                                                                     \
                                                                              \
	static ssize_t pwm1_auto_point##index##_temp_store(                   \
		struct device *dev, struct device_attribute *attr,            \
		const char *buf, size_t count)                                \
	{                                                                     \
		struct zotac_fan_data *data = dev_get_drvdata(dev);           \
		unsigned long temp;                                           \
		int err;                                                      \
                                                                              \
		err = kstrtoul(buf, 10, &temp);                               \
		if (err)                                                      \
			return err;                                           \
                                                                              \
		mutex_lock(&data->update_lock);                               \
		data->curve_temp[index - 1] = temp;                           \
		mutex_unlock(&data->update_lock);                             \
                                                                              \
		return count;                                                 \
	}                                                                     \
	static DEVICE_ATTR_RW(pwm1_auto_point##index##_temp)

/* Macro to generate PWM point attributes */
#define CURVE_PWM_ATTR(index)                                                 \
	static ssize_t pwm1_auto_point##index##_pwm_show(                     \
		struct device *dev, struct device_attribute *attr, char *buf) \
	{                                                                     \
		struct zotac_fan_data *data = dev_get_drvdata(dev);           \
		return sprintf(buf, "%u\n", data->curve_pwm[index - 1]);      \
	}                                                                     \
                                                                              \
	static ssize_t pwm1_auto_point##index##_pwm_store(                    \
		struct device *dev, struct device_attribute *attr,            \
		const char *buf, size_t count)                                \
	{                                                                     \
		struct zotac_fan_data *data = dev_get_drvdata(dev);           \
		unsigned long pwm;                                            \
		int err;                                                      \
                                                                              \
		err = kstrtoul(buf, 10, &pwm);                                \
		if (err)                                                      \
			return err;                                           \
                                                                              \
		if (pwm > 100)                                                \
			return -EINVAL;                                       \
                                                                              \
		mutex_lock(&data->update_lock);                               \
		data->curve_pwm[index - 1] = pwm;                             \
		mutex_unlock(&data->update_lock);                             \
                                                                              \
		return count;                                                 \
	}                                                                     \
	static DEVICE_ATTR_RW(pwm1_auto_point##index##_pwm)

/* Generate attributes for each point */
CURVE_TEMP_ATTR(1);
CURVE_PWM_ATTR(1);
CURVE_TEMP_ATTR(2);
CURVE_PWM_ATTR(2);
CURVE_TEMP_ATTR(3);
CURVE_PWM_ATTR(3);
CURVE_TEMP_ATTR(4);
CURVE_PWM_ATTR(4);
CURVE_TEMP_ATTR(5);
CURVE_PWM_ATTR(5);
CURVE_TEMP_ATTR(6);
CURVE_PWM_ATTR(6);
CURVE_TEMP_ATTR(7);
CURVE_PWM_ATTR(7);
CURVE_TEMP_ATTR(8);
CURVE_PWM_ATTR(8);
CURVE_TEMP_ATTR(9);
CURVE_PWM_ATTR(9);

/* Temperature reading */
static ssize_t temp1_input_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct zotac_fan_data *data = zotac_fan_update_device(dev);
	return sprintf(buf, "%u\n",
		       data->temp * 1000); /* Convert to milli-degrees */
}
static DEVICE_ATTR_RO(temp1_input);

static struct attribute *zotac_fan_attrs[] = {
	&dev_attr_fan1_input.attr,
	&dev_attr_fan1_mode.attr,
	&dev_attr_fan1_duty.attr,
	&dev_attr_temp1_input.attr,
	&dev_attr_pwm1_auto_point1_temp.attr,
	&dev_attr_pwm1_auto_point1_pwm.attr,
	&dev_attr_pwm1_auto_point2_temp.attr,
	&dev_attr_pwm1_auto_point2_pwm.attr,
	&dev_attr_pwm1_auto_point3_temp.attr,
	&dev_attr_pwm1_auto_point3_pwm.attr,
	&dev_attr_pwm1_auto_point4_temp.attr,
	&dev_attr_pwm1_auto_point4_pwm.attr,
	&dev_attr_pwm1_auto_point5_temp.attr,
	&dev_attr_pwm1_auto_point5_pwm.attr,
	&dev_attr_pwm1_auto_point6_temp.attr,
	&dev_attr_pwm1_auto_point6_pwm.attr,
	&dev_attr_pwm1_auto_point7_temp.attr,
	&dev_attr_pwm1_auto_point7_pwm.attr,
	&dev_attr_pwm1_auto_point8_temp.attr,
	&dev_attr_pwm1_auto_point8_pwm.attr,
	&dev_attr_pwm1_auto_point9_temp.attr,
	&dev_attr_pwm1_auto_point9_pwm.attr,
	NULL
};

ATTRIBUTE_GROUPS(zotac_fan);

static int zotac_fan_probe(struct platform_device *pdev)
{
	struct zotac_fan_data *data;
	struct device *hwmon_dev;
	int i;

	data = devm_kzalloc(&pdev->dev, sizeof(struct zotac_fan_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->valid = false;
	data->curve_enabled = false;
	mutex_init(&data->update_lock);

	for (i = 0; i < FAN_CURVE_POINTS; i++) {
		/* Set default temperature points from 10°C to 90°C */
		data->curve_temp[i] = 10 + (i * 10);
		/* Set default PWM values - simple linear curve from 20% to 100% */
		data->curve_pwm[i] = 20 + (i * 10);
		if (data->curve_pwm[i] > 100)
			data->curve_pwm[i] = 100;
	}

	platform_set_drvdata(pdev, data);

	hwmon_dev = devm_hwmon_device_register_with_groups(
		&pdev->dev, "zotac_platform", data, zotac_fan_groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	data->hwmon_dev = hwmon_dev;

	timer_setup(&fan_curve_timer, fan_curve_function, 0);

	zotac_fan_update_device(&pdev->dev);

	return 0;
}

static struct platform_driver zotac_fan_driver = {
	.driver = {
		.name = DRIVER_NAME,
	},
	.probe = zotac_fan_probe,
};

static const struct dmi_system_id zotac_fan_dmi_table[] __initconst = {
    {
        .ident = "Zotac Gaming Handheld",
        .matches = {
            DMI_MATCH(DMI_SYS_VENDOR, "ZOTAC"),
            DMI_MATCH(DMI_BOARD_NAME, "G0A1W"),
        },
    },
    {
        .ident = "Zotac ZONE",
        .matches = {
            DMI_MATCH(DMI_SYS_VENDOR, "ZOTAC"),
            DMI_MATCH(DMI_PRODUCT_NAME, "ZOTAC GAMING ZONE"),
        },
    },
    {}  /* Terminate list */
};
MODULE_DEVICE_TABLE(dmi, zotac_fan_dmi_table);

static int __init zotac_fan_init(void)
{
	int err;

	if (!dmi_check_system(zotac_fan_dmi_table)) {
		pr_info("No compatible Zotac hardware found\n");
		return -ENODEV;
	}

	/* Request I/O regions */
	if (!request_region(EC_COMMAND_PORT, 1, "zotac_fan_ec") ||
	    !request_region(EC_DATA_PORT, 1, "zotac_fan_ec")) {
		pr_err("Failed to request EC I/O ports\n");
		err = -EBUSY;
		goto err_release;
	}

	zotac_fan_device = platform_device_register_simple(
		DRIVER_NAME, -1, ec_io_ports, ARRAY_SIZE(ec_io_ports));
	if (IS_ERR(zotac_fan_device)) {
		err = PTR_ERR(zotac_fan_device);
		goto err_release;
	}

	err = platform_driver_register(&zotac_fan_driver);
	if (err)
		goto err_device_unregister;

	return 0;

err_device_unregister:
	platform_device_unregister(zotac_fan_device);
err_release:
	release_region(EC_COMMAND_PORT, 1);
	release_region(EC_DATA_PORT, 1);
	return err;
}

static void __exit zotac_fan_exit(void)
{
	del_timer_sync(&fan_curve_timer);

	platform_driver_unregister(&zotac_fan_driver);
	platform_device_unregister(zotac_fan_device);
	release_region(EC_COMMAND_PORT, 1);
	release_region(EC_DATA_PORT, 1);
}

module_init(zotac_fan_init);
module_exit(zotac_fan_exit);

MODULE_AUTHOR("Luke D. Jones");
MODULE_DESCRIPTION("Zotac Handheld Platform Driver");
MODULE_LICENSE("GPL");
