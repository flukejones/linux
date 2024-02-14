// SPDX-License-Identifier: GPL-2.0
/*
 * I2C driver for Bosch BMI260 IMU.
 *
 * Copyright (C) 2023, Justin Weiss
 *
 * This driver is also based on the BMI160 driver, which is:
 * Copyright (c) 2016, Intel Corporation.
 * Copyright (c) 2019, Martin Kelly.
 */
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/version.h>

#include "bmi260.h"

static int bmi260_i2c_probe(struct i2c_client *client)
{
	const struct i2c_device_id *id = i2c_client_get_device_id(client);
	struct regmap *regmap;
	const char *name;

	regmap = devm_regmap_init_i2c(client, &bmi260_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&client->dev, "Failed to register i2c regmap: %pe\n",
			regmap);
		return PTR_ERR(regmap);
	}

	if (id)
		name = id->name;
	else
		name = dev_name(&client->dev);

	return bmi260_core_probe(&client->dev, regmap, client->irq, name);
}

static const struct i2c_device_id bmi260_i2c_id[] = {
	{"bmi260", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, bmi260_i2c_id);

static const struct acpi_device_id bmi260_acpi_match[] = {
	{"BOSC0260", 0},
	{"BMI0260", 0},
	{"BOSC0160", 0},
	{"BMI0160", 0},
	{"10EC5280", 0},
	{ },
};
MODULE_DEVICE_TABLE(acpi, bmi260_acpi_match);

static const struct of_device_id bmi260_of_match[] = {
	{ .compatible = "bosch,bmi260" },
	{ },
};
MODULE_DEVICE_TABLE(of, bmi260_of_match);

static struct i2c_driver bmi260_i2c_driver = {
	.driver = {
		.name			= "bmi260_i2c",
		.acpi_match_table	= bmi260_acpi_match,
		.of_match_table		= bmi260_of_match,
	},
	.probe		= bmi260_i2c_probe,
	.id_table	= bmi260_i2c_id,
};
module_i2c_driver(bmi260_i2c_driver);

MODULE_AUTHOR("Justin Weiss <justin@justinweiss.com>");
MODULE_DESCRIPTION("BMI260 I2C driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_BMI260);
