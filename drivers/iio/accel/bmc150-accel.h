/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _BMC150_ACCEL_H_
#define _BMC150_ACCEL_H_

#include <linux/atomic.h>
#include <linux/iio/iio.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

/*
 * the bmi323 needs raw access to spi and i2c: I cannot use regmap
 * as this device expects i2c writes to be 2 bytes,
 * spi reads to be 3 bytes and i2c reads to be 4 bytes.
 */
#include <linux/i2c.h>
#include <linux/spi/spi.h>

struct regmap;
struct i2c_client;
struct bmc150_accel_chip_info;
struct bmc150_accel_interrupt_info;

/*
 * We can often guess better than "UNKNOWN" based on the device IDs
 * but unfortunately this information is not always accurate. There are some
 * devices where ACPI firmware specifies an ID like "BMA250E" when the device
 * actually has a BMA222E. The driver attempts to detect those by reading the
 * chip ID from the registers but this information is not always enough either.
 *
 * Therefore, this enum should be only used when the chip ID detection is not
 * enough and we can be reasonably sure that the device IDs are reliable
 * in practice (e.g. for device tree platforms).
 */
enum bmc150_type {
	BOSCH_UNKNOWN,
	BOSCH_BMC156,
};

struct bmc150_accel_interrupt {
	const struct bmc150_accel_interrupt_info *info;
	atomic_t users;
};

enum bmc150_device_type {
	BMC150,
	BMI323,
};

struct bmc150_accel_trigger {
	struct bmc150_accel_data *data;
	struct iio_trigger *indio_trig;
	int (*setup)(struct bmc150_accel_trigger *t, bool state);
	int intr;
	bool enabled;
};

enum bmc150_accel_interrupt_id {
	BMC150_ACCEL_INT_DATA_READY,
	BMC150_ACCEL_INT_ANY_MOTION,
	BMC150_ACCEL_INT_WATERMARK,
	BMC150_ACCEL_INTERRUPTS,
};

enum bmc150_accel_trigger_id {
	BMC150_ACCEL_TRIGGER_DATA_READY,
	BMC150_ACCEL_TRIGGER_ANY_MOTION,
	BMC150_ACCEL_TRIGGERS,
};

#define BMI323_FLAGS_RESET_FAILED 0x00000001U

struct bmi323_private_data {
	struct regulator_bulk_data regulators[2];
	struct i2c_client *i2c_client;
	struct spi_device *spi_client;
	struct device *dev; /* pointer at i2c_client->dev or spi_client->dev */
	struct mutex mutex;
	int irq;
	u32 flags;
	u16 acc_conf_reg_value;
	u16 gyr_conf_reg_value;
	u16 fifo_conf_reg_value;
	struct iio_trigger *trig[1];
	s64 fifo_frame_time_diff_ns;
	s64 acc_odr_time_ns;
	s64 gyr_odr_time_ns;
};

struct bmc150_accel_data {
	struct regmap *regmap;
	struct regulator_bulk_data regulators[2];
	struct bmc150_accel_interrupt interrupts[BMC150_ACCEL_INTERRUPTS];
	struct bmc150_accel_trigger triggers[BMC150_ACCEL_TRIGGERS];
	struct mutex mutex;
	u8 fifo_mode, watermark;
	s16 buffer[8];
	/*
	 * Ensure there is sufficient space and correct alignment for
	 * the timestamp if enabled
	 */
	struct {
		__le16 channels[3];
		s64 ts __aligned(8);
	} scan;
	u8 bw_bits;
	u32 slope_dur;
	u32 slope_thres;
	u32 range;
	int ev_enable_state;
	int64_t timestamp, old_timestamp; /* Only used in hw fifo mode. */
	const struct bmc150_accel_chip_info *chip_info;
	enum bmc150_type type;
	struct i2c_client *second_device;
	void (*resume_callback)(struct device *dev);
	struct delayed_work resume_work;
	struct iio_mount_matrix orientation;
	enum bmc150_device_type dev_type;
	struct bmi323_private_data bmi323;
	};

/**
 * This function performs a write of a u16 little-endian (regardless of CPU architecture) integer
 * to a device register. Returns 0 on success or an error code otherwise.
 *
 * PRE: in_value holds the data to be sent to the sensor, in little endian format even on big endian
 *      architectures.
 *
 * NOTE: bmi323->dev can be NULL (not yet initialized) when this function is called
 *			therefore it is not needed and is not used inside the function
 *
 * WARNING: this function does not lock any mutex and synchronization MUST be performed by the caller
 */
int bmi323_write_u16(struct bmi323_private_data *bmi323, u8 in_reg, u16 in_value);

/**
 * This function performs a read of "good" values from the bmi323 discarding what
 * in the datasheet is described as "dummy data": additional useles bytes.
 *
 * PRE: bmi323 has been partially initialized: i2c_device and spi_devices MUST be set to either
 *      the correct value or NULL
 *
 * NOTE: bmi323->dev can be NULL (not yet initialized) when this function is called
 *			therefore it is not needed and is not used inside the function
 *
 * POST: on success out_value is written with data from the sensor, as it came out, so the
 *       content is little-endian even on big endian architectures
 *
 * WARNING: this function does not lock any mutex and synchronization MUST be performed by the caller
 */
int bmi323_read_u16(struct bmi323_private_data *bmi323, u8 in_reg, u16* out_value);

int bmi323_chip_check(struct bmi323_private_data *bmi323);

/**
 * Reset the chip in a known state that is ready to accept commands, but is not configured therefore after calling this function
 * it is required to load a new configuration to start data acquisition.
 *
 * PRE: bmi323 has been fully identified and partially initialized
 *
 * NOTE: after issuing a reset the the chip will be in what it is called "suspended mode" and the feature angine is
 * ready to be set. This mode has everything disabled and consumes aroud 15uA.
 *
 * When removing the driver or suspend has been requested it's best to reset the chip so that power consumption
 * will be the lowest possible.
 */
int bmi323_chip_rst(struct bmi323_private_data *bmi323);

/**
 * This function MUST be called in probe and is responsible for registering the userspace sysfs.
 *
 * The indio_dev MUST have been allocated but not registered. This function will perform userspace registration.
 *
 * @param indio_dev the industrual io device already allocated but not yet registered
 */
int bmi323_iio_init(struct iio_dev *indio_dev);

void bmi323_iio_deinit(struct iio_dev *indio_dev);

int bmc150_accel_core_probe(struct device *dev, struct regmap *regmap, int irq,
			    enum bmc150_type type, const char *name,
			    bool block_supported);
void bmc150_accel_core_remove(struct device *dev);
extern const struct dev_pm_ops bmc150_accel_pm_ops;
extern const struct regmap_config bmc150_regmap_conf;

#endif  /* _BMC150_ACCEL_H_ */
