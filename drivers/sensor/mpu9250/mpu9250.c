/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_mpu9250

#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>

#include "mpu9250.h"

#ifdef CONFIG_MPU9250_MAGN_EN
#include "ak8963.h"
#endif

LOG_MODULE_REGISTER(MPU9250, CONFIG_SENSOR_LOG_LEVEL);

#define MPU9250_REG_CHIP_ID 0x75
#define MPU9250_CHIP_ID 0x71

#define MPU9250_REG_SR_DIV 0x19

#define MPU9250_REG_CONFIG 0x1A
#define MPU9250_GYRO_DLPF_MAX 7

#define MPU9250_REG_GYRO_CFG 0x1B
#define MPU9250_GYRO_FS_SHIFT 3
#define MPU9250_GYRO_FS_MAX 3

#define MPU9250_REG_ACCEL_CFG		0x1C
#define MPU9250_ACCEL_FS_SHIFT		3
#define MPU9250_ACCEL_FS_MAX		3

#define MPU9250_REG_ACCEL_CFG2 0x1D
#define MPU9250_ACCEL_DLPF_MAX 7

#define MPU9250_REG_DATA_START 0x3B

#define MPU0259_TEMP_SENSITIVITY 334
#define MPU9250_TEMP_OFFSET 21

#define MPU9250_REG_PWR_MGMT1 0x6B
#define MPU9250_SLEEP_EN BIT(6)
#define MPU9250_RESET BIT(7)

#define MPU9250_REG_PWR_MGMT2 0x6C

#define MPU9250_REG_INT_ENABLE 0x38
#define MPU9250_REG_FIFO_EN 0x23
#define MPU9250_REG_I2C_MST_CTRL 0x24
#define MPU9250_REG_USER_CTRL 0x6A
#define MPU9250_REG_FIFO_COUNTH 0x72
#define MPU9250_REG_FIFO_COUNTL 0x73
#define MPU9250_REG_FIFO_R_W 0x74
#define MPU9250_REG_XG_OFFSET_H 0x13 // User-defined trim values for gyroscope
#define MPU9250_REG_XG_OFFSET_L 0x14
#define MPU9250_REG_YG_OFFSET_H 0x15
#define MPU9250_REG_YG_OFFSET_L 0x16
#define MPU9250_REG_ZG_OFFSET_H 0x17
#define MPU9250_REG_ZG_OFFSET_L 0x18
#define MPU9250_REG_XA_OFFSET_H 0x77
#define MPU9250_REG_XA_OFFSET_L 0x78
#define MPU9250_REG_YA_OFFSET_H 0x7A
#define MPU9250_REG_YA_OFFSET_L 0x7B
#define MPU9250_REG_ZA_OFFSET_H 0x7D
#define MPU9250_REG_ZA_OFFSET_L 0x7E

#ifdef CONFIG_MPU9250_MAGN_EN
#define MPU9250_READ_BUF_SIZE 11
#else
#define MPU9250_READ_BUF_SIZE 7
#endif

/* see "Accelerometer Measurements" section from register map description */
static void mpu9250_convert_accel(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void mpu9250_convert_gyro(struct sensor_value *val, int16_t raw_val,
				 uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void mpu9250_convert_temp(struct sensor_value *val,
					int16_t raw_val)
{
	/* Temp[*C] = (raw / sensitivity) + offset */
	val->val1 = (raw_val / MPU0259_TEMP_SENSITIVITY) + MPU9250_TEMP_OFFSET;
	val->val2 = (((int64_t)(raw_val % MPU0259_TEMP_SENSITIVITY) * 1000000)
				/ MPU0259_TEMP_SENSITIVITY);

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int mpu9250_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mpu9250_data *drv_data = dev->data;
#ifdef CONFIG_MPU9250_MAGN_EN
	int ret;
#endif

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		mpu9250_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		mpu9250_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		mpu9250_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		mpu9250_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		mpu9250_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		mpu9250_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		mpu9250_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		mpu9250_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		mpu9250_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		mpu9250_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		mpu9250_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		mpu9250_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
#ifdef CONFIG_MPU9250_MAGN_EN
	case SENSOR_CHAN_MAGN_XYZ:
		ret = ak8963_convert_magn(val, drv_data->magn_x,
					  drv_data->magn_scale_x,
					  drv_data->magn_st2);
		if (ret < 0) {
			return ret;
		}
		ret = ak8963_convert_magn(val + 1, drv_data->magn_y,
					  drv_data->magn_scale_y,
					  drv_data->magn_st2);
		if (ret < 0) {
			return ret;
		}
		ret = ak8963_convert_magn(val + 2, drv_data->magn_z,
					  drv_data->magn_scale_z,
					  drv_data->magn_st2);
		return ret;
	case SENSOR_CHAN_MAGN_X:
		return ak8963_convert_magn(val, drv_data->magn_x,
				    drv_data->magn_scale_x,
				    drv_data->magn_st2);
	case SENSOR_CHAN_MAGN_Y:
		return ak8963_convert_magn(val, drv_data->magn_y,
				    drv_data->magn_scale_y,
				    drv_data->magn_st2);
	case SENSOR_CHAN_MAGN_Z:
		return ak8963_convert_magn(val, drv_data->magn_z,
				    drv_data->magn_scale_z,
				    drv_data->magn_st2);
	case SENSOR_CHAN_DIE_TEMP:
		mpu9250_convert_temp(val, drv_data->temp);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mpu9250_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	int16_t buf[MPU9250_READ_BUF_SIZE];
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c,
				MPU9250_REG_DATA_START, (uint8_t *)buf,
				sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read data sample.");
		return ret;
	}

	drv_data->accel_x = sys_be16_to_cpu(buf[0]);
	drv_data->accel_y = sys_be16_to_cpu(buf[1]);
	drv_data->accel_z = sys_be16_to_cpu(buf[2]);
	drv_data->temp = sys_be16_to_cpu(buf[3]);
	drv_data->gyro_x = sys_be16_to_cpu(buf[4]);
	drv_data->gyro_y = sys_be16_to_cpu(buf[5]);
	drv_data->gyro_z = sys_be16_to_cpu(buf[6]);
#ifdef CONFIG_MPU9250_MAGN_EN
	drv_data->magn_x = sys_be16_to_cpu(buf[7]);
	drv_data->magn_y = sys_be16_to_cpu(buf[8]);
	drv_data->magn_z = sys_be16_to_cpu(buf[9]);
	drv_data->magn_st2 = ((uint8_t *)buf)[20];
	LOG_DBG("magn_st2: %u", drv_data->magn_st2);
#endif

	return 0;
}

static int mpu9250_calibrate(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;

	uint8_t calibData[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	int ret = i2c_reg_update_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, MPU9250_RESET, 1);
	if (ret < 0) {
		LOG_ERR("Failed to reset.");
		return ret;
	}

	k_sleep(K_MSEC(100));

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, 0x01);
	if (ret < 0) {
		LOG_ERR("Failed to set PLL1.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT2, 0x00);
	if (ret < 0) {
		LOG_ERR("Failed to set PLL2.");
		return ret;
	}

	k_sleep(K_MSEC(200));

	// Configure device for bias calculation
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_INT_ENABLE,
				    0x00); // Disable all interrupts
	if (ret < 0) {
		LOG_ERR("Failed to Disable all interrupts.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_FIFO_EN, 0x00); // Disable FIFO
	if (ret < 0) {
		LOG_ERR("Failed to Disable FIFO");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1,
				    0x00); // Turn on internal clock source
	if (ret < 0) {
		LOG_ERR("Failed to Turn on internal clock source.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_I2C_MST_CTRL,
				    0x00); // Disable I2C master
	if (ret < 0) {
		LOG_ERR("Failed to Disable I2C master.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_USER_CTRL,
				    0x00); // Disable FIFO and I2C master modes
	if (ret < 0) {
		LOG_ERR("Failed to Disable FIFO and I2C master modes.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_USER_CTRL, 0x0C); // Reset FIFO and DMP
	if (ret < 0) {
		LOG_ERR("Failed to Reset FIFO and DMP.");
		return ret;
	}

	k_sleep(K_MSEC(15));

	// Configure MPU6050 gyro and accelerometer for bias calculation
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_CONFIG,
				    0x01); // Set low-pass filter to 188 Hz
	if (ret < 0) {
		LOG_ERR("Failed to Set low-pass filter to 188 Hz.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_SR_DIV,
				    0x00); // Set sample rate to 1 kHz
	if (ret < 0) {
		LOG_ERR("Failed to Set sample rate to 1 kHz.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(
		&cfg->i2c, MPU9250_REG_GYRO_CFG,
		0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	if (ret < 0) {
		LOG_ERR("Failed to Set gyro full-scale to 250 degrees per second, maximum sensitivity.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(
		&cfg->i2c, MPU9250_REG_ACCEL_CFG,
		0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
	if (ret < 0) {
		LOG_ERR("Failed to Set accelerometer full-scale to 2 g, maximum sensitivity.");
		return ret;
	}

	uint16_t gyrosensitivity = 131; // = 131 LSB/degrees/sec
	uint16_t accelsensitivity = 16384; // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_USER_CTRL, 0x40); // Enable FIFO
	if (ret < 0) {
		LOG_ERR("Failed to Enable FIFO.");
		return ret;
	}
	ret = i2c_reg_write_byte_dt(
		&cfg->i2c, MPU9250_REG_FIFO_EN,
		0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
	if (ret < 0) {
		LOG_ERR("Failed to Enable gyro and accelerometer sensors for FIFO.");
		return ret;
	}
	k_sleep(K_MSEC(40)); // accumulate 40 samples in 40 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_FIFO_EN,
				    0x00); // Disable gyro and accelerometer sensors for FIFO
	if (ret < 0) {
		LOG_ERR("Failed to Disable gyro and accelerometer sensors for FIFO.");
		return ret;
	}
	ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_FIFO_COUNTH, &calibData[0],
				2); // read FIFO sample count
	if (ret < 0) {
		LOG_ERR("Failed to read FIFO sample count.");
		return ret;
	}

	fifo_count = ((uint16_t)calibData[0] << 8) | calibData[1];
	packet_count =
		fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_FIFO_R_W, &calibData[0], 12);
		if (ret < 0) {
			LOG_ERR("Failed to read calib data from FIFO.");
			return ret;
		}

		//Form signed 16-bit integer for each sample in FIFO
		accel_temp[0] = (int16_t)(((int16_t)calibData[0] << 8) | calibData[1]);
		accel_temp[1] = (int16_t)(((int16_t)calibData[2] << 8) | calibData[3]);
		accel_temp[2] = (int16_t)(((int16_t)calibData[4] << 8) | calibData[5]);
		gyro_temp[0] = (int16_t)(((int16_t)calibData[6] << 8) | calibData[7]);
		gyro_temp[1] = (int16_t)(((int16_t)calibData[8] << 8) | calibData[9]);
		gyro_temp[2] = (int16_t)(((int16_t)calibData[10] << 8) | calibData[11]);

		//Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[0] += (int32_t)accel_temp[0];
		accel_bias[1] += (int32_t)accel_temp[1];
		accel_bias[2] += (int32_t)accel_temp[2];
		gyro_bias[0] += (int32_t)gyro_temp[0];
		gyro_bias[1] += (int32_t)gyro_temp[1];
		gyro_bias[2] += (int32_t)gyro_temp[2];
	}

	//Normalize sums to get average count biases
	accel_bias[0] /= (int32_t)packet_count;
	accel_bias[1] /= (int32_t)packet_count;
	accel_bias[2] /= (int32_t)packet_count;
	gyro_bias[0] /= (int32_t)packet_count;
	gyro_bias[1] /= (int32_t)packet_count;
	gyro_bias[2] /= (int32_t)packet_count;

	//Remove gravity from the z-axis accelerometer bias calculation
	if (accel_bias[2] > 0L) {
		accel_bias[2] -= (int32_t)accelsensitivity;
	} else {
		accel_bias[2] += (int32_t)accelsensitivity;
	}

	//Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	calibData[0] =
		(-gyro_bias[0] / 4 >> 8) &
		0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	calibData[1] =
		(-gyro_bias[0] / 4) &
		0xFF; // Biases are additive, so change sign on calculated average gyro biases
	calibData[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
	calibData[3] = (-gyro_bias[1] / 4) & 0xFF;
	calibData[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
	calibData[5] = (-gyro_bias[2] / 4) & 0xFF;

	//Push gyro biases to hardware registers
	ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_XG_OFFSET_H, &calibData[0], 6);
	if (ret < 0) {
		LOG_ERR("Failed to Push gyro biases to hardware registers.");
		return ret;
	}

	drv_data->gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
	drv_data->gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
	drv_data->gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

	LOG_DBG("Gyro bias X: %f, Gyro bias Y: %f, Gyro bias Z: %f", drv_data->gyroBias[0],
		drv_data->gyroBias[1], drv_data->gyroBias[2]);

	//Construct the accelerometer biases for push to the hardware accelerometer bias registers.
	int32_t accel_bias_reg[3] = { 0, 0,
				      0 }; //A place to hold the factory accelerometer trim biases

	ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_H, &calibData[0],
				2); //Read factory accelerometer trim values
	if (ret < 0) {
		LOG_ERR("Failed to Read factory accelerometer trim values.");
		return ret;
	}
	accel_bias_reg[0] = (int32_t)(((int16_t)calibData[0] << 8) | calibData[1]);

	ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_YA_OFFSET_H, &calibData[0], 2);
	if (ret < 0) {
		LOG_ERR("Failed to Read factory accelerometer trim values.");
		return ret;
	}
	accel_bias_reg[1] = (int32_t)(((int16_t)calibData[0] << 8) | calibData[1]);

	ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_ZA_OFFSET_H, &calibData[0], 2);
	if (ret < 0) {
		LOG_ERR("Failed to Read factory accelerometer trim values.");
		return ret;
	}
	accel_bias_reg[2] = (int32_t)(((int16_t)calibData[0] << 8) | calibData[1]);

	//Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint32_t mask = 1uL;
	//Define array to hold mask bit for each accelerometer bias axis
	uint8_t mask_bit[3] = { 0, 0, 0 };

	for (ii = 0; ii < 3; ii++) {
		//If temperature compensation bit is set, record that fact in mask_bit
		if ((accel_bias_reg[ii] & mask))
			mask_bit[ii] = 0x01;
	}

	//Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -=
		(accel_bias[0] /
		 8); //Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	calibData[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	calibData[1] = (accel_bias_reg[0]) & 0xFF;
	calibData[1] =
		calibData[1] |
		mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	calibData[3] = (accel_bias_reg[1]) & 0xFF;
	calibData[3] =
		calibData[3] |
		mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
	calibData[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	calibData[5] = (accel_bias_reg[2]) & 0xFF;
	calibData[5] =
		calibData[5] |
		mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

	//Push accelerometer biases to hardware registers
	ret = i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_H, &calibData[0], 6);
	if (ret < 0) {
		LOG_ERR("Failed to Push accelerometer biases to hardware registers.");
		return ret;
	}

	//Output scaled gyro biases for display in the main program
	drv_data->accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
	drv_data->accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
	drv_data->accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;

	LOG_DBG("Accel bias X: %f, Accel bias Y: %f, Accel bias Z: %f", drv_data->accelBias[0],
		drv_data->accelBias[1], drv_data->accelBias[2]);

	return 0;
}

static const struct sensor_driver_api mpu9250_driver_api = {
#if CONFIG_MPU9250_TRIGGER
	.trigger_set = mpu9250_trigger_set,
#endif
	.sample_fetch = mpu9250_sample_fetch,
	.channel_get = mpu9250_channel_get,
};

/* measured in degrees/sec x10 to avoid floating point */
static const uint16_t mpu9250_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

static int mpu9250_init(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	uint8_t id;
	int ret;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	/* check chip ID */
	ret = i2c_reg_read_byte_dt(&cfg->i2c, MPU9250_REG_CHIP_ID, &id);
	if (ret < 0) {
		LOG_ERR("Failed to read chip ID.");
		return ret;
	}

	if (id != MPU9250_CHIP_ID) {
		LOG_ERR("Invalid chip ID.");
		return -ENOTSUP;
	}

	// TODO: calibrate
	ret = mpu9250_calibrate(dev);
	if (ret < 0) {
		LOG_ERR("Failed to calibrate.");
	}

	/* wake up chip */
	ret = i2c_reg_update_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, MPU9250_SLEEP_EN, 0);
	if (ret < 0) {
		LOG_ERR("Failed to wake up chip.");
		return ret;
	}

	if (cfg->accel_fs > MPU9250_ACCEL_FS_MAX) {
		LOG_ERR("Accel FS is too big: %d", cfg->accel_fs);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ACCEL_CFG,
				    cfg->accel_fs << MPU9250_ACCEL_FS_SHIFT);
	if (ret < 0) {
		LOG_ERR("Failed to write accel full-scale range.");
		return ret;
	}
	drv_data->accel_sensitivity_shift = 14 - cfg->accel_fs;

	if (cfg->gyro_fs > MPU9250_GYRO_FS_MAX) {
		LOG_ERR("Gyro FS is too big: %d", cfg->accel_fs);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_GYRO_CFG,
				    cfg->gyro_fs << MPU9250_GYRO_FS_SHIFT);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro full-scale range.");
		return ret;
	}

	if (cfg->gyro_dlpf > MPU9250_GYRO_DLPF_MAX) {
		LOG_ERR("Gyro DLPF is too big: %d", cfg->gyro_dlpf);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_CONFIG,
				    cfg->gyro_dlpf);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro digital LPF settings.");
		return ret;
	}

	if (cfg->accel_dlpf > MPU9250_ACCEL_DLPF_MAX) {
		LOG_ERR("Accel DLPF is too big: %d", cfg->accel_dlpf);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ACCEL_CFG2,
				    cfg->gyro_dlpf);
	if (ret < 0) {
		LOG_ERR("Failed to write accel digital LPF settings.");
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_SR_DIV,
				    cfg->gyro_sr_div);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro ODR divider.");
		return ret;
	}

	drv_data->gyro_sensitivity_x10 =
				mpu9250_gyro_sensitivity_x10[cfg->gyro_fs];

#ifdef CONFIG_MPU9250_MAGN_EN
	ret = ak8963_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize AK8963.");
		return ret;
	}
#endif

#ifdef CONFIG_MPU9250_TRIGGER
	ret = mpu9250_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return ret;
	}
#endif

	return 0;
}


#define INIT_MPU9250_INST(inst)						\
	static struct mpu9250_data mpu9250_data_##inst;			\
	static const struct mpu9250_config mpu9250_cfg_##inst = {	\
	.i2c = I2C_DT_SPEC_INST_GET(inst),				\
	.gyro_sr_div = DT_INST_PROP(inst, gyro_sr_div),			\
	.gyro_dlpf = DT_INST_ENUM_IDX(inst, gyro_dlpf),			\
	.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs),			\
	.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),			\
	.accel_dlpf = DT_INST_ENUM_IDX(inst, accel_dlpf),		\
	IF_ENABLED(CONFIG_MPU9250_TRIGGER,				\
		  (.int_pin = GPIO_DT_SPEC_INST_GET(inst, irq_gpios)))	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, mpu9250_init, NULL,			\
			      &mpu9250_data_##inst, &mpu9250_cfg_##inst,\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
			      &mpu9250_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_MPU9250_INST)
