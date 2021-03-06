/*
 *  Copyright (C) 2012, Samsung Electronics Co. Ltd. All Rights Reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 */
#include "ssp.h"
#if defined (CONFIG_SEC_MONTBLANC_PROJECT)
#include <linux/regulator/lp8720.h>
#endif
/*************************************************************************/
/* SSP data delay function                                              */
/*************************************************************************/

unsigned int get_msdelay(int64_t dDelayRate)
{
	if (dDelayRate <= SENSOR_NS_DELAY_FASTEST)
		return SENSOR_MS_DELAY_FASTEST;
	else if (dDelayRate <= SENSOR_NS_DELAY_GAME)
		return SENSOR_MS_DELAY_GAME;
	else if (dDelayRate <= SENSOR_NS_DELAY_UI)
		return SENSOR_MS_DELAY_UI;
	else
		return SENSOR_MS_DELAY_NORMAL;
}

unsigned int get_delay_cmd(u8 uDelayRate)
{
	if (uDelayRate <= SENSOR_MS_DELAY_FASTEST)
		return SENSOR_CMD_DELAY_FASTEST;
	else if (uDelayRate <= SENSOR_MS_DELAY_GAME)
		return SENSOR_CMD_DELAY_GAME;
	else if (uDelayRate <= SENSOR_MS_DELAY_UI)
		return SENSOR_CMD_DELAY_UI;
	else
		return SENSOR_CMD_DELAY_NORMAL;
}

static void change_sensor_delay(struct ssp_data *data,
	int iSensorType, int64_t dNewDelay)
{
	u8 uBuf[2];
	unsigned int uNewEnable = 0;
	int64_t dTempDelay = data->adDelayBuf[iSensorType];

	if (!(atomic_read(&data->aSensorEnable) & (1 << iSensorType))) {
		data->aiCheckStatus[iSensorType] = NO_SENSOR_STATE;
		return;
	}

	data->adDelayBuf[iSensorType] = dNewDelay;

	switch (data->aiCheckStatus[iSensorType]) {
	case ADD_SENSOR_STATE:
		ssp_dbg("[SSP]: %s - add %u, New = %lldns\n",
			 __func__, 1 << iSensorType, dNewDelay);

		uBuf[1] = (u8)get_msdelay(dNewDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);

		if (send_instruction(data, ADD_SENSOR, iSensorType, uBuf, 2)
			!= SUCCESS) {
			uNewEnable =
				(unsigned int)atomic_read(&data->aSensorEnable)
				& (~(unsigned int)(1 << iSensorType));
			atomic_set(&data->aSensorEnable, uNewEnable);

			data->aiCheckStatus[iSensorType] = NO_SENSOR_STATE;
			data->uMissSensorCnt++;
			break;
		}

		data->aiCheckStatus[iSensorType] = RUNNING_SENSOR_STATE;

		if (iSensorType == PROXIMITY_SENSOR) {
			proximity_open_lcd_ldi(data);
			proximity_open_calibration(data);

			input_report_abs(data->prox_input_dev, ABS_DISTANCE, 1);
			input_sync(data->prox_input_dev);
		}
		break;
	case RUNNING_SENSOR_STATE:
		if (get_msdelay(dTempDelay)
			== get_msdelay(data->adDelayBuf[iSensorType]))
			break;

		ssp_dbg("[SSP]: %s - Change %u, New = %lldns\n",
			__func__, 1 << iSensorType, dNewDelay);

		uBuf[1] = (u8)get_msdelay(dNewDelay);
		uBuf[0] = (u8)get_delay_cmd(uBuf[1]);
		send_instruction(data, CHANGE_DELAY, iSensorType, uBuf, 2);

		break;
	default:
		data->aiCheckStatus[iSensorType] = ADD_SENSOR_STATE;
	}
}

/*************************************************************************/
/* SSP data enable function                                              */
/*************************************************************************/

static int ssp_remove_sensor(struct ssp_data *data,
	unsigned int uChangedSensor, unsigned int uNewEnable)
{
	u8 uBuf[2];
	int iRet = 0;
	int64_t dSensorDelay = data->adDelayBuf[uChangedSensor];
	ssp_dbg("[SSP]: %s - remove sensor = %d, current state = %d\n",
		__func__, (1 << uChangedSensor), uNewEnable);

	data->adDelayBuf[uChangedSensor] = DEFUALT_POLLING_DELAY;

	if (data->aiCheckStatus[uChangedSensor] == INITIALIZATION_STATE) {
		data->aiCheckStatus[uChangedSensor] = NO_SENSOR_STATE;
		if (uChangedSensor == ACCELEROMETER_SENSOR) {
			accel_open_calibration(data);
			iRet = set_accel_cal(data);
			if (iRet < 0)
				pr_err("[SSP]: %s - set_accel_cal failed %d\n", __func__, iRet);
		}
		else if (uChangedSensor == GYROSCOPE_SENSOR) {
			gyro_open_calibration(data);
			iRet = set_gyro_cal(data);
			if (iRet < 0)
				pr_err("[SSP]: %s - set_gyro_cal failed %d\n", __func__,  iRet);
		}
		else if (uChangedSensor == PRESSURE_SENSOR)
			pressure_open_calibration(data);
		else if (uChangedSensor == PROXIMITY_SENSOR) {
			proximity_open_lcd_ldi(data);
			proximity_open_calibration(data);
		} else if (uChangedSensor == GEOMAGNETIC_SENSOR) {
			iRet = mag_open_hwoffset(data);
			if (iRet < 0)
				pr_err("[SSP]: %s - mag_open_hw_offset"
				" failed, %d\n", __func__, iRet);

			iRet = set_hw_offset(data);
			if (iRet < 0) {
				pr_err("[SSP]: %s - set_hw_offset failed\n",
					__func__);
			}
		}
		return 0;
	} else if (uChangedSensor == ORIENTATION_SENSOR) {
		if (!(atomic_read(&data->aSensorEnable)
			& (1 << ACCELEROMETER_SENSOR))) {
			uChangedSensor = ACCELEROMETER_SENSOR;
		} else {
			change_sensor_delay(data, ACCELEROMETER_SENSOR,
				data->adDelayBuf[ACCELEROMETER_SENSOR]);
			return 0;
		}
	} else if (uChangedSensor == ACCELEROMETER_SENSOR) {
		if (atomic_read(&data->aSensorEnable)
			& (1 << ORIENTATION_SENSOR)) {
			change_sensor_delay(data, ORIENTATION_SENSOR,
				data->adDelayBuf[ORIENTATION_SENSOR]);
			return 0;
		}
	} else if (uChangedSensor == GEOMAGNETIC_SENSOR) {
		if (mag_store_hwoffset(data))
			pr_err("mag_store_hwoffset success\n");
	}

	if (!data->bSspShutdown)
		if (atomic_read(&data->aSensorEnable) & (1 << uChangedSensor)) {
			uBuf[1] = (u8) get_msdelay(dSensorDelay);
			uBuf[0] = (u8) get_delay_cmd(uBuf[1]);

			send_instruction(data, REMOVE_SENSOR, uChangedSensor, uBuf, 2);
		}
	data->aiCheckStatus[uChangedSensor] = NO_SENSOR_STATE;

#if defined (CONFIG_SEC_MONTBLANC_PROJECT)
	if(((1 << uChangedSensor)&0x20)&&((uNewEnable&0x30)==0x00)){
		proximity_ldo_enable(0);
	}else if(((1 << uChangedSensor)&0x10)&&((uNewEnable&0x30)==0x00)){
		proximity_ldo_enable(0);
	}
#endif

	return 0;
}

/*************************************************************************/
/* ssp Sysfs                                                             */
/*************************************************************************/

static ssize_t show_enable_irq(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - %d\n", __func__, !data->bSspShutdown);

	return sprintf(buf, "%d\n", !data->bSspShutdown);
}

static ssize_t set_enable_irq(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	u8 dTemp;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtou8(buf, 10, &dTemp) < 0)
		return -1;

	pr_info("[SSP] %s - %d start\n", __func__, dTemp);
	if (dTemp) {
		reset_mcu(data);
		enable_debug_timer(data);
	} else if (!dTemp) {
		disable_debug_timer(data);
		ssp_enable(data, 0);
	} else
		pr_err("[SSP] %s - invalid value\n", __func__);
	pr_info("[SSP] %s - %d end\n", __func__, dTemp);
	return size;
}

static ssize_t show_sensors_enable(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	ssp_dbg("[SSP]: %s - cur_enable = %d\n", __func__,
		 atomic_read(&data->aSensorEnable));

	return sprintf(buf, "%9u\n", atomic_read(&data->aSensorEnable));
}

static ssize_t set_sensors_enable(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dTemp;
	unsigned int uNewEnable = 0, uChangedSensor = 0;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dTemp) < 0)
		return -EINVAL;

	uNewEnable = (unsigned int)dTemp;
	ssp_dbg("[SSP]: %s - new_enable = %u, old_enable = %u\n", __func__,
		 uNewEnable, atomic_read(&data->aSensorEnable));

#if defined (CONFIG_SEC_MONTBLANC_PROJECT)
	if((uNewEnable&0x20)&&((atomic_read(&data->aSensorEnable)&0x30)==0x00)){
		proximity_ldo_enable(1);
	}else if((uNewEnable&0x10)&&((atomic_read(&data->aSensorEnable)&0x30)==0x00)){
		proximity_ldo_enable(1);
	}
#endif
	if (uNewEnable == atomic_read(&data->aSensorEnable))
		return size;

	for (uChangedSensor = 0; uChangedSensor < SENSOR_MAX; uChangedSensor++) {
		if ((atomic_read(&data->aSensorEnable) & (1 << uChangedSensor))
			!= (uNewEnable & (1 << uChangedSensor))) {

			if (!(uNewEnable & (1 << uChangedSensor)))
				ssp_remove_sensor(data, uChangedSensor,
					uNewEnable); /* disable */
			/* In case of enabling */
			/* we will sensor instruction on change_sensor_delay. */
			else if (!((uNewEnable & (1 << uChangedSensor)) & data->uSensorState)) {
				pr_err("[SSP] %s - Sensor was not connected(%u)!!\n", __func__, uChangedSensor);
				return -ENODEV; //-19
			}
			break;
		}
	}
	atomic_set(&data->aSensorEnable, uNewEnable);

	return size;
}

static ssize_t show_acc_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[ACCELEROMETER_SENSOR]);
}

static ssize_t set_acc_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	if ((atomic_read(&data->aSensorEnable) & (1 << ORIENTATION_SENSOR)) &&
		(data->adDelayBuf[ORIENTATION_SENSOR] < dNewDelay))
		data->adDelayBuf[ACCELEROMETER_SENSOR] = dNewDelay;
	else
		change_sensor_delay(data, ACCELEROMETER_SENSOR, dNewDelay);

	return size;
}
/*
static ssize_t show_ori_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[ORIENTATION_SENSOR]);
}

static ssize_t set_ori_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	if (data->aiCheckStatus[ACCELEROMETER_SENSOR] == NO_SENSOR_STATE) {
		data->aiCheckStatus[ACCELEROMETER_SENSOR] = ADD_SENSOR_STATE;
		change_sensor_delay(data, ORIENTATION_SENSOR, dNewDelay);
	} else if (data->aiCheckStatus[ACCELEROMETER_SENSOR] ==
			RUNNING_SENSOR_STATE) {
		if (dNewDelay < data->adDelayBuf[ACCELEROMETER_SENSOR])
			change_sensor_delay(data,
				ORIENTATION_SENSOR, dNewDelay);
		else
			data->adDelayBuf[ORIENTATION_SENSOR] = dNewDelay;
	}
	return size;
}
*/
static ssize_t show_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GYROSCOPE_SENSOR]);
}

static ssize_t set_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GYROSCOPE_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_mag_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GEOMAGNETIC_SENSOR]);
}

static ssize_t set_mag_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GEOMAGNETIC_SENSOR, dNewDelay);

	return size;
}

static ssize_t show_uncalib_mag_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GEOMAGNETIC_UNCALIB_SENSOR]);
}

static ssize_t set_uncalib_mag_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GEOMAGNETIC_UNCALIB_SENSOR, dNewDelay);

	return size;
}

static ssize_t show_rot_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[ROTATION_VECTOR]);
}

static ssize_t set_rot_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, ROTATION_VECTOR, dNewDelay);

	return size;
}

static ssize_t show_game_rot_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GAME_ROTATION_VECTOR]);
}

static ssize_t set_game_rot_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GAME_ROTATION_VECTOR, dNewDelay);

	return size;
}

static ssize_t show_step_det_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n",
		data->adDelayBuf[STEP_DETECTOR]);
}

static ssize_t set_step_det_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, STEP_DETECTOR, dNewDelay);
	return size;
}

static ssize_t show_sig_motion_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n",
		data->adDelayBuf[SIG_MOTION_SENSOR]);
}

static ssize_t set_sig_motion_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -1;

	change_sensor_delay(data, SIG_MOTION_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_uncalib_gyro_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GYRO_UNCALIB_SENSOR]);
}

static ssize_t set_uncalib_gyro_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GYRO_UNCALIB_SENSOR, dNewDelay);

	return size;
}

static ssize_t show_pressure_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[PRESSURE_SENSOR]);
}

static ssize_t set_pressure_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, PRESSURE_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_gesture_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[GESTURE_SENSOR]);
}

static ssize_t set_gesture_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, GESTURE_SENSOR, dNewDelay);

	return size;
}

static ssize_t show_light_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[LIGHT_SENSOR]);
}

static ssize_t set_light_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, LIGHT_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_prox_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n", data->adDelayBuf[PROXIMITY_SENSOR]);
}

static ssize_t set_prox_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, PROXIMITY_SENSOR, dNewDelay);
	return size;
}

static ssize_t show_temp_humi_delay(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data  = dev_get_drvdata(dev);

	return sprintf(buf, "%lld\n",
		data->adDelayBuf[TEMPERATURE_HUMIDITY_SENSOR]);
}

static ssize_t set_temp_humi_delay(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int64_t dNewDelay;
	struct ssp_data *data  = dev_get_drvdata(dev);

	if (kstrtoll(buf, 10, &dNewDelay) < 0)
		return -EINVAL;

	change_sensor_delay(data, TEMPERATURE_HUMIDITY_SENSOR, dNewDelay);
	return size;
}

ssize_t ssp_sensorhub_voicel_pcmdump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct ssp_data *data = dev_get_drvdata(dev);
	int status = ssp_sensorhub_pcm_dump(data->hub_data);

	return sprintf(buf, "%s\n", (status ? "OK" : "NG"));
}

static DEVICE_ATTR(voice_pcmdump, S_IRUGO, ssp_sensorhub_voicel_pcmdump_show, NULL);

static struct device_attribute *voice_attrs[] = {
	&dev_attr_voice_pcmdump,
	NULL,
};

static void initialize_voice_sysfs(struct ssp_data *data)
{
	sensors_register(data->voice_device, data, voice_attrs, "ssp_voice");
}

static void remove_voice_sysfs(struct ssp_data *data)
{
	sensors_unregister(data->voice_device, voice_attrs);
}


static DEVICE_ATTR(mcu_rev, S_IRUGO, mcu_revision_show, NULL);
static DEVICE_ATTR(mcu_name, S_IRUGO, mcu_model_name_show, NULL);
static DEVICE_ATTR(mcu_update, S_IRUGO, mcu_update_kernel_bin_show, NULL);
static DEVICE_ATTR(mcu_update2, S_IRUGO,
	mcu_update_kernel_crashed_bin_show, NULL);
static DEVICE_ATTR(mcu_update_ums, S_IRUGO, mcu_update_ums_bin_show, NULL);
static DEVICE_ATTR(mcu_reset, S_IRUGO, mcu_reset_show, NULL);
static DEVICE_ATTR(mcu_dump, S_IRUGO, mcu_dump_show, NULL);

static DEVICE_ATTR(mcu_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_factorytest_show, mcu_factorytest_store);
static DEVICE_ATTR(mcu_sleep_test, S_IRUGO | S_IWUSR | S_IWGRP,
	mcu_sleep_factorytest_show, mcu_sleep_factorytest_store);
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sensors_enable, set_sensors_enable);
static DEVICE_ATTR(enable_irq, S_IRUGO | S_IWUSR | S_IWGRP,
	show_enable_irq, set_enable_irq);

static struct device_attribute dev_attr_accel_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_acc_delay, set_acc_delay);
static struct device_attribute dev_attr_gyro_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gyro_delay, set_gyro_delay);
static struct device_attribute dev_attr_mag_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_mag_delay, set_mag_delay);
static struct device_attribute dev_attr_uncalib_mag_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_uncalib_mag_delay, set_uncalib_mag_delay);
static struct device_attribute dev_attr_rot_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_rot_delay, set_rot_delay);
static struct device_attribute dev_attr_game_rot_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_game_rot_delay, set_game_rot_delay);
static struct device_attribute dev_attr_step_det_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_step_det_delay, set_step_det_delay);
static struct device_attribute dev_attr_pressure_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_pressure_delay, set_pressure_delay);
static struct device_attribute dev_attr_gesture_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_gesture_delay, set_gesture_delay);
static struct device_attribute dev_attr_light_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_light_delay, set_light_delay);
static struct device_attribute dev_attr_prox_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_prox_delay, set_prox_delay);
static struct device_attribute dev_attr_temp_humi_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_temp_humi_delay, set_temp_humi_delay);
static struct device_attribute dev_attr_sig_motion_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_sig_motion_delay, set_sig_motion_delay);
static struct device_attribute dev_attr_uncalib_gyro_poll_delay
	= __ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
	show_uncalib_gyro_delay, set_uncalib_gyro_delay);

static struct device_attribute *mcu_attrs[] = {
	&dev_attr_enable,
	&dev_attr_mcu_rev,
	&dev_attr_mcu_name,
	&dev_attr_mcu_test,
	&dev_attr_mcu_reset,
	&dev_attr_mcu_dump,
	&dev_attr_mcu_update,
	&dev_attr_mcu_update2,
	&dev_attr_mcu_update_ums,
	&dev_attr_mcu_sleep_test,
	&dev_attr_enable_irq,
	NULL,
};

static void initialize_mcu_factorytest(struct ssp_data *data)
{
	sensors_register(data->mcu_device, data, mcu_attrs, "ssp_sensor");
}

static void remove_mcu_factorytest(struct ssp_data *data)
{
	sensors_unregister(data->mcu_device, mcu_attrs);
}

int initialize_sysfs(struct ssp_data *data)
{
	if (device_create_file(&data->acc_input_dev->dev,
		&dev_attr_accel_poll_delay))
		goto err_acc_input_dev;

	if (device_create_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay))
		goto err_gyro_input_dev;

	if (device_create_file(&data->pressure_input_dev->dev,
		&dev_attr_pressure_poll_delay))
		goto err_pressure_input_dev;

	if (device_create_file(&data->gesture_input_dev->dev,
		&dev_attr_gesture_poll_delay))
		goto err_gesture_input_dev;

	if (device_create_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay))
		goto err_light_input_dev;

	if (device_create_file(&data->prox_input_dev->dev,
		&dev_attr_prox_poll_delay))
		goto err_prox_input_dev;

	if (device_create_file(&data->temp_humi_input_dev->dev,
			&dev_attr_temp_humi_poll_delay))
			goto err_temp_humi_input_dev;

	if (device_create_file(&data->mag_input_dev->dev,
		&dev_attr_mag_poll_delay))
		goto err_mag_input_dev;

	if (device_create_file(&data->uncalib_mag_input_dev->dev,
		&dev_attr_uncalib_mag_poll_delay))
		goto err_uncalib_mag_input_dev;

	if (device_create_file(&data->rot_input_dev->dev,
		&dev_attr_rot_poll_delay))
		goto err_rot_input_dev;

	if (device_create_file(&data->game_rot_input_dev->dev,
		&dev_attr_game_rot_poll_delay))
		goto err_game_rot_input_dev;
	if (device_create_file(&data->step_det_input_dev->dev,
		&dev_attr_step_det_poll_delay))
		goto err_step_det_input_dev;

	if (device_create_file(&data->sig_motion_input_dev->dev,
		&dev_attr_sig_motion_poll_delay))
		goto err_sig_motion_input_dev;

	if (device_create_file(&data->uncalib_gyro_input_dev->dev,
		&dev_attr_uncalib_gyro_poll_delay))
		goto err_uncalib_gyro_input_dev;

	initialize_accel_factorytest(data);
	initialize_gyro_factorytest(data);
	initialize_prox_factorytest(data);
	initialize_light_factorytest(data);
	initialize_pressure_factorytest(data);
	initialize_magnetic_factorytest(data);
	initialize_mcu_factorytest(data);
#if defined (CONFIG_SENSORS_SSP_MAX88921)
	initialize_gesture_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_SHTC1
	initialize_temphumidity_factorytest(data);
#endif
	/*snamy.jeong_0630 voice dump & data*/
	initialize_voice_sysfs(data);

	return SUCCESS;

err_uncalib_gyro_input_dev:
	device_remove_file(&data->sig_motion_input_dev->dev,
		&dev_attr_sig_motion_poll_delay);
err_sig_motion_input_dev:
	device_remove_file(&data->step_det_input_dev->dev,
		&dev_attr_step_det_poll_delay);
err_step_det_input_dev:
	device_remove_file(&data->game_rot_input_dev->dev,
		&dev_attr_game_rot_poll_delay);
err_game_rot_input_dev:
	device_remove_file(&data->rot_input_dev->dev,
		&dev_attr_rot_poll_delay);
err_rot_input_dev:
	device_remove_file(&data->uncalib_mag_input_dev->dev,
		&dev_attr_uncalib_mag_poll_delay);
err_uncalib_mag_input_dev:
	device_remove_file(&data->mag_input_dev->dev,
		&dev_attr_mag_poll_delay);
err_mag_input_dev:
	device_remove_file(&data->temp_humi_input_dev->dev,
		&dev_attr_temp_humi_poll_delay);
err_temp_humi_input_dev:
	device_remove_file(&data->prox_input_dev->dev,
		&dev_attr_prox_poll_delay);
err_prox_input_dev:
	device_remove_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay);
err_light_input_dev:
	device_remove_file(&data->gesture_input_dev->dev,
		&dev_attr_gesture_poll_delay);
err_gesture_input_dev:
	device_remove_file(&data->pressure_input_dev->dev,
		&dev_attr_pressure_poll_delay);
err_pressure_input_dev:
	device_remove_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay);
err_gyro_input_dev:
	device_remove_file(&data->acc_input_dev->dev,
		&dev_attr_accel_poll_delay);
err_acc_input_dev:
	return ERROR;
}

void remove_sysfs(struct ssp_data *data)
{
	device_remove_file(&data->acc_input_dev->dev,
		&dev_attr_accel_poll_delay);
	device_remove_file(&data->gyro_input_dev->dev,
		&dev_attr_gyro_poll_delay);
	device_remove_file(&data->pressure_input_dev->dev,
		&dev_attr_pressure_poll_delay);
	device_remove_file(&data->gesture_input_dev->dev,
		&dev_attr_gesture_poll_delay);
	device_remove_file(&data->light_input_dev->dev,
		&dev_attr_light_poll_delay);
	device_remove_file(&data->prox_input_dev->dev,
		&dev_attr_prox_poll_delay);
	device_remove_file(&data->temp_humi_input_dev->dev,
		&dev_attr_temp_humi_poll_delay);
	device_remove_file(&data->mag_input_dev->dev,
		&dev_attr_mag_poll_delay);
	device_remove_file(&data->uncalib_mag_input_dev->dev,
		&dev_attr_uncalib_mag_poll_delay);
	device_remove_file(&data->step_det_input_dev->dev,
		&dev_attr_step_det_poll_delay);
	device_remove_file(&data->sig_motion_input_dev->dev,
		&dev_attr_sig_motion_poll_delay);
	device_remove_file(&data->uncalib_gyro_input_dev->dev,
		&dev_attr_uncalib_gyro_poll_delay);
	remove_accel_factorytest(data);
	remove_gyro_factorytest(data);
	remove_prox_factorytest(data);
	remove_light_factorytest(data);
	remove_pressure_factorytest(data);
	remove_magnetic_factorytest(data);
	remove_mcu_factorytest(data);
#ifdef CONFIG_SENSORS_SSP_MAX88920
	remove_gesture_factorytest(data);
#endif
#ifdef CONFIG_SENSORS_SSP_SHTC1
	remove_temphumidity_factorytest(data);
#endif

	/*snamy.jeong_0630 voice dump & data*/
	remove_voice_sysfs(data);

	destroy_sensor_class();
}

#if defined (CONFIG_SEC_MONTBLANC_PROJECT)
static struct regulator *sub_ldo1 = NULL;
void proximity_ldo_enable(int onoff)
{
	int ret = 0;
	if(!sub_ldo1){
		sub_ldo1 = regulator_get(NULL, "lp8720_ldo1");
		if (IS_ERR(sub_ldo1)){
			pr_err("lp8720 : could not get sub_ldo1, rc = %ld\n", PTR_ERR(sub_ldo1));
			sub_ldo1 = NULL;
		}
		if(sub_ldo1 != NULL){
			ret = regulator_set_voltage(sub_ldo1, 1800000, 1800000);
			if (ret) 
				pr_err("set_voltage sub_ldo1 failed, rc=%d\n", ret);
		}
	}
	if(onoff){
		printk(KERN_ERR "[SSP] %s : on\n",__func__);
		ret = regulator_enable(sub_ldo1);
		if (ret) 
			pr_err("enable sub_ldo1 failed, rc=%d\n", ret);
	}
	else{
		printk(KERN_ERR "[SSP] %s : off ",__func__);
		ret = regulator_disable(sub_ldo1);
		if (ret) 
			pr_err("enable sub_ldo1 failed, rc=%d\n", ret);
	}
	gpio_tlmm_config(GPIO_CFG(561, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);	
	gpio_set_value(561, 1);
}
#endif
