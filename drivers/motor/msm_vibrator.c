/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <mach/gpio.h>
#include <mach/pmic.h>
#include <linux/vibrator.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>

#include "../staging/android/timed_output.h"

/* default timeout */
#define VIB_DEFAULT_TIMEOUT 15000

/* need to change for each project */
#define PMIC_LDO_NAME		"8941_l22"

struct msm_vib {
	struct hrtimer vib_timer;
	struct timed_output_dev timed_dev;
	struct work_struct work;
	struct workqueue_struct *queue;

	int state;
	int timeout;
	struct mutex lock;
};

static void set_vibrator(int on)
{
	int ret = 0;
	static struct regulator *reg_ldo_num;

	pr_info("[VIB] %s, on=%d\n",__func__, on);

	if (!reg_ldo_num) {
		reg_ldo_num = regulator_get(NULL, PMIC_LDO_NAME);
		ret = regulator_set_voltage(reg_ldo_num, 3300000, 3300000);

		if (IS_ERR(reg_ldo_num)) {
			pr_err("could not get vib ldo, ret= %ld\n", PTR_ERR(reg_ldo_num));
			return;
		}
	}

	if (on) {
		/* sometimes vibrator is not disabled. 
		   so, if vibrator is already enalble, return. */
		if (regulator_is_enabled(reg_ldo_num)) {
			pr_err("[VIB]already enalbled\n");
			return;
		}
		
		ret = regulator_enable(reg_ldo_num);
		if (ret) {
			pr_err("enable vib ldo enable failed, ret= %d\n", ret);
			return;
		}
	} else {
		if (regulator_is_enabled(reg_ldo_num)) {
			ret = regulator_disable(reg_ldo_num);
			if (ret) {
				pr_err("disable vib ldo disable failed, ret= %d\n", ret);
				return;
			}
		}
	}
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	struct msm_vib *vib = container_of(dev, struct msm_vib,
					 timed_dev);

	mutex_lock(&vib->lock);
	hrtimer_cancel(&vib->vib_timer);

	if (value == 0) {
		pr_info("[VIB] OFF\n");
		vib->state = 0;
	}
	else {
		pr_info("[VIB] ON, Duration : %d msec\n" , value);
		vib->state = 1;

		if (value == 0x7fffffff){
			pr_info("[VIB] No Use Timer %d \n", value);
		}
		else	{
			value = (value > vib->timeout ?
					vib->timeout : value);
			
			hrtimer_start(&vib->vib_timer,
					ktime_set(value / 1000, (value % 1000) * 1000000),
					HRTIMER_MODE_REL);
                }
	}
	mutex_unlock(&vib->lock);
	queue_work(vib->queue, &vib->work);	
}

static void msm_vibrator_update(struct work_struct *work)
{
	struct msm_vib *vib = container_of(work, struct msm_vib,
					 work);

	set_vibrator(vib->state);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	struct msm_vib *vib = container_of(dev, struct msm_vib,
							 timed_dev);

	if (hrtimer_active(&vib->vib_timer)) {
		ktime_t r = hrtimer_get_remaining(&vib->vib_timer);
		return (int)ktime_to_us(r);
	}
	else 
	return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	struct msm_vib *vib = container_of(timer, struct msm_vib,
							 vib_timer);

	vib->state = 0;
	queue_work(vib->queue, &vib->work);

	return HRTIMER_NORESTART;
}

static int msm_vibrator_probe(struct platform_device *pdev)
{
	struct msm_vib *vib;
	int rc = 0;

	pr_info("[VIB] %s\n",__func__);

	vib = devm_kzalloc(&pdev->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)	{
		pr_err("%s : Failed to allocate memory\n", __func__);
		return -ENOMEM;
	}

	vib->timeout = VIB_DEFAULT_TIMEOUT;

	INIT_WORK(&vib->work, msm_vibrator_update);
	mutex_init(&vib->lock);

	vib->queue = create_singlethread_workqueue("msm_vibrator");
	if (!vib->queue) {
		pr_err("%s(): can't create workqueue\n", __func__);
		rc = -ENOMEM;
		goto err_create_work_queue;
	}

	hrtimer_init(&vib->vib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vib->vib_timer.function = vibrator_timer_func;

	vib->timed_dev.name = "vibrator";
	vib->timed_dev.get_time = vibrator_get_time;
	vib->timed_dev.enable = vibrator_enable;

	dev_set_drvdata(&pdev->dev, vib);

	rc = timed_output_dev_register(&vib->timed_dev);
	if (rc < 0) {
		goto err_read_vib;
	}
	
	return 0;

err_read_vib:
	pr_err("[VIB] timed_output_dev_register fail (rc=%d)\n", rc);

err_create_work_queue:
	destroy_workqueue(vib->queue);

	return rc;
}

static int msm_vibrator_remove(struct platform_device *pdev)
{
	struct msm_vib *vib = dev_get_drvdata(&pdev->dev);

	destroy_workqueue(vib->queue);

	return 0;
	}

static struct platform_driver msm_vibrator_platdrv =
{
	.driver =
	{
		.name = "msm_vibrator",
		.owner = THIS_MODULE,
	},
	.probe = msm_vibrator_probe,
	.remove = msm_vibrator_remove,
};

static int __init msm_timed_vibrator_init(void)
{
	return platform_driver_register(&msm_vibrator_platdrv);
}
module_init(msm_timed_vibrator_init);

void __exit msm_timed_vibrator_exit(void)
{
	platform_driver_unregister(&msm_vibrator_platdrv);
}
module_exit(msm_timed_vibrator_exit);

MODULE_DESCRIPTION("timed output vibrator device");
MODULE_LICENSE("GPL");
