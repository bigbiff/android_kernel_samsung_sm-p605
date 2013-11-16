/***************************************************************************
 *
 *   Silicon Image SiI8246 MHL Transmitter Driver
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/

/* ========================================================

   daniel.lee@siliconimage.com

   ========================================================*/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/gpio.h>

#include <asm/irq.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/battery/sec_charger.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include "sii8246_platform.h"
#include "sii8246_mhl_tx.h"
#include <linux/timer.h>
#include <linux/input.h>
#include "../../mdss/mdss_panel.h"
#include "../../mdss/mdss_hdmi_tx.h"
#if defined(CONFIG_OF)
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#endif
#ifdef CONFIG_MFD_MAX77803
#include <linux/mfd/max77803.h>
#endif

static struct mhl_tx *g_mhl_tx;
#define SFEATURE_INT_HPD_PP_MODE
static int __devinit simgC8_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devinit simg72_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devinit simg7A_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devinit simg92_probe(struct i2c_client *client, const struct i2c_device_id *id);

static int __devexit simgC8_remove(struct i2c_client *client);
static int __devexit simg72_remove(struct i2c_client *client);
static int __devexit simg7A_remove(struct i2c_client *client);
static int __devexit simg92_remove(struct i2c_client *client);



#if 1
#define	MHL_MAX_RCP_KEY_CODE	(0x7F + 1)	// inclusive
u8 rcpSupportTable [MHL_MAX_RCP_KEY_CODE] = {
	(MHL_DEV_LD_GUI),		// 0x00 = Select
	(MHL_DEV_LD_GUI),		// 0x01 = Up
	(MHL_DEV_LD_GUI),		// 0x02 = Down
	(MHL_DEV_LD_GUI),		// 0x03 = Left
	(MHL_DEV_LD_GUI),		// 0x04 = Right
	0, 0, 0, 0,				// 05-08 Reserved
	(MHL_DEV_LD_GUI),		// 0x09 = Root Menu
	0, 0, 0,				// 0A-0C Reserved
	(MHL_DEV_LD_GUI),		// 0x0D = Select
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,	// 0E-1F Reserved
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Numeric keys 0x20-0x29
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),
	0,						// 0x2A = Dot
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Enter key = 0x2B
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA | MHL_DEV_LD_TUNER),	// Clear key = 0x2C
	0, 0, 0,				// 2D-2F Reserved
	(MHL_DEV_LD_TUNER),		// 0x30 = Channel Up
	(MHL_DEV_LD_TUNER),		// 0x31 = Channel Dn
	(MHL_DEV_LD_TUNER),		// 0x32 = Previous Channel
	(MHL_DEV_LD_AUDIO),		// 0x33 = Sound Select
	0,						// 0x34 = Input Select
	0,						// 0x35 = Show Information
	0,						// 0x36 = Help
	0,						// 0x37 = Page Up
	0,						// 0x38 = Page Down
	0, 0, 0, 0, 0, 0, 0,	// 0x39-0x3F Reserved
	0,						// 0x40 = Undefined

	(MHL_DEV_LD_SPEAKER),	// 0x41 = Volume Up
	(MHL_DEV_LD_SPEAKER),	// 0x42 = Volume Down
	(MHL_DEV_LD_SPEAKER),	// 0x43 = Mute
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x44 = Play
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x45 = Stop
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x46 = Pause
	(MHL_DEV_LD_RECORD),	// 0x47 = Record
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x48 = Rewind
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x49 = Fast Forward
	(MHL_DEV_LD_MEDIA),		// 0x4A = Eject
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),	// 0x4B = Forward
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_MEDIA),	// 0x4C = Backward
	0, 0, 0,				// 4D-4F Reserved
	0,						// 0x50 = Angle
	0,						// 0x51 = Subpicture
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 52-5F Reserved
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x60 = Play Function
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO),	// 0x61 = Pause the Play Function
	(MHL_DEV_LD_RECORD),	// 0x62 = Record Function
	(MHL_DEV_LD_RECORD),	// 0x63 = Pause the Record Function
	(MHL_DEV_LD_VIDEO | MHL_DEV_LD_AUDIO | MHL_DEV_LD_RECORD),	// 0x64 = Stop Function

	(MHL_DEV_LD_SPEAKER),	// 0x65 = Mute Function
	(MHL_DEV_LD_SPEAKER),	// 0x66 = Restore Mute Function
	0, 0, 0, 0, 0, 0, 0, 0, 0, 	                        // 0x67-0x6F Undefined or reserved
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 		// 0x70-0x7F Undefined or reserved
};
#endif

#ifdef CONFIG_OF
static int __devinit of_sii8246_probe_dt(struct i2c_client *client, const struct i2c_device_id *id);
static struct of_device_id sii8246_dt_ids[] = {
	{ .compatible = "sii8246,tmds",},
	{ .compatible = "sii8246,cbus",},
	{ .compatible = "sii8246,hdmi",},
	{ .compatible = "sii8246,tpi",},
	{}
};
MODULE_DEVICE_TABLE(of, sii8246_dt_ids);

static const struct i2c_device_id sii8246_id[] = {
	{"sii8246_mhlv2", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, sii8246_id);

static struct i2c_driver sii8246_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "sii8246_mhlv2dt",
		.of_match_table = of_match_ptr(sii8246_dt_ids),
	},
	.id_table = sii8246_id,
	.probe = of_sii8246_probe_dt,
};
#else
static struct i2c_driver simg72_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SIMG72",
	},
	.id_table	= SIMG72_id,
	.probe	= simg72_probe,
	.remove	= __devexit_p(simg72_remove),
	.command = NULL,
};

static struct i2c_driver simg7A_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SIMG7A",
	},
	.id_table	= SIMG7A_id,
	.probe	= simg7A_probe,
	.remove	= __devexit_p(simg7A_remove),
	.command = NULL,
};

static struct i2c_driver simg92_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SIMG92",
	},
	.id_table	= SIMG92_id,
	.probe	= simg92_probe,
	.remove	= __devexit_p(simg92_remove),
	.command = NULL,
};

static struct i2c_driver simgC8_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "SIMGC8",
	},
	.id_table	= SIMGC8_id,
	.probe	= simgC8_probe,
	.remove	= __devexit_p(simgC8_remove),
	.command = NULL,
};
#endif
void mhl_enable_irq(struct mhl_platform_data *pdata)
{
	if (pdata->irq_enabled)
		return;

	pr_info("Irq Enabled\n");
	enable_irq(pdata->irq);
	pdata->irq_enabled = true;
}

void mhl_disable_irq(struct mhl_platform_data *pdata)
{
	if (!pdata->irq_enabled)
		return;

	pr_info("Irq Disabled\n");
	disable_irq(pdata->irq);
	pdata->irq_enabled = false;
}

void mhl_disable_irq_nosync(struct mhl_platform_data *pdata)
{
	if (!pdata->irq_enabled)
		return;

	pr_info("Irq Disabled\n");
	disable_irq_nosync(pdata->irq);
	pdata->irq_enabled = false;
}

static void hw_resst(struct mhl_platform_data *pdata)
{
	if (pdata->hw_reset)
		pdata->hw_reset();
}

struct i2c_client* get_simgI2C_client(struct mhl_platform_data *pdata, u8 device_id)
{
	struct i2c_client* client_ptr;

	if (device_id == 0x72)
		client_ptr = pdata->simg72_tx_client;
	else if (device_id == 0x7A)
		client_ptr = pdata->simg7A_tx_client;
	else if (device_id == 0x92)
		client_ptr = pdata->simg92_tx_client;
	else if (device_id == 0xC8)
		client_ptr = pdata->simgC8_tx_client;
	else
		client_ptr = NULL;

	return client_ptr;
}


static int I2C_ReadByte(struct mhl_platform_data *pdata, u8 deviceID, u8 offset, u8 *data)
{
	int ret = 0;
	struct i2c_client* client_ptr = get_simgI2C_client(pdata, deviceID);

	if (!data) {
		printk(KERN_INFO"[MHL]I2C_ReadByte error1 %x\n", deviceID);
		return -EINVAL;
	}

	if (!client_ptr) {
		printk(KERN_INFO"[MHL]I2C_ReadByte error2 %x\n", deviceID);
		return -EINVAL;
	}

	ret = i2c_smbus_read_byte_data(client_ptr, offset);

	if (ret < 0) {
		printk(KERN_INFO"[MHL]I2C_ReadByte error3 %x\n", deviceID);
		return ret;
	}

	*data = (ret & 0x000000FF);
	return ret;
}
#if 0
static int I2C_ReadByte_block(struct mhl_platform_data *pdata, u8 deviceID, u8 offset, u8 length, u8 *data)
{
	int ret = 0;
	struct i2c_client* client_ptr = get_simgI2C_client(pdata, deviceID);

	if (!client_ptr) {
		printk(KERN_INFO"[MHL]I2C_ReadByte error2 %x\n", deviceID);
		return -EINVAL;
	}

	ret = i2c_smbus_read_i2c_block_data(client_ptr, offset, length, data);
	return ret;
}

static int I2C_WriteByte_block(struct mhl_platform_data *pdata, u8 deviceID, u8 offset, u8 length, u8 *data)
{
	int ret = 0;
	struct i2c_client* client_ptr = get_simgI2C_client(pdata, deviceID);

	if (!client_ptr) {
		printk(KERN_INFO"[MHL]I2C_ReadByte error2 %x\n", deviceID);
		return -EINVAL;
	}

	ret = i2c_smbus_write_i2c_block_data(client_ptr, offset, length, data);
	return ret;
}
#endif

static int I2C_WriteByte(struct mhl_platform_data *pdata, u8 deviceID, u8 offset, u8 value)
{
	int ret;
	struct i2c_client* client_ptr = get_simgI2C_client(pdata, deviceID);

	if (client_ptr == NULL) {
		printk("I2C_WriteByte (client_ptr == NULL) \n");
		return 0;
	}

	ret = i2c_smbus_write_byte_data(client_ptr, offset, value);
	return ret;
}

static int I2CReadModify(struct mhl_platform_data *pdata, u8 deviceID, u8 Offset, u8 Mask, u8 Data)
{
	u8 rd;
	int ret;

	ret = I2C_ReadByte(pdata, deviceID, Offset, &rd);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d devide ID:%x offset:%x failed !\n", __func__, __LINE__, deviceID, Offset);
		return ret;
	}

	rd &= ~Mask;
	rd |= (Data & Mask);
	ret = I2C_WriteByte(pdata, deviceID, Offset, rd);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d devide ID: %x offset:%x failed !\n", __func__, __LINE__, deviceID, Offset);
		return ret;
	}

	return ret;
}

#define	SET_BIT(pdata, deviceID, offset, bitnumber)		I2CReadModify(pdata, deviceID, offset,(1<<bitnumber),(1<<bitnumber))
#define	CLR_BIT(pdata, deviceID, offset, bitnumber)		I2CReadModify(pdata, deviceID, offset,(1<<bitnumber),0x00)


u8 sii8240_support_packedpixel(void)
{
	//	struct sii8240_data *sii8240 = dev_get_drvdata(sii8246_mhldev);
	//	return (sii8240->regs.peer_devcap[MHL_DEVCAP_VID_LINK_MODE] >> 3) & 0x01;
	return false;
}


static int hpd_drive_low(struct mhl_tx *mhl)
{
	int ret = 0;

	pr_info("debug_message : %s. %d ", __func__, __LINE__);
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x79, BIT5 | BIT4, BIT4);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return ret;
	}

	if (mhl->hpd_state) {
		mhl->hpd_state = false;
	}

	return ret;
}

static int hpd_drive_high(struct mhl_tx *mhl)
{
	int ret = 0;

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x79, BIT5 | BIT4, BIT5 | BIT4);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return ret;
	}

	if (!mhl->hpd_state) {
		mhl_hpd_handler(true);
		mhl->hpd_state = true;
	}

	return ret;
}

static int forceusbidswitchopen(struct mhl_tx *mhl)
{
	int ret = 0;

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x90, BIT0, 0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x95, BIT6, BIT6);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return ret;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x92, 0x86);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = hpd_drive_low(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

exit_func:
	return ret;
}

static int releaseubidswichopen(struct mhl_tx *mhl)
{
	int ret = 0;

	msleep(50);
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x95, BIT6, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x90, BIT0, BIT0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

exit_func:
	return ret;
}


static int TX_GO2D3(struct mhl_tx *mhl)
{
	int ret = 0;

	ret = forceusbidswitchopen(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x93, BIT7 | BIT6 | BIT5 | BIT4, 0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x94, BIT1 | BIT0, 0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	// all interrupt status register clearing...
	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR1_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR2_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR3_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR5_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR7_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR8_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR0_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR1_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR2_STATUS_ADD, 0xFF);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	// all interrupt status register clearing end
	ret = releaseubidswichopen(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = hpd_drive_low(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x01, 0x03);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_1, 0x3D, BIT0, 0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	mutex_lock(&mhl->pdata->mhl_status_lock);
	mhl->pdata->status.op_status = MHL_READY_RGND_DETECT;
	mutex_unlock(&mhl->pdata->mhl_status_lock);
	printk(KERN_INFO "sii8246: %s() end.. %d  !\n", __func__, __LINE__);
exit_func:
	return ret;
}



static int cbus_reset(struct mhl_tx *mhl)
{
	int ret = 0;
	u8 i = 0;

	pr_info("debug_message : %s. %d ", __func__, __LINE__);
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x05, BIT3, BIT3);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	msleep(2);
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x05, BIT3, 0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	for (i = 0; i < 4; i++) {
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, (0xE0 + i), 0xFF);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, (0xF0 + i), 0xFF);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}
	}

	pr_info("debug_message : %s. %d ", __func__, __LINE__);
exit_func:
	return ret;
}

static int init_cbus_regs(struct mhl_tx *mhl)
{
	int ret = 0;
	u8 data = 0;

	pr_info("debug_message : %s. %d ", __func__, __LINE__);
	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x07, 0xF2);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x40, 0x03);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x42, 0x06);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x36, 0x0B);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x3D, 0xFD);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x1C, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x1D, 0x0F);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x80, DEVCAP_VAL_DEV_STATE);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x81, DEVCAP_VAL_MHL_VERSION);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x82, DEVCAP_VAL_DEV_CAT);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x83, DEVCAP_VAL_ADOPTER_ID_H);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x84, DEVCAP_VAL_ADOPTER_ID_L);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x85, DEVCAP_VAL_VID_LINK_MODE);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x86, DEVCAP_VAL_AUD_LINK_MODE);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x87, DEVCAP_VAL_VIDEO_TYPE);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x88, DEVCAP_VAL_LOG_DEV_MAP);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x89, DEVCAP_VAL_BANDWIDTH);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x8A, DEVCAP_VAL_FEATURE_FLAG);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x8B, DEVCAP_VAL_DEVICE_ID_H);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x8C, DEVCAP_VAL_DEVICE_ID_L);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x8D, DEVCAP_VAL_SCRATCHPAD_SIZE);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x8E, DEVCAP_VAL_INT_STAT_SIZE);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x8F, DEVCAP_VAL_RESERVED);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, REG_CBUS_LINK_CONTROL_2, &data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, REG_CBUS_LINK_CONTROL_2, (data | 0x0C));

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, REG_MSC_TIMEOUT_LIMIT, &data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	data &= ~ MSC_TIMEOUT_LIMIT_MSB_MASK;
	data |= 0x0F;
	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, REG_MSC_TIMEOUT_LIMIT, (data & MSC_TIMEOUT_LIMIT_MSB_MASK));

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, REG_CBUS_LINK_CONTROL_1, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_CBUS, REG_CBUS_LINK_CONTROL_11, BIT5 | BIT4 | BIT3, BIT5 | BIT4);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_CBUS, REG_MSC_TIMEOUT_LIMIT, 0x0F, 0x0D);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_CBUS, 0x2E, BIT4 | BIT2, BIT4 | BIT2);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	pr_info("debug_message : %s. %d ", __func__, __LINE__);
exit_func:
	return ret;
}




static int mhl_tx_interrupt_set(struct mhl_tx *mhl)
{
	int ret;

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR1_ENABLE_ADD,  mhl->pdata->status.intr1_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR2_ENABLE_ADD,  mhl->pdata->status.intr2_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR3_ENABLE_ADD,  mhl->pdata->status.intr3_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR4_ENABLE_ADD, mhl->pdata->status.intr4_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR5_ENABLE_ADD, mhl->pdata->status.intr5_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR7_ENABLE_ADD, mhl->pdata->status.intr7_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR8_ENABLE_ADD, mhl->pdata->status.intr8_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR_ENABLE_ADD, mhl->pdata->status.intr_cbus_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR0_ENABLE_ADD,   mhl->pdata->status.intr_cbus0_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR1_ENABLE_ADD,    mhl->pdata->status.intr_cbus1_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR2_ENABLE_ADD, mhl->pdata->status.intr_cbus2_mask_value);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

exit_func:
	return ret;
}



static int mhl_init_func(struct mhl_tx *mhl)
{
	int ret = 0;
	u8 data = 0;

	printk(KERN_INFO "[$$$$$]sii8246: %s():%d  !\n", __func__, __LINE__);
	ret = I2C_WriteByte(mhl->pdata, PAGE_1, 0x3D, 0x35);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	pr_info("debug_message : %s. %d ", __func__, __LINE__);
	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x11, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x12, 0x15);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x08, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = cbus_reset(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x17, 0x03);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x1A, 0x20);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x22, 0xE0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x23, 0xC0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x24, 0xA0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x25, 0x80);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x26, 0x60);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x27, 0x40);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x28, 0x20);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x29, 0x10);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x4C, 0xD0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x4D, 0x02);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x80, 0x3C);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x85, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x13, 0x40);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x45, 0x06);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x4B, 0x06);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x31, 0x0A);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA0, 0xD0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA1, 0xBC);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA3, 0x74);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA6, 0x03);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x2B, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x90, BIT3 | BIT2, BIT2);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x91, 0xA5);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x94, 0x77);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x31, &data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x31, (data | 0x0C));

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA5, 0xA0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA7, 0x08);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x95, 0x11);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x97, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x92, 0x86);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x93, 0x8C);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

#ifdef SFEATURE_INT_HPD_PP_MODE
	// push pull mode...
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x79, BIT6 | BIT2 | BIT1, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return ret;
	}

#endif/*SFEATURE_INT_HPD_PP_MODE*/
	ret = hpd_drive_low(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	msleep(25);
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x95, BIT6, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x9A, BIT_DC9_CBUS_LOW_TO_DISCONNECT | BIT_DC9_WAKE_DRVFLT | BIT_DC9_DISC_PULSE_PROCEED);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x90, 0x26);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = init_cbus_regs(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x05, 0x04);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x0D, 0x1C);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	printk(KERN_INFO "[*****]sii8246: %s():%d  !\n", __func__, __LINE__);
exit_func:
	return ret;
}
static int mhl_power_on_cb(struct notifier_block *this,
                                        unsigned long event, void *ptr)
{
        struct mhl_tx *mhl = container_of(this, struct mhl_tx,
                                                        mhl_nb);
	int ret;

	pr_info("mhl_onoff_ex: mhl  %s\n", event ? "on" : "off");

	if (!mhl || !mhl->pdata) {
		pr_info("sii8246: mhl_onoff_ex: getting resource is failed\n");
		return 0;
	}

	if (mhl->power_state == event) {
		pr_info("sii8246 : Same muic event, Ignored!\n");
		return 0;
	}

	mhl->power_state = event;
	mhl->pdata->power(mhl->power_state);
        if (event) {
                pr_info("sii8246:detection started\n");
                wake_lock(&mhl->mhl_wake_lock);
                /*MHL IC power On/Off using onoff param*/
        } else {
                pr_info("sii8246:disconnection\n");
		mhl_disable_irq(mhl->pdata);
		wake_unlock(&mhl->mhl_wake_lock);
		if (mhl->pdata->vbus_present)
			mhl->pdata->vbus_present(false, -1);

		if (mhl->hpd_state) {
			mhl_hpd_handler(false);
			mhl->hpd_state = false;
		}
                return 0;
	}

	hw_resst(mhl->pdata);
	mhl->pdata->status.op_status = NO_MHL_STATUS;
	mhl->pdata->status.linkmode = 0x03;
	mhl->pdata->hpd_status = false;
	mhl->pdata->status.mhl_rgnd = false;
	mhl->pdata->status.cbus_connected = false;
	mhl->pdata->status.connected_ready = false;
	mhl->msc_cmd_abord = false;
	ret = mhl_init_func(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return 0;
	}

	mhl->pdata->status.intr1_mask_value = 0;
	mhl->pdata->status.intr2_mask_value = 0;
	mhl->pdata->status.intr3_mask_value = 0;
	mhl->pdata->status.intr4_mask_value = BIT_INTR4_RGND;
	mhl->pdata->status.intr5_mask_value = 0;
	mhl->pdata->status.intr7_mask_value = 0;
	mhl->pdata->status.intr8_mask_value = 0;
	mhl->pdata->status.intr_cbus_mask_value = 0;
	mhl->pdata->status.intr_cbus0_mask_value = 0;
	mhl->pdata->status.intr_cbus1_mask_value = 0;
	mhl->pdata->status.intr_cbus2_mask_value = 0;
	ret = mhl_tx_interrupt_set(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return 0;
	}

	ret = TX_GO2D3(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return 0;
	}

	if (mhl->power_state)
		mhl_enable_irq(mhl->pdata);
	return 0;
}

static int mhl_rx_connected_func(struct mhl_tx *mhl)
{
	int ret;
	u8 int4status = 0;
	u8 cbus_status;
	struct cbus_msg *msc_msg;
	
	ret = I2C_ReadByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, &int4status);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (int4status == 0x00) {
		goto exit_func;
	}

	//clear interrupt status...
	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, int4status);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x0A, &cbus_status);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	mhl->pdata->status.cbus_connected = (cbus_status & 0x01);

	if ((int4status & BIT_INTR4_MHL_EST) && (mhl->pdata->status.cbus_connected)) {
		mutex_lock(&mhl->pdata->mhl_status_lock);
		mhl->pdata->status.op_status = MHL_DISCOVERY_SUCCESS;
		mutex_unlock(&mhl->pdata->mhl_status_lock);
		printk(KERN_INFO "sii8246: MHL_DISCOVERY_SUCCESS :%d \n", __LINE__);
		ret = hpd_drive_low(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA0, 0x10);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x07, 0xF2);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x66, 0x00);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2CReadModify(mhl->pdata, PAGE_0, 0x90, BIT0, BIT0);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		mhl->pdata->status.intr1_mask_value = BIT_INTR1_RSEN_CHG; //(BIT_INTR1_HPD_CHG | BIT_INTR1_RSEN_CHG);
		mhl->pdata->status.intr2_mask_value = 0;
		mhl->pdata->status.intr3_mask_value = 0;
		mhl->pdata->status.intr4_mask_value = (BIT_INTR4_CBUS_LKOUT | BIT_INTR4_CBUS_DISCONNECT);
		mhl->pdata->status.intr5_mask_value = 0; //BIT_INTR5_CKDT_CHANGE;
		mhl->pdata->status.intr7_mask_value = 0;
		mhl->pdata->status.intr8_mask_value = 0;
		mhl->pdata->status.intr_cbus_mask_value = BIT_CBUS_INTR_CONNECTION_CHG |
				BIT_CBUS_INTR_CEC_ABORT | BIT_CBUS_INTR_DDC_ABORT |
				BIT_CBUS_INTR_MSC_MSG_CMD_RCV | BIT_CBUS_INTR_MSC_CMD_DONE |
				BIT_CBUS_INTR_MSC_MT_ABORT | BIT_CBUS_INTR_MSC_MR_ABORT;
		mhl->pdata->status.intr_cbus0_mask_value = BIT_CBUS_INTR0_HPD_RCV | BIT_CBUS_INTR0_MSC_DONE_NACK;
		mhl->pdata->status.intr_cbus1_mask_value = BIT_CBUS_INTR1_MSC_ABORT | BIT_CBUS_INTR1_MSC_CMD_ABORT;
		mhl->pdata->status.intr_cbus2_mask_value = BIT_CBUS_INTR2_MSC_WRITE_BURST_RCV |
				BIT_CBUS_INTR2_MSC_HEARTBEAT_MAX_FAIL |
				BIT_CBUS_INTR2_MSC_SET_INT |
				BIT_CBUS_INTR2_MSC_WRITE_STAT_RCV;
		ret = mhl_tx_interrupt_set(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			return IRQ_HANDLED;
		}

		mutex_lock(&mhl->cbus_cmd_lock);
		mhl->cmd_rx_cnt = 0;
		mhl->cmd_tx_cnt = 0;
		mutex_unlock(&mhl->cbus_cmd_lock);
		mhl->pdata->status.connected_ready |= MHL_STATUS_DCAP_RDY;
		mutex_lock(&mhl->cbus_cmd_lock);

		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}

		msc_msg->command = MHL_WRITE_STAT;
		msc_msg->offset = MHL_STATUS_REG_CONNECTED_RDY;
		msc_msg->lenght = 0x01;
		msc_msg->buff[0] = mhl->pdata->status.connected_ready;
		mhl->cmd_rx_cnt += 1;
		
		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
	} else if (int4status & BIT_INTR4_NON_MHL_EST) {
		mutex_lock(&mhl->pdata->mhl_status_lock);
		mhl->pdata->status.op_status = MHL_DISCOVERY_FAIL;
		printk(KERN_INFO "sii8246: MHL_DISCOVERY_FAIL %s():%d \n", __func__, __LINE__);
		mutex_unlock(&mhl->pdata->mhl_status_lock);
	} else if (mhl->pdata->status.cbus_connected) {
		mutex_lock(&mhl->pdata->mhl_status_lock);
		mhl->pdata->status.op_status = MHL_DISCOVERY_ON;
		printk(KERN_INFO "sii8246: MHL_DISCOVERY_ON %s():%d \n", __func__, __LINE__);
		mutex_unlock(&mhl->pdata->mhl_status_lock);
		mhl->pdata->status.intr1_mask_value = BIT_INTR1_RSEN_CHG;
		mhl->pdata->status.intr2_mask_value = 0;
		mhl->pdata->status.intr3_mask_value = 0;
		mhl->pdata->status.intr4_mask_value = (BIT_INTR4_MHL_EST | BIT_INTR4_NON_MHL_EST);
		mhl->pdata->status.intr5_mask_value = 0;
		mhl->pdata->status.intr7_mask_value = 0;
		mhl->pdata->status.intr8_mask_value = 0;
		mhl->pdata->status.intr_cbus_mask_value = BIT_CBUS_INTR_CONNECTION_CHG |
				BIT_CBUS_INTR_CEC_ABORT |
				BIT_CBUS_INTR_DDC_ABORT |
				BIT_CBUS_INTR_MSC_MSG_CMD_RCV |
				BIT_CBUS_INTR_MSC_CMD_DONE |
				BIT_CBUS_INTR_MSC_MT_ABORT |
				BIT_CBUS_INTR_MSC_MR_ABORT;
		mhl->pdata->status.intr_cbus0_mask_value = BIT_CBUS_INTR0_HPD_RCV | BIT_CBUS_INTR0_MSC_DONE_NACK;
		mhl->pdata->status.intr_cbus1_mask_value = BIT_CBUS_INTR1_MSC_ABORT | BIT_CBUS_INTR1_MSC_CMD_ABORT;
		mhl->pdata->status.intr_cbus2_mask_value = BIT_CBUS_INTR2_MSC_WRITE_BURST_RCV |
				BIT_CBUS_INTR2_MSC_HEARTBEAT_MAX_FAIL |
				BIT_CBUS_INTR2_MSC_SET_INT |
				BIT_CBUS_INTR2_MSC_WRITE_STAT_RCV;
		ret = mhl_tx_interrupt_set(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}
	}

exit_func:
	return ret;
}

static void sii8246_setup_charging(struct mhl_tx *mhl)
{
	u8 plim, dev_cat;
	u16 adopter_id;

	if (mhl->pdata->rx_cap.mhl_ver == 0x20) {
		dev_cat = mhl->pdata->rx_cap.dev_type;
		pr_info("sii8240: DEV_CAT 0x%x\n", dev_cat);

		if (((dev_cat >> 4) & 0x1) == 1) {
			plim = ((dev_cat >> 5) & 0x3);
			pr_info("sii8240 : PLIM 0x%x\n", plim);

			if (mhl->pdata->vbus_present)
				mhl->pdata->vbus_present(false, plim);
		}
	} else {
		adopter_id = mhl->pdata->rx_cap.adopter_id |
					 mhl->pdata->rx_cap.adopter_id << 8;
		pr_info("sii8240: adopter id:%d, reserved:%d\n",
				adopter_id, mhl->pdata->rx_cap.reserved);

		if (adopter_id == 321 && mhl->pdata->rx_cap.reserved == 2) {
			if (mhl->pdata->vbus_present)
				mhl->pdata->vbus_present(false, 0x01);
		}
	}
}

static int cbus_cmd_send(struct mhl_tx *mhl, struct cbus_msg *msc_msg)
{
	int ret = 0;
	u8 startbit = 0;

	switch (msc_msg->command) {
	case MHL_SET_INT:
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x13, msc_msg->offset);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x14, msc_msg->buff[0]);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x12,  BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		break;

	case MHL_WRITE_STAT:
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x13, msc_msg->offset);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x14, msc_msg->buff[0]);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x12,  BIT_CBUS_MSC_WRITE_STAT_OR_SET_INT);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		break;

	case MHL_READ_DEVCAP:
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x13, msc_msg->offset);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x14, msc_msg->buff[0]);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x12,  BIT_CBUS_MSC_READ_DEVCAP);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (msc_msg->offset == MHL_DEV_CATEGORY_OFFSET)
			sii8246_setup_charging(mhl);

		break;

	case MHL_GET_STATE:
	case MHL_GET_VENDOR_ID:
	case MHL_SET_HPD:
	case MHL_CLR_HPD:
	case MHL_GET_SC1_ERRORCODE:
	case MHL_GET_DDC_ERRORCODE:
	case MHL_GET_MSC_ERRORCODE:
	case MHL_GET_SC3_ERRORCODE:
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x13, msc_msg->command);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x14, msc_msg->buff[0]);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x12,  BIT_CBUS_MSC_PEER_CMD);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		break;

	case MHL_MSC_MSG:
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x13, msc_msg->command);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x14, msc_msg->buff[0]);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0xBB,  msc_msg->buff[1]);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x12,  BIT_CBUS_MSC_MSG);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		break;

	case MHL_WRITE_BURST:
		//will add code later...
		startbit = BIT_CBUS_MSC_WRITE_BURST;
		break;

	default:
		goto exit_func;
		break;
	}

exit_func:

	if (ret < 0)
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);

	return ret;
}


static int cbus_cmd_done_reaction(struct mhl_tx *mhl, struct cbus_msg *msc_msg)
{
	int ret = 0;
	u8 cap_data;

	switch (msc_msg->command) {
	case MHL_WRITE_STAT:
		if ((msc_msg->offset == MHL_STATUS_REG_CONNECTED_RDY) &&
			(msc_msg->buff[0] == MHL_STATUS_DCAP_RDY)) {
			mutex_lock(&mhl->cbus_cmd_lock);
			msc_msg->command = MHL_SET_INT;
			msc_msg->offset = MHL_RCHANGE_INT;
			msc_msg->lenght = 0x01;
			msc_msg->buff[0] = MHL_INT_DCAP_CHG;
			mhl->cmd_rx_cnt += 1;
			mutex_unlock(&mhl->cbus_cmd_lock);
		}

		break;

	case MHL_READ_DEVCAP:
		switch (msc_msg->offset) {
		case MHL_CAP_MHL_VERSION:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.mhl_ver = cap_data;
			//printk(KERN_INFO "sii8246: %s():%d  MHL_CAP_MHL_VERSION:%x \n", __func__, __LINE__, cap_data);
			break;

		case MHL_DEV_CATEGORY_OFFSET:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			cap_data = (0x1F & cap_data);

			switch (cap_data) {
			case MHL_DEV_CAT_SOURCE:
				mhl->pdata->rx_cap.dev_type = MHL_DEV_CAT_SOURCE;
				//printk(KERN_INFO "sii8246: %s():%d  MHL_DEV_CAT_SOURCE \n", __func__, __LINE__);
				break;

			case MHL_SINK_W_POW:
				mhl->pdata->rx_cap.dev_type = MHL_SINK_W_POW;
				// printk(KERN_INFO "sii8246: %s():%d MHL_SINK_W_POW !\n", __func__, __LINE__);
				//mhl tx doesn't need power out
				break;

			case MHL_SINK_WO_POW:
				mhl->pdata->rx_cap.dev_type = MHL_SINK_WO_POW;
				//printk(KERN_INFO "sii8246: %s():%d MHL_SINK_WO_POW !\n", __func__, __LINE__);
				break;

			case MHL_DONGLE_W_POW:
				mhl->pdata->rx_cap.dev_type = MHL_DONGLE_W_POW;
				//printk(KERN_INFO "sii8246: %s():%d  MHL_DONGLE_W_POW!\n", __func__, __LINE__);
				break;

			case MHL_DONGLE_WO_POW:
				mhl->pdata->rx_cap.dev_type = MHL_DONGLE_WO_POW;
				//printk(KERN_INFO "sii8246: %s():%d MHL_DONGLE_WO_POW !\n", __func__, __LINE__);
				break;

			default:
				mhl->pdata->rx_cap.dev_type = cap_data;
				break;
			}

			break;

		case MHL_CAP_ADOPTER_ID_H:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.adopter_id = cap_data;
			mhl->pdata->rx_cap.adopter_id <<= 8;
			break;

		case MHL_CAP_ADOPTER_ID_L:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.adopter_id |= cap_data;
			break;

		case MHL_CAP_VID_LINK_MODE:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.vid_link_mode = (0x3F & cap_data);

			if (mhl->pdata->rx_cap.vid_link_mode & MHL_DEV_VID_LINK_SUPPRGB444) {
				//printk(KERN_INFO "sii8246: %s():%d  MHL_DEV_VID_LINK_SUPPRGB444  \n", __func__, __LINE__);
			}

			if (mhl->pdata->rx_cap.vid_link_mode & MHL_DEV_VID_LINK_SUPPYCBCR444) {
				//printk(KERN_INFO "sii8246: %s():%d  MHL_DEV_VID_LINK_SUPPYCBCR444  \n", __func__, __LINE__);
			}

			if (mhl->pdata->rx_cap.vid_link_mode & MHL_DEV_VID_LINK_SUPPYCBCR422) {
				//printk(KERN_INFO "sii8246: %s():%d  MHL_DEV_VID_LINK_SUPPYCBCR422  \n", __func__, __LINE__);
			}

			if (mhl->pdata->rx_cap.vid_link_mode & MHL_DEV_VID_LINK_SUPP_PPIXEL) {
				//printk(KERN_INFO "sii8246: %s():%d  MHL_DEV_VID_LINK_SUPP_PPIXEL  \n", __func__, __LINE__);
			}

			break;

		case MHL_CAP_AUD_LINK_MODE:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.aud_link_mode = (0x03 & cap_data);
			break;

		case MHL_CAP_VIDEO_TYPE:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.video_type = (0x8F & cap_data);
			break;

		case MHL_CAP_LOG_DEV_MAP:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.log_dev_map = cap_data;
			break;

		case MHL_CAP_BANDWIDTH:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.bandwidth = cap_data;
			break;

		case MHL_CAP_FEATURE_FLAG:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.feature_flag = (0x07 & cap_data);
			mhl->pdata->rx_cap.rcp_support = (mhl->pdata->rx_cap.feature_flag & MHL_FEATURE_RCP_SUPPORT) ? true : false;
			mhl->pdata->rx_cap.rap_support = (mhl->pdata->rx_cap.feature_flag & MHL_FEATURE_RAP_SUPPORT) ? true : false;
			mhl->pdata->rx_cap.sp_support = (mhl->pdata->rx_cap.feature_flag & MHL_FEATURE_SP_SUPPORT) ? true : false;
			break;

		case MHL_CAP_DEVICE_ID_H:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.device_id = cap_data;
			mhl->pdata->rx_cap.device_id <<= 8;
			break;

		case MHL_CAP_DEVICE_ID_L:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.device_id |= cap_data;
			break;

		case MHL_CAP_SCRATCHPAD_SIZE:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.scratchpad_size = cap_data;
			break;

		case MHL_CAP_INT_STAT_SIZE:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.int_stat_size = cap_data;
			break;

		case MHL_CAP_DEV_STATE:
		case MHL_CAP_RESERVED:
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x16, &cap_data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mhl->pdata->rx_cap.reserved = cap_data;

		default:
			break;
		}

		break;

	case MHL_MSC_MSG:
		if (msc_msg->buff[0] == MHL_MSC_MSG_RCPE) {
			mutex_lock(&mhl->cbus_cmd_lock);
			msc_msg->command = MHL_MSC_MSG;
			msc_msg->offset = 0x00;
			msc_msg->lenght = 0x02;
			msc_msg->buff[0] = MHL_MSC_MSG_RCPK;
			msc_msg->buff[1] = msc_msg->buff[2];
			mhl->cmd_rx_cnt += 1;
			mutex_unlock(&mhl->cbus_cmd_lock);
		}

		break;

	case MHL_WRITE_BURST:
	case MHL_SET_INT:
	default:
		break;
	}

exit_func:

	if (ret < 0)
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);

	return ret;
}


static void cbus_cmd_thread(struct work_struct *work)
{
	struct mhl_tx *mhl =
		container_of(work, struct mhl_tx, cbus_cmd_work);
	int ret = 0;
	struct cbus_msg *msc_msg, *next_msg;

pr_info("cbus_dbg: cbus_cmd_thread START  <=====\n");
	list_for_each_entry_safe(msc_msg, next_msg, &mhl->cbus_msg_list, list)  {
		mhl->msc_cmd_done_intr = MSC_SEND;
		ret = cbus_cmd_send(mhl, msc_msg);
		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}
pr_info("cbus_dbg: sent command %d & cmd_rx_cnt=%d=\n", msc_msg->command, mhl->cmd_rx_cnt);
		//wait_event_func:
		wait_event(mhl->cbus_cmd_wq,
				   ((mhl->msc_cmd_done_intr != MSC_SEND) || (mhl->msc_cmd_abord != false)
					|| (mhl->pdata->status.op_status != MHL_DISCOVERY_SUCCESS)));

		if (mhl->pdata->status.op_status != MHL_DISCOVERY_SUCCESS) {
			mutex_lock(&mhl->cbus_cmd_lock);
			mhl->cmd_rx_cnt = 0;
			mhl->cmd_tx_cnt = 0;
			mutex_unlock(&mhl->cbus_cmd_lock);
			printk("(line:%d) Not MHL_DISCOVERY_SUCCESS\n", (int) __LINE__);
			goto exit_func;
		}

		if (mhl->msc_cmd_done_intr == MSC_DONE_ACK) {
			mhl->cmd_tx_cnt += 1;
			ret = cbus_cmd_done_reaction(mhl, next_msg);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}
		} else if (mhl->msc_cmd_done_intr == MSC_DONE_NACK) {
			mhl->cmd_tx_cnt += 1;
		} else if (mhl->msc_cmd_abord == true) {
			mhl->msc_cmd_abord = false;
			printk(KERN_INFO "sii8246: %s():%d CMD ABOART WAIT over 2s !\n", __func__, __LINE__);
			msleep(2010);
		} else {
			mutex_lock(&mhl->cbus_cmd_lock);
			mhl->cmd_rx_cnt = 0;
			mhl->cmd_tx_cnt = 0;
			mutex_unlock(&mhl->cbus_cmd_lock);
			goto exit_func;
		}
		pr_info("cbus_dbg: Deletd command %d & cmd_rx_cnt=%d=\n", msc_msg->command, mhl->cmd_rx_cnt);
		list_del(&msc_msg->list);
		kfree(msc_msg);
	}

exit_func:

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
	}
pr_info("cbus_dbg: cbus_cmd_thread END  <=====\n");
	return ;
}


static int mhl_rgnd_check_func(struct mhl_tx *mhl)
{
	u8 int4status = 0, rgnd_value = 0, data = 0;
	int ret = 0;

	ret = I2C_WriteByte(mhl->pdata, PAGE_1, 0x3D, 0x35);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x11, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x12, 0x15);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x08, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = cbus_reset(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x10, 0xC1);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x19, 0x07);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x17, 0x03);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x1A, 0x20);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x22, 0xE0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x23, 0xC0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x24, 0xA0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x25, 0x80);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x26, 0x60);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x27, 0x40);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x28, 0x20);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x29, 0x10);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x4C, 0xD0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x4D, 0x02);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x80, 0x3C);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x85, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x13, 0x40);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x45, 0x06);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x4B, 0x06);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_2, 0x31, 0x0A);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA0, 0xD0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA1, 0xBC);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA3, 0x74);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA6, 0x03);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x2B, 0x01);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x90, BIT3 | BIT2, BIT2);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x91, 0xA5);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x94, 0x77);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x31, &data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x31, (data | 0x0C));

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA5, 0xBC);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA7, 0x08);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x95, 0x71);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x97, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x92, 0x86);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x93, 0x8C);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

#ifdef SFEATURE_INT_HPD_PP_MODE
	// push pull mode...
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x79, BIT6 | BIT2 | BIT1, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		return ret;
	}

#endif/*SFEATURE_INT_HPD_PP_MODE*/
	ret = hpd_drive_low(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	msleep(25);
	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x95, BIT6, 0x00);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x9A, BIT_DC9_CBUS_LOW_TO_DISCONNECT | BIT_DC9_WAKE_DRVFLT | BIT_DC9_DISC_PULSE_PROCEED);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x90, 0x27);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = init_cbus_regs(mhl);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x05, 0x04);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0x0D, 0x1C);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2CReadModify(mhl->pdata, PAGE_0, 0x90, BIT1, 0);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, &int4status);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		msleep(2);
		ret = I2C_ReadByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, &int4status);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}
	}

	if (int4status == 0x00) {
		goto exit_func;
	}

	//clear interrupt status...
	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, int4status);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (int4status & BIT_INTR4_RGND) {
		//read rgnd value...
		ret = I2C_ReadByte(mhl->pdata, PAGE_0, 0x99, &rgnd_value);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		rgnd_value = (rgnd_value & 0x03);

		if (rgnd_value == 0x02) {
			mhl->pdata->status.mhl_rgnd = true;
			mutex_lock(&mhl->pdata->mhl_status_lock);
			mhl->pdata->status.op_status = MHL_RX_CONNECTED;
			mutex_unlock(&mhl->pdata->mhl_status_lock);
			printk(KERN_INFO "sii8246: %s():%d MHL RGND(%x) !\n", __func__, __LINE__, rgnd_value);

			if (mhl->pdata->vbus_present)
				mhl->pdata->vbus_present(false, -1);

			msleep(100);

			if (mhl->pdata->vbus_present)
				mhl->pdata->vbus_present(true, 0x03);

			mhl->pdata->status.intr1_mask_value = 0;
			mhl->pdata->status.intr4_mask_value = (BIT_INTR4_MHL_EST | BIT_INTR4_NON_MHL_EST);
			ret = mhl_tx_interrupt_set(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return ret;
			}
		} else {
			mhl->pdata->status.mhl_rgnd = false;
			mutex_lock(&mhl->pdata->mhl_status_lock);
			mhl->pdata->status.op_status = MHL_USB_CONNECTED;
			mutex_unlock(&mhl->pdata->mhl_status_lock);
			printk(KERN_INFO "sii8246: %s():%d USB RGND(%x) !\n", __func__, __LINE__, rgnd_value);
		}
	}

exit_func:
	return ret;
}


static int get_write_burst_data(struct mhl_tx *mhl)
{
	int ret = 0;
	u8 tmp[16];

	memset(tmp, 0x00, 16);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
	}

	return ret;
}



static int tmds_control(struct mhl_tx *mhl, bool tmds_on)
{
	int ret = 0;

	if (tmds_on) {
		printk(KERN_INFO "sii8240: %s(ON):%d  !\n", __func__, __LINE__);
		ret = I2CReadModify(mhl->pdata, PAGE_0, 0x80, BIT4, BIT4);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA1, 0xBC);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}
	} else {
		printk(KERN_INFO "sii8240: %s(OFF):%d  !\n", __func__, __LINE__);
		ret = I2CReadModify(mhl->pdata, PAGE_0, 0x80, BIT4, 0);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA1, 0xC0);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}
	}

exit_func:
	return ret;
}


static int cbusprocsess_error(struct mhl_tx *mhl, u8 status, bool *abort)
{
	int ret = 0;
	u8 ddcabortreason = 0;
	u8 msc_abort_intstatus = 0;

	status &= (BIT_MSC_XFR_ABORT | BIT_DDC_ABORT | BIT_MSC_ABORT);
	*abort = false;

	if (status) {
		if (status & BIT_DDC_ABORT) {
			printk(KERN_INFO "sii8246: %s():%d BIT_DDC_ABORT \n", __func__, __LINE__);
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x0C, &ddcabortreason);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x57, ddcabortreason);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			*abort = true;
		}

		if (status & BIT_MSC_XFR_ABORT) {
			printk(KERN_INFO "sii8246: %s():%d BIT_MSC_XFR_ABORT \n", __func__, __LINE__);
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x0D, &msc_abort_intstatus);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x0D, msc_abort_intstatus);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			*abort = true;
		}

		if (status & BIT_MSC_ABORT) {
			printk(KERN_INFO "sii8246: %s():%d BIT_MSC_ABORT \n", __func__, __LINE__);
			ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, 0x0E, msc_abort_intstatus);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			if (msc_abort_intstatus) {
				*abort = true;
				printk(KERN_INFO "sii8246: %s():%d CBUS:: MSC Transfer ABORTED. Clearing 0x0D \n", __func__, __LINE__);

				if (BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL & msc_abort_intstatus) {
					printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_MSC_MT_ABORT_INT_MAX_FAIL \n", __func__, __LINE__);
				}

				if (BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR & msc_abort_intstatus) {
					printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_MSC_MT_ABORT_INT_PROTO_ERR \n", __func__, __LINE__);
				}

				if (BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT & msc_abort_intstatus) {
					printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_MSC_MT_ABORT_INT_TIMEOUT \n", __func__, __LINE__);
				}

				if (BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD & msc_abort_intstatus) {
					printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_MSC_MT_ABORT_INT_UNDEF_CMD \n", __func__, __LINE__);
				}

				if (BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT & msc_abort_intstatus) {
					printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_MSC_MT_ABORT_INT_MSC_MT_PEER_ABORT \n", __func__, __LINE__);
				}
			}
		}
	}

exit_func:
	return ret;
}


static int cbus_set_int_check(struct mhl_tx *mhl, u8 intr_0, u8 intr_1)
{
	int ret = 0;
	struct cbus_msg *msc_msg;

	if (MHL_INT_DCAP_CHG & intr_0) {
		mutex_lock(&mhl->cbus_cmd_lock);

		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}
		
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_DEV_CATEGORY_OFFSET;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: Added command %d & cmd_rx_cnt=%d=\n", msc_msg->command, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}
		mutex_unlock(&mhl->cbus_cmd_lock);
		
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}
		
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_FEATURE_FLAG;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_VID_LINK_MODE;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_MHL_VERSION;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}		
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_ADOPTER_ID_H;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_ADOPTER_ID_L;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_RESERVED;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
	}

	if (MHL_INT_DSCR_CHG & intr_0) {
		printk(KERN_INFO "Sii8240: %s():%d MHL_INT_DSCR_CHG !\n", __func__, __LINE__);
		ret = get_write_burst_data(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}
	}

	if (MHL_INT_REQ_WRT  & intr_0) {
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_SET_INT;
		msc_msg->offset = MHL_RCHANGE_INT;
		msc_msg->lenght = 0x01;
		msc_msg->buff[0] = MHL_INT_GRT_WRT;
		mhl->cmd_rx_cnt += 1;
		
		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
	}

	if (MHL_INT_GRT_WRT  & intr_0) {
		printk(KERN_INFO "Sii8240: %s():%d MHL_INT_GRT_WRT send  !\n", __func__, __LINE__);
	}

	// removed "else", since interrupts are not mutually exclusive of each other.
	if (MHL_INT_EDID_CHG & intr_1) {
		printk(KERN_INFO "Sii8240: %s():%d  MHL_INT_EDID_CHG \n", __func__, __LINE__);
		mhl->pdata->hpd_status = false;
		ret = tmds_control(mhl, false);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}

		ret = hpd_drive_low(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		msleep(110);
		mhl->pdata->hpd_status = true;
		ret = hpd_drive_high(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}
	}

exit_func:
	return ret;
}



static int cbus_status_cmd_check(struct mhl_tx *mhl, u8 status0, u8 status1)
{
	int ret = 0;
	struct cbus_msg *msc_msg;

	if ((MHL_STATUS_PATH_ENABLED & status1) &&
		!(mhl->pdata->status.linkmode & MHL_STATUS_PATH_ENABLED)) {
		mhl->pdata->status.linkmode |= MHL_STATUS_PATH_ENABLED;
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_WRITE_STAT;
		msc_msg->offset = MHL_STATUS_REG_LINK_MODE;
		msc_msg->lenght = 0x01;
		msc_msg->buff[0] = mhl->pdata->status.linkmode;
		mhl->cmd_rx_cnt += 1;
		
		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		printk(KERN_INFO "sii8240: %s():%d MHL_STATUS_PATH_ENABLED !\n", __func__, __LINE__);
	} else if (!(MHL_STATUS_PATH_ENABLED & status1) &&
			   (mhl->pdata->status.linkmode & MHL_STATUS_PATH_ENABLED)) {
		mhl->pdata->status.linkmode &= ~MHL_STATUS_PATH_ENABLED;
		mutex_lock(&mhl->cbus_cmd_lock);

		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_WRITE_STAT;
		msc_msg->offset = MHL_STATUS_REG_LINK_MODE;
		msc_msg->lenght = 0x01;
		msc_msg->buff[0] = mhl->pdata->status.linkmode;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		printk(KERN_INFO "sii8240: %s():%d MHL_STATUS_PATH_DISABLED !\n", __func__, __LINE__);
		ret = tmds_control(mhl, false);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}
	}

	if (MHL_STATUS_DCAP_RDY & status0) {
		printk(KERN_INFO "sii8240: %s():%d MHL_STATUS_DCAP_RDY  !\n", __func__, __LINE__);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_DEV_CATEGORY_OFFSET;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_FEATURE_FLAG;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_VID_LINK_MODE;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_MHL_VERSION;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
		mutex_lock(&mhl->cbus_cmd_lock);
		msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
		if (!msc_msg) {
			pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
			ret = -ENOMEM;
			goto exit_func;
		}	
		msc_msg->command = MHL_READ_DEVCAP;
		msc_msg->offset = MHL_CAP_RESERVED;
		msc_msg->lenght = 0;
		mhl->cmd_rx_cnt += 1;

		pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
		list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
		if (mhl->cmd_rx_cnt == 1) {
			pr_info("cbus_dbg: CBUS Work scheduled\n");
			queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
		}

		mutex_unlock(&mhl->cbus_cmd_lock);
	}

exit_func:
	return ret;
}

#ifdef SFEATURE_UCP_FEATURE
static void SiiMhlTxUCPKSend(struct mhl_tx *mhl, u8 keycode)
{
	struct cbus_msg *msc_msg;
	
	mutex_lock(&mhl->cbus_cmd_lock);
	msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
	if (!msc_msg) {
		pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
		goto exit_func;
	}
	msc_msg->command = MHL_MSC_MSG;
	msc_msg->offset = 0x00;
	msc_msg->lenght = 0x02;
	msc_msg->buff[0] = MHL_MSC_MSG_UCPK;
	msc_msg->buff[1] = keycode;
	mhl->cmd_rx_cnt += 1;

	pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
	list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
	if (mhl->cmd_rx_cnt == 1) {
		pr_info("cbus_dbg: CBUS Work scheduled\n");
		queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
	}
exit_func:
	mutex_unlock(&mhl->cbus_cmd_lock);
}

static void SiiMhlTxUCPESend(struct mhl_tx *mhl, u8 key_code)
{
	struct cbus_msg *msc_msg;

	mutex_lock(&mhl->cbus_cmd_lock);
	msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
	if (!msc_msg) {
		pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
		goto exit_func;
	}
	msc_msg->command = MHL_MSC_MSG;
	msc_msg->offset = 0x00;
	msc_msg->lenght = 0x02;
	msc_msg->buff[0] = MHL_MSC_MSG_UCPE;
	msc_msg->buff[1] = key_code;
	mhl->cmd_rx_cnt += 1;

	pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
	list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
	if (mhl->cmd_rx_cnt == 1) {
		pr_info("cbus_dbg: CBUS Work scheduled\n");
		queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
	}

exit_func:
	mutex_unlock(&mhl->cbus_cmd_lock);
}

#endif/*SFEATURE_UCP_FEATURE*/


static void SiiMhlTxRapkSend(struct mhl_tx *mhl)
{
	struct cbus_msg *msc_msg;

	mutex_lock(&mhl->cbus_cmd_lock);
	msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
	if (!msc_msg) {
		pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
		goto exit_func;
	}

	msc_msg->command = MHL_MSC_MSG;
	msc_msg->offset = 0x00;
	msc_msg->lenght = 0x02;
	msc_msg->buff[0] = MHL_MSC_MSG_RAPK;
	msc_msg->buff[1] = 0x00;
	mhl->cmd_rx_cnt += 1;

	pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
	list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
	if (mhl->cmd_rx_cnt == 1) {
		pr_info("cbus_dbg: CBUS Work scheduled\n");
		queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
	}
	
exit_func:
	mutex_unlock(&mhl->cbus_cmd_lock);
}

static void SiiMhlTxRcpkSend(struct mhl_tx *mhl, u8 keycode)
{
	struct cbus_msg *msc_msg;
	mutex_lock(&mhl->cbus_cmd_lock);
	msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
	if (!msc_msg) {
		pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
		goto exit_func;
	}
	msc_msg->command = MHL_MSC_MSG;
	msc_msg->offset = 0x00;
	msc_msg->lenght = 0x02;
	msc_msg->buff[0] = MHL_MSC_MSG_RCPK;
	msc_msg->buff[1] = keycode;
	mhl->cmd_rx_cnt += 1;

	pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
	list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
	if (mhl->cmd_rx_cnt == 1) {
		pr_info("cbus_dbg: CBUS Work scheduled\n");
		queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
	}

exit_func:
	mutex_unlock(&mhl->cbus_cmd_lock);
}

static void SiiMhlTxRcpeSend(struct mhl_tx *mhl, u8 erro_code, u8 key_code)
{
	struct cbus_msg *msc_msg;

	mutex_lock(&mhl->cbus_cmd_lock);
	msc_msg = kzalloc(sizeof(struct cbus_msg), GFP_KERNEL);
	if (!msc_msg) {
		pr_err("[ERROR] sii8246: failed to allocate cbus data\n");
		goto exit_func;
	}
	msc_msg->command = MHL_MSC_MSG;
	msc_msg->offset = 0x00;
	msc_msg->lenght = 0x03;
	msc_msg->buff[0] = MHL_MSC_MSG_RCPE;
	msc_msg->buff[1] = erro_code;
	msc_msg->buff[2] = key_code;
	mhl->cmd_rx_cnt += 1;

	pr_info("cbus_dbg: (%d) Added cmd_rx_cnt =%d=\n", __LINE__, mhl->cmd_rx_cnt);
	list_add_tail(&msc_msg->list, &mhl->cbus_msg_list);
	if (mhl->cmd_rx_cnt == 1) {
		pr_info("cbus_dbg: CBUS Work scheduled\n");
		queue_work(mhl->cbus_cmd_wqs, &mhl->cbus_cmd_work);
	}
	
exit_func:
	mutex_unlock(&mhl->cbus_cmd_lock);
}


static int cbus_msc_cmd_check(struct mhl_tx *mhl, u8 msc_cmd, u8 data)
{
	int ret = 0;

	switch (msc_cmd) {
	case	MHL_MSC_MSG_RAP:
		if (MHL_RAP_CONTENT_ON == data) {
			printk(KERN_INFO "Sii8240: %s():%d  MHL_RAP_CONTENT_ON \n", __func__, __LINE__);
#if 0
			ret = tmds_control(mhl, true);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
				return ret;
			}

#endif
		} else if (MHL_RAP_CONTENT_OFF == data) {
			printk(KERN_INFO "Sii8240: %s():%d  MHL_RAP_CONTENT_OFF \n", __func__, __LINE__);
			ret = tmds_control(mhl, false);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
				return ret;
			}
		}

		SiiMhlTxRapkSend(mhl);
		break;

	case	MHL_MSC_MSG_RCP:
		if (MHL_LOGICAL_DEVICE_MAP & rcpSupportTable [data & 0x7F]) {
			printk(KERN_INFO "Sii8240: %s():%d  SiiMhlTxRcpkSend Data[0x%02x] \n", __func__, __LINE__, data);
			SiiMhlTxRcpkSend(mhl, data);
		} else {
			printk(KERN_INFO "Sii8240: %s():%d  SiiMhlTxRcpeSend Data[0x%02x] \n", __func__, __LINE__, data);
			SiiMhlTxRcpeSend(mhl, RCPE_INEEFECTIVE_KEY_CODE, data);
		}

		break;

	case	MHL_MSC_MSG_RCPK:
		break;

	case	MHL_MSC_MSG_RCPE:
		break;

	case	MHL_MSC_MSG_RAPK:
		break;
#ifdef SFEATURE_UCP_FEATURE

	case	MHL_MSC_MSG_UCP:
		// Customer needs to check UCP code range...
#ifdef SFEATURE_SUPPORT_ASCII
		if ((data & 0x7F) <= 0x7F) {
			// need to send ascii data to framework.
			SiiMhlTxUCPKSend(mhl, data);
		} else {
			SiiMhlTxUCPESend(mhl, data);
		}

#else
		SiiMhlTxUCPESend(mhl, data);
#endif
		break;

	case	MHL_MSC_MSG_UCPK:
		break;

	case	MHL_MSC_MSG_UCPE:
		break;
#endif/*SFEATURE_UCP_FEATURE*/

	default:
		// Any freak value here would continue with no event to app
		break;
	}

	return ret;
}


static int cbus_link_check(struct mhl_tx *mhl)
{
	int ret = 0, i = 0;
	u8 cbus_intr, data;
	u8 status[4];

	memset(status, 0x00, 4);
	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, CBUS_INTR_STATUS_ADD, &cbus_intr);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (cbus_intr) {
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR_STATUS_ADD, cbus_intr);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (cbus_intr & (BIT_CBUS_INTR_MSC_MT_ABORT | BIT_CBUS_INTR_MSC_MR_ABORT | BIT_CBUS_INTR_DDC_ABORT)) {
			ret = cbusprocsess_error(mhl, cbus_intr, &mhl->msc_cmd_abord);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			if (mhl->msc_cmd_abord == true) {
				printk(KERN_INFO "sii8246: %s():%d (mhl->msc_cmd_abord == true) !\n", __func__, __LINE__);

				if (waitqueue_active(&mhl->cbus_cmd_wq)) {
					wake_up(&mhl->cbus_cmd_wq);
				}
			}
		}

		if (cbus_intr & BIT_CBUS_INTR_CONNECTION_CHG) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR_CONNECTION_CHG !\n", __func__, __LINE__);
		}

		if (cbus_intr & BIT_CBUS_INTR_CEC_ABORT) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR_CEC_ABORT !\n", __func__, __LINE__);
		}

		if (cbus_intr & BIT_CBUS_INTR_MSC_MSG_CMD_RCV) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR_MSC_MSG_CMD_RCV !\n", __func__, __LINE__);
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x18, &status[0]);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x19, &status[1]);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			ret = cbus_msc_cmd_check(mhl, status[0], status[1]);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
				return ret;
			}
		}

		if (cbus_intr & BIT_CBUS_INTR_MSC_CMD_DONE) {
			//printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR_MSC_CMD_DONE !\n", __func__, __LINE__);
			mhl->msc_cmd_done_intr = MSC_DONE_ACK;

			if (waitqueue_active(&mhl->cbus_cmd_wq)) {
				wake_up(&mhl->cbus_cmd_wq);
			}
		}
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, CBUS_INTR0_STATUS_ADD, &cbus_intr);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (cbus_intr) {
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR0_STATUS_ADD, cbus_intr);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (cbus_intr & BIT_CBUS_INTR0_HPD_RCV) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR0_HPD_RCV !\n", __func__, __LINE__);
			ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, 0x0D, &data);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			if (data & BIT6) {
				printk(KERN_INFO "sii8246: %s():%d mhl->pdata->hpd_status = true !\n", __func__, __LINE__);
				mhl->pdata->hpd_status = true;
				ret = hpd_drive_high(mhl);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
					goto exit_func;
				}

				ret = tmds_control(mhl, true);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
					return ret;
				}
			} else {
				printk(KERN_INFO "sii8246: %s():%d mhl->pdata->hpd_status = false !\n", __func__, __LINE__);
				mhl->pdata->hpd_status = false;
				ret = tmds_control(mhl, false);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
					return ret;
				}

				ret = hpd_drive_low(mhl);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
					goto exit_func;
				}
			}
		}

		if (cbus_intr & BIT_CBUS_INTR0_MSC_DONE_NACK) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR0_MSC_DONE_NACK !\n", __func__, __LINE__);
			mhl->msc_cmd_done_intr = MSC_DONE_NACK;

			if (waitqueue_active(&mhl->cbus_cmd_wq)) {
				wake_up(&mhl->cbus_cmd_wq);
			}
		}
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, CBUS_INTR1_STATUS_ADD, &cbus_intr);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (cbus_intr) {
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR1_STATUS_ADD, cbus_intr);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (cbus_intr & BIT_CBUS_INTR1_MSC_ABORT) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR1_MSC_ABORT !\n", __func__, __LINE__);
		}

		if (cbus_intr & BIT_CBUS_INTR1_MSC_CMD_ABORT) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR1_MSC_CMD_ABORT !\n", __func__, __LINE__);
		}
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, CBUS_INTR2_STATUS_ADD, &cbus_intr);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (cbus_intr) {
		ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, CBUS_INTR2_STATUS_ADD, cbus_intr);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (cbus_intr & BIT_CBUS_INTR2_MSC_WRITE_BURST_RCV) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR2_MSC_WRITE_BURST_RCV !\n", __func__, __LINE__);
		}

		if (cbus_intr & BIT_CBUS_INTR2_MSC_HEARTBEAT_MAX_FAIL) {
			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR2_MSC_HEARTBEAT_MAX_FAIL !\n", __func__, __LINE__);
		}

		if (cbus_intr & BIT_CBUS_INTR2_MSC_SET_INT) {
			for (i = 0; i < 4; i++) {
				ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, (0xA0 + i), &status[i]);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
					goto exit_func;
				}

				ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, (0xA0 + i), 0xFF);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
					goto exit_func;
				}
			}

			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR2_MSC_SET_INT !\n", __func__, __LINE__);
			ret = cbus_set_int_check(mhl, status[0], status[1]);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
				return ret;
			}
		}

		if (cbus_intr & BIT_CBUS_INTR2_MSC_WRITE_STAT_RCV) {
			for (i = 0; i < 4; i++) {
				ret = I2C_ReadByte(mhl->pdata, PAGE_CBUS, (0xB0 + i), &status[i]);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
					goto exit_func;
				}

				ret = I2C_WriteByte(mhl->pdata, PAGE_CBUS, (0xB0 + i), 0xFF);

				if (ret < 0) {
					printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
					goto exit_func;
				}
			}

			printk(KERN_INFO "sii8246: %s():%d BIT_CBUS_INTR2_MSC_WRITE_STAT_RCV !\n", __func__, __LINE__);
			ret = cbus_status_cmd_check(mhl, status[0], status[1]);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8240: %s():%d failed !\n", __func__, __LINE__);
				return ret;
			}
		}
	}

exit_func:
	return ret;
}


static int mhl_tx_intr_handle(struct mhl_tx *mhl)
{
	int ret = 0;
	u8 data = 0, intr_data = 0;

	ret = I2C_ReadByte(mhl->pdata, PAGE_0, INTR1_STATUS_ADD, &intr_data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (intr_data == 0x00) {
		goto intr4_check;
	}

	//clear interrupt status...
	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR1_STATUS_ADD, intr_data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	ret = I2C_ReadByte(mhl->pdata, PAGE_0, 0x09, &data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (intr_data & BIT_INTR1_RSEN_CHG) {
		if (data & 0x04) {
			printk(KERN_INFO "sii8246: %s():%d  RSEN event high \n", __func__, __LINE__);
		}

		msleep(100);
		ret = I2C_ReadByte(mhl->pdata, PAGE_0, 0x09, &data);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (data & 0x04) {
			//printk(KERN_INFO "sii8246: %s():%d   \n", __func__, __LINE__);
		} else {
			ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA0, 0xD0);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			mutex_lock(&mhl->pdata->mhl_status_lock);
			mhl->pdata->status.op_status = MHL_RX_DISCONNECTED;
			mutex_unlock(&mhl->pdata->mhl_status_lock);
			printk(KERN_INFO "sii8246: %s():%d  RSEN event2 low \n", __func__, __LINE__);
			goto exit_func;
		}
	}

	if (intr_data & BIT_INTR1_HPD_CHG) {
		if (data & 0x02) {
			printk(KERN_INFO "sii8246: %s():%d  HPD event high \n", __func__, __LINE__);
			mhl->pdata->hpd_status = true;
			ret = hpd_drive_high(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}
		} else {
			printk(KERN_INFO "sii8246: %s():%d  HPD event low \n", __func__, __LINE__);
			mhl->pdata->hpd_status = false;
			ret = hpd_drive_low(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}
		}
	}

intr4_check:
	ret = I2C_ReadByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, &intr_data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (intr_data == 0x00) {
		goto intr5_check;
	}

	//clear interrupt status...
	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR4_STATUS_ADD, intr_data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (intr_data & BIT_INTR4_CBUS_LKOUT) {
		printk(KERN_INFO "sii8246: %s():%d  BIT_INTR4_CBUS_LKOUT \n", __func__, __LINE__);
		ret = forceusbidswitchopen(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}

		ret = releaseubidswichopen(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			return ret;
		}
	}

	if (intr_data & BIT_INTR4_CBUS_DISCONNECT) {
		printk(KERN_INFO "sii8246: %s():%d  BIT_INTR4_CBUS_DISCONNECT \n", __func__, __LINE__);
		ret = I2C_WriteByte(mhl->pdata, PAGE_0, 0xA0, 0xD0);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		mutex_lock(&mhl->pdata->mhl_status_lock);
		mhl->pdata->status.op_status = MHL_RX_DISCONNECTED;
		mutex_unlock(&mhl->pdata->mhl_status_lock);
		goto exit_func;
	}

intr5_check:
	ret = I2C_ReadByte(mhl->pdata, PAGE_0, INTR5_STATUS_ADD, &intr_data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (intr_data == 0x00) {
		goto exit_func;
	}

	//clear interrupt status...
	ret = I2C_WriteByte(mhl->pdata, PAGE_0, INTR5_STATUS_ADD, intr_data);

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
		goto exit_func;
	}

	if (intr_data & BIT_INTR5_CKDT_CHANGE) {
		printk(KERN_INFO "sii8246:  BIT_INTR5_CKDT_CHANGE %s():%d !\n", __func__, __LINE__);
	}

exit_func:
	return ret;
}


static irqreturn_t mhl_irq_thread(int irq, void *data)
{
	struct mhl_tx *mhl = data;
	int ret = 0;
	u8 status = 0;

	if (mhl->power_state == false) {
		pr_info("mhl_irq_thread: MHL IC Turned Off Return Irq Handle\n");
		return IRQ_HANDLED ;
	}

	mutex_lock(&mhl->pdata->mhl_status_lock);
	status = mhl->pdata->status.op_status;
	mutex_unlock(&mhl->pdata->mhl_status_lock);

	if (status == NO_MHL_STATUS) {
		printk(KERN_INFO "sii8246: %s():%d !\n", __func__, __LINE__);
		return IRQ_HANDLED;
	}

	switch (status) {
	case MHL_READY_RGND_DETECT:
		//printk(KERN_INFO "sii8246: MHL_READY_RGND_DETECT %s():%d \n", __func__, __LINE__);
		ret = mhl_rgnd_check_func(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			return IRQ_HANDLED;
		}

		if (mhl->pdata->status.op_status == MHL_USB_CONNECTED) {
			mutex_lock(&mhl->cbus_cmd_lock);
			mhl->cmd_rx_cnt = 0;
			mhl->cmd_tx_cnt = 0;
			mutex_unlock(&mhl->cbus_cmd_lock);

			if (waitqueue_active(&mhl->cbus_cmd_wq)) {
				wake_up(&mhl->cbus_cmd_wq);
			}

			mhl->pdata->status.op_status = NO_MHL_STATUS;
			hw_resst(mhl->pdata);
			mhl->pdata->status.linkmode = 0x03;
			mhl->pdata->hpd_status = false;
			mhl->pdata->status.mhl_rgnd = false;
			mhl->pdata->status.cbus_connected = false;
			mhl->pdata->status.connected_ready = false;
			                                                                     mhl->msc_cmd_abord = false;
			ret = mhl_init_func(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			mhl->pdata->status.intr1_mask_value = 0;
			mhl->pdata->status.intr2_mask_value = 0;
			mhl->pdata->status.intr3_mask_value = 0;
			mhl->pdata->status.intr4_mask_value = BIT_INTR4_RGND;
			mhl->pdata->status.intr5_mask_value = 0;
			mhl->pdata->status.intr7_mask_value = 0;
			mhl->pdata->status.intr8_mask_value = 0;
			mhl->pdata->status.intr_cbus_mask_value = 0;
			mhl->pdata->status.intr_cbus0_mask_value = 0;
			mhl->pdata->status.intr_cbus1_mask_value = 0;
			mhl->pdata->status.intr_cbus2_mask_value = 0;
			ret = mhl_tx_interrupt_set(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			ret = TX_GO2D3(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			return IRQ_HANDLED;
		}

		break;

	case MHL_RX_CONNECTED:
		printk(KERN_INFO "sii8246: MHL_RX_CONNECTED %s():%d \n", __func__, __LINE__);
		ret = mhl_rx_connected_func(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		if (mhl->pdata->status.op_status == MHL_DISCOVERY_SUCCESS) {
			ret = mhl_tx_intr_handle(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}

			ret = cbus_link_check(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				goto exit_func;
			}
		}

		if (mhl->pdata->status.op_status == MHL_DISCOVERY_FAIL) {
			printk(KERN_INFO "sii8246: MHL_DISCOVERY_FAIL %s():%d \n", __func__, __LINE__);
			mutex_lock(&mhl->cbus_cmd_lock);
			mhl->cmd_rx_cnt = 0;
			mhl->cmd_tx_cnt = 0;
			mutex_unlock(&mhl->cbus_cmd_lock);

			if (mhl->pdata->vbus_present)
				mhl->pdata->vbus_present(false, -1);

			if (waitqueue_active(&mhl->cbus_cmd_wq)) {
				wake_up(&mhl->cbus_cmd_wq);
			}

			mhl->pdata->status.op_status = NO_MHL_STATUS;
			hw_resst(mhl->pdata);
			mhl->pdata->status.linkmode = 0x03;
			mhl->pdata->hpd_status = false;
			mhl->pdata->status.mhl_rgnd = false;
			mhl->pdata->status.cbus_connected = false;
			mhl->pdata->status.connected_ready = false;
																				 mhl->msc_cmd_abord = false;
			ret = mhl_init_func(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			mhl->pdata->status.intr1_mask_value = 0;
			mhl->pdata->status.intr2_mask_value = 0;
			mhl->pdata->status.intr3_mask_value = 0;
			mhl->pdata->status.intr4_mask_value = BIT_INTR4_RGND;
			mhl->pdata->status.intr5_mask_value = 0;
			mhl->pdata->status.intr7_mask_value = 0;
			mhl->pdata->status.intr8_mask_value = 0;
			mhl->pdata->status.intr_cbus_mask_value = 0;
			mhl->pdata->status.intr_cbus0_mask_value = 0;
			mhl->pdata->status.intr_cbus1_mask_value = 0;
			mhl->pdata->status.intr_cbus2_mask_value = 0;
			ret = mhl_tx_interrupt_set(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			ret = TX_GO2D3(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			return IRQ_HANDLED;
		} else if (mhl->pdata->status.op_status == MHL_DISCOVERY_ON) {
			if (waitqueue_active(&mhl->cbus_cmd_wq)) {
				wake_up(&mhl->cbus_cmd_wq);
			}
		} else if (mhl->pdata->status.op_status == MHL_RX_CONNECTED) {
			printk(KERN_INFO "sii8246: %s():%d MHL_RX_CONNECTED \n", __func__, __LINE__);
		}

		break;

	case MHL_DISCOVERY_SUCCESS:
		ret = mhl_tx_intr_handle(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		mutex_lock(&mhl->pdata->mhl_status_lock);
		status = mhl->pdata->status.op_status;
		mutex_unlock(&mhl->pdata->mhl_status_lock);

		if (status == MHL_RX_DISCONNECTED) {
			mutex_lock(&mhl->cbus_cmd_lock);
			mhl->cmd_rx_cnt = 0;
			mhl->cmd_tx_cnt = 0;
			mutex_unlock(&mhl->cbus_cmd_lock);

			if (waitqueue_active(&mhl->cbus_cmd_wq)) {
				wake_up(&mhl->cbus_cmd_wq);
			}

			if (mhl->pdata->vbus_present)
				mhl->pdata->vbus_present(false, -1);

			mhl->pdata->status.op_status = NO_MHL_STATUS;
			hw_resst(mhl->pdata);
			mhl->pdata->status.linkmode = 0x03;
			mhl->pdata->hpd_status = false;
			mhl->pdata->status.mhl_rgnd = false;
			mhl->pdata->status.cbus_connected = false;
			mhl->pdata->status.connected_ready = false;
																				 mhl->msc_cmd_abord = false;
			ret = mhl_init_func(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			mhl->pdata->status.intr1_mask_value = 0;
			mhl->pdata->status.intr2_mask_value = 0;
			mhl->pdata->status.intr3_mask_value = 0;
			mhl->pdata->status.intr4_mask_value = BIT_INTR4_RGND;
			mhl->pdata->status.intr5_mask_value = 0;
			mhl->pdata->status.intr7_mask_value = 0;
			mhl->pdata->status.intr8_mask_value = 0;
			mhl->pdata->status.intr_cbus_mask_value = 0;
			mhl->pdata->status.intr_cbus0_mask_value = 0;
			mhl->pdata->status.intr_cbus1_mask_value = 0;
			mhl->pdata->status.intr_cbus2_mask_value = 0;
			ret = mhl_tx_interrupt_set(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			ret = TX_GO2D3(mhl);

			if (ret < 0) {
				printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
				return IRQ_HANDLED;
			}

			return IRQ_HANDLED;
		}

		ret = cbus_link_check(mhl);

		if (ret < 0) {
			printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
			goto exit_func;
		}

		break;

	default:
		break;
	}

exit_func:

	if (ret < 0) {
		printk(KERN_INFO "[ERROR]sii8246: %s():%d failed !\n", __func__, __LINE__);
	}

	return IRQ_HANDLED;
}


#ifdef CONFIG_OF
/*FIXME, need to use more common/proper function
  for checking a VBUS regardless of H/W charger IC*/

static bool check_vbus_present(void)
{
	bool ret = true;
	union power_supply_propval value;
	psy_do_property("sec-charger", get, POWER_SUPPLY_PROP_ONLINE, value);
	pr_info("sec-charger : %d\n", value.intval);

	if (value.intval == POWER_SUPPLY_TYPE_BATTERY
		|| value.intval == POWER_SUPPLY_TYPE_WIRELESS)
		ret = false;

	pr_info("VBUS : %s in %s\n", ret ? "IN" : "OUT", __func__);
	return ret;
}

#ifdef CONFIG_SAMSUNG_LPM_MODE
extern int poweroff_charging;
#endif

static void sii8246_charger_mhl_cb(bool otg_enable, int charger)
{
	struct mhl_tx *mhl = dev_get_drvdata(sii8246_mhldev);
	union power_supply_propval value;
	int i, ret = 0;
	struct power_supply *psy;
	int current_cable_type = POWER_SUPPLY_TYPE_MISC;
	int sub_type = ONLINE_SUB_TYPE_MHL;
	int power_type = ONLINE_POWER_TYPE_UNKNOWN;
#ifdef CONFIG_MFD_MAX77803
	int muic_cable_type = max77803_muic_get_charging_type();
	pr_info("%s: muic cable_type = %d, otg_enable : %d, charger: %d\n",
			__func__, muic_cable_type, otg_enable, charger);

	switch (muic_cable_type) {
	case CABLE_TYPE_SMARTDOCK_MUIC:
	case CABLE_TYPE_SMARTDOCK_TA_MUIC:
	case CABLE_TYPE_SMARTDOCK_USB_MUIC:
		return;

	default:
		break;
	}
#else
	pr_info("%s, otg_en:%d, charger:%d\n", __func__, otg_enable, charger);
#endif
	if (charger == 0x00) {
		pr_info("%s() TA charger 500mA\n", __func__);
		power_type = ONLINE_POWER_TYPE_MHL_500;
	} else if (charger == 0x01) {
		pr_info("%s() TA charger 900mA\n", __func__);
		power_type = ONLINE_POWER_TYPE_MHL_900;
	} else if (charger == 0x02) {
		pr_info("%s() TA charger 1500mA\n", __func__);
		power_type = ONLINE_POWER_TYPE_MHL_1500;
	} else if (charger == 0x03) {
		pr_info("%s() USB charger\n", __func__);
		power_type = ONLINE_POWER_TYPE_USB;
	} else
		current_cable_type = POWER_SUPPLY_TYPE_BATTERY;

	if (otg_enable) {
		if (!check_vbus_present()) {
#ifdef CONFIG_SAMSUNG_LPM_MODE

			if (!poweroff_charging) {
#else
			{
#endif

				if (mhl->pdata->muic_otg_set)
					mhl->pdata->muic_otg_set(true);

				power_type = ONLINE_POWER_TYPE_UNKNOWN;
				current_cable_type = POWER_SUPPLY_TYPE_OTG;
			}
		}
	} else {
		if (mhl->pdata->muic_otg_set)
			mhl->pdata->muic_otg_set(false);
	}

	for (i = 0; i < 10; i++) {
		psy = power_supply_get_by_name("battery");

		if (psy)
			break;
	}

	if (i == 10) {
		pr_err("[ERROR] %s: fail to get battery ps\n", __func__);
		return;
	}

	value.intval = current_cable_type << ONLINE_TYPE_MAIN_SHIFT
				   | sub_type << ONLINE_TYPE_SUB_SHIFT
				   | power_type << ONLINE_TYPE_PWR_SHIFT;
	ret = psy->set_property(psy, POWER_SUPPLY_PROP_ONLINE, &value);

	if (ret) {
		pr_err("[ERROR] %s: fail to set power_suppy ONLINE property(%d)\n",
			   __func__, ret);
		return;
	}
}

static void of_sii8246_gpio_init(struct mhl_platform_data *pdata)
{
	if (pdata->gpio_mhl_en > 0) {
		if (gpio_request(pdata->gpio_mhl_en, "mhl_en")) {
			pr_err("[ERROR] %s: unable to request gpio_mhl_en [%d]\n",
				   __func__, pdata->gpio_mhl_en);
			return;
		}

		if (gpio_direction_output(pdata->gpio_mhl_en, 0)) {
			pr_err("[ERROR] %s: unable to  gpio_mhl_en low[%d]\n",
				   __func__, pdata->gpio_mhl_en);
			return;
		}
	}

	if (pdata->gpio_mhl_reset > 0) {
		if (gpio_request(pdata->gpio_mhl_reset, "mhl_reset")) {
			pr_err("[ERROR] %s: unable to request gpio_mhl_reset [%d]\n",
				   __func__, pdata->gpio_mhl_reset);
			return;
		}

		if (gpio_direction_output(pdata->gpio_mhl_reset, 0)) {
			pr_err("[ERROR] %s: unable to gpio_mhl_reset low[%d]\n",
				   __func__, pdata->gpio_mhl_reset);
			return;
		}
	}
}

static void of_sii8246_gpio_config(void)
{
	struct mhl_tx *mhl = dev_get_drvdata(sii8246_mhldev);
	gpio_tlmm_config(GPIO_CFG(mhl->pdata->gpio_mhl_reset, 0, GPIO_CFG_OUTPUT,
							  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
	gpio_tlmm_config(GPIO_CFG(mhl->pdata->gpio_mhl_en, 0, GPIO_CFG_INPUT,
							  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), 1);
}

static void of_sii8246_hw_onoff(bool onoff)
{
	int ret;
	struct mhl_tx *mhl = dev_get_drvdata(sii8246_mhldev);
	struct mhl_platform_data *pdata = mhl->pdata;
	pr_info("%s: Onoff: %d\n", __func__, onoff);

	if (onoff) {
		if (mhl->pdata->gpio_mhl_en > 0)
			gpio_set_value_cansleep(mhl->pdata->gpio_mhl_en, onoff);

		if (pdata->vcc_1p2v) {
			ret = regulator_set_voltage(pdata->vcc_1p2v, 1200000, 1200000);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_1p2v set_vtg failed rc\n");
				return;
			}

			ret = regulator_enable(pdata->vcc_1p2v);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_1p2v enable failed rc\n");
				return;
			}
		}

		if (pdata->vcc_1p8v) {
			ret = regulator_set_voltage(pdata->vcc_1p8v, 1800000, 1800000);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc 1p8v set_vtg failed rc\n");
				goto err_regulator_1p8v;
			}

			ret = regulator_enable(pdata->vcc_1p8v);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc 1p8v enable failed rc\n");
				goto err_regulator_1p8v;
			}
		}

		if (pdata->vcc_3p3v) {
			ret = regulator_set_voltage(pdata->vcc_3p3v, 3300000, 3300000);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_3p3v set_vtg failed rc\n");
				goto err_regulator_3p3v;
			}

			ret = regulator_enable(pdata->vcc_3p3v);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_3p3v enable failed rc\n");
				goto err_regulator_3p3v;
			}
		}
	} else {
		if (mhl->pdata->gpio_mhl_en > 0)
			gpio_set_value_cansleep(mhl->pdata->gpio_mhl_en, onoff);

		if (pdata->vcc_1p2v) {
			ret = regulator_disable(pdata->vcc_1p2v);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_1p2v disable failed rc\n");
				return;
			}
		}

		if (pdata->vcc_1p8v) {
			ret = regulator_disable(pdata->vcc_1p8v);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_1p8v disable failed rc\n");
				return;
			}
		}

		if (pdata->vcc_3p3v) {
			ret = regulator_disable(pdata->vcc_3p3v);

			if (unlikely(ret < 0)) {
				pr_err("[ERROR] regulator vcc_3pv3 disable failed rc\n");
				return;
			}
		}

		usleep_range(10000, 20000);

		if (mhl->pdata->gpio_mhl_reset > 0)
			gpio_set_value_cansleep(mhl->pdata->gpio_mhl_reset, 0);
	}

	return;
err_regulator_3p3v:

	if (pdata->vcc_1p8v)
		regulator_disable(pdata->vcc_1p8v);

err_regulator_1p8v:

	if (pdata->vcc_1p2v)
		regulator_disable(pdata->vcc_1p2v);
}


#if defined (CONFIG_MACH_HLTEDCM)
/*
   This is only for H-DCM HW REV0.9 and REV1.0.
   In those schematics the FELICA I2C pull up voltage source are connected to V_MHL
   which is different voltage source from previous HW revision.
   This means whenever the FELICA is working the V_MHL is controlled by FELICA driver
   and this is why the duplicated extern function is declared and utilized here
   even this is kind of prohibited way to make a code.
   This has no harmful factor on the other models. This is only for H-DCM HW REV0.9/1.0.
 */
void of_sii8246_hw_poweron(bool enable)
{
	if (g_sii8240 && g_sii8240->pdata && g_sii8240->pdata->power)
		g_sii8240->pdata->power(enable);
	else
		pr_err("%s: some pointer is not initialized (either pdata or pdata->power)\n" , __func__);
}

#endif

static void of_sii8246_hw_reset(void)
{
	struct mhl_tx *mhl = dev_get_drvdata(sii8246_mhldev);
	pr_info("%s: hw_reset\n" , __func__);
	usleep_range(10000, 20000);
	gpio_set_value_cansleep(mhl->pdata->gpio_mhl_reset, 1);
	usleep_range(5000, 20000);
	gpio_set_value_cansleep(mhl->pdata->gpio_mhl_reset, 0);
	usleep_range(10000, 20000);
	gpio_set_value_cansleep(mhl->pdata->gpio_mhl_reset, 1);
	msleep(30);
}


static int of_sii8246_parse_dt(struct mhl_platform_data *pdata)
{
	struct device_node *np = pdata->simgC8_tx_client->dev.of_node;
	struct device *pdev = &pdata->simgC8_tx_client->dev;

	pdata->gpio_mhl_irq = of_get_named_gpio_flags(np,
						  "sii8246,gpio_mhl_irq", 0, NULL);

	if (pdata->gpio_mhl_irq > 0)
		pr_info("gpio: mhl_irq = %d\n", pdata->gpio_mhl_irq);

	pdata->gpio_mhl_reset = of_get_named_gpio_flags(np,
							"sii8246,gpio_mhl_reset", 0, NULL);

	if (pdata->gpio_mhl_reset > 0)
		pr_info("gpio: mhl_reset = %d\n", pdata->gpio_mhl_reset);

	pdata->gpio_mhl_wakeup = of_get_named_gpio_flags(np,
							 "sii8246,gpio_mhl_wakeup", 0, NULL);

	if (pdata->gpio_mhl_wakeup > 0)
		pr_info("gpio: mhl_wakeup = %d\n", pdata->gpio_mhl_wakeup);

	pdata->gpio_mhl_scl = of_get_named_gpio_flags(np,
						  "sii8246,gpio_mhl_scl", 0, NULL);

	if (pdata->gpio_mhl_scl > 0)
		pr_info("gpio: mhl_scl = %d\n",
				pdata->gpio_mhl_scl);

	pdata->gpio_mhl_sda = of_get_named_gpio_flags(np,
						  "sii8246,gpio_mhl_sda", 0, NULL);

	if (pdata->gpio_mhl_sda > 0)
		pr_info("gpio: mhl_sda = %d\n", pdata->gpio_mhl_sda);

	pdata->gpio_mhl_en = of_get_named_gpio_flags(np,
						 "sii8246,gpio_mhl_en", 0, NULL);

	if (pdata->gpio_mhl_en > 0)
		pr_info("gpio: mhl_en = %d\n", pdata->gpio_mhl_en);

	if (!of_property_read_u32(np, "sii8246,swing_level", &pdata->swing_level))
		pr_info("swing_level = 0x%X\n", pdata->swing_level);

	pdata->vcc_1p2v = regulator_get(pdev, "vcc_1p2v");

	if (IS_ERR(pdata->vcc_1p2v)) {
		pr_err("sii8246,vcc_1p2v is not exist in device tree\n");
		pdata->vcc_1p2v = NULL;
	}

	pdata->vcc_1p8v = regulator_get(pdev, "vcc_1p8v");

	if (IS_ERR(pdata->vcc_1p8v)) {
		pr_err("sii8246,vcc_1p8v is not exist in device tree\n");
		pdata->vcc_1p8v = NULL;
	}

	pdata->vcc_3p3v = regulator_get(pdev, "vcc_3p3v");

	if (IS_ERR(pdata->vcc_3p3v)) {
		pr_err("sii8246,vcc_3p3v is not exist in device tree\n");
		pdata->vcc_3p3v = NULL;
	}

	return 0;
}


static int __devinit of_sii8246_probe_dt(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	const struct i2c_device_id *id_table = client->driver->id_table;
	struct mhl_platform_data *pdata = NULL;
	u32 client_id = -1;

	if (!client->dev.of_node) {
		dev_err(&client->dev, "sii8246: Client node not-found\n");
		return -1;
	}

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	/* going to use block read/write, so check for this too */
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_I2C_BLOCK))
		return -EIO;

	if (of_property_read_u32(client->dev.of_node, "sii8246,client_id", &client_id))
		dev_err(&client->dev, "Wrong Client_id# %d", client_id);

	if (0 == client_id) {
		pdata = kzalloc(sizeof(struct mhl_platform_data), GFP_KERNEL);

		if (!pdata) {
			dev_err(&client->dev, "failed to allocate driver data\n");
			return -ENOMEM;
		}

		pdata->simgC8_tx_client = client;
		pdata->power = of_sii8246_hw_onoff;
		pdata->hw_reset = of_sii8246_hw_reset;
		pdata->gpio_cfg = of_sii8246_gpio_config;
		pdata->vbus_present = sii8246_charger_mhl_cb;
#ifdef CONFIG_MFD_MAX77803
		pdata->muic_otg_set = muic_otg_control;
#endif
		of_sii8246_parse_dt(pdata);
		of_sii8246_gpio_init(pdata);
		client->dev.platform_data = pdata;
		pr_info("debug_message : %s, %d", __func__, __LINE__);
		simgC8_probe(client, id_table);
	} else if (g_mhl_tx) {
		client->dev.platform_data = g_mhl_tx->pdata;

		if (1 == client_id)
			simg72_probe(client, id_table);
		else if (2 == client_id)
			simg7A_probe(client, id_table);
		else if (3 == client_id)
			simg92_probe(client, id_table);
	}

	return 0;
}
#endif /* CONFIG_OF */

static int __devinit simg72_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tx_page0 *page0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	page0 = kzalloc(sizeof(struct tx_page0), GFP_KERNEL);

	if (!page0) {
		dev_err(&client->dev, " page0 failed to allocate driver data\n");
		return -ENOMEM;
	}

	page0->pdata = client->dev.platform_data;
	page0->pdata->simg72_tx_client = client;

	if (!page0->pdata) {
		printk(KERN_INFO"\n SIMG72 no platform data \n");
		kfree(page0);
		return -EINVAL;
	}

	i2c_set_clientdata(client, page0);
	return 0;
}

static int __devinit simg7A_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tx_page1 *page1;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	page1 = kzalloc(sizeof(struct tx_page1), GFP_KERNEL);
	if (!page1) {
		dev_err(&client->dev, " page1 failed to allocate driver data\n");
		return -ENOMEM;
	}

	page1->pdata = client->dev.platform_data;
	page1->pdata->simg7A_tx_client = client;

	if (!page1->pdata) {
		printk(KERN_INFO"\n SIMG7A no platform data \n");
		kfree(page1);
		return -EINVAL;
	}

	i2c_set_clientdata(client, page1);
	return 0;
}

static int __devinit simg92_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct tx_page2 *page2;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	page2 = kzalloc(sizeof(struct tx_page2), GFP_KERNEL);

	if (!page2) {
		dev_err(&client->dev, " page2 failed to allocate driver data\n");
		return -ENOMEM;
	}

	page2->pdata = client->dev.platform_data;
	page2->pdata->simg92_tx_client = client;

	if (!page2->pdata) {
		printk(KERN_INFO"\n SIMG92 no platform data \n");
		kfree(page2);
		return -EINVAL;
	}

	i2c_set_clientdata(client, page2);
	return 0;
}

static int __devinit simgC8_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mhl_tx *mhl;
	int ret = 0;
	dev_info(&client->dev, "success client_addr 0x%X\n", client->addr);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	mhl = kzalloc(sizeof(struct mhl_tx), GFP_KERNEL);

	if (!mhl) {
		dev_err(&client->dev, " cbus failed to allocate driver data\n");
		return -ENOMEM;
	}

	mhl->pdata = client->dev.platform_data;

	if (!mhl->pdata) {
		dev_err(&client->dev, "failed to find platform data\n");
		ret = -EINVAL;
		goto err_exit0;
	}

	mhl->pdata->simgC8_tx_client = client;
	mhl->pdata->status.op_status = NO_MHL_STATUS;
	mhl->cmd_rx_cnt = 0;
	mhl->cmd_tx_cnt = 0;

	if (!mhl->pdata) {
		printk(KERN_INFO"\n SIMGC8 no platform data \n");
		kfree(mhl);
		return -EINVAL;
	}

	init_waitqueue_head(&mhl->cbus_cmd_wq);
	mutex_init(&mhl->cbus_cmd_lock);
	mutex_init(&mhl->pdata->mhl_status_lock);
	i2c_set_clientdata(client, mhl);
	sii8246_mhldev = &client->dev;

	INIT_LIST_HEAD(&mhl->cbus_msg_list);
	
	INIT_WORK(&mhl->cbus_cmd_work, cbus_cmd_thread);
	dev_info(&client->dev, "sii8246 irq : %d\n", client->irq);
	mhl->pdata->irq = client->irq;
	mhl->cbus_cmd_wqs = create_singlethread_workqueue("cbus_cmd_wqs");

	if (!mhl->cbus_cmd_wqs)
		return -ENOMEM;

	g_mhl_tx = mhl;
	ret = request_threaded_irq(mhl->pdata->irq, NULL, mhl_irq_thread,
							   IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "mhl_intr", mhl);

	if (ret < 0) {
		printk(KERN_INFO "mhl_intr regist fail \n");
		kfree(mhl);
		return ret;
	}
	wake_lock_init(&mhl->mhl_wake_lock,
			WAKE_LOCK_SUSPEND, "mhl_wake_lock");
	mhl->mhl_nb.notifier_call = mhl_power_on_cb;
	acc_register_notifier(&mhl->mhl_nb);

	disable_irq(mhl->pdata->irq);
	return 0;
err_exit0:
	kfree(mhl);
	return ret;
}


static int __devexit simg72_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit simg7A_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit simg92_remove(struct i2c_client *client)
{
	return 0;
}

static int __devexit simgC8_remove(struct i2c_client *client)
{
	return 0;
}
#ifndef CONFIG_OF
static struct i2c_device_id SIMGC8_id[] = {
	{"SIMGC8", 0},
	{}
};
static struct i2c_device_id SIMG72_id[] = {
	{"SIMG72", 0},
	{}
};

static struct i2c_device_id SIMG7A_id[] = {
	{"SIMG7A", 0},
	{}
};

static struct i2c_device_id SIMG92_id[] = {
	{"SIMG92", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, SIMGC8_id);
MODULE_DEVICE_TABLE(i2c, SIMG72_id);
MODULE_DEVICE_TABLE(i2c, SIMG7A_id);
MODULE_DEVICE_TABLE(i2c, SIMG92_id);
#endif
static int __init simg_init(void)
{
	int ret = 0;
#ifdef CONFIG_OF
	ret = i2c_add_driver(&sii8246_i2c_driver);

	if (ret < 0) {
		pr_err("[ERROR] sii8246: mhl_v2 i2c driver init failed");
		return ret;
	}

#else
	ret = i2c_add_driver(&simg72_driver);

	if (ret != 0)
		goto err_exit1;

	ret = i2c_add_driver(&simg7A_driver);

	if (ret != 0)
		goto err_exit2;

	ret = i2c_add_driver(&simg92_driver);

	if (ret != 0)
		goto err_exit3;

	ret = i2c_add_driver(&simgC8_driver);

	if (ret != 0)
		goto err_exit4;

#endif
	printk(KERN_INFO"\n simg_init done \n");
	return 0;
#ifndef CONFIG_OF
err_exit4:
	printk(KERN_INFO"simgC8_driver fail\n");
	i2c_del_driver(&simgC8_driver);
err_exit3:
	printk(KERN_INFO"simg92_driver fail\n");
	i2c_del_driver(&simg92_driver);
err_exit2:
	printk(KERN_INFO"simg7A_driver fail\n");
	i2c_del_driver(&simg7A_driver);
err_exit1:
	printk(KERN_INFO"simg72_driver fail\n");
	i2c_del_driver(&simg72_driver);
#endif
#ifdef CONFIG_OF
	printk(KERN_INFO"simg_driver_dt fail\n");
	i2c_del_driver(&sii8246_i2c_driver);
#endif
	return ret;
}

static void __exit simg_exit(void)
{
#ifdef CONFIG_OF
	i2c_del_driver(&sii8246_i2c_driver);
#else
	i2c_del_driver(&simg72_driver);
	i2c_del_driver(&simg7A_driver);
	i2c_del_driver(&simg92_driver);
	i2c_del_driver(&simgC8_driver);
#endif
}


module_init(simg_init);
module_exit(simg_exit);

MODULE_DESCRIPTION("Silicon Image MHL Transmitter driver");
MODULE_AUTHOR("Daniel(Philju) Lee <daniel.lee@siliconimage.com>");
MODULE_LICENSE("GPL");

