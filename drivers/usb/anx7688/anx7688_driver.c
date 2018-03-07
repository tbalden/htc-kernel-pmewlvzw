/******************************************************************************

Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V2.1.12

Filename : anx7688_driver.c

Project  : ANX7688

Created  : 28 Nov. 2016

Devices  : ANX7688

Toolchain: Android

Description:

Revision History:

******************************************************************************/
/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/htc_flags.h>
#include <linux/power/htc_battery.h>
#include "anx7688_driver.h"
#include "anx7688_private_interface.h"
#include "anx7688_public_interface.h"
#include "eeprom.h"

#include "../../video/msm/mdss/mdss_hdmi_mhl.h"

/* Use device tree structure data when defined "CONFIG_OF"  */
/* #define CONFIG_OF */

#define ANX7688_DRV_VERSION "2.1.12"

extern int dwc3_pd_vbus_ctrl(int on);
extern void dwc3_otg_set_id_state(int id);
extern int dwc3_pd_drswap(int new_role);
extern int usb_set_dwc_property(int prop_type,unsigned int value);
extern int usb_lock_speed;
static int create_sysfs_interfaces(struct device *dev);

static int anx7688_audio_accessory(int attach);
static int anx7688_cc_change(u8 cc_status);
static void anx7688_enable_drole(bool on);
static void anx7688_get_max_rdo(void);
static int anx7688_platform_vconn_ctl(bool on);
static int anx7688_vbus_ctl(bool on);
static int anx7688_handle_intr_vector(void);
static int anx7688_enable_cbl_det(void);

/* to access global platform data */
static struct anx7688_platform_data *g_pdata;
static struct anx7688_data *g_data;

#define DONGLE_CABLE_INSERT  1
#define VDD18_ACTIVE_UA 105000

atomic_t anx7688_power_status;

struct i2c_client *anx7688_client;

struct anx7688_platform_data {
	int gpio_p_on;
	int gpio_reset;
	int gpio_cbl_det;
#ifdef SUP_INT_VECTOR
	int gpio_intr_comm;
#endif
#ifdef SUP_VBUS_CTL
	int gpio_vbus_ctrl;
#endif
	spinlock_t lock;
	int gpio_aud_hsdet;
	int gpio_fsa3030_sel0;
	int gpio_fsa3030_sel1;
	int gpio_fsa3030_sel2;
	int gpio_vconn_5v_en;
	int gpio_usb_ptn_c1;
	int gpio_usb_ptn_c2;
	int gpio_ovp_cc_sbu;
	int cbl_det_irq;
	struct regulator *vdd10;
	struct regulator *vdd18;	/* for USB3.0 redriver */

	/* HTC: DisplayPort */
	struct platform_device *hdmi_pdev;
	struct msm_hdmi_sp_ops *hdmi_sp_ops;
	bool disable_hbr25;
};

struct anx7688_data {
	struct anx7688_platform_data *pdata;
	struct delayed_work work;
	struct delayed_work drole_work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
	struct wake_lock anx7688_lock;
	struct dual_role_phy_instance *anx7688_dual_role_instance;
	struct power_supply *batt_psy;
	bool aud_acc;
	bool drole_on;
	u8 curr_cc;
	enum port_mode pmode;
	enum power_role prole;
	enum data_role drole;
	enum vconn_supply vconn;
	enum pr_change pr_change;
	enum cc_orientation cc_ori;
};

unsigned char device_addr = OCM_SLAVE_I2C_ADDR1;
unsigned char debug_on = 0;
unsigned char ocm_bootload_done;
#if AUTO_UPDATE_OCM_FW
unsigned char auto_update = 1; // auto update OCM FW
static unsigned short int OcmFwVersion = 0;
#endif
unsigned int ptn_c1 = C1C2_HIGH;  // 0: L; 1: H; 2: High-Z
unsigned int ptn_c2 = C1C2_HIGH;

int oc_enable = 0;

/* anx7688 power status, sync with interface and cable detection thread */

inline unsigned char ReadReg(unsigned char RegAddr)
{
	int ret = 0;

	anx7688_client->addr = (device_addr >> 1);
	ret = i2c_smbus_read_byte_data(anx7688_client, RegAddr);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
				__func__, device_addr);
	}
	return (uint8_t) ret;

}

inline int ReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
	int ret = 0;

	anx7688_client->addr = (device_addr >> 1);
	ret = i2c_smbus_read_i2c_block_data(anx7688_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
				__func__, device_addr);
		return -EPERM;
	}

	return (int)ret;
}


inline int WriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
	int ret = 0;

	anx7688_client->addr = (device_addr >> 1);
	ret = i2c_smbus_write_i2c_block_data(anx7688_client, RegAddr, len, dat);
	if (ret < 0) {
		pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
				__func__, device_addr);
		return -EPERM;
	}

	return (int)ret;
}

inline void WriteReg(unsigned char RegAddr, unsigned char RegVal)
{
	int ret = 0;
	anx7688_client->addr = (device_addr >> 1);
	ret = i2c_smbus_write_byte_data(anx7688_client, RegAddr, RegVal);
	if (ret < 0) {
		pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
				__func__, device_addr);
	}
}

static int usb3_redriver_ctl(int c1, int c2)
{
	struct anx7688_platform_data *pdata = g_pdata;

	if (!pdata) {
		pr_err("%s: no platform data\n", __func__);
		return -ENODEV;
	}

	pr_info("(c1, c2)= (%d, %d)\n", c1, c2);

	if (gpio_is_valid(pdata->gpio_usb_ptn_c1) &&
					gpio_is_valid(pdata->gpio_usb_ptn_c2)) {
		if (c1 == C1C2_LOW && c2 == C1C2_LOW) {
			gpio_direction_output(pdata->gpio_usb_ptn_c1, 0);
			gpio_direction_output(pdata->gpio_usb_ptn_c2, 0);
			return 0;
		}
		/* in case current state is in deep power-saving mode,
		    we have to let the redriver exit this mode first.
		    Pull-high C1 or C2 makes the redriver return to normal
		    active mode. Here we additionally pull-high both
		    C1 and C2 only if none of them is going to be pulled-high. */
		if (c1 != C1C2_HIGH && c2 != C1C2_HIGH) {
			gpio_direction_output(pdata->gpio_usb_ptn_c1, 1);
			gpio_direction_output(pdata->gpio_usb_ptn_c2, 1);
			pr_info("usb3 redriver enters active mode\n");
			mdelay(1);
		}

		if (c1 == C1C2_HIGH_Z)  /* C1 is set to high-z */
			gpio_direction_input(pdata->gpio_usb_ptn_c1);
		else  /* C1 is set to low or high */
			gpio_direction_output(pdata->gpio_usb_ptn_c1, c1);

		if (c2 == C1C2_HIGH_Z)  /* C2 is set to high-z */
			gpio_direction_input(pdata->gpio_usb_ptn_c1);
		else  /* C2 is set to low or high */
			gpio_direction_output(pdata->gpio_usb_ptn_c2, c2);

		pr_debug("USB3.0 redriver status (c1, c2)= (%d, %d)\n",
			gpio_get_value(pdata->gpio_usb_ptn_c1),
			gpio_get_value(pdata->gpio_usb_ptn_c2));
	}
	else {
		pr_err("usb3 redriver is unavailable\n");
		return -EINVAL;
	}
	return 0;
}

void MI1_power_on(void)
{
#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif

	int rc;

	rc = regulator_enable(pdata->vdd10);
	if (rc) {
		pr_err("%s: vdd10 enable failed\n", __func__);
		return;
	}
	usleep_range(1000, 2000);

	/*power on pin enable */
	gpio_set_value(pdata->gpio_p_on, 1);
	mdelay(10);
	/*power reset pin enable */
	gpio_set_value(pdata->gpio_reset, 1);
	mdelay(10);
	pr_info("%s: MI-1 power on !\n", __func__);
}

void anx7688_hardware_reset(int enable)
{
#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif
	gpio_set_value(pdata->gpio_reset, enable);
}


void anx7688_power_standby(void)
{
#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif

	gpio_set_value(pdata->gpio_reset, 0);
	mdelay(1);
	gpio_set_value(pdata->gpio_p_on, 0);
	mdelay(1);

	if (regulator_is_enabled(pdata->vdd10)) {
		regulator_disable(pdata->vdd10);
		pr_info("disabling vdd10\n");
	}

	pr_info("%s: anx7688 power down\n", __func__);
}

#define EEPROM_LOAD_STA 0x12
#define EEPROM_LOAD_STA_CHK	(1<<0)

void anx7688_hardware_poweron(void)
{
#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif
	int retry_count, i, rc;

	pr_debug("%s: anx7688 power on\n", __func__);

	rc = regulator_enable(pdata->vdd10);
	if (rc) {
		pr_err("vdd10 enable failed\n");
		return ;
	}

	/* Vdd18 is for USB3.0 redriver */
	if (pdata->vdd18) {
		rc = regulator_set_optimum_mode(pdata->vdd18, VDD18_ACTIVE_UA);
		if (rc <= 0)
			pr_err("vdd18 set high power mode failed, rc = %d\n", rc);
		rc = regulator_enable(pdata->vdd18);
		if (rc)
			pr_err("vdd18 enable failed, rc = %d\n", rc);
	}

	for (retry_count = 0; retry_count < 3; retry_count++) {
#ifdef OCM_DEBUG
		pr_debug("%s: anx7688 check ocm loading...\n", __func__);
#endif

		if (usb_lock_speed != 1)
			// Set USB3.0 redriver IC (c1, c2)
			usb3_redriver_ctl(ptn_c1, ptn_c2);

		/*power on pin enable */
		gpio_set_value(pdata->gpio_p_on, 1);
		mdelay(2);

		/*power reset pin enable */
		gpio_set_value(pdata->gpio_reset, 1);
		mdelay(10);

		/* load delay T3 : eeprom 3.2s,  OTP 20ms*/
		for (i = 0; i < OCM_LOADING_TIME; i++) {
			/*Interface work? */
			if ((ReadReg(EEPROM_LOAD_STA)&EEPROM_LOAD_STA_CHK) == EEPROM_LOAD_STA_CHK) {
#ifdef OCM_DEBUG
				pr_debug("%s: interface initialization\n", __func__);
#endif
				chip_register_init();
				interface_init();
				send_initialized_setting();

#ifdef OCM_DEBUG
				pr_info("%s: firmware version is %02x.%02x, Driver version: %s\n",\
							__func__, ReadReg(0x15), ReadReg(0x16), ANX7688_DRV_VERSION);
#endif
				ocm_bootload_done = 1;
				return;
			}
			mdelay(1);
			//printk(".");
		}
		anx7688_power_standby();
		mdelay(10);
	}

}

/* This function directly controls the Vconn PMOS from 7688 */
static int __maybe_unused anx7688_vconn_ctl(unsigned int target_cc, bool on)
{
	struct anx7688_data *data = g_data;
	u8 vconn_mask;

	if (!data)
		return -ENODEV;

	if ((target_cc != CC1) && (target_cc != CC2)) {
		pr_err("target_cc = %u\n", target_cc);
		pr_err("invalid cc selection\n");
		return -EINVAL;
	}

	if (!gpio_is_valid(data->pdata->gpio_vconn_5v_en)) {
		pr_err("vconn is unavailable\n");
		anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_NO);
		return -EINVAL;
	}

	if (on) {
		gpio_set_value(data->pdata->gpio_vconn_5v_en, 1);
		if (target_cc == CC1) {
			vconn_mask = ReadReg(R_N_GPIO_CTRL_0);
			vconn_mask |= VCONN1_EN;
			WriteReg(R_N_GPIO_CTRL_0, vconn_mask);
			pr_info("Vconn enabled on CC1\n");
		}
		else if (target_cc == CC2) {
			vconn_mask = ReadReg(R_N_GPIO_CTRL_0);
			vconn_mask |= VCONN2_EN;
			WriteReg(R_N_GPIO_CTRL_0, vconn_mask);
			pr_info("Vconn enabled on CC2\n");
		}
		anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_YES);
	}
	else {
		if (target_cc == CC1) {
			vconn_mask = ReadReg(R_N_GPIO_CTRL_0);
			vconn_mask &= ~VCONN1_EN;
			WriteReg(R_N_GPIO_CTRL_0, vconn_mask);
			pr_info("Vconn disabled on CC1\n");
		}
		else if (target_cc == CC2) {
			vconn_mask = ReadReg(R_N_GPIO_CTRL_0);
			vconn_mask &= ~VCONN2_EN;
			WriteReg(R_N_GPIO_CTRL_0, vconn_mask);
			pr_info("Vconn disabled on CC2\n");
		}
		gpio_set_value(data->pdata->gpio_vconn_5v_en, 0);
		anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_NO);
	}
	return 0;
}

static int anx7688_cc_change(u8 cc_status)
{
	struct anx7688_data *data = g_data;
	union power_supply_propval batt_prop;
	bool notify_batt = 0;
	int ret = 0;

	if(!data)
		return -ENODEV;

	pr_info("cc change, 0x%02X -> 0x%02x\n", data->curr_cc, cc_status);

	if (data->curr_cc == cc_status)
		return ret;

	data->curr_cc = cc_status;
	switch (cc_status) {
		case 0x00:		/* cable out */
			batt_prop.intval = utccNone;
			notify_batt = 1;
			data->cc_ori = CC_NONE;
			break;
		case 0x01:		/* Rd/Open */
			data->cc_ori = CC1;
			if (VCONN_SUPPLY_YES == data->vconn)
				anx7688_platform_vconn_ctl(0);
			break;
		case 0x10:
			data->cc_ori = CC2;
			if (VCONN_SUPPLY_YES == data->vconn)
				anx7688_platform_vconn_ctl(0);
			break;
		case 0x02:		/* Ra/Open */
			data->cc_ori = CC1;
			break;
		case 0x20:
			data->cc_ori = CC2;
			break;
		case 0x12:		/* Rd/Ra */
			data->cc_ori = CC1;
			break;
		case 0x21:
			data->cc_ori = CC2;
			break;
		case 0x22:		/* Ra/Ra */
			anx7688_audio_accessory(1);
			data->cc_ori = CC_NONE;
			break;
		case 0x04:		/* RpDefault (SNK.Default) */
			batt_prop.intval = utccDefault;
			notify_batt = 1;
			data->cc_ori = CC1;
			//anx7688_vconn_ctl(CC2, 1);
			break;
		case 0x40:
			batt_prop.intval = utccDefault;
			notify_batt = 1;
			data->cc_ori = CC2;
			//anx7688_vconn_ctl(CC1, 1);
			break;
		case 0x08:		/* Rp1A5 (SNK.Power1.5) */
			batt_prop.intval = utcc1p5A;
			notify_batt = 1;
			data->cc_ori = CC1;
			break;
		case 0x80:
			batt_prop.intval = utcc1p5A;
			notify_batt = 1;
			data->cc_ori = CC2;
			break;
		case 0x0c:		/* Rp3A0 (SNK.Power3.0) */
			batt_prop.intval = utcc3p0A;
			notify_batt = 1;
			data->cc_ori = CC1;
			break;
		case 0xc0:
			batt_prop.intval = utcc3p0A;
			notify_batt = 1;
			data->cc_ori = CC2;
			break;
		default:
			data->cc_ori = CC_NONE;
			break;
	}

#ifdef CONFIG_MACH_DUMMY
	if (notify_batt && data->batt_psy && data->batt_psy->set_property) {
		ret = data->batt_psy->set_property(
			data->batt_psy, POWER_SUPPLY_PROP_TYPEC_SINK_CURRENT,
			&batt_prop);
	}
#endif
	return ret;
}

/* This function controls the Vconn source */
static int anx7688_platform_vconn_ctl(bool on)
{
#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif

	if (!pdata)
		return -ENODEV;

	if (!gpio_is_valid(pdata->gpio_vconn_5v_en)) {
		pr_err("vconn is unavailable\n");
		anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_NO);
		return -EINVAL;
	}

	if (on) {
		gpio_set_value(pdata->gpio_vconn_5v_en, 1);
		anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_YES);
	}
	else {
		gpio_set_value(pdata->gpio_vconn_5v_en, 0);
		anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_NO);
	}

	pr_info("vconn %s\n", on?"on":"off");

	return 0;
}

void anx7688_vbus_control(bool value)
{
#ifdef SUP_VBUS_CTL

#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif

	if(value)
		gpio_set_value(pdata->gpio_vbus_ctrl, 1);
	else
		gpio_set_value(pdata->gpio_vbus_ctrl, 0);

#endif
}

static int anx7688_vbus_ctl(bool on)
{
	struct anx7688_data *data = g_data;

	if (!data)
		return -ENODEV;

#ifdef OCM_DEBUG
	pr_info("vbus control %d\n", (int)on);
#endif

	if (on) {
		if (data->prole != PR_SOURCE) {
			data->prole = PR_SOURCE;
			data->pmode = MODE_DFP;
			dwc3_pd_vbus_ctrl(1);
		}
	}
	else {
		if (data->prole == PR_SOURCE) {
			dwc3_pd_vbus_ctrl(0);
		}
		data->prole = PR_SINK;
		data->pmode = MODE_UFP;
	}
	return 0;
}

static int anx7688_get_max_pclk(struct anx7688_platform_data *pdata, u8 bw, u8 lane)
{
	int pclk = 0;

	if (pdata->disable_hbr25 && bw == 0x19)
		bw = 0x14;
	if (lane > 2)
		lane = 2;

	switch (bw) {
	case 0x6: /* 1.62G */
		pclk = 54000;
		break;
	case 0xa: /* 2.7G */
		pclk = 90000;
		break;
	case 0x14: /* 5.4G */
		pclk = 180000;
		break;
	case 0x19: /* 6.75G */
		pclk = 297000;
		break;
	default:
		pr_err("%s: unknown bw (0x%x)\n", __func__, bw);
		pclk = 180000; // Default as 5.4G
		break;
	}

	if (pclk * lane > 297000)
		return 297000;
	return pclk * lane;
}

void anx7688_dp_event(int event)
{
#ifdef CONFIG_OF
	struct anx7688_platform_data *pdata = g_pdata;
#else
	struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif

	pr_info("%s: event=%d\n", __func__, event);
	if (event) {
		int wait_dongle = 20;
		int pclk = 0;
		u8 bandwidth = 0, lane_count = 0, proc_state = 0, status = 0, config = 0;

		while (wait_dongle--) {
			device_addr = OCM_SLAVE_I2C_ADDR2;
			status = ReadReg(0x82);
			bandwidth = ReadReg(0x85);
			config = ReadReg(0x84);
			lane_count = ReadReg(0x86);
			proc_state = ReadReg(0x87);
			device_addr = OCM_SLAVE_I2C_ADDR1;
			pr_info("%s: status=%x, bandwidth=%x, config=%x, lane_count=%x, proc_state=%x\n",
				 __func__, status, bandwidth, config, lane_count, proc_state);

			if (bandwidth && lane_count)
				break;
			pr_info("%s: Wait link training...\n", __func__);
			msleep(20);
		}
		pclk = anx7688_get_max_pclk(pdata, bandwidth, lane_count);

		if (pdata->hdmi_sp_ops->set_upstream_hpd) {
			pdata->hdmi_sp_ops->set_upstream_hpd(pdata->hdmi_pdev, 1);
		}
		if (pdata->hdmi_sp_ops->set_sp_max_pclk)
			pdata->hdmi_sp_ops->set_sp_max_pclk(pdata->hdmi_pdev, pclk);

		wait_dongle = 20;
		while (wait_dongle--) {
			device_addr = OCM_SLAVE_I2C_ADDR2;
			status = ReadReg(0x82);
			config = ReadReg(0x84);
			proc_state = ReadReg(0x87);
			device_addr = OCM_SLAVE_I2C_ADDR1;
			pr_info("%s: status=%x, config=%x, proc_state=%x\n",
				 __func__, status, config, proc_state);

			if (status)
				break;
			pr_info("%s: Wait link training...\n", __func__);
			usleep_range(10000, 11000);
		}

		if (pdata->hdmi_sp_ops->set_upstream_hdcp)
			pdata->hdmi_sp_ops->set_upstream_hdcp(pdata->hdmi_pdev, status & BIT(4));
	} else {
		if (pdata->hdmi_sp_ops->set_upstream_hpd) {
			pdata->hdmi_sp_ops->set_upstream_hpd(pdata->hdmi_pdev, 0);
		}
	}
}

void anx7688_main_process(void)
{
	/* do main loop, do what you want to do */
#if AUTO_UPDATE_OCM_FW
	if(auto_update) {
		auto_update = 0;
		OcmFwVersion = burnhexauto();
	}
#endif
}

#if AUTO_UPDATE_OCM_FW
#define MAIN_PROC_RELOAD 0
static void anx7688_work_func(struct work_struct *work)
{
	struct anx7688_data *td = container_of(work, struct anx7688_data,
                                           work.work);
#if MAIN_PROC_RELOAD
	int workqueu_timer = 0;
	workqueu_timer = 1000;
#endif
	mutex_lock(&td->lock);
	anx7688_main_process();
	mutex_unlock(&td->lock);
#if MAIN_PROC_RELOAD
	queue_delayed_work(td->workqueue, &td->work,
					msecs_to_jiffies(workqueu_timer));
#endif
	anx7688_enable_cbl_det();
}
#endif

static int anx7688_audio_accessory(int attach)
{
	struct anx7688_data *platform = g_data;

	if (!platform || !platform->pdata) {
		pr_err("%s: no device data\n", __func__);
		return -ENODEV;
	}

	/* If this function is called, it means that a Ra/Ra cable is attached.
          Therefore put usb3 redriver into deep power saving mode */
	// Set USB3.0 redriver IC (0,0)
	usb3_redriver_ctl(C1C2_LOW, C1C2_LOW);

	if (!gpio_is_valid(platform->pdata->gpio_fsa3030_sel0) &&
				!gpio_is_valid(platform->pdata->gpio_fsa3030_sel1) &&
				!gpio_is_valid(platform->pdata->gpio_fsa3030_sel2) &&
				!gpio_is_valid(platform->pdata->gpio_aud_hsdet)) {
		pr_err("audio accessory mode is unavailable\n");
		platform->aud_acc = 0;
		return -EINVAL;
	}

	if (attach) {
		gpio_set_value(platform->pdata->gpio_fsa3030_sel0, 1);
		gpio_set_value(platform->pdata->gpio_fsa3030_sel1, 0);
		gpio_set_value(platform->pdata->gpio_fsa3030_sel2, 0);
		mdelay(5);
		gpio_set_value(platform->pdata->gpio_aud_hsdet, 0);
		platform->aud_acc = 1;
		/* power down 7688 to improve power consumption */
		gpio_set_value(platform->pdata->gpio_p_on, 0);
		if (regulator_is_enabled(platform->pdata->vdd10)) {
			regulator_disable(platform->pdata->vdd10);
			pr_info("disabling vdd10\n");
		}
		return 0;
	}
	else {
		gpio_set_value(platform->pdata->gpio_fsa3030_sel0, 0);
		gpio_set_value(platform->pdata->gpio_fsa3030_sel1, 1);
		gpio_set_value(platform->pdata->gpio_fsa3030_sel2, 0);
		mdelay(5);
		gpio_set_value(platform->pdata->gpio_aud_hsdet, 1);
		platform->aud_acc = 0;
		return 0;
	}
}

#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data)
{
	struct anx7688_data *platform = data;
	unsigned int count = 9;
	unsigned int cable_det_count = 0;
	u8 val = 0;

	do {
		val = gpio_get_value(platform->pdata->gpio_cbl_det);
		if (DONGLE_CABLE_INSERT == val)
			cable_det_count++;
		mdelay(1);;
	} while (count--);

	if (cable_det_count > 7)
		return 1;
	else if (cable_det_count < 3)
		return 0;
	else
		return atomic_read(&anx7688_power_status);
}
#endif

static irqreturn_t anx7688_cbl_det_isr(int irq, void *data)
{
	struct anx7688_data *platform = data;
	union power_supply_propval batt_prop;
	int cable_connected = 0;
	int rc = 0;

	if (debug_on) return IRQ_NONE;

#ifdef CABLE_DET_PIN_HAS_GLITCH
	cable_connected = confirmed_cable_det((void *)platform);
#else
	cable_connected = gpio_get_value(platform->pdata->gpio_cbl_det);
#endif

#ifdef OCM_DEBUG
	pr_info_ratelimited("%s: cable plug pin status %d\n",
						__func__, cable_connected);
#endif

	if (cable_connected == DONGLE_CABLE_INSERT) {
		if (atomic_read(&anx7688_power_status) == 1) {
#ifdef CABLE_DET_PIN_HAS_GLITCH
			mdelay(2);
			//anx7688_power_standby();
			return IRQ_HANDLED;
#else
            return IRQ_HANDLED;
#endif
		}
		atomic_set(&anx7688_power_status, 1);
		ocm_bootload_done = 0;
		anx7688_hardware_poweron();

	} else {
		if (atomic_read(&anx7688_power_status) == 0) {
			mdelay(2);
			return IRQ_HANDLED;
		}
		atomic_set(&anx7688_power_status, 0);
		cancel_delayed_work(&platform->drole_work);
		dwc3_pd_vbus_ctrl(-1);
		dwc3_otg_set_id_state(1);
		platform->drole = UNKNOWN_DATA_ROLE;
		platform->prole = UNKNOWN_POWER_ROLE;
		platform->pmode = MODE_UNKNOWN;
		platform->pr_change = PR_NOCHANGE;
		if (platform->vconn == VCONN_SUPPLY_YES)
			anx7688_platform_vconn_ctl(0);
		platform->curr_cc = 0x00;
		batt_prop.intval = utccNone;
#ifdef CONFIG_MACH_DUMMY
		if (platform->batt_psy && platform->batt_psy->set_property) {
			platform->batt_psy->set_property(
			platform->batt_psy, POWER_SUPPLY_PROP_TYPEC_SINK_CURRENT,
				&batt_prop);
		}
#endif

#ifdef SUP_VBUS_CTL
		gpio_set_value(platform->pdata->gpio_vbus_ctrl, 0);
#endif
		if (platform->aud_acc)
			anx7688_audio_accessory(0);
		// TYPE_DP_ALT_EXIT message was not generated, use calbe out as
		// displayport disconnect instead.
		anx7688_dp_event(0);

		dual_role_instance_changed(platform->anx7688_dual_role_instance);

		// Set USB3.0 redriver IC (0,0)
		usb3_redriver_ctl(C1C2_LOW, C1C2_LOW);

		anx7688_power_standby();

		/* Vdd18 is for USB3.0 redriver */
		if (platform->pdata->vdd18) {
			rc = regulator_disable(platform->pdata->vdd18);
			if (rc)
				pr_err("vdd18 disable failed, rc = %d\n", rc);
			rc = regulator_set_optimum_mode(platform->pdata->vdd18, 0);
			if (rc <= 0)
				pr_err("vdd18 set low power mode failed, rc = %d\n", rc);
		}
	}

	return IRQ_HANDLED;
}

static int anx7688_enable_cbl_det(void)
{
	struct anx7688_platform_data *pdata = g_pdata;
	struct anx7688_data *data = g_data;
	int ret;

	if (!data || !pdata)
		return -ENODEV;

	if (confirmed_cable_det((void *)data)) {
		atomic_set(&anx7688_power_status, 1);
		anx7688_hardware_poweron();
	}

	pdata->cbl_det_irq = gpio_to_irq(pdata->gpio_cbl_det);
	if (pdata->cbl_det_irq < 0) {
		pr_err("%s : failed to get gpio irq\n", __func__);
		return -ENXIO;
	}

	ret = request_threaded_irq(pdata->cbl_det_irq, NULL, anx7688_cbl_det_isr,
							IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
							| IRQF_ONESHOT, "anx7688-cbl-det", data);
	if (ret < 0) {
		pr_err("%s : failed to request irq\n", __func__);
		return -EINVAL;
	}

	ret = irq_set_irq_wake(pdata->cbl_det_irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq for cable detect", __func__);
		pr_err("interrupt wake set fail\n");
		free_irq(pdata->cbl_det_irq, data);
		return -EINVAL;
	}

	ret = enable_irq_wake(pdata->cbl_det_irq);
	if (ret < 0) {
		pr_err("%s : Enable irq for cable detect", __func__);
		pr_err("interrupt wake enable fail\n");
		free_irq(pdata->cbl_det_irq, data);
		return -EINVAL;
	}

	return 0;
}

#define STS_DATA_ROLE_CHANGE \
	(((sys_status&DATA_ROLE)!=(sys_sta_bak&DATA_ROLE)) ? DATA_ROLE_CHANGE:0)
#define STS_VCONN_CHANGE \
	(((sys_status&VCONN_STATUS)!=(sys_sta_bak&VCONN_STATUS)) ? VCONN_CHANGE:0)
#define STS_VBUS_CHANGE \
	(((sys_status&VBUS_STATUS)!=(sys_sta_bak&VBUS_STATUS)) ? VBUS_CHANGE:0)
static int anx7688_handle_intr_vector(void)
{
	static unsigned char sys_sta_bak = 0;
	unsigned char sys_status;
	u8 intr_vector = ReadReg(INTERFACE_CHANGE_INT);
	u8 status;
	struct anx7688_data *data = g_data;

	if (!data)
		return -ENODEV;

#ifdef OCM_DEBUG
	pr_debug(" intr vector = 0x%02x\n",  intr_vector);
#endif
	WriteReg(INTERFACE_CHANGE_INT, intr_vector & (~intr_vector));
	if ((~INTR_MASK_SETTING) & intr_vector & RECEIVED_MSG)
		polling_interface_msg(INTERACE_TIMEOUT_MS);
	if ((~INTR_MASK_SETTING) & intr_vector & CC_STATUS_CHANGE) {
		status = ReadReg(NEW_CC_STATUS);
		anx7688_cc_change(status);
		/* If audio accessory mode is on, we will power down 7688. Hence return here. */
		if (data->aud_acc)
			return 0;
    }
	if ((~INTR_MASK_SETTING) & (intr_vector & 0x40)) {
		anx7688_get_max_rdo();
	}

	if (atomic_read(&anx7688_power_status) != 1)
		return 0;
	sys_status = ReadReg(SYSTEM_STSTUS);

	if ((~INTR_MASK_SETTING) & ((intr_vector & VBUS_CHANGE) | STS_VBUS_CHANGE)) {
		status = ReadReg(SYSTEM_STSTUS);
		anx7688_vbus_ctl(status & VBUS_STATUS);
	}
	if ((~INTR_MASK_SETTING) & ((intr_vector & VCONN_CHANGE) | STS_VCONN_CHANGE)) {
		status = ReadReg(SYSTEM_STSTUS);
		anx7688_platform_vconn_ctl(status & VCONN_STATUS);
	}
	if ((~INTR_MASK_SETTING) & ((intr_vector & DATA_ROLE_CHANGE) | STS_DATA_ROLE_CHANGE)) {
		status = ReadReg(SYSTEM_STSTUS);
		anx7688_enable_drole(status & DATA_ROLE);
	}

	dual_role_instance_changed(data->anx7688_dual_role_instance);

	sys_sta_bak = sys_status;

	clear_soft_interrupt();
	return 0;
}

static irqreturn_t anx7688_intr_comm_isr(int irq, void *data)
{
	unsigned char c;

	if (atomic_read(&anx7688_power_status) != 1)
		return IRQ_NONE;

	if (ocm_bootload_done != 1)
		return IRQ_NONE;

	device_addr= OCM_SLAVE_I2C_ADDR2;
	//clear interrupt
	c=ReadReg(0x10);
	if (c!=0) WriteReg(0x10, c);
	device_addr= OCM_SLAVE_I2C_ADDR1;


	if (is_soft_reset_intr()) {
#ifdef OCM_DEBUG
		pr_info("%s %s : ======I=====\n", LOG_TAG, __func__);
#endif

		anx7688_handle_intr_vector();
	}
	return IRQ_HANDLED;
}

static void anx7688_enable_drole(bool on)
{
	struct anx7688_data *td = g_data;
	if (!td)
		return;

#ifdef OCM_DEBUG
		pr_info("data role control %d\n", (int)on);
#endif

	if (on) {
		if (td->drole != DR_HOST) {
			td->drole_on = on;
			td->drole = DR_HOST;
			queue_delayed_work(td->workqueue,
				&td->drole_work, msecs_to_jiffies(1000));
		}
	}
	else {
		if (td->drole == DR_HOST) {
			td->drole_on = on;
			td->drole = DR_DEVICE;
			queue_delayed_work(td->workqueue, &td->drole_work, 0);
		}
		else {
			td->drole = DR_DEVICE;
			pr_info("no need to change drole\n");
		}
	}
}

static void anx7688_drole_work_func(struct work_struct *work)
{
	struct anx7688_data *td = g_data;
	/* data role changed */

	if (atomic_read(&anx7688_power_status) != 1) {
		pr_err("%s: cable out, should not do anything with event irq\n", __func__);
		return;
	}

	/* TODO: remember to add check current state */
	/* to DFP  */
	//mutex_lock(&td->drole_lock);
	if (td->drole_on) {
		dwc3_pd_drswap(DR_HOST);
		dwc3_otg_set_id_state(0);
		td->drole = DR_HOST;
		/* power down 7688 to improve power consumption */
		if (!downstream_pd_cap && (1 == usb_lock_speed)) {
			gpio_set_value(td->pdata->gpio_p_on, 0);
			if (regulator_is_enabled(td->pdata->vdd10)) {
				regulator_disable(td->pdata->vdd10);
				pr_info("disabling vdd10\n");
			}
		}
	}
	/* to UFP  */
	else {
		dwc3_pd_drswap(DR_DEVICE);
		dwc3_otg_set_id_state(1);
		td->drole = DR_DEVICE;
	}
	dual_role_instance_changed(td->anx7688_dual_role_instance);
	//mutex_unlock(&td->drole_lock);

}

static void anx7688_get_max_rdo()
{
	int RDO_Max_Voltage, RDO_Max_Power;
	struct htc_pd_data pdo_data;
	int device_max_ma = 0;
	int sel_voltage_pdo_index = 0;

	if (atomic_read(&anx7688_power_status) != 1) {
		pr_err("%s: power not ready\n", __func__);
		return;
	}

	RDO_Max_Voltage = (int)ReadReg(0x1E);
	RDO_Max_Power = (int)ReadReg(0x1F);
	pr_debug("max_voltage = 0x%02x, max_power = 0x%02x\n",
		RDO_Max_Voltage, RDO_Max_Power);

	if (0 == RDO_Max_Voltage)
		return;

	pdo_data.pd_list[0][0] = RDO_Max_Voltage * 100;	// voltage (mV)
	pdo_data.pd_list[0][1] =
		((RDO_Max_Power * 500) / RDO_Max_Voltage) * 10;  // current (mA)
	pr_debug("voltage = %d, current = %d\n",
		pdo_data.pd_list[0][0], pdo_data.pd_list[0][1]);

	sel_voltage_pdo_index = htc_battery_pd_charger_support(
							1, pdo_data, &device_max_ma);
	pr_debug("device_max_ma=%d, index %d\n", device_max_ma, sel_voltage_pdo_index);

	if (sel_voltage_pdo_index < 0) {
		pr_err("RDO Mismatch !!!\n");
		return;
	}

	usb_set_dwc_property(PROPERTY_CURRENT_MAX, (unsigned int)device_max_ma * 1000);

	return;
}

int anx7688_get_prop(u8 prop)
{
	struct anx7688_data *data = g_data;
	if (!data)
		return -ENODEV;

	switch (prop) {
		case ANX_DROLE: //data role
			return data->drole;
		case ANX_PROLE: // power role
			return data->prole;
		case ANX_PMODE: // port mode
			return data->pmode;
		case ANX_PROLE_CHANGE: // power role change
			return data->pr_change;
		case ANX_VCONN: // vconn
			return data->vconn;
		case ANX_START_HOST_FLAG: // start host flag (7418 only)
		case ANX_EMARKER: // emarker cable connect (7418 only)
		case ANX_NON_STANDARD: // For no resistor on cc cable (7418 only)
			return -EINVAL;
		case ANX_PD_CAP: // pd capability of connected device
			return downstream_pd_cap;
		case ANX_FW_VERSION: // fw version (7418 only)
			return -EINVAL;
		default:
			return -EINVAL;
	}
}
EXPORT_SYMBOL(anx7688_get_prop);

/* for compatibality with PME */
int ohio_get_data_value(int data_member)
{
	return 0;
}
EXPORT_SYMBOL(ohio_get_data_value);

int anx7688_set_prop(u8 prop, int val)
{
	struct anx7688_data *data = g_data;
	if (!data)
		return -ENODEV;

	switch (prop) {
		case ANX_DROLE: //data role
			data->drole = val;
			break;
		case ANX_PROLE: // power role
			data->prole = val;
			break;
		case ANX_PMODE: // port mode
			data->pmode = val;
			break;
		case ANX_PROLE_CHANGE: // power role change
			data->pr_change = val;
			break;
		case ANX_VCONN: // vconn
			data->vconn = val;
			break;
		case ANX_START_HOST_FLAG: // start host flag (7418 only)
		case ANX_EMARKER: // emarker cable connect (7418 only)
		default:
			return -EINVAL;
	}
	return 0;
}
EXPORT_SYMBOL(anx7688_set_prop);

void ufp_switch_usb_speed(int on)
{
	if (atomic_read(&anx7688_power_status) != 1){
		pr_err("%s : system is not ready.\n", __func__);
		return;
	}

	/* on: switch to USB3.0 */
	if (on) {
		// Set USB3.0 redriver IC (c1, c2)
		usb3_redriver_ctl(ptn_c1, ptn_c2);
	}
	else {
		usb3_redriver_ctl(C1C2_LOW, C1C2_LOW);
	}
}

/* for compatibality with PME */
int ohio_set_data_value(int data_member, int val)
{
	return -1;
}

/* for compatibality with PME */
int ohio_release_wakelock(void)
{
	return 0;
}
EXPORT_SYMBOL(ohio_release_wakelock);

int workable_charging_cable(void)
{
	return 0;
}
EXPORT_SYMBOL(workable_charging_cable);

static int anx7688_dual_role_get_property(
				struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				unsigned int *val)
{
	struct anx7688_data *data = g_data;

	if(!data)
		return -ENODEV;

	switch (prop) {
		case DUAL_ROLE_PROP_SUPPORTED_MODES:
			*val = 0; // "ufp dfp"
			break;
		case DUAL_ROLE_PROP_MODE:
			*val = (unsigned int)data->pmode;
			break;
		case DUAL_ROLE_PROP_PR:
			*val = (unsigned int)data->prole;
			break;
		case DUAL_ROLE_PROP_DR:
			*val = (unsigned int)data->drole;
			break;
		case DUAL_ROLE_PROP_VCONN_SUPPLY:
			*val = (unsigned int)data->vconn;
			break;
		case DUAL_ROLE_PROP_PARTNER_SUPPORTS_USB_PD:
			if (downstream_pd_cap)
				*val = DUAL_ROLE_PROP_PARTNER_SUPPORTS_USB_PD_YES;
			else
				*val = DUAL_ROLE_PROP_PARTNER_SUPPORTS_USB_PD_NO;
			break;
		default:
			break;
	}
	return 0;
}

static int anx7688_dual_role_set_property(
				struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop,
				const unsigned int *val)
{
	struct anx7688_data *data = g_data;

	if(!data)
		return -ENODEV;

	switch (prop) {
		case DUAL_ROLE_PROP_SUPPORTED_MODES:
			break;
		case DUAL_ROLE_PROP_MODE:
			pr_info("%s: set dual role mode %d -> %u\n", __func__, data->pmode, *val);
			switch (*val) {
				case MODE_UFP:
					if (data->pmode == MODE_DFP) {
						if (downstream_pd_cap)
							data->pr_change = SRC_TO_SNK;
						try_sink();
					}
					else
						pr_err("%s: change mode fail, %d -> %u\n", __func__, data->pmode, *val);
					break;
				case MODE_DFP:
					if (data->pmode == MODE_UFP) {
						if (downstream_pd_cap)
							data->pr_change = SNK_TO_SRC;
						try_source();
					}
					else
						pr_err("%s: change mode fail, %d -> %u\n", __func__, data->pmode, *val);
					break;
				case MODE_UNKNOWN:
				default:
					break;
			}
			break;
		case DUAL_ROLE_PROP_PR:
			pr_info("%s: set power role %d -> %u\n", __func__, data->prole, *val);
			switch (*val) {
				case PR_SOURCE:
					if (data->prole == PR_SINK) {
						if (downstream_pd_cap) {
							send_pd_msg(TYPE_PSWAP_REQ, NULL, 0);
							data->pr_change = SNK_TO_SRC;
							pr_info("%s: send pswap and set pr_change\n", __func__);
						}
						else
							try_source();
					}
					break;
				case PR_SINK:
					if (data->prole == PR_SOURCE) {
						if (downstream_pd_cap) {
							send_pd_msg(TYPE_PSWAP_REQ, NULL, 0);
							data->pr_change = SRC_TO_SNK;
							pr_info("%s: send pswap and set pr_change\n", __func__);
						}
						else
							try_sink();
					}
					break;
				case UNKNOWN_POWER_ROLE:
				default:
					break;
			}
			break;
		case DUAL_ROLE_PROP_DR:
			pr_info("%s: set data role %d -> %u\n", __func__, data->drole, *val);
			switch (*val) {
				case DR_HOST:
					if (data->drole == DR_DEVICE)
						if (downstream_pd_cap) {
							send_pd_msg(TYPE_DSWAP_REQ, NULL, 0);
							pr_info("%s: send dswap\n", __func__);
						}
					break;
				case DR_DEVICE:
					if (data->drole == DR_HOST)
						if (downstream_pd_cap) {
							send_pd_msg(TYPE_DSWAP_REQ, NULL, 0);
							pr_info("%s: send dswap\n", __func__);
						}
					break;
				case UNKNOWN_DATA_ROLE:
				default:
					break;
			}
			break;
		case DUAL_ROLE_PROP_VCONN_SUPPLY:
			break;
		default:
			break;
	}
	return 0;
}

static int anx7688_dual_role_property_is_writeable(
				struct dual_role_phy_instance *dual_role,
				enum dual_role_property prop)
{
	int val = 0;
	switch (prop) {
		case DUAL_ROLE_PROP_SUPPORTED_MODES:
			val = 0;
			break;
		case DUAL_ROLE_PROP_MODE:
			val = 1;
			break;
		case DUAL_ROLE_PROP_PR:
		case DUAL_ROLE_PROP_DR:
			val = 1;
			break;
		case DUAL_ROLE_PROP_VCONN_SUPPLY:
			val = 1;
			break;
		default:
			break;
	}
	return val;
}

struct dual_role_phy_instance *anx7688_get_dual_role_instance(void)
{
	struct anx7688_data *data = g_data;

	if(!data)
		return NULL;

	return data->anx7688_dual_role_instance;
}
EXPORT_SYMBOL(anx7688_get_dual_role_instance);

struct dual_role_phy_instance *ohio_get_dual_role_instance(void)
{
	struct anx7688_data *data = g_data;

	if(!data)
		return NULL;

	return data->anx7688_dual_role_instance;
}
EXPORT_SYMBOL(ohio_get_dual_role_instance);

static ssize_t anx7688_vconn_en_store(
				struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	struct anx7688_platform_data *pdata = g_pdata;
	unsigned int cmd;
	uint8_t vconn_mask;
	char boot_mode[64] = "";
	bool skip = 0;

	if(!pdata)
		return size;

	strlcpy(boot_mode, htc_get_bootmode(), sizeof(boot_mode));
	if (!strncmp(boot_mode, "ftm", 3))
		skip = 0;
	else if (!strncmp(boot_mode, "mfgkernel", 9))
		skip = 0;
	else if (!strncmp(boot_mode, "mfgkernel:diag58", 16))
		skip = 0;
	else
		skip = 1;

	if (skip) {
		return size;
	}

	if (!gpio_is_valid(pdata->gpio_vconn_5v_en))
		return size;

	sscanf(buff, "%u", &cmd);

	if (atomic_read(&anx7688_power_status) == 1) {
		switch (cmd) {
			case 0:  // disable vconn
				vconn_mask = ReadReg(INTP_CTRL);
				vconn_mask &= ~VCONN1_EN_OUT_OEN & ~VCONN2_EN_OUT_OEN;
				WriteReg(INTP_CTRL, vconn_mask);
				gpio_set_value(pdata->gpio_vconn_5v_en, 0);
				anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_NO);
				break;
			case 1:  // enable vconn1 only
				gpio_set_value(pdata->gpio_vconn_5v_en, 1);
				anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_YES);
				vconn_mask = ReadReg(INTP_CTRL);
				WriteReg(INTP_CTRL, (vconn_mask | VCONN1_EN_OUT_OEN));
				break;
			case 2:  // enable vconn2 only
				gpio_set_value(pdata->gpio_vconn_5v_en, 1);
				anx7688_set_prop(ANX_VCONN, VCONN_SUPPLY_YES);
				vconn_mask = ReadReg(INTP_CTRL);
				WriteReg(INTP_CTRL, (vconn_mask | VCONN2_EN_OUT_OEN));
				break;
			default:
				break;
		}
		pr_info("%s: vconn_en %2x\n", __func__, cmd);
	}
	return size;
}

static ssize_t anx7688_redriver_cfg_store(
				struct device *pdev, struct device_attribute *attr,
				const char *buff, size_t size)
{
	unsigned int c1, c2;
	sscanf(buff, "%u %u", &c1, &c2);
	if (c1 <= 2 && c2 <= 2) {
		ptn_c1 = c1;
		ptn_c2 = c2;
		pr_info("%s: set c1 = %u, c2 = %u\n", __func__, c1, c2);
	}
	return size;
}

static void anx7688_free_gpio(struct anx7688_data *platform)
{
	gpio_free(platform->pdata->gpio_cbl_det);
	gpio_free(platform->pdata->gpio_reset);
	gpio_free(platform->pdata->gpio_p_on);

	if (gpio_is_valid(platform->pdata->gpio_ovp_cc_sbu))
		gpio_free(platform->pdata->gpio_ovp_cc_sbu);
	if (gpio_is_valid(platform->pdata->gpio_usb_ptn_c2))
		gpio_free(platform->pdata->gpio_usb_ptn_c2);
	if (gpio_is_valid(platform->pdata->gpio_usb_ptn_c1))
		gpio_free(platform->pdata->gpio_usb_ptn_c1);
	if (gpio_is_valid(platform->pdata->gpio_vconn_5v_en))
		gpio_free(platform->pdata->gpio_vconn_5v_en);
	if (gpio_is_valid(platform->pdata->gpio_fsa3030_sel2))
		gpio_free(platform->pdata->gpio_fsa3030_sel2);
	if (gpio_is_valid(platform->pdata->gpio_fsa3030_sel1))
		gpio_free(platform->pdata->gpio_fsa3030_sel1);
	if (gpio_is_valid(platform->pdata->gpio_fsa3030_sel0))
		gpio_free(platform->pdata->gpio_fsa3030_sel0);
	if (gpio_is_valid(platform->pdata->gpio_aud_hsdet))
		gpio_free(platform->pdata->gpio_aud_hsdet);

	gpio_free(platform->pdata->gpio_intr_comm);
#ifdef SUP_VBUS_CTL
	gpio_free(platform->pdata->gpio_vbus_ctrl);
#endif
}

static int anx7688_init_gpio(struct anx7688_data *platform)
{
	int ret = 0;

	pr_debug("%s: anx7688 init gpio\n", __func__);
	/*  gpio for chip power down  */
	ret = gpio_request(platform->pdata->gpio_p_on, "anx7688_p_on_ctl");
	if (ret) {
		pr_err("%s : failed to request p_on gpio %d\n", __func__,
			platform->pdata->gpio_p_on);
		goto err0;
	}
	gpio_direction_output(platform->pdata->gpio_p_on, 0);
	/*  gpio for chip reset  */
	ret = gpio_request(platform->pdata->gpio_reset, "anx7688_reset_n");
	if (ret) {
		pr_err("%s : failed to request reset gpio %d\n", __func__,
				platform->pdata->gpio_reset);
		goto err1;
	}
	gpio_direction_output(platform->pdata->gpio_reset, 0);

	/*  gpio for cable detect  */
	ret = gpio_request(platform->pdata->gpio_cbl_det, "anx7688_cbl_det");
	if (ret) {
		pr_err("%s : failed to request cbl_det gpio %d\n", __func__,
				platform->pdata->gpio_cbl_det);
		goto err2;
	}
	gpio_direction_input(platform->pdata->gpio_cbl_det);

	/*  gpio for audio accessory cable interrupt  */
	ret = gpio_request(platform->pdata->gpio_aud_hsdet, "aud_hsdet");
	if (ret) {
		pr_err("%s : failed to request hsdet gpio %d\n", __func__,
				platform->pdata->gpio_aud_hsdet);
	}
	gpio_direction_output(platform->pdata->gpio_aud_hsdet, 1);

	/*  gpio for audio accessory data path switch  */
	ret = gpio_request(platform->pdata->gpio_fsa3030_sel0, "fsa3030_sel0");
	if (ret) {
		pr_err("%s : failed to request sel0 gpio %d\n", __func__,
				platform->pdata->gpio_fsa3030_sel0);
	}
	gpio_direction_output(platform->pdata->gpio_fsa3030_sel0, 0);

	/*  gpio for audio accessory data path switch  */
	ret = gpio_request(platform->pdata->gpio_fsa3030_sel1, "fsa3030_sel1");
	if (ret) {
		pr_err("%s : failed to request sel1 gpio %d\n", __func__,
				platform->pdata->gpio_fsa3030_sel1);
	}
	gpio_direction_output(platform->pdata->gpio_fsa3030_sel1, 1);

	/*  gpio for audio accessory data path switch  */
	ret = gpio_request(platform->pdata->gpio_fsa3030_sel2, "fsa3030_sel2");
	if (ret) {
		pr_err("%s : failed to request sel2 gpio %d\n", __func__,
				platform->pdata->gpio_fsa3030_sel2);
	}
	gpio_direction_output(platform->pdata->gpio_fsa3030_sel2, 0);

	/*  gpio for Vconn 5V */
	ret = gpio_request(platform->pdata->gpio_vconn_5v_en, "vconn_5v");
	if (ret) {
		pr_err("%s : failed to request vconn gpio %d\n", __func__,
				platform->pdata->gpio_vconn_5v_en);
	}
	gpio_direction_output(platform->pdata->gpio_vconn_5v_en, 0);

	/*  gpio for USB3.0 redriver PTN_C1 */
	ret = gpio_request(platform->pdata->gpio_usb_ptn_c1, "usb_ptn_c1");
	if (ret) {
		pr_err("%s : failed to request c1 gpio %d\n", __func__,
				platform->pdata->gpio_usb_ptn_c1);
	}
	gpio_direction_output(platform->pdata->gpio_usb_ptn_c1, 0);

	/*  gpio for USB3.0 redriver PTN_C2 */
	ret = gpio_request(platform->pdata->gpio_usb_ptn_c2, "usb_ptn_c2");
	if (ret) {
		pr_err("%s : failed to request c2 gpio %d\n", __func__,
				platform->pdata->gpio_usb_ptn_c2);
	}
	gpio_direction_output(platform->pdata->gpio_usb_ptn_c2, 1);

	/*  gpio for CC/SBU OVP */
	ret = gpio_request(platform->pdata->gpio_ovp_cc_sbu, "ovp_cc_sbu");
	if (ret) {
		pr_err("%s : failed to request ovp gpio %d\n", __func__,
				platform->pdata->gpio_ovp_cc_sbu);
	}
	gpio_direction_input(platform->pdata->gpio_ovp_cc_sbu);

	/*  gpio for chip interface communaction */
	ret = gpio_request(platform->pdata->gpio_intr_comm, "anx7688_intr_comm");
	if (ret) {
		pr_err("%s : failed to request intr gpio %d\n", __func__,
				platform->pdata->gpio_intr_comm);
		goto err3;
	}
	gpio_direction_input(platform->pdata->gpio_intr_comm);

#ifdef SUP_VBUS_CTL
	/*  gpio for vbus control  */
	ret = gpio_request(platform->pdata->gpio_vbus_ctrl, "anx7688_vbus_ctrl");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
				platform->pdata->gpio_vbus_ctrl);
		goto err4;
	}
	gpio_direction_output(platform->pdata->gpio_vbus_ctrl, 0);
#endif

	goto out;

#ifdef SUP_VBUS_CTL
err4:
	gpio_free(platform->pdata->gpio_intr_comm);
#endif

err3:
	if (gpio_is_valid(platform->pdata->gpio_ovp_cc_sbu))
		gpio_free(platform->pdata->gpio_ovp_cc_sbu);
	if (gpio_is_valid(platform->pdata->gpio_usb_ptn_c2))
		gpio_free(platform->pdata->gpio_usb_ptn_c2);
	if (gpio_is_valid(platform->pdata->gpio_usb_ptn_c1))
		gpio_free(platform->pdata->gpio_usb_ptn_c1);
	if (gpio_is_valid(platform->pdata->gpio_vconn_5v_en))
		gpio_free(platform->pdata->gpio_vconn_5v_en);
	if (gpio_is_valid(platform->pdata->gpio_fsa3030_sel2))
		gpio_free(platform->pdata->gpio_fsa3030_sel2);
	if (gpio_is_valid(platform->pdata->gpio_fsa3030_sel1))
		gpio_free(platform->pdata->gpio_fsa3030_sel1);
	if (gpio_is_valid(platform->pdata->gpio_fsa3030_sel0))
		gpio_free(platform->pdata->gpio_fsa3030_sel0);
	if (gpio_is_valid(platform->pdata->gpio_aud_hsdet))
		gpio_free(platform->pdata->gpio_aud_hsdet);

	gpio_free(platform->pdata->gpio_cbl_det);
err2:
	gpio_free(platform->pdata->gpio_reset);
err1:
	gpio_free(platform->pdata->gpio_p_on);
err0:
	return -EINVAL;
out:
	return 0;
}


#ifdef CONFIG_OF
static int anx7688_parse_dt(struct device *dev, struct anx7688_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	struct platform_device *hdmi_pdev = NULL;
	struct device_node *hdmi_tx_node = NULL;
	int ret = 0;

	pdata->gpio_p_on =
		of_get_named_gpio_flags(np, "analogix,p-on-gpio", 0, NULL);

	pdata->gpio_reset =
		of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

	pdata->gpio_cbl_det =
		of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);

#ifdef SUP_VBUS_CTL
	pdata->gpio_vbus_ctrl =
		of_get_named_gpio_flags(np, "analogix,v33-ctrl-gpio", 0, NULL); /*reuse previous unless gpio(v33_ctrl) for vbus control*/
//	    of_get_named_gpio_flags(np, "analogix,vbus-ctrl-gpio", 0, NULL);
#endif

	pdata->gpio_intr_comm =
		of_get_named_gpio_flags(np, "analogix,intr-comm-gpio", 0, NULL);

	pdata->gpio_aud_hsdet =
		of_get_named_gpio_flags(np, "htc,aud-hsdet-gpio", 0, NULL);
	pdata->gpio_fsa3030_sel0 =
		of_get_named_gpio_flags(np, "htc,usb_hph_fsa3030_sel0", 0, NULL);
	pdata->gpio_fsa3030_sel1 =
		of_get_named_gpio_flags(np, "htc,usb_hph_fsa3030_sel1", 0, NULL);
	pdata->gpio_fsa3030_sel2 =
		of_get_named_gpio_flags(np, "htc,usb_hph_fsa3030_sel2", 0, NULL);

	pdata->gpio_vconn_5v_en =
		of_get_named_gpio_flags(np, "htc,vconn_5v_en", 0, NULL);

	pdata->gpio_usb_ptn_c1 =
		of_get_named_gpio_flags(np, "htc,usb-ptn-c1-gpio", 0, NULL);
	pdata->gpio_usb_ptn_c2 =
		of_get_named_gpio_flags(np, "htc,usb-ptn-c2-gpio", 0, NULL);

	pdata->gpio_ovp_cc_sbu =
		of_get_named_gpio_flags(np, "htc,ovp-cc-sbu", 0, NULL);

	pdata->vdd10 = devm_regulator_get(dev, "vdd10");
	if (IS_ERR(pdata->vdd10)) {
		ret = PTR_ERR(pdata->vdd10);
		pr_err("%s : Regulator get failed vdd10 rc=%d\n",
		       __func__, ret);
		pdata->vdd10 = NULL;
		return ret;
	}

	pdata->vdd18 = devm_regulator_get(dev, "vdd18");
	if (IS_ERR(pdata->vdd18)) {
		ret = PTR_ERR(pdata->vdd18);
		pr_err("%s : Regulator get failed vdd18 rc=%d\n",
		       __func__, ret);
		pdata->vdd18 = NULL;
	}

	pr_info("p_on %d, reset %d, cbl_det %d, intr_comm %d, hsdet %d, \
sel0 %d, sel1 %d, sel2 %d, vconn %d, c1 %d, c2 %d, ovp %d\n",
			pdata->gpio_p_on, pdata->gpio_reset, pdata->gpio_cbl_det,
			pdata->gpio_intr_comm, pdata->gpio_aud_hsdet, pdata->gpio_fsa3030_sel0,
			pdata->gpio_fsa3030_sel1, pdata->gpio_fsa3030_sel2, pdata->gpio_vconn_5v_en,
			pdata->gpio_usb_ptn_c1, pdata->gpio_usb_ptn_c2, pdata->gpio_ovp_cc_sbu);

	/* parse phandle for hdmi tx handle */
	hdmi_tx_node = of_parse_phandle(np, "analogix,hdmi-tx-map", 0);
	if (!hdmi_tx_node) {
		pr_err("can't find hdmi phandle\n");
		ret = -EINVAL;
	} else {
		hdmi_pdev = of_find_device_by_node(hdmi_tx_node);
		if (!hdmi_pdev) {
			pr_err("can't find the deivce by node\n");
			ret = -EINVAL;
		}
		pdata->hdmi_pdev = hdmi_pdev;
		of_node_put(hdmi_tx_node);
	}

	pdata->disable_hbr25 = of_property_read_bool(np, "htc,disable-hbr25");;

	return ret;
}
#else
static int anx7688_parse_dt(struct device *dev, struct anx7688_platform_data *pdata)
{
	return -ENODEV;
}
#endif

static enum dual_role_property anx7688_dual_role_properties[] = {
	DUAL_ROLE_PROP_SUPPORTED_MODES,
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
	DUAL_ROLE_PROP_VCONN_SUPPLY,
	DUAL_ROLE_PROP_PARTNER_SUPPORTS_USB_PD,
};

static const struct dual_role_phy_desc anx7688_dual_role_desc = {
	.name = "otg_default",
	.properties = anx7688_dual_role_properties,
	.num_properties = ARRAY_SIZE(anx7688_dual_role_properties),
	.get_property = anx7688_dual_role_get_property,
	.set_property = anx7688_dual_role_set_property,
	.property_is_writeable = anx7688_dual_role_property_is_writeable,
};

static int anx7688_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{

	struct anx7688_data *platform;
	struct anx7688_platform_data *pdata;
	int ret = 0;
	struct pinctrl *anx7688_pinctrl;
	struct pinctrl_state *anx7688_pinctrl_state;
	struct power_supply *batt_psy;
	struct msm_hdmi_sp_ops *hdmi_sp_ops = NULL;

	pr_info("anx7688_i2c_probe\n");
	if ((get_debug_flag() & 0x200)  && !((strcmp(htc_get_bootmode(), "download") == 0)
		|| (strcmp(htc_get_bootmode(), "RUU") == 0))) {
		pr_info("%s skip ANX7688 driver probe on %s mode\n", __func__, htc_get_bootmode());
		goto exit;
	}

	batt_psy = power_supply_get_by_name("battery");
	if (!batt_psy) {
		pr_err("battery supply not found, deferring probe\n");
		return -EPROBE_DEFER;
	}

	if (!i2c_check_functionality(client->adapter,
								I2C_FUNC_SMBUS_I2C_BLOCK)) {
		pr_err("%s:anx7688's i2c bus doesn't support\n", __func__);
		ret = -ENODEV;
		goto exit;
	}

	platform = kzalloc(sizeof(struct anx7688_data), GFP_KERNEL);
	if (!platform) {
		pr_err("%s: failed to allocate driver data\n", __func__);
		ret = -ENOMEM;
		goto exit;
	}

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
							sizeof(struct anx7688_platform_data),
							GFP_KERNEL);
		if (!pdata) {
			pr_err("%s: Failed to allocate memory\n", __func__);
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;

		/* device tree parsing function call */
		ret = anx7688_parse_dt(&client->dev, pdata);
		if (ret != 0)	/* if occurs error */
			goto err0;

		platform->pdata = pdata;
	} else {
		platform->pdata = client->dev.platform_data;
	}

	/* initialize hdmi_sp_ops */
	hdmi_sp_ops = devm_kzalloc(&client->dev,
				   sizeof(struct msm_hdmi_sp_ops),
				   GFP_KERNEL);
	if (!hdmi_sp_ops) {
		pr_err("alloc hdmi sp ops failed\n");
		goto err0;
	}

	if (platform->pdata->hdmi_pdev) {
		ret = msm_hdmi_register_sp(platform->pdata->hdmi_pdev,
					  hdmi_sp_ops);
		if (ret) {
			pr_err("register with hdmi as downstream failed\n");
			goto err0;
		}
	}
	platform->pdata->hdmi_sp_ops = hdmi_sp_ops;

	/* to access global platform data */
	g_pdata = platform->pdata;
	g_data = platform;
	anx7688_client = client;
	anx7688_client->addr = (device_addr >> 1);
	i2c_set_clientdata(client, platform);

	atomic_set(&anx7688_power_status, 0);
	platform->batt_psy = batt_psy;

	mutex_init(&platform->lock);

	if (!platform->pdata) {
		ret = -EINVAL;
		goto err0;
	}

	ret = anx7688_init_gpio(platform);
	if (ret) {
		pr_err("%s: failed to initialize gpio\n", __func__);
		goto err0;
	}

#if AUTO_UPDATE_OCM_FW
	INIT_DELAYED_WORK(&platform->work, anx7688_work_func);
#endif
	INIT_DELAYED_WORK(&platform->drole_work, anx7688_drole_work_func);

	platform->workqueue = create_singlethread_workqueue("anx7688_work");
	if (platform->workqueue == NULL) {
		pr_err("%s: failed to create work queue\n", __func__);
		ret = -ENOMEM;
		goto err1;
	}

	anx7688_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(anx7688_pinctrl)) {
#ifdef CONFIG_OF
		if (of_property_read_bool(client->dev.of_node, "pinctrl-names")) {
			pr_err("Error encountered while getting pinctrl\n");
			ret = -EINVAL;
			goto err1;
		}
#endif
		pr_err("Target does not use pinctrl\n");
		anx7688_pinctrl = NULL;
	}

	if (anx7688_pinctrl) {
		anx7688_pinctrl_state = pinctrl_lookup_state(anx7688_pinctrl , "anx7688_cbl_det");
		if (IS_ERR(anx7688_pinctrl_state))
			pr_err("anx7688_cbl_det pinctrl lookup fail\n");

		ret = pinctrl_select_state(anx7688_pinctrl , anx7688_pinctrl_state);
		if (ret < 0)
			pr_err("anx7688_cbl_det pinctrl select fail\n");

		anx7688_pinctrl_state = pinctrl_lookup_state(anx7688_pinctrl , "anx7688_p_on");
		if (IS_ERR(anx7688_pinctrl_state))
			pr_err("anx7688_p_on pinctrl lookup fail\n");

		ret = pinctrl_select_state(anx7688_pinctrl , anx7688_pinctrl_state);
		if (ret < 0)
			pr_err("anx7688_p_on pinctrl select fail\n");

		anx7688_pinctrl_state = pinctrl_lookup_state(anx7688_pinctrl , "anx7688_intr_comm");
		if (IS_ERR(anx7688_pinctrl_state))
			pr_err("anx7688_intr_comm pinctrl lookup fail\n");

		ret = pinctrl_select_state(anx7688_pinctrl , anx7688_pinctrl_state);
		if (ret < 0)
			pr_err("anx7688_intr_comm pinctrl select fail\n");

		anx7688_pinctrl_state = pinctrl_lookup_state(anx7688_pinctrl , "fsa3030_default");
		if (IS_ERR(anx7688_pinctrl_state))
			pr_err("fsa3030_default pinctrl lookup fail\n");

		ret = pinctrl_select_state(anx7688_pinctrl , anx7688_pinctrl_state);
		if (ret < 0)
			pr_err("fsa3030_default pinctrl select fail\n");

		anx7688_pinctrl_state = pinctrl_lookup_state(anx7688_pinctrl , "usb3_redriver");
		if (IS_ERR(anx7688_pinctrl_state))
			pr_err("usb3_redriver pinctrl lookup fail\n");

		ret = pinctrl_select_state(anx7688_pinctrl , anx7688_pinctrl_state);
		if (ret < 0)
			pr_err("usb3_redriver pinctrl select fail\n");

		anx7688_pinctrl_state = pinctrl_lookup_state(anx7688_pinctrl , "ovp_cc_sbu");
		if (IS_ERR(anx7688_pinctrl_state))
			pr_err("ovp_cc_sbu pinctrl lookup fail\n");

		ret = pinctrl_select_state(anx7688_pinctrl , anx7688_pinctrl_state);
		if (ret < 0)
			pr_err("ovp_cc_sbu pinctrl select fail\n");
	}

	// Set USB3.0 redriver IC (0,0)
	usb3_redriver_ctl(C1C2_LOW, C1C2_LOW);

	wake_lock_init(&platform->anx7688_lock, WAKE_LOCK_SUSPEND, "anx7688_wake_lock");

	platform->pmode = MODE_UNKNOWN;
	platform->drole = UNKNOWN_DATA_ROLE;
	platform->prole = UNKNOWN_POWER_ROLE;
	platform->vconn = VCONN_SUPPLY_NO;

	platform->anx7688_dual_role_instance =
		devm_dual_role_instance_register(&client->dev, &anx7688_dual_role_desc);
	if (IS_ERR(platform->anx7688_dual_role_instance)) {
		pr_err("%s: dual_role_instance register fail\n", __func__);
		goto err1;
	}

	client->irq = gpio_to_irq(platform->pdata->gpio_intr_comm);
	if (client->irq < 0) {
		pr_err("%s : failed to get anx7688 gpio comm irq\n", __func__);
		goto err2;
	}

	ret = request_threaded_irq(client->irq, NULL, anx7688_intr_comm_isr,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "anx7688-intr-comm", platform);

	if (ret < 0) {
		pr_err("%s : failed to request interface irq\n", __func__);
		goto err2;
	}

	ret = irq_set_irq_wake(client->irq, 1);
	if (ret < 0) {
		pr_err("%s : Request irq_wake for interface communaction", __func__);
		goto err3;
	}

	ret = enable_irq_wake(client->irq);
	if (ret < 0) {
		pr_err("%s : Enable irq_wake for interface communaction", __func__);
		goto err3;
	}

	ret = create_sysfs_interfaces(&client->dev);
	if (ret < 0) {
		pr_err("%s : sysfs register failed", __func__);
	}

	/*when probe anx7688 device, enter standy mode */
	anx7688_power_standby();
#if AUTO_UPDATE_OCM_FW
	/*add work function*/
	queue_delayed_work(platform->workqueue, &platform->work, msecs_to_jiffies(100));
#endif // AUTO_UPDATE_OCM_FW

	pr_info("anx7688_i2c_probe successfully end\n");
	goto exit;

err3:
	free_irq(client->irq, platform);
err2:
	devm_dual_role_instance_unregister(&client->dev, platform->anx7688_dual_role_instance);
	destroy_workqueue(platform->workqueue);
err1:
	anx7688_free_gpio(platform);
err0:
	anx7688_client = NULL;
	g_data = NULL;
	g_pdata = NULL;
	kfree(platform);
exit:
	return ret;
}

static int anx7688_i2c_remove(struct i2c_client *client)
{
	struct anx7688_data *platform = i2c_get_clientdata(client);
	pr_info("anx7688_i2c_remove\n");
	free_irq(client->irq, platform);
	devm_dual_role_instance_unregister(&client->dev, platform->anx7688_dual_role_instance);
	anx7688_free_gpio(platform);
	destroy_workqueue(platform->workqueue);
	wake_lock_destroy(&platform->anx7688_lock);
	kfree(platform);
	return 0;
}

static int anx7688_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
	return 0;
}

static int anx7688_i2c_resume(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id anx7688_id[] = {
	{"anx7688", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, anx7688_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
	{.compatible = "analogix,anx7688",},
	{},
};
#endif

static struct i2c_driver anx7688_driver = {
	.driver = {
		.name = "anx7688",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = anx_match_table,
#endif
	},
	.probe = anx7688_i2c_probe,
	.remove = anx7688_i2c_remove,
	.suspend = anx7688_i2c_suspend,
	.resume = anx7688_i2c_resume,
	.id_table = anx7688_id,
};

static void __init anx7688_init_async(void *data, async_cookie_t cookie)
{
	int ret = 0;

	ret = i2c_add_driver(&anx7688_driver);
	if (ret < 0)
		pr_err("%s: failed to register anx7688 i2c drivern", __func__);
}

static int __init anx7688_init(void)
{
	async_schedule(anx7688_init_async, NULL);
	return 0;
}

static void __exit anx7688_exit(void)
{
	i2c_del_driver(&anx7688_driver);
}

#ifdef OCM_DEBUG
void dump_reg(void)
{
	int i = 0;
	u8 val = 0;

	printk("dump registerad:\n");
	printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
	for (i = 0; i < 256; i++) {
		val = ReadReg(i);

		if ((i) % 0x10 == 0x00)
			printk("\n[%x]:%02x ", i, val);
		else
			printk("%02x ", val);

	}
	printk("\n");
}

ssize_t anx7688_send_pd_cmd(struct device *dev,
							struct device_attribute *attr,
							const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%d", &cmd);
	switch (cmd) {
	case TYPE_PWR_SRC_CAP:
		send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
		break;

	case TYPE_DP_SNK_IDENTITY:
		send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
		break;

	case TYPE_PSWAP_REQ:
		send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
		break;
	case TYPE_DSWAP_REQ:
		send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
		break;

	case TYPE_GOTO_MIN_REQ:
		send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
		break;

	case TYPE_PWR_OBJ_REQ:
		interface_send_request();
		break;
	case TYPE_ACCEPT:
		interface_send_accept();
		break;
	case TYPE_REJECT:
		interface_send_reject();
		break;
	case TYPE_SOFT_RST:
		send_pd_msg(TYPE_SOFT_RST, 0, 0);
		break;
	case TYPE_HARD_RST:
		send_pd_msg(TYPE_HARD_RST, 0, 0);
		break;

	case 0xFD:
		pr_info("fetch powerrole: %d\n", get_power_role());
		break;
	case 0xFE:
		pr_info("fetch datarole: %d\n", get_data_role());
		break;

	case 0xff:
		dump_reg();
		break;
	}
	return count;
}


ssize_t anx7688_send_pswap(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_power_swap());
}

ssize_t anx7688_send_dswap(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", send_data_swap());
}

ssize_t anx7688_try_source(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_source());
}

ssize_t anx7688_try_sink(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", try_sink());
}

ssize_t anx7688_get_data_role(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx7688_get_power_role(struct device *dev,
							struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx7688_rd_reg(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int cmd;
	int result;

	result = sscanf(buf, "%x", &cmd);
	printk("reg[%x] = %x\n", cmd, ReadReg(cmd));

	return count;

}

ssize_t anx7688_wr_reg(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int cmd, val;
	int result;

	result = sscanf(buf, "%x  %x", &cmd, &val);
	pr_info("c %x val %x\n", cmd, val);
	WriteReg(cmd, val);
	pr_info("reg[%x] = %x\n", cmd, ReadReg(cmd));
	return count;
}

ssize_t anx7688_dump_register(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	int i = 0;
	for (i = 0; i < 256; i++) {
		if (i % 0x10 == 0)
			pr_info("\n");
		printk(" %.2x", ReadReg(i));

		snprintf(&buf[i], sizeof(u8), "%d", ReadReg(i));
	}

	printk("\n");

	return i;
}

ssize_t anx7688_select_rdo_index(struct device *dev,
								struct device_attribute *attr,
								const char *buf, size_t count)
{
	int cmd;
	cmd = sscanf(buf, "%d", &cmd);
	if (cmd <= 0)
		return 0;

	pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
	sel_voltage_pdo_index = cmd;
	return count;
}

ssize_t anx7688_chg_addr(struct device *dev,
						struct device_attribute *attr,
						const  char *buf, size_t count)
{
	int val;
	int result;

	result = sscanf(buf, "%x", &val);
	device_addr = (unsigned char) val;
	pr_info( "Change Device Address to 0x%x\n", (int)device_addr);
	return count;
}



ssize_t anx7688_burn_hex(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	burnhex();

	return 256+32;
}

ssize_t anx7688_read_hex(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	printk( "readhex() \n");
	readhex();
	printk("\n");

	return 256+32;
}

ssize_t anx7688_debug(struct device *dev,
						struct device_attribute *attr,
						const char *buf, size_t count)
{
	int param[4];
	int result,i;
	char CommandName[10];
	extern unsigned char debug_on;

	memset(param,0,sizeof(param));
	result = sscanf(buf, "%s %d %d %d %d",CommandName, param, param+1, param+2, param+3);
	//printk("anx7688 count: %d\n", (int)count);
	//printk("anx7688 buf: %s", buf);
	//printk("anx7688 param no: %d\n", result);
	printk("anx7688 cmd[%s", CommandName);
	for(i=0; i<result-1; i++)
		printk(" %d", param[i]);
	printk("]\n");

	if(strcmp(CommandName, "poweron") == 0) {
		printk("MI1_power_on\n");
		MI1_power_on();
	} else if(strcmp(CommandName, "powerdown") == 0) {
		anx7688_power_standby();
	} else if(strcmp(CommandName, "debugon") == 0) {
		debug_on = 1;
		printk("debug_on = %d\n",debug_on);
	} else if(strcmp(CommandName, "debugoff") == 0) {
		debug_on = 0;
		printk("debug_on = %d\n",debug_on);
	} else if(strcmp(CommandName, "burntest") == 0) {
		extern void burntest(unsigned short int select);
		burntest(2);
	} else {
		printk("Usage:\n");
		printk("  echo poweron > cmd       : power on\n");
		printk("  echo powerdown > cmd     : power off\n");
		printk("  echo debugon > cmd       : debug on\n");
		printk("  echo debugoff > cmd      : debug off\n");
		printk("  echo burntest > cmd      : write test fw\n");
	}

	return count;
}
#endif

static ssize_t disable_hbr25_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct anx7688_platform_data *pdata = dev_get_platdata(dev);
	int ret;

	ret = scnprintf(buf, PAGE_SIZE, "%d\n", pdata->disable_hbr25);

	return ret;
}

static ssize_t disable_hbr25_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct anx7688_platform_data *pdata = dev_get_platdata(dev);
	int ret, value;

	ret = kstrtoint(buf, 10, &value);
	if (ret) {
		pr_err("Invalid input for %s\n", attr->attr.name);
		return -EINVAL;
	}
	pdata->disable_hbr25 = !!value;

	return count;
}

/* for debugging */
static struct device_attribute anx7688_device_attrs[] = {
#ifdef OCM_DEBUG
	__ATTR(pdcmd, S_IWUSR, NULL, anx7688_send_pd_cmd),
	__ATTR(rdreg, S_IWUSR, NULL, anx7688_rd_reg),
	__ATTR(wrreg, S_IWUSR, NULL, anx7688_wr_reg),
	__ATTR(addr, S_IWUSR, NULL, anx7688_chg_addr),
	__ATTR(rdoidx, S_IWUSR, NULL, anx7688_select_rdo_index),
	__ATTR(dumpreg, S_IRUGO , anx7688_dump_register, NULL),
	__ATTR(prole, S_IRUGO, anx7688_get_power_role, NULL),
	__ATTR(drole, S_IRUGO, anx7688_get_data_role, NULL),
	__ATTR(trysrc, S_IRUGO, anx7688_try_source, NULL),
	__ATTR(trysink, S_IRUGO, anx7688_try_sink, NULL),
	__ATTR(pswap, S_IRUGO, anx7688_send_pswap, NULL),
	__ATTR(dswap, S_IRUGO, anx7688_send_dswap, NULL),
	__ATTR(burnhex, S_IWUSR, NULL, anx7688_burn_hex),
	__ATTR(readhex, S_IRUGO, anx7688_read_hex, NULL),
	__ATTR(cmd, S_IWUSR, NULL, anx7688_debug),
#endif
	__ATTR(vconn_en, S_IWUSR, NULL, anx7688_vconn_en_store),
	__ATTR(c1c2, S_IWUSR, NULL, anx7688_redriver_cfg_store),
	__ATTR(disable_hbr25, S_IWUSR | S_IRUGO, disable_hbr25_show, disable_hbr25_store),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	pr_debug("anx7688 create system fs interface ...\n");
	for (i = 0; i < ARRAY_SIZE(anx7688_device_attrs); i++)
		if (device_create_file(dev, &anx7688_device_attrs[i]))
			goto error;
	pr_debug("success\n");
	return 0;
error:

	for (; i >= 0; i--)
		device_remove_file(dev, &anx7688_device_attrs[i]);
	pr_err("%s %s: anx7688 Unable to create interface", LOG_TAG, __func__);
	return -EINVAL;
}

module_init(anx7688_init);
module_exit(anx7688_exit);

MODULE_DESCRIPTION("USB PD Anx7688 driver");
MODULE_AUTHOR("Xia Junhua <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION(ANX7688_DRV_VERSION);
