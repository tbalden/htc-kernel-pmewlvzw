/*
 * siw_touch_of.c - SiW touch device tree driver
 *
 * Copyright (C) 2016 Silicon Works - http://www.siliconworks.co.kr
 * Author: Hyunho Kim <kimhh@siliconworks.co.kr>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/gpio.h>
#include <linux/string.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <asm/page.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <asm/memory.h>

#include "siw_touch.h"

#ifndef __weak
#define __weak __attribute__((weak))
#endif

#define LCM_CMD_SIZE 256
char disp_info_str[LCM_CMD_SIZE] = {0x00};

#if defined(__SIW_CONFIG_OF)	//See siw_touch_cfg.h
static int siw_touch_of_gpio(struct device *dev, void *np,
					void *string, enum of_gpio_flags *flags)
{
	int gpio = 0;

	gpio = of_get_named_gpio_flags(np, string, 0, flags);
	if (gpio_is_valid(gpio)) {
		if (flags)
			t_dev_info(dev, "of gpio  : %s(0x%X), %d\n",
					(char *)string, (*flags), gpio);
		else
			t_dev_info(dev, "of gpio  : %s, %d\n",
					(char *)string, gpio);
	}

	return gpio;
}

static bool siw_touch_of_bool(struct device *dev, void *np,
					void *string)
{
	u32 val = 0;
	int ret = 0;

	ret = of_property_read_u32(np, string, &val);
	if (ret < 0) {
		return 0;
	}

	t_dev_dbg_of(dev, "of bool  : %s, %d\n", (char *)string, val);
	return (bool)val;
}

static u32 siw_touch_of_u32(struct device *dev, void *np,
					void *string)
{
	u32 val = 0;
	int ret = 0;

	ret = of_property_read_u32(np, string, &val);
	if (ret < 0) {
		return 0;
	}

	t_dev_dbg_of(dev, "of u32   : %s, %d\n", (char *)string, val);
	return val;
}

/* Caution : MSB(bit31) unavailable */
static int siw_touch_of_int(struct device *dev, void *np,
					void *string)
{
	u32 val = 0;
	int ret = 0;

	ret = of_property_read_u32(np, string, &val);
	if (ret < 0) {
		return -ENXIO;
	}

	t_dev_dbg_of(dev, "of int   : %s, %d\n", (char *)string, val);
	return val;
}

static void siw_touch_of_array(struct device *dev, void *np,
					void *string, const char **name, int *cnt)
{
	int i;
	int ret = 0;

	(*cnt) = 0;

	ret = of_property_count_strings(np, string);
	if (ret < 0) {
		t_dev_warn(dev, "of count : %s not found\n", (char *)string);
		return;
	}

	(*cnt) = ret;
	t_dev_dbg_of(dev, "of count : %s, %d\n", (char *)string, (*cnt));
	for (i = 0; i < (*cnt); i++) {
		ret = of_property_read_string_index(np, string, i, (const char **)&name[i]);
		if (!ret) {
			t_dev_info(dev, "of string[%d/%d] : %s\n", i+1, (*cnt), name[i]);
		}
	}
}

static int siw_touch_of_string(struct device *dev, void *np,
					void *string, const char **name)
{
	int ret;

	ret = of_property_read_string(np, string, name);
	if (!ret) {
		t_dev_info(dev, "of_string : %s, %s\n", (char *)string, *name);
	} else {
		t_dev_info(dev, "of_string : %s not found\n", (char *)string);
	}

	return ret;
}

static int siw_touch_parse_dts_watch(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device_node *np = dev->of_node;
#if !defined(CONFIG_TOUCHSCREEN_HTC_DEBUG)
	int ret = 0;
#endif

	if (touch_test_quirks(ts, CHIP_QUIRK_NOT_SUPPORT_WATCH)) {
		goto out;
	}

#if defined(CONFIG_TOUCHSCREEN_HTC_DEBUG)
	siw_touch_of_array(dev, np, "font_image", ts->watch_font_image_list, &ts->def_font_cnt);
	if (ts->def_font_cnt >= MAX_FONT_IMAGE_COUNT)
		ts->def_font_cnt = MAX_FONT_IMAGE_COUNT;

	if (ts->watch_font_image_list)
		ts->watch_font_image = ts->watch_font_image_list[ts->font_index];
#else
	ret = siw_touch_of_string(dev, np, "font_image", &ts->watch_font_image);
#endif

out:
	return 0;
}

/*
 * weak(dummy) function for PRD control
 * These are deactivated by enabling __SIW_SUPPORT_PRD
 * and the actual functions can be found in siw_touch_hal_prd.c
 */
int __weak siw_hal_set_prd_file(struct device *dev, const char *path, int idx)
{
	t_dev_warn(dev, "PRD disabled\n");
	return 0;
}

static int siw_touch_parse_dts_prd(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device_node *np = dev->of_node;
	int ret = 0;

	ret = siw_touch_of_string(dev, np, "prd_in_file", &ts->prd_in_file_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_in_file_path, 0);
	}
	ret = siw_touch_of_string(dev, np, "prd_in_file_m", &ts->prd_in_file_m_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_in_file_m_path, 1);
	}
	ret = siw_touch_of_string(dev, np, "prd_out_file", &ts->prd_out_file_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_out_file_path, 2);
	}
	ret = siw_touch_of_string(dev, np, "prd_out_file_mo_aat", &ts->prd_out_file_mo_aat_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_out_file_mo_aat_path, 3);
	}
	ret = siw_touch_of_string(dev, np, "prd_out_file_mo_mfo", &ts->prd_out_file_mo_mfo_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_out_file_mo_mfo_path, 4);
	}
	ret = siw_touch_of_string(dev, np, "prd_out_file_mo_mfl", &ts->prd_out_file_mo_mfl_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_out_file_mo_mfl_path, 5);
	}
	ret = siw_touch_of_string(dev, np, "prd_out_file_mo_mcv", &ts->prd_out_file_mo_mcv_path);
	if (!ret) {
		siw_hal_set_prd_file(dev, ts->prd_out_file_mo_mcv_path, 6);
	}

	return 0;
}

static uint32_t parse_eng_id(struct device *dev, uint32_t eng_id_mask){
	uint32_t eng_id = 0;
	uint8_t temp[4] = {0};
	struct device_node *np = of_find_node_by_path("/chosen/mfg");

	if (np) {
		if (of_property_read_u8_array(np, "skuid.engineer_id", temp, sizeof(uint32_t)/sizeof(uint8_t)))
			t_dev_err(dev, "Failed to get property: engineer_id\n");
	} else {
		t_dev_err(dev, "Failed to find device node contains engineer_id\n");
		return 0;
	}

	eng_id = temp[3] << 24 | temp[2] << 16 | temp[1] << 8 | temp[0];
	t_dev_dbg_of(dev, "eng_id_mask = 0x%08X, temp = 0x%02X%02X%02X%02X, eng_id = 0x%08X\n",
			eng_id_mask, temp[0], temp[1], temp[2], temp[3], eng_id);

	eng_id &= eng_id_mask;
	t_dev_dbg_of(dev, "eng_id = 0x%08X\n", eng_id);
	return eng_id;
}

static int siw_touch_do_parse_dts(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct device_node *np = dev->of_node;
	struct touch_pins *pins = &ts->pins;
	struct touch_device_caps *caps = &ts->caps;
	struct touch_operation_role *role = &ts->role;
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
	int tmp;
	u32 p_flags = 0;
	int chip_flags = 0;
	int irq_flags = 0;
	enum of_gpio_flags pin_flags;
	int pin_val;

	if (!ts->dev->of_node) {
		t_dev_err(dev, "No dts data\n");
		return -ENOENT;
	}

	t_dev_dbg_of(dev, "start dts parsing\n");

	pin_val = siw_touch_of_gpio(dev, np, "reset-gpio", &pin_flags);
	if (!gpio_is_valid(pin_val)) {
		return -EINVAL;
	}
	pins->reset_pin = pin_val;
	pins->reset_pin_pol = !!(pin_flags & OF_GPIO_ACTIVE_LOW);

	pin_val = siw_touch_of_gpio(dev, np, "irq-gpio", NULL);
	if (!gpio_is_valid(pin_val)) {
		return -EINVAL;
	}
	pins->irq_pin= pin_val;

#ifdef CONFIG_NANOHUB_TP_SWITCH
	pin_val = siw_touch_of_gpio(dev, np, "switch-gpio", NULL);
	if (!gpio_is_valid(pin_val)) {
		pins->switch_pin = -1;
		return -EINVAL;
	}
	pins->switch_pin = pin_val;
#endif

	pins->maker_id_pin = siw_touch_of_gpio(dev, np, "maker_id-gpio", NULL);

	/* Power */
	pins->vdd_pin = siw_touch_of_gpio(dev, np, "vdd-gpio", NULL);
	pins->vio_pin = siw_touch_of_gpio(dev, np, "vio-gpio", NULL);

	irq_flags = siw_touch_of_u32(dev, np, "irqflags");
	p_flags = pdata_irqflags(ts->pdata);
	if ((p_flags & TOUCH_IGNORE_DT_FLAGS) || (irq_flags < 0)) {
		ts->irqflags = p_flags;
	} else {
		ts->irqflags = irq_flags & 0xFFFF;
		ts->irqflags |= p_flags & (0xFFFFUL<<16);
	}
	if (irq_flags >= 0) {
		t_dev_info(dev, "irqflags(of) = 0x%08X (0x%08X, 0x%08X)\n",
			(u32)ts->irqflags, p_flags, irq_flags);
	} else {
		t_dev_info(dev, "irqflags(of) = 0x%08X (0x%08X, %d)\n",
			(u32)ts->irqflags, p_flags, irq_flags);
	}

	chip_flags = siw_touch_of_int(dev, np, "chip_flags");
	p_flags = pdata_flags(ts->pdata);
	if ((p_flags & TOUCH_IGNORE_DT_FLAGS) || (chip_flags < 0)) {
		ts->flags = p_flags;
	} else {
		ts->flags = chip_flags & 0xFFFF;
		ts->flags |= p_flags & (0xFFFFUL<<16);
	}
	if (chip_flags >= 0) {
		t_dev_info(dev, "flags(of) = 0x%08X (0x%08X, 0x%08X)\n",
			ts->flags, p_flags, chip_flags);
	} else {
		t_dev_info(dev, "flags(of) = 0x%08X (0x%08X, %d)\n",
			ts->flags, p_flags, chip_flags);
	}

	/* Caps */
	caps->max_x = siw_touch_of_u32(dev, np, "max_x");
	caps->max_y = siw_touch_of_u32(dev, np, "max_y");
	caps->max_pressure = siw_touch_of_u32(dev, np, "max_pressure");
	caps->max_width = siw_touch_of_u32(dev, np, "max_width");
	caps->max_orientation = siw_touch_of_u32(dev, np, "max_orientation");
	caps->max_id = siw_touch_of_u32(dev, np, "max_id");
	caps->hw_reset_delay = siw_touch_of_u32(dev, np, "hw_reset_delay");
	caps->sw_reset_delay = siw_touch_of_u32(dev, np, "sw_reset_delay");

	memcpy((void *)&ts->i_id, (void *)&ts->pdata->i_id, sizeof(ts->i_id));
	tmp = siw_touch_of_int(dev, np, "i_product");
	if (tmp >= 0)
		ts->i_id.product = tmp;
	tmp = siw_touch_of_int(dev, np, "i_vendor");
	if (tmp >= 0)
		ts->i_id.vendor = tmp;
	tmp = siw_touch_of_int(dev, np, "i_version");
	if (tmp >= 0)
		ts->i_id.version = tmp;

	/* Role */
	if (touch_test_quirks(ts, CHIP_QUIRK_NOT_SUPPORT_LPWG)) {
		role->use_lpwg = 0;
		role->use_lpwg_test = 0;
	} else {
		role->use_lpwg = siw_touch_of_bool(dev, np, "use_lpwg");
		role->use_lpwg_test = siw_touch_of_u32(dev, np, "use_lpwg_test");
	}

	mode_ctrl->support_glove = of_property_read_bool(np, "support_glove");
	if (mode_ctrl->support_glove) {
		t_dev_info(dev, "glove mode default off");
	}

#if defined(CONFIG_AK8789_HALLSENSOR)
	mode_ctrl->support_cover = of_property_read_bool(np, "support_cover");
#endif

	/* Parse eng_id */
	if (of_property_read_u32(np, "eng_id", &ts->eng_id) == 0) {
		t_dev_info(dev, "eng_id = 0x%08X\n", ts->eng_id);
	} else if (of_property_read_u32(np, "eng_id_mask", &ts->eng_id_mask) == 0) {
		ts->eng_id = parse_eng_id(dev, ts->eng_id_mask);
		t_dev_info(dev, "eng_id (filtered) = 0x%08X\n", ts->eng_id);
	} else {
		ts->eng_id = 0;
	}

	ts->u3_disp_setting =
		(strstr(disp_info_str, "video") ? true : false);
	t_dev_info(dev, "u3_disp_setting = %d\n", ts->u3_disp_setting);

	/* Firmware */
	role->use_firmware = siw_touch_of_bool(dev, np, "use_firmware");
	if (role->use_firmware == 1) {
		role->use_fw_upgrade = siw_touch_of_bool(dev, np, "use_fw_upgrade");

		siw_touch_of_array(dev, np, "fw_image", ts->def_fwpath, &ts->def_fwcnt);

		siw_touch_of_string(dev, np, "panel_spec", &ts->panel_spec);
		siw_touch_of_string(dev, np, "panel_spec_mfts", &ts->panel_spec_mfts);
	} else {
		role->use_fw_upgrade = 0;
		ts->def_fwcnt = 0;
		memset(ts->def_fwpath, 0, sizeof(ts->def_fwpath));
		ts->panel_spec = NULL;
		ts->panel_spec_mfts = NULL;
	}

	siw_touch_parse_dts_watch(ts);

	siw_touch_parse_dts_prd(ts);

	t_dev_info(dev,   "caps max_x           = %d\n", caps->max_x);
	t_dev_info(dev,   "caps max_y           = %d\n", caps->max_y);
	t_dev_dbg_of(dev, "caps max_pressure    = %d\n", caps->max_pressure);
	t_dev_dbg_of(dev, "caps max_width       = %d\n", caps->max_width);
	t_dev_dbg_of(dev, "caps max_orientation = %d\n", caps->max_orientation);
	t_dev_dbg_of(dev, "caps max_id          = %d\n", caps->max_id);
	t_dev_dbg_of(dev, "caps hw_reset_delay  = %d\n", caps->hw_reset_delay);
	t_dev_dbg_of(dev, "caps sw_reset_delay  = %d\n", caps->sw_reset_delay);
	t_dev_dbg_of(dev, "role use_lpwg        = %d\n", role->use_lpwg);
	t_dev_dbg_of(dev, "role use_lpwg_test   = %d\n", role->use_lpwg_test);
	t_dev_dbg_of(dev, "role use_firmware    = %d\n", role->use_firmware);
	t_dev_dbg_of(dev, "role use_fw_upgrade  = %d\n", role->use_fw_upgrade);

	t_dev_dbg_of(dev, "dts parsing done\n");

	return 0;
}

static int siw_touch_parse_dts(struct siw_ts *ts)
{
	struct siw_touch_fquirks *fquirks = touch_fquirks(ts);
	int ret = 0;

	if (fquirks->parse_dts) {
		ret = fquirks->parse_dts(ts);
		if (ret != -EAGAIN) {
			goto out;
		}
	}

	ret = siw_touch_do_parse_dts(ts);

out:
	return ret;
}


#else	/* __SIW_CONFIG_OF */

static int siw_touch_parse_dts(struct siw_ts *ts)
{
	struct device *dev = ts->dev;

	ts->irqflags = pdata_irqflags(ts->pdata);

	ts->flags = pdata_flags(ts->pdata);
	t_dev_info(dev, "flags = 0x%08X", ts->flags);

	touch_set_pins(ts, &ts->pdata->pins);
	touch_set_caps(ts, &ts->pdata->caps);

	memcpy((void *)&ts->i_id, (void *)&ts->pdata->i_id, sizeof(ts->i_id));

	/* Role */
	ts->role.use_lpwg = !touch_test_quirks(ts, CHIP_QUIRK_NOT_SUPPORT_LPWG);

	ts->role.use_firmware = 0;
	ts->role.use_fw_upgrade = 0;
	ts->def_fwcnt = 0;
	memset(ts->def_fwpath, 0, sizeof(ts->def_fwpath));
	ts->panel_spec = NULL;
	ts->panel_spec_mfts = NULL;

	ts->vdd_id = -1;
	ts->vio_id = -1;

	return 0;
}
#endif	/* __SIW_CONFIG_OF */

int siw_touch_parse_data(struct siw_ts *ts)
{
	int ret = 0;

	ret = siw_touch_parse_dts(ts);
	if (ret) {
		t_dev_err(ts->dev, "!! DTS parsing failed !!\n");
	}

	return ret;
}

static int __init get_disp_info(char *str)
{
	strlcpy(disp_info_str, str, LCM_CMD_SIZE);
	pr_info("%s: (%s)\n", __func__, disp_info_str);
	return 0;
} early_param("mdss_mdp.panel", get_disp_info);


