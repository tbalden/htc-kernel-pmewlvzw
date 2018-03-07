/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 * Copyright (C) 2007 Google Incorporated
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
#ifndef HTC_UTIL_H
#define HTC_UTIL_H

#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_dsi.h"

/* bl_cali will receive parameter from 1 to 20000
 * this mean can support scale rate from 0.0001 to 2
 */
#define CALIBRATION_DATA_PATH "/calibration_data"
#define DISP_FLASH_DATA "disp_flash"
#define DISP_FLASH_DATA_SIZE 64
#define LIGHT_CALI_OFFSET 36
#define LIGHT_CALI_SIZE 8
#define LIGHT_RATIO_INDEX 0
#define LIGHT_R_INDEX 1
#define LIGHT_G_INDEX 2
#define LIGHT_B_INDEX 3
#define RGB_CALI_DEF 255
#define RGB_GAIN_CHECK(x) (x>0 && x<256)
#define RGB_CALIBRATION(ori,comp) ((long)(ori*comp/RGB_CALI_DEF))
#define BL_CALI_DEF  10000
#define BL_CALI_MAX  20000
#define BRI_GAIN_CHECK(x) (x>0 && x<=20000)
#define BACKLIGHT_CALI(ori,comp) ((unsigned int)(ori*comp/BL_CALI_DEF))
#define VALID_CALI_BKLT(val,min,max) ((min) > (val) ? (min) : ((val) > (max) ? (max) : (val)))

enum {
	CABC_INDEX = 0,
	COLOR_TEMP_INDEX = 1,
	COLOR_PROFILE_INDEX = 2,
	VDDIO_INDEX = 3,
	BURST_SWITCH_INDEX = 4,
	BL_CALI_ENABLE_INDEX = 5,
	RGB_CALI_ENABLE_INDEX = 6,
};

void htc_register_camera_bkl(int level);
void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd);
void htc_register_sub_attrs(struct kobject *led_kobj);
void htc_unregister_sub_attrs(struct kobject *led_kobj);

void htc_set_cabc(struct msm_fb_data_type *mfd, bool force);
void htc_set_color_temp(struct msm_fb_data_type *mfd, bool force);
void htc_set_color_profile(struct msm_fb_data_type *mfd, bool force);
void htc_reset_status(void);
void htc_set_vddio_switch(struct msm_fb_data_type *mfd);

int htc_mdss_fb_create_sysfs(struct msm_fb_data_type *mfd);
void htc_mdss_fb_remove_sysfs(struct msm_fb_data_type *mfd);
int aod_send_notify(struct fb_info *info);

int compass_notifier_fn(struct notifier_block *nb,
                        unsigned long action, void *data);
void htc_set_burst(struct msm_fb_data_type *mfd);
bool htc_is_burst_bl_on(struct msm_fb_data_type *mfd, int value);
void htc_update_bl_cali_data(struct msm_fb_data_type *mfd);
void htc_update_rgb_cali_data(struct msm_fb_data_type *mfd, struct mdp_pcc_cfg_data *config);
#endif /* MDSS_FB_H */
