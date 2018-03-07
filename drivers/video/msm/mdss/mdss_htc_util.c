/* Copyright (c) 2009-2013, The Linux Foundation. All rights reserved.
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

#include <linux/debugfs.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/printk.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "mdss_htc_util.h"
#include "mdss_dsi.h"
#include "mdss_fb.h"
#include "mdss_mdp.h"
#include "mdss_panel.h"
#include <linux/CwMcuSensor.h>

struct attribute_status {
       char *title;
       u32 req_value;
       u32 cur_value;
       u32 def_value;
};

static struct attribute_status htc_attr_status[] = {
	{"cabc_level_ctl", 0, 0, 0},
	{"color_temp_ctl", 0, 0, 0},
	{"color_profile_ctl", 0, 0, 0},
	{"vddio_switch", 0, 0, 0},
	{"burst_switch", 0, 0, 0},
	{"bklt_cali_enable", 0, 0, 0},
	{"disp_cali_enable", 0, 0, 0},
};

static bool sre_enable;

static struct calibration_gain aux_gain;
#define RGB_MIN_COUNT   9
static ssize_t rgb_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%x\n%s%x\n%s%x\n", "GAIN_R=0x", aux_gain.R, "GAIN_G=0x",
				aux_gain.G, "GAIN_B=0x", aux_gain.B);
	return ret;
}

static ssize_t rgb_gain_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned temp, temp1, temp2;

	if (count < RGB_MIN_COUNT)
		return -EFAULT;

	/* Format:
	ex: echo f8 f0 ef > disp_cali
	    [gain.R] space [gain.G] space [gain.B]
	    +---+---+------+---+---+------+---+---+-
	bit   0   1    2     3   4     5    6   7
	ex:     f8             f0             ef
	*/
	/* min count = 9, format: gain.R gain.G gain.B */

	if (sscanf(buf, "%x %x %x ", &temp, &temp1, &temp2) != 3) {
		pr_err("%s sscanf buf fail\n",__func__);
	} else if (RGB_GAIN_CHECK(temp) && RGB_GAIN_CHECK(temp1) && RGB_GAIN_CHECK(temp2)) {
		aux_gain.R = temp;
		aux_gain.G = temp1;
		aux_gain.B = temp2;
		pr_info("%s %d, gain_r=%x, gain_g=%x, gain_b=%x \n",__func__, __LINE__,
				aux_gain.R, aux_gain.G, aux_gain.B);
	}

	return count;
}

static ssize_t bklt_gain_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%d\n", "GAIN_BKLT=", aux_gain.BKL);
	return ret;
}

static ssize_t bklt_gain_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int temp = 0;

	if (sscanf(buf, "%d", &temp) != 1) {
		pr_err("%s sscanf buf fail\n",__func__);
	} else if(BRI_GAIN_CHECK(temp)) {
		aux_gain.BKL = temp;
		pr_info("[DISP]%s %d, gain_bkl=%d \n",__func__, __LINE__, aux_gain.BKL);
	}

	return count;
}

static unsigned backlightvalue = 0;
static ssize_t camera_bl_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret =0;
	ret = scnprintf(buf, PAGE_SIZE, "%s%u\n", "BL_CAM_MIN=", backlightvalue);
	return ret;
}

static ssize_t attrs_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			ret = scnprintf(buf, PAGE_SIZE, "%d\n", htc_attr_status[i].cur_value);
			break;
		}
	}

	return ret;
}

static ssize_t attr_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	unsigned long res;
	int rc, i;

	rc = kstrtoul(buf, 10, &res);
	if (rc) {
		pr_err("invalid parameter, %s %d\n", buf, rc);
		count = -EINVAL;
		goto err_out;
	}

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		if (!strcmp(attr->attr.name, htc_attr_status[i].title)) {
			htc_attr_status[i].req_value = res;
			break;
		}
	}

err_out:
	return count;
}

static ssize_t switch_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	int ret;
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);
	ret = attr_store(dev, attr, buf, count);
	htc_set_vddio_switch(mfd);
	return ret;
}

static ssize_t profile_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0;

	switch (htc_attr_status[COLOR_PROFILE_INDEX].cur_value) {
	case PANEL_COLOR_PROFILE_NATIVE:
		ret = scnprintf(buf, PAGE_SIZE, "native\n");
		break;
	case PANEL_COLOR_PROFILE_SRGB:
		ret = scnprintf(buf, PAGE_SIZE, "srgb\n");
		break;
	default:
		ret = scnprintf(buf, PAGE_SIZE, "unknown\n");
		break;
	}

	return ret;
}

static ssize_t profile_store(struct device *dev, struct device_attribute *attr,
        const char *buf, size_t count)
{
	if (!strcmp(buf, "native")) {
		htc_attr_status[COLOR_PROFILE_INDEX].req_value = PANEL_COLOR_PROFILE_NATIVE;
	} else if (!strcmp(buf, "srgb")) {
		htc_attr_status[COLOR_PROFILE_INDEX].req_value = PANEL_COLOR_PROFILE_SRGB;
	} else {
		pr_err("%s: unsupport color profile name [%s]\n", __func__, buf);
	}

	return count;
}

static DEVICE_ATTR(backlight_info, S_IRUGO, camera_bl_show, NULL);
static DEVICE_ATTR(cabc_level_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_temp_ctl, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(color_profile_ctl, S_IRUGO | S_IWUSR, profile_show, profile_store);
static DEVICE_ATTR(vddio_switch, S_IRUGO | S_IWUSR, attrs_show, switch_store);
static DEVICE_ATTR(bklt_cali, S_IRUGO | S_IWUSR, bklt_gain_show, bklt_gain_store);
static DEVICE_ATTR(bklt_cali_enable, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_ATTR(disp_cali, S_IRUGO | S_IWUSR, rgb_gain_show, rgb_gain_store);
static DEVICE_ATTR(disp_cali_enable, S_IRUGO | S_IWUSR, attrs_show, attr_store);
static DEVICE_BOOL_ATTR(sre_enable, S_IRUGO | S_IWUSR, sre_enable);

static struct attribute *htc_extend_attrs[] = {
	&dev_attr_backlight_info.attr,
	&dev_attr_cabc_level_ctl.attr,
	&dev_attr_color_temp_ctl.attr,
	&dev_attr_color_profile_ctl.attr,
	&dev_attr_vddio_switch.attr,
	&dev_attr_bklt_cali.attr,
	&dev_attr_bklt_cali_enable.attr,
	&dev_attr_disp_cali.attr,
	&dev_attr_disp_cali_enable.attr,
	&dev_attr_sre_enable.attr.attr,
	NULL,
};

static struct attribute_group htc_extend_attr_group = {
	.attrs = htc_extend_attrs,
};

void htc_register_attrs(struct kobject *led_kobj, struct msm_fb_data_type *mfd)
{
	int rc;
	struct mdss_panel_data *pdata = dev_get_platdata(&mfd->pdev->dev);
	struct calibration_gain *gain = &(pdata->panel_info.cali_gain);

	pr_err("htc_register_attrs\n");

	rc = sysfs_create_group(led_kobj, &htc_extend_attr_group);
	if (rc)
		pr_err("sysfs group creation failed, rc=%d\n", rc);

	mfd->compass_notifier_block.notifier_call = compass_notifier_fn;
	compass_en_register_notifier(&mfd->compass_notifier_block);

	/* rgb calibration initial value*/
	if (RGB_GAIN_CHECK(gain->R) && RGB_GAIN_CHECK(gain->G) && RGB_GAIN_CHECK(gain->B)) {
		aux_gain.R = gain->R;
		aux_gain.G = gain->G;
		aux_gain.B = gain->B;
	}

	/* backlight calibration initial value*/
	if (BRI_GAIN_CHECK(gain->BKL))
		aux_gain.BKL = gain->BKL;

	return;
}

void htc_reset_status(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(htc_attr_status); i++) {
		htc_attr_status[i].cur_value = htc_attr_status[i].def_value;
	}

	return;
}

static ssize_t htc_set_bl_sync(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);

	if (strtobool(buf, &mfd->bl_sync) < 0)
		return -EINVAL;

	return count;
}

static ssize_t htc_get_bl_sync(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct msm_fb_data_type *mfd = dev_get_drvdata(dev->parent);

	return scnprintf(buf, PAGE_SIZE, "%d\n", mfd->bl_sync);
}

static DEVICE_ATTR(bl_sync, S_IRUGO | S_IWUSR, htc_get_bl_sync, htc_set_bl_sync);

static struct attribute *htc_sub_attrs[] = {
	&dev_attr_bl_sync.attr,
	NULL
};

static struct attribute_group htc_sub_attr_group = {
	.attrs = htc_sub_attrs,
};

void htc_register_sub_attrs(struct kobject *led_kobj)
{
	int rc;
	rc = sysfs_create_group(led_kobj, &htc_sub_attr_group);
	if (rc)
		pr_err("%s: sysfs group creation failed, rc=%d\n", __func__, rc);

	return;
}

void htc_unregister_sub_attrs(struct kobject *led_kobj)
{
	sysfs_remove_group(led_kobj, &htc_sub_attr_group);
}

void htc_register_camera_bkl(int level)
{
	backlightvalue = level;
}

void htc_set_cabc(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (htc_attr_status[CABC_INDEX].req_value > 2)
		return;

	if (!ctrl_pdata->cabc_off_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_ui_cmds.cmds)
		return;

	if (!ctrl_pdata->cabc_video_cmds.cmds)
		return;

	if (!force && (htc_attr_status[CABC_INDEX].req_value == htc_attr_status[CABC_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[CABC_INDEX].req_value == 0) {
		cmdreq.cmds = ctrl_pdata->cabc_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_off_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 1) {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	} else if (htc_attr_status[CABC_INDEX].req_value == 2) {
		cmdreq.cmds = ctrl_pdata->cabc_video_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_video_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->cabc_ui_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->cabc_ui_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[CABC_INDEX].cur_value = htc_attr_status[CABC_INDEX].req_value;
	pr_info("%s: cabc mode=%d\n", __func__, htc_attr_status[CABC_INDEX].cur_value);
	return;
}

void htc_set_color_temp(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;
	int req_mode = 0;
	int i = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_temp_cnt)
		return;

	for (i = 0; i < ctrl_pdata->color_temp_cnt ; i++) {
		if (!ctrl_pdata->color_temp_cmds[i].cmds)
			return;
	}

	if (htc_attr_status[COLOR_TEMP_INDEX].req_value >= ctrl_pdata->color_temp_cnt)
		return;

	if (!force && (htc_attr_status[COLOR_TEMP_INDEX].req_value == htc_attr_status[COLOR_TEMP_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	req_mode = htc_attr_status[COLOR_TEMP_INDEX].req_value;
	cmdreq.cmds = ctrl_pdata->color_temp_cmds[req_mode].cmds;
	cmdreq.cmds_cnt = ctrl_pdata->color_temp_cmds[req_mode].cmd_cnt;

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

//	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_TEMP_INDEX].cur_value = htc_attr_status[COLOR_TEMP_INDEX].req_value;
	pr_info("%s: color temp mode=%d\n", __func__, htc_attr_status[COLOR_TEMP_INDEX].cur_value);
	return;
}

void htc_set_color_profile(struct msm_fb_data_type *mfd, bool force)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->color_default_cmds.cmd_cnt || !ctrl_pdata->color_srgb_cmds.cmd_cnt)
		return;

	if (!force && (htc_attr_status[COLOR_PROFILE_INDEX].req_value == htc_attr_status[COLOR_PROFILE_INDEX].cur_value))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (htc_attr_status[COLOR_PROFILE_INDEX].req_value == PANEL_COLOR_PROFILE_SRGB) {
		cmdreq.cmds = ctrl_pdata->color_srgb_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_srgb_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->color_default_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->color_default_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	htc_attr_status[COLOR_PROFILE_INDEX].cur_value = htc_attr_status[COLOR_PROFILE_INDEX].req_value;
	pr_info("%s: color profile mode=%d\n", __func__, htc_attr_status[COLOR_PROFILE_INDEX].cur_value);
	return;
}

void compass_set_vddio_switch(struct msm_fb_data_type *mfd, int enable)
{
	struct mdss_mdp_ctl *ctl = mfd_to_ctl(mfd);
	int event = enable ? MDSS_EVENT_PANEL_VDDIO_SWITCH_ON : MDSS_EVENT_PANEL_VDDIO_SWITCH_OFF;

	mdss_mdp_ctl_intf_event(ctl, event, NULL, CTL_INTF_EVENT_FLAG_DEFAULT);
}

void htc_set_vddio_switch(struct msm_fb_data_type *mfd)
{
	if (htc_attr_status[VDDIO_INDEX].req_value == htc_attr_status[VDDIO_INDEX].cur_value)
		return;

	compass_set_vddio_switch(mfd, htc_attr_status[VDDIO_INDEX].req_value);

	htc_attr_status[VDDIO_INDEX].cur_value = htc_attr_status[VDDIO_INDEX].req_value;
	pr_info("%s: vddio switch=%d\n", __func__, htc_attr_status[VDDIO_INDEX].cur_value);
	return;
}

int compass_notifier_fn(struct notifier_block *nb,
                        unsigned long action, void *data)
{
	struct msm_fb_data_type *mfd;
	mfd = container_of(nb, struct msm_fb_data_type, compass_notifier_block);
	pr_info("%s: action=%d\n", __func__, (int)action);

	compass_set_vddio_switch(mfd, (action) ? 1 : 0);

	return 0;
}

void htc_set_burst(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct dcs_cmd_req cmdreq;

	static bool current_sre_mode = false;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (!ctrl_pdata->burst_off_cmds.cmds || !ctrl_pdata->burst_on_cmds.cmds)
		return;

	if (current_sre_mode == sre_enable)
		return;

	current_sre_mode = sre_enable;

	if(!mdss_fb_is_power_on(mfd))
		return;

	memset(&cmdreq, 0, sizeof(cmdreq));

	if (current_sre_mode) {
		cmdreq.cmds = ctrl_pdata->burst_on_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->burst_on_cmds.cmd_cnt;
	} else {
		cmdreq.cmds = ctrl_pdata->burst_off_cmds.cmds;
		cmdreq.cmds_cnt = ctrl_pdata->burst_off_cmds.cmd_cnt;
	}

	cmdreq.flags = CMD_REQ_COMMIT;
	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl_pdata, &cmdreq);

	pr_info("%s burst mode=%d\n", __func__, current_sre_mode);
	return;
}

bool htc_is_burst_bl_on(struct msm_fb_data_type *mfd, int value)
{
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct htc_backlight1_table *brt_bl_table = &panel_info->brt_bl_table[0];
	int size = brt_bl_table->size;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	/* Support burst mode backlight  */
	if (mfd->panel_info->burst_bl_value == 0)
		return false;

	if(!size || size < 2 || !brt_bl_table->brt_data)
		return false;

	pr_debug("%s: sre_enable=%d, value=%d, max brt=%d\n", __func__,
		sre_enable, value, brt_bl_table->brt_data[size - 1]);

	if (sre_enable && (value >= brt_bl_table->brt_data[size - 1]))
		return true;

	return false;
}

static void mdss_dsi_panel_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds, u32 flags)
{
	struct dcs_cmd_req cmdreq;
	struct mdss_panel_info *pinfo;

	pinfo = &(ctrl->panel_data.panel_info);
	if (pinfo->dcs_cmd_by_left) {
		if (ctrl->ndx != DSI_CTRL_LEFT)
			return;
	}

	memset(&cmdreq, 0, sizeof(cmdreq));
	cmdreq.cmds = pcmds->cmds;
	cmdreq.cmds_cnt = pcmds->cmd_cnt;
	cmdreq.flags = flags;

	/*Panel ON/Off commands should be sent in DSI Low Power Mode*/
	if (pcmds->link_state == DSI_LP_MODE)
		cmdreq.flags  |= CMD_REQ_LP_MODE;
	else if (pcmds->link_state == DSI_HS_MODE)
		cmdreq.flags |= CMD_REQ_HS_MODE;

	cmdreq.rlen = 0;
	cmdreq.cb = NULL;

	mdss_dsi_cmdlist_put(ctrl, &cmdreq);
}



static ssize_t htc_fb_set_aod_ctrl(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	unsigned long res;
	int rc = 0;
	struct mdss_panel_data *pdata;
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	rc = kstrtoul(buf, 10, &res);
	if (rc) {
		pr_err("%s: invalid parameter, %s, rc=%d\n", __func__, buf, rc);
		count = -EINVAL;
		goto err_out;
	}
	if (res >= FB_AOD_MAX) {
		pr_err("%s: invalid parameter for req_state=%lu\n", __func__, res);
		count = -EINVAL;
		goto err_out;
	}

	mutex_lock(&mfd->aod_lock);
	if (res == panel_info->aod.req_state)
		goto unlock;

	if (!mdss_fb_is_power_on(mfd)) {
		pr_info("%s: Request AOD state from %d to %lu during screen off\n", __func__, panel_info->aod.req_state, res);
	} else if (panel_info->aod.power_state == FB_AOD_FULL_ON && res == FB_AOD_PARTIAL_ON) {
		panel_info->aod.next_state = FB_AOD_PARTIAL_ON;
		pr_info("%s: Change to AOD Partial Mode\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->aod_cmds[0], CMD_REQ_COMMIT);
	} else if (panel_info->aod.power_state == FB_AOD_PARTIAL_ON && res == FB_AOD_FULL_ON) {
		panel_info->aod.next_state = FB_AOD_FULL_ON;
		pr_info("%s: Change to AOD Full Mode\n", __func__);
		mdss_dsi_panel_cmds_send(ctrl_pdata, &ctrl_pdata->aod_cmds[1], CMD_REQ_COMMIT);
	} else {
		// TBD
		pr_info("%s: Request AOD state from %d to %lu\n", __func__, panel_info->aod.req_state, res);
	}

	panel_info->aod.req_state = res;
	aod_send_notify(fbi);

unlock:
	mutex_unlock(&mfd->aod_lock);

err_out:
	return count;
}

static ssize_t htc_fb_get_aod_ctrl(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	ret = scnprintf(buf, PAGE_SIZE, "req_state=%d, power_state=%d\n",
		panel_info->aod.req_state, panel_info->aod.power_state);

	return ret;
}

static ssize_t htc_fb_set_sw49407_debug(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	unsigned long res;
	int rc = 0;

	rc = kstrtoul(buf, 10, &res);
	if (rc)
		return -EINVAL;

	panel_info->aod.debug = res;

	return count;
}

static ssize_t htc_fb_get_sw49407_debug(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct fb_info *fbi = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)fbi->par;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	return scnprintf(buf, PAGE_SIZE, "%d\n", panel_info->aod.debug);
}

static DEVICE_ATTR(aod_ctrl, S_IRUGO | S_IWUSR, htc_fb_get_aod_ctrl, htc_fb_set_aod_ctrl);
static DEVICE_ATTR(sw49407_debug, S_IRUGO | S_IWUSR, htc_fb_get_sw49407_debug, htc_fb_set_sw49407_debug);

static struct attribute *mdss_fb_aod_attrs[] = {
	&dev_attr_aod_ctrl.attr,
	&dev_attr_sw49407_debug.attr,
	NULL
};

static struct attribute_group mdss_fb_aod_attr_group = {
	.attrs = mdss_fb_aod_attrs,
};

int htc_mdss_fb_create_sysfs(struct msm_fb_data_type *mfd)
{
	int rc = 0;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	if (panel_info->aod.supported) {
		rc = sysfs_create_group(&mfd->fbi->dev->kobj, &mdss_fb_aod_attr_group);
		if (rc)
			pr_err("sysfs file creation failed, rc=%d\n", rc);
	}

	return rc;
}

void htc_mdss_fb_remove_sysfs(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_info *panel_info = mfd->panel_info;

	if (panel_info->aod.supported)
		sysfs_remove_group(&mfd->fbi->dev->kobj, &mdss_fb_aod_attr_group);
}

int aod_send_notify(struct fb_info *info)
{
	struct msm_fb_data_type *mfd = (struct msm_fb_data_type *)info->par;
	struct aod_panel_info *aod = NULL;

	if (!mfd || !mfd->panel_info)
		return -EINVAL;

	aod = &mfd->panel_info->aod;

	if (aod->supported) {
		struct fb_event event;
		int value;

		if (aod->power_state != aod->next_state) {
			value = aod->next_state;
			event.info = info;
			event.data = &value;

			pr_info("[DISP] Send AOD_MODE notify %d\n", value);
			fb_notifier_call_chain(FB_EVENT_AOD_MODE, &event);
		}
		aod->power_state = aod->next_state;

		return 1;
	}

	return 0;
}

void htc_update_bl_cali_data(struct msm_fb_data_type *mfd)
{
	struct mdss_panel_data *pdata;
	struct calibration_gain *gain = NULL;
	struct mdss_panel_info *panel_info = mfd->panel_info;
	struct htc_backlight1_table *brt_bl_table = &panel_info->brt_bl_table[0];
	int size = brt_bl_table->size;
	int bl_lvl = 0;

	pdata = dev_get_platdata(&mfd->pdev->dev);

	gain = &(pdata->panel_info.cali_gain);

	if (pdata->panel_info.cali_data_format == PANEL_CALIB_NOT_SUPPORTED)
		return;

	if (htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value == htc_attr_status[BL_CALI_ENABLE_INDEX].req_value)
		return;

	htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value = htc_attr_status[BL_CALI_ENABLE_INDEX].req_value;

	/* update backlight calibration data from user change*/
	if ((aux_gain.BKL != gain->BKL)) {
		gain->BKL = aux_gain.BKL;
		pr_info("%s change bkl calibration value, bkl=%d\n", __func__, gain->BKL);
	}

	if (!BRI_GAIN_CHECK(gain->BKL)) {
		pr_info("%s bkl=%d out of range\n", __func__, gain->BKL);
		return;
	}

	brt_bl_table->apply_cali = htc_attr_status[BL_CALI_ENABLE_INDEX].cur_value;

	/* free the old calibrated data first, then restore raw bl data again */
	kfree(brt_bl_table->bl_data);
	brt_bl_table->bl_data = kzalloc(size * sizeof(u16), GFP_KERNEL);
	if (!brt_bl_table->bl_data) {
		pr_err("unable to allocate memory for bl_data\n");
		return;
	}
	memcpy(brt_bl_table->bl_data, brt_bl_table->bl_data_raw, size * sizeof(u16));

	/* Calibrate brightness here */
	if (brt_bl_table->apply_cali) {
		u16 *bl_data_raw;
		u16 tmp_cali_value = 0;

		/* Not define brt table */
		if(!size || size < 2 || !brt_bl_table->brt_data || !brt_bl_table->bl_data)
			return;

		if (pdata->panel_info.cali_data_format == PANEL_CALIB_REV_1) {
			bl_data_raw = brt_bl_table->bl_data_raw;

			/* only calibrates on Min and Max node */
			tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[0], gain->BKL);
			brt_bl_table->bl_data[0] = VALID_CALI_BKLT(tmp_cali_value, 0, bl_data_raw[1]);
			tmp_cali_value = BACKLIGHT_CALI(bl_data_raw[size - 1], gain->BKL);
			brt_bl_table->bl_data[size - 1] = VALID_CALI_BKLT(tmp_cali_value, bl_data_raw[size - 2], panel_info->bl_max);
		}
	}

	if (mfd->bl_level && mfd->last_bri1) {
		/* always calibrates based on last time brightness value rather than calibrated brightness */
		bl_lvl = mdss_backlight_trans(mfd->last_bri1, &mfd->panel_info->brt_bl_table[0], true);

		/* Update the brightness when bl_cali be set */
		if (bl_lvl) {
			mfd->allow_bl_update = false;
			mfd->unset_bl_level = bl_lvl;
		}
	}

	pr_info("%s bl_cali=%d, unset_bl_level=%d \n", __func__, gain->BKL,  mfd->unset_bl_level);
}
void htc_update_rgb_cali_data(struct msm_fb_data_type *mfd, struct mdp_pcc_cfg_data *config)
{
	struct mdss_panel_data *pdata;
	struct calibration_gain *gain = NULL;
	struct mdp_pcc_data_v1_7 *pcc_data = config->cfg_payload;


	pdata = dev_get_platdata(&mfd->pdev->dev);

	gain = &(pdata->panel_info.cali_gain);

	if (pdata->panel_info.cali_data_format == PANEL_CALIB_NOT_SUPPORTED)
		return;

	/*
	 * Allow update request value directly, since it could apply calibration result without switching back
	 * to original un-calibration situation leads to flicker
	 */
	htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value = htc_attr_status[RGB_CALI_ENABLE_INDEX].req_value;

	/* update rgb calibration data from user change*/
	if ((aux_gain.R != gain->R) || (aux_gain.G != gain->G) || (aux_gain.B != gain->B)) {
		gain->R = aux_gain.R;
		gain->G = aux_gain.G;
		gain->B = aux_gain.B;
		pr_info("%s change calibration value, RGB(0x%x, 0x%x, 0x%x) \n",
				__func__, gain->R, gain->G, gain->B);
	}

	if (!RGB_GAIN_CHECK(gain->R) || !RGB_GAIN_CHECK(gain->G) || !RGB_GAIN_CHECK(gain->B)) {
		pr_info("%s RGB(0x%x, 0x%x, 0x%x) out of range\n", __func__, gain->R, gain->G, gain->B);
		return;
	}

	/* Apply calibration data to config only if calibration enabled */
	if (htc_attr_status[RGB_CALI_ENABLE_INDEX].cur_value) {
		pcc_data->r.r = RGB_CALIBRATION(pcc_data->r.r, gain->R);
		pcc_data->g.g = RGB_CALIBRATION(pcc_data->g.g, gain->G);
		pcc_data->b.b = RGB_CALIBRATION(pcc_data->b.b, gain->B);

		pr_info("%s apply calibration value, RGB(0x%x, 0x%x, 0x%x) \n",
			__func__, gain->R, gain->G, gain->B);
	}
}
