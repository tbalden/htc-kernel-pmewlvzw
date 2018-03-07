#if defined(CONFIG_TOUCHSCREEN_HTC_ATTR)
#include <linux/gpio.h>

#include "siw_touch.h"
#include "siw_touch_hal.h"
#include "siw_touch_feature_htc.h"

#define siw_sysfs_err_invalid_param(_dev)	\
		t_dev_err(_dev, "Invalid param\n");

struct device *tsdev = NULL;

static ssize_t glove_setting_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
	int size = 0;

	if (mode_ctrl->support_glove) {
		size += siw_snprintf(buf, size, "%d\n", atomic_read(&ts->state.glove));
	} else {
		size += siw_snprintf(buf, size, "glove_setting : Not support\n");
	}

	return size;
}

extern ssize_t _store_glove_state(struct device *dev,
				const char *buf, size_t count);
static ssize_t glove_setting_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	return _store_glove_state(tsdev, buf, count);
}

#if defined(CONFIG_AK8789_HALLSENSOR)
static ssize_t cover_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
	int size = 0;

	if (mode_ctrl->support_cover) {
		size += siw_snprintf(buf, size, "%d\n", mode_ctrl->cover_mode);
	} else {
		size += siw_snprintf(buf, size, "cover : Not support\n");
	}
	return size;
}

static ssize_t cover_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;

	if (!mode_ctrl->support_cover)
		return count;

	if (sysfs_streq(buf, "0"))
		mode_ctrl->cover_mode = 0;
	else if (sysfs_streq(buf, "1"))
		mode_ctrl->cover_mode = 1;
	else
		return -EINVAL;

	if(siw_set_status(ts, mode_ctrl->cover_mode << 1)) {
		t_dev_err(tsdev, "set cover mode error\n");
		return -EINVAL;
	}
	t_dev_info(tsdev, "%s: cover_mode = %d.\n", __func__, mode_ctrl->cover_mode);

	return count;
}
#endif	/* #if defined(CONFIG_AK8789_HALLSENSOR) */

enum SR_REG_STATE{
	ALLOCATE_DEV_FAIL = -2,
	REGISTER_DEV_FAIL,
	SUCCESS,
};

static int register_sr_touch_device(void)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	struct touch_device_caps *caps = &ts->caps;

	ts->sr_input_dev = input_allocate_device();

	if (ts->sr_input_dev == NULL) {
		t_dev_err(tsdev, "[SR] Failed to allocate SR input device\n");
		return ALLOCATE_DEV_FAIL;
	}

	ts->sr_input_dev->name = "sr_touchscreen";
	set_bit(EV_SYN, ts->sr_input_dev->evbit);
	set_bit(EV_ABS, ts->sr_input_dev->evbit);
	set_bit(EV_KEY, ts->sr_input_dev->evbit);

	set_bit(KEY_BACK, ts->sr_input_dev->keybit);
	set_bit(KEY_HOME, ts->sr_input_dev->keybit);
	set_bit(KEY_MENU, ts->sr_input_dev->keybit);
	set_bit(KEY_SEARCH, ts->sr_input_dev->keybit);
	set_bit(BTN_TOUCH, ts->sr_input_dev->keybit);
	//set_bit(KEY_APP_SWITCH, ts->sr_input_dev->keybit);
	set_bit(KEY_APPSELECT, ts->sr_input_dev->keybit);
	set_bit(INPUT_PROP_DIRECT, ts->sr_input_dev->propbit);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TRACKING_ID,
			0, caps->max_id, 0, 0);
	t_dev_info(tsdev, "[SR] input_set_abs_params: mix_x %d, max_x %d,"
		" min_y %d, max_y %d", 0,
		 caps->max_x, 0, caps->max_y);

	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_X,
			0, caps->max_x, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_POSITION_Y,
			0, caps->max_y, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_TOUCH_MAJOR,
			0, 255, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_PRESSURE,
			0, 30, 0, 0);
	input_set_abs_params(ts->sr_input_dev, ABS_MT_WIDTH_MAJOR,
			0, 30, 0, 0);

	if (input_register_device(ts->sr_input_dev)) {
		input_free_device(ts->sr_input_dev);
		t_dev_err(tsdev, "[SR] Unable to register %s input device\n",
			ts->sr_input_dev->name);
		return REGISTER_DEV_FAIL;
	}
	return SUCCESS;
}

static ssize_t sr_en_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;

	if (buf[0]) {
		if (ts->sr_input_dev)
			printk(KERN_INFO "[TP]%s: SR device already exist!\n",
					__func__);
		else
			printk(KERN_INFO "[TP]%s: SR touch device enable result:%X\n",
					__func__, register_sr_touch_device());
	}
	return count;
}

static ssize_t sr_en_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;

	if (ts->sr_input_dev)
		return snprintf(buf, PAGE_SIZE, "%s \n", ts->sr_input_dev->name);
	else
		return snprintf(buf, PAGE_SIZE, "0\n");
}

static int enabled_flag = 1;
extern ssize_t _store_irq_state(struct device *dev,
				const char *buf, size_t count);
static ssize_t enabled_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	char buf_tmp[2] = {'1', '\n'};
	int tmp = -1;

	if (sscanf(buf, "%d", &tmp) <= 0) {
		siw_sysfs_err_invalid_param(tsdev);
		return count;
	}

	if (tmp >= 0 && tmp <= 3) {
		enabled_flag = tmp;

		if (enabled_flag == 2) {
			tmp = 0;
			t_dev_info(tsdev, "try to change driving mode: %s -> %s\n",
					siw_lcd_driving_mode_str(chip->lcd_mode),
					siw_lcd_driving_mode_str(tmp));
			siw_ops_notify(ts, LCD_EVENT_LCD_MODE, &tmp);
		} else if (enabled_flag == 3) {
			tmp = 3;
			t_dev_info(tsdev, "try to change driving mode: %s -> %s\n",
					siw_lcd_driving_mode_str(chip->lcd_mode),
					siw_lcd_driving_mode_str(tmp));
			siw_ops_notify(ts, LCD_EVENT_LCD_MODE, &tmp);

			siw_ops_reset(ts, HW_RESET_ASYNC);
		}
		buf_tmp[0] = (enabled_flag % 2) ? '1': '0';
		_store_irq_state(tsdev, buf_tmp, count);
	}

	return count;
}

static ssize_t enabled_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int size = 0;
	size += siw_snprintf(buf, size,
			"enabled : %s, %d\n",
			(enabled_flag % 2) ? "Enabled" : "Disabled",
			enabled_flag);
	return size;
}

static ssize_t diag_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;

	if (sscanf(buf, "%u", &ts->diag_flag) <= 0) {
		siw_sysfs_err_invalid_param(tsdev);
		return count;
	}

	if (ts->diag_flag > 2) {
		t_dev_info(tsdev, "diag : Unknown %u\n", ts->diag_flag);
		ts->diag_flag = DIAG_OFF;
	}
	return count;
}

extern ssize_t prd_show_delta(struct device *dev, char *buf);
extern ssize_t prd_show_rawdata_prd(struct device *dev, char *buf);

static ssize_t diag_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;

	switch (ts->diag_flag) {
	case DIAG_OFF:
		break;
	case DIAG_DELTA:
		return prd_show_delta(tsdev, buf);
	case DIAG_RAW:
		return prd_show_rawdata_prd(tsdev, buf);
	default:
		t_dev_info(tsdev, "diag : Unknown %d\n", ts->diag_flag);
		ts->diag_flag = DIAG_OFF;
		break;
	}

	return 0;
}

#if defined(CONFIG_TOUCHSCREEN_HTC_DEBUG)
static ssize_t debug_level_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;

	/* input a decimal number */
	if (sscanf(buf, "%ux", &ts->debug_mask) != 1) {
		siw_sysfs_err_invalid_param(tsdev);
	}

	return count;
}

static ssize_t debug_level_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	int size = 0;
	size += siw_snprintf(buf, size,
			"%08X\n", ts->debug_mask);
	return size;
}
#endif

static ssize_t vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
//	struct siw_hal_reg *reg = chip->reg;
	struct siw_hal_fw_info *fw = &chip->fw;
	ssize_t offset = 0;

	/*
	 * Update IC info
	 * Reference: _show_atcmd_version_info()
	 */
	offset = siw_touch_get(tsdev, CMD_ATCMD_VERSION, buf);
	if (strstr(buf, "-1") != 0)
		return offset;

	/*
	 * Reference: siw_hal_get_cmd_version() at siw_touch_hal.c
	 */
	offset = 0;
	offset += siw_snprintf(buf, offset, "%s-FW:", touch_chip_name(ts));

	if (fw->version_ext) {
		offset += siw_snprintf(buf, offset, "%08X(%u.%02u)\n",
					fw->version_ext,
					fw->v.version.major, fw->v.version.minor);
	} else {
		offset += siw_snprintf(buf, offset, "v%u.%02u\n",
					fw->v.version.major, fw->v.version.minor);
	}

	return offset;
}

static int attn = 0;
static int attn_stat = 0;
static ssize_t attn_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	if (sscanf(buf, "%d,%d", &attn, &attn_stat) <= 0) {
		siw_sysfs_err_invalid_param(tsdev);
		return count;
	}
	if (attn >= 0 && (attn_stat == 1 || attn_stat == 0))
		gpio_direction_output(attn, attn_stat);
	else
		t_dev_info(tsdev, "attn : Unknown parameters\n");

	return count;
}

static ssize_t attn_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	int size = 0;

	if (attn == 0)
		attn = touch_irq_pin(ts);

	size += siw_snprintf(buf, size,
			"gpio%d: %d\n", attn, gpio_get_value(attn));
	return size;
}

static ssize_t reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	int type = 0;

	if (sscanf(buf, "%d", &type) <= 0) {
		siw_sysfs_err_invalid_param(tsdev);
		return count;
	}
	switch (type) {
	case 1:
		siw_ops_reset(ts, HW_RESET_ASYNC);
		break;
	case 2:
		siw_touch_notifier_call_chain(LCD_EVENT_HW_RESET, NULL);
		break;
	default:
		t_dev_info(tsdev, "reset : Unknown parameter\n");
		break;
	}

	return count;
}

static ssize_t bus_switch_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;

	if (sscanf(buf, "%d", &value) <= 0) {
		siw_sysfs_err_invalid_param(tsdev);
		return count;
	}
	switch(value) {
	case 0:
		ts->mode_ctrl.tp_bus_sel_en = 0;
		break;
	case 1:
		ts->mode_ctrl.tp_bus_sel_en = 1;
		break;
	case 2:
		switch_sensor_hub(tsdev, 0);/* Switch bus to CPU */
		break;
	case 3:
		switch_sensor_hub(tsdev, 1);/* Switch bus to MCU */
		break;
	default:
		t_dev_info(tsdev, "bus_switch : Unknown parameters\n");
		break;
	}

	return count;
}

static ssize_t bus_switch_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	int switch_pin = touch_switch_pin(ts);
	int size = 0;

	if (ts->mode_ctrl.tp_bus_sel_en == 0)
		size += siw_snprintf(buf, size,
				"Switch disabled\n");
	else
		size += siw_snprintf(buf, size,
				"Switch bus to %s\n",
				gpio_get_value(switch_pin) ? "MCU" : "CPU");

	return size;
}

#if defined(__SIW_SUPPORT_WATCH)
extern void siw_hal_watch_display_off(struct device *dev);
extern int siw_hal_watch_update_font_effect_24h(
		struct device *dev, int hour);
extern int siw_hal_watch_is_disp_waton(struct device *dev);
extern void siw_hal_watch_set_disp_waton(struct device *dev, bool value);
#define SIW_WATCH_TAG 	"watch: "
#define t_watch_info(_dev, fmt, args...)	\
		__t_dev_info(_dev, SIW_WATCH_TAG fmt, ##args)
#define t_watch_err(_dev, fmt, args...)	\
		__t_dev_err(_dev, SIW_WATCH_TAG fmt, ##args)
static ssize_t enable_clock_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	int size = 0;

	size += siw_snprintf(buf, size,
			"%d %d %d\n",
			ts->enable_clock_setting[0],
			ts->enable_clock_setting[1],
			ts->enable_clock_setting[2]);
	return size;
}

static ssize_t enable_clock_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct siw_touch_chip *chip = to_touch_chip(tsdev);
	struct siw_ts *ts = chip->ts;
	unsigned int onoff = 0;
	unsigned int hour = 0;
	unsigned int language = 0;
	struct touch_feature_ctrl *mode_ctrl = &ts->mode_ctrl;
#ifdef CONFIG_NANOHUB_TP_SWITCH
	uint8_t tp_mode = 0x0;
#endif

	if (sscanf(buf, "%u\n%u\n%u", &onoff, &hour, &language) <= 0) {
		t_watch_err(tsdev, "Invalid param\n");
		return count;
	}

	if (((onoff > 3) && (onoff != 0xFF))
			|| (hour != 24 && hour != 12)
			|| (language > 255)) {

		t_watch_info(tsdev, "enable_clock : Unknown parameter\n");
		return count;
	}

	mutex_lock(&ts->fb_lock);

	if (onoff != 0xFF) {
		mutex_lock(&ts->lock);
		ts->enable_clock_setting[0] = onoff & BIT(0);
		siw_hal_watch_set_disp_waton(tsdev, ts->enable_clock_setting[0]);
		mutex_unlock(&ts->lock);

		//turn off the clock
		if ((onoff & BIT(0)) == 0) {
#ifdef CONFIG_NANOHUB_TP_SWITCH
			if (mode_ctrl->bus_to_mcu & NANOHUB_TP_SWITCH_MCU_NORMAL) {
				tp_mode = (chip->lcd_mode | (chip->driving_mode << 3)) | (0x3 << 6);
				t_dev_info(tsdev, "[SensorHub] " SIW_WATCH_TAG
						"Cancel clock. mode: (0x%X, 0x%X)\n",
						chip->lcd_mode, chip->driving_mode);
				nanohub_tp_mode(tp_mode);
			} else
				siw_hal_watch_display_off(tsdev);
#else
			siw_hal_watch_display_off(tsdev);
#endif
		}
	} else
		ts->enable_clock_setting[0] = onoff;

	if ((onoff & BIT(0)) == 1) {
		//update font effect parameter if necessary
		if (siw_hal_watch_update_font_effect_24h(tsdev, hour)) {
			ts->enable_clock_setting[1] = hour;
			t_watch_info(tsdev, "enable_clock : 24h/midnight_hour_zero %s\n",
					(ts->enable_clock_setting[1] == 24) ?
					"on" : "off");
		}

		//check if update font data is needed
		if (ts->enable_clock_setting[2] != language) {
			ts->enable_clock_setting[2] = language;
			t_watch_info(tsdev, "enable_clock : lang index %u\n",
					ts->enable_clock_setting[2]);

#ifdef CONFIG_NANOHUB_TP_SWITCH
			if (mode_ctrl->bus_to_mcu == NANOHUB_TP_SWITCH_AP) {
				siw_hal_watch_font_update(tsdev);
			} else {
				//Should update font data once the bus is connectetd to CPU
				t_dev_info(tsdev, "[SensorHub] " SIW_WATCH_TAG
					"bus connect to MCU, update font later.\n");
			}
#else
			siw_hal_watch_font_update(tsdev);
#endif
		}
	}

	if (onoff != 0xFF) {
		//turn on the clock
		if ((onoff & BIT(0)) == 1) {
#ifdef CONFIG_NANOHUB_TP_SWITCH
			if (mode_ctrl->bus_to_mcu == NANOHUB_TP_SWITCH_AP)
				siw_hal_watch_font_onoff(tsdev, ts->enable_clock_setting[0]);
#else
			siw_hal_watch_font_onoff(tsdev, ts->enable_clock_setting[0]);
#endif
		}
	}

	t_watch_info(tsdev, "enable_clock : %d %d %d\n",
			ts->enable_clock_setting[0],
			ts->enable_clock_setting[1],
			ts->enable_clock_setting[2]);

	mutex_unlock(&ts->fb_lock);
	return count;
}
#endif

static DEVICE_ATTR_RW(glove_setting);
#if defined(CONFIG_AK8789_HALLSENSOR)
static DEVICE_ATTR_RW(cover);
#endif
static DEVICE_ATTR_RW(sr_en);
static DEVICE_ATTR_RW(enabled);
static DEVICE_ATTR_RW(diag);
//static DEVICE_ATTR_RW(config);	/* There is no config for SW49407 */
static DEVICE_ATTR_RW(debug_level);
static DEVICE_ATTR_RO(vendor);
static DEVICE_ATTR_RW(attn);
static DEVICE_ATTR_WO(reset);
static DEVICE_ATTR_RW(bus_switch);
#if defined(__SIW_SUPPORT_WATCH)
static DEVICE_ATTR_RW(enable_clock);
#endif

static struct attribute *siw_touch_attribute_list_htc[] = {
	&dev_attr_glove_setting.attr,
#if defined(CONFIG_AK8789_HALLSENSOR)
	&dev_attr_cover.attr,
#endif
	&dev_attr_sr_en.attr,
	&dev_attr_enabled.attr,
	&dev_attr_diag.attr,
//	&dev_attr_config.attr,		/* There is no config for SW49407 */
	&dev_attr_debug_level.attr,
	&dev_attr_vendor.attr,
	&dev_attr_attn.attr,
	&dev_attr_reset.attr,
	&dev_attr_bus_switch.attr,
#if defined(__SIW_SUPPORT_WATCH)
	&dev_attr_enable_clock.attr,
#endif
	NULL,
};

static const struct attribute_group siw_touch_attribute_group_htc = {
	.attrs = siw_touch_attribute_list_htc,
};

static struct kobject *android_touch_kobj = NULL;
int siw_touch_init_sysfs_htc(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);
	int ret = 0;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL);
	if (android_touch_kobj == NULL) {
		t_dev_err(ts->dev, "failed to create HTC sysfs entry\n");
		return -ENOMEM;
	}

	tsdev = dev;
	ts->enable_clock_setting[1] = 24;

	if (ts->kobj.state_initialized) {
		ret = sysfs_create_link(android_touch_kobj, &ts->kobj, ts->kobj.name);
		if (ret < 0) {
			t_dev_err(tsdev, "failed to link [%s] sysfs\n", ts->kobj.name);
			goto out;
		}
	}

	if (chip->kobj.state_initialized) {
		ret = sysfs_create_link(android_touch_kobj, &chip->kobj, chip->kobj.name);
		if (ret < 0) {
			t_dev_err(tsdev, "failed to link [%s] sysfs\n", chip->kobj.name);
			goto out;
		}
	}

	ret = sysfs_create_group(android_touch_kobj, &siw_touch_attribute_group_htc);
	if (ret < 0) {
		t_dev_err(tsdev, "failed to create HTC sysfs\n");
		goto out_link_sysfs;
	}

	return 0;

out_link_sysfs:
	if (chip->kobj.state_initialized)
		sysfs_remove_link(android_touch_kobj, chip->kobj.name);

	if (ts->kobj.state_initialized)
		sysfs_remove_link(android_touch_kobj, ts->kobj.name);
out:
	kobject_del(android_touch_kobj);
	return ret;
}

void siw_touch_free_sysfs_htc(struct siw_ts *ts)
{
	struct device *dev = ts->dev;
	struct siw_touch_chip *chip = to_touch_chip(dev);

	if (chip->kobj.state_initialized)
		sysfs_remove_link(android_touch_kobj, chip->kobj.name);

	if (ts->kobj.state_initialized)
		sysfs_remove_link(android_touch_kobj, ts->kobj.name);

	sysfs_remove_group(android_touch_kobj, &siw_touch_attribute_group_htc);

	kobject_del(android_touch_kobj);
}
#else
int siw_touch_init_sysfs_htc(struct siw_ts *ts){ return 0; };
void siw_touch_free_sysfs_htc(struct siw_ts *ts){ };
#endif	/* CONFIG_TOUCHSCREEN_HTC_ATTR */
