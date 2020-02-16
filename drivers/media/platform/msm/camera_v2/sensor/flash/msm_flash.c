/* Copyright (c) 2009-2017, The Linux Foundation. All rights reserved.
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

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include <linux/of_gpio.h>
#include "msm_flash.h"
#include "msm_camera_dt_util.h"
#include "msm_cci.h"
//HTC_START, porting flashlight control
#include <linux/htc_flashlight.h>
//HTC_END

#ifndef CONFIG_LEDS_QPNP_BUTTON_BLINK
#define CONFIG_LEDS_QPNP_BUTTON_BLINK
#endif

#ifdef CONFIG_LEDS_QPNP_BUTTON_BLINK
#include <linux/alarmtimer.h>
#include <linux/notification/notification.h>
#include <linux/uci/uci.h>
#endif

#undef CDBG
#define CDBG(fmt, args...) pr_info("[CAM][FL]"fmt, ##args)

//HTC_START
#define CONFIG_HTC_FLASHLIGHT_COMMON
#define HTC_CAM_FEATURE_FLASH_RESTRICTION
/*#define CONFIG_MSMB_CAMERA_DEBUG*/
//HTC_END

DEFINE_MSM_MUTEX(msm_flash_mutex);

#if 1
static int init_done = 0;
static struct alarm flash_blink_rtc;
static struct alarm flash_blink_do_blink_rtc;
static struct alarm vib_rtc;
static struct work_struct flash_blink_work;
static struct work_struct flash_start_blink_work;
static struct work_struct flash_stop_blink_work;
static struct workqueue_struct *flash_blink_workqueue;
static struct workqueue_struct *flash_start_blink_workqueue;
static struct workqueue_struct *flash_stop_blink_workqueue;
#endif

static struct v4l2_file_operations msm_flash_v4l2_subdev_fops;
static struct led_trigger *torch_trigger;

//HTC_START
#ifdef HTC_CAM_FEATURE_FLASH_RESTRICTION
static struct kobject *led_status_obj; // tmp remove for fc-1
#endif //HTC_CAM_FEATURE_FLASH_RESTRICTION
//HTC_END

static const struct of_device_id msm_flash_dt_match[] = {
	{.compatible = "qcom,camera-flash", .data = NULL},
	{}
};

static struct msm_flash_table msm_i2c_flash_table;
static struct msm_flash_table msm_gpio_flash_table;
static struct msm_flash_table msm_pmic_flash_table;

static struct msm_flash_table *flash_table[] = {
	&msm_i2c_flash_table,
	&msm_gpio_flash_table,
	&msm_pmic_flash_table
};

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
	.i2c_poll =  msm_camera_cci_i2c_poll,
};

void msm_torch_brightness_set(struct led_classdev *led_cdev,
				enum led_brightness value)
{
	if (!torch_trigger) {
		pr_err("No torch trigger found, can't set brightness\n");
		return;
	}

	led_trigger_event(torch_trigger, value);
};

static struct led_classdev msm_torch_led[MAX_LED_TRIGGERS] = {
	{
		.name		= "torch-light0",
		.brightness_set	= msm_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light1",
		.brightness_set	= msm_torch_brightness_set,
		.brightness	= LED_OFF,
	},
	{
		.name		= "torch-light2",
		.brightness_set	= msm_torch_brightness_set,
		.brightness	= LED_OFF,
	},
};

#if 1

static int currently_torch_mode = 0;
static int currently_blinking = 0;

#define DEFAULT_BLINK_NUMBER 46
#define DEFAULT_BLINK_WAIT_SEC 4
#define DEFAULT_WAIT_INC 1
#define DEFAULT_WAIT_INC_MAX 8

// default switches
static int flash_blink_on  = 1;
static int flash_blink_bright  = 1; // apply bright flash on each X number
static int flash_blink_bright_number  = 5; // X number when bright flash should be done
static int flash_blink_number = DEFAULT_BLINK_NUMBER;
static int flash_blink_wait_sec = DEFAULT_BLINK_WAIT_SEC;
static int flash_blink_wait_inc = DEFAULT_WAIT_INC;
static int flash_blink_wait_inc_max = DEFAULT_WAIT_INC_MAX;
static int haptic_mode = 1; // 0 - always blink, 1 - only blink with haptic vibration notifications
static int flash_only_face_down = 1;

static int uci_get_flash_haptic_mode(void) {
	return uci_get_user_property_int_mm("flash_haptic_mode", haptic_mode, 0, 1);
}
static int uci_get_flash_blink_bright(void) {
	return uci_get_user_property_int_mm("flash_blink_bright", flash_blink_bright, 0, 1);
}
static int uci_get_flash_blink_bright_number(void) {
	return uci_get_user_property_int_mm("flash_blink_bright_number", flash_blink_bright_number, 1, 10);
}
static int uci_get_flash_blink_number(void) {
	return uci_get_user_property_int_mm("flash_blink_number", flash_blink_number, 1, 50);
}
static int uci_get_flash_blink_wait_sec(void) {
	return uci_get_user_property_int_mm("flash_blink_wait_sec", flash_blink_wait_sec, 1, 50);
}
static int uci_get_flash_blink_wait_inc(void) {
	return uci_get_user_property_int_mm("flash_blink_wait_inc", flash_blink_wait_inc, 0, 1);
}
static int uci_get_flash_blink_wait_inc_max(void) {
	return uci_get_user_property_int_mm("flash_blink_wait_inc_max", flash_blink_wait_inc_max, 1, 8);
}

// dim mode switches
static int dim_mode = 1; // 0 - no , 1 - darker dim flash, 2 - fully off dim
static int dim_use_period = 1; // 0 - don't use dimming period hours, 1 - use hours, dim only between them
static int dim_start_hour = 22; // start hour
static int dim_end_hour = 6; // end hour

static int uci_get_flash_dim_mode(void) {
	return uci_get_user_property_int_mm("flash_dim_mode", dim_mode, 0, 1);
}
static int uci_get_flash_dim_use_period(void) {
	return uci_get_user_property_int_mm("flash_dim_use_period", dim_use_period, 0, 1);
}
static int uci_get_flash_dim_start_hour(void) {
	return uci_get_user_property_int_mm("flash_dim_start_hour", dim_start_hour, 0, 23);
}
static int uci_get_flash_dim_end_hour(void) {
	return uci_get_user_property_int_mm("flash_dim_end_hour", dim_end_hour, 0, 23);
}


void set_flash_blink_on(int value) {
	flash_blink_on = !!value;
}
EXPORT_SYMBOL(set_flash_blink_on);
int get_flash_blink_on(void) {
	return flash_blink_on;
}
EXPORT_SYMBOL(get_flash_blink_on);

void set_flash_only_face_down(int value) {
	flash_only_face_down = !!value;
}
EXPORT_SYMBOL(set_flash_only_face_down);
int get_flash_only_face_down(void) {
	return flash_only_face_down;
}
EXPORT_SYMBOL(get_flash_only_face_down);

void set_flash_blink_number(int value) {
	flash_blink_number = value%51; // max 50
}
EXPORT_SYMBOL(set_flash_blink_number);
int get_flash_blink_number(void) {
	return flash_blink_number;
}
EXPORT_SYMBOL(get_flash_blink_number);

void set_flash_blink_bright(int value) {
	flash_blink_bright = !!value;
}
EXPORT_SYMBOL(set_flash_blink_bright);
int get_flash_blink_bright(void) {
	return flash_blink_bright;
}
EXPORT_SYMBOL(get_flash_blink_bright);

void set_flash_blink_bright_number(int value) {
	flash_blink_bright_number = max(1,value%11); // min 1, max 10
}
EXPORT_SYMBOL(set_flash_blink_bright_number);
int get_flash_blink_bright_number(void) {
	return flash_blink_bright_number;
}
EXPORT_SYMBOL(get_flash_blink_bright_number);


void set_flash_blink_wait_sec(int value) {
	flash_blink_wait_sec = max(1,value%11); // min 1/max 10
}
EXPORT_SYMBOL(set_flash_blink_wait_sec);
int get_flash_blink_wait_sec(void) {
	return flash_blink_wait_sec;
}
EXPORT_SYMBOL(get_flash_blink_wait_sec);

void set_flash_blink_wait_inc(int value) {
	flash_blink_wait_inc = !!value;
}
EXPORT_SYMBOL(set_flash_blink_wait_inc);
int get_flash_blink_wait_inc(void) {
	return flash_blink_wait_inc;
}
EXPORT_SYMBOL(get_flash_blink_wait_inc);

void set_flash_blink_wait_inc_max(int value) {
	flash_blink_wait_inc_max = max(1,value%9); // min 1/max 8
}
EXPORT_SYMBOL(set_flash_blink_wait_inc_max);
int get_flash_blink_wait_inc_max(void) {
	return flash_blink_wait_inc_max;
}
EXPORT_SYMBOL(get_flash_blink_wait_inc_max);


void set_flash_haptic_mode(int value) {
	haptic_mode = !!value;
}
EXPORT_SYMBOL(set_flash_haptic_mode);
int get_flash_haptic_mode(void) {
	return haptic_mode;
}
EXPORT_SYMBOL(get_flash_haptic_mode);

void set_flash_dim_mode(int value) {
	dim_mode = value%3; // 0/1/2
}
EXPORT_SYMBOL(set_flash_dim_mode);
int get_flash_dim_mode(void) {
	return dim_mode;
}
EXPORT_SYMBOL(get_flash_dim_mode);

void set_flash_dim_use_period(int value) {
	dim_use_period = !!value;
}
EXPORT_SYMBOL(set_flash_dim_use_period);
int get_flash_dim_use_period(void) {
	return dim_use_period;
}
EXPORT_SYMBOL(get_flash_dim_use_period);

void set_flash_dim_period_hours(int startValue, int endValue) {
	dim_start_hour = startValue;
	dim_end_hour = endValue;
}
EXPORT_SYMBOL(set_flash_dim_period_hours);
void get_flash_dim_period_hours(int *r) {
	r[0] = dim_start_hour;
	r[1] = dim_end_hour;
}
EXPORT_SYMBOL(get_flash_dim_period_hours);


static int uci_get_flash_only_face_down(void) {
	return uci_get_user_property_int_mm("flash_only_face_down", flash_only_face_down, 0, 1);
}

static bool face_down = false;
static bool proximity = false;
static bool silent = false;
static bool ringing = false;
static bool in_call = false;

void flash_blink(bool haptic);
void flash_stop_blink(void);

// register sys uci listener
void flash_uci_sys_listener(void) {
	pr_info("%s uci sys parse happened...\n",__func__);
	{
		bool ringing_new = !!uci_get_sys_property_int_mm("ringing", 0, 0, 1);
		bool in_call_new = !!uci_get_sys_property_int_mm("in_call", 0, 0, 1);
		face_down = !!uci_get_sys_property_int_mm("face_down", 0, 0, 1);
		proximity = !!uci_get_sys_property_int_mm("proximity", 0, 0, 1);
		silent = !!uci_get_sys_property_int_mm("silent", 0, 0, 1);

		if (uci_get_user_property_int_mm("flash_blink_ringing", 0, 0, 1)) {
			if (ringing_new && !ringing) {
				flash_blink(true);
			}
		}
		if (!ringing_new && ringing) {
			ringing = false;
			flash_stop_blink();
		}
		ringing = ringing_new;

		if (in_call_new && !in_call) {
			flash_stop_blink();
		}
		in_call = in_call_new;

		pr_info("%s uci sys face_down %d\n",__func__,face_down);
		pr_info("%s uci sys proximity %d\n",__func__,proximity);
		pr_info("%s uci sys silent %d\n",__func__,silent);
		pr_info("%s uci sys ringing %d\n",__func__,ringing);
	}
}


static int smart_get_flash_blink_on(void) {
	int ret = 0;
	int level = smart_get_notification_level(NOTIF_FLASHLIGHT);
	if (uci_get_user_property_int_mm("flash_blink", flash_blink_on, 0, 1)) {
		if (level!=NOTIF_STOP) {
			ret = 1;
		}
	}
	pr_info("%s smart_notif =========== level: %d  uci_get_flash_blink_on() %d \n",__func__, level, ret);
	return ret;
}
static int smart_get_flash_dim_mode(void) {
	int ret = uci_get_flash_dim_mode();
	int level = smart_get_notification_level(NOTIF_FLASHLIGHT);
	if (!ret) { // not yet dimming...
		if (level==NOTIF_DIM) {
			ret = 1; // should DIM!
		}
	}
	pr_info("%s smart_notif =========== level: %d  flash_dim_mode %d \n",__func__, level, ret);
	return ret;
}
static int smart_get_flash_dim_use_period(void) {
	int ret = uci_get_flash_dim_use_period();
	int level = smart_get_notification_level(NOTIF_FLASHLIGHT);
	if (ret) { // using period for dimming...
		if (level==NOTIF_DIM) {
			ret = 0; // should dim anyway, override to Not use Dim period
		}
	}
	pr_info("%s smart_notif =========== level: %d  flash_dim_use_period %d \n",__func__, level, ret);
	return ret;
}
static int smart_get_flash_blink_wait_sec(void) {
	int ret = uci_get_flash_blink_wait_sec();
	int level = smart_get_notification_level(NOTIF_FLASHLIGHT);
	if (level!=NOTIF_DEFAULT) {
		ret = ret * 2;
	}
	pr_info("%s smart_notif =========== level: %d  flash_blink_wait_sec %d \n",__func__, level, ret);
	return ret;
}
static int smart_get_flash_blink_bright_number(void) {
	int ret = uci_get_flash_blink_bright_number();
	int level = smart_get_notification_level(NOTIF_FLASHLIGHT);
	if (level!=NOTIF_DEFAULT) {
		ret = ret * 2;
	}
	pr_info("%s smart_notif =========== level: %d  flash_blink_bright_number %d \n",__func__, level, ret);
	return ret;
}


static int current_blink_num = 0;
static int interrupt_retime = 0;
static DEFINE_MUTEX(flash_blink_lock);


int get_hour_of_day(void) {
	struct timespec ts;
	unsigned long local_time;
	getnstimeofday(&ts);
	local_time = (u32)(ts.tv_sec - (sys_tz.tz_minuteswest * 60));
	return (local_time / 3600) % (24);
}

int is_dim_blink_needed(void)
{
	int hour = get_hour_of_day();
	int in_dim_period = 0;
	int start_hour = uci_get_flash_dim_start_hour();
	int end_hour = uci_get_flash_dim_end_hour();

	if (!smart_get_flash_dim_mode()) return 0;

	pr_info("%s hour %d\n",__func__,hour);
	in_dim_period = (smart_get_flash_dim_use_period() && ( (start_hour>end_hour && ( (hour<24 && hour>=start_hour) || (hour>=0 && hour<end_hour) )) || (start_hour<end_hour && hour>=start_hour && hour<end_hour)));

	if (smart_get_flash_dim_mode() && (!smart_get_flash_dim_use_period() || (smart_get_flash_dim_use_period() && in_dim_period))) return smart_get_flash_dim_mode();
	return 0;
}

#define DEFAULT_VIB_SLOW 12
#define DEFAULT_VIB_LENGTH 250

// on off:
static int vib_notification_reminder = 0;
// how oftern vib
static int vib_notification_slowness = DEFAULT_VIB_SLOW;
// how long vibration motor should be on for one reminder buzz...
static int vib_notification_length = DEFAULT_VIB_LENGTH;

static int uci_get_vib_notification_slowness(void) {
	return uci_get_user_property_int_mm("vib_notification_slowness", vib_notification_slowness, 0, 30);
}
static int uci_get_vib_notification_length(void) {
	return uci_get_user_property_int_mm("vib_notification_length", vib_notification_length, 0, 499);
}

void set_vib_notification_reminder(int value) {
	vib_notification_reminder = !!value;
}
EXPORT_SYMBOL(set_vib_notification_reminder);
int get_vib_notification_reminder(void) {
	return vib_notification_reminder;
}
EXPORT_SYMBOL(get_vib_notification_reminder);
void set_vib_notification_slowness(int value) {
	vib_notification_slowness = value%31;
}
EXPORT_SYMBOL(set_vib_notification_slowness);
int get_vib_notification_slowness(void) {
	return vib_notification_slowness;
}
EXPORT_SYMBOL(get_vib_notification_slowness);
void set_vib_notification_length(int value) {
	vib_notification_length = value%500;
}
EXPORT_SYMBOL(set_vib_notification_length);
int get_vib_notification_length(void) {
	return vib_notification_length;
}
EXPORT_SYMBOL(get_vib_notification_length);

static int smart_get_vib_notification_reminder(void) {
	int ret = 0;
	if (silent) return 0; // do not vibrate in silent mode at all, regadless of configurations
	if (uci_get_user_property_int_mm("vib_notification_reminder", vib_notification_reminder, 0, 1)) {
		int level = smart_get_notification_level(NOTIF_VIB_REMINDER);
		if (level!=NOTIF_STOP) {
			pr_info("%s smart_notif =========== level: %d vib_notification_reminder %d \n",__func__, level, ret);
			ret = 1;
		}
	}
	return ret;
}

static int smart_get_vib_notification_slowness(void) {
	int ret = uci_get_vib_notification_slowness();
	int level = smart_get_notification_level(NOTIF_VIB_REMINDER);
	if (level!=NOTIF_DEFAULT) {
		pr_info("%s smart_notif =========== level: %d vib_notification_slowness %d \n",__func__, level, ret);
		ret = ret * 2;
	}
	return ret;
}

extern void boosted_vib(int time);

#define DIM_USEC 2
#define BRIGHT_USEC 1250

void precise_delay(int usec) {
	ktime_t start, end;
	s64 diff;
	start = ktime_get();
	while (1) {
		end = ktime_get();
		diff = ktime_to_us(ktime_sub(end, start));
		if (diff>=usec) return;
	}
}

extern void set_vibrate(int value);

// should be true if phone was not in flashlight ready state, like not on a table face down. Then next flashblink start should reschedule work.
static bool in_no_flash_long_alarm_wake_time = false;

void do_flash_blink(void) {
	ktime_t wakeup_time;
	ktime_t wakeup_time_vib;
	int count = 0;
	int limit = 3;
	int dim = 0;
	int bright = 0;
	int flash_next = 0;
	int vib_slowness = smart_get_vib_notification_slowness();

	pr_info("%s flash_blink\n",__func__);
	alarm_cancel(&flash_blink_do_blink_rtc); // stop pending alarm... no need to unidle cpu in that alarm...

	if (currently_torch_mode || interrupt_retime || in_call) return;

	dim = is_dim_blink_needed();
	pr_info("%s dim %d\n",__func__,dim);

	if (dim==2) {
		currently_blinking = 0;
		goto exit;
	}
	if (dim == 0 && uci_get_flash_blink_bright() && current_blink_num % smart_get_flash_blink_bright_number() == 0) {
		bright = 1;
	}

	if (uci_get_flash_blink_bright() && ringing) bright = 1;

	htc_flash_main(0,0);

	if (uci_get_flash_blink_wait_inc() && !dim) {
		// while in the first fast paced periodicity, don't do that much of flashing in one blink...
		if (current_blink_num < 16) limit = 3;
		if (current_blink_num > 30) limit = 3;
		if (current_blink_num > 40) limit = 4;
	}

	limit -= dim * 2;


	if ((uci_get_flash_only_face_down() && face_down) || !uci_get_flash_only_face_down()) {
		flash_next = 1; // should flash next time, alarm wait normal... if no flashing is being done, vibrating reminder wait period should be waited instead!
		while (count++<limit) {
			htc_torch_main_sync(0,150*(bright+1),true);  // [o] [ ]
			precise_delay(5 -(dim * DIM_USEC) +(bright * BRIGHT_USEC));
			htc_torch_main_sync(0,0,true);	// [ ] [ ]
			udelay(15000);


			if (!dim) {
				htc_torch_main_sync(150*(bright+1),0,true);  // [o] [ ]
				precise_delay(5 -(dim * DIM_USEC) +(bright * BRIGHT_USEC));
				htc_torch_main_sync(0,0,true);	// [ ] [ ]
				udelay(15000);
			}
		}
	} else {
		pr_info("%s skipping flashing because of not face down\n",__func__);
	}

	if (!ringing) {
	if (smart_get_vib_notification_reminder() && current_blink_num % vib_slowness == (vib_slowness - 1)) {
		{
			ktime_t curr_time = { .tv64 = 0 };
			wakeup_time_vib = ktime_add_us(curr_time,
				(1 * 1000LL * 1000LL)); // msec to usec 
		}
		// call vibration from a real time alarm thread, otherwise it can get stuck vibrating
		alarm_cancel(&vib_rtc); // stop pending alarm...
		alarm_start_relative(&vib_rtc, wakeup_time_vib); // start new...
	}
	}

	mutex_lock(&flash_blink_lock);
	pr_info("%s flash_blink lock\n",__func__);
	// make sure this part is running in a few cycles, as blocking the lock will block vibrator driver resulting in kernel freeze and panic

	if ( (uci_get_flash_blink_number() > 0 && current_blink_num > uci_get_flash_blink_number()) || interrupt_retime) {
		currently_blinking = 0;
		goto exit;
	}
	current_blink_num++;
	if (smart_get_flash_blink_on()) // only reschedule if still flashblink is on...
	{
		ktime_t curr_time = { .tv64 = 0 };
		int multiplicator = 1;
		int calc_with_blink_num = current_blink_num;
		if (!flash_next) {
			in_no_flash_long_alarm_wake_time = true;
			// won't need flashing next, skip a few blinks till next is a vibrating notifiaction, also count multiplicator, to multiply wait time...
			while (current_blink_num % vib_slowness != (vib_slowness - 1)) {
				current_blink_num++;
				multiplicator++;
			}
		} else {
			in_no_flash_long_alarm_wake_time = false;
		}
		wakeup_time = ktime_add_us(curr_time,
			( (smart_get_flash_blink_wait_sec() + min(max(((calc_with_blink_num-6)/4),0),uci_get_flash_blink_wait_inc_max()) * uci_get_flash_blink_wait_inc()) * 1000LL * 1000LL) * multiplicator); // msec to usec 
		pr_info("%s: Flash_next %d -- Current Time tv_sec: %ld, Alarm set to tv_sec: %ld\n",
			__func__, flash_next,
			ktime_to_timeval(curr_time).tv_sec,
			ktime_to_timeval(wakeup_time).tv_sec);

			alarm_cancel(&flash_blink_rtc); // stop pending alarm...
			alarm_start_relative(&flash_blink_rtc, wakeup_time); // start new...

	} else {
			alarm_cancel(&flash_blink_rtc); // stop pending alarm...
	}


exit:
	mutex_unlock(&flash_blink_lock);
	pr_info("%s flash_blink unlock\n",__func__);

}

static void flash_start_blink_work_func(struct work_struct *work)
{
	pr_info("%s flash_blink\n",__func__);
	mutex_lock(&flash_blink_lock);
	pr_info("%s flash_blink lock\n",__func__);

	interrupt_retime = 0;
	if (currently_blinking) {
		// already blinking, check if we should go back with blink num count to a faster pace...
		if (current_blink_num>8) {
			// if started blinking already over a lot of blinks, move back to the beginning, 
			// to shorter periodicity...
			current_blink_num = 5;
		} // otherwise if only a few blinks yet, don't reset count...
		if (in_no_flash_long_alarm_wake_time) { // if flashless long wait, start right now, so in case now it could flash, let it do right now
			// restart blinking with async work
			alarm_cancel(&flash_blink_rtc); // stop pending alarm...
			currently_blinking = 1;
			queue_work(flash_blink_workqueue, &flash_blink_work);
			in_no_flash_long_alarm_wake_time = false;
		}
		mutex_unlock(&flash_blink_lock);
		pr_info("%s flash_blink unlock\n",__func__);
	}
	if (!currently_blinking) {
		currently_blinking = 1;
		current_blink_num = 0;
		mutex_unlock(&flash_blink_lock);
		queue_work(flash_blink_workqueue, &flash_blink_work);
//		do_flash_blink();
		pr_info("%s flash_blink unlock\n",__func__);
	}
}

static void flash_stop_blink_work_func(struct work_struct *work)
{
	mutex_lock(&flash_blink_lock);

	if (!currently_blinking) goto exit;
	if (currently_torch_mode) goto exit;
	pr_info("%s flash_blink\n",__func__);
	currently_blinking = 0;
	htc_torch_main(0,0);
	interrupt_retime = 1;
	alarm_cancel(&flash_blink_rtc); // stop pending alarm...
exit:
	mutex_unlock(&flash_blink_lock);
}

void flash_blink(bool haptic) {
	pr_info("%s flash_blink\n",__func__);
	// is flash blink on?
	if (!smart_get_flash_blink_on()) return;
	// if not a haptic notificcation and haptic blink mode on, do not do blinking...
	if (!haptic && uci_get_flash_haptic_mode()) return;
	// if torch i on, don't blink
	if (currently_torch_mode) return;

	if (!init_done) return;

	queue_work(flash_start_blink_workqueue, &flash_start_blink_work);
}
EXPORT_SYMBOL(flash_blink);

static void flash_blink_work_func(struct work_struct *work)
{
	pr_info("%s flash_blink\n",__func__);
	do_flash_blink();
}


static enum alarmtimer_restart vib_rtc_callback(struct alarm *al, ktime_t now)
{
	pr_info("%s flash_blink\n",__func__);
	set_vibrate(uci_get_vib_notification_length());
	return ALARMTIMER_NORESTART;
}


static int smp_processor = 0;
static enum alarmtimer_restart flash_blink_rtc_callback(struct alarm *al, ktime_t now)
{
	pr_info("%s flash_blink\n",__func__);
	if (!interrupt_retime) {
		ktime_t wakeup_time_vib;
		ktime_t curr_time = { .tv64 = 0 };


		smp_processor = smp_processor_id();
		pr_info("%s flash_blink cpu %d\n",__func__, smp_processor);
		// queue work on current CPU for avoiding sleeping CPU...
		queue_work_on(smp_processor,flash_blink_workqueue, &flash_blink_work);

		wakeup_time_vib = ktime_add_us(curr_time,
			(2000LL * 1000LL)); // 2000 msec to usec 
		alarm_cancel(&flash_blink_do_blink_rtc); // stop pending alarm...
		alarm_start_relative(&flash_blink_do_blink_rtc, wakeup_time_vib); // start new...

	}
	return ALARMTIMER_NORESTART;
}

static enum alarmtimer_restart flash_blink_do_blink_rtc_callback(struct alarm *al, ktime_t now)
{
	pr_info("%s flash_blink cpu %d \n",__func__, smp_processor);
	if (!interrupt_retime) {
		// make sure Queue execution is not stuck... would mean longer pauses between blinks than should...
		wake_up_if_idle(smp_processor);
//		wake_up_all_idle_cpus();
	}
	return ALARMTIMER_NORESTART;
}


void flash_stop_blink(void) {
//	pr_info("%s flash_blink\n",__func__);
	if (!init_done) return;
	if (ringing) return; // screen on/user input shouldn't stop ringing triggered flashing!
	queue_work(flash_stop_blink_workqueue, &flash_stop_blink_work);
}
EXPORT_SYMBOL(flash_stop_blink);

#endif

static int32_t msm_torch_create_classdev(struct platform_device *pdev,
				void *data)
{
	int32_t rc = 0;
	int32_t i = 0;
	struct msm_flash_ctrl_t *fctrl =
		(struct msm_flash_ctrl_t *)data;

	if (!fctrl) {
		pr_err("Invalid fctrl\n");
		return -EINVAL;
	}

	for (i = 0; i < fctrl->torch_num_sources; i++) {
		if (fctrl->torch_trigger[i]) {
			torch_trigger = fctrl->torch_trigger[i];
			CDBG("%s:%d msm_torch_brightness_set for torch %d",
				__func__, __LINE__, i);
			msm_torch_brightness_set(&msm_torch_led[i],
				LED_OFF);

			rc = led_classdev_register(&pdev->dev,
				&msm_torch_led[i]);
			if (rc) {
				pr_err("Failed to register %d led dev. rc = %d\n",
						i, rc);
				return rc;
			}
		} else {
			pr_err("Invalid fctrl->torch_trigger[%d]\n", i);
			return -EINVAL;
		}
	}

	return 0;
};

static int32_t msm_flash_get_subdev_id(
	struct msm_flash_ctrl_t *flash_ctrl, void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (flash_ctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = flash_ctrl->pdev->id;
	else
		*subdev_id = flash_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static int32_t msm_flash_i2c_write_table(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_camera_i2c_reg_setting_array *settings)
{
	struct msm_camera_i2c_reg_setting conf_array;

	conf_array.addr_type = settings->addr_type;
	conf_array.data_type = settings->data_type;
	conf_array.delay = settings->delay;
	conf_array.reg_setting = settings->reg_setting_a;
	conf_array.size = settings->size;

	/* Validate the settings size */
	if ((!conf_array.size) || (conf_array.size > MAX_I2C_REG_SET)) {
		pr_err("failed: invalid size %d", conf_array.size);
		return -EINVAL;
	}

	return flash_ctrl->flash_i2c_client.i2c_func_tbl->i2c_write_table(
		&flash_ctrl->flash_i2c_client, &conf_array);
}

#ifdef CONFIG_COMPAT
static void msm_flash_copy_power_settings_compat(
	struct msm_sensor_power_setting *ps,
	struct msm_sensor_power_setting32 *ps32, uint32_t size)
{
	uint16_t i = 0;

	for (i = 0; i < size; i++) {
		ps[i].config_val = ps32[i].config_val;
		ps[i].delay = ps32[i].delay;
		ps[i].seq_type = ps32[i].seq_type;
		ps[i].seq_val = ps32[i].seq_val;
	}
}
#endif

static int32_t msm_flash_i2c_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t rc = 0;
	struct msm_flash_init_info_t *flash_init_info =
		flash_data->cfg.flash_init_info;
	struct msm_camera_i2c_reg_setting_array *settings = NULL;
	struct msm_camera_cci_client *cci_client = NULL;
#ifdef CONFIG_COMPAT
	struct msm_sensor_power_setting_array32 *power_setting_array32 = NULL;
#endif
	if (!flash_init_info || !flash_init_info->power_setting_array) {
		pr_err("%s:%d failed: Null pointer\n", __func__, __LINE__);
		return -EFAULT;
	}

#ifdef CONFIG_COMPAT
	if (is_compat_task()) {
		power_setting_array32 = kzalloc(
			sizeof(struct msm_sensor_power_setting_array32),
			GFP_KERNEL);
		if (!power_setting_array32) {
			pr_err("%s mem allocation failed %d\n",
				__func__, __LINE__);
			return -ENOMEM;
		}

		if (copy_from_user(power_setting_array32,
			(void *)flash_init_info->power_setting_array,
			sizeof(struct msm_sensor_power_setting_array32))) {
			pr_err("%s copy_from_user failed %d\n",
				__func__, __LINE__);
			kfree(power_setting_array32);
			return -EFAULT;
		}

		flash_ctrl->power_setting_array.size =
			power_setting_array32->size;
		flash_ctrl->power_setting_array.size_down =
			power_setting_array32->size_down;
		flash_ctrl->power_setting_array.power_down_setting =
			compat_ptr(power_setting_array32->power_down_setting);
		flash_ctrl->power_setting_array.power_setting =
			compat_ptr(power_setting_array32->power_setting);

		/* Validate power_up array size and power_down array size */
		if ((!flash_ctrl->power_setting_array.size) ||
			(flash_ctrl->power_setting_array.size >
			MAX_POWER_CONFIG) ||
			(!flash_ctrl->power_setting_array.size_down) ||
			(flash_ctrl->power_setting_array.size_down >
			MAX_POWER_CONFIG)) {

			pr_err("failed: invalid size %d, size_down %d",
				flash_ctrl->power_setting_array.size,
				flash_ctrl->power_setting_array.size_down);
			kfree(power_setting_array32);
			power_setting_array32 = NULL;
			return -EINVAL;
		}
		/* Copy the settings from compat struct to regular struct */
		msm_flash_copy_power_settings_compat(
			flash_ctrl->power_setting_array.power_setting_a,
			power_setting_array32->power_setting_a,
			flash_ctrl->power_setting_array.size);

		msm_flash_copy_power_settings_compat(
			flash_ctrl->power_setting_array.power_down_setting_a,
			power_setting_array32->power_down_setting_a,
			flash_ctrl->power_setting_array.size_down);
	} else
#endif
	if (copy_from_user(&flash_ctrl->power_setting_array,
		(void *)flash_init_info->power_setting_array,
		sizeof(struct msm_sensor_power_setting_array))) {
		pr_err("%s copy_from_user failed %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (flash_ctrl->flash_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		cci_client = flash_ctrl->flash_i2c_client.cci_client;
		cci_client->sid = flash_init_info->slave_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->i2c_freq_mode = flash_init_info->i2c_freq_mode;
	}

	flash_ctrl->power_info.power_setting =
		flash_ctrl->power_setting_array.power_setting_a;
	flash_ctrl->power_info.power_down_setting =
		flash_ctrl->power_setting_array.power_down_setting_a;
	flash_ctrl->power_info.power_setting_size =
		flash_ctrl->power_setting_array.size;
	flash_ctrl->power_info.power_down_setting_size =
		flash_ctrl->power_setting_array.size_down;

	if ((flash_ctrl->power_info.power_setting_size > MAX_POWER_CONFIG) ||
	(flash_ctrl->power_info.power_down_setting_size > MAX_POWER_CONFIG)) {
		pr_err("%s:%d invalid power setting size=%d size_down=%d\n",
			__func__, __LINE__,
			flash_ctrl->power_info.power_setting_size,
			flash_ctrl->power_info.power_down_setting_size);
		rc = -EINVAL;
		goto msm_flash_i2c_init_fail;
	}

	rc = msm_camera_power_up(&flash_ctrl->power_info,
		flash_ctrl->flash_device_type,
		&flash_ctrl->flash_i2c_client);
	if (rc < 0) {
		pr_err("%s msm_camera_power_up failed %d\n",
			__func__, __LINE__);
		goto msm_flash_i2c_init_fail;
	}

	if (flash_data->cfg.flash_init_info->settings) {
		settings = kzalloc(sizeof(
			struct msm_camera_i2c_reg_setting_array), GFP_KERNEL);
		if (!settings) {
			pr_err("%s mem allocation failed %d\n",
				__func__, __LINE__);
			return -ENOMEM;
		}

		if (copy_from_user(settings, (void *)flash_init_info->settings,
			sizeof(struct msm_camera_i2c_reg_setting_array))) {
			kfree(settings);
			pr_err("%s copy_from_user failed %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}

		rc = msm_flash_i2c_write_table(flash_ctrl, settings);
		kfree(settings);

		if (rc < 0) {
			pr_err("%s:%d msm_flash_i2c_write_table rc %d failed\n",
				__func__, __LINE__, rc);
		}
	}

	return 0;

msm_flash_i2c_init_fail:
	return rc;
}

static int32_t msm_flash_gpio_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t i = 0;
	int32_t rc = 0;

//	CDBG("Enter");
	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		flash_ctrl->flash_op_current[i] = LED_FULL;

	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		flash_ctrl->torch_op_current[i] = LED_HALF;

	for (i = 0; i < flash_ctrl->torch_num_sources; i++) {
		if (!flash_ctrl->torch_trigger[i]) {
			if (i < flash_ctrl->flash_num_sources)
				flash_ctrl->torch_trigger[i] =
					flash_ctrl->flash_trigger[i];
			else
				flash_ctrl->torch_trigger[i] =
					flash_ctrl->flash_trigger[
					flash_ctrl->flash_num_sources - 1];
		}
	}

	rc = flash_ctrl->func_tbl->camera_flash_off(flash_ctrl, flash_data);

//	CDBG("Exit");
	return rc;
}

static int32_t msm_flash_i2c_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = 0;

	if (!(&flash_ctrl->power_info) || !(&flash_ctrl->flash_i2c_client)) {
		pr_err("%s:%d failed: %pK %pK\n",
			__func__, __LINE__, &flash_ctrl->power_info,
			&flash_ctrl->flash_i2c_client);
		return -EINVAL;
	}

	rc = msm_camera_power_down(&flash_ctrl->power_info,
		flash_ctrl->flash_device_type,
		&flash_ctrl->flash_i2c_client);
	if (rc < 0) {
		pr_err("%s msm_camera_power_down failed %d\n",
			__func__, __LINE__);
		return -EINVAL;
	}
	return 0;
}

static int32_t msm_flash_off(struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	//HTC_START, porting flashlight control
	#ifndef CONFIG_HTC_FLASHLIGHT_COMMON
	//HTC_END
	int32_t i = 0;
	//HTC_START, porting flashlight control
	#endif
	//HTC_END

	CDBG("Enter\n");

	//HTC_START, porting flashlight control
	#ifdef CONFIG_HTC_FLASHLIGHT_COMMON
	if(htc_flash_main && htc_torch_main)
	{
		if (flash_ctrl->pdev->id == 0)
		{
		    htc_flash_main(0, 0);
		    htc_torch_main(0, 0);
		}
	}
	else
		pr_err("[CAM][FL] msm_flash_off, flashlight control is NULL\n");

	if(htc_flash_front && htc_torch_front)
	{
	    if (flash_ctrl->pdev->id == 1)
        {
		    htc_flash_front(0, 0);
		    htc_torch_front(0, 0);
        }
	}
	else
		pr_err("[CAM][FL]Front msm_flash_off, flashlight control is NULL\n");

	#else
	//HTC_END

	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		if (flash_ctrl->flash_trigger[i])
			led_trigger_event(flash_ctrl->flash_trigger[i], 0);

	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		if (flash_ctrl->torch_trigger[i])
			led_trigger_event(flash_ctrl->torch_trigger[i], 0);
	if (flash_ctrl->switch_trigger)
		led_trigger_event(flash_ctrl->switch_trigger, 0);

	//HTC_START, porting flashlight control
	#endif
	//HTC_END
	CDBG("Exit\n");
#if 1
	currently_blinking = 0;
	currently_torch_mode = 0;
#endif
	return 0;
}

static int32_t msm_flash_i2c_write_setting_array(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	int32_t rc = 0;
	struct msm_camera_i2c_reg_setting_array *settings = NULL;

	if (!flash_data->cfg.settings) {
		pr_err("%s:%d failed: Null pointer\n", __func__, __LINE__);
		return -EFAULT;
	}

	settings = kzalloc(sizeof(struct msm_camera_i2c_reg_setting_array),
		GFP_KERNEL);
	if (!settings) {
		pr_err("%s mem allocation failed %d\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (copy_from_user(settings, (void *)flash_data->cfg.settings,
		sizeof(struct msm_camera_i2c_reg_setting_array))) {
		kfree(settings);
		pr_err("%s copy_from_user failed %d\n", __func__, __LINE__);
		return -EFAULT;
	}

	rc = msm_flash_i2c_write_table(flash_ctrl, settings);
	kfree(settings);

	if (rc < 0) {
		pr_err("%s:%d msm_flash_i2c_write_table rc = %d failed\n",
			__func__, __LINE__, rc);
	}
	return rc;
}

static int32_t msm_flash_init(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	uint32_t i = 0;
	int32_t rc = -EFAULT;
	enum msm_flash_driver_type flash_driver_type = FLASH_DRIVER_DEFAULT;

	CDBG("Enter");

	if (flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT) {
		pr_err("%s:%d Invalid flash state = %d",
			__func__, __LINE__, flash_ctrl->flash_state);
		return 0;
	}

	if (flash_data->cfg.flash_init_info->flash_driver_type ==
		FLASH_DRIVER_DEFAULT) {
		flash_driver_type = flash_ctrl->flash_driver_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_data->flash_current[i] =
				flash_ctrl->flash_max_current[i];
			flash_data->flash_duration[i] =
				flash_ctrl->flash_max_duration[i];
		}
	} else if (flash_data->cfg.flash_init_info->flash_driver_type ==
		flash_ctrl->flash_driver_type) {
		flash_driver_type = flash_ctrl->flash_driver_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_ctrl->flash_max_current[i] =
				flash_data->flash_current[i];
			flash_ctrl->flash_max_duration[i] =
					flash_data->flash_duration[i];
		}
	}

	if (flash_driver_type == FLASH_DRIVER_DEFAULT) {
		pr_err("%s:%d invalid flash_driver_type", __func__, __LINE__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(flash_table); i++) {
		if (flash_driver_type == flash_table[i]->flash_driver_type) {
			flash_ctrl->func_tbl = &flash_table[i]->func_tbl;
			rc = 0;
		}
	}

	if (rc < 0) {
		pr_err("%s:%d failed invalid flash_driver_type %d\n",
			__func__, __LINE__,
			flash_data->cfg.flash_init_info->flash_driver_type);
	}

	if (flash_ctrl->func_tbl->camera_flash_init) {
		rc = flash_ctrl->func_tbl->camera_flash_init(
				flash_ctrl, flash_data);
		if (rc < 0) {
			pr_err("%s:%d camera_flash_init failed rc = %d",
				__func__, __LINE__, rc);
			return rc;
		}
	}

	flash_ctrl->flash_state = MSM_CAMERA_FLASH_INIT;

	CDBG("Exit");
	return 0;
}

static int32_t msm_flash_init_prepare(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
#ifdef CONFIG_COMPAT
	struct msm_flash_cfg_data_t flash_data_k;
	struct msm_flash_init_info_t flash_init_info;
	int32_t i = 0;

	if (!is_compat_task()) {
		/*for 64-bit usecase,it need copy the data to local memory*/
		flash_data_k.cfg_type = flash_data->cfg_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_data_k.flash_current[i] =
				flash_data->flash_current[i];
			flash_data_k.flash_duration[i] =
				flash_data->flash_duration[i];
		}

		flash_data_k.cfg.flash_init_info = &flash_init_info;
		if (copy_from_user(&flash_init_info,
			(void __user *)(flash_data->cfg.flash_init_info),
			sizeof(struct msm_flash_init_info_t))) {
			pr_err("%s copy_from_user failed %d\n",
				__func__, __LINE__);
			return -EFAULT;
		}
		return msm_flash_init(flash_ctrl, &flash_data_k);
	}
	/*
	 * for 32-bit usecase,it already copy the userspace
	 * data to local memory in msm_flash_subdev_do_ioctl()
	 * so here do not need copy from user
	 */
	return msm_flash_init(flash_ctrl, flash_data);
#else
	struct msm_flash_cfg_data_t flash_data_k;
	struct msm_flash_init_info_t flash_init_info;
	int32_t i = 0;
	flash_data_k.cfg_type = flash_data->cfg_type;
	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		flash_data_k.flash_current[i] =
			flash_data->flash_current[i];
		flash_data_k.flash_duration[i] =
			flash_data->flash_duration[i];
	}

	flash_data_k.cfg.flash_init_info = &flash_init_info;
	if (copy_from_user(&flash_init_info,
		(void __user *)(flash_data->cfg.flash_init_info),
		sizeof(struct msm_flash_init_info_t))) {
		pr_err("%s copy_from_user failed %d\n",
			__func__, __LINE__);
		return -EFAULT;
	}
	return msm_flash_init(flash_ctrl, &flash_data_k);
#endif
}

static int32_t msm_flash_low(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	//HTC_START, porting flashlight control
	#ifndef CONFIG_HTC_FLASHLIGHT_COMMON
	//HTC_END
	uint32_t curr = 0, max_current = 0;
	int32_t i = 0;
	//HTC_START, porting flashlight control
	#endif
	//HTC_END

	CDBG("Enter\n");
    //HTC_START, porting flashlight control
    CDBG("pdev->id = %d\n", flash_ctrl->pdev->id);
    //HTC_END

	//HTC_START, porting flashlight control
	#ifdef CONFIG_HTC_FLASHLIGHT_COMMON
	if(htc_torch_main && htc_flash_main)
	{
	    if (flash_ctrl->pdev->id == 0) {
                htc_torch_main((int)flash_data->flash_current[0], (int)flash_data->flash_current[1]);
                pr_info("[CAM][FL] set main torch (%d,%d)\n",(int)flash_data->flash_current[0],(int)flash_data->flash_current[1]);
            }
	}
	else
		pr_err("[CAM][FL] Main msm_flash_low, flashlight control is NULL\n");

	if(htc_flash_front && htc_torch_front)
	{
		if (flash_ctrl->pdev->id == 1)
                    htc_torch_front(50,50);
	}
	else
		pr_err("[CAM][FL] Front msm_flash_low, flashlight control is NULL\n");

	#else
	//HTC_END

	/* Turn off flash triggers */
	for (i = 0; i < flash_ctrl->flash_num_sources; i++)
		if (flash_ctrl->flash_trigger[i])
			led_trigger_event(flash_ctrl->flash_trigger[i], 0);

	/* Turn on flash triggers */
	for (i = 0; i < flash_ctrl->torch_num_sources; i++) {
		if (flash_ctrl->torch_trigger[i]) {
			max_current = flash_ctrl->torch_max_current[i];
			if (flash_data->flash_current[i] >= 0 &&
				flash_data->flash_current[i] <
				max_current) {
				curr = flash_data->flash_current[i];
			} else {
				curr = flash_ctrl->torch_op_current[i];
				pr_debug("LED current clamped to %d\n",
					curr);
			}
			CDBG("low_flash_current[%d] = %d", i, curr);
			led_trigger_event(flash_ctrl->torch_trigger[i],
				curr);
		}
	}
	if (flash_ctrl->switch_trigger)
		led_trigger_event(flash_ctrl->switch_trigger, 1);

	//HTC_START, porting flashlight control
	#endif
	//HTC_END

	CDBG("Exit\n");
#if 1
	currently_blinking = 0;
	currently_torch_mode = 1;
#endif
	return 0;
}

static int32_t msm_flash_high(
	struct msm_flash_ctrl_t *flash_ctrl,
	struct msm_flash_cfg_data_t *flash_data)
{
	//HTC_START, porting flashlight control
	#ifndef CONFIG_HTC_FLASHLIGHT_COMMON
	//HTC_END
	int32_t curr = 0;
	int32_t max_current = 0;
	int32_t i = 0;
	//HTC_START, porting flashlight control
	#endif
	//HTC_END

	CDBG("Enter\n");

	//HTC_START, porting flashlight control
	#ifdef CONFIG_HTC_FLASHLIGHT_COMMON
	if(htc_flash_main && htc_torch_main)
	{
		if(flash_ctrl->pdev->id ==0)
		{
		    pr_info("[CAM][FL] Main msm_flash_high, called linear flashlight current value (%d, %d)\n", flash_data->flash_current[0], flash_data->flash_current[1]);
		    htc_flash_main((int)flash_data->flash_current[0], (int)flash_data->flash_current[1]);
		}
	}
	else
		pr_err("[CAM][FL] Main msm_flash_high, flashlight control is NULL\n");

	if(htc_flash_front && htc_torch_front)
	{
		if(flash_ctrl->pdev->id ==1)
		{
		    pr_info("[CAM][FL] Front msm_flash_high, called linear flashlight current value (%d, %d)\n", flash_data->flash_current[0], flash_data->flash_current[1]);
		    htc_flash_front((int)flash_data->flash_current[0], (int)flash_data->flash_current[1]);
		}
	}
	else
		pr_err("[CAM][FL] Front msm_flash_high, flashlight control is NULL\n");

	#else
	//HTC_END

	/* Turn off torch triggers */
	for (i = 0; i < flash_ctrl->torch_num_sources; i++)
		if (flash_ctrl->torch_trigger[i])
			led_trigger_event(flash_ctrl->torch_trigger[i], 0);

	/* Turn on flash triggers */
	for (i = 0; i < flash_ctrl->flash_num_sources; i++) {
		if (flash_ctrl->flash_trigger[i]) {
			max_current = flash_ctrl->flash_max_current[i];
			if (flash_data->flash_current[i] >= 0 &&
				flash_data->flash_current[i] <
				max_current) {
				curr = flash_data->flash_current[i];
			} else {
				curr = flash_ctrl->flash_op_current[i];
				pr_debug("LED flash_current[%d] clamped %d\n",
					i, curr);
			}
			CDBG("high_flash_current[%d] = %d", i, curr);
			led_trigger_event(flash_ctrl->flash_trigger[i],
				curr);
		}
	}
	if (flash_ctrl->switch_trigger)
		led_trigger_event(flash_ctrl->switch_trigger, 1);

	//HTC_START, porting flashlight control
	#endif
	//HTC_END

	return 0;
}

static int32_t msm_flash_release(
	struct msm_flash_ctrl_t *flash_ctrl)
{
	int32_t rc = 0;

	rc = flash_ctrl->func_tbl->camera_flash_off(flash_ctrl, NULL);
	if (rc < 0) {
		pr_err("%s:%d camera_flash_init failed rc = %d",
			__func__, __LINE__, rc);
		return rc;
	}
	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	return 0;
}

static int32_t msm_flash_config(struct msm_flash_ctrl_t *flash_ctrl,
	void __user *argp)
{
	int32_t rc = 0;
	struct msm_flash_cfg_data_t *flash_data =
		(struct msm_flash_cfg_data_t *) argp;

	mutex_lock(flash_ctrl->flash_mutex);

//	CDBG("Enter %s type %d\n", __func__, flash_data->cfg_type);

	switch (flash_data->cfg_type) {
	case CFG_FLASH_INIT:
		rc = msm_flash_init_prepare(flash_ctrl, flash_data);
		break;
	case CFG_FLASH_RELEASE:
		if (flash_ctrl->flash_state != MSM_CAMERA_FLASH_RELEASE) {
			rc = flash_ctrl->func_tbl->camera_flash_release(
				flash_ctrl);
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	case CFG_FLASH_OFF:
		if ((flash_ctrl->flash_state != MSM_CAMERA_FLASH_RELEASE) &&
			(flash_ctrl->flash_state != MSM_CAMERA_FLASH_OFF)) {
			rc = flash_ctrl->func_tbl->camera_flash_off(
				flash_ctrl, flash_data);
			if (!rc)
				flash_ctrl->flash_state = MSM_CAMERA_FLASH_OFF;
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	case CFG_FLASH_LOW:
		if ((flash_ctrl->flash_state == MSM_CAMERA_FLASH_OFF) ||
			(flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT)) {
			rc = flash_ctrl->func_tbl->camera_flash_low(
				flash_ctrl, flash_data);
			if (!rc)
				flash_ctrl->flash_state = MSM_CAMERA_FLASH_LOW;
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	case CFG_FLASH_HIGH:
		if ((flash_ctrl->flash_state == MSM_CAMERA_FLASH_OFF) ||
			(flash_ctrl->flash_state == MSM_CAMERA_FLASH_INIT)) {
			rc = flash_ctrl->func_tbl->camera_flash_high(
				flash_ctrl, flash_data);
			if (!rc)
				flash_ctrl->flash_state = MSM_CAMERA_FLASH_HIGH;
		} else {
			CDBG(pr_fmt("Invalid state : %d\n"),
				flash_ctrl->flash_state);
		}
		break;
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(flash_ctrl->flash_mutex);

//	CDBG("Exit %s type %d\n", __func__, flash_data->cfg_type);

	return rc;
}

static long msm_flash_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	struct msm_flash_ctrl_t *fctrl = NULL;
	void __user *argp = (void __user *)arg;

//	CDBG("Enter\n");

	if (!sd) {
		pr_err("sd NULL\n");
		return -EINVAL;
	}
	fctrl = v4l2_get_subdevdata(sd);
	if (!fctrl) {
		pr_err("fctrl NULL\n");
		return -EINVAL;
	}
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return msm_flash_get_subdev_id(fctrl, argp);
	case VIDIOC_MSM_FLASH_CFG:
		return msm_flash_config(fctrl, argp);
	case MSM_SD_NOTIFY_FREEZE:
		return 0;
	case MSM_SD_UNNOTIFY_FREEZE:
		return 0;
	case MSM_SD_SHUTDOWN:
		if (!fctrl->func_tbl) {
			pr_err("fctrl->func_tbl NULL\n");
			return -EINVAL;
		} else {
			return fctrl->func_tbl->camera_flash_release(fctrl);
		}
	default:
		pr_err_ratelimited("invalid cmd %d\n", cmd);
		return -ENOIOCTLCMD;
	}
//	CDBG("Exit\n");
}

static struct v4l2_subdev_core_ops msm_flash_subdev_core_ops = {
	.ioctl = msm_flash_subdev_ioctl,
};

static struct v4l2_subdev_ops msm_flash_subdev_ops = {
	.core = &msm_flash_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops msm_flash_internal_ops;

static int32_t msm_flash_get_pmic_source_info(
	struct device_node *of_node,
	struct msm_flash_ctrl_t *fctrl)
{
	int32_t rc = 0;
	uint32_t count = 0, i = 0;
	struct device_node *flash_src_node = NULL;
	struct device_node *torch_src_node = NULL;
	struct device_node *switch_src_node = NULL;

	switch_src_node = of_parse_phandle(of_node, "qcom,switch-source", 0);
	if (!switch_src_node) {
		CDBG("%s:%d switch_src_node NULL\n", __func__, __LINE__);
	} else {
		rc = of_property_read_string(switch_src_node,
			"qcom,default-led-trigger",
			&fctrl->switch_trigger_name);
		if (rc < 0) {
			rc = of_property_read_string(switch_src_node,
				"linux,default-trigger",
				&fctrl->switch_trigger_name);
			if (rc < 0)
				pr_err("default-trigger read failed\n");
		}
		of_node_put(switch_src_node);
		switch_src_node = NULL;
		if (!rc) {
			CDBG("switch trigger %s\n",
				fctrl->switch_trigger_name);
			led_trigger_register_simple(
				fctrl->switch_trigger_name,
				&fctrl->switch_trigger);
		}
	}

	if (of_get_property(of_node, "qcom,flash-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl->flash_num_sources = count;
		CDBG("%s:%d flash_num_sources = %d",
			__func__, __LINE__, fctrl->flash_num_sources);
		for (i = 0; i < count; i++) {
			flash_src_node = of_parse_phandle(of_node,
				"qcom,flash-source", i);
			if (!flash_src_node) {
				pr_err("flash_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(flash_src_node,
				"qcom,default-led-trigger",
				&fctrl->flash_trigger_name[i]);
			if (rc < 0) {
				rc = of_property_read_string(flash_src_node,
					"linux,default-trigger",
					&fctrl->flash_trigger_name[i]);
				if (rc < 0) {
					pr_err("default-trigger read failed\n");
					of_node_put(flash_src_node);
					continue;
				}
			}

			CDBG("default trigger %s\n",
				fctrl->flash_trigger_name[i]);

			/* Read operational-current */
			rc = of_property_read_u32(flash_src_node,
				"qcom,current",
				&fctrl->flash_op_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			/* Read max-current */
			rc = of_property_read_u32(flash_src_node,
				"qcom,max-current",
				&fctrl->flash_max_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(flash_src_node);
				continue;
			}

			/* Read max-duration */
			rc = of_property_read_u32(flash_src_node,
				"qcom,duration",
				&fctrl->flash_max_duration[i]);
			if (rc < 0) {
				pr_err("duration: read failed\n");
				of_node_put(flash_src_node);
				/* Non-fatal; this property is optional */
			}

			of_node_put(flash_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->flash_op_current[i]);

			led_trigger_register_simple(
				fctrl->flash_trigger_name[i],
				&fctrl->flash_trigger[i]);
		}
		if (fctrl->flash_driver_type == FLASH_DRIVER_DEFAULT)
			fctrl->flash_driver_type = FLASH_DRIVER_PMIC;
		CDBG("%s:%d fctrl->flash_driver_type = %d", __func__, __LINE__,
			fctrl->flash_driver_type);
	}

	if (of_get_property(of_node, "qcom,torch-source", &count)) {
		count /= sizeof(uint32_t);
		CDBG("count %d\n", count);
		if (count > MAX_LED_TRIGGERS) {
			pr_err("invalid count\n");
			return -EINVAL;
		}
		fctrl->torch_num_sources = count;
		CDBG("%s:%d torch_num_sources = %d",
			__func__, __LINE__, fctrl->torch_num_sources);
		for (i = 0; i < count; i++) {
			torch_src_node = of_parse_phandle(of_node,
				"qcom,torch-source", i);
			if (!torch_src_node) {
				pr_err("torch_src_node NULL\n");
				continue;
			}

			rc = of_property_read_string(torch_src_node,
				"qcom,default-led-trigger",
				&fctrl->torch_trigger_name[i]);
			if (rc < 0) {
				rc = of_property_read_string(torch_src_node,
					"linux,default-trigger",
					&fctrl->torch_trigger_name[i]);
				if (rc < 0) {
					pr_err("default-trigger read failed\n");
					of_node_put(torch_src_node);
					continue;
				}
			}

			CDBG("default trigger %s\n",
				fctrl->torch_trigger_name[i]);

			/* Read operational-current */
			rc = of_property_read_u32(torch_src_node,
				"qcom,current",
				&fctrl->torch_op_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(torch_src_node);
				continue;
			}

			/* Read max-current */
			rc = of_property_read_u32(torch_src_node,
				"qcom,max-current",
				&fctrl->torch_max_current[i]);
			if (rc < 0) {
				pr_err("current: read failed\n");
				of_node_put(torch_src_node);
				continue;
			}

			of_node_put(torch_src_node);

			CDBG("max_current[%d] %d\n",
				i, fctrl->torch_op_current[i]);

			led_trigger_register_simple(
				fctrl->torch_trigger_name[i],
				&fctrl->torch_trigger[i]);
		}
		if (fctrl->flash_driver_type == FLASH_DRIVER_DEFAULT)
			fctrl->flash_driver_type = FLASH_DRIVER_PMIC;
		CDBG("%s:%d fctrl->flash_driver_type = %d", __func__, __LINE__,
			fctrl->flash_driver_type);
	}

	return 0;
}

static int32_t msm_flash_get_dt_data(struct device_node *of_node,
	struct msm_flash_ctrl_t *fctrl)
{
	int32_t rc = 0;

	CDBG("called\n");

	if (!of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	/* Read the sub device */
	rc = of_property_read_u32(of_node, "cell-index", &fctrl->pdev->id);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	CDBG("subdev id %d\n", fctrl->subdev_id);

	fctrl->flash_driver_type = FLASH_DRIVER_DEFAULT;

	/* Read the CCI master. Use M0 if not available in the node */
	rc = of_property_read_u32(of_node, "qcom,cci-master",
		&fctrl->cci_i2c_master);
	CDBG("%s qcom,cci-master %d, rc %d\n", __func__, fctrl->cci_i2c_master,
		rc);
	if (rc < 0) {
		/* Set default master 0 */
		fctrl->cci_i2c_master = MASTER_0;
		rc = 0;
	} else {
		fctrl->flash_driver_type = FLASH_DRIVER_I2C;
	}

	/* Read the flash and torch source info from device tree node */
	rc = msm_flash_get_pmic_source_info(of_node, fctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_flash_get_pmic_source_info failed rc %d\n",
			__func__, __LINE__, rc);
		return rc;
	}

	/* Read the gpio information from device tree */
	rc = msm_sensor_driver_get_gpio_data(
		&(fctrl->power_info.gpio_conf), of_node);
	if (-ENODEV == rc) {
		pr_notice("No valid flash GPIOs data\n");
		rc = 0;
	} else if (rc < 0) {
		pr_err("Error flash GPIOs rc %d\n", rc);
		return rc;
	}

	if (fctrl->flash_driver_type == FLASH_DRIVER_DEFAULT)
		fctrl->flash_driver_type = FLASH_DRIVER_GPIO;
	CDBG("%s:%d fctrl->flash_driver_type = %d", __func__, __LINE__,
		fctrl->flash_driver_type);

	return rc;
}

#ifdef CONFIG_COMPAT
static long msm_flash_subdev_do_ioctl(
	struct file *file, unsigned int cmd, void *arg)
{
	int32_t i = 0;
	int32_t rc = 0;
	struct video_device *vdev;
	struct v4l2_subdev *sd;
	struct msm_flash_cfg_data_t32 *u32;
	struct msm_flash_cfg_data_t flash_data;
	struct msm_flash_init_info_t32 flash_init_info32;
	struct msm_flash_init_info_t flash_init_info;

//	CDBG("Enter");

	if (!file || !arg) {
		pr_err("%s:failed NULL parameter\n", __func__);
		return -EINVAL;
	}
	vdev = video_devdata(file);
	sd = vdev_to_v4l2_subdev(vdev);
	u32 = (struct msm_flash_cfg_data_t32 *)arg;

	switch (cmd) {
	case VIDIOC_MSM_FLASH_CFG32:
		flash_data.cfg_type = u32->cfg_type;
		for (i = 0; i < MAX_LED_TRIGGERS; i++) {
			flash_data.flash_current[i] = u32->flash_current[i];
			flash_data.flash_duration[i] = u32->flash_duration[i];
		}
		cmd = VIDIOC_MSM_FLASH_CFG;
		switch (flash_data.cfg_type) {
		case CFG_FLASH_OFF:
		case CFG_FLASH_LOW:
		case CFG_FLASH_HIGH:
			flash_data.cfg.settings = compat_ptr(u32->cfg.settings);
			break;
		case CFG_FLASH_INIT:
			flash_data.cfg.flash_init_info = &flash_init_info;
			if (copy_from_user(&flash_init_info32,
				(void *)compat_ptr(u32->cfg.flash_init_info),
				sizeof(struct msm_flash_init_info_t32))) {
				pr_err("%s copy_from_user failed %d\n",
					__func__, __LINE__);
				return -EFAULT;
			}
			flash_init_info.flash_driver_type =
				flash_init_info32.flash_driver_type;
			flash_init_info.slave_addr =
				flash_init_info32.slave_addr;
			flash_init_info.i2c_freq_mode =
				flash_init_info32.i2c_freq_mode;
			flash_init_info.settings =
				compat_ptr(flash_init_info32.settings);
			flash_init_info.power_setting_array =
				compat_ptr(
				flash_init_info32.power_setting_array);
			break;
		default:
			break;
		}
		break;
	default:
		return msm_flash_subdev_ioctl(sd, cmd, arg);
	}

	rc =  msm_flash_subdev_ioctl(sd, cmd, &flash_data);
	for (i = 0; i < MAX_LED_TRIGGERS; i++) {
		u32->flash_current[i] = flash_data.flash_current[i];
		u32->flash_duration[i] = flash_data.flash_duration[i];
	}
//	CDBG("Exit");
	return rc;
}

static long msm_flash_subdev_fops_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	return video_usercopy(file, cmd, arg, msm_flash_subdev_do_ioctl);
}
#endif
static int32_t msm_flash_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_flash_ctrl_t *flash_ctrl = NULL;
	struct msm_camera_cci_client *cci_client = NULL;

	CDBG("Enter");
	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	flash_ctrl = kzalloc(sizeof(struct msm_flash_ctrl_t), GFP_KERNEL);
	if (!flash_ctrl) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	memset(flash_ctrl, 0, sizeof(struct msm_flash_ctrl_t));

	flash_ctrl->pdev = pdev;

	rc = msm_flash_get_dt_data(pdev->dev.of_node, flash_ctrl);
	if (rc < 0) {
		pr_err("%s:%d msm_flash_get_dt_data failed\n",
			__func__, __LINE__);
		kfree(flash_ctrl);
		return -EINVAL;
	}

	flash_ctrl->flash_state = MSM_CAMERA_FLASH_RELEASE;
	flash_ctrl->power_info.dev = &flash_ctrl->pdev->dev;
	flash_ctrl->flash_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	flash_ctrl->flash_mutex = &msm_flash_mutex;
	flash_ctrl->flash_i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	flash_ctrl->flash_i2c_client.cci_client = kzalloc(
		sizeof(struct msm_camera_cci_client), GFP_KERNEL);
	if (!flash_ctrl->flash_i2c_client.cci_client) {
		kfree(flash_ctrl);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = flash_ctrl->flash_i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = flash_ctrl->cci_i2c_master;

	/* Initialize sub device */
	v4l2_subdev_init(&flash_ctrl->msm_sd.sd, &msm_flash_subdev_ops);
	v4l2_set_subdevdata(&flash_ctrl->msm_sd.sd, flash_ctrl);

	flash_ctrl->msm_sd.sd.internal_ops = &msm_flash_internal_ops;
	flash_ctrl->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(flash_ctrl->msm_sd.sd.name,
		ARRAY_SIZE(flash_ctrl->msm_sd.sd.name),
		"msm_camera_flash");
	media_entity_init(&flash_ctrl->msm_sd.sd.entity, 0, NULL, 0);
	flash_ctrl->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	flash_ctrl->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_FLASH;
	flash_ctrl->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x1;
	msm_sd_register(&flash_ctrl->msm_sd);

	CDBG("%s:%d flash sd name = %s", __func__, __LINE__,
		flash_ctrl->msm_sd.sd.entity.name);
	msm_cam_copy_v4l2_subdev_fops(&msm_flash_v4l2_subdev_fops);
#ifdef CONFIG_COMPAT
	msm_flash_v4l2_subdev_fops.compat_ioctl32 =
		msm_flash_subdev_fops_ioctl;
#endif
	flash_ctrl->msm_sd.sd.devnode->fops = &msm_flash_v4l2_subdev_fops;

	if (flash_ctrl->flash_driver_type == FLASH_DRIVER_PMIC)
		rc = msm_torch_create_classdev(pdev, flash_ctrl);

	CDBG("probe success\n");
	return rc;
}

MODULE_DEVICE_TABLE(of, msm_flash_dt_match);

static struct platform_driver msm_flash_platform_driver = {
	.probe = msm_flash_platform_probe,
	.driver = {
		.name = "qcom,camera-flash",
		.owner = THIS_MODULE,
		.of_match_table = msm_flash_dt_match,
	},
};

//HTC_START
#ifdef HTC_CAM_FEATURE_FLASH_RESTRICTION
static uint32_t led_ril_status_value;
static uint32_t led_wimax_status_value;
static uint32_t led_hotspot_status_value;
static uint16_t led_low_temp_limit = 5;
static uint16_t led_low_cap_limit = 14;
static uint16_t led_low_cap_limit_dual = 14;

static ssize_t led_ril_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_ril_status_value);
	return length;
}

static ssize_t led_ril_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_ril_status_value = tmp;
	pr_info("[CAM][FL] led_ril_status_value = %d\n", led_ril_status_value);
	return count;
}

static ssize_t led_wimax_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_wimax_status_value);
	return length;
}

static ssize_t led_wimax_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	if (buf[1] == '\n')
		tmp = buf[0] - 0x30;

	led_wimax_status_value = tmp;
	pr_info("[CAM][FL] led_wimax_status_value = %d\n", led_wimax_status_value);
	return count;
}

static ssize_t led_hotspot_status_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_hotspot_status_value);
	return length;
}

static ssize_t led_hotspot_status_set(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	uint32_t tmp = 0;

	tmp = buf[0] - 0x30; /* only get the first char */

	led_hotspot_status_value = tmp;
	pr_info("[CAM][FL] led_hotspot_status_value = %d\n", led_hotspot_status_value);
	return count;
}

static ssize_t low_temp_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_temp_limit);
	return length;
}

static ssize_t low_cap_limit_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit);
	return length;
}

static ssize_t low_cap_limit_dual_get(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	ssize_t length;
	length = sprintf(buf, "%d\n", led_low_cap_limit_dual);
	return length;
}

static DEVICE_ATTR(led_ril_status, 0644,
	led_ril_status_get,
	led_ril_status_set);

static DEVICE_ATTR(led_wimax_status, 0644,
	led_wimax_status_get,
	led_wimax_status_set);

static DEVICE_ATTR(led_hotspot_status, 0644,
	led_hotspot_status_get,
	led_hotspot_status_set);

static DEVICE_ATTR(low_temp_limit, 0444,
	low_temp_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit, 0444,
	low_cap_limit_get,
	NULL);

static DEVICE_ATTR(low_cap_limit_dual, 0444,
	low_cap_limit_dual_get,
	NULL);

static int __init msm_led_trigger_sysfs_init(void)
{
	int ret = 0;

	pr_info("[CAM][FL] %s:%d\n", __func__, __LINE__);

	led_status_obj = kobject_create_and_add("camera_led_status", NULL);
	if (led_status_obj == NULL) {
		pr_info("[CAM][FL] msm_camera_led: subsystem_register failed\n");
		ret = -ENOMEM;
		goto error;
	}

	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_ril_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_ril_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_wimax_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_wimax_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_led_hotspot_status.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_led_hotspot_status failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_temp_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_temp_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit failed\n");
		ret = -EFAULT;
		goto error;
	}
	ret = sysfs_create_file(led_status_obj,
		&dev_attr_low_cap_limit_dual.attr);
	if (ret) {
		pr_err("[CAM][FL] msm_camera_led: sysfs_create_file dev_attr_low_cap_limit_dual failed\n");
		ret = -EFAULT;
		goto error;
	}

	pr_info("[CAM][FL] %s:%d ret %d\n", __func__, __LINE__, ret);
	return ret;

error:
	kobject_del(led_status_obj);
	return ret;

}
#endif //HTC_CAM_FEATURE_FLASH_RESTRICTION
//HTC_END

static int __init msm_flash_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_register(&msm_flash_platform_driver);
//HTC_START, HTC_CAM_FEATURE_FLASH_RESTRICTION
#ifdef HTC_CAM_FEATURE_FLASH_RESTRICTION
	if (!rc)
	{
		rc = msm_led_trigger_sysfs_init();
		pr_info("%s:%d rc %d\n", __func__, __LINE__, rc);
#if 1
		alarm_init(&flash_blink_rtc, ALARM_REALTIME,
			flash_blink_rtc_callback);
		alarm_init(&flash_blink_do_blink_rtc, ALARM_REALTIME,
			flash_blink_do_blink_rtc_callback);
		alarm_init(&vib_rtc, ALARM_REALTIME,
			vib_rtc_callback);
		flash_blink_workqueue = alloc_workqueue("flash_blink", WQ_HIGHPRI, 1);
		flash_start_blink_workqueue = alloc_workqueue("flash_start_blink", WQ_HIGHPRI, 1);
		flash_stop_blink_workqueue = alloc_workqueue("flash_stop_blink", WQ_HIGHPRI, 1);
		INIT_WORK(&flash_blink_work, flash_blink_work_func);
		INIT_WORK(&flash_start_blink_work, flash_start_blink_work_func);
		INIT_WORK(&flash_stop_blink_work, flash_stop_blink_work_func);
		uci_add_sys_listener(flash_uci_sys_listener);
		init_done = 1;
#endif
		return rc;
	}
#endif //HTC_CAM_FEATURE_FLASH_RESTRICTION
//HTC_END, HTC_CAM_FEATURE_FLASH_RESTRICTION
	if (rc)
		pr_err("platform probe for flash failed");

	return rc;
}

static void __exit msm_flash_exit_module(void)
{
	platform_driver_unregister(&msm_flash_platform_driver);
	return;
}

static struct msm_flash_table msm_pmic_flash_table = {
	.flash_driver_type = FLASH_DRIVER_PMIC,
	.func_tbl = {
		.camera_flash_init = NULL,
		.camera_flash_release = msm_flash_release,
		.camera_flash_off = msm_flash_off,
		.camera_flash_low = msm_flash_low,
		.camera_flash_high = msm_flash_high,
	},
};

static struct msm_flash_table msm_gpio_flash_table = {
	.flash_driver_type = FLASH_DRIVER_GPIO,
	.func_tbl = {
		.camera_flash_init = msm_flash_gpio_init,
		.camera_flash_release = msm_flash_release,
		.camera_flash_off = msm_flash_off,
		.camera_flash_low = msm_flash_low,
		.camera_flash_high = msm_flash_high,
	},
};

static struct msm_flash_table msm_i2c_flash_table = {
	.flash_driver_type = FLASH_DRIVER_I2C,
	.func_tbl = {
		.camera_flash_init = msm_flash_i2c_init,
		.camera_flash_release = msm_flash_i2c_release,
		.camera_flash_off = msm_flash_i2c_write_setting_array,
		.camera_flash_low = msm_flash_i2c_write_setting_array,
		.camera_flash_high = msm_flash_i2c_write_setting_array,
	},
};

module_init(msm_flash_init_module);
module_exit(msm_flash_exit_module);
MODULE_DESCRIPTION("MSM FLASH");
MODULE_LICENSE("GPL v2");
