#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/vibtrig.h>

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <linux/alarmtimer.h>
#include <linux/notification/notification.h>
#include <linux/uci/uci.h>

#define DRIVER_AUTHOR "illes pal <illespal@gmail.com>"
#define DRIVER_DESCRIPTION "fingerprint_filter driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#ifdef CONFIG_HZ_300
#define JIFFY_MUL 3
#else
#define JIFFY_MUL 1
#endif

#define fpf_PWRKEY_DUR          60
#define FUNC_CYCLE_DUR          9 + JIFFY_MUL
#define VIB_STRENGTH		20

// touchscreen input handler input work queue and work
static struct workqueue_struct *ts_input_wq;
static struct work_struct ts_input_work;

extern void set_vibrate(int value);

static int fpf_switch = 2;
static struct input_dev * fpf_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static DEFINE_MUTEX(fpfuncworklock);
static struct workqueue_struct *fpf_input_wq;
static struct work_struct fpf_input_work;
extern struct vib_trigger *vib_trigger;
static int vib_strength = VIB_STRENGTH;
static int unlock_vib_strength = VIB_STRENGTH;

static unsigned int last_screen_off_seconds = 0;

unsigned int get_global_seconds(void) {
	struct timespec ts;
	unsigned int ret = 0;
	getnstimeofday(&ts);
	ret = (unsigned int)(ts.tv_sec);
	return ret;
}

static int get_fpf_switch(void) {
	return uci_get_user_property_int_mm("fingerprint_mode", fpf_switch, 0, 2);
}
static int get_vib_strength(void) {
	return uci_get_user_property_int_mm("fp_vib_strength", vib_strength, 0, 90);
}
static int get_unlock_vib_strength(void) {
	return uci_get_user_property_int_mm("fp_unlock_vib_strength", unlock_vib_strength, 0, 90);
}

#ifdef CONFIG_FB
	// early screen on flag
	static int screen_on = 1;
	struct notifier_block *fb_notifier;

int input_is_screen_on(void) {
	return screen_on;
}
EXPORT_SYMBOL(input_is_screen_on);
#endif

#define S_MIN_SECS 60

// --- smart notification settings --
// how many inactive minutes to start trimming some types of notifications' periodicity/length of timeout/repetitions
// should be set to 0 if smart trim is inactive
static int smart_trim_inactive_seconds = 6 * S_MIN_SECS;// 6 mins
// notif settings for trim period
static int smart_trim_kad = NOTIF_TRIM;
static int smart_trim_flashlight = NOTIF_TRIM;
static int smart_trim_vib_reminder = NOTIF_DEFAULT;
static int smart_trim_notif_booster = NOTIF_DEFAULT;
static int smart_trim_bln_light = NOTIF_DEFAULT;
static int smart_trim_pulse_light = NOTIF_DEFAULT;

// how many inactive minutes to start stopping some types of notifications
// should be set to 0 if smart stop is inactive
static int smart_stop_inactive_seconds = 60 * S_MIN_SECS; // 60 mins
// notif settings for stop period
static int smart_stop_kad = NOTIF_STOP;
static int smart_stop_flashlight = NOTIF_DIM;
static int smart_stop_vib_reminder = NOTIF_TRIM;
static int smart_stop_notif_booster = NOTIF_DEFAULT;
static int smart_stop_bln_light = NOTIF_TRIM;
static int smart_stop_pulse_light = NOTIF_DEFAULT;

// how many inactive minutes to start hibarnete (extended stop) some types of notifications
// should be set to 0 if smart stop is inactive
static int smart_hibernate_inactive_seconds = 4 * 60 * S_MIN_SECS; // 4 * 60 mins
// notif settings for hibernate period
static int smart_hibernate_kad = NOTIF_STOP;
static int smart_hibernate_flashlight = NOTIF_STOP;
static int smart_hibernate_vib_reminder = NOTIF_STOP;
static int smart_hibernate_notif_booster = NOTIF_STOP;
static int smart_hibernate_bln_light = NOTIF_DIM;
static int smart_hibernate_pulse_light = NOTIF_DIM;


static int smart_silent_mode_stop = 1;
static int smart_silent_mode_hibernate = 1;

int uci_get_smart_trim_inactive_seconds(void) {
	return uci_get_user_property_int_mm("smart_trim_inactive_mins", smart_trim_inactive_seconds/60, 0, 2 * 24 * 60)*60;
}
int uci_get_smart_stop_inactive_seconds(void) {
	return uci_get_user_property_int_mm("smart_stop_inactive_mins", smart_stop_inactive_seconds/60, 0, 2 * 24 * 60)*60;
}
int uci_get_smart_hibernate_inactive_seconds(void) {
	return uci_get_user_property_int_mm("smart_hibernate_inactive_mins", smart_hibernate_inactive_seconds/60, 0, 2 * 24 * 60)*60;
}
int uci_get_smart_silent_mode_hibernate(void) {
	return uci_get_user_property_int_mm("smart_silent_mode_hibernate", smart_silent_mode_hibernate, 0, 1);
}
int uci_get_smart_silent_mode_stop(void) {
	return uci_get_user_property_int_mm("smart_silent_mode_stop", smart_silent_mode_stop, 0, 1);
}

int fpf_silent_mode = 0;
// KAD should run if in ringing mode... companion app channels the info
int fpf_ringing = 0;
/**
* If an app that is waking screen from sleep like Alarm or Phone, this should be set higher than 0
* If that happens, KAD should STOP running and no new KAD screen should start till value is back to 0,
* meaning apps were closed/dismissed. Companion app channels this number.
*/
int fpf_screen_waking_app = 0;

int silent_mode_hibernate(void) {
	if (uci_get_smart_silent_mode_hibernate()) {
		return fpf_silent_mode;
	}
	return 0;
}
int silent_mode_stop(void) {
	if (uci_get_smart_silent_mode_stop()) {
		return fpf_silent_mode;
	}
	return 0;
}

static int phone_ring_in_silent_mode = 0;
static int get_phone_ring_in_silent_mode(void) {
	return uci_get_user_property_int_mm("phone_ring_in_silent_mode", phone_ring_in_silent_mode, 0, 1);
}

static struct alarm vibrate_rtc;
static enum alarmtimer_restart vibrate_rtc_callback(struct alarm *al, ktime_t now)
{
	pr_info("%s kad\n",__func__);
	set_vibrate(998);
	return ALARMTIMER_NORESTART;
}


static int face_down_screen_off = 1;
static int get_face_down_screen_off(void) {
	return uci_get_user_property_int_mm("face_down_screen_off", face_down_screen_off, 0, 1);
}

bool should_screen_off_face_down(int screen_timeout_sec, int face_down);
static void fpf_pwrtrigger(int vibration, const char caller[]);


// register input event alarm timer
extern void register_input_event(void);

void stop_kernel_ambient_display(bool interrupt_ongoing) { }
EXPORT_SYMBOL(stop_kernel_ambient_display);

void kernel_ambient_display(void) { }
EXPORT_SYMBOL(kernel_ambient_display);

void kernel_ambient_display_led_based(void) { }
EXPORT_SYMBOL(kernel_ambient_display_led_based);

int is_kernel_ambient_display(void) { return 0; }
EXPORT_SYMBOL(is_kernel_ambient_display);

int stored_lock_state = 0;
// register sys uci listener
void fpf_uci_sys_listener(void) {
	int locked = 0;
	pr_info("%s uci sys parse happened...\n",__func__);
	{
		int silent = uci_get_sys_property_int_mm("silent", 0, 0, 1);
		int ringing = uci_get_sys_property_int_mm("ringing", 0, 0, 1);

		int face_down = uci_get_sys_property_int_mm("face_down", 0, 0, 1);
		int screen_timeout_sec = uci_get_sys_property_int_mm("screen_timeout", 15, 0, 600);

		int screen_waking_app = uci_get_sys_property_int("screen_waking_apps", 0);
		locked = uci_get_sys_property_int_mm("locked", 0, 0, 1);
		if (screen_waking_app != -EINVAL) fpf_screen_waking_app = screen_waking_app;

		pr_info("%s uci sys silent %d ringing %d face_down %d timeout %d \n",__func__,silent, ringing, face_down, screen_timeout_sec);
		fpf_silent_mode = silent;
		if (fpf_silent_mode && ringing && (ringing!=fpf_ringing) && get_phone_ring_in_silent_mode()) {
			ktime_t wakeup_time;
			ktime_t curr_time = { .tv64 = 0 };
			wakeup_time = ktime_add_us(curr_time,
			    (2 * 1000LL * 1000LL)); // msec to usec
			alarm_cancel(&vibrate_rtc);
			alarm_start_relative(&vibrate_rtc, wakeup_time); // start new...
			set_vibrate(999);
		} else {
			if (!ringing) {
				alarm_cancel(&vibrate_rtc);
			}
		}
		fpf_ringing = ringing;
		if (screen_on && !ringing && !fpf_screen_waking_app) {
			if (should_screen_off_face_down(screen_timeout_sec, face_down)) {
				fpf_pwrtrigger(0,__func__);
			}
		}
	}
	if (!locked&&stored_lock_state!=locked) {
		register_input_event();
	} else
	if (fpf_ringing || fpf_screen_waking_app) {
		register_input_event();
	}
	stored_lock_state = locked;
}



static unsigned int smart_last_user_activity_time = 0;
void smart_set_last_user_activity_time(void) {
	smart_last_user_activity_time = get_global_seconds();
}
EXPORT_SYMBOL(smart_set_last_user_activity_time);

int smart_get_inactivity_time(void) {
	unsigned int diff = 0;
	int diff_in_sec = 0;
	if (smart_last_user_activity_time==0) smart_last_user_activity_time = get_global_seconds();
	diff = get_global_seconds() - smart_last_user_activity_time;
	diff_in_sec = diff / 1;
	pr_info("%s smart_notif - inactivity in sec: %d\n",__func__, diff_in_sec);
	return diff_in_sec;
}


int smart_get_notification_level(int notif_type) {
	int diff_in_sec = smart_get_inactivity_time();
	int ret = NOTIF_DEFAULT;
	bool trim = uci_get_smart_trim_inactive_seconds() > 0 && diff_in_sec > uci_get_smart_trim_inactive_seconds();
	bool stop = uci_get_smart_stop_inactive_seconds() > 0 && diff_in_sec > uci_get_smart_stop_inactive_seconds();
	bool hibr = uci_get_smart_hibernate_inactive_seconds() > 0 && diff_in_sec > uci_get_smart_hibernate_inactive_seconds();
	if (silent_mode_hibernate()) hibr = true;
	if (silent_mode_stop()) stop = true;
	switch (notif_type) {
		case NOTIF_KAD:
			ret = hibr?smart_hibernate_kad:(stop?smart_stop_kad:(trim?smart_trim_kad:NOTIF_DEFAULT));
			break;
		case NOTIF_FLASHLIGHT:
			ret = hibr?smart_hibernate_flashlight:(stop?smart_stop_flashlight:(trim?smart_trim_flashlight:NOTIF_DEFAULT));
			break;
		case NOTIF_VIB_REMINDER:
			if ( (hibr?smart_hibernate_flashlight:(stop?smart_stop_flashlight:(trim?smart_trim_flashlight:NOTIF_DEFAULT) )) == NOTIF_STOP ) ret = NOTIF_STOP; else // without flashlight, no reminder possible
			ret = hibr?smart_hibernate_vib_reminder:(stop?smart_stop_vib_reminder:(trim?smart_trim_vib_reminder:NOTIF_DEFAULT));
			break;
		case NOTIF_VIB_BOOSTER:
			ret = hibr?smart_hibernate_notif_booster:(stop?smart_stop_notif_booster:(trim?smart_trim_notif_booster:NOTIF_DEFAULT));
			break;
		case NOTIF_BUTTON_LIGHT:
			ret = hibr?smart_hibernate_bln_light:(stop?smart_stop_bln_light:(trim?smart_trim_bln_light:NOTIF_DEFAULT));
			break;
		case NOTIF_PULSE_LIGHT:
			ret = hibr?smart_hibernate_pulse_light:(stop?smart_stop_pulse_light:(trim?smart_trim_pulse_light:NOTIF_DEFAULT));
			break;
	}
	pr_info("%s smart_notif - level for type %d is %d -- state trim %d stop %d hibr %d \n",__func__, notif_type, ret, trim,stop,hibr);
	return ret;
}
EXPORT_SYMBOL(smart_get_notification_level);

// /// smart notif ////


int last_screen_lock_check_was_false = 0;


bool is_screen_locked(void) {
	int lock_timeout_sec = uci_get_sys_property_int_mm("lock_timeout", 0, 0, 1900);
	int locked = uci_get_sys_property_int_mm("locked", 1, 0, 1);
	int time_passed = get_global_seconds() - last_screen_off_seconds;

	pr_info("%s fpf locked; %d lock timeout: %d time passed after blank: %d \n",__func__,locked, lock_timeout_sec, time_passed);

	if (locked) return true;

	if (!last_screen_lock_check_was_false && time_passed>=lock_timeout_sec) {
		return true;
	}
	if (screen_on) {
		// screen was just turned but not enough time passed...
		// ...till next screen off lock_timeout shouldn't be checked, as with screen on, lock timeout obviously won't happen
		last_screen_lock_check_was_false = 1;
	}
	return false;
}

/**
* tells if currently a facedown event from companion app UCI sys triggering, should at the same time do a screen off as well
*/
bool should_screen_off_face_down(int screen_timeout_sec, int face_down) {
	if (get_face_down_screen_off() && screen_on) {
		if (smart_get_inactivity_time()<(screen_timeout_sec-3) && face_down) {
			pr_info("%s face down screen off! \n",__func__);
			return true;
		}
	}
	return false;
}

extern void set_notification_booster(int value);
extern int get_notification_booster(void);
extern void set_notification_boost_only_in_pocket(int value);
extern int get_notification_boost_only_in_pocket(void);

// value used to signal that HOME button release event should be synced as well in home button func work if it was not interrupted.
static int do_home_button_off_too_in_work_func = 0;

/* PowerKey work func */
static void fpf_presspwr(struct work_struct * fpf_presspwr_work) {

	if (!mutex_trylock(&pwrkeyworklock))
                return;
	input_event(fpf_pwrdev, EV_KEY, KEY_POWER, 1);
	input_event(fpf_pwrdev, EV_SYN, 0, 0);
	msleep(fpf_PWRKEY_DUR);
	input_event(fpf_pwrdev, EV_KEY, KEY_POWER, 0);
	input_event(fpf_pwrdev, EV_SYN, 0, 0);
	msleep(fpf_PWRKEY_DUR/2);
	// resetting this do_home_button_off_too_in_work_func when powering down, as it causes the running HOME button func work 
	//	to trigger a HOME button release sync event to input device, resulting in an unwanted  screen on.
	do_home_button_off_too_in_work_func = 0;
        mutex_unlock(&pwrkeyworklock);
	return;
}
static DECLARE_WORK(fpf_presspwr_work, fpf_presspwr);

static void fpf_vib(void) {
	// avoid using squeeze vib length 15...
	set_vibrate(get_vib_strength()==15?14:get_vib_strength());
}

/* PowerKey trigger */
static void fpf_pwrtrigger(int vibration, const char caller[]) {
	if (vibration) fpf_vib();
	pr_info("%s power press - screen_on: %d caller %s\n",__func__, screen_on,caller);
	schedule_work(&fpf_presspwr_work);
        return;
}


static void fpf_input_callback(struct work_struct *unused) {
	return;
}

static void fpf_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
}

static int input_dev_filter(struct input_dev *dev) {
	if (strstr(dev->name, "fpc1020")) {
		return 0;
	} else {
		return 1;
	}
}

static int fpf_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "fpf";

	error = input_register_handle(handle);

	error = input_open_device(handle);

	return 0;

}


// break home button func -> in the HOME button work func, where we count from a sync-surpressend first press of HOME button, this is used in external sources 
// 	to break counting of time passing. This way, HOME button press sync can be avoided, and double tap of HOME button can be translated into a POWER OFF even instead.
//	If this value is set to 1, counting will break, and work func will exit without calling INPUT device HOME sync.
//	meanwhile the other func_trigger call will still face LOCK locking, and based on the locking=true will start a Power off.
static int break_home_button_func_work = 1;

// time_count_done_in_home_button_func_work -> represents if the time counting in the HOME button work func is over, meaning that the next HOME button press in func_trigger code
//	shouldn't be interpreted as a double tap instead 
// 	just exit the func trigger call without Power off and leaving normal HOME button sync to work in the home_button work func after the time count
static int time_count_done_in_home_button_func_work = 0;

// job_done_in_home_button_func_work -> represents if we arrived inside the home button work func at the counting of time, without interruption (break_home_button still 0), thus it can be set to 1,
//	HOME button 1 sync will be done in work, and it's also signalling that when the FP device sends release event, in the filter code, HOME button 0 sync can be done.
//	The trigger func will set it to 0 always, so it shows the job was interrupted, which is important when the release of the button is done after
//		trigger job found that the LOCK is not holding anymore, and does nothing, in which case the filter call should send HOME - 0 sync to input device.
static int job_done_in_home_button_func_work = 0;

// signals if fingerprint PRESS was registered, so we can track that no multiple releases happen from FP device
static int fingerprint_pressed = 0;

// signals when the powering down of screen happens while FP is still being pressed, so filter won't turn screen on, when the button is released based on this value.
static int powering_down_with_fingerprint_still_pressed = 0;


// minimum doubletap wait latency will be: (BASE_VALUE + PERIOD) * FUNC_CYLCE_DUR -> minimum is right now (9+0) * 9 = 81msec
#define DT_WAIT_PERIOD_MAX 9
#define DT_WAIT_PERIOD_BASE_VALUE 12
#define DT_WAIT_PERIOD_DEFAULT 2
static int doubletap_wait_period = DT_WAIT_PERIOD_DEFAULT;
static int get_doubletap_wait_period(void) {
	return uci_get_user_property_int_mm("fp_doubletap_wait_period", doubletap_wait_period, 0, 9);
}

/* Home button work func 
	will start with trying to lock worklock
	then use vibrator to signal button press 'imitation'
	Will set break_home_button_func_work to 0, as we just started, and interruptions are signalled through this integer
	While break is not done, it will count the maximum time that is acceptable between two BUTTON presses whchi is interpreted as double press
	- If it's exiting due to Interruption (break_home_button_func_work called from another func_trigger call)
	    it won't do anything just release lock and return. The trigger call will then power down screen, as this counts as double tap
	- If it exited with counting done (break == 0) it will sync a HOME = 1 event to itself
	    - and it will signal job_done_in_home_button_func_work = 1, so when filter func receives Key released, it can Sync a HOME = 0 to input device,
		or set do_home_buttons_too -> 1, so the hom button func work job will itself send the HOME = 0 synced before exiting
*/
static void fpf_home_button_func(struct work_struct * fpf_presspwr_work) {
	int count_cycles = 0;
	if (!mutex_trylock(&fpfuncworklock)) {
		return;
	}
	break_home_button_func_work = 0;
	time_count_done_in_home_button_func_work = 0;
	fpf_vib();
	while (!break_home_button_func_work) {
		count_cycles++;
		if (count_cycles > (DT_WAIT_PERIOD_BASE_VALUE + get_doubletap_wait_period())) {
			break;
		}
		msleep(FUNC_CYCLE_DUR);
		pr_debug("fpf %s counting in cycle before KEY_HOME 1 synced: %d / %d cycles \n",__func__, count_cycles, DT_WAIT_PERIOD_BASE_VALUE+get_doubletap_wait_period());
	}
	time_count_done_in_home_button_func_work = 1;
	if (break_home_button_func_work == 0) {
		job_done_in_home_button_func_work = 1;
		pr_info("fpf %s home 1 \n",__func__);
		input_event(fpf_pwrdev, EV_KEY, KEY_HOME, 1);
		input_event(fpf_pwrdev, EV_SYN, 0, 0);
		msleep(1);
		if (do_home_button_off_too_in_work_func) {
			pr_info("fpf %s home 0 \n",__func__);
			input_event(fpf_pwrdev, EV_KEY, KEY_HOME, 0);
			input_event(fpf_pwrdev, EV_SYN, 0, 0);
			do_home_button_off_too_in_work_func = 0;
			msleep(1);
//			fpf_vib();
		}
	} 
	mutex_unlock(&fpfuncworklock);
	pr_info("fpf %s mutex unlocked \n",__func__);
	return;
}
static DECLARE_WORK(fpf_home_button_func_work, fpf_home_button_func);


/* fpf home button func trigger */
static void fpf_home_button_func_trigger(void) {
	pr_info("fpf %s time_count_done_in_home_button_func_work %d job_done_in_home_button_func_work %d\n",__func__, time_count_done_in_home_button_func_work, job_done_in_home_button_func_work);
	job_done_in_home_button_func_work = 0;
	break_home_button_func_work = 1;
	if (mutex_is_locked(&fpfuncworklock)) {
		// mutex in hold, this means the HOME button was pressed again in a short time...
		pr_info("fpf %s is locked, checkin %d time_count_done_in_home_button_func_work...", __func__, time_count_done_in_home_button_func_work);
		if (!time_count_done_in_home_button_func_work) { // and we still counting the cycles in the job, so double tap poweroff can be done...
			// double home:
			pr_info("fpf double tap home, power off\n");
			if (fingerprint_pressed == 1) { // there was no release of the fingerprint button, so go screen off with signalling that here...
				powering_down_with_fingerprint_still_pressed = 1;
			} else { 
				powering_down_with_fingerprint_still_pressed = 0; 
			}
			fpf_pwrtrigger(1,__func__);
			do_home_button_off_too_in_work_func = 0;
		}
                return;
	}
	schedule_work(&fpf_home_button_func_work);
        return;
}

extern void register_input_event(void);
void register_fp_wake(void) {
	pr_info("%s fpf fp wake registered\n",__func__);
	register_input_event();
}
EXPORT_SYMBOL(register_fp_wake);
void register_fp_irq(void) {
	pr_info("%s fpf fp tap irq registered\n",__func__);
	register_input_event();
}
EXPORT_SYMBOL(register_fp_irq);
/*
    filter will work on FP card events.
    if screen is not on it will work on powering it on when needed (except when Button released start (button press) was started while screen was still on: powering_down_with_fingerprint_still_pressed = 1)
    Otherwise if
	- pressed (value > 0)
		it will call home button trigger job, to handle single fp button presses or double taps.
	- if released:
		if home button work job is done already, finish with syncing HOME release to input device
		if home button work job is still running, set 'do_home_buttons_off_too' to 1, so job will sync HOME release as well
*/
static bool fpf_input_filter(struct input_handle *handle,
                                    unsigned int type, unsigned int code,
                                    int value)
{
	if (type != EV_KEY)
		return false;

	register_input_event();

	// if it's not on, don't filter anything...
	if (get_fpf_switch() == 0) return false;


	if (code == KEY_WAKEUP) {
		pr_debug("fpf - wakeup %d %d \n",code,value);


	if (get_fpf_switch() == 2) {
	//standalone kernel mode. double tap means switch off
	if (value > 0) {
		if (!screen_on) {
			return false; // don't filter so pin appears
		} else {
			fingerprint_pressed = 1;
			pr_info("fpf %s starting trigger \n",__func__);
			fpf_home_button_func_trigger();
		}
		return true;
	} else {
		if (fingerprint_pressed) {
			if (!screen_on) {
				if (!powering_down_with_fingerprint_still_pressed) {
					return false; // don't filter so pin appears
				} else {
					// fingerprint release happens after a screen off that started AFTER the fingerprint was pressed. So do not wake the screen
					powering_down_with_fingerprint_still_pressed = 0;
					return false;
				}
			} else {
				// screen is on...
				// release the fingerprint_pressed variable...
				fingerprint_pressed = 0;
				// if job was all finished inside the work func, we need to call the HOME = 0 release event here, as we couldn't signal to the work to do it on it's own
				if (job_done_in_home_button_func_work) {
						pr_info("fpf %s do key_home 0 sync as job was done, but without the possible signalling for HOME 0\n",__func__);
						input_report_key(fpf_pwrdev, KEY_HOME, 0);
						input_sync(fpf_pwrdev);
				} else {
				// job is not yet finished in home button func work, let's signal it, to do the home button = 0 sync as well
					if (screen_on) {
						do_home_button_off_too_in_work_func = 1;
					} else {
						return false;
					}
				}
			}
			return true;
		} else 
		{ // let event flow through
			return false;
		}
	}
	}
	if (get_fpf_switch() == 1) {
		// simple home button mode, user space handles behavior
		if (!screen_on) {
			return false;
		}
		if (value > 0) {
			fpf_vib();
			input_report_key(fpf_pwrdev, KEY_HOME, 1);
			input_sync(fpf_pwrdev);
		} else {
			input_report_key(fpf_pwrdev, KEY_HOME, 0);
			input_sync(fpf_pwrdev);
		}
	}
	return true;
	}
	return false;
}

// this callback allows registration of FP vibration, and tweaking of length...
int register_fp_vibration(void) {
	return get_unlock_vib_strength();
}
EXPORT_SYMBOL(register_fp_vibration);



// ==================================
// ---------------fingerprint handler
// ==================================

static void fpf_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id fpf_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler fpf_input_handler = {
	.filter		= fpf_input_filter,
	.event		= fpf_input_event,
	.connect	= fpf_input_connect,
	.disconnect	= fpf_input_disconnect,
	.name		= "fpf_inputreq",
	.id_table	= fpf_ids,
};



// ==================================
// ------------- touch screen handler
// ==================================

static unsigned long last_vol_key_1_timestamp = 0;
static unsigned long last_vol_key_2_timestamp = 0;
static unsigned long last_vol_keys_start = 0;

extern void register_double_volume_key_press(int long_press);

static int block_power_key_in_pocket = 0;
int get_block_power_key_in_pocket(void) {
	int proximity = uci_get_sys_property_int_mm("proximity", 0, 0, 1);
	return proximity && uci_get_user_property_int_mm("block_power_key_in_pocket", block_power_key_in_pocket, 0, 1);
}

static bool ts_input_filter(struct input_handle *handle,
                                    unsigned int type, unsigned int code,
                                    int value)
{
#if 1

	register_input_event();

	if (type == EV_KEY) {
		pr_info("%s ts_input key %d %d %d\n",__func__,type,code,value);
		if (code == 116 && !screen_on && get_block_power_key_in_pocket()) {
			pr_info("%s proximity ts_input power key filter\n",__func__);
			return true;
		}
	}

	if (type == EV_KEY && code == KEY_VOLUMEUP && value == 1) {
		last_vol_keys_start = jiffies;
		goto skip_ts;
	}
	if (type == EV_KEY && code == KEY_VOLUMEDOWN && value == 1) {
		last_vol_keys_start = jiffies;
		goto skip_ts;
	}


	if (type == EV_KEY && code == KEY_VOLUMEUP && value == 0) {
		last_vol_key_1_timestamp = jiffies;
		if (last_vol_key_1_timestamp - last_vol_key_2_timestamp < 7 * JIFFY_MUL) {
			unsigned int start_diff = jiffies - last_vol_keys_start;
			register_double_volume_key_press( (start_diff > 50 * JIFFY_MUL) ? ((start_diff > 100 * JIFFY_MUL)?2:1):0 );
		}
		goto skip_ts;
	}
	if (type == EV_KEY && code == KEY_VOLUMEDOWN && value == 0) {
		last_vol_key_2_timestamp = jiffies;
		if (last_vol_key_2_timestamp - last_vol_key_1_timestamp < 7 * JIFFY_MUL) {
			unsigned int start_diff = jiffies - last_vol_keys_start;
			register_double_volume_key_press(start_diff > 50 * JIFFY_MUL ? ((start_diff > 100 * JIFFY_MUL)?2:1):0 );
		}
		goto skip_ts;
	}

#endif
skip_ts:
	if (screen_on) {
	}
	return false;
}

static void ts_input_callback(struct work_struct *unused) {
	return;
}

static void ts_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {
}

static int ts_input_dev_filter(struct input_dev *dev) {
	if (
		strstr(dev->name, "synaptics_dsx") ||
		strstr(dev->name, "max1187x_touchscreen_0") ||
		strstr(dev->name, "cyttsp") ||
		strstr(dev->name, "gpio")
	    ) {
		return 0;
	} else {
		return 1;
	}
}


static int ts_input_connect(struct input_handler *handler,
				struct input_dev *dev, const struct input_device_id *id) {
	struct input_handle *handle;
	int error;

	if (ts_input_dev_filter(dev))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
	if (!handle)
		return -ENOMEM;

	handle->dev = dev;
	handle->handler = handler;
	handle->name = "fpf_ts";


	error = input_register_handle(handle);

	error = input_open_device(handle);

	return 0;

}

static void ts_input_disconnect(struct input_handle *handle)
{
	input_close_device(handle);
	input_unregister_handle(handle);
	kfree(handle);
}


static const struct input_device_id ts_ids[] = {
	{ .driver_info = 1 },
	{ },
};

static struct input_handler ts_input_handler = {
	.filter		= ts_input_filter,
	.event		= ts_input_event,
	.connect	= ts_input_connect,
	.disconnect	= ts_input_disconnect,
	.name		= "ts_inputreq",
	.id_table	= ts_ids,
};

// ------------------------------------------------------


static ssize_t face_down_screen_off_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", face_down_screen_off);
}

static ssize_t face_down_screen_off_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;

	face_down_screen_off = input;
	return count;
}

static DEVICE_ATTR(face_down_screen_off, (S_IWUSR|S_IRUGO),
	face_down_screen_off_show, face_down_screen_off_dump);

static ssize_t block_power_key_in_pocket_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", block_power_key_in_pocket);
}

static ssize_t block_power_key_in_pocket_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;

	block_power_key_in_pocket = input;
	return count;
}

static DEVICE_ATTR(block_power_key_in_pocket, (S_IWUSR|S_IRUGO),
	block_power_key_in_pocket_show, block_power_key_in_pocket_dump);

static ssize_t fpf_dt_wait_period_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", doubletap_wait_period);
}

static ssize_t fpf_dt_wait_period_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > DT_WAIT_PERIOD_MAX)
		input = DT_WAIT_PERIOD_DEFAULT;

	doubletap_wait_period = input;
	return count;
}

static DEVICE_ATTR(fpf_dt_wait_period, (S_IWUSR|S_IRUGO),
	fpf_dt_wait_period_show, fpf_dt_wait_period_dump);


static ssize_t fpf_dt_wait_period_max_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", DT_WAIT_PERIOD_MAX);
}

static ssize_t fpf_dt_wait_period_max_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;
	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;
	return count;
}

static DEVICE_ATTR(fpf_dt_wait_period_max, (S_IWUSR|S_IRUGO),
	fpf_dt_wait_period_max_show, fpf_dt_wait_period_max_dump);


static ssize_t fpf_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", fpf_switch);
}

static ssize_t fpf_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 2)
		input = 0;				

	fpf_switch = input;			
	
	return count;
}

static DEVICE_ATTR(fpf, (S_IWUSR|S_IRUGO),
	fpf_show, fpf_dump);

static ssize_t vib_strength_show(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", vib_strength);
}

static ssize_t vib_strength_dump(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 90)
		input = 20;				

	vib_strength = input;			
	
	return count;
}

static DEVICE_ATTR(vib_strength, (S_IWUSR|S_IRUGO),
	vib_strength_show, vib_strength_dump);

static ssize_t unlock_vib_strength_show(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", unlock_vib_strength);
}

static ssize_t unlock_vib_strength_dump(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 90) 
		input = 20;				

	unlock_vib_strength = input;			
	
	return count;
}

static DEVICE_ATTR(unlock_vib_strength, (S_IWUSR|S_IRUGO),
	unlock_vib_strength_show, unlock_vib_strength_dump);


static ssize_t phone_ring_in_silent_mode_show(struct device *dev,
		 struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", phone_ring_in_silent_mode);
}

static ssize_t phone_ring_in_silent_mode_dump(struct device *dev,
		 struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1) 
		input = 0;

	phone_ring_in_silent_mode = input;
	
	return count;
}

static DEVICE_ATTR(phone_ring_in_silent_mode, (S_IWUSR|S_IRUGO),
	phone_ring_in_silent_mode_show, phone_ring_in_silent_mode_dump);

// -------------------- notification booster
static ssize_t notification_booster_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_notification_booster());
}

static ssize_t notification_booster_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 100)
		input = 0;

	set_notification_booster(input);

	return count;
}

static DEVICE_ATTR(notification_booster, (S_IWUSR|S_IRUGO),
	notification_booster_show, notification_booster_dump);

static ssize_t notification_boost_only_in_pocket_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", get_notification_boost_only_in_pocket());
}

static ssize_t notification_boost_only_in_pocket_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 100)
		input = 0;

	set_notification_boost_only_in_pocket(input);

	return count;
}

static DEVICE_ATTR(notification_boost_only_in_pocket, (S_IWUSR|S_IRUGO),
	notification_boost_only_in_pocket_show, notification_boost_only_in_pocket_dump);
// --------------------------------------------------
// ------------- smart notification timing ------------------------
//smart_trim_inactive_seconds
//smart_stop_inactive_seconds
//smart_hibernate_inactive_seconds
static ssize_t smart_silent_mode_hibernate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", smart_silent_mode_hibernate);
}

static ssize_t smart_silent_mode_hibernate_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;

	smart_silent_mode_hibernate = input;

	return count;
}

static DEVICE_ATTR(smart_silent_mode_hibernate, (S_IWUSR|S_IRUGO),
	smart_silent_mode_hibernate_show, smart_silent_mode_hibernate_dump);


static ssize_t smart_silent_mode_stop_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", smart_silent_mode_stop);
}

static ssize_t smart_silent_mode_stop_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 1)
		input = 0;

	smart_silent_mode_stop = input;

	return count;
}

static DEVICE_ATTR(smart_silent_mode_stop, (S_IWUSR|S_IRUGO),
	smart_silent_mode_stop_show, smart_silent_mode_stop_dump);


static ssize_t smart_trim_inactive_minutes_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", smart_trim_inactive_seconds/60);
}

static ssize_t smart_trim_inactive_minutes_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 2 * 24 * 60)
		input = 0;

	smart_trim_inactive_seconds = input * 60;

	return count;
}

static DEVICE_ATTR(smart_trim_inactive_minutes, (S_IWUSR|S_IRUGO),
	smart_trim_inactive_minutes_show, smart_trim_inactive_minutes_dump);


static ssize_t smart_stop_inactive_minutes_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", smart_stop_inactive_seconds/60);
}

static ssize_t smart_stop_inactive_minutes_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 2 * 24 * 60)
		input = 0;

	smart_stop_inactive_seconds = input * 60;

	return count;
}

static DEVICE_ATTR(smart_stop_inactive_minutes, (S_IWUSR|S_IRUGO),
	smart_stop_inactive_minutes_show, smart_stop_inactive_minutes_dump);


static ssize_t smart_hibernate_inactive_minutes_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", smart_hibernate_inactive_seconds/60);
}

static ssize_t smart_hibernate_inactive_minutes_dump(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long input;

	ret = kstrtoul(buf, 0, &input);
	if (ret < 0)
		return ret;

	if (input < 0 || input > 2 * 24 * 60)
		input = 0;

	smart_hibernate_inactive_seconds = input * 60;

	return count;
}

static DEVICE_ATTR(smart_hibernate_inactive_minutes, (S_IWUSR|S_IRUGO),
	smart_hibernate_inactive_minutes_show, smart_hibernate_inactive_minutes_dump);


static struct kobject *fpf_kobj;




#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;

	if (evdata && evdata->data && event == FB_EVENT_AOD_MODE) {
		int aod_mode = *(int *)evdata->data;
		if (aod_mode == FB_AOD_IDLE || aod_mode == FB_AOD_PARTIAL_ON) {
			pr_info("%s fpf off intent... \n", __func__);
			if (screen_on) {
				// screen off
				screen_on = 0;
				last_screen_off_seconds = get_global_seconds();
				last_screen_lock_check_was_false = 0;
				pr_info("fpf kad screen off\n");
			}
		} else {
			pr_info("%s fpf on intent... \n", __func__);
			if (!screen_on) {
				// screen on
				screen_on = 1;
			}
		}
	}

    return 0;
}
#endif

static int __init fpf_init(void)
{
	int rc = 0;
	pr_info("fpf - init\n");

	fpf_pwrdev = input_allocate_device();
	if (!fpf_pwrdev) {
		pr_err("Failed to allocate fpf_pwrdev\n");
		goto err_alloc_dev;
	}

	input_set_capability(fpf_pwrdev, EV_KEY, KEY_POWER);
	input_set_capability(fpf_pwrdev, EV_KEY, KEY_HOME);
	
	set_bit(EV_KEY, fpf_pwrdev->evbit);
	set_bit(KEY_HOME, fpf_pwrdev->keybit);


	fpf_pwrdev->name = "qwerty";
	fpf_pwrdev->phys = "qwerty/input0";

	rc = input_register_device(fpf_pwrdev);
	if (rc) {
		pr_err("%s: input_register_device err=%d\n", __func__, rc);
		goto err_input_dev;
	}

	// fpf handler
	fpf_input_wq = create_workqueue("fpf_iwq");
	if (!fpf_input_wq) {
		pr_err("%s: Failed to create workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&fpf_input_work, fpf_input_callback);

	rc = input_register_handler(&fpf_input_handler);
	if (rc)
		pr_err("%s: Failed to register fpf_input_handler\n", __func__);

	// ts handler
	ts_input_wq = create_workqueue("ts_iwq");
	if (!ts_input_wq) {
		pr_err("%s: Failed to create workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&ts_input_work, ts_input_callback);

	rc = input_register_handler(&ts_input_handler);
	if (rc)
		pr_err("%s: Failed to register ts_input_handler\n", __func__);


#ifdef CONFIG_FB
	fb_notifier = kzalloc(sizeof(struct notifier_block), GFP_KERNEL);;
	fb_notifier->notifier_call = fb_notifier_callback;
	fb_register_client(fb_notifier);
#endif

	fpf_kobj = kobject_create_and_add("fpf", NULL) ;
	if (fpf_kobj == NULL) {
		pr_warn("%s: fpf_kobj failed\n", __func__);
	}

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_notification_booster.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for notif booster\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_notification_boost_only_in_pocket.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for notif boost only in pocket\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_block_power_key_in_pocket.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for block_power_key_in_pocket\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_face_down_screen_off.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for face_down_screen_off\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf_dt_wait_period.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf_dt_wait_period\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf_dt_wait_period_max.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf_dt_wait_period_max\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_vib_strength.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for vib_strength\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_unlock_vib_strength.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for unlock_vib_strength\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_phone_ring_in_silent_mode.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for phone_ring_in_silent_mode\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_smart_trim_inactive_minutes.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for smart_trim_inactive_minutes\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_smart_silent_mode_hibernate.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for smart_silent_mode_hibernate\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_smart_silent_mode_stop.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for smart_silent_mode_stop\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_smart_stop_inactive_minutes.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for smart_stop_inactive_minutes\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_smart_hibernate_inactive_minutes.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for smart_hibernate_inactive_minutes\n", __func__);

	alarm_init(&vibrate_rtc, ALARM_REALTIME,
		vibrate_rtc_callback);
	uci_add_sys_listener(fpf_uci_sys_listener);
	smart_last_user_activity_time = get_global_seconds();
err_input_dev:
	input_free_device(fpf_pwrdev);

err_alloc_dev:
	pr_info("%s fpf done\n", __func__);

	return 0;
}

static void __exit fpf_exit(void)
{
	kobject_del(fpf_kobj);
	input_unregister_handler(&fpf_input_handler);
	destroy_workqueue(fpf_input_wq);
	input_unregister_device(fpf_pwrdev);
	input_free_device(fpf_pwrdev);

	return;
}

module_init(fpf_init);
module_exit(fpf_exit);
