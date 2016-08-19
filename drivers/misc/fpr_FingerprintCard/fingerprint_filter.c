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

#define DRIVER_AUTHOR "illes pal <illespal@gmail.com>"
#define DRIVER_DESCRIPTION "fingerprint_filter driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#define fpf_PWRKEY_DUR          60
#define FUNC_CYCLE_DUR          9
#define VIB_STRENGTH		20

static int fpf_switch = 2;
static struct input_dev * fpf_pwrdev;
static DEFINE_MUTEX(pwrkeyworklock);
static DEFINE_MUTEX(fpfuncworklock);
static struct workqueue_struct *fpf_input_wq;
static struct work_struct fpf_input_work;
extern struct vib_trigger *vib_trigger;
static int vib_strength = VIB_STRENGTH;

#ifdef CONFIG_FB
	static int screen_on = 1;
	struct notifier_block *fb_notifier;
#endif

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
	vib_trigger_event(vib_trigger, vib_strength);
}

/* PowerKey trigger */
static void fpf_pwrtrigger(int vibration) {
	if (vibration) fpf_vib();
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


// minimum doubletap wait latency will be: (BASE_VALUE + PERIOD) * FUNC_CYLCE_DUR -> minimum is right now (8+0) * 9 = 72msec
#define DT_WAIT_PERIOD_MAX 9
#define DT_WAIT_PERIOD_BASE_VALUE 8
#define DT_WAIT_PERIOD_DEFAULT 2
static int doubletap_wait_period = DT_WAIT_PERIOD_DEFAULT;

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
		if (count_cycles > (DT_WAIT_PERIOD_BASE_VALUE + doubletap_wait_period)) {
			break;
		}
		msleep(FUNC_CYCLE_DUR);
		pr_debug("fpf %s counting in cycle before KEY_HOME 1 synced: %d / %d cycles \n",__func__, count_cycles, DT_WAIT_PERIOD_BASE_VALUE+doubletap_wait_period);
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
			fpf_pwrtrigger(1);
			do_home_button_off_too_in_work_func = 0;
		}
                return;
	}
	schedule_work(&fpf_home_button_func_work);
        return;
}

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
	// if it's not on, don't filter anything...
	if (fpf_switch == 0) return false;

	if (type != EV_KEY)
		return false;

	if (code == KEY_WAKEUP) {
		pr_debug("fpf - wakeup %d %d \n",code,value);
	}            

	if (fpf_switch == 2) {
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
	if (fpf_switch == 1) {
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

static struct kobject *fpf_kobj;




#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    // catch early events as well, as this helps a lot correct functioning knowing when screen is almost off/on, preventing many problems 
    // interpreting still screen ON while it's almost off and vica versa
    if (evdata && evdata->data && event == FB_EARLY_EVENT_BLANK && fpf_pwrdev) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		screen_on = 1;
		pr_info("fpf screen on -early\n");
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		screen_on = 0;
		pr_info("fpf screen off -early\n");
            break;
        }
    }
    if (evdata && evdata->data && event == FB_EVENT_BLANK && fpf_pwrdev) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		screen_on = 1;
		pr_info("fpf screen on\n");
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		screen_on = 0;
		pr_info("fpf screen off\n");
            break;
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

	fpf_input_wq = create_workqueue("fpf_iwq");
	if (!fpf_input_wq) {
		pr_err("%s: Failed to create workqueue\n", __func__);
		return -EFAULT;
	}
	INIT_WORK(&fpf_input_work, fpf_input_callback);

	rc = input_register_handler(&fpf_input_handler);
	if (rc)
		pr_err("%s: Failed to register fpf_input_handler\n", __func__);


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

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf_dt_wait_period.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf_dt_wait_period\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_fpf_dt_wait_period_max.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for fpf_dt_wait_period_max\n", __func__);

	rc = sysfs_create_file(fpf_kobj, &dev_attr_vib_strength.attr);
	if (rc)
		pr_err("%s: sysfs_create_file failed for vib_strength\n", __func__);

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
