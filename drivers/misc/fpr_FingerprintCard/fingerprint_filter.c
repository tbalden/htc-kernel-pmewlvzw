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

#define DRIVER_AUTHOR "illes pal aka tbalden illespal@gmail.com"
#define DRIVER_DESCRIPTION "fingerprint_filter driver"
#define DRIVER_VERSION "1.0"

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");

#define fpf_PWRKEY_DUR          60
#define FUNC_CYCLE_DUR          10
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

/* reset on finger release */
static void fpf_reset(void) {
}

/* fpf main function */
//static void detect_fpf()
//{
//}


static void fpf_input_callback(struct work_struct *unused) {

	if (1==0) {
		fpf_pwrtrigger(0);
	}

	return;
}

static void fpf_input_event(struct input_handle *handle, unsigned int type,
				unsigned int code, int value) {




	if (code == KEY_WAKEUP) {
		pr_err("fpf - wakeup event %d %d \n",code,value);
		return;
	}

	if (code == ABS_MT_TRACKING_ID && value == -1) {
		fpf_reset();
		return;
	}

	if (code == ABS_MT_POSITION_X) {
	}

	if (code == ABS_MT_POSITION_Y) {
	}

	if (1==1) {
		queue_work_on(0, fpf_input_wq, &fpf_input_work);
	}
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


static int break_home_button_func = 1;
static int do_home_button_off_too = 0;
static int over_cycles = 0;
static int job_done = 0;
static int first_tap_cycle = 1;


/* Home button work func */
static void fpf_home_button_func(struct work_struct * fpf_presspwr_work) {
	int count_cycles = 0;
	if (!mutex_trylock(&fpfuncworklock)) {
		return;
	}
	break_home_button_func = 0;
	over_cycles = 0;
	while (!break_home_button_func) {
		count_cycles++;
		if (count_cycles>15) {
			break;
		}
		msleep(FUNC_CYCLE_DUR);
		pr_err("fpf %s counting in cycle before KEY_HOME 1 synced: %d / 30 cycles \n",__func__, count_cycles);
	}
	over_cycles = 1;
	if (break_home_button_func == 0) {
		job_done = 1;
		pr_err("fpf %s home 1 \n",__func__);
		input_event(fpf_pwrdev, EV_KEY, KEY_HOME, 1);
		input_event(fpf_pwrdev, EV_SYN, 0, 0);
		msleep(1);
		if (do_home_button_off_too) {
			pr_err("fpf %s home 0 \n",__func__);
			input_event(fpf_pwrdev, EV_KEY, KEY_HOME, 0);
			input_event(fpf_pwrdev, EV_SYN, 0, 0);
			do_home_button_off_too = 0;
			msleep(1);
			fpf_vib();
		}
		first_tap_cycle = 1; // waiting was not interrupted by a new button press, back to first tap cycle
	} else {
		first_tap_cycle = 0; // waiting was interrupted, so waiting for second tap
	}
	mutex_unlock(&fpfuncworklock);
	pr_err("fpf %s mutex unlocked \n",__func__);
	return;
}
static DECLARE_WORK(fpf_home_button_func_work, fpf_home_button_func);

/* PowerKey trigger */
static void fpf_home_button_func_trigger(void) {
	pr_err("fpf %s over_cycles %d job_done %d first_tap_cylce %d \n",__func__, over_cycles, job_done, first_tap_cycle);
	job_done = 0;
	break_home_button_func = 1;
	if (mutex_is_locked(&fpfuncworklock)) {
		// mutex in hold, this means the HOME button was pressed again in a short time...
		pr_err("fpf %s is locked, checkin %d over_cycles...", __func__, over_cycles);
		if (!over_cycles) { // and we still counting the cycles in the job, so double tap poweroff can be done...
			// double home:
			pr_err("fpf double tap home, power off\n");
			first_tap_cycle = 1;
			do_home_button_off_too = 0;
			fpf_pwrtrigger(1);
		}
                return;
	}
	schedule_work(&fpf_home_button_func_work);
        return;
}

static int started_home_button = 0;

static bool fpf_input_filter(struct input_handle *handle,
                                    unsigned int type, unsigned int code,
                                    int value)
{
	if (type != EV_KEY)
		return false;
	if (code == KEY_WAKEUP) {
		pr_err("fpf - wakeup %d %d \n",code,value);
	}            

	if (value > 0) {
		if (!screen_on) {
			fpf_pwrtrigger(0);
		} else {
			started_home_button = 1;
			pr_err("fpf %s starting trigger, first tap cycle: %d \n",__func__, first_tap_cycle);
			fpf_home_button_func_trigger();
		}
		return true;
	} else {
		if (started_home_button) {
			if (!screen_on) {
				fpf_pwrtrigger(1);
			} else {
				started_home_button = 0;
				if (job_done) {
						pr_err("fpf %s do home 0 as job was done, not first tap ending\n",__func__);
						input_report_key(fpf_pwrdev, KEY_HOME, 0);
						input_sync(fpf_pwrdev);
				} else {
					// only set need fro HOME 0, when screen is not already beings shut down, otherwise a HOME->1 will trigger a HOME->0 too in work, and preventling long home working
					do_home_button_off_too = 1;
				}
			}
			return true;
		} else {
			// TODO
		}
	}
        return false;
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

	if (input < 0 || input > 3)
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

    pr_err("%s\n", __func__);
    if (evdata && evdata->data && event == FB_EVENT_BLANK && fpf_pwrdev) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
		screen_on = 1;
		pr_err("fpf screen on\n");
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
		screen_on = 0;
		pr_err("fpf screen off\n");
            break;
        }
    }
    return 0;
}
#endif

static int __init fpf_init(void)
{
	int rc = 0;
	pr_err("fpf - init\n");

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
