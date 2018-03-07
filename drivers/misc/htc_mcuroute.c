#define pr_fmt(fmt)	"%s: " fmt, __func__

#include <linux/fb.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/rculist.h>
#include <linux/poll.h>
#include <linux/ctype.h>


static DEFINE_SEMAPHORE(mcuroute_sem);
static int mcuroute_locked;
static int master_active;

/**
 * mcuroute_lock - lock the mcuroute operation for exclusive use.
 *
 * Can sleep, returns nothing.
 */
void mcuroute_lock(void)
{
	BUG_ON(in_interrupt());

	down(&mcuroute_sem);
	mcuroute_locked = 1;
}
EXPORT_SYMBOL(mcuroute_lock);

/**
 * mcuroute_trylock - try to lock the mcuroute operation for exclusive use.
 *
 * returns 1 on success, and 0 on failure to acquire the lock.
 */
int mcuroute_trylock(void)
{
	if (down_trylock(&mcuroute_sem))
		return 0;
	mcuroute_locked = 1;
	return 1;
}
EXPORT_SYMBOL(mcuroute_trylock);

/**
 * mcuroute_unlock - unlock the mcuroute operation
 *
 * mcuroute_unlock(); may be called from any context.
 */
void mcuroute_unlock(void)
{
	mcuroute_locked = 0;
	up(&mcuroute_sem);
}
EXPORT_SYMBOL(mcuroute_unlock);

/**
 * mcuroute_get_master_active - get current control path direction
 *
 * mcuroute_get_master_active: must be called while mcuroute_lock hold
 */
int mcuroute_get_master_active(void)
{
	return master_active;
}
EXPORT_SYMBOL(mcuroute_get_master_active);

/**
 * mcuroute_set_master_active - set current control path direction
 *
 * mcuroute_set_master_active: must be called while mcuroute_lock hold
 */
void mcuroute_set_master_active(bool en)
{
	BUG_ON(in_interrupt());

	if (master_active != en) {
		if (en) {
			// TODO: Notify slave to stop or abort current activities on shared interface
			//       before switch to master
		} else {
			// TODO: Notify slave can access shared interface
		}
		master_active = en;
		pr_info("Set masterctive = %d by %pS\n", en, __builtin_return_address(0));
	}
}
EXPORT_SYMBOL(mcuroute_set_master_active);

/**
 * HTC: FIXME
 *      First draft was using AOD mode instead of HW GPIO Switch status
 */
static struct notifier_block fb_notif;

static int fb_notifier_callback(struct notifier_block *self,
				unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int aod_mode;

	/* If we aren't interested in this event, skip it immediately ... */
	if (event != FB_EVENT_AOD_MODE)
		return 0;

	aod_mode = *(int *)evdata->data;
	mcuroute_lock();

	if (aod_mode == FB_AOD_IDLE || aod_mode == FB_AOD_PARTIAL_ON)
		mcuroute_set_master_active(false);
	else
		mcuroute_set_master_active(true);

	mcuroute_unlock();
	return 0;
}

static int __init mcuroute_init(void)
{
        int ret;

	fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&fb_notif);
        if (ret)
                pr_err("%s fb client register failed", __func__);
	master_active = true;

        return ret;
}
module_init(mcuroute_init);

static void __exit mcuroute_cleanup(void)
{
	fb_unregister_client(&fb_notif);
}
module_exit(mcuroute_cleanup);
