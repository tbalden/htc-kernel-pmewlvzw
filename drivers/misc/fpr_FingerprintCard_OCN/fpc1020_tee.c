/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, enabling and disabling of platform
 * clocks, controlling GPIOs such as SPI chip select, sensor reset line, sensor
 * IRQ line, MISO and MOSI lines.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node. This makes it possible for a user
 * space process to poll the input node and receive IRQ events easily. Usually
 * this node is available under /dev/input/eventX where 'X' is a number given by
 * the event system. A user space process will need to traverse all the event
 * nodes and ask for its parent's name (through EVIOCGNAME) which should match
 * the value in device tree named input-device-name.
 *
 * This driver will NOT send any SPI commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <soc/qcom/scm.h>
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING
#include <linux/power/htc_battery.h>
#endif

#define FPC1020_RESET_LOW_US 1000
#define FPC1020_RESET_HIGH1_US 100
#define FPC1020_RESET_HIGH2_US 1250
#define FPC_TTW_HOLD_TIME 1000

#define CONFIG_HTC_ID_PIN 1
#define CONFIG_HTC_HAL_WRITE_TIME 1

static const char * const pctl_names[] = {
	"fpc1020_id_init",
	"fpc1020_id_active",
	"fpc1020_id_sleep",
	"fpc1020_irq_active",
	"fpc1020_reset_active",
	"fpc1020_reset_reset"
};

struct fpc1020_data {
	struct device *dev;
	int irq_gpio;
	int rst_gpio;
	int irq_num;
	struct mutex lock;
	bool prepared;
	bool wakeup_enabled;

	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

	struct wake_lock ttw_wl;
#ifdef CONFIG_HTC_ID_PIN
        int id_gpio;
        int fp_source;
#endif
#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
        int irq_count;
#endif
#ifdef CONFIG_FPC_HTC_IRQ_LOGGING
        int irq_logging_enable;
#endif


#ifdef CONFIG_FPC_HTC_ADD_INPUT_DEVICE
	int event_type;
	int event_code;
	struct input_dev *idev;
#endif //CONFIG_FPC_HTC_ADD_INPUT_DEVICE
};

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);
	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;
	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request gpio %d\n", *gpio);
		return rc;
	}
	return rc;
}

/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;
	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_info(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}
	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);
exit:
	return rc;
}

/* ----------------------------------------------------------------------------------- */
/* Add this attribute for hal_footprint */
static char hal_footprint_str[128] = {0};
static ssize_t hal_footprint_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct timespec ts;
	struct rtc_time tm;

	//get current time
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);

	scnprintf(hal_footprint_str, sizeof(hal_footprint_str), "fpc footprint:%s at (%02d-%02d %02d:%02d:%02d.%03lu)", buf,
		tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec/1000000);

	pr_info("%s\n", hal_footprint_str);

	return count;
}
static DEVICE_ATTR(hal_footprint, S_IRUGO, NULL, hal_footprint_set);
/* ----------------------------------------------------------------------------------- */

#ifdef CONFIG_HTC_HAL_WRITE_TIME
static ssize_t hal_meature_time_set(struct device *device,
	struct device_attribute *attribute, const char *buffer, size_t count)
{
	static char time_str[128] = {0};

	snprintf(time_str, sizeof(time_str), "hal_time = %s",buffer);
	printk("[fp][HAL] %s\n",time_str);

	return count;
}
static DEVICE_ATTR(hal_meature_time, S_IWUSR, NULL, hal_meature_time_set);
#endif //CONFIG_HTC_HAL_WRITE_TIME

/* ----------------------------------------------------------------------------------- */
/* Add this attribute for disable charging while captiruing fp image */

#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING
static ssize_t fp_disable_charging_set(struct device *device,
	struct device_attribute *attribute, const char *buf, size_t count)
{
	if (!strncmp(buf, "enable", strlen("enable"))) {
		pr_info("[fp] %s:%s\n", __func__, buf);
		htc_battery_charger_switch_internal(ENABLE_PWRSRC_FINGERPRINT);
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		pr_info("[fp] %s:%s\n", __func__, buf);
		htc_battery_charger_switch_internal(DISABLE_PWRSRC_FINGERPRINT);
	} else {
		pr_err("[fp] Wrong Parameter!!:%s\n", buf);
		return -EINVAL;
	}
	return count;
}
static DEVICE_ATTR(fp_disable_charge, S_IWUSR, NULL, fp_disable_charging_set);
#endif //CONFIG_FPC_HTC_DISABLE_CHARGING
/* ----------------------------------------------------------------------------------- */

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	struct device *dev = fpc1020->dev;

	int rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;

	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;

	usleep_range(FPC1020_RESET_LOW_US, FPC1020_RESET_LOW_US + 100);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;

	usleep_range(FPC1020_RESET_HIGH1_US, FPC1020_RESET_HIGH1_US + 100);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d\n", irq_gpio);

exit:
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset")))
		rc = hw_reset(fpc1020);
	else
		return -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

#ifdef CONFIG_HTC_ID_PIN
static ssize_t id_pin_get(struct device *device,
	struct device_attribute *attribute, char *buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	pr_info("fpc fpc1020->fp_source = %d\n", fpc1020->fp_source);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc1020->fp_source);
}
static DEVICE_ATTR(id_pin, S_IRUSR, id_pin_get, NULL);
#endif

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable"))) {
		fpc1020->wakeup_enabled = true;
	} else if (!strncmp(buf, "disable", strlen("disable"))) {
		fpc1020->wakeup_enabled = false;
	} else
		return -EINVAL;

	return count;
}
static DEVICE_ATTR(wakeup_enable, S_IWUSR, NULL, wakeup_enable_set);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *device,
			     struct device_attribute *attribute,
			     char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	int irq = gpio_get_value(fpc1020->irq_gpio);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", irq);
}


/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *device,
			     struct device_attribute *attribute,
			     const char *buffer, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	dev_dbg(fpc1020->dev, "%s\n", __func__);
	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
/**
 * sysf node to check the interrupt count of the sensor, the interrupt
 */
static ssize_t irq_count_get(struct device *device,
	struct device_attribute *attribute, char* buffer)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
	return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc1020->irq_count);
}
static DEVICE_ATTR(irq_count, S_IRUSR, irq_count_get, NULL);
#endif //CONFIG_FPC_HTC_RECORD_IRQ_COUNT

#ifdef CONFIG_FPC_HTC_IRQ_LOGGING
static ssize_t irq_logging_enable_set(struct device *dev,
    struct device_attribute *attr, const char *buf, size_t count)
{
    struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

    if (!strncmp(buf, "1", strlen("1"))) {
        fpc1020->irq_logging_enable = 1;
    } else if (!strncmp(buf, "0", strlen("0"))) {
        fpc1020->irq_logging_enable = 0;
    } else
        return -EINVAL;

    dev_info(fpc1020->dev, "%s irq_logging_enable:%d\n", __func__, fpc1020->irq_logging_enable);

    return count;
}

static ssize_t irq_logging_enable_get(struct device *device,
    struct device_attribute *attribute, char* buffer)
{
    struct fpc1020_data *fpc1020 = dev_get_drvdata(device);
    return scnprintf(buffer, PAGE_SIZE, "%i\n", fpc1020->irq_logging_enable);
}

static DEVICE_ATTR(irq_logging_enable, S_IRUSR | S_IWUSR, irq_logging_enable_get, irq_logging_enable_set);
#endif //CONFIG_FPC_HTC_IRQ_LOGGING


/**
 * sysfs node for select pinctl
 */
static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc = select_pin_ctl(fpc1020, buf);
	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static struct attribute *attributes[] = {
#ifdef CONFIG_FPC_HTC_DISABLE_CHARGING
        &dev_attr_fp_disable_charge.attr,
#endif
	&dev_attr_hal_footprint.attr,
#ifdef CONFIG_HTC_HAL_WRITE_TIME
        &dev_attr_hal_meature_time.attr,
#endif
	&dev_attr_hw_reset.attr,
#ifdef CONFIG_HTC_ID_PIN
	&dev_attr_id_pin.attr,
#endif
	&dev_attr_irq.attr,
#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
        &dev_attr_irq_count.attr,
#endif //CONFIG_FPC_HTC_RECORD_IRQ_COUNT
#ifdef CONFIG_FPC_HTC_IRQ_LOGGING
	&dev_attr_irq_logging_enable.attr,
#endif //CONFIG_FPC_HTC_IRQ_LOGGING
	&dev_attr_wakeup_enable.attr,
	&dev_attr_pinctl_set.attr,
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

#ifdef CONFIG_HTC_ID_PIN
static int id_pin_detect(struct fpc1020_data *fp)
{
	struct device *dev = fp->dev;
	int rc;
	int id_pin_1st, id_pin_2nd = 0;

	dev_info(dev, "fpc %s +++\n", __func__);
	fp->fp_source = 1;

	rc = select_pin_ctl(fp, "fpc1020_id_init");
	if (rc) {
		dev_err(dev, "fpc %s, select_pin_ctrl(fp, fpc1020_id_init) failed !", __func__);
		goto error;
	}

	id_pin_1st = gpio_get_value(fp->id_gpio);
	dev_info(dev, "[FP] detect first time, id pin = %s\n", (id_pin_1st ? "High" : "Low"));

	msleep(50);

	rc = select_pin_ctl(fp, "fpc1020_id_active");
	if (rc) {
		dev_err(dev, "fpc %s, select_pin_ctrl(fp, fpc1020_id_active) failed !", __func__);
		goto error;
	}

	id_pin_2nd = gpio_get_value(fp->id_gpio);
	dev_info(dev, "[FP] detect second time, id pin = %s\n", (id_pin_2nd ? "High" : "Low"));

	if (id_pin_1st == 0 && id_pin_2nd == 1) {
		fp->fp_source = 0;
		dev_info(dev, "fpc Fingerprint source = FPC+CT\n");
	} else if (id_pin_1st == 0 && id_pin_2nd == 0) {
		fp->fp_source = 1;
		dev_info(dev, "fpc Fingerprint source = IDEX+O-Film\n");
	} else if (id_pin_1st == 1 && id_pin_2nd == 1) {
		fp->fp_source = 2;
		dev_info(dev, "fpc Fingerprint source = Third Source\n");
	} else {
		dev_err(dev, "fpc Fingerprint source = abnormal GPIO\n");
		goto error;
	}

	dev_info(dev, "fpc %s ---\n", __func__);
	rc = select_pin_ctl(fp, "fpc1020_id_sleep");
	if (rc) {
		dev_err(dev, "fpc %s, select_pin_ctrl(fp, fpc1020_id_sleep) failed !", __func__);
		goto error;
	}

	return fp->fp_source;

error:
	return rc;
}
#endif

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;
	dev_dbg(fpc1020->dev, "%s\n", __func__);

	if (fpc1020->wakeup_enabled) {
		wake_lock_timeout(&fpc1020->ttw_wl,
					msecs_to_jiffies(FPC_TTW_HOLD_TIME));
	}

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
        fpc1020->irq_count++;
#endif

#ifdef CONFIG_FPC_HTC_IRQ_LOGGING
	if (fpc1020->irq_logging_enable)
		pr_info("%s %d %d\n", __func__, fpc1020->wakeup_enabled, fpc1020->irq_count);
#endif

	return IRQ_HANDLED;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc = 0;
	int irqf;
	struct device_node *np = dev->of_node;
	size_t i;
#ifdef CONFIG_FPC_HTC_ADD_INPUT_DEVICE
	struct input_dev *input_dev;
#endif //CONFIG_FPC_HTC_ADD_INPUT_DEVICE

	struct fpc1020_data *fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020),
			GFP_KERNEL);
	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	dev_set_drvdata(dev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}

	rc = fpc1020_request_named_gpio(fpc1020, "fpc,irq-gpio",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;

	rc = gpio_direction_input(fpc1020->irq_gpio);

	if (rc) {
		dev_err(fpc1020->dev,
			"gpio_direction_input (irq) failed.\n");
		goto exit;
	}

#ifdef CONFIG_HTC_ID_PIN
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,id-gpio",
			&fpc1020->id_gpio);
	if (rc)
		goto exit;
#endif

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
	}

#ifdef CONFIG_HTC_ID_PIN
	id_pin_detect(fpc1020);
#endif

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;

	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;

#ifdef CONFIG_FPC_HTC_ADD_INPUT_DEVICE
	input_dev = devm_input_allocate_device(dev);
	if (!input_dev) {
		dev_err(dev, "%s failed to allocate input device\n", __func__);
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->idev = input_dev;

	input_dev->name = "fpc1020";

	fpc1020->event_type = EV_MSC;
	fpc1020->event_code = MSC_SCAN;

	input_set_capability(fpc1020->idev, EV_MSC, MSC_SCAN);

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(KEY_WAKEUP, input_dev->keybit);
	rc = input_register_device(input_dev);
	if (rc) {
		dev_err(dev, "%s failed to register input device\n", __func__);
		goto exit;
	}
#endif //CONFIG_FPC_HTC_ADD_INPUT_DEVICE

	fpc1020->wakeup_enabled = false;

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}
	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	wake_lock_init(&fpc1020->ttw_wl, WAKE_LOCK_SUSPEND, "fpc_ttw_wl");

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

#ifdef CONFIG_FPC_HTC_RECORD_IRQ_COUNT
        fpc1020->irq_count = 0;
#endif

#ifdef CONFIG_FPC_HTC_IRQ_LOGGING
	fpc1020->irq_logging_enable = 0;
#endif

	dev_info(dev, "%s: ok\n", __func__);
exit:
	return rc;
}

static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name		= "fpc1020",
		.owner		= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
	},
	.probe = fpc1020_probe,
};
module_platform_driver(fpc1020_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_AUTHOR("Martin Trulsson <martin.trulsson@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");

