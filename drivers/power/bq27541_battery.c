/*
 * Virtual battery driver for batteryless Nexus 7 2013 (flo).
 *
 * The original hardware used a TI BQ27541 fuel gauge. This build is for a
 * board permanently powered from a regulated external VBAT source, so no
 * physical gauge is accessed. The I2C driver name and cable callbacks remain
 * for compatibility with the flo board and SMB345 charger code.
 *
 * Copyright (c) 2012, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/power_supply.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#define BATTERY_POLLING_RATE		60
#define USB_DETECT_CABLE		1
#define USB_SHIFT			0
#define AC_SHIFT			1
#define USB_Cable			((1 << USB_SHIFT) | USB_DETECT_CABLE)
#define USB_AC_Adapter			((1 << AC_SHIFT) | USB_DETECT_CABLE)

#define VIRTUAL_BATTERY_CAPACITY	100
#define VIRTUAL_BATTERY_TEMP		250
#define VIRTUAL_BATTERY_VOLTAGE_UV	4100000
#define VIRTUAL_BATTERY_CURRENT		0

#define BAT_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "%s " format, __func__, ##arg)
#define BAT_ERR(format, arg...)		\
	printk(KERN_ERR format, ##arg)

unsigned bq27541_battery_cable_status;
unsigned bq27541_battery_driver_ready;
int ac_on;
int usb_on;
extern bool wireless_on;
extern void reconfig_AICL(void);

static struct workqueue_struct *bq27541_battery_work_queue;

typedef enum {
	Charger_Type_Battery = 0,
	Charger_Type_AC,
	Charger_Type_USB,
	Charger_Type_WIRELESS,
	Charger_Type_Num,
	Charger_Type_Force32 = 0x7FFFFFFF
} Charger_Type;

static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val);

static enum power_supply_property bq27541_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

/*
 * restart.c uses this to decide whether power-off should reboot into charger
 * mode. A batteryless device has nothing to charge, even when USB is online.
 */
unsigned get_cable_status(void)
{
	return 0;
}

static void bq27541_check_cable_type(void)
{
	if (bq27541_battery_cable_status == USB_AC_Adapter) {
		ac_on = 1;
		usb_on = 0;
	} else if (bq27541_battery_cable_status == USB_Cable) {
		usb_on = 1;
		ac_on = 0;
	} else {
		ac_on = 0;
		usb_on = 0;
	}
}

static enum power_supply_property power_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int power_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	if (psp != POWER_SUPPLY_PROP_ONLINE)
		return -EINVAL;

	if (psy->type == POWER_SUPPLY_TYPE_MAINS && ac_on)
		val->intval = 1;
	else if (psy->type == POWER_SUPPLY_TYPE_USB && usb_on)
		val->intval = 1;
	else if (psy->type == POWER_SUPPLY_TYPE_WIRELESS && wireless_on)
		val->intval = 1;
	else
		val->intval = 0;

	return 0;
}

static char *supply_list[] = {
	"battery",
	"ac",
#ifndef REMOVE_USB_POWER_SUPPLY
	"usb",
#endif
};

static struct power_supply bq27541_supply[] = {
	{
		.name = "battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = bq27541_properties,
		.num_properties = ARRAY_SIZE(bq27541_properties),
		.get_property = bq27541_get_property,
	},
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#ifndef REMOVE_USB_POWER_SUPPLY
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
#endif
	{
		.name = "wireless",
		.type = POWER_SUPPLY_TYPE_WIRELESS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = power_properties,
		.num_properties = ARRAY_SIZE(power_properties),
		.get_property = power_get_property,
	},
};

static struct bq27541_device_info {
	struct i2c_client *client;
	struct delayed_work status_poll_work;
	struct wake_lock cable_wake_lock;
} *bq27541_device;

static void battery_status_poll(struct work_struct *work)
{
	struct bq27541_device_info *batt_dev = container_of(work,
		struct bq27541_device_info, status_poll_work.work);

	/* AICL belongs to the USB input path and remains useful without a cell. */
	reconfig_AICL();
	power_supply_changed(&bq27541_supply[Charger_Type_Battery]);

	queue_delayed_work(bq27541_battery_work_queue,
		&batt_dev->status_poll_work, BATTERY_POLLING_RATE * HZ);
}

static void bq27541_reschedule_poll(unsigned long delay)
{
	cancel_delayed_work(&bq27541_device->status_poll_work);
	queue_delayed_work(bq27541_battery_work_queue,
		&bq27541_device->status_poll_work, delay);
}

int bq27541_battery_callback(unsigned usb_cable_state)
{
	unsigned old_cable_status;

	if (!bq27541_battery_driver_ready) {
		BAT_NOTICE("battery driver not ready\n");
		return 0;
	}

	old_cable_status = bq27541_battery_cable_status;
	bq27541_battery_cable_status = usb_cable_state;

	if (old_cable_status != bq27541_battery_cable_status)
		wake_lock_timeout(&bq27541_device->cable_wake_lock, 5 * HZ);

	bq27541_check_cable_type();

	if (old_cable_status == USB_AC_Adapter)
		power_supply_changed(&bq27541_supply[Charger_Type_AC]);
#ifndef REMOVE_USB_POWER_SUPPLY
	else if (old_cable_status == USB_Cable)
		power_supply_changed(&bq27541_supply[Charger_Type_USB]);
#endif

	if (bq27541_battery_cable_status == USB_AC_Adapter)
		power_supply_changed(&bq27541_supply[Charger_Type_AC]);
#ifndef REMOVE_USB_POWER_SUPPLY
	else if (bq27541_battery_cable_status == USB_Cable)
		power_supply_changed(&bq27541_supply[Charger_Type_USB]);
#endif

	power_supply_changed(&bq27541_supply[Charger_Type_Battery]);
	bq27541_reschedule_poll(2 * HZ);

	return 1;
}
EXPORT_SYMBOL(bq27541_battery_callback);

int bq27541_wireless_callback(unsigned wireless_state)
{
	if (!bq27541_battery_driver_ready)
		return 0;

	power_supply_changed(&bq27541_supply[Charger_Type_WIRELESS]);
	power_supply_changed(&bq27541_supply[Charger_Type_Battery]);
	bq27541_reschedule_poll(2 * HZ);

	return 1;
}
EXPORT_SYMBOL(bq27541_wireless_callback);

static int bq27541_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = VIRTUAL_BATTERY_CAPACITY;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = VIRTUAL_BATTERY_VOLTAGE_UV;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = VIRTUAL_BATTERY_TEMP;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = VIRTUAL_BATTERY_CURRENT;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bq27541_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int ret;
	int i;

	BAT_NOTICE("registering batteryless virtual battery\n");

	bq27541_device = kzalloc(sizeof(*bq27541_device), GFP_KERNEL);
	if (!bq27541_device)
		return -ENOMEM;

	bq27541_device->client = client;
	i2c_set_clientdata(client, bq27541_device);
	wake_lock_init(&bq27541_device->cable_wake_lock,
		WAKE_LOCK_SUSPEND, "cable_state_changed");

	bq27541_battery_work_queue =
		create_singlethread_workqueue("bq27541_battery_work_queue");
	if (!bq27541_battery_work_queue) {
		ret = -ENOMEM;
		goto fail_workqueue;
	}
	INIT_DELAYED_WORK(&bq27541_device->status_poll_work,
		battery_status_poll);

	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++) {
		ret = power_supply_register(&client->dev, &bq27541_supply[i]);
		if (ret) {
			BAT_ERR("Failed to register power supply\n");
			goto fail_supply;
		}
	}

	bq27541_battery_driver_ready = 1;
	queue_delayed_work(bq27541_battery_work_queue,
		&bq27541_device->status_poll_work, 15 * HZ);

	return 0;

fail_supply:
	while (i--)
		power_supply_unregister(&bq27541_supply[i]);
	destroy_workqueue(bq27541_battery_work_queue);
	bq27541_battery_work_queue = NULL;
fail_workqueue:
	wake_lock_destroy(&bq27541_device->cable_wake_lock);
	kfree(bq27541_device);
	bq27541_device = NULL;
	return ret;
}

static int bq27541_remove(struct i2c_client *client)
{
	struct bq27541_device_info *device = i2c_get_clientdata(client);
	int i;

	bq27541_battery_driver_ready = 0;
	cancel_delayed_work_sync(&device->status_poll_work);
	destroy_workqueue(bq27541_battery_work_queue);
	bq27541_battery_work_queue = NULL;

	for (i = 0; i < ARRAY_SIZE(bq27541_supply); i++)
		power_supply_unregister(&bq27541_supply[i]);

	wake_lock_destroy(&device->cable_wake_lock);
	kfree(device);
	bq27541_device = NULL;

	return 0;
}

#if defined(CONFIG_PM)
static int bq27541_suspend(struct i2c_client *client, pm_message_t state)
{
	cancel_delayed_work_sync(&bq27541_device->status_poll_work);
	flush_workqueue(bq27541_battery_work_queue);
	return 0;
}

static int bq27541_resume(struct i2c_client *client)
{
	bq27541_reschedule_poll(HZ / 10);
	return 0;
}
#endif

static const struct i2c_device_id bq27541_id[] = {
	{ "bq27541-battery", 0 },
	{},
};

static struct i2c_driver bq27541_battery_driver = {
	.probe = bq27541_probe,
	.remove = bq27541_remove,
#if defined(CONFIG_PM)
	.suspend = bq27541_suspend,
	.resume = bq27541_resume,
#endif
	.id_table = bq27541_id,
	.driver = {
		.name = "bq27541-battery",
	},
};

static int __init bq27541_battery_init(void)
{
	int ret = i2c_add_driver(&bq27541_battery_driver);

	if (ret)
		pr_err("bq27541-battery: i2c_add_driver failed: %d\n", ret);
	return ret;
}
module_init(bq27541_battery_init);

static void __exit bq27541_battery_exit(void)
{
	i2c_del_driver(&bq27541_battery_driver);
}
module_exit(bq27541_battery_exit);

MODULE_AUTHOR("ASUSTek Inc.");
MODULE_DESCRIPTION("Batteryless virtual battery driver for Nexus 7 2013");
MODULE_LICENSE("GPL");
