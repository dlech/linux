// SPDX-License-Identifier: GPL-2.0
/*
 * Generic Counter interface
 * Copyright (C) 2020 William Breathitt Gray
 */
#include <linux/counter.h>
#include <linux/device.h>
#include <linux/export.h>
#include <linux/gfp.h>
#include <linux/idr.h>
#include <linux/init.h>
#include <linux/module.h>

#include "counter-sysfs.h"

/* Provides a unique ID for each counter device */
static DEFINE_IDA(counter_ida);

static void counter_device_release(struct device *dev)
{
	struct counter_device *const counter = dev_get_drvdata(dev);

	counter_chrdev_remove(counter);
	ida_simple_remove(&counter_ida, counter->id);
}

static struct device_type counter_device_type = {
	.name = "counter_device",
	.release = counter_device_release
};

static struct bus_type counter_bus_type = {
	.name = "counter"
};

/**
 * counter_register - register Counter to the system
 * @counter:	pointer to Counter to register
 *
 * This function registers a Counter to the system. A sysfs "counter" directory
 * will be created and populated with sysfs attributes correlating with the
 * Counter Signals, Synapses, and Counts respectively.
 */
int counter_register(struct counter_device *const counter)
{
	struct device *const dev = &counter->dev;
	int err;

	/* Acquire unique ID */
	counter->id = ida_simple_get(&counter_ida, 0, 0, GFP_KERNEL);
	if (counter->id < 0)
		return counter->id;

	/* Configure device structure for Counter */
	dev->type = &counter_device_type;
	dev->bus = &counter_bus_type;
	if (counter->parent) {
		dev->parent = counter->parent;
		dev->of_node = counter->parent->of_node;
	}
	dev_set_name(dev, "counter%d", counter->id);
	device_initialize(dev);
	dev_set_drvdata(dev, counter);

	/* Add Counter sysfs attributes */
	err = counter_sysfs_add(counter);
	if (err < 0)
		goto err_free_id;

	/* Add device to system */
	err = device_add(dev);
	if (err < 0)
		goto err_free_id;

	return 0;

err_free_id:
	put_device(dev);
	return err;
}
EXPORT_SYMBOL_GPL(counter_register);

/**
 * counter_unregister - unregister Counter from the system
 * @counter:	pointer to Counter to unregister
 *
 * The Counter is unregistered from the system; all allocated memory is freed.
 */
void counter_unregister(struct counter_device *const counter)
{
	if (!counter)
		return;

	device_unregister(&counter->dev);
}
EXPORT_SYMBOL_GPL(counter_unregister);

static void devm_counter_unregister(struct device *dev, void *res)
{
	counter_unregister(*(struct counter_device **)res);
}

/**
 * devm_counter_register - Resource-managed counter_register
 * @dev:	device to allocate counter_device for
 * @counter:	pointer to Counter to register
 *
 * Managed counter_register. The Counter registered with this function is
 * automatically unregistered on driver detach. This function calls
 * counter_register internally. Refer to that function for more information.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int devm_counter_register(struct device *dev,
			  struct counter_device *const counter)
{
	struct counter_device **ptr;
	int err;

	ptr = devres_alloc(devm_counter_unregister, sizeof(*ptr), GFP_KERNEL);
	if (!ptr)
		return -ENOMEM;

	err = counter_register(counter);
	if (err < 0) {
		devres_free(ptr);
		return err;
	}

	*ptr = counter;
	devres_add(dev, ptr);

	return 0;
}
EXPORT_SYMBOL_GPL(devm_counter_register);

static int __init counter_init(void)
{
	return bus_register(&counter_bus_type);
}

static void __exit counter_exit(void)
{
	bus_unregister(&counter_bus_type);
}

subsys_initcall(counter_init);
module_exit(counter_exit);

MODULE_AUTHOR("William Breathitt Gray <vilhelm.gray@gmail.com>");
MODULE_DESCRIPTION("Generic Counter interface");
MODULE_LICENSE("GPL v2");
