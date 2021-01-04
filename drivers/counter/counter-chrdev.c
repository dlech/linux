// SPDX-License-Identifier: GPL-2.0
/*
 * Generic Counter character device interface
 * Copyright (C) 2020 William Breathitt Gray
 */

#include <linux/cdev.h>
#include <linux/counter.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/fs.h>
#include <linux/kdev_t.h>
#include <linux/kfifo.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/nospec.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timekeeping.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/uaccess.h>

#include "counter-chrdev.h"

struct counter_comp_node {
	struct list_head l;
	struct counter_component component;
	struct counter_comp comp;
	void *parent;
};

static ssize_t counter_chrdev_read(struct file *filp, char __user *buf,
				   size_t len, loff_t *f_ps)
{
	struct counter_device *const counter = filp->private_data;
	int err;
	unsigned int copied;

	if (len < sizeof(struct counter_event))
		return -EINVAL;

	do {
		if (kfifo_is_empty(&counter->events)) {
			if (filp->f_flags & O_NONBLOCK)
				return -EAGAIN;

			err = wait_event_interruptible(counter->events_wait,
					!kfifo_is_empty(&counter->events));
			if (err < 0)
				return err;
		}

		if (mutex_lock_interruptible(&counter->events_lock))
			return -ERESTARTSYS;
		err = kfifo_to_user(&counter->events, buf, len, &copied);
		mutex_unlock(&counter->events_lock);
		if (err < 0)
			return err;
	} while (!copied);

	return copied;
}

static __poll_t counter_chrdev_poll(struct file *filp,
				    struct poll_table_struct *pollt)
{
	struct counter_device *const counter = filp->private_data;
	__poll_t events = 0;

	poll_wait(filp, &counter->events_wait, pollt);

	if (!kfifo_is_empty(&counter->events))
		events = EPOLLIN | EPOLLRDNORM;

	return events;
}

static void counter_events_list_free(struct list_head *const events_list)
{
	struct counter_event_node *p, *n;
	struct counter_comp_node *q, *o;

	list_for_each_entry_safe(p, n, events_list, l) {
		/* Free associated component nodes */
		list_for_each_entry_safe(q, o, &p->comp_list, l) {
			list_del(&q->l);
			kfree(q);
		}

		/* Free event node */
		list_del(&p->l);
		kfree(p);
	}
}

static int counter_set_event_node(struct counter_device *const counter,
				  struct counter_watch *const watch,
				  const struct counter_comp_node *const cfg)
{
	struct counter_event_node *event_node;
	struct counter_comp_node *comp_node;

	/* Search for event in the list */
	list_for_each_entry(event_node, &counter->next_events_list, l)
		if (event_node->event == watch->event &&
		    event_node->channel == watch->channel)
			break;

	/* If event is not already in the list */
	if (&event_node->l == &counter->next_events_list) {
		/* Allocate new event node */
		event_node = kmalloc(sizeof(*event_node), GFP_ATOMIC);
		if (!event_node)
			return -ENOMEM;

		/* Configure event node and add to the list */
		event_node->event = watch->event;
		event_node->channel = watch->channel;
		INIT_LIST_HEAD(&event_node->comp_list);
		list_add(&event_node->l, &counter->next_events_list);
	}

	/* Check if component watch has already been set before */
	list_for_each_entry(comp_node, &event_node->comp_list, l)
		if (comp_node->parent == cfg->parent &&
		    comp_node->comp.count_u8_read == cfg->comp.count_u8_read)
			return -EINVAL;

	/* Allocate component node */
	comp_node = kmalloc(sizeof(*comp_node), GFP_ATOMIC);
	if (!comp_node) {
		/* Free event node if no one else is watching */
		if (list_empty(&event_node->comp_list)) {
			list_del(&event_node->l);
			kfree(event_node);
		}
		return -ENOMEM;
	}
	*comp_node = *cfg;

	/* Add component node to event node */
	list_add_tail(&comp_node->l, &event_node->comp_list);

	return 0;
}

static int counter_clear_watches(struct counter_device *const counter)
{
	unsigned long flags;
	int err = 0;

	raw_spin_lock_irqsave(&counter->events_list_lock, flags);

	counter_events_list_free(&counter->events_list);

	if (counter->ops->events_configure)
		err = counter->ops->events_configure(counter);

	raw_spin_unlock_irqrestore(&counter->events_list_lock, flags);

	counter_events_list_free(&counter->next_events_list);

	return err;
}

static int counter_add_watch(struct counter_device *const counter,
			     const unsigned long arg)
{
	void __user *const uwatch = (void __user *)arg;
	struct counter_watch watch;
	struct counter_comp_node comp_node = {0};
	size_t parent, id;
	struct counter_comp *ext;
	size_t num_ext;
	int err;

	if (copy_from_user(&watch, uwatch, sizeof(watch)))
		return -EFAULT;

	if (watch.component.type == COUNTER_COMPONENT_NONE)
		goto no_component;

	parent = watch.component.parent;

	/* Configure parent component info for comp node */
	switch (watch.component.scope) {
	case COUNTER_SCOPE_DEVICE:
		ext = counter->ext;
		num_ext = counter->num_ext;
		break;
	case COUNTER_SCOPE_SIGNAL:
		if (parent >= counter->num_signals)
			return -EINVAL;
		parent = array_index_nospec(parent, counter->num_signals);

		comp_node.parent = counter->signals + parent;

		ext = counter->signals[parent].ext;
		num_ext = counter->signals[parent].num_ext;
		break;
	case COUNTER_SCOPE_COUNT:
		if (parent >= counter->num_counts)
			return -EINVAL;
		parent = array_index_nospec(parent, counter->num_counts);

		comp_node.parent = counter->counts + parent;

		ext = counter->counts[parent].ext;
		num_ext = counter->counts[parent].num_ext;
		break;
	}

	id = watch.component.id;

	/* Configure component info for comp node */
	switch (watch.component.type) {
	case COUNTER_COMPONENT_SIGNAL:
		if (watch.component.scope != COUNTER_SCOPE_SIGNAL)
			return -EINVAL;

		comp_node.comp.type = COUNTER_COMP_SIGNAL_LEVEL;
		comp_node.comp.signal_u32_read = counter->ops->signal_read;
		break;
	case COUNTER_COMPONENT_COUNT:
		if (watch.component.scope != COUNTER_SCOPE_COUNT)
			return -EINVAL;

		comp_node.comp.type = COUNTER_COMP_U64;
		comp_node.comp.count_u64_read = counter->ops->count_read;
		break;
	case COUNTER_COMPONENT_FUNCTION:
		if (watch.component.scope != COUNTER_SCOPE_COUNT)
			return -EINVAL;

		comp_node.comp.type = COUNTER_COMP_FUNCTION;
		comp_node.comp.count_u32_read = counter->ops->function_read;
		break;
	case COUNTER_COMPONENT_SYNAPSE_ACTION:
		if (watch.component.scope != COUNTER_SCOPE_COUNT)
			return -EINVAL;
		if (id >= counter->counts[parent].num_synapses)
			return -EINVAL;
		id = array_index_nospec(id, counter->counts[parent].num_synapses);

		comp_node.comp.type = COUNTER_COMP_SYNAPSE_ACTION;
		comp_node.comp.action_read = counter->ops->action_read;
		comp_node.comp.priv = counter->counts[parent].synapses + id;
		break;
	case COUNTER_COMPONENT_EXTENSION:
		if (id >= num_ext)
			return -EINVAL;
		id = array_index_nospec(id, num_ext);

		comp_node.comp = ext[id];
		break;
	default:
		return -EINVAL;
	}
	/* Check if any read callback is set; this is part of a union */
	if (!comp_node.comp.count_u8_read)
		return -EOPNOTSUPP;

no_component:
	if (counter->ops->watch_validate) {
		err = counter->ops->watch_validate(counter, &watch);
		if (err < 0)
			return err;
	}

	comp_node.component = watch.component;

	return counter_set_event_node(counter, &watch, &comp_node);
}

static long counter_chrdev_ioctl(struct file *filp, unsigned int cmd,
				 unsigned long arg)
{
	struct counter_device *const counter = filp->private_data;
	unsigned long flags;
	int err = 0;

	switch (cmd) {
	case COUNTER_CLEAR_WATCHES_IOCTL:
		return counter_clear_watches(counter);
	case COUNTER_ADD_WATCH_IOCTL:
		return counter_add_watch(counter, arg);
	case COUNTER_LOAD_WATCHES_IOCTL:
		raw_spin_lock_irqsave(&counter->events_list_lock, flags);

		counter_events_list_free(&counter->events_list);
		list_replace_init(&counter->next_events_list,
				  &counter->events_list);

		if (counter->ops->events_configure)
			err = counter->ops->events_configure(counter);

		raw_spin_unlock_irqrestore(&counter->events_list_lock, flags);
		break;
	default:
		return -ENOIOCTLCMD;
	}

	return err;
}

static int counter_chrdev_open(struct inode *inode, struct file *filp)
{
	struct counter_device *const counter = container_of(inode->i_cdev,
							    typeof(*counter),
							    chrdev);

	get_device(&counter->dev);
	filp->private_data = counter;

	return nonseekable_open(inode, filp);
}

static int counter_chrdev_release(struct inode *inode, struct file *filp)
{
	struct counter_device *const counter = filp->private_data;
	int err;

	err = counter_clear_watches(counter);
	if (err < 0)
		return err;

	put_device(&counter->dev);

	return 0;
}

static const struct file_operations counter_fops = {
	.llseek = no_llseek,
	.read = counter_chrdev_read,
	.poll = counter_chrdev_poll,
	.unlocked_ioctl = counter_chrdev_ioctl,
	.open = counter_chrdev_open,
	.release = counter_chrdev_release,
};

int counter_chrdev_add(struct counter_device *const counter,
		       const dev_t counter_devt)
{
	struct device *const dev = &counter->dev;
	struct cdev *const chrdev = &counter->chrdev;

	/* Initialize Counter events lists */
	INIT_LIST_HEAD(&counter->events_list);
	INIT_LIST_HEAD(&counter->next_events_list);
	raw_spin_lock_init(&counter->events_list_lock);

	/* Initialize Counter events queue */
	INIT_KFIFO(counter->events);
	init_waitqueue_head(&counter->events_wait);
	mutex_init(&counter->events_lock);

	/* Initialize character device */
	cdev_init(chrdev, &counter_fops);
	dev->devt = MKDEV(MAJOR(counter_devt), counter->id);
	cdev_set_parent(chrdev, &dev->kobj);

	return cdev_add(chrdev, dev->devt, 1);
}

void counter_chrdev_remove(struct counter_device *const counter)
{
	cdev_del(&counter->chrdev);
}

static int counter_get_data(struct counter_device *const counter,
			    const struct counter_comp_node *const comp_node,
			    u64 *const value)
{
	const struct counter_comp *const comp = &comp_node->comp;
	void *const parent = comp_node->parent;
	int err = 0;
	u8 value_u8 = 0;
	u32 value_u32 = 0;

	if (comp_node->component.type == COUNTER_COMPONENT_NONE)
		return 0;

	switch (comp->type) {
	case COUNTER_COMP_U8:
	case COUNTER_COMP_BOOL:
		switch (comp_node->component.scope) {
		case COUNTER_SCOPE_DEVICE:
			err = comp->device_u8_read(counter, &value_u8);
			break;
		case COUNTER_SCOPE_SIGNAL:
			err = comp->signal_u8_read(counter, parent, &value_u8);
			break;
		case COUNTER_SCOPE_COUNT:
			err = comp->count_u8_read(counter, parent, &value_u8);
			break;
		}
		*value = value_u8;
		break;
	case COUNTER_COMP_SIGNAL_LEVEL:
	case COUNTER_COMP_FUNCTION:
	case COUNTER_COMP_ENUM:
	case COUNTER_COMP_COUNT_DIRECTION:
	case COUNTER_COMP_COUNT_MODE:
		switch (comp_node->component.scope) {
		case COUNTER_SCOPE_DEVICE:
			err = comp->device_u32_read(counter, &value_u32);
			break;
		case COUNTER_SCOPE_SIGNAL:
			err = comp->signal_u32_read(counter, parent,
						    &value_u32);
			break;
		case COUNTER_SCOPE_COUNT:
			err = comp->count_u32_read(counter, parent, &value_u32);
			break;
		}
		*value = value_u32;
		break;
	case COUNTER_COMP_U64:
		switch (comp_node->component.scope) {
		case COUNTER_SCOPE_DEVICE:
			return comp->device_u64_read(counter, value);
		case COUNTER_SCOPE_SIGNAL:
			return comp->signal_u64_read(counter, parent, value);
		case COUNTER_SCOPE_COUNT:
			return comp->count_u64_read(counter, parent, value);
		}
		break;
	case COUNTER_COMP_SYNAPSE_ACTION:
		err = comp->action_read(counter, parent, comp->priv,
					&value_u32);
		*value = value_u32;
		break;
	}

	return err;
}

/**
 * counter_push_event - queue event for userspace reading
 * @counter:	pointer to Counter structure
 * @event:	triggered event
 * @channel:	event channel
 *
 * Note: If no one is watching for the respective event, it is silently
 * discarded.
 */
void counter_push_event(struct counter_device *const counter, const u8 event,
			const u8 channel)
{
	struct counter_event ev = {0};
	unsigned int copied = 0;
	unsigned long flags;
	struct counter_event_node *event_node;
	struct counter_comp_node *comp_node;

	ev.timestamp = ktime_get_ns();
	ev.watch.event = event;
	ev.watch.channel = channel;

	raw_spin_lock_irqsave(&counter->events_list_lock, flags);

	/* Search for event in the list */
	list_for_each_entry(event_node, &counter->events_list, l)
		if (event_node->event == event &&
		    event_node->channel == channel)
			break;

	/* If event is not in the list */
	if (&event_node->l == &counter->events_list)
		goto exit_early;

	/* Read and queue relevant comp for userspace */
	list_for_each_entry(comp_node, &event_node->comp_list, l) {
		ev.watch.component = comp_node->component;
		ev.errno = -counter_get_data(counter, comp_node, &ev.value);

		copied += kfifo_put(&counter->events, ev);
	}

	if (copied)
		wake_up_poll(&counter->events_wait, EPOLLIN);

exit_early:
	raw_spin_unlock_irqrestore(&counter->events_list_lock, flags);
}
EXPORT_SYMBOL_GPL(counter_push_event);
