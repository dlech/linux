/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Counter character device interface
 * Copyright (C) 2020 William Breathitt Gray
 */
#ifndef _COUNTER_CHRDEV_H_
#define _COUNTER_CHRDEV_H_

#include <linux/counter.h>
#include <linux/types.h>

int counter_chrdev_add(struct counter_device *const counter,
		       const dev_t counter_devt);
void counter_chrdev_remove(struct counter_device *const counter);

#endif /* _COUNTER_CHRDEV_H_ */
