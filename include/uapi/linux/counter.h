/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Userspace ABI for Counter character devices
 * Copyright (C) 2020 William Breathitt Gray
 */
#ifndef _UAPI_COUNTER_H_
#define _UAPI_COUNTER_H_

#include <linux/ioctl.h>
#include <linux/types.h>

/* Component type definitions */
enum counter_component_type {
	COUNTER_COMPONENT_NONE,
	COUNTER_COMPONENT_SIGNAL,
	COUNTER_COMPONENT_COUNT,
	COUNTER_COMPONENT_FUNCTION,
	COUNTER_COMPONENT_SYNAPSE_ACTION,
	COUNTER_COMPONENT_EXTENSION,
};

/* Component scope definitions */
enum counter_scope {
	COUNTER_SCOPE_DEVICE,
	COUNTER_SCOPE_SIGNAL,
	COUNTER_SCOPE_COUNT,
};

/**
 * struct counter_component - Counter component identification
 * @type: component type (Count, extension, etc.)
 * @scope: component scope (Device, Count, or Signal)
 * @parent: parent component identification number
 * @id: component identification number
 */
struct counter_component {
	__u8 type;
	__u8 scope;
	__u8 parent;
	__u8 id;
};

/* Event type definitions */
enum counter_event_type {
	COUNTER_EVENT_OVERFLOW,
	COUNTER_EVENT_UNDERFLOW,
	COUNTER_EVENT_OVERFLOW_UNDERFLOW,
	COUNTER_EVENT_THRESHOLD,
	COUNTER_EVENT_INDEX,
	COUNTER_EVENT_DIRECTION_CHANGE,
	COUNTER_EVENT_TIMEOUT,
};

/**
 * struct counter_watch - Counter component watch configuration
 * @component: component to watch when event triggers
 * @event: event that triggers
 * @channel: event channel
 */
struct counter_watch {
	struct counter_component component;
	__u8 event;
	__u8 channel;
};

/* ioctl commands */
#define COUNTER_CLEAR_WATCHES_IOCTL _IO(0x3E, 0x00)
#define COUNTER_ADD_WATCH_IOCTL _IOW(0x3E, 0x01, struct counter_watch)
#define COUNTER_LOAD_WATCHES_IOCTL _IO(0x3E, 0x02)

/**
 * struct counter_event - Counter event data
 * @timestamp: best estimate of time of event occurrence, in nanoseconds
 * @value: component value
 * @watch: component watch configuration
 * @errno: system error number
 */
struct counter_event {
	__aligned_u64 timestamp;
	__aligned_u64 value;
	struct counter_watch watch;
	__u8 errno;
};

/* Count direction values */
enum counter_count_direction {
	COUNTER_COUNT_DIRECTION_FORWARD,
	COUNTER_COUNT_DIRECTION_BACKWARD,
};

/* Count mode values */
enum counter_count_mode {
	COUNTER_COUNT_MODE_NORMAL,
	COUNTER_COUNT_MODE_RANGE_LIMIT,
	COUNTER_COUNT_MODE_NON_RECYCLE,
	COUNTER_COUNT_MODE_MODULO_N,
};

/* Count function values */
enum counter_function {
	COUNTER_FUNCTION_INCREASE,
	COUNTER_FUNCTION_DECREASE,
	COUNTER_FUNCTION_PULSE_DIRECTION,
	COUNTER_FUNCTION_QUADRATURE_X1_A,
	COUNTER_FUNCTION_QUADRATURE_X1_B,
	COUNTER_FUNCTION_QUADRATURE_X2_A,
	COUNTER_FUNCTION_QUADRATURE_X2_B,
	COUNTER_FUNCTION_QUADRATURE_X4,
};

/* Signal values */
enum counter_signal_level {
	COUNTER_SIGNAL_LEVEL_LOW,
	COUNTER_SIGNAL_LEVEL_HIGH,
};

/* Action mode values */
enum counter_synapse_action {
	COUNTER_SYNAPSE_ACTION_NONE,
	COUNTER_SYNAPSE_ACTION_RISING_EDGE,
	COUNTER_SYNAPSE_ACTION_FALLING_EDGE,
	COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
};

#endif /* _UAPI_COUNTER_H_ */
