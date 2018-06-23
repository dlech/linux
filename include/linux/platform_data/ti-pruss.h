/* SPDX-License-Identifier: GPL-2.0 */
/*
 * TI PRU remote processor platform data
 */

#ifndef _LINUX_PLATFORM_DATA_TI_PRU_H
#define _LINUX_PLATFORM_DATA_TI_PRU_H

struct platform_device;

struct ti_pruss_platform_data {
	const char *reset_name;

	int (*assert_reset)(struct platform_device *pdev, const char *name);
	int (*deassert_reset)(struct platform_device *pdev, const char *name);
};

#endif /* _LINUX_PLATFORM_DATA_TI_PRU_H */
