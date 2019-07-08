/* SPDX-License-Identifier: GPL-2.0 OR MIT */
/*
 * This header provides constants for the Texas Instruments Programmable
 * Realtime Unit Subsystem (PRUSS) interrupt controller.
 */

#ifndef _DT_BINDINGS_INTERRUPT_CONTROLLER_TI_PRUSS_H
#define _DT_BINDINGS_INTERRUPT_CONTROLLER_TI_PRUSS_H

/* interrupt specifier for optional cell 1 */

#define TI_PRUSS_INTC_EVTSEL_ANY	0xffffffff

/* interrupt specifier for cell #interrupt-cells - 1 */

/* host interrupt is connected to PRU cores, e.g. host events 0 and 1 */
#define TI_PRUSS_INTC_DOMAIN_PRU	0
/* host interrupt is connected to MCU's interrupt controller  */
#define TI_PRUSS_INTC_DOMAIN_MCU	1
/* host interrupt is connected to DSP's interrupt controller  */
#define TI_PRUSS_INTC_DOMAIN_DSP	2
/* host interrupt is connected to the auxillary PRU cores  */
#define TI_PRUSS_INTC_DOMAIN_RTU_PRU	3
/* host interrupt is connected to the task managers  */
#define TI_PRUSS_INTC_DOMAIN_TASK	4

#endif /* _DT_BINDINGS_INTERRUPT_CONTROLLER_TI_PRUSS_H */
