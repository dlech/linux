
#ifndef _PRU_H_
#define _PRU_H_

#include <linux/pruss.h>
#include <linux/remoteproc.h>
#include <linux/types.h>

typedef struct arm_pru_iomap {
	struct rproc *pru[PRUSS_NUM_PRUS];
	void *pru_dram_io_addr[PRUSS_NUM_PRUS];
	void *mcasp_io_addr;
	dma_addr_t pFifoBufferPhysBase;
	void *pFifoBufferVirtBase;
	int arm_to_pru_irq[PRUSS_NUM_PRUS];
} arm_pru_iomap;

#endif				// End _PRU_H_
