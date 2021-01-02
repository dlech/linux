
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


short pru_ram_write_data(u32 u32offset, u8 * pu8datatowrite, u16 u16wordstowrite,
			 arm_pru_iomap * pru_arm_iomap);
short pru_ram_read_data(u32 u32offset, u8 * pu8datatoread, u16 u16wordstoread,
			arm_pru_iomap * pru_arm_iomap);
short pru_ram_read_data_4byte(unsigned int u32offset,
				  unsigned int *pu32datatoread,
				  short u16wordstoread);
short pru_ram_write_data_4byte(unsigned int u32offset,
				   unsigned int *pu32datatoread,
				   short u16wordstoread);

#endif				// End _PRU_H_
