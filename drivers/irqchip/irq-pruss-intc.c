// SPDX-License-Identifier: GPL-2.0
/*
 * PRU-ICSS INTC IRQChip driver for various TI SoCs
 *
 * Copyright (C) 2016-2019 Texas Instruments Incorporated - http://www.ti.com/
 *	Andrew F. Davis <afd@ti.com>
 *	Suman Anna <s-anna@ti.com>
 *
 * Copyright (C) 2019 David Lechner <david@lechnology.com>
 */

#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <dt-bindings/interrupt-controller/ti-pruss.h>

/* The number of possible interrupt domains, see TI_PRUSS_INTC_DOMAIN_* in
 * dt-bindings/interrupt-controller/ti-pruss.h
 */
#define NUM_TI_PRUSS_INTC_DOMAIN 5

/*
 * Number of host interrupts reaching the main MPU sub-system. Note that this
 * is not the same as the total number of host interrupts supported by the PRUSS
 * INTC instance
 */
#define MAX_NUM_HOST_IRQS	8

/* minimum starting host interrupt number for MPU */
#define MIN_PRU_HOST_INT	2

/* maximum number of host interrupts */
#define MAX_PRU_HOST_INT	10

/* maximum number of interrupt channels */
#define MAX_PRU_CHANNELS	10

/* maximum number of system events */
#define MAX_PRU_SYS_EVENTS	64

/* PRU_ICSS_INTC registers */
#define PRU_INTC_REVID		0x0000
#define PRU_INTC_CR		0x0004
#define PRU_INTC_GER		0x0010
#define PRU_INTC_GNLR		0x001c
#define PRU_INTC_SISR		0x0020
#define PRU_INTC_SICR		0x0024
#define PRU_INTC_EISR		0x0028
#define PRU_INTC_EICR		0x002c
#define PRU_INTC_HIEISR		0x0034
#define PRU_INTC_HIDISR		0x0038
#define PRU_INTC_GPIR		0x0080
#define PRU_INTC_SRSR(x)	(0x0200 + (x) * 4)
#define PRU_INTC_SECR0		0x0280
#define PRU_INTC_SECR1		0x0284
#define PRU_INTC_ESR0		0x0300
#define PRU_INTC_ESR1		0x0304
#define PRU_INTC_ECR0		0x0380
#define PRU_INTC_ECR1		0x0384
#define PRU_INTC_CMR(x)		(0x0400 + (x) * 4)
#define PRU_INTC_HMR(x)		(0x0800 + (x) * 4)
#define PRU_INTC_HIPIR(x)	(0x0900 + (x) * 4)
#define PRU_INTC_SIPR0		0x0d00
#define PRU_INTC_SIPR1		0x0d04
#define PRU_INTC_SITR0		0x0d80
#define PRU_INTC_SITR1		0x0d84
#define PRU_INTC_HINLR(x)	(0x1100 + (x) * 4)
#define PRU_INTC_HIER		0x1500

/* CMR register bit-field macros */
#define CMR_EVT_MAP_MASK	0xf
#define CMR_EVT_MAP_BITS	8
#define CMR_EVT_PER_REG		4

/* HMR register bit-field macros */
#define HMR_CH_MAP_MASK		0xf
#define HMR_CH_MAP_BITS		8
#define HMR_CH_PER_REG		4

/* HIPIR register bit-fields */
#define INTC_HIPIR_NONE_HINT	0x80000000

/**
 * struct pruss_intc_hwirq_data - additional metadata associated with a PRU
 * system event
 * @evtsel: The event select index (AM18xx only)
 * @channel: The PRU INTC channel that the system event should be mapped to
 * @host: The PRU INTC host that the channel should be mapped to
 */
struct pruss_intc_hwirq_data {
	u8 evtsel;
	u8 channel;
	u8 host;
};

/**
 * struct pruss_intc_map_record - keeps track of actual mapping state
 * @value: The currently mapped value (evtsel, channel or host)
 * @ref_count: Keeps track of number of current users of this resource
 */
struct pruss_intc_map_record {
	u8 value;
	u8 ref_count;
};

/**
 * struct pruss_intc_domain - information specific to an external IRQ domain
 * @hwirq_data: Table of additional mapping data received from device tree
 *	or PRU firmware
 * @domain: irq domain
 * @intc: the interrupt controller
 * @id: Unique domain identifier (from device tree bindings)
 */
struct pruss_intc_domain {
	struct pruss_intc_hwirq_data hwirq_data[MAX_PRU_SYS_EVENTS];
	struct irq_domain *domain;
	struct pruss_intc *intc;
	u32 id;
};

/**
 * struct pruss_intc - PRUSS interrupt controller structure
 * @domain: External interrupt domains
 * @evtsel: Tracks the current state of CFGCHIP3[3].PRUSSEVTSEL (AM18xx only)
 * @event_channel: Tracks the current state of system event to channel mappings
 * @channel_host: Tracks the current state of channel to host mappings
 * @irqs: kernel irq numbers corresponding to PRUSS host interrupts
 * @base: base virtual address of INTC register space
 * @irqchip: irq chip for this interrupt controller
 * @lock: mutex to serialize access to INTC
 * @shared_intr: bit-map denoting if the MPU host interrupt is shared
 * @invalid_intr: bit-map denoting if host interrupt is not connected to MPU
 * @has_evtsel: indicates that the chip has an event select mux
 */
struct pruss_intc {
	struct pruss_intc_domain domain[NUM_ISA_INTERRUPTS];
	struct pruss_intc_map_record evtsel;
	struct pruss_intc_map_record event_channel[MAX_PRU_SYS_EVENTS];
	struct pruss_intc_map_record channel_host[MAX_PRU_CHANNELS];
	unsigned int irqs[MAX_NUM_HOST_IRQS];
	void __iomem *base;
	struct irq_chip *irqchip;
	struct mutex lock; /* PRUSS INTC lock */
	u16 shared_intr;
	u16 invalid_intr;
	bool has_evtsel;
};

static inline u32 pruss_intc_read_reg(struct pruss_intc *intc, unsigned int reg)
{
	return readl_relaxed(intc->base + reg);
}

static inline void pruss_intc_write_reg(struct pruss_intc *intc,
					unsigned int reg, u32 val)
{
	writel_relaxed(val, intc->base + reg);
}

static int pruss_intc_check_write(struct pruss_intc *intc, unsigned int reg,
				  unsigned int sysevent)
{
	if (!intc)
		return -EINVAL;

	if (sysevent >= MAX_PRU_SYS_EVENTS)
		return -EINVAL;

	pruss_intc_write_reg(intc, reg, sysevent);

	return 0;
}

/**
 * pruss_intc_map() - configure the PRUSS INTC
 * @domain: pru intc domain pointer
 * @hwirq: the system event number
 *
 * Configures the PRUSS INTC with the provided configuration from the one
 * parsed in the xlate function. Any existing event to channel mappings or
 * channel to host interrupt mappings are checked to make sure there are no
 * conflicting configuration between both the PRU cores.
 *
 * Returns 0 on success, or a suitable error code otherwise
 */
static int pruss_intc_map(struct pruss_intc_domain *domain, unsigned long hwirq)
{
	struct pruss_intc *intc = domain->intc;
	struct device* dev = intc->irqchip->parent_device;
	u32 val;
	int idx, ret;
	u8 evtsel, ch, host;

	if (hwirq >= MAX_PRU_SYS_EVENTS)
		return -EINVAL;

	mutex_lock(&intc->lock);

	evtsel = domain->hwirq_data[hwirq].evtsel;
	ch = domain->hwirq_data[hwirq].channel;
	host = domain->hwirq_data[hwirq].host;

	if (intc->has_evtsel && intc->evtsel.ref_count > 0 &&
	    intc->evtsel.value != evtsel) {
		dev_err(dev, "event %lu (req. evtsel %d) already assigned to evtsel %d\n",
			hwirq, evtsel, intc->evtsel.value);
		ret = -EBUSY;
		goto unlock;
	}

	/* check if sysevent already assigned */
	if (intc->event_channel[hwirq].ref_count > 0 &&
	    intc->event_channel[hwirq].value != ch) {
		dev_err(dev, "event %lu (req. channel %d) already assigned to channel %d\n",
			hwirq, ch, intc->event_channel[hwirq].value);
		ret = -EBUSY;
		goto unlock;
	}

	/* check if channel already assigned */
	if (intc->channel_host[ch].ref_count > 0 &&
	    intc->channel_host[ch].value != host) {
		dev_err(dev, "channel %d (req. host %d) already assigned to host %d\n",
			ch, host, intc->channel_host[ch].value);
		ret = -EBUSY;
		goto unlock;
	}

	if (++intc->evtsel.ref_count == 1) {
		intc->evtsel.value = evtsel;

		/* TODO: need to implement CFGCHIP3[3].PRUSSEVTSEL */
	}

	if (++intc->event_channel[hwirq].ref_count == 1) {
		intc->event_channel[hwirq].value = ch;

		/*
		 * configure channel map registers - each register holds map
		 * info for 4 events, with each event occupying the lower nibble
		 * in a register byte address in little-endian fashion
		 */
		idx = hwirq / CMR_EVT_PER_REG;

		val = pruss_intc_read_reg(intc, PRU_INTC_CMR(idx));
		val &= ~(CMR_EVT_MAP_MASK <<
				((hwirq % CMR_EVT_PER_REG) * CMR_EVT_MAP_BITS));
		val |= ch << ((hwirq % CMR_EVT_PER_REG) * CMR_EVT_MAP_BITS);
		pruss_intc_write_reg(intc, PRU_INTC_CMR(idx), val);

		dev_dbg(dev, "SYSEV%lu -> CH%d (CMR%d 0x%08x)\n", hwirq, ch,
			idx, pruss_intc_read_reg(intc, PRU_INTC_CMR(idx)));

		/* clear and enable system event */
		pruss_intc_write_reg(intc, PRU_INTC_SICR, hwirq);
		pruss_intc_write_reg(intc, PRU_INTC_EISR, hwirq);
	}

	if (++intc->channel_host[ch].ref_count == 1) {
		intc->channel_host[ch].value = host;

		/*
		 * set host map registers - each register holds map info for
		 * 4 channels, with each channel occupying the lower nibble in
		 * a register byte address in little-endian fashion
		 */
		idx = ch / HMR_CH_PER_REG;

		val = pruss_intc_read_reg(intc, PRU_INTC_HMR(idx));
		val &= ~(HMR_CH_MAP_MASK <<
				((ch % HMR_CH_PER_REG) * HMR_CH_MAP_BITS));
		val |= host << ((ch % HMR_CH_PER_REG) * HMR_CH_MAP_BITS);
		pruss_intc_write_reg(intc, PRU_INTC_HMR(idx), val);

		dev_dbg(dev, "CH%d -> HOST%d (HMR%d 0x%08x)\n", ch, host, idx,
			pruss_intc_read_reg(intc, PRU_INTC_HMR(idx)));

		/* enable host interrupts */
		pruss_intc_write_reg(intc, PRU_INTC_HIEISR, host);
	}

	dev_info(dev, "mapped system_event = %lu channel = %d host = %d domain = %u\n",
		 hwirq, ch, host, domain->id);

	/* global interrupt enable */
	pruss_intc_write_reg(intc, PRU_INTC_GER, 1);

	mutex_unlock(&intc->lock);
	return 0;

unlock:
	mutex_unlock(&intc->lock);
	return ret;
}

/**
 * pruss_intc_unmap() - unconfigure the PRUSS INTC
 * @domain: pru intc domain pointer
 * @hwirq: the system event number
 *
 * Undo whatever was done in pruss_intc_map() for a PRU core.
 * Mappings are reference counted, so resources are only disabled when there
 * are no longer any users.
 */
static void pruss_intc_unmap(struct pruss_intc_domain *domain, unsigned long hwirq)
{
	struct pruss_intc *intc = domain->intc;
	struct device* dev = intc->irqchip->parent_device;
	u8 ch, host;

	if (hwirq >= MAX_PRU_SYS_EVENTS)
		return;

	mutex_lock(&intc->lock);

	ch = intc->event_channel[hwirq].value;
	host = intc->channel_host[ch].value;

	if (--intc->channel_host[ch].ref_count == 0) {
		/* disable host interrupts */
		pruss_intc_write_reg(intc, PRU_INTC_HIDISR, host);
	}

	if (--intc->event_channel[hwirq].ref_count == 0) {
		/* disable system events */
		pruss_intc_write_reg(intc, PRU_INTC_EICR, hwirq);
		/* clear any pending status */
		pruss_intc_write_reg(intc, PRU_INTC_SICR, hwirq);
	}

	if (intc->has_evtsel)
		intc->evtsel.ref_count--;

	dev_info(dev, "unmapped system_event = %lu channel = %d host = %d\n",
		 hwirq, ch, host);

	mutex_unlock(&intc->lock);
}

static void pruss_intc_init(struct pruss_intc *intc)
{
	int i;

	/* configure polarity to active high for all system interrupts */
	pruss_intc_write_reg(intc, PRU_INTC_SIPR0, 0xffffffff);
	pruss_intc_write_reg(intc, PRU_INTC_SIPR1, 0xffffffff);

	/* configure type to pulse interrupt for all system interrupts */
	pruss_intc_write_reg(intc, PRU_INTC_SITR0, 0);
	pruss_intc_write_reg(intc, PRU_INTC_SITR1, 0);

	/* clear all 16 interrupt channel map registers */
	for (i = 0; i < 16; i++)
		pruss_intc_write_reg(intc, PRU_INTC_CMR(i), 0);

	/* clear all 3 host interrupt map registers */
	for (i = 0; i < 3; i++)
		pruss_intc_write_reg(intc, PRU_INTC_HMR(i), 0);
}

static void pruss_intc_irq_ack(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc, PRU_INTC_SICR, hwirq);
}

static void pruss_intc_irq_mask(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc, PRU_INTC_EICR, hwirq);
}

static void pruss_intc_irq_unmask(struct irq_data *data)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	unsigned int hwirq = data->hwirq;

	pruss_intc_check_write(intc, PRU_INTC_EISR, hwirq);
}

static int pruss_intc_irq_reqres(struct irq_data *data)
{
	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return 0;
}

static void pruss_intc_irq_relres(struct irq_data *data)
{
	module_put(THIS_MODULE);
}

static int pruss_intc_irq_get_irqchip_state(struct irq_data *data,
					    enum irqchip_irq_state which,
					    bool *state)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);
	u32 reg, mask, srsr;

	if (which != IRQCHIP_STATE_PENDING)
		return -EINVAL;

	reg = PRU_INTC_SRSR(data->hwirq / 32);
	mask = BIT(data->hwirq % 32);

	srsr = pruss_intc_read_reg(intc, reg);

	*state = !!(srsr & mask);

	return 0;
}

static int pruss_intc_irq_set_irqchip_state(struct irq_data *data,
					    enum irqchip_irq_state which,
					    bool state)
{
	struct pruss_intc *intc = irq_data_get_irq_chip_data(data);

	if (which != IRQCHIP_STATE_PENDING)
		return -EINVAL;

	if (state)
		return pruss_intc_check_write(intc, PRU_INTC_SISR, data->hwirq);

	return pruss_intc_check_write(intc, PRU_INTC_SICR, data->hwirq);
}

static int pruss_intc_irq_domain_select(struct irq_domain *d,
					struct irq_fwspec *fwspec,
					enum irq_domain_bus_token bus_token)
{
	struct pruss_intc_domain *domain = d->host_data;
	int num_cells = domain->intc->has_evtsel ? 5 : 4;
	u32 domain_id;

	if (!fwspec || fwspec->fwnode != domain->domain->fwnode)
		return 0;

	if (bus_token != DOMAIN_BUS_ANY && bus_token != domain->domain->bus_token)
		return 0;

	if (WARN_ON_ONCE(fwspec->param_count != num_cells))
		return 0;

	domain_id = fwspec->param[fwspec->param_count - 1];
	if (domain_id != domain->id)
		return 0;

	return 1;
}

static int
pruss_intc_irq_domain_xlate(struct irq_domain *d, struct device_node *node,
			    const u32 *intspec, unsigned int intsize,
			    unsigned long *out_hwirq, unsigned int *out_type)
{
	struct pruss_intc_domain *domain = d->host_data;
	struct pruss_intc *intc = domain->intc;
	int num_cells = intc->has_evtsel ? 5 : 4;
	u32 sys_event, channel, host, domain_id;
	u32 evtsel = 0;

	if (WARN_ON_ONCE(intsize != num_cells))
		return -EINVAL;

	sys_event = intspec[0];
	if (sys_event >= MAX_PRU_SYS_EVENTS)
		return -EINVAL;

	if (intc->has_evtsel)
		evtsel = intspec[1];

	channel = intspec[intsize - 3];
	if (channel >= MAX_PRU_CHANNELS)
		return -EINVAL;

	host = intspec[intsize - 2];
	if (host >= MAX_PRU_HOST_INT)
		return -EINVAL;

	domain_id = intspec[intsize - 1];
	if (domain_id != domain->id)
		return -EINVAL;

	domain->hwirq_data[sys_event].evtsel = evtsel;
	domain->hwirq_data[sys_event].channel = channel;
	domain->hwirq_data[sys_event].host = host;

	*out_hwirq = sys_event;
	*out_type = IRQ_TYPE_NONE;

	return 0;
}

static int pruss_intc_irq_domain_map(struct irq_domain *d, unsigned int virq,
				     irq_hw_number_t hw)
{
	struct pruss_intc_domain *domain = d->host_data;
	struct pruss_intc *intc = domain->intc;
	int err;

	err = pruss_intc_map(domain, hw);
	if (err < 0)
		return err;

	irq_set_chip_data(virq, intc);
	irq_set_chip_and_handler(virq, intc->irqchip, handle_level_irq);

	return 0;
}

static void pruss_intc_irq_domain_unmap(struct irq_domain *d, unsigned int virq)
{
	struct pruss_intc_domain *domain = d->host_data;
	unsigned long hwirq = irqd_to_hwirq(irq_get_irq_data(virq));

	irq_set_chip_and_handler(virq, NULL, NULL);
	irq_set_chip_data(virq, NULL);
	pruss_intc_unmap(domain, hwirq);
}

static const struct irq_domain_ops pruss_intc_irq_domain_ops = {
	.select	= pruss_intc_irq_domain_select,
	.xlate	= pruss_intc_irq_domain_xlate,
	.map	= pruss_intc_irq_domain_map,
	.unmap	= pruss_intc_irq_domain_unmap,
};

static void pruss_intc_irq_handler(struct irq_desc *desc)
{
	unsigned int irq = irq_desc_get_irq(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pruss_intc *intc = irq_get_handler_data(irq);
	u32 hipir;
	unsigned int virq;
	int i, hwirq;

	chained_irq_enter(chip, desc);

	/* find our host irq number */
	for (i = 0; i < MAX_NUM_HOST_IRQS; i++)
		if (intc->irqs[i] == irq)
			break;
	if (i == MAX_NUM_HOST_IRQS)
		goto err;

	i += MIN_PRU_HOST_INT;

	/* get highest priority pending PRUSS system event */
	hipir = pruss_intc_read_reg(intc, PRU_INTC_HIPIR(i));
	while (!(hipir & INTC_HIPIR_NONE_HINT)) {
		hwirq = hipir & GENMASK(9, 0);
		virq = irq_linear_revmap(
			intc->domain[TI_PRUSS_INTC_DOMAIN_MCU].domain, hwirq);

		/*
		 * NOTE: manually ACK any system events that do not have a
		 * handler mapped yet
		 */
		if (unlikely(!virq))
			pruss_intc_check_write(intc, PRU_INTC_SICR, hwirq);
		else
			generic_handle_irq(virq);

		/* get next system event */
		hipir = pruss_intc_read_reg(intc, PRU_INTC_HIPIR(i));
	}
err:
	chained_irq_exit(chip, desc);
}

static int pruss_intc_probe(struct platform_device *pdev)
{
	static const char * const irq_names[] = {
		"host_intr0", "host_intr1", "host_intr2", "host_intr3",
		"host_intr4", "host_intr5", "host_intr6", "host_intr7", };
	struct device *dev = &pdev->dev;
	struct pruss_intc *intc;
	struct resource *res;
	struct irq_chip *irqchip;
	int i, err, irq, count;
	u32 num_cells;
	u8 temp_intr[MAX_NUM_HOST_IRQS] = { 0 };

	intc = devm_kzalloc(dev, sizeof(*intc), GFP_KERNEL);
	if (!intc)
		return -ENOMEM;
	platform_set_drvdata(pdev, intc);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	intc->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(intc->base)) {
		dev_err(dev, "failed to parse and map intc memory resource\n");
		return PTR_ERR(intc->base);
	}

	dev_dbg(dev, "intc memory: pa %pa size 0x%zx va %pK\n", &res->start,
		(size_t)resource_size(res), intc->base);

	count = of_property_read_variable_u8_array(dev->of_node,
						   "ti,irqs-reserved",
						   temp_intr, 0,
						   MAX_NUM_HOST_IRQS);
	if (count < 0 && count != -EINVAL)
		return count;
	count = (count == -EINVAL ? 0 : count);
	for (i = 0; i < count; i++) {
		if (temp_intr[i] < MAX_NUM_HOST_IRQS) {
			intc->invalid_intr |= BIT(temp_intr[i]);
		} else {
			dev_warn(dev, "ignoring invalid reserved irq %d\n",
				 temp_intr[i]);
		}
		temp_intr[i] = 0;
	}

	count = of_property_read_variable_u8_array(dev->of_node,
						   "ti,irqs-shared",
						   temp_intr, 0,
						   MAX_NUM_HOST_IRQS);
	if (count < 0 && count != -EINVAL)
		return count;
	count = (count == -EINVAL ? 0 : count);
	for (i = 0; i < count; i++) {
		if (temp_intr[i] < MAX_NUM_HOST_IRQS) {
			intc->shared_intr |= BIT(temp_intr[i]);
		} else {
			dev_warn(dev, "ignoring invalid shared irq %d\n",
				 temp_intr[i]);
		}
	}

	err = of_property_read_u32(dev->of_node, "#interrupt-cells", &num_cells);
	if (!err && num_cells == 5)
		intc->has_evtsel = true;

	mutex_init(&intc->lock);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	pruss_intc_init(intc);

	irqchip = devm_kzalloc(dev, sizeof(*irqchip), GFP_KERNEL);
	if (!irqchip) {
		err = -ENOMEM;
		goto fail_alloc;
	}

	irqchip->irq_ack = pruss_intc_irq_ack;
	irqchip->irq_mask = pruss_intc_irq_mask;
	irqchip->irq_unmask = pruss_intc_irq_unmask;
	irqchip->irq_request_resources = pruss_intc_irq_reqres;
	irqchip->irq_release_resources = pruss_intc_irq_relres;
	irqchip->irq_get_irqchip_state = pruss_intc_irq_get_irqchip_state;
	irqchip->irq_set_irqchip_state = pruss_intc_irq_set_irqchip_state;
	irqchip->parent_device = dev;
	irqchip->name = dev_name(dev);
	intc->irqchip = irqchip;

	for (i = 0; i < NUM_TI_PRUSS_INTC_DOMAIN; i++) {
		intc->domain[i].intc = intc;
		intc->domain[i].id = i;
		/* always 64 events */
		intc->domain[i].domain = irq_domain_add_linear(dev->of_node,
				MAX_PRU_SYS_EVENTS, &pruss_intc_irq_domain_ops,
				&intc->domain[i]);
		if (!intc->domain[i].domain) {
			while (--i >= 0)
				irq_domain_remove(intc->domain[i].domain);
			err = -ENOMEM;
			goto fail_alloc;
		}
	}

	for (i = 0; i < MAX_NUM_HOST_IRQS; i++) {
		irq = platform_get_irq_byname(pdev, irq_names[i]);
		if (irq < 0) {
			if (intc->shared_intr & BIT(i) ||
			    intc->invalid_intr & BIT(i))
				continue;

			dev_err(dev, "platform_get_irq_byname failed for %s : %d\n",
				irq_names[i], irq);
			err = irq;
			goto fail_irq;
		}

		intc->irqs[i] = irq;
		irq_set_handler_data(irq, intc);
		irq_set_chained_handler(irq, pruss_intc_irq_handler);
	}

	return 0;

fail_irq:
	while (--i >= 0) {
		if (intc->irqs[i])
			irq_set_chained_handler_and_data(intc->irqs[i], NULL,
							 NULL);
	}
	for (i = 0; i < NUM_TI_PRUSS_INTC_DOMAIN; i++)
		irq_domain_remove(intc->domain[i].domain);

fail_alloc:
	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return err;
}

static int pruss_intc_remove(struct platform_device *pdev)
{
	struct pruss_intc *intc = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;
	unsigned int hwirq;
	int i;

	for (i = 0; i < MAX_NUM_HOST_IRQS; i++) {
		if (intc->irqs[i])
			irq_set_chained_handler_and_data(intc->irqs[i], NULL,
							 NULL);
	}

	for (i = 0; i < NUM_TI_PRUSS_INTC_DOMAIN; i++) {
		for (hwirq = 0; hwirq < MAX_PRU_SYS_EVENTS; hwirq++)
			irq_dispose_mapping(irq_find_mapping(
					    intc->domain[i].domain, hwirq));
		irq_domain_remove(intc->domain[i].domain);
	}

	pm_runtime_put(dev);
	pm_runtime_disable(dev);

	return 0;
}

static const struct of_device_id pruss_intc_of_match[] = {
	{ .compatible = "ti,pruss-intc", },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, pruss_intc_of_match);

static struct platform_driver pruss_intc_driver = {
	.driver = {
		.name = "pruss-intc",
		.of_match_table = pruss_intc_of_match,
	},
	.probe  = pruss_intc_probe,
	.remove = pruss_intc_remove,
};
module_platform_driver(pruss_intc_driver);

MODULE_AUTHOR("Andrew F. Davis <afd@ti.com>");
MODULE_AUTHOR("Suman Anna <s-anna@ti.com>");
MODULE_DESCRIPTION("TI PRU-ICSS INTC Driver");
MODULE_LICENSE("GPL v2");
