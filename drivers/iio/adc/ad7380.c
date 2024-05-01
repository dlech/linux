// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD738x Simultaneous Sampling SAR ADCs
 *
 * Copyright 2017 Analog Devices Inc.
 * Copyright 2024 BayLibre, SAS
 *
 * Datasheets of supported parts:
 * ad7380/1 : https://www.analog.com/media/en/technical-documentation/data-sheets/AD7380-7381.pdf
 * ad7383/4 : https://www.analog.com/media/en/technical-documentation/data-sheets/ad7383-7384.pdf
 * ad7380-4 : https://www.analog.com/media/en/technical-documentation/data-sheets/ad7380-4.pdf
 * ad7381-4 : https://www.analog.com/media/en/technical-documentation/data-sheets/ad7381-4.pdf
 * ad7383/4-4 : https://www.analog.com/media/en/technical-documentation/data-sheets/ad7383-4-ad7384-4.pdf
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/units.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define MAX_NUM_CHANNELS		4
/* 2.5V internal reference voltage */
#define AD7380_INTERNAL_REF_MV		2500

/* reading and writing registers is more reliable at lower than max speed */
#define AD7380_REG_WR_SPEED_HZ		10000000

#define AD7380_REG_WR			BIT(15)
#define AD7380_REG_REGADDR		GENMASK(14, 12)
#define AD7380_REG_DATA			GENMASK(11, 0)

#define AD7380_REG_ADDR_NOP		0x0
#define AD7380_REG_ADDR_CONFIG1		0x1
#define AD7380_REG_ADDR_CONFIG2		0x2
#define AD7380_REG_ADDR_ALERT		0x3
#define AD7380_REG_ADDR_ALERT_LOW_TH	0x4
#define AD7380_REG_ADDR_ALERT_HIGH_TH	0x5

#define AD7380_CONFIG1_OS_MODE		BIT(9)
#define OS_MODE_NORMAL_AVERAGE		0
#define OS_MODE_ROLLING_AVERAGE		1
#define AD7380_CONFIG1_OSR		GENMASK(8, 6)
#define AD7380_CONFIG1_CRC_W		BIT(5)
#define AD7380_CONFIG1_CRC_R		BIT(4)
#define AD7380_CONFIG1_ALERTEN		BIT(3)
#define AD7380_CONFIG1_RES		BIT(2)
#define RESOLUTION_BOOST_DISABLE	0
#define RESOLUTION_BOOST_ENABLE		1
#define AD7380_CONFIG1_REFSEL		BIT(1)
#define AD7380_CONFIG1_PMODE		BIT(0)

#define AD7380_CONFIG2_SDO2		GENMASK(9, 8)
#define AD7380_CONFIG2_SDO		BIT(8)
#define AD7380_CONFIG2_RESET		GENMASK(7, 0)

#define AD7380_CONFIG2_RESET_SOFT	0x3C
#define AD7380_CONFIG2_RESET_HARD	0xFF

#define AD7380_ALERT_LOW_TH		GENMASK(11, 0)
#define AD7380_ALERT_HIGH_TH		GENMASK(11, 0)

#define T_CONVERT_NS 190		/* conversion time */
struct ad7380_timing_specs {
	const unsigned int t_csh_ns;	/* CS minimum high time */
};

struct ad7380_chip_info {
	const char *name;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	const char * const *vcm_supplies;
	unsigned int num_vcm_supplies;
	const unsigned long *available_scan_masks;
	const struct ad7380_timing_specs *timing_specs;
};

/*
 * realbits/storagebits cannot be dynamically changed, so in order to
 * support the resolution boost (additional 2  bits of resolution)
 * we need to set realbits/storagebits to the maximum value i.e :
 *   - realbits = 16 + 2 = 18, and storagebits = 32 for 16-bits chips
 *   - realbits = 14 + 2 = 16, and storagebits = 16 for 14-bits chips
 * We need to adjust the scale depending on resolution boost status
 */
#define AD7380_CHANNEL(index, bits, diff) {			\
	.type = IIO_VOLTAGE,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		((diff) ? 0 : BIT(IIO_CHAN_INFO_OFFSET)),	\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |	\
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.info_mask_shared_by_type_available =			\
		BIT(IIO_CHAN_INFO_SCALE) |			\
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.indexed = 1,						\
	.differential = (diff),					\
	.channel = (diff) ? (2 * (index)) : (index),		\
	.channel2 = (diff) ? (2 * (index) + 1) : 0,		\
	.scan_index = (index),					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = (bits) + 2,				\
		.storagebits = ((bits) + 2 > 16) ? 32 : 16,	\
		.endianness = IIO_CPU,				\
	},							\
}

#define DEFINE_AD7380_2_CHANNEL(name, bits, diff)	\
static const struct iio_chan_spec name[] = {		\
	AD7380_CHANNEL(0, bits, diff),			\
	AD7380_CHANNEL(1, bits, diff),			\
	IIO_CHAN_SOFT_TIMESTAMP(2),			\
}

#define DEFINE_AD7380_4_CHANNEL(name, bits, diff)	\
static const struct iio_chan_spec name[] = {		\
	AD7380_CHANNEL(0, bits, diff),			\
	AD7380_CHANNEL(1, bits, diff),			\
	AD7380_CHANNEL(2, bits, diff),			\
	AD7380_CHANNEL(3, bits, diff),			\
	IIO_CHAN_SOFT_TIMESTAMP(4),			\
}

/* fully differential */
DEFINE_AD7380_2_CHANNEL(ad7380_channels, 16, 1);
DEFINE_AD7380_2_CHANNEL(ad7381_channels, 14, 1);
DEFINE_AD7380_4_CHANNEL(ad7380_4_channels, 16, 1);
DEFINE_AD7380_4_CHANNEL(ad7381_4_channels, 14, 1);
/* pseudo differential */
DEFINE_AD7380_2_CHANNEL(ad7383_channels, 16, 0);
DEFINE_AD7380_2_CHANNEL(ad7384_channels, 14, 0);
DEFINE_AD7380_4_CHANNEL(ad7383_4_channels, 16, 0);
DEFINE_AD7380_4_CHANNEL(ad7384_4_channels, 14, 0);

static const char * const ad7380_2_channel_vcm_supplies[] = {
	"aina", "ainb",
};

static const char * const ad7380_4_channel_vcm_supplies[] = {
	"aina", "ainb", "ainc", "aind",
};

/* Since this is simultaneous sampling, we don't allow individual channels. */
static const unsigned long ad7380_2_channel_scan_masks[] = {
	GENMASK(1, 0),
	0
};

static const unsigned long ad7380_4_channel_scan_masks[] = {
	GENMASK(3, 0),
	0
};

static const struct ad7380_timing_specs ad7380_timing = {
	.t_csh_ns = 10,
};

static const struct ad7380_timing_specs ad7380_4_timing = {
	.t_csh_ns = 20,
};

/*
 * Available oversampling modes.
 */
static const char * const ad7380_oversampling_average_modes[] = {
	[OS_MODE_NORMAL_AVERAGE]	= "normal",
	[OS_MODE_ROLLING_AVERAGE]	= "rolling",
};

/*
 * Available oversampling ratios. The indices correspond
 * with the bit value expected by the chip.
 * The available ratios depend on the averaging mode.
 */
static const int ad7380_normal_average_oversampling_ratios[] = {
	1, 2, 4, 8, 16, 32,
};

static const int ad7380_rolling_average_oversampling_ratios[] = {
	1, 2, 4, 8,
};

static const struct ad7380_chip_info ad7380_chip_info = {
	.name = "ad7380",
	.channels = ad7380_channels,
	.num_channels = ARRAY_SIZE(ad7380_channels),
	.available_scan_masks = ad7380_2_channel_scan_masks,
	.timing_specs = &ad7380_timing,
};

static const struct ad7380_chip_info ad7381_chip_info = {
	.name = "ad7381",
	.channels = ad7381_channels,
	.num_channels = ARRAY_SIZE(ad7381_channels),
	.available_scan_masks = ad7380_2_channel_scan_masks,
	.timing_specs = &ad7380_timing,
};

static const struct ad7380_chip_info ad7383_chip_info = {
	.name = "ad7383",
	.channels = ad7383_channels,
	.num_channels = ARRAY_SIZE(ad7383_channels),
	.vcm_supplies = ad7380_2_channel_vcm_supplies,
	.num_vcm_supplies = ARRAY_SIZE(ad7380_2_channel_vcm_supplies),
	.available_scan_masks = ad7380_2_channel_scan_masks,
	.timing_specs = &ad7380_timing,
};

static const struct ad7380_chip_info ad7384_chip_info = {
	.name = "ad7384",
	.channels = ad7384_channels,
	.num_channels = ARRAY_SIZE(ad7384_channels),
	.vcm_supplies = ad7380_2_channel_vcm_supplies,
	.num_vcm_supplies = ARRAY_SIZE(ad7380_2_channel_vcm_supplies),
	.available_scan_masks = ad7380_2_channel_scan_masks,
	.timing_specs = &ad7380_timing,
};

static const struct ad7380_chip_info ad7380_4_chip_info = {
	.name = "ad7380-4",
	.channels = ad7380_4_channels,
	.num_channels = ARRAY_SIZE(ad7380_4_channels),
	.available_scan_masks = ad7380_4_channel_scan_masks,
	.timing_specs = &ad7380_4_timing,
};

static const struct ad7380_chip_info ad7381_4_chip_info = {
	.name = "ad7381-4",
	.channels = ad7381_4_channels,
	.num_channels = ARRAY_SIZE(ad7381_4_channels),
	.available_scan_masks = ad7380_4_channel_scan_masks,
	.timing_specs = &ad7380_4_timing,
};

static const struct ad7380_chip_info ad7383_4_chip_info = {
	.name = "ad7383-4",
	.channels = ad7383_4_channels,
	.num_channels = ARRAY_SIZE(ad7383_4_channels),
	.vcm_supplies = ad7380_4_channel_vcm_supplies,
	.num_vcm_supplies = ARRAY_SIZE(ad7380_4_channel_vcm_supplies),
	.available_scan_masks = ad7380_4_channel_scan_masks,
	.timing_specs = &ad7380_4_timing,
};

static const struct ad7380_chip_info ad7384_4_chip_info = {
	.name = "ad7384-4",
	.channels = ad7384_4_channels,
	.num_channels = ARRAY_SIZE(ad7384_4_channels),
	.vcm_supplies = ad7380_4_channel_vcm_supplies,
	.num_vcm_supplies = ARRAY_SIZE(ad7380_4_channel_vcm_supplies),
	.available_scan_masks = ad7380_4_channel_scan_masks,
	.timing_specs = &ad7380_4_timing,
};

struct ad7380_state {
	const struct ad7380_chip_info *chip_info;
	struct spi_device *spi;
	unsigned int oversampling_mode;
	unsigned int oversampling_ratio;
	unsigned int scales[2][2];
	bool resolution_boost_enable;
	struct regmap *regmap;
	unsigned int vref_mv;
	unsigned int vcm_mv[MAX_NUM_CHANNELS];
	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 * Make the buffer large enough for MAX_NUM_CHANNELS 16-bit samples and one 64-bit
	 * aligned 64 bit timestamp.
	 * As MAX_NUM_CHANNELS is 4 the layout of the structure is the same for all parts
	 */
	struct {
		union {
			u16 u16[MAX_NUM_CHANNELS];
			u32 u32[MAX_NUM_CHANNELS];
		} raw;

		s64 ts __aligned(8);
	} scan_data __aligned(IIO_DMA_MINALIGN);
	u16 tx;
	u16 rx;
};

static int ad7380_regmap_reg_write(void *context, unsigned int reg,
				   unsigned int val)
{
	struct ad7380_state *st = context;
	struct spi_transfer xfer = {
		.speed_hz = AD7380_REG_WR_SPEED_HZ,
		.bits_per_word = 16,
		.len = 2,
		.tx_buf = &st->tx,
	};

	st->tx = FIELD_PREP(AD7380_REG_WR, 1) |
		 FIELD_PREP(AD7380_REG_REGADDR, reg) |
		 FIELD_PREP(AD7380_REG_DATA, val);

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad7380_regmap_reg_read(void *context, unsigned int reg,
				  unsigned int *val)
{
	struct ad7380_state *st = context;
	struct spi_transfer xfers[] = {
		{
			.speed_hz = AD7380_REG_WR_SPEED_HZ,
			.bits_per_word = 16,
			.len = 2,
			.tx_buf = &st->tx,
			.cs_change = 1,
			.cs_change_delay = {
				.value = st->chip_info->timing_specs->t_csh_ns,
				.unit = SPI_DELAY_UNIT_NSECS,
			},
		}, {
			.speed_hz = AD7380_REG_WR_SPEED_HZ,
			.bits_per_word = 16,
			.len = 2,
			.rx_buf = &st->rx,
		},
	};
	int ret;

	st->tx = FIELD_PREP(AD7380_REG_WR, 0) |
		 FIELD_PREP(AD7380_REG_REGADDR, reg) |
		 FIELD_PREP(AD7380_REG_DATA, 0);

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret < 0)
		return ret;

	*val = FIELD_GET(AD7380_REG_DATA, st->rx);

	return 0;
}

static const struct regmap_config ad7380_regmap_config = {
	.reg_bits = 3,
	.val_bits = 12,
	.reg_read = ad7380_regmap_reg_read,
	.reg_write = ad7380_regmap_reg_write,
	.max_register = AD7380_REG_ADDR_ALERT_HIGH_TH,
	.can_sleep = true,
};

static int ad7380_debugfs_reg_access(struct iio_dev *indio_dev, u32 reg,
				     u32 writeval, u32 *readval)
{
	iio_device_claim_direct_scoped(return  -EBUSY, indio_dev) {
		struct ad7380_state *st = iio_priv(indio_dev);
		int ret;

		if (readval)
			ret = regmap_read(st->regmap, reg, readval);
		else
			ret = regmap_write(st->regmap, reg, writeval);

		return ret;
	}
	unreachable();
}

static int ad7380_prepare_spi_xfer(struct ad7380_state *st, struct spi_transfer *xfer)
{
	int bits_per_word;

	memset(xfer, 0, sizeof(*xfer));

	xfer->rx_buf = &st->scan_data.raw;

	if (st->resolution_boost_enable == RESOLUTION_BOOST_ENABLE)
		bits_per_word = st->chip_info->channels[0].scan_type.realbits;
	else
		bits_per_word = st->chip_info->channels[0].scan_type.realbits - 2;

	xfer->bits_per_word = bits_per_word;

	xfer->len = (st->chip_info->num_channels - 1) * BITS_TO_BYTES(bits_per_word);

	return bits_per_word;
}

static irqreturn_t ad7380_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7380_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer;
	int bits_per_word, realbits, i, ret;

	realbits = st->chip_info->channels[0].scan_type.realbits;
	bits_per_word = ad7380_prepare_spi_xfer(st, &xfer);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		goto out;

	/*
	 * If bits_per_word == realbits (resolution boost enabled), we don't
	 * need to manipulate the raw data, otherwise we may need to fix things
	 * up a bit to fit the scan_type specs
	 */
	if (bits_per_word < realbits) {
		if (realbits > 16 && bits_per_word <= 16) {
			/*
			 * Here realbits > 16 so storagebits is 32 and bits_per_word is <= 16
			 * so we need to sign extend u16 to u32 using reverse order to
			 * avoid writing over union data
			 */
			for (i = st->chip_info->num_channels - 2; i >= 0; i--)
				st->scan_data.raw.u32[i] = sign_extend32(st->scan_data.raw.u16[i],
									 bits_per_word - 1);
		} else if (bits_per_word < 16) {
			/*
			 * Here realbits <= 16 so storagebits is 16.
			 * We only need to sign extend only if bits_per_word is < 16
			 */
			for (i = 0; i < st->chip_info->num_channels - 1; i++)
				st->scan_data.raw.u16[i] = sign_extend32(st->scan_data.raw.u16[i],
									 bits_per_word - 1);
		}
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &st->scan_data,
					   pf->timestamp);

out:
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static int ad7380_read_direct(struct ad7380_state *st,
			      struct iio_chan_spec const *chan, int *val)
{
	struct spi_transfer xfers[2] = {
		/* toggle CS (no data xfer) to trigger a conversion */
		{
			.speed_hz = AD7380_REG_WR_SPEED_HZ,
			.bits_per_word = chan->scan_type.realbits,
			.delay = {
				.value = T_CONVERT_NS,
				.unit = SPI_DELAY_UNIT_NSECS,
			},
			.cs_change = 1,
			.cs_change_delay = {
				.value = st->chip_info->timing_specs->t_csh_ns,
				.unit = SPI_DELAY_UNIT_NSECS,
			},
		},
		/* then read all channels, it will be filled by ad7380_prepare_spi_xfer */
		{
		},
	};
	int bits_per_word, ret;

	/*
	 * In normal average oversampling we need to wait for multiple conversions to be done
	 */
	if (st->oversampling_mode == OS_MODE_NORMAL_AVERAGE && st->oversampling_ratio > 1)
		xfers[0].delay.value = T_CONVERT_NS + 500 * st->oversampling_ratio;

	bits_per_word = ad7380_prepare_spi_xfer(st, &xfers[1]);

	ret = spi_sync_transfer(st->spi, xfers, ARRAY_SIZE(xfers));
	if (ret < 0)
		return ret;

	if (bits_per_word > 16)
		*val = sign_extend32(st->scan_data.raw.u32[chan->scan_index],
				     bits_per_word - 1);
	else
		*val = sign_extend32(st->scan_data.raw.u16[chan->scan_index],
				     bits_per_word - 1);

	return IIO_VAL_INT;
}

static int ad7380_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad7380_state *st = iio_priv(indio_dev);
	int realbits;

	if (st->resolution_boost_enable == RESOLUTION_BOOST_ENABLE)
		realbits = chan->scan_type.realbits;
	else
		realbits = chan->scan_type.realbits - 2;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
			return ad7380_read_direct(st, chan, val);
		}
		unreachable();
	case IIO_CHAN_INFO_SCALE:
		*val = st->scales[st->resolution_boost_enable][0];
		*val2 = st->scales[st->resolution_boost_enable][1];

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		/*
		 * According to IIO ABI, offset is applied before scale,
		 * so offset is: vcm_mv / scale
		 */
		*val = st->vcm_mv[chan->channel] * (1 << realbits)
			/ st->vref_mv;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = st->oversampling_ratio;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad7380_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long mask)
{
	struct ad7380_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (st->oversampling_mode) {
		case OS_MODE_NORMAL_AVERAGE:
			*vals = ad7380_normal_average_oversampling_ratios;
			*length = ARRAY_SIZE(ad7380_normal_average_oversampling_ratios);
			break;
		case OS_MODE_ROLLING_AVERAGE:
			*vals = ad7380_rolling_average_oversampling_ratios;
			*length = ARRAY_SIZE(ad7380_rolling_average_oversampling_ratios);
			break;
		default:
			return -EINVAL;
		}
		*type = IIO_VAL_INT;

		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SCALE:
		*vals = (const int *)st->scales[0];
		/*
		 * Values are stored into a 2D matrix.
		 * We have 2 available scales depending on the resolution boost status
		 * (enabled/disabled). Resolution boost can be enabled only when oversampling
		 * is enabled (i.e OSR > 1).
		 * So depending on the oversampling ratio we display only the currently
		 * available scales. First scale is for normal mode, second scale is for resolution
		 * boost enabled.
		 */
		if (st->oversampling_ratio > 1)
			*length = 4;
		else
			*length = 2;
		*type = IIO_VAL_INT_PLUS_MICRO;

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

/**
 * check_osr - Check the oversampling ratio
 * @available_ratio: available ratios's array
 * @size: size of the available_ratio array
 * ratio: ratio to check
 *
 * Check if ratio is present in @available_ratio. Check for exact match.
 * @available_ratio is an array of the available ratios (depending on the oversampling mode).
 * The indices must correspond with the bit value expected by the chip.
 */
static inline int check_osr(const int *available_ratio, int size, int ratio)
{
	int i;

	for (i = 0; i < size; i++) {
		if (ratio == available_ratio[i])
			return i;
	}

	return -EINVAL;
}

static int ad7380_set_resolution_boost(struct iio_dev *indio_dev, bool enable)
{
	struct ad7380_state *st = iio_priv(indio_dev);
	int ret;

	if (st->resolution_boost_enable == enable)
		return 0;

	ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG1,
				 AD7380_CONFIG1_RES,
				 FIELD_PREP(AD7380_CONFIG1_RES, enable));

	if (ret)
		return ret;

	st->resolution_boost_enable = enable;
	return 0;
}

static int ad7380_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long mask)
{
	struct ad7380_state *st = iio_priv(indio_dev);
	int ret, osr;

	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		switch (st->oversampling_mode) {
		case OS_MODE_NORMAL_AVERAGE:
			osr = check_osr(ad7380_normal_average_oversampling_ratios,
					ARRAY_SIZE(ad7380_normal_average_oversampling_ratios),
					val);
			break;
		case OS_MODE_ROLLING_AVERAGE:
			osr = check_osr(ad7380_rolling_average_oversampling_ratios,
					ARRAY_SIZE(ad7380_rolling_average_oversampling_ratios),
					val);
			break;
		default:
			return -EINVAL;
		}

		if (osr < 0)
			return osr;

		iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
			ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG1,
						 AD7380_CONFIG1_OSR,
						 FIELD_PREP(AD7380_CONFIG1_OSR, osr));

			if (ret)
				return ret;

			st->oversampling_ratio = val;

			/*
			 * If oversampling is disabled (OSR == 1),
			 * we need to disable resolution boost
			 */
			if (st->oversampling_ratio == 1)
				ad7380_set_resolution_boost(indio_dev, RESOLUTION_BOOST_DISABLE);

			/*
			 * Perform a soft reset.
			 * This will flush the oversampling block and FIFO but will
			 * maintain the content of the configurable registers.
			 */
			ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG2,
						 AD7380_CONFIG2_RESET,
						 FIELD_PREP(AD7380_CONFIG2_RESET,
							    AD7380_CONFIG2_RESET_SOFT));
		}
		return 0;
	case IIO_CHAN_INFO_SCALE:
		if (val == st->scales[RESOLUTION_BOOST_DISABLE][0] &&
		    val2 == st->scales[RESOLUTION_BOOST_DISABLE][1]) {
			iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
				return ad7380_set_resolution_boost(indio_dev,
								   RESOLUTION_BOOST_DISABLE);
			}
			unreachable();
		}
		if (st->oversampling_ratio > 1 &&
		    val == st->scales[RESOLUTION_BOOST_ENABLE][0] &&
		    val2 == st->scales[RESOLUTION_BOOST_ENABLE][1]) {
			iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
				return ad7380_set_resolution_boost(indio_dev,
								   RESOLUTION_BOOST_ENABLE);
			}
			unreachable();
		}
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static ssize_t oversampling_mode_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct ad7380_state *st = iio_priv(dev_to_iio_dev(dev));
	unsigned int os_mode;

	os_mode = st->oversampling_mode;

	return sysfs_emit(buf, "%s\n", ad7380_oversampling_average_modes[os_mode]);
}

static ssize_t oversampling_mode_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7380_state *st = iio_priv(indio_dev);
	int os_mode, ret;

	ret = sysfs_match_string(ad7380_oversampling_average_modes, buf);
	if (ret < 0)
		return ret;

	os_mode = ret;

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG1,
					 AD7380_CONFIG1_OS_MODE,
					 FIELD_PREP(AD7380_CONFIG1_OS_MODE, os_mode));

		if (ret)
			return  ret;

		st->oversampling_mode = os_mode;

		/*
		 * Oversampling ratio depends on oversampling mode, to avoid
		 * misconfiguration when changing oversampling mode,
		 * disable oversampling by setting OSR to 0.
		 * Since we disable the oversampling, we also need to
		 * disable the resolution boost
		 */
		ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG1,
					 AD7380_CONFIG1_OSR, FIELD_PREP(AD7380_CONFIG1_OSR, 0));

		if (ret)
			return ret;

		st->oversampling_ratio = 1;
		ad7380_set_resolution_boost(indio_dev, RESOLUTION_BOOST_DISABLE);

		/*
		 * Perform a soft reset.
		 * This will flush the oversampling block and FIFO but will
		 * maintain the content of the configurable registers.
		 */
		ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG2,
					 AD7380_CONFIG2_RESET,
					 FIELD_PREP(AD7380_CONFIG2_RESET,
						    AD7380_CONFIG2_RESET_SOFT));
	}
	return ret ?: len;
}

static ssize_t oversampling_mode_available_show(struct device *dev,
						struct device_attribute *attr, char *buf)
{
	int i;
	size_t len = 0;

	for (i = 0; i < ARRAY_SIZE(ad7380_oversampling_average_modes); i++)
		len += sysfs_emit_at(buf, len, "%s ", ad7380_oversampling_average_modes[i]);

	buf[len - 1] = '\n';

	return len;
}

static IIO_DEVICE_ATTR_RW(oversampling_mode, 0);
static IIO_DEVICE_ATTR_RO(oversampling_mode_available, 0);

static struct attribute *ad7380_attributes[] = {
	&iio_dev_attr_oversampling_mode.dev_attr.attr,
	&iio_dev_attr_oversampling_mode_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad7380_attribute_group = {
	.attrs = ad7380_attributes,
};

static const struct iio_info ad7380_info = {
	.attrs = &ad7380_attribute_group,
	.read_raw = &ad7380_read_raw,
	.read_avail = &ad7380_read_avail,
	.write_raw = &ad7380_write_raw,
	.debugfs_reg_access = &ad7380_debugfs_reg_access,
};

static void ad7380_init_available_scales(struct ad7380_state *st)
{
	s64 tmp;
	int value_micro, value_int, realbits, differential;

	/*
	 * Resolution boost allow to enable 2 higher bits resolution
	 * when oversampling is enabled, so we can have only two
	 * scales depending on the resolution boost status.
	 */
	realbits = st->chip_info->channels[0].scan_type.realbits;
	differential = st->chip_info->channels[0].differential;

	/*
	 * According to the datasheet, the LSB size is:
	 *    * (2 × VREF) / 2^N, for differential chips
	 *    * VREF / 2^N, for pseudo-differential chips
	 * where N is the ADC resolution (i.e realbits)
	 */
	tmp = (s64)st->vref_mv * MEGA >> (realbits - 2 - differential);
	value_int = div_s64_rem(tmp, MEGA, &value_micro);
	st->scales[RESOLUTION_BOOST_DISABLE][0] = value_int;
	st->scales[RESOLUTION_BOOST_DISABLE][1] = value_micro;

	tmp = (s64)st->vref_mv * MEGA >> (realbits - differential);
	value_int = div_s64_rem(tmp, MEGA, &value_micro);
	st->scales[RESOLUTION_BOOST_ENABLE][0] = value_int;
	st->scales[RESOLUTION_BOOST_ENABLE][1] = value_micro;
}

static int ad7380_init(struct ad7380_state *st, struct regulator *vref)
{
	int ret;

	/* perform hard reset */
	ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG2,
				 AD7380_CONFIG2_RESET,
				 FIELD_PREP(AD7380_CONFIG2_RESET,
					    AD7380_CONFIG2_RESET_HARD));
	if (ret < 0)
		return ret;

	/* select internal or external reference voltage */
	ret = regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG1,
				 AD7380_CONFIG1_REFSEL,
				 FIELD_PREP(AD7380_CONFIG1_REFSEL,
					    vref ? 1 : 0));
	if (ret < 0)
		return ret;

	/* Disable oversampling and resolution boost by default.
	 * This are the default values after reset,
	 * so just initialize internal data
	 */
	st->oversampling_mode = OS_MODE_NORMAL_AVERAGE;
	st->oversampling_ratio = 1;
	st->resolution_boost_enable = RESOLUTION_BOOST_DISABLE;

	/* initialize available scales */
	ad7380_init_available_scales(st);

	/* SPI 1-wire mode */
	return regmap_update_bits(st->regmap, AD7380_REG_ADDR_CONFIG2,
				  AD7380_CONFIG2_SDO,
				  FIELD_PREP(AD7380_CONFIG2_SDO, 1));
}

static void ad7380_regulator_disable(void *p)
{
	regulator_disable(p);
}

static int ad7380_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad7380_state *st;
	struct regulator *vref;
	int ret, i;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return dev_err_probe(&spi->dev, -EINVAL, "missing match data\n");

	vref = devm_regulator_get_optional(&spi->dev, "refio");
	if (IS_ERR(vref)) {
		if (PTR_ERR(vref) != -ENODEV)
			return dev_err_probe(&spi->dev, PTR_ERR(vref),
					     "Failed to get refio regulator\n");

		vref = NULL;
	}

	/*
	 * If there is no REFIO supply, then it means that we are using
	 * the internal 2.5V reference, otherwise REFIO is reference voltage.
	 */
	if (vref) {
		ret = regulator_enable(vref);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(&spi->dev,
					       ad7380_regulator_disable, vref);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vref);
		if (ret < 0)
			return ret;

		st->vref_mv = ret / 1000;
	} else {
		st->vref_mv = AD7380_INTERNAL_REF_MV;
	}

	if (st->chip_info->num_vcm_supplies > ARRAY_SIZE(st->vcm_mv))
		return dev_err_probe(&spi->dev, -EINVAL,
				     "invalid number of VCM supplies\n");

	/*
	 * pseudo-differential chips have common mode supplies for the negative
	 * input pin.
	 */
	for (i = 0; i < st->chip_info->num_vcm_supplies; i++) {
		struct regulator *vcm;

		vcm = devm_regulator_get(&spi->dev,
					 st->chip_info->vcm_supplies[i]);
		if (IS_ERR(vcm))
			return dev_err_probe(&spi->dev, PTR_ERR(vcm),
					     "Failed to get %s regulator\n",
					     st->chip_info->vcm_supplies[i]);

		ret = regulator_enable(vcm);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(&spi->dev,
					       ad7380_regulator_disable, vcm);
		if (ret)
			return ret;

		ret = regulator_get_voltage(vcm);
		if (ret < 0)
			return ret;

		st->vcm_mv[i] = ret / 1000;
	}

	st->regmap = devm_regmap_init(&spi->dev, NULL, st, &ad7380_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "failed to allocate register map\n");

	indio_dev->channels = st->chip_info->channels;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->name = st->chip_info->name;
	indio_dev->info = &ad7380_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = st->chip_info->available_scan_masks;

	ret = devm_iio_triggered_buffer_setup(&spi->dev, indio_dev,
					      iio_pollfunc_store_time,
					      ad7380_trigger_handler, NULL);
	if (ret)
		return ret;

	ret = ad7380_init(st, vref);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad7380_of_match_table[] = {
	{ .compatible = "adi,ad7380", .data = &ad7380_chip_info },
	{ .compatible = "adi,ad7381", .data = &ad7381_chip_info },
	{ .compatible = "adi,ad7383", .data = &ad7383_chip_info },
	{ .compatible = "adi,ad7384", .data = &ad7384_chip_info },
	{ .compatible = "adi,ad7380-4", .data = &ad7380_4_chip_info },
	{ .compatible = "adi,ad7381-4", .data = &ad7381_4_chip_info },
	{ .compatible = "adi,ad7383-4", .data = &ad7383_4_chip_info },
	{ .compatible = "adi,ad7384-4", .data = &ad7384_4_chip_info },
	{ }
};

static const struct spi_device_id ad7380_id_table[] = {
	{ "ad7380", (kernel_ulong_t)&ad7380_chip_info },
	{ "ad7381", (kernel_ulong_t)&ad7381_chip_info },
	{ "ad7383", (kernel_ulong_t)&ad7383_chip_info },
	{ "ad7384", (kernel_ulong_t)&ad7384_chip_info },
	{ "ad7380-4", (kernel_ulong_t)&ad7380_4_chip_info },
	{ "ad7381-4", (kernel_ulong_t)&ad7381_4_chip_info },
	{ "ad7383-4", (kernel_ulong_t)&ad7383_4_chip_info },
	{ "ad7384-4", (kernel_ulong_t)&ad7384_4_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad7380_id_table);

static struct spi_driver ad7380_driver = {
	.driver = {
		.name = "ad7380",
		.of_match_table = ad7380_of_match_table,
	},
	.probe = ad7380_probe,
	.id_table = ad7380_id_table,
};
module_spi_driver(ad7380_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD738x ADC driver");
MODULE_LICENSE("GPL");
