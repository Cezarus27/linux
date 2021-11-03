// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2021 Samuel Holland <samuel@sholland.org>
//

#include <linux/completion.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_event.h>
#include <linux/regmap.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/driver.h>

#define TPS65185_REG_TMST_VALUE		0x00
#define TPS65185_REG_ENABLE		0x01
#define TPS65185_REG_VADJ		0x02
#define TPS65185_REG_VCOM1		0x03
#define TPS65185_REG_VCOM2		0x04
#define TPS65185_REG_INT_EN1		0x05
#define TPS65185_REG_INT_EN2		0x06
#define TPS65185_REG_INT1		0x07
#define TPS65185_REG_INT2		0x08
#define TPS65185_REG_UPSEQ0		0x09
#define TPS65185_REG_UPSEQ1		0x0a
#define TPS65185_REG_DWNSEQ0		0x0b
#define TPS65185_REG_DWNSEQ1		0x0c
#define TPS65185_REG_TMST1		0x0d
#define TPS65185_REG_TMST2		0x0e
#define TPS65185_REG_PG			0x0f
#define TPS65185_REG_REVID		0x10

#define TPS65185_ENABLE_ACTIVE		BIT(7)
#define TPS65185_ENABLE_STANDBY		BIT(6)
#define TPS65185_ENABLE_V3P3_EN		BIT(5)
#define TPS65185_ENABLE_VCOM_EN		BIT(4)
#define TPS65185_ENABLE_VDDH_EN		BIT(3)
#define TPS65185_ENABLE_VPOS_EN		BIT(2)
#define TPS65185_ENABLE_VEE_EN		BIT(1)
#define TPS65185_ENABLE_VNEG_EN		BIT(0)

#define TPS65185_VADJ_VSET		(0x7 << 0)

#define TPS65185_VCOM2_VCOM_MSB		BIT(0)

#define TPS65185_INT1_DTX		BIT(7)
#define TPS65185_INT1_TSD		BIT(6)
#define TPS65185_INT1_HOT		BIT(5)
#define TPS65185_INT1_TMST_HOT		BIT(4)
#define TPS65185_INT1_TMST_COLD		BIT(3)
#define TPS65185_INT1_UVLO		BIT(2)
#define TPS65185_INT1_ACQC		BIT(1)
#define TPS65185_INT1_PRGC		BIT(0)

#define TPS65185_INT2_VB_UV		BIT(7)
#define TPS65185_INT2_VDDH_UV		BIT(6)
#define TPS65185_INT2_VN_UV		BIT(5)
#define TPS65185_INT2_VPOS_UV		BIT(4)
#define TPS65185_INT2_VEE_UV		BIT(3)
#define TPS65185_INT2_VCOMF		BIT(2)
#define TPS65185_INT2_VNEG_UV		BIT(1)
#define TPS65185_INT2_EOC		BIT(0)

#define TPS65185_TMST1_READ_THERM	BIT(7)
#define TPS65185_TMST1_DT		(0x3 << 0)

#define TPS65185_TMST2_TMST_COLD	(0xf << 4)
#define TPS65185_TMST2_TMST_COLD_SHIFT	4
#define TPS65185_TMST2_TMST_HOT		(0xf << 0)
#define TPS65185_TMST2_TMST_HOT_SHIFT	0

#define TPS65185_TMST_DT_MIN_CELSIUS	2
#define TPS65185_TMST_DT_MAX_CELSIUS	5

#define TPS65185_TMST_COLD_MIN_CELSIUS	-7
#define TPS65185_TMST_HOT_MIN_CELSIUS	42

#define TPS65185_WAKEUP_DELAY_MS	2

enum {
	TPS65185_REGULATOR_V3P3,
	TPS65185_REGULATOR_VCOM,
	TPS65185_REGULATOR_VDRIVE,
	TPS65185_NUM_REGULATORS
};

struct tps65185 {
	struct gpio_desc	*wakeup_gpio;
	struct regmap		*regmap;
	struct completion	temp_completion;
	u8			int_en1;
	u8			int_en2;
};

static const struct iio_event_spec tps65185_iio_temp_events[] = {
	{
		.type			= IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir			= IIO_EV_DIR_EITHER,
		.mask_separate		= BIT(IIO_EV_INFO_VALUE) |
					  BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_chan_spec tps65185_iio_channels[] = {
	{
		.type			= IIO_TEMP,
		.info_mask_separate	= BIT(IIO_CHAN_INFO_PROCESSED),
		.event_spec		= tps65185_iio_temp_events,
		.num_event_specs	= ARRAY_SIZE(tps65185_iio_temp_events),
	},
};

static int tps65185_iio_read_temp(struct tps65185 *tps)
{
	reinit_completion(&tps->temp_completion);

	return regmap_update_bits(tps->regmap, TPS65185_REG_TMST1,
				  TPS65185_TMST1_READ_THERM,
				  TPS65185_TMST1_READ_THERM);
}

static int tps65185_iio_read_raw(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 int *val, int *val2, long mask)
{
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);
	unsigned int temp;
	int ret;

	if (mask != IIO_CHAN_INFO_PROCESSED)
		return -EINVAL;

	ret = tps65185_iio_read_temp(tps);
	if (ret)
		return ret;

	if (!wait_for_completion_timeout(&tps->temp_completion,
					 msecs_to_jiffies(10)))
		return -ETIMEDOUT;

	ret = regmap_read(tps->regmap, TPS65185_REG_TMST_VALUE, &temp);
	if (ret)
		return ret;

	*val = sign_extend32(temp, 7);

	return IIO_VAL_INT;
}

static int tps65185_read_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir)
{
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);
	u8 mask = TPS65185_INT1_DTX;

	return !!(tps->int_en1 & mask);
}

static int tps65185_write_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir, int state)
{
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);
	u8 mask = TPS65185_INT1_DTX;

	if (state)
		tps->int_en1 |= mask;
	else
		tps->int_en1 &= ~mask;

	return regmap_write(tps->regmap, TPS65185_REG_INT_EN1, tps->int_en1);
}

static int tps65185_read_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int *val, int *val2)
{
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);
	unsigned int dt;
	int ret;

	ret = regmap_read(tps->regmap, TPS65185_REG_TMST1, &dt);
	if (ret)
		return ret;

	*val = TPS65185_TMST_DT_MIN_CELSIUS + (dt & TPS65185_TMST1_DT);

	return IIO_VAL_INT;
}

static int tps65185_write_event_value(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int val, int val2)
{
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);
	unsigned int dt;

	if (val < TPS65185_TMST_DT_MIN_CELSIUS ||
	    val > TPS65185_TMST_DT_MAX_CELSIUS)
		return -EINVAL;

	dt = val - TPS65185_TMST_DT_MIN_CELSIUS;

	return regmap_update_bits(tps->regmap, TPS65185_REG_TMST1,
				  TPS65185_TMST1_DT, dt);
}

static const struct iio_info tps65185_iio_info = {
	.read_raw		= tps65185_iio_read_raw,
	.read_event_config	= tps65185_read_event_config,
	.write_event_config	= tps65185_write_event_config,
	.read_event_value	= tps65185_read_event_value,
	.write_event_value	= tps65185_write_event_value,
};

static irqreturn_t tps65185_iio_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);

	tps65185_iio_read_temp(tps);
	iio_trigger_notify_done(indio_dev->trig);

	return IRQ_HANDLED;
}

static void tps65185_iio_triggered_event_cleanup(void *indio_dev)
{
	iio_triggered_event_cleanup(indio_dev);
}

static irqreturn_t tps65185_irq_handler(int irq, void *p)
{
	struct iio_dev *indio_dev = p;
	struct tps65185 *tps = iio_device_get_drvdata(indio_dev);
	unsigned int status1, status2;
	int ret;

	ret = regmap_read(tps->regmap, TPS65185_REG_INT1, &status1);
	if (!ret && (status1 & TPS65185_INT1_DTX))
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
						    IIO_EV_TYPE_THRESH_ADAPTIVE,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(indio_dev));

	ret = regmap_read(tps->regmap, TPS65185_REG_INT2, &status2);
	if (!ret && (status2 & TPS65185_INT2_EOC))
		complete(&tps->temp_completion);

	dev_info(indio_dev->dev.parent, "status: 0x%02x 0x%02x\n", status1, status2);

	return IRQ_HANDLED;
}

static const struct regulator_ops tps65185_v3p3_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

static const struct regulator_desc tps65185_v3p3_desc = {
	.name			= "v3p3",
	.supply_name		= "vin3p3",
	.of_match		= "v3p3",
	.regulators_node	= "regulators",
	.ops			= &tps65185_v3p3_ops,
	.owner			= THIS_MODULE,
	.enable_reg		= TPS65185_REG_ENABLE,
	.enable_mask		= TPS65185_ENABLE_V3P3_EN,
};

static int tps65185_vcom_set_voltage_sel(struct regulator_dev *rdev,
					 unsigned int sel)
{
	int ret;

	ret = regmap_write(rdev->regmap, TPS65185_REG_VCOM1, sel);
	if (ret)
		return ret;

	return regmap_update_bits(rdev->regmap, TPS65185_REG_VCOM2,
				  TPS65185_VCOM2_VCOM_MSB, sel >> 8);
}

static int tps65185_vcom_get_voltage_sel(struct regulator_dev *rdev)
{
	int ret, sel, val;

	ret = regmap_read(rdev->regmap, TPS65185_REG_VCOM1, &sel);
	if (ret)
		return ret;

	ret = regmap_read(rdev->regmap, TPS65185_REG_VCOM2, &val);
	if (ret)
		return ret;

	sel |= (val & TPS65185_VCOM2_VCOM_MSB) << 8;

	return sel;
}

static const struct regulator_ops tps65185_vcom_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
	.set_voltage_sel	= tps65185_vcom_set_voltage_sel,
	.map_voltage		= regulator_map_voltage_linear,
	.get_voltage_sel	= tps65185_vcom_get_voltage_sel,
	.list_voltage		= regulator_list_voltage_linear,
};

static const struct regulator_desc tps65185_vcom_desc = {
	.name			= "vcom",
	.supply_name		= "vin",
	.of_match		= "vcom",
	.regulators_node	= "regulators",
	.ops			= &tps65185_vcom_ops,
	.owner			= THIS_MODULE,
	.n_voltages		= 0x200,
	.min_uV			= 0,
	.uV_step		= 10000,
	.enable_reg		= TPS65185_REG_ENABLE,
	.enable_mask		= TPS65185_ENABLE_VCOM_EN,
};

static const struct regulator_ops tps65185_vdrive_ops = {
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.map_voltage		= regulator_map_voltage_iterate,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.list_voltage		= regulator_list_voltage_table,
};

static const struct regulator_desc tps65185_vdrive_desc = {
	.name			= "vdrive",
	.supply_name		= "vin",
	.of_match		= "vdrive",
	.regulators_node	= "regulators",
	.ops			= &tps65185_vdrive_ops,
	.owner			= THIS_MODULE,
	.n_voltages		= 7,
	.linear_min_sel		= 3,
	.volt_table		= (const unsigned int [7]) {
		0, 0, 0,
		15000000, 14750000, 14500000, 14250000
	},
	.vsel_reg		= TPS65185_REG_VADJ,
	.vsel_mask		= TPS65185_VADJ_VSET,
	.enable_reg		= TPS65185_REG_ENABLE,
	.enable_mask		= TPS65185_ENABLE_ACTIVE |
				  TPS65185_ENABLE_STANDBY,
	.enable_val		= TPS65185_ENABLE_ACTIVE,
	.disable_val		= TPS65185_ENABLE_STANDBY,
};

static const struct regmap_config tps65185_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= TPS65185_REG_REVID,
};

static const struct {
	u8		revid;
	const char	*name;
} tps65185_revisions[] = {
	{ 0x45, "TPS65185 r1p0" },
	{ 0x55, "TPS65185 r1p1" },
	{ 0x65, "TPS65185 r1p2" },
	{ 0x66, "TPS651851 r1p0" },
};

static int tps65185_set_sequence(const struct device_node *np,
				 const char *propname, struct regmap *regmap,
				 unsigned int reg)
{
	u32 sequence[4];
	int ret;

	ret = of_property_read_u32_array(np, propname, sequence, 4);
	if (ret)
		return ret == -EINVAL ? 0 : ret;

	return regmap_write(regmap, reg,
			    (sequence[0] & 0x3) << 0 |
			    (sequence[1] & 0x3) << 2 |
			    (sequence[2] & 0x3) << 4 |
			    (sequence[3] & 0x3) << 6);
}

static int tps65185_set_defaults(struct device *dev, struct tps65185 *tps)
{
	const struct device_node *np = dev->of_node;
	int ret;

	ret = tps65185_set_sequence(np, "ti,up-sequence", tps->regmap,
				    TPS65185_REG_UPSEQ0);
	if (ret)
		return ret;

	ret = tps65185_set_sequence(np, "ti,down-sequence", tps->regmap,
				    TPS65185_REG_DWNSEQ0);
	if (ret)
		return ret;

	return 0;
}

static int tps65185_probe(struct i2c_client *client)
{
	struct regulator_config config = {};
	struct device *dev = &client->dev;
	const char *revision = "unknown";
	struct gpio_desc *vcom_ctrl_gpio;
	struct gpio_desc *powerup_gpio;
	struct iio_dev *indio_dev;
	struct tps65185 *tps;
	int i, ret, revid;

	tps = devm_kzalloc(dev, sizeof(*tps), GFP_KERNEL);
	if (!tps)
		return -ENOMEM;

	init_completion(&tps->temp_completion);
	i2c_set_clientdata(client, tps);

	/*
	 * Drive these two GPIOs low before driving WAKEUP high to avoid
	 * powering up the output rails.
	 */
	powerup_gpio = devm_gpiod_get_optional(dev, "powerup", GPIOD_OUT_LOW);
	if (IS_ERR(powerup_gpio))
		return dev_err_probe(dev, PTR_ERR(powerup_gpio),
				     "Failed to get powerup GPIO\n");

	vcom_ctrl_gpio = devm_gpiod_get_optional(dev, "vcom_ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(vcom_ctrl_gpio))
		return dev_err_probe(dev, PTR_ERR(vcom_ctrl_gpio),
				     "Failed to get vcom_ctrl GPIO\n");

	tps->wakeup_gpio = devm_gpiod_get(dev, "wakeup", GPIOD_OUT_HIGH);
	if (IS_ERR(tps->wakeup_gpio))
		return dev_err_probe(dev, PTR_ERR(tps->wakeup_gpio),
				     "Failed to get wakeup GPIO\n");

	msleep(TPS65185_WAKEUP_DELAY_MS);

	tps->regmap = devm_regmap_init_i2c(client, &tps65185_regmap_config);
	if (IS_ERR(tps->regmap))
		return PTR_ERR(tps->regmap);

	ret = regmap_read(tps->regmap, TPS65185_REG_REVID, &revid);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(tps65185_revisions); ++i) {
		if (revid == tps65185_revisions[i].revid) {
			revision = tps65185_revisions[i].name;
			break;
		}
	}
	dev_info(dev, "Chip revision: %s\n", revision);

	ret = tps65185_set_defaults(dev, tps);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to set defaults\n");

	indio_dev = devm_iio_device_alloc(dev, 0);
	if (!indio_dev)
		return -ENOMEM;

	iio_device_set_drvdata(indio_dev, tps);

	indio_dev->modes	= INDIO_DIRECT_MODE;
	indio_dev->channels	= tps65185_iio_channels;
	indio_dev->num_channels	= ARRAY_SIZE(tps65185_iio_channels);
	indio_dev->name		= dev_name(dev);
	indio_dev->info		= &tps65185_iio_info;

	ret = iio_triggered_event_setup(indio_dev, NULL,
					tps65185_iio_trigger_handler);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add IIO trigger\n");

	ret = devm_add_action_or_reset(dev, tps65185_iio_triggered_event_cleanup,
				       indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register IIO device\n");

	ret = regmap_write(tps->regmap, TPS65185_REG_ENABLE, 0);
	if (ret)
		return ret;

	tps->int_en1 = 0;
	ret = regmap_write(tps->regmap, TPS65185_REG_INT_EN1, tps->int_en1);
	if (ret)
		return ret;

	tps->int_en2 = TPS65185_INT2_EOC;
	ret = regmap_write(tps->regmap, TPS65185_REG_INT_EN2, tps->int_en2);
	if (ret)
		return ret;

	ret = devm_request_threaded_irq(dev, client->irq, NULL,
					tps65185_irq_handler, IRQF_ONESHOT,
					dev_name(dev), indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request IRQ\n");

	config.dev = dev;
	config.driver_data = tps;
	config.regmap = tps->regmap;

	for (i = TPS65185_REGULATOR_V3P3; i < TPS65185_NUM_REGULATORS; ++i) {
		const struct regulator_desc *rdesc;
		struct regulator_dev *rdev;

		switch (i) {
		case TPS65185_REGULATOR_V3P3:
			rdesc = &tps65185_v3p3_desc;
			config.ena_gpiod = NULL;
			break;
		case TPS65185_REGULATOR_VCOM:
			rdesc = &tps65185_vcom_desc;
			config.ena_gpiod = vcom_ctrl_gpio;
			break;
		case TPS65185_REGULATOR_VDRIVE:
			rdesc = &tps65185_vdrive_desc;
			config.ena_gpiod = powerup_gpio;
			break;
		}

		rdev = devm_regulator_register(dev, rdesc, &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "Failed to register %s\n",
					     rdesc->name);

		devm_gpiod_unhinge(dev, config.ena_gpiod);
	}

	return 0;
}

static const struct of_device_id tps65185_of_match[] = {
	{ .compatible = "ti,tps65185" },
	{ }
};

static struct i2c_driver tps65185_driver = {
	.probe_new	= tps65185_probe,
	.driver		= {
		.name		= "tps65185",
		.of_match_table	= tps65185_of_match,
	},
};
module_i2c_driver(tps65185_driver);

MODULE_AUTHOR("Samuel Holland <samuel@sholland.org>");
MODULE_DESCRIPTION("TPS65185 PMIC driver");
MODULE_LICENSE("GPL v2");
