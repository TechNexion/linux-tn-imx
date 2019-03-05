/*
 * Copyright 2011 bct electronic GmbH
 * Copyright 2013 Qtechnology/AS
 *
 * Author: Peter Meerwald <p.meerwald@bct-electronic.com>
 * Author: Ricardo Ribalda <ricardo.ribalda@gmail.com>
 *
 * Based on leds-pca963x.c
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 * LED driver for the PCA995x
 * 
 *
 * Note that hardware blinking violates the leds infrastructure driver
 * interface since the hardware only supports blinking all LEDs with the
 * same delay_on/delay_off rates.  That is, only the LEDs that are set to
 * blink will actually blink but all LEDs that are set to blink will blink
 * in identical fashion.  The delay_on/delay_off values of the last LED
 * that is set to blink will be used for all of the blinking LEDs.
 * Hardware blinking is disabled by default but can be enabled by setting
 * the 'blink_type' member in the platform_data struct to 'PCA963X_HW_BLINK'
 * or by adding the 'nxp,hw-blink' property to the DTS.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/leds.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/platform_data/leds-pca995x.h>

/* LED select registers determine the source that drives LED outputs */
#define PCA963X_LED_OFF		0x0	/* LED driver off */
#define PCA963X_LED_ON		0x1	/* LED driver on */
#define PCA963X_LED_PWM		0x2	/* Controlled through PWM */
#define PCA963X_LED_GRP_PWM	0x3	/* Controlled through PWM/GRPPWM */

#define PCA963X_MODE2_DMBLNK	0x20	/* Enable blinking */

#define PCA963X_MODE1		0x00
#define PCA963X_MODE2		0x01
#define PCA963X_PWM_BASE	0x0A
#define PCA963X_IREFALL		0x43

enum pca995x_type {
	pca9955,
};

struct pca995x_chipdef {
	u8			grppwm;
	u8			grpfreq;
	u8			ledout_base;
	int			n_leds;
};

static struct pca995x_chipdef pca995x_chipdefs[] = {

	[pca9955] = {
		.grppwm		= 0x08,
		.grpfreq	= 0x09,
		.ledout_base	= 0x02,
		.n_leds  	= 16,
		},
	};

/* Total blink period in milliseconds */
#define PCA963X_BLINK_PERIOD_MIN	42
#define PCA963X_BLINK_PERIOD_MAX	10667

static const struct i2c_device_id pca995x_id[] = {
	{ "pca9955", pca9955 },
	{ }
};


MODULE_DEVICE_TABLE(i2c, pca995x_id);

enum pca995x_cmd {
	BRIGHTNESS_SET,
	BLINK_SET,
};

struct pca995x_led;

struct pca995x {
	struct pca995x_chipdef *chipdef;
	struct mutex mutex;
	struct i2c_client *client;
	struct pca995x_led *leds;
};

struct pca995x_led {
	struct pca995x *chip;
	struct work_struct work;
	enum led_brightness brightness;
	struct led_classdev led_cdev;
	int led_num; /* 0 .. 15 potentially */
	enum pca995x_cmd cmd;
	char name[32];
	u8 gdc;
	u8 gfrq;
};

static void pca995x_brightness_work(struct pca995x_led *pca995x)
{
	u8 ledout_addr = pca995x->chip->chipdef->ledout_base
		+ (pca995x->led_num / 4);
	u8 ledout;
	int shift = 2 * (pca995x->led_num % 4);
	u8 mask = 0x3 << shift;

	mutex_lock(&pca995x->chip->mutex);
	ledout = i2c_smbus_read_byte_data(pca995x->chip->client, ledout_addr);
	switch (pca995x->brightness) {
	case LED_FULL:
		i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_ON << shift));
		break;
	case LED_OFF:
		i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			ledout & ~mask);
		break;
	default:
		i2c_smbus_write_byte_data(pca995x->chip->client,
			PCA963X_PWM_BASE + pca995x->led_num,
			pca995x->brightness);
		i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_PWM << shift));
		break;
	}
	mutex_unlock(&pca995x->chip->mutex);
}

static void pca995x_blink_work(struct pca995x_led *pca995x)
{
	u8 ledout_addr = pca995x->chip->chipdef->ledout_base +
		(pca995x->led_num / 4);
	u8 ledout;
	u8 mode2 = i2c_smbus_read_byte_data(pca995x->chip->client,
							PCA963X_MODE2);
	int shift = 2 * (pca995x->led_num % 4);
	u8 mask = 0x3 << shift;

	i2c_smbus_write_byte_data(pca995x->chip->client,
			pca995x->chip->chipdef->grppwm,	pca995x->gdc);

	i2c_smbus_write_byte_data(pca995x->chip->client,
			pca995x->chip->chipdef->grpfreq, pca995x->gfrq);

	if (!(mode2 & PCA963X_MODE2_DMBLNK))
		i2c_smbus_write_byte_data(pca995x->chip->client, PCA963X_MODE2,
			mode2 | PCA963X_MODE2_DMBLNK);

	mutex_lock(&pca995x->chip->mutex);
	ledout = i2c_smbus_read_byte_data(pca995x->chip->client, ledout_addr);
	if ((ledout & mask) != (PCA963X_LED_GRP_PWM << shift))
		i2c_smbus_write_byte_data(pca995x->chip->client, ledout_addr,
			(ledout & ~mask) | (PCA963X_LED_GRP_PWM << shift));
	mutex_unlock(&pca995x->chip->mutex);
}

static void pca995x_work(struct work_struct *work)
{
	struct pca995x_led *pca995x = container_of(work,
		struct pca995x_led, work);

	switch (pca995x->cmd) {
	case BRIGHTNESS_SET:
		pca995x_brightness_work(pca995x);
		break;
	case BLINK_SET:
		pca995x_blink_work(pca995x);
		break;
	}
}

static void pca995x_led_set(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct pca995x_led *pca995x;

	pca995x = container_of(led_cdev, struct pca995x_led, led_cdev);

	pca995x->cmd = BRIGHTNESS_SET;
	pca995x->brightness = value;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&pca995x->work);
}

static int pca995x_blink_set(struct led_classdev *led_cdev,
		unsigned long *delay_on, unsigned long *delay_off)
{
	struct pca995x_led *pca995x;
	unsigned long time_on, time_off, period;
	u8 gdc, gfrq;

	pca995x = container_of(led_cdev, struct pca995x_led, led_cdev);

	time_on = *delay_on;
	time_off = *delay_off;

	/* If both zero, pick reasonable defaults of 500ms each */
	if (!time_on && !time_off) {
		time_on = 500;
		time_off = 500;
	}

	period = time_on + time_off;

	/* If period not supported by hardware, default to someting sane. */
	if ((period < PCA963X_BLINK_PERIOD_MIN) ||
	    (period > PCA963X_BLINK_PERIOD_MAX)) {
		time_on = 500;
		time_off = 500;
		period = time_on + time_off;
	}

	/*
	 * From manual: duty cycle = (GDC / 256) ->
	 *	(time_on / period) = (GDC / 256) ->
	 *		GDC = ((time_on * 256) / period)
	 */
	gdc = (time_on * 256) / period;

	/*
	 * From manual: period = ((GFRQ + 1) / 24) in seconds.
	 * So, period (in ms) = (((GFRQ + 1) / 24) * 1000) ->
	 *		GFRQ = ((period * 24 / 1000) - 1)
	 */
	gfrq = (period * 24 / 1000) - 1;

	pca995x->cmd = BLINK_SET;
	pca995x->gdc = gdc;
	pca995x->gfrq = gfrq;

	/*
	 * Must use workqueue for the actual I/O since I2C operations
	 * can sleep.
	 */
	schedule_work(&pca995x->work);

	*delay_on = time_on;
	*delay_off = time_off;

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static struct pca995x_platform_data *
pca995x_dt_init(struct i2c_client *client, struct pca995x_chipdef *chip)
{
	struct device_node *np = client->dev.of_node, *child;
	struct pca995x_platform_data *pdata;
	struct led_info *pca995x_leds;
	int count;


	pr_info("ALL \n");
	count = of_get_child_count(np);
	if (!count || count > chip->n_leds)
		return ERR_PTR(-ENODEV);
	pr_info("ALL 0\n");
	pca995x_leds = devm_kzalloc(&client->dev,
			sizeof(struct led_info) * chip->n_leds, GFP_KERNEL);
	if (!pca995x_leds)
		return ERR_PTR(-ENOMEM);

	pr_info("ALL 1\n");
	for_each_child_of_node(np, child) {
		struct led_info led = {};
		u32 reg;
		int res;

		res = of_property_read_u32(child, "reg", &reg);
		if ((res != 0) || (reg >= chip->n_leds))
			continue;
		pr_info("ALL 2\n");

		led.name =
			of_get_property(child, "label", NULL) ? : child->name;
		led.default_trigger =
			of_get_property(child, "linux,default-trigger", NULL);
		pca995x_leds[reg] = led;
	}
	pdata = devm_kzalloc(&client->dev,
			     sizeof(struct pca995x_platform_data), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);
	pr_info("ALL 3\n");

	pdata->leds.leds = pca995x_leds;
	pdata->leds.num_leds = chip->n_leds;


	return pdata;
}

static const struct of_device_id of_pca995x_match[] = {
	{ .compatible = "nxp,pca9955", },
	{},
};
#else
static struct pca995x_platform_data *
pca995x_dt_init(struct i2c_client *client, struct pca995x_chipdef *chip)
{
	return ERR_PTR(-ENODEV);
}
#endif

static int pca995x_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	struct pca995x *pca995x_chip;
	struct pca995x_led *pca995x;
	struct pca995x_platform_data *pdata;
	struct pca995x_chipdef *chip;
	int i, err;

	chip = &pca995x_chipdefs[id->driver_data];
	pdata = dev_get_platdata(&client->dev);

	if (!pdata) {
		pdata = pca995x_dt_init(client, chip);
		if (IS_ERR(pdata)) {
			dev_warn(&client->dev, "could not parse configuration\n");
			pdata = NULL;
		}
	}

	if (pdata && (pdata->leds.num_leds < 1 ||
				 pdata->leds.num_leds > chip->n_leds)) {
		dev_err(&client->dev, "board info must claim 1-%d LEDs",
								chip->n_leds);
		return -EINVAL;
	}

	pca995x_chip = devm_kzalloc(&client->dev, sizeof(*pca995x_chip),
								GFP_KERNEL);
	if (!pca995x_chip)
		return -ENOMEM;
	pca995x = devm_kzalloc(&client->dev, chip->n_leds * sizeof(*pca995x),
								GFP_KERNEL);
	if (!pca995x)
		return -ENOMEM;

	i2c_set_clientdata(client, pca995x_chip);

	mutex_init(&pca995x_chip->mutex);
	pca995x_chip->chipdef = chip;
	pca995x_chip->client = client;
	pca995x_chip->leds = pca995x;

	/* Turn off LEDs by default*/
	for (i = 0; i < chip->n_leds / 4; i++)
		i2c_smbus_write_byte_data(client, chip->ledout_base + i, 0x00);

	for (i = 0; i < chip->n_leds; i++) {
		pca995x[i].led_num = i;
		pca995x[i].chip = pca995x_chip;

		/* Platform data can specify LED names and default triggers */
		if (pdata && i < pdata->leds.num_leds) {
			if (pdata->leds.leds[i].name)
				snprintf(pca995x[i].name,
					 sizeof(pca995x[i].name), "pca995x:%s",
					 pdata->leds.leds[i].name);
			if (pdata->leds.leds[i].default_trigger)
				pca995x[i].led_cdev.default_trigger =
					pdata->leds.leds[i].default_trigger;
		}
		if (!pdata || i >= pdata->leds.num_leds ||
						!pdata->leds.leds[i].name)
			snprintf(pca995x[i].name, sizeof(pca995x[i].name),
				 "pca995x:%d:%.2x:%d", client->adapter->nr,
				 client->addr, i);

		pca995x[i].led_cdev.name = pca995x[i].name;
		pca995x[i].led_cdev.brightness_set = pca995x_led_set;

		if (pdata && pdata->blink_type == PCA963X_HW_BLINK)
			pca995x[i].led_cdev.blink_set = pca995x_blink_set;

		INIT_WORK(&pca995x[i].work, pca995x_work);

		err = led_classdev_register(&client->dev, &pca995x[i].led_cdev);
		if (err < 0)
			goto exit;
	}

	/* Disable LED all-call address and set normal mode */
	i2c_smbus_write_byte_data(client, PCA963X_MODE1, 0x00);
	/*IREF Output current value for all LEDn outputs*/
	i2c_smbus_write_byte_data(client, PCA963X_IREFALL, 0x7F);


	return 0;

exit:
	while (i--) {
		led_classdev_unregister(&pca995x[i].led_cdev);
		cancel_work_sync(&pca995x[i].work);
	}

	return err;
}

static int pca995x_remove(struct i2c_client *client)
{
	struct pca995x *pca995x = i2c_get_clientdata(client);
	int i;

	for (i = 0; i < pca995x->chipdef->n_leds; i++) {
		led_classdev_unregister(&pca995x->leds[i].led_cdev);
		cancel_work_sync(&pca995x->leds[i].work);
	}

	return 0;
}

static struct i2c_driver pca995x_driver = {
	.driver = {
		.name	= "leds-pca995x",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(of_pca995x_match),
	},
	.probe	= pca995x_probe,
	.remove	= pca995x_remove,
	.id_table = pca995x_id,
};

module_i2c_driver(pca995x_driver);

MODULE_AUTHOR("Alejandro Lozano <alejandro.lozano@nxp.com>");
MODULE_DESCRIPTION("PCA995x LED driver");
MODULE_LICENSE("GPL v2");
