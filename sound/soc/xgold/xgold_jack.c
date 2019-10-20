/*
 * Component: XGOLD audio jack driver
 *
 * Copyright (C) 2014, Intel Mobile Communications GmbH.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * You should have received a copy of the GNU General Public License Version 2
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Contributor(s):
 */
#define DEBUG 1
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/driver.h>
#include <sound/soc.h>
#include <linux/of_gpio.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#endif

#include <sofia/vmm_pmic.h>

#include "xgold_jack.h"

/* FIXME */
#include "../codecs/afe_acc_det.h"

#define	xgold_err(fmt, arg...) \
		pr_err("snd: jack: "fmt, ##arg)

#define	xgold_debug(fmt, arg...) \
		pr_debug("snd: jack: "fmt, ##arg)

#define PROP_HP_DET_NAME	"intel,hp-gpio-sd"
#define PROP_HP_DET_PIN 72

#define AHJ_TYPE_MIN_MV 475
#define AHJ_TYPE_MAX_MV 1950

#define HEADPHONE_MIN_MV 0
#define HEADPHONE_MAX_MV 100

/* this variable to overcome the mute after slow removal of jack */
#define JACK_CHECK_PROSS_START 1
#define JACK_CHECK_PROSS_END   0
/**
 * Different VBIAS settings
**/
enum xgold_vbias {
	XGOLD_VBIAS_ENABLE,
	XGOLD_VBIAS_ULP_ON,
	XGOLD_VBIAS_DISABLE,
};

enum xgold_headset_type {
	XGOLD_HEADSET_REMOVED,
	XGOLD_HEADSET,
	XGOLD_HEADPHONE,
	XGOLD_INVALID,
	XGOLD_ERROR
};

/*struct hs_cfg {
	int min_mv;
	int max_mv;
	enum xgold_headset_type type;
};*/

struct hs_key_cfg {
	int min_mv;
	int max_mv;
	enum snd_jack_types type;
	int key_code;
	int pressed;
};

/* AFE register values */
#define XGOLD_DETECT_INSERTION \
	0x800B /* ACD1: insertion; ACD2: disabled; DEBT: 1msc */
#define XGOLD_DETECT_REMOVAL_HEADSET \
	0xC003 /* ACD1: removal; ACD2: headset insertion; DEBT: 1msc  */
#define XGOLD_DETECT_REMOVAL_HOOK \
	0xCA03 /* ACD1: headset removal; ACD2: hook key press; DEBT: 1msc  */
#define XGOLD_DETECT_HOOK_RELEASE \
	0xC203 /* ACD1: removal; ACD2: hook key release; DEBT: 1msc  */

/* PMIC register offset */
/* IRQ registers offsets */
#define IRQMULT_REG			0x1e
#define MIRQMULT_REG			0x1f

/* Masks and bits */
#define IRQMULT_ACCDET1_M		0x01
#define IRQMULT_ACCDET2_M		0x02
#define IRQMULT_ACCDETAUX_M		0x04
#define IRQMULT_ACCDETALL_M \
	(IRQMULT_ACCDET1_M | IRQMULT_ACCDET2_M | IRQMULT_ACCDETAUX_M)

/* PMIC registers offsets */
#define ACC_DET_LOW_REG			0x21
#define ACC_DET_HIGH_REG		0x20
#define ACC_DET_AUX_REG			0x23

#define VBIAS_SETTLING_TIME_MS		20
#define JACK_DET_RETRY_TIMES		4

/* Headset keymap */
struct hs_key_cfg xgold_hs_keymap[] = {
	{0, 200, SND_JACK_BTN_0 , KEY_MEDIA, 0},
	//{100, 150, SND_JACK_BTN_1, KEY_VOLUMEUP, 0},
	//{275, 325, SND_JACK_BTN_2, KEY_VOLUMEDOWN, 0},
};

static int jack_write(struct xgold_jack *jack, unsigned val)
{
#ifdef CONFIG_X86_INTEL_SOFIA
	return mv_svc_reg_write(jack->base_phys, val, -1);
#else
	iowrite(val, jack->mmio_base);
	return 0;
#endif
}

/* PMIC reg accesses */
static int xgold_jack_pmic_reg_read(u32 dev_addr, u32 reg_addr,
		u8 *p_reg_val)
{
	u32 vmm_addr, reg_val = 0;
	int ret;

	vmm_addr = ((dev_addr & 0xFF) << 24) | (reg_addr & 0xFF);
	ret = vmm_pmic_reg_read(vmm_addr, &reg_val);
	*p_reg_val = (u8)(reg_val & 0xFF);
	xgold_debug("%s: read @%X return %X\n", __func__, reg_addr, reg_val);

	return ret;
}

static int xgold_jack_pmic_reg_write(u32 dev_addr, u32 reg_addr, u8 reg_val)
{
	u32 vmm_addr, val = reg_val;

	vmm_addr = ((dev_addr & 0xFF) << 24) | (reg_addr & 0xFF);
	xgold_debug("%s: write @%X value %X\n", __func__, reg_addr, val);
	return vmm_pmic_reg_write(vmm_addr, val);
}

/* Call to AFE to change the VBIAS settings */
static void configure_vbias(struct xgold_jack *jack, enum xgold_vbias state)
{
	struct afe_acc_det acc_det_par;
	int ret;

	//xgold_debug("--> %s: %s\n", __func__, (state == XGOLD_VBIAS_ENABLE) ?
			//"XGOLD_VBIAS_ENABLE" : "XGOLD_VBIAS_ULP_ON");
	xgold_debug("--> %s: (0:ENABLE 1:ULP_ON 2:DISABLE) %d\n", __func__, state);

	//acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_ULP;
	acc_det_par.vumic_conf.hzmic = AFE_HZVUMIC_NORMAL_POWER_DOWN;

	switch (state) {
	case XGOLD_VBIAS_ENABLE:
		acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_ULP;
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_2_1_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_NORMAL;
		acc_det_par.xb_mode = AFE_XB_ON;
		break;
	case XGOLD_VBIAS_ULP_ON:
		acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_ULP;
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_LOW_POWER;
		acc_det_par.xb_mode = AFE_XB_OFF;
		break;
	case XGOLD_VBIAS_DISABLE:
		acc_det_par.vumic_conf.vmode = AFE_VUMIC_MODE_POWER_DOWN;
		acc_det_par.vumic_conf.vmicsel = AFE_VMICSEL_1_9_V;
		acc_det_par.micldo_mode = AFE_MICLDO_MODE_OFF;
		acc_det_par.xb_mode = AFE_XB_OFF;
		break;
	default:
		return;
	}

	if (jack->flags & XGOLD_JACK_PMIC)
		ret = pmic_afe_set_acc_det_with_lock(acc_det_par);
	else
		ret = agold_afe_set_acc_det_with_lock(acc_det_par);

	if (ret)
		xgold_err("Error when setting VBIAS!\n");

	xgold_debug("<-- %s\n", __func__);
}

static u32 read_state(struct xgold_jack *jack)
{
	int volt, ret, det;

	ret = iio_read_channel_processed(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		return XGOLD_ERROR;
	}

	xgold_debug("%s: measured voltage %d\n", __func__, volt);

  // plug in det = 1, gpio input = 0
  // no plug det = 0, gpio input = 1
  det = !gpio_get_value(jack->hp_det_pin);
  xgold_debug("%s: det = %d\n", __func__, det);

	if (det == 1 && (volt >= AHJ_TYPE_MIN_MV && volt <= AHJ_TYPE_MAX_MV))
		return XGOLD_HEADSET;
	else if (det == 1 && (volt >= HEADPHONE_MIN_MV && volt <= HEADPHONE_MAX_MV))
		return XGOLD_HEADPHONE;
	else if (det == 1 && volt <= 2100) // for iphone headset acd 207x, set it as headphone
		return XGOLD_HEADPHONE;
	else if (det == 0 || volt > 2100)
		return XGOLD_HEADSET_REMOVED;
	else
		return XGOLD_INVALID;
}

static void xgold_jack_acc_det_write(struct xgold_jack *jack,
		unsigned val)
{
	int ret;

	xgold_debug("%s: write val 0x%X, mode %s\n", __func__, val,
			(jack->flags & XGOLD_JACK_PMIC) ? "PMIC" : "IO");

	if (jack->flags & XGOLD_JACK_PMIC) {
		ret = xgold_jack_pmic_reg_write(jack->pmic_addr,
				ACC_DET_HIGH_REG, (val >> 8) & 0xFF);
		if (ret) {
			xgold_err("%s: cannot write ACC_DET_HIGH\n",
					__func__);
			return;
		}

		ret = xgold_jack_pmic_reg_write(jack->pmic_addr,
				ACC_DET_LOW_REG, val & 0xFF);
		if (ret) {
			xgold_err("%s: cannot write ACC_DET_LOW\n",
					__func__);
			return;
		}
	} else
		jack_write(jack, val);
}

extern int ret_headset_status;

static void xgold_jack_check(struct xgold_jack *jack)
{
	u32 state, old_state;
	u32 detect;
	int status = 0;
	enum xgold_vbias vbias;
	int i, det;
	unsigned int mask = SND_JACK_HEADSET;

  for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
    mask |= xgold_hs_keymap[i].type;
  }

	wake_lock_timeout(&jack->suspend_lock,
			(JACK_DET_RETRY_TIMES/2 + 2) * HZ);

	/*  set the flag for button thread to wait until release it.*/
	configure_vbias(jack, XGOLD_VBIAS_ENABLE);

	/* First, make sure we have a stable state.
	   Headset insertion takes a bit of time(~> 500ms),
	   so make sure that two consecutive reads agree.
	*/
	state = XGOLD_ERROR;
	for (i = 0; (i < JACK_DET_RETRY_TIMES) && (XGOLD_ERROR == state); i++) {
		xgold_debug("Jack detect - try %d times\n", i);
		msleep(600);
		state = read_state(jack);
		do {
			old_state = state;
			msleep(100);
			state = read_state(jack);
		} while (state != old_state);
	}


	if (XGOLD_ERROR == state) {
		xgold_err("Unable to determine state.\n");
		// ASUS BSP workaround if gpio det = 1 set as headphone
    // plug in det = 1, gpio input = 0
    // no plug det = 0, gpio input = 1
    det = !gpio_get_value(jack->hp_det_pin);
    if (det) state = XGOLD_HEADPHONE;
    else state = XGOLD_HEADSET_REMOVED;
		//return;
	}

	switch (state) {
	case XGOLD_HEADPHONE:
		xgold_debug("Headphone inserted\n");
		vbias = XGOLD_VBIAS_DISABLE;
		detect = XGOLD_DETECT_REMOVAL_HEADSET;
		status = SND_JACK_HEADPHONE;
		jack->buttons_enabled = false;
		break;
	case XGOLD_HEADSET:
		xgold_debug("Headset inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		detect = XGOLD_DETECT_REMOVAL_HOOK;
		status = SND_JACK_HEADSET;
		jack->buttons_enabled = true;
		break;
	case XGOLD_HEADSET_REMOVED:
		xgold_debug("Headphone/headset removed\n");
		vbias = XGOLD_VBIAS_DISABLE;
		detect = XGOLD_DETECT_INSERTION;
		jack->buttons_enabled = false;
		break;
	default:
		xgold_debug("Invalid headset state!\n");
		return;
	}
        
  ret_headset_status = state;

	configure_vbias(jack, vbias);
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Check if there really is a state change */
	if (status != (jack->hs_jack->status & mask)) {
		if ((status != XGOLD_HEADSET_REMOVED) && ((jack->hs_jack->status & mask) != XGOLD_HEADSET_REMOVED)) {
			//when android wired accessory status from headphone to headset (cause by slow plugin)
			//asus status bar headphone icon will disappear
			//send remove status before another plugin status to avoid icon disappear
			xgold_debug("send remove status before another plugin status\n");
			snd_soc_jack_report(jack->hs_jack, XGOLD_HEADSET_REMOVED, mask);
		}
		xgold_jack_acc_det_write(jack, detect);
		snd_soc_jack_report(jack->hs_jack, status, mask);
		/* clear all key status */
    for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++)
      xgold_hs_keymap[i].pressed = 0;
	}
}

struct xgold_jack *xgold_jack_priv;
void xgold_jack_headset_set(int state)
{
	u32 detect;
	int status = 0;
	enum xgold_vbias vbias;
	int i;
	unsigned int mask = SND_JACK_HEADSET;

  for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
    mask |= xgold_hs_keymap[i].type;
  }

	xgold_debug("xgold_jack_set headset_set state = %d\n", state);
	// should map state to snd status. 0->0, 1->SND_JACK_HEADSET, 2->SND_JACK_HEADPHONE

	switch (state) {
	case XGOLD_HEADPHONE:
		xgold_debug("Headphone inserted\n");
		vbias = XGOLD_VBIAS_DISABLE;
		detect = XGOLD_DETECT_REMOVAL_HEADSET;
		status = SND_JACK_HEADPHONE;
		xgold_jack_priv->buttons_enabled = false;
		break;
	case XGOLD_HEADSET:
		xgold_debug("Headset inserted\n");
		vbias = XGOLD_VBIAS_ENABLE;
		detect = XGOLD_DETECT_REMOVAL_HOOK;
		status = SND_JACK_HEADSET;
		xgold_jack_priv->buttons_enabled = true;
		break;
	case XGOLD_HEADSET_REMOVED:
		xgold_debug("Headphone/headset removed\n");
		vbias = XGOLD_VBIAS_DISABLE;
		detect = XGOLD_DETECT_INSERTION;
		xgold_jack_priv->buttons_enabled = false;
		break;
	default:
		xgold_debug("Invalid headset state!\n");
		return;
	}

	ret_headset_status = state;

	configure_vbias(xgold_jack_priv, vbias);
	msleep(VBIAS_SETTLING_TIME_MS);

	/* Check if there really is a state change */
	if (status != (xgold_jack_priv->hs_jack->status & mask)) {
		xgold_jack_acc_det_write(xgold_jack_priv, detect);
		snd_soc_jack_report(xgold_jack_priv->hs_jack, status, mask);
		/* clear all key status */
    for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++)
      xgold_hs_keymap[i].pressed = 0;
	}
	return;
}
EXPORT_SYMBOL(xgold_jack_headset_set);

static irqreturn_t xgold_jack_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;

	xgold_debug("%s\n", __func__);

	if (jack->flags & XGOLD_JACK_PMIC)
		xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
				IRQMULT_REG, IRQMULT_ACCDET1_M);

	/* set the flag for button thread to wait until release it.*/
	if (jack->jack_check_in_progress)
		return IRQ_HANDLED;

	jack->jack_check_in_progress = JACK_CHECK_PROSS_START;

	xgold_jack_check((struct xgold_jack *)data);

	/* release the flag for button thread to continue.*/
	jack->jack_check_in_progress = JACK_CHECK_PROSS_END;

	return IRQ_HANDLED;
}

static void xgold_button_check(struct xgold_jack *jack)
{
	int ret, volt, i;
	int key_index = -1;
	u32 detect;
	int status;
	enum snd_jack_types type;
	int det;

	wake_lock_timeout(&jack->suspend_lock,
			(JACK_DET_RETRY_TIMES/2 + 2) * HZ);

	/* wait to check the interrupt due to slow removal of jack */
	if (0 == xgold_hs_keymap[0].pressed) 
	    msleep_interruptible(80);
	/* If the interrupt due to slow removal of jack,*/
	/*return without action */
	if (jack->jack_check_in_progress) {
		xgold_debug("%s: jack check in progress, return\n", __func__);
		return;
	}

  // plug in det = 1, gpio input = 0
  // no plug det = 0, gpio input = 1
  det = !gpio_get_value(jack->hp_det_pin);
  if (!det) {
    xgold_debug("%s: det = %d return\n", __func__, det);
    return;
  }

	ret = iio_read_channel_processed(jack->iio_client, &volt);
	if (ret < 0) {
		xgold_err("Unable to read channel volt\n");
		//return;
		//ASUS BSP workaround
		if (xgold_hs_keymap[0].pressed == 0) {
			xgold_hs_keymap[0].pressed = 1;
			key_index = 0;
		}
	} else {
		xgold_debug("%s: measured voltage %d\n", __func__, volt);

		/* check if a key has been pressed and remember this*/
		for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
			if ((volt >= xgold_hs_keymap[i].min_mv) &&
				(volt <= xgold_hs_keymap[i].max_mv)) {
				xgold_hs_keymap[i].pressed = 1;
				key_index = i;
				break;
			}
		}
	}

	if (key_index > -1) {
		xgold_debug("button press index %d\n", key_index);
		type = xgold_hs_keymap[key_index].type;
		status = type;
		detect = XGOLD_DETECT_HOOK_RELEASE;
	} else {
		/* key released, figure out which */
		for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
			if (1 == xgold_hs_keymap[i].pressed) {
				xgold_debug("button release, index %d\n", i);
				xgold_hs_keymap[i].pressed = 0;
				key_index = i;
				type = xgold_hs_keymap[key_index].type;
				status = 0;
			}
		}
		detect = XGOLD_DETECT_REMOVAL_HOOK;
	}
	if (key_index > -1) {
		snd_soc_jack_report(jack->hs_jack, status, type);
		xgold_jack_acc_det_write(jack, detect);
	}
}

static irqreturn_t xgold_button_detection(int irq, void *data)
{
	struct xgold_jack *jack = data;

	xgold_debug("%s\n", __func__);

	if (jack->flags & XGOLD_JACK_PMIC)
		xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
				IRQMULT_REG, IRQMULT_ACCDET2_M);

	if ((jack->hs_jack->status & SND_JACK_HEADSET) != SND_JACK_HEADSET) {
		/* this interrupt may occurs in case of slow jack insertion */
		xgold_debug("button detection while no headset\n");
		/* this may cause multi jack detect race condition */
		//return xgold_jack_detection(irq, data);
		return IRQ_HANDLED;
	}

	if (jack->buttons_enabled)
		xgold_button_check(jack);
	else
		xgold_debug("button detection while buttons_enabled=false\n");
	return IRQ_HANDLED;
}

int xgold_jack_setup(struct snd_soc_codec *codec, struct snd_soc_jack *hs_jack)
{
	int i, type, ret;

	xgold_debug("%s\n", __func__);

	type = SND_JACK_HEADSET;
	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++)
		type |= xgold_hs_keymap[i].type;

	ret = snd_soc_jack_new(codec, "Headset", type, hs_jack);

	if (ret) {
		xgold_err("Jack creation failed\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(xgold_hs_keymap); i++) {
		ret = snd_jack_set_key(hs_jack->jack,
			xgold_hs_keymap[i].type,
			xgold_hs_keymap[i].key_code);
		if (ret)
			xgold_err("Failed to set headset key\n");
	}

	return ret;
}

struct xgold_jack *of_xgold_jack_probe(struct platform_device *pdev,
		struct device_node *np, struct snd_soc_jack *hs_jack)
{
	struct xgold_jack *jack;
	struct resource regs;
	struct resource *res;
	int num_irq, i, ret;
	unsigned value;

	jack = devm_kzalloc(&pdev->dev, sizeof(*jack), GFP_ATOMIC);
	if (!jack) {
		xgold_err("Allocation failed!\n");
		ret = -ENOMEM;
		goto out;
	}

	jack->buttons_enabled = false;
	jack->jack_irq = jack->button_irq = -1;
	jack->hs_jack = hs_jack;
	jack->jack_check_in_progress = JACK_CHECK_PROSS_END;
	wake_lock_init(&jack->suspend_lock, WAKE_LOCK_SUSPEND, "jack_wakelock");

	if (of_device_is_compatible(np, "intel,headset,pmic"))
		jack->flags |= XGOLD_JACK_PMIC;

	if (jack->flags & XGOLD_JACK_PMIC) {
		/* PMIC device address */
		ret = of_property_read_u32_index(np, "intel,reg", 0, &value);
		if (ret)
			goto out;

		if (value > 0xFF) {
			ret = -ERANGE;
			goto out;
		}

		jack->pmic_addr = (unsigned char)value;

		/* FIXME: should be handled by VMM, not linux driver */
		/* PMIC device address for IRQ handling */
		ret = of_property_read_u32_index(np, "intel,irq-reg", 0,
				&value);
		if (ret)
			goto out;

		if (value > 0xFF) {
			ret = -ERANGE;
			goto out;
		}

		jack->pmic_irq_addr = (unsigned char)value;
	} else {
		if (of_address_to_resource(np, 0, &regs)) {
			ret = -ENOENT;
			goto out;
		}

		jack->mmio_base = devm_ioremap(
				&pdev->dev, regs.start, resource_size(&regs));
		if (jack->mmio_base == NULL) {
			xgold_err("failed to remap I/O memory\n");
			ret = -ENXIO;
			goto out;
		}
		jack->base_phys = regs.start;
		xgold_debug("ioremap %p\n", jack->mmio_base);
	}

	/* hp_det gpio */
	//jack->hp_det_pin = of_get_named_gpio_flags(np,
			//PROP_HP_DET_NAME, 0, NULL);
	jack->hp_det_pin = PROP_HP_DET_PIN;

	if (jack->hp_det_pin <= 0) {
		xgold_debug("%s: unable to get hp detection node %s\n",
			  __func__, PROP_HP_DET_NAME);
	} else {
		xgold_debug("%s: Get hp detection node %s value: %d !!!\n",
			  __func__, PROP_HP_DET_NAME, jack->hp_det_pin);
		ret = gpio_request(jack->hp_det_pin, PROP_HP_DET_NAME);
		if (!ret) {
			xgold_debug("req gpio_request success!:%d\n", ret);
			gpiod_direction_input(
				gpio_to_desc(jack->hp_det_pin));
		} else {
			xgold_err("req gpio_request failed:%d\n", ret);
		}
	}

	num_irq = of_irq_count(np);
	if (!num_irq) {
		xgold_err("no headset plug irq defined\n");
		ret = -EINVAL;
		goto out;
	}

	res = devm_kzalloc(&pdev->dev, sizeof(*res) * num_irq, GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}

	of_irq_to_resource_table(np, res, num_irq);
	for (i = 0; i < num_irq; i++)
		//if (strcmp(res[i].name, "acd1") == 0)
		if (strcmp(res[i].name, "eint9") == 0)
			jack->jack_irq = res[i].start;

	jack->iio_client = iio_channel_get(NULL, "ACCID_SENSOR");
	if (IS_ERR(jack->iio_client)) {
		xgold_err("iio channel error\n");
		ret = -EINVAL;
		goto out;
	}

	/* Configure the Accessory settings to detect Insertion */
	xgold_jack_acc_det_write(jack, XGOLD_DETECT_INSERTION);

	ret = devm_request_threaded_irq(&(pdev->dev), jack->jack_irq,
			NULL,
			xgold_jack_detection,
			//IRQF_SHARED | IRQF_ONESHOT, "jack_irq", jack);
			IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND, "jack_irq", jack);
	if (ret) {
		xgold_err("setup of jack irq failed!\n");
		ret = -EINVAL;
		goto out;
	}
 
	for (i = 0; i < num_irq; i++)
		if (strcmp(res[i].name, "acd2") == 0)
			jack->button_irq = res[i].start;

	ret = devm_request_threaded_irq(&(pdev->dev), jack->button_irq,
			NULL,
			xgold_button_detection,
			//IRQF_SHARED | IRQF_ONESHOT, "button_irq", jack);
			IRQF_SHARED | IRQF_ONESHOT | IRQF_NO_SUSPEND, "button_irq", jack);
	if (ret < 0) {
		xgold_err("setup of button irq failed!\n");
		ret = -EINVAL;
		goto out;
	}

	/* FIXME: below code should be handled by irqchip level/vmm, when
	 * requesting for the PMIC ACD interrupt, and not in this driver */
	if (jack->flags & XGOLD_JACK_PMIC) {
		int tries;
		char val;

		/* Unmask IRQMULT interrupt */
		xgold_err("%s: Warning! may apply changes to MIRQMULT register\n",
				__func__);

		xgold_jack_pmic_reg_read(jack->pmic_irq_addr,
				MIRQMULT_REG, &val);
		tries = 0;
		while ((val & IRQMULT_ACCDETALL_M) && tries++ < 20) {
			xgold_jack_pmic_reg_write(jack->pmic_irq_addr,
					MIRQMULT_REG,
					val & ~IRQMULT_ACCDETALL_M);

			/* read again to ensure Mask is correctly configured */
			xgold_jack_pmic_reg_read(jack->pmic_irq_addr,
					MIRQMULT_REG, &val);
			xgold_debug("%s: MIRQMULT is 0x%02X\n", __func__, val);
		}

		if (tries >= 20) {
			ret = -EIO;
			goto out;
		}

		xgold_err("%s MIRQLVL1 is 0x%02X\n", __func__, val);
	}
	/* end of FIXME */

	xgold_jack_priv = jack;

	/* check jack status after boot */
	msleep(1000);
	xgold_debug("check jack status after boot\n");
	xgold_jack_check(jack);

	return jack;

out:
	return ERR_PTR(ret);
}

void xgold_jack_remove(struct xgold_jack *jack)
{
	if (jack && !IS_ERR(jack->iio_client))
		iio_channel_release(jack->iio_client);
	if (jack)
		wake_lock_destroy(&jack->suspend_lock);
}
