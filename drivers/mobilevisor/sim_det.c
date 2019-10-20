#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/switch.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <../arch/x86/platform/asustek/include/asustek_boardinfo.h>

#define SIM_DELAY HZ/10
#define NAME_SIM_PLUG "ril_sim_plug"
#define SIM1_PRESENT (1 << 0)
#define SIM2_PRESENT (1 << 1)

#define SIM_IRQ_NOT_READY

static bool is_switch;
static struct workqueue_struct *sim_det_wq;
struct sim_detect_drvdata {
    struct device *dev;
    struct switch_dev sim_switch;
    struct delayed_work sim_det_work;
    int sim_detect_state;
    int sim1_detect_gpio;
    int sim2_detect_gpio;
    int sim1_detect_irq;
    int sim2_detect_irq;
};

#ifdef SIM_IRQ_NOT_READY
static ssize_t print_sim_plug_state(struct switch_dev *sdev, char *buf)
{
    int value = 0;
    int sim_plug_state = 0;
    int proj_id = 0;
    struct sim_detect_drvdata *pdata = container_of(
            sdev, struct sim_detect_drvdata, sim_switch);
    pr_info("SIM_DETECT: sim1_det_gpio=%d, sim2_det_gpio=%d\n",
        pdata->sim1_detect_gpio, pdata->sim2_detect_gpio);

    if (pdata->sim1_detect_gpio > 0) {
        value = gpio_get_value(pdata->sim1_detect_gpio);
        pr_info("SIM_DETECT: %s GPIO_SIM1_DET_VALUE = 0x%x\n", __func__, value);
        if (value) sim_plug_state |= SIM1_PRESENT;
    }
#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
    // WA: do not read sim2 detect pin on non-dual sim sku
    proj_id = asustek_boardinfo_get(FUN_PROJECT_ID);
    if (Z170CG_DSV != proj_id) {
        pr_info("SIM_DETECT: %s proj_id %d not support dual sim\n", __func__, proj_id);
        goto sim2_end;
    }

    if (pdata->sim2_detect_gpio > 0) {
        value = gpio_get_value(pdata->sim2_detect_gpio);
        pr_info("SIM_DETECT: %s GPIO_SIM2_DET_VALUE = 0x%x\n", __func__, value);
        if (value) sim_plug_state |= SIM2_PRESENT;
    }
sim2_end:
#endif
    return sprintf(buf, "%d\n", sim_plug_state);
}
#endif

void sim_detect_work(struct work_struct *work)
{
    // DO NOTHING ABOUT INITIAL DETECTION
}

#ifndef SIM_IRQ_NOT_READY
static irqreturn_t sim1_ap_cd_isr(int irq, void *data)
{
    struct sim_detect_drvdata *pdata = (struct sim_detect_drvdata *)data;
    queue_delayed_work(sim_det_wq, &pdata->sim_det_work, SIM_DELAY);
    return IRQ_HANDLED;
}

static irqreturn_t sim2_ap_cd_isr(int irq, void *data)
{
    struct sim_detect_drvdata *pdata = (struct sim_detect_drvdata *)data;
    queue_delayed_work(sim_det_wq, &pdata->sim_det_work, SIM_DELAY);
    return IRQ_HANDLED;
}
#endif

int sim_detect_probe(struct platform_device *pdev)
{
    int ret = 0;
    struct sim_detect_drvdata *pdata;
    struct device *dev = &pdev->dev;
    struct device_node *np = dev->of_node;
    pr_info("SIM_DETECT: %s\n", __func__);

    /* Allocate driver data record */
    pdata = devm_kzalloc(dev,
            sizeof(struct sim_detect_drvdata), GFP_KERNEL);
    if (!pdata) {
        dev_err(dev, "Couldn't allocate driver data record\n");
        goto fail;
    }
    pdata->dev = &pdev->dev;
    pdata->sim_detect_state = 0;

    // aquire gpio number
    pdata->sim1_detect_gpio = of_get_named_gpio(np, "usim1_ccin_o", 0);
    if (!gpio_is_valid(pdata->sim1_detect_gpio)) {
        pr_err("can't get gpio: usim1_ccin_o\n");
        goto fail;
    }
#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
    pdata->sim2_detect_gpio = of_get_named_gpio(np, "usim2_ccin_o", 0);
    if (!gpio_is_valid(pdata->sim2_detect_gpio)) {
        pr_err("can't get gpio: usim2_ccin_o\n");
        goto fail;
    }
#endif

    // configure sim1_detect_gpio
    ret = gpio_request(pdata->sim1_detect_gpio, "usim1_ccin_o");
    ret += gpio_direction_input(pdata->sim1_detect_gpio);
    if (ret) {
        pr_err("SIM_DETECT: Unable to configure GPIO%d usim1_ccin_o\n", pdata->sim1_detect_gpio);
        gpio_free(pdata->sim1_detect_gpio);
        goto fail;
    }

#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
    // configure sim2_detect_gpio
    ret = gpio_request(pdata->sim2_detect_gpio, "usim2_ccin_o");
    ret += gpio_direction_input(pdata->sim2_detect_gpio);
    if (ret) {
        pr_err("SIM_DETECT: Unable to configure GPIO%d usim2_ccin_o\n", pdata->sim2_detect_gpio);
        gpio_free(pdata->sim2_detect_gpio);
        goto fail;
    }

    pr_info("SIM_DETECT: sim1_detect_gpio=%d, sim2_detect_gpio=%d",
            pdata->sim1_detect_gpio, pdata->sim2_detect_gpio);
#endif

#ifndef SIM_IRQ_NOT_READY
    sim1_det_gpio = pdata->sim1_detect_gpio;
    sim2_det_gpio = pdata->sim2_detect_gpio;

    // configure sim1_detect_irq
    pdata->sim1_detect_irq = gpio_to_irq(pdata->sim1_detect_gpio);
    ret = request_irq(pdata->sim1_detect_irq, sim1_ap_cd_isr,
                  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
                  "usim1_ccin_o", pdata);
    if (ret) {
        pr_err("SIM_DETECT: IRQ request failed for GPIO%d usim1_ccin_o\n", pdata->sim1_detect_gpio);
        pdata->sim1_detect_irq = -1;
        goto fail;
    }
#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
    // configure sim2_detect_irq
    pdata->sim2_detect_irq = gpio_to_irq(pdata->sim2_detect_gpio);
    ret = request_irq(pdata->sim2_detect_irq, sim2_ap_cd_isr,
                  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_SUSPEND,
                  "usim2_ccin_o", pdata);
    if (ret) {
        pr_err("SIM_DETECT: IRQ request failed for GPIO%d usim2_ccin_o\n", pdata->sim2_detect_gpio);
        pdata->sim2_detect_irq = -1;
        goto fail;
    }
#endif
#endif

    /* Register switch device in "/sys/class/switch" */
    pdata->sim_switch.name = NAME_SIM_PLUG;
    pdata->sim_switch.state = 0;
    pdata->sim_switch.print_state = print_sim_plug_state;
    ret = switch_dev_register(&pdata->sim_switch);
    if (ret) {
        dev_err(dev, "failed to register switch device.\n");
        is_switch = 0;
    } else {
        is_switch = 1;
    }

    sim_det_wq = create_singlethread_workqueue("sim_det_wq");
    INIT_DELAYED_WORK(&pdata->sim_det_work, sim_detect_work);

    platform_set_drvdata(pdev, pdata);

    /* initial detection */
    if (pdata->sim1_detect_gpio > 0 || pdata->sim2_detect_gpio > 0)
        queue_delayed_work(sim_det_wq, &pdata->sim_det_work, 0);
fail:
    return 0;
}

static int sim_detect_remove(struct platform_device *pdev)
{
    int ret = 0;
    struct sim_detect_drvdata *pdata =
        (struct sim_detect_drvdata *)platform_get_drvdata(pdev);
    pr_info("SIM_DETECT: %s remove resources\n", __func__);

    if (pdata->sim1_detect_irq > 0) free_irq(pdata->sim1_detect_irq, NULL);
#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
    if (pdata->sim2_detect_irq > 0) free_irq(pdata->sim2_detect_irq, NULL);
#endif
    cancel_delayed_work_sync(&pdata->sim_det_work);
    destroy_workqueue(sim_det_wq);

    if (is_switch) {
        switch_dev_unregister(&pdata->sim_switch);
        is_switch = 0;
    }

    if (pdata->sim1_detect_gpio > 0) gpio_free(pdata->sim1_detect_gpio);
#if defined (CONFIG_Z170C) || defined (CONFIG_Z170CG)
    if (pdata->sim2_detect_gpio > 0) gpio_free(pdata->sim2_detect_gpio);
#endif
    devm_kfree(&pdev->dev, pdata);
    return ret;
}

static struct of_device_id sim_detect_of_match[] = {
    { .compatible = "intel,sim_detect", },
    { },
};

static struct platform_driver sim_detect_driver = {
    .probe = sim_detect_probe,
    .remove = sim_detect_remove,
    .driver = {
        .name = NAME_SIM_PLUG,
        .owner = THIS_MODULE,
        .of_match_table = sim_detect_of_match,
    },
};

int __init sim_detect_init(void)
{
    pr_info("SIM_DETECT: %s\n", __func__);
    return platform_driver_register(&sim_detect_driver);
}

static void __exit sim_detect_exit(void)
{
    platform_driver_unregister(&sim_detect_driver);
}

module_init(sim_detect_init);
module_exit(sim_detect_exit);

MODULE_DESCRIPTION("Sim Detection Runtime Loading");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");
