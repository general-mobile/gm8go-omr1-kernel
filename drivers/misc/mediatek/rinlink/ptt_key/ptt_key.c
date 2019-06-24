#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>

#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/input.h>

#include "ptt_key.h"

#define PTT_KEY_INPUT_NAME     "PTT_KEY"
static int ptt_key_gpio = 0;
static int soc_key_gpio = 0;

extern int pinctrl_select_state(struct pinctrl *p, struct pinctrl_state *state);
static int ptt_key_setup_eint(void);

struct input_dev *ptt_key_input_dev;

static int ptt_key_irq;
static int soc_key_irq;
static struct pinctrl *ptt_keyctrl;
//static struct pinctrl_state *irqpins_default;
static struct pinctrl_state *ptt_key_eint;
static struct pinctrl_state *ptt_key_inmode;
//extern int hwgpio_to_vgpio(int gpio);

static irqreturn_t ptt_key_eint_func(int irq, void *desc)
{
    int ptt_value = 0;
	PTT_KEY_FUNC();
    ptt_value = gpio_get_value(ptt_key_gpio);
    pr_err("ptt_key_eint_func ptt_value:%d\n",ptt_value); 

    disable_irq_nosync(ptt_key_irq);
    if(ptt_value == 0){
        irq_set_irq_type(ptt_key_irq, IRQ_TYPE_LEVEL_HIGH);
        input_report_key(ptt_key_input_dev, KEY_KBDILLUMUP, 1);
        input_sync(ptt_key_input_dev);
    } else {
        irq_set_irq_type(ptt_key_irq, IRQ_TYPE_LEVEL_LOW);
        input_report_key(ptt_key_input_dev, KEY_KBDILLUMUP, 0);
        input_sync(ptt_key_input_dev);
    }
	enable_irq(ptt_key_irq);

    return IRQ_HANDLED;
}
static irqreturn_t soc_key_eint_func(int irq, void *desc)
{
    int soc_value = 0;
    PTT_KEY_FUNC();
    soc_value = gpio_get_value(soc_key_gpio);
    pr_err("ptt_key_eint_func ptt_value:%d\n",soc_value);

    disable_irq_nosync(soc_key_irq);
    if(soc_value == 0){
        irq_set_irq_type(soc_key_irq, IRQ_TYPE_LEVEL_HIGH);
        input_report_key(ptt_key_input_dev, KEY_HELP, 1);
        input_sync(ptt_key_input_dev);
    } else {
        irq_set_irq_type(soc_key_irq, IRQ_TYPE_LEVEL_LOW);
        input_report_key(ptt_key_input_dev, KEY_HELP, 0);
        input_sync(ptt_key_input_dev);
    }
    enable_irq(soc_key_irq);

    return IRQ_HANDLED;
}


static int ptt_key_setup_eint(void)
{

    unsigned int g_gpio_ptt_deb = 0;
    unsigned int g_gpio_soc_deb = 0;
    struct device_node *node_ptt_key;
    struct device_node *node_soc_key;
    int ptt_value = 0;
    int soc_value = 0;
    int ret = 0;
    
    PTT_KEY_FUNC();
    node_ptt_key = of_find_compatible_node(NULL, NULL, "mediatek,key-ptt");
    
    if(node_ptt_key)
        pr_err("find irq node success!!\n");
    else
        pr_err("null irq node!!\n");
    
    if(node_ptt_key)
    {
        ptt_key_gpio = of_get_named_gpio(node_ptt_key, "deb-gpios", 0);
        ret = of_property_read_u32(node_ptt_key, "debounce", &g_gpio_ptt_deb);
        if (ret < 0) {
            pr_err("ptt_key debounce time not found\n");
        }
		gpio_request(ptt_key_gpio, "ptt_key");
		gpio_set_debounce(ptt_key_gpio, g_gpio_ptt_deb);
		pr_err("ptt_key_gpio = %d, g_gpio_ptt_deb = %d!!\n", ptt_key_gpio, g_gpio_ptt_deb);
        ptt_value = gpio_get_value(ptt_key_gpio);
        pr_err("ptt_value = %d\n", ptt_value);
		ptt_key_irq = irq_of_parse_and_map(node_ptt_key, 0);
		pr_err("ptt_key_irq = %d\n", ptt_key_irq);
		if (!ptt_key_irq) {
			pr_err("irq_of_parse_and_map fail!!\n");
			return -EINVAL;
		}
      
        if (request_irq(ptt_key_irq, ptt_key_eint_func, (ptt_value ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH), "PTT_KEY-eint", NULL)) {
			pr_err("IRQ LINE NOT AVAILABLE!!\n");
			return -EINVAL;
		}
		enable_irq(ptt_key_irq);
	} else {
		pr_err("null irq node!!\n");
		return -EINVAL;
	}
    node_soc_key = of_find_compatible_node(NULL, NULL, "mediatek,key-soc");

    if(node_soc_key)
        pr_err("find soc irq node success!!\n");
    else
        pr_err("null soc irq node!!\n");

    if(node_soc_key)
    {
        soc_key_gpio = of_get_named_gpio(node_soc_key, "deb-gpios", 0);
        ret = of_property_read_u32(node_soc_key, "debounce", &g_gpio_soc_deb);
        if (ret < 0) {
            pr_err("soc_key debounce time not found\n");
        }
        gpio_request(soc_key_gpio, "soc_key");
        gpio_set_debounce(soc_key_gpio, g_gpio_soc_deb);
        pr_err("soc_key_gpio = %d, g_gpio_soc_deb = %d!!\n", soc_key_gpio, g_gpio_soc_deb);
        soc_value = gpio_get_value(soc_key_gpio);
        pr_err("soc_value = %d\n", soc_value);
		ptt_key_irq = irq_of_parse_and_map(node_ptt_key, 0);
        soc_key_irq = irq_of_parse_and_map(node_soc_key, 0);
        pr_err("soc_key_irq = %d\n", soc_key_irq);
        if (!soc_key_irq) {
            pr_err("irq_of_parse_and_map fail!!\n");
            return -EINVAL;
        }

        if (request_irq(soc_key_irq, soc_key_eint_func, (soc_value ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH), "SOC_KEY-eint", NULL)) {
            pr_err("Soc IRQ LINE NOT AVAILABLE!!\n");
            return -EINVAL;
        }
        enable_irq(soc_key_irq);
    } else {
        pr_err("null soc irq node!!\n");
        return -EINVAL;
    }
    
	return 0;
}

static void ptt_key_input_register(void)
{
    int err;

    ptt_key_input_dev = input_allocate_device();
    if (!ptt_key_input_dev){
        err = -ENOMEM;
        pr_err("failed to allocate input device\n");
        goto exit_input_register_device_failed;
    }

    __set_bit(EV_KEY, ptt_key_input_dev->evbit);
    __set_bit(EV_SYN, ptt_key_input_dev->evbit);

    __set_bit(KEY_KBDILLUMUP, ptt_key_input_dev->keybit);
    __set_bit(KEY_HELP, ptt_key_input_dev->keybit);

    ptt_key_input_dev->name = PTT_KEY_INPUT_NAME;
    err = input_register_device(ptt_key_input_dev);
    if (err) {
        pr_err("ptt_key_input_register: failed to register input device\n");
        goto exit_input_register_device_failed;
    }

exit_input_register_device_failed:
    input_free_device(ptt_key_input_dev);
}


static int ptt_key_probe(struct platform_device *dev)
{
	int ret = 0;
	
	PTT_KEY_FUNC();

    ptt_keyctrl = devm_pinctrl_get(&dev->dev);    
	if (IS_ERR(ptt_keyctrl)) {
		ret = PTR_ERR(ptt_keyctrl);
		pr_err("Cannot find ptt_key ptt_keyctrl!\n");
	}
    
    ptt_key_input_register();
	ptt_key_setup_eint();
	
	return 0;
}

static int ptt_key_remove(struct platform_device *dev)
{
	PTT_KEY_FUNC();
    input_unregister_device(ptt_key_input_dev);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id ptt_key_of_match[] = {
    { .compatible = "mediatek,key-ptt", },
    {},
};
#endif
const struct dev_pm_ops ptt_key_pm_ops = {
    .suspend = NULL,
    .resume = NULL,
};

static struct platform_driver ptt_key_driver = {
	.probe = ptt_key_probe,
    .shutdown = NULL,
	.remove = ptt_key_remove,
	.driver = {
        .owner	= THIS_MODULE,
		.name = "ptt_key",
        .pm = &ptt_key_pm_ops,
        #ifdef CONFIG_OF
        .of_match_table = ptt_key_of_match,
        #endif
	},
};

static int ptt_key_mod_init(void)
{
	PTT_KEY_FUNC();

	if(platform_driver_register(&ptt_key_driver) != 0)
	{
		PTT_KEY_DEBUG("unable to register ptt_key driver\n");
		return -1;
	}
	
	return 0;
}

static void ptt_key_mod_exit(void)
{
	PTT_KEY_FUNC();

	platform_driver_unregister(&ptt_key_driver);
}

module_init(ptt_key_mod_init);
module_exit(ptt_key_mod_exit);

MODULE_DESCRIPTION("Rinlink ptt_key driver");
MODULE_AUTHOR("AL <wuzhiyong@rinlink.com>");
MODULE_LICENSE("GPL");
