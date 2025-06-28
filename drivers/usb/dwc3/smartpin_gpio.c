#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/list.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <../../gpio/gpiolib.h>

#include <linux/gpio/machine.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/iio/consumer.h>
#include <linux/usb/role.h>
#include <linux/usb/dwc3-msm.h>


struct smartpin_gpio {
	struct device *dev;
	struct delayed_work sp_work;
	int irq_gpio; // gpio27
	int irq;
	int typec_vbus_det_gpio; // gpio60
	int sp_vbus_det_gpio; // gpio97
	struct mutex lock;
};


static const struct of_device_id smartpin_of_match[] = {
	{ .compatible = "qcom,smartpin_otg", },
	{},
};


static irqreturn_t smartpin_irq_handler(int irq, void *dev_id)
{
	struct smartpin_gpio *sp = dev_id;

	dev_err(sp->dev, "%s ---\n", __func__);

	schedule_delayed_work(&sp->sp_work, msecs_to_jiffies(1000));

	return IRQ_HANDLED;
}

static void smartpin_otg_work(struct work_struct *w)
{
	struct smartpin_gpio *sp = container_of(w, struct smartpin_gpio, sp_work.work);

	int gpio27_val;

	mutex_lock(&sp->lock);
	dev_err(sp->dev, "%s---\n", __func__);

	gpio27_val = gpio_get_value(sp->irq_gpio);
	pr_err("smartpin_otg_work: gpio27 = %d\n", gpio27_val);

	if(gpio27_val == 0){
		smartpin_dwc3_msm_set_role(USB_ROLE_HOST);
		pr_err("ontim: smartpin otg in, set role host\n");
	}
	else if(gpio27_val == 1){
		smartpin_dwc3_msm_set_role(USB_ROLE_NONE);
		pr_err("ontim: smartpin otg out, set role none\n");
	}
	mutex_unlock(&sp->lock);

	return;
}


static int smartpin_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct device_node *node = NULL;
	struct smartpin_gpio *sp;
	int gpio27_val;
	int gpio60_val;
	int gpio97_val;
	u32 irq_gpio_debounce = 0;

	pr_err("ontim: smartpin_probe entry\n");
	node = of_find_matching_node(NULL, smartpin_of_match);
	if (!node) {
		pr_err("can't find compatible node\n");
		return -1;
	}

	sp = devm_kzalloc(&pdev->dev, sizeof(*sp), GFP_KERNEL);
	if (!sp)
		return -ENOMEM;

	sp->dev = &pdev->dev;
	platform_set_drvdata(pdev, sp);

	INIT_DELAYED_WORK(&sp->sp_work, smartpin_otg_work);
	mutex_init(&sp->lock);

	ret = of_property_read_u32(node, "debounce", &irq_gpio_debounce);
	if (ret < 0) {
		pr_err("gpiodebounce not found,ret:%d\n", ret);
		return ret;
	}


	// gpio27
	sp->irq_gpio = of_get_named_gpio(node, "sp,irq-gpio", 0);
	if (!gpio_is_valid(sp->irq_gpio)) {
		pr_err("fail to valid irq gpio27 : %d\n", sp->irq_gpio);
		return -EINVAL;
	}

	ret = gpio_request_one(sp->irq_gpio, GPIOF_DIR_IN, "smartpin_otg_irq");
	if (ret) {
		pr_err("fail to request irq_gpio\n");
		return -EINVAL;
	}

	// gpio60
	sp->typec_vbus_det_gpio = of_get_named_gpio(node, "vbus-det1-gpio", 0);
	if (!gpio_is_valid(sp->typec_vbus_det_gpio)) {
		pr_err("fail to valid gpio60 : %d\n", sp->typec_vbus_det_gpio);
		return -EINVAL;
	}

	ret = gpio_request_one(sp->typec_vbus_det_gpio, GPIOF_DIR_IN, "typec_vbus_det");
	if (ret) {
		pr_err("fail to request typec_vbus_det_gpio\n");
		return -EINVAL;
	}

	// gpio97
	sp->sp_vbus_det_gpio = of_get_named_gpio(node, "vbus-det2-gpio", 0);
	if (!gpio_is_valid(sp->sp_vbus_det_gpio)) {
		pr_err("fail to valid gpio97 : %d\n", sp->sp_vbus_det_gpio);
		return -EINVAL;
	}

	ret = gpio_request_one(sp->sp_vbus_det_gpio, GPIOF_DIR_IN, "sp_vbus_det");
	if (ret) {
		pr_err("fail to request sp_vbus_det_gpio\n");
		return -EINVAL;
	}

	ret = gpio_direction_input(sp->irq_gpio);
	if (ret < 0) {
		pr_err("gpio-%d input set fail\n", sp->irq_gpio);
		return ret;
	}

	gpio_set_debounce(sp->irq_gpio, irq_gpio_debounce);
	pr_err("gpio_num<%d>debounce<%d>,\n", sp->irq_gpio, irq_gpio_debounce);

	sp->irq =gpio_to_irq(sp->irq_gpio);
	if (sp->irq < 0) {
		pr_err("fail to gpio to irq\n");
		return -EINVAL;
	}

	ret = devm_request_threaded_irq(sp->dev, sp->irq, NULL, smartpin_irq_handler,
					IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "smartpin_otg", sp);
	if (ret) {
		pr_err("Request IRQ failed!ERRNO:%d.", ret);
		return -1;
	}

	enable_irq_wake(sp->irq);

	gpio27_val = gpio_get_value(sp->irq_gpio);
	pr_err("smartpin_probe: gpio27 = %d\n", gpio27_val);
	gpio60_val = gpio_get_value(sp->typec_vbus_det_gpio);
	pr_err("smartpin_probe: gpio60 = %d\n", gpio60_val);
	gpio97_val = gpio_get_value(sp->sp_vbus_det_gpio);
	pr_err("smartpin_probe: gpio97 = %d\n", gpio97_val);

	if(gpio27_val == 0)
		schedule_delayed_work(&sp->sp_work, msecs_to_jiffies(5000));

	return 0;
}

static struct platform_driver smartpin_driver = {
	.probe = smartpin_probe,
	.driver = {
		.name = "smartpin_driver",
		.owner	= THIS_MODULE,
		.of_match_table = smartpin_of_match,
	},
};

static int __init smartpin_mod_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&smartpin_driver);
 	if (ret)
 	   pr_err("platform_driver_register error:(%d)\n", ret);

  	return ret;
}
module_init(smartpin_mod_init);

static void __exit smartpin_mod_exit(void)
{
	platform_driver_unregister(&smartpin_driver);
}
module_exit(smartpin_mod_exit);

MODULE_AUTHOR("houzenan");
MODULE_DESCRIPTION("smartpin otg irq driver");
MODULE_LICENSE("GPL");
