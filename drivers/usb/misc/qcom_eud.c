// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015-2021, The Linux Foundation. All rights reserved.
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/usb/role.h>

#define EUD_REG_COM_TX_ID	0x0000
#define EUD_REG_COM_TX_LEN	0x0004
#define EUD_REG_COM_TX_DAT	0x0008
#define EUD_REG_COM_RX_ID	0x000C
#define EUD_REG_COM_RX_LEN	0x0010
#define EUD_REG_COM_RX_DAT	0x0014

#define EUD_REG_INT1_EN_MASK	0x0024
#define EUD_REG_INT_STATUS_1	0x0044
#define EUD_REG_CTL_OUT_1	0x0074
#define EUD_REG_VBUS_INT_CLR	0x0080
#define EUD_REG_CSR_EUD_EN	0x1014
#define EUD_REG_SW_ATTACH_DET	0x1018
#define EUD_REG_EUD_EN2		0x0000

#define EUD_ENABLE		BIT(0)
#define EUD_INT_PET_EUD		BIT(0)

#define EUD_INT_RX		BIT(0)
#define EUD_INT_TX		BIT(1)
#define EUD_INT_VBUS		BIT(2)
#define EUD_INT_SAFE_MODE	BIT(4)
#define EUD_INT_ALL		(EUD_INT_RX | EUD_INT_TX | EUD_INT_VBUS | \
				 EUD_INT_SAFE_MODE)

#define EUD_NR			1
#define EUD_CONSOLE		NULL
#define UART_ID			0x90
#define MAX_FIFO_SIZE		14

#define PORT_EUD_UART		300

static struct uart_port eud_port;

struct eud_chip {
	struct device			*dev;
	struct usb_role_switch		*role_sw;
	struct phy			*usb2_phy;
	struct uart_port		*port;

	/* mode lock */
	struct mutex			mutex;
	void __iomem			*base;
	void __iomem			*mode_mgr;
	unsigned int			int_status;
	int				irq;
	bool				enabled;
	bool				usb_attached;
	enum usb_role			current_role;
};

static int eud_phy_enable(struct eud_chip *chip)
{
	int ret;

	ret = phy_init(chip->usb2_phy);
	if (ret)
		return ret;

	ret = phy_power_on(chip->usb2_phy);
	if (ret)
		phy_exit(chip->usb2_phy);

	return ret;
}

static void eud_phy_disable(struct eud_chip *chip)
{
	phy_power_off(chip->usb2_phy);
	phy_exit(chip->usb2_phy);
}

static int eud_usb_role_set(struct eud_chip *chip, enum usb_role role)
{
	struct usb_role_switch *sw;
	int ret = 0;

	mutex_lock(&chip->mutex);

	/* Avoid duplicate role handling */
	if (role == chip->current_role)
		goto err;

	sw = usb_role_switch_get(chip->dev);
	if (IS_ERR_OR_NULL(sw)) {
		dev_err(chip->dev, "failed to get usb switch\n");
		ret = -EINVAL;
		goto err;
	}

	ret = usb_role_switch_set_role(sw, role);
	usb_role_switch_put(sw);

	if (ret) {
		dev_err(chip->dev, "failed to set role\n");
		goto err;
	}
	chip->current_role = role;
err:
	mutex_unlock(&chip->mutex);

	return ret;
}

static int enable_eud(struct eud_chip *priv)
{
	int ret;

	ret = eud_phy_enable(priv);
	if (ret)
		return ret;

	writel(EUD_ENABLE, priv->base + EUD_REG_CSR_EUD_EN);
	writel(EUD_INT_VBUS | EUD_INT_SAFE_MODE, priv->base + EUD_REG_INT1_EN_MASK);

	writel((__force u32)~EUD_INT_TX, priv->base + EUD_REG_INT_STATUS_1);
	writel((__force u32)EUD_INT_RX, priv->base + EUD_REG_INT_STATUS_1);
	//writel(1, priv->mode_mgr + EUD_REG_EUD_EN2);

	return ret;
}

static void disable_eud(struct eud_chip *priv)
{
	writel(0, priv->base + EUD_REG_CSR_EUD_EN);
	writel(0, priv->mode_mgr + EUD_REG_EUD_EN2);
	eud_phy_disable(priv);
}

static ssize_t enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct eud_chip *chip = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", chip->enabled);
}

static ssize_t enable_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct eud_chip *chip = dev_get_drvdata(dev);
	bool enable;
	int ret;

	if (kstrtobool(buf, &enable))
		return -EINVAL;

	/* EUD enable is applicable only in DEVICE mode */
	if (enable && chip->current_role != USB_ROLE_DEVICE)
		return -EINVAL;

	if (enable) {
		ret = enable_eud(chip);
		if (ret) {
			dev_err(chip->dev, "failed to enable eud\n");
			return count;
		}
	} else {
		disable_eud(chip);
	}
	chip->enabled = enable;

	return count;
}

static DEVICE_ATTR_RW(enable);

static struct attribute *eud_attrs[] = {
	&dev_attr_enable.attr,
	NULL,
};
ATTRIBUTE_GROUPS(eud);

static void usb_attach_detach(struct eud_chip *chip)
{
	u32 reg;

	/* read ctl_out_1[4] to find USB attach or detach event */
	reg = readl(chip->base + EUD_REG_CTL_OUT_1);
	chip->usb_attached = reg & EUD_INT_SAFE_MODE;
}

static void pet_eud(struct eud_chip *chip)
{
	u32 reg;
	int ret;

	/* When the EUD_INT_PET_EUD in SW_ATTACH_DET is set, the cable has been
	 * disconnected and we need to detach the pet to check if EUD is in safe
	 * mode before attaching again.
	 */
	reg = readl(chip->base + EUD_REG_SW_ATTACH_DET);
	if (reg & EUD_INT_PET_EUD) {
		/* Detach & Attach pet for EUD */
		writel(0, chip->base + EUD_REG_SW_ATTACH_DET);
		/* Delay to make sure detach pet is done before attach pet */
		ret = readl_poll_timeout(chip->base + EUD_REG_SW_ATTACH_DET,
					reg, (reg == 0), 1, 100);
		if (ret) {
			dev_err(chip->dev, "Detach pet failed\n");
			return;
		}
	}
	/* Attach pet for EUD */
	writel(EUD_INT_PET_EUD, chip->base + EUD_REG_SW_ATTACH_DET);
}

static unsigned int eud_tx_empty(struct uart_port *port)
{
	u32 reg;

	/* read status register and cross check for Tx interrupt */
	reg = readl_relaxed(port->membase + EUD_REG_INT_STATUS_1);
	if (reg & EUD_INT_TX)
		return TIOCSER_TEMT;
	else
		return 0;
}

static void eud_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* Nothing to set */
}

static unsigned int eud_get_mctrl(struct uart_port *port)
{
	return 0;
}

static void eud_set_termios(struct uart_port *port, struct ktermios *new,
				const struct ktermios *old)
{
	/* Nothing to do here, but to satisfy the serial core */
}

static void eud_stop_tx(struct uart_port *port)
{
	/* Disable Tx interrupt */
	writel_relaxed((__force u32)~EUD_INT_TX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static void eud_start_tx(struct uart_port *port)
{
	/* Enable Tx interrupt */
	writel_relaxed(EUD_INT_TX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static void eud_stop_rx(struct uart_port *port)
{
	/* Disable Rx interrupt */
	writel_relaxed((__force u32)~EUD_INT_RX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static int eud_startup(struct uart_port *port)
{
	/* Enable Rx interrupt */
	writel_relaxed(EUD_INT_RX, port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();

	return 0;
}

static void eud_shutdown(struct uart_port *port)
{
	/* Disable both Tx & Rx interrupts */
	writel_relaxed((__force u32)(~EUD_INT_TX | ~EUD_INT_RX),
			port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static const char *eud_type(struct uart_port *port)
{
	return (port->type == PORT_EUD_UART) ? "EUD UART" : NULL;
}

static int eud_request_port(struct uart_port *port)
{
	/* Nothing to request */
	return 0;
}

static void eud_release_port(struct uart_port *port)
{
	/* Nothing to release */
}

static void eud_config_port(struct uart_port *port, int flags)
{
	/* set port type, clear Tx and Rx interrupts */
	port->type = PORT_EUD_UART;
	writel_relaxed((__force u32)(~EUD_INT_TX | ~EUD_INT_RX),
			port->membase + EUD_REG_INT_STATUS_1);
	/* Ensure Register Writes Complete */
	wmb();
}

static int eud_verify_port(struct uart_port *port,
				       struct serial_struct *ser)
{
	if (ser->type != PORT_UNKNOWN && ser->type != PORT_EUD_UART)
		return -EINVAL;
	return 0;
}

static void eud_uart_rx(struct eud_chip *chip)
{
	struct uart_port *port = chip->port;
	unsigned char ch, flag;
	unsigned int len;
	u32 reg;
	int i;

	reg = readl_relaxed(chip->base + EUD_REG_COM_RX_ID);
	if (reg != UART_ID) {
		dev_dbg(chip->dev, "Rx interrupt isn't for us\n");
		return;
	}
	/* Read Rx Len & Data registers */
	spin_lock(&port->lock);
	len = readl_relaxed(chip->base + EUD_REG_COM_RX_LEN);
	for (i = 0; i < len; i++) {
		ch = readl_relaxed(chip->base + EUD_REG_COM_RX_DAT);
		flag = TTY_NORMAL;
		port->icount.rx++;

		if (uart_handle_sysrq_char(port, ch))
			continue;
		uart_insert_char(port, 0, 0, ch, flag);
	}

	spin_unlock(&port->lock);
	tty_flip_buffer_push(&port->state->port);
}

static void eud_uart_tx(struct eud_chip *chip)
{
	struct uart_port *port = chip->port;
	struct tty_port *tport = &port->state->port;
	unsigned int len;
	u32 reg;

	writel_relaxed(UART_ID, chip->base + EUD_REG_COM_TX_ID);
	reg = readl_relaxed(chip->base + EUD_REG_COM_TX_ID);
	if (reg != UART_ID) {
		dev_dbg(chip->dev, "Tx interrupt isn't for us\n");
		return;
	}
	/* Write to Tx Len & Data registers */
	spin_lock(&port->lock);
	len = kfifo_len(&tport->xmit_fifo);
	if (len > 0) {
		if (len > port->fifosize)
			len = port->fifosize;
		while (len--) {
			u8 c;

			if (!kfifo_get(&tport->xmit_fifo, &c))
				break;
			writel_relaxed(c, port->membase + EUD_REG_COM_TX_DAT);
			port->icount.tx++;
		}
	}
	spin_unlock(&port->lock);
}

static int eud_serial_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';

	if (co->index < 0 || co->index > EUD_NR)
		return -EINVAL;

	port = &eud_port;
	if (!port->membase)
		return -ENODEV;

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);


	return uart_set_options(port, co, baud, parity, bits, flow);
}

static void eud_serial_write_char(struct uart_port *port, unsigned char c)
{
	return;

	while (readl_relaxed(port->membase + EUD_REG_INT_STATUS_1) & EUD_INT_TX)
		cpu_relax();

	writel_relaxed(c, port->membase + EUD_REG_COM_TX_DAT);
	port->icount.tx++;
}

static void eud_serial_console_write(struct console *co, const char *s,
				     unsigned int count)
{
	struct uart_port *port;

	port = &eud_port;

	uart_console_write(port, s, count, eud_serial_write_char);
}

/* serial functions supported */
static const struct uart_ops eud_uart_ops = {
	.tx_empty	= eud_tx_empty,
	.set_mctrl	= eud_set_mctrl,
	.get_mctrl	= eud_get_mctrl,
	.set_termios	= eud_set_termios,
	.stop_tx	= eud_stop_tx,
	.start_tx	= eud_start_tx,
	.stop_rx	= eud_stop_rx,
	.startup	= eud_startup,
	.shutdown	= eud_shutdown,
	.type		= eud_type,
	.release_port	= eud_release_port,
	.request_port	= eud_request_port,
	.config_port	= eud_config_port,
	.verify_port	= eud_verify_port,
};

static struct uart_driver eud_uart_driver;

static struct console eud_console = {
	.name = "ttyEUD",
	.write = eud_serial_console_write,
	.device = uart_console_device,
	.setup = eud_serial_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &eud_uart_driver,
};

static struct uart_driver eud_uart_driver = {
	.owner		= THIS_MODULE,
	.driver_name	= "qcom-eud",
	.dev_name	= "ttyEUD",
	.nr		= EUD_NR,
	.cons		= &eud_console,
};

static int __init eud_serial_console_init(void)
{
	dump_stack();
	register_console(&eud_console);
	return 0;
}

console_initcall(eud_serial_console_init);

static irqreturn_t handle_eud_irq(int irq, void *data)
{
	struct eud_chip *chip = data;
	u32 reg;

	reg = readl(chip->base + EUD_REG_INT_STATUS_1);
	switch (reg & EUD_INT_ALL) {
	case EUD_INT_RX:
		eud_uart_rx(chip);
		return IRQ_HANDLED;
	case EUD_INT_TX:
		eud_uart_tx(chip);
		return IRQ_HANDLED;
	case EUD_INT_VBUS:
		usb_attach_detach(chip);
		return IRQ_WAKE_THREAD;
	case EUD_INT_SAFE_MODE:
		pet_eud(chip);
		return IRQ_HANDLED;
	default:
		return IRQ_NONE;
	}
}

static irqreturn_t handle_eud_irq_thread(int irq, void *data)
{
	struct eud_chip *chip = data;
	int ret;

	if (chip->usb_attached)
		ret = eud_usb_role_set(chip, USB_ROLE_DEVICE);
	else
		ret = eud_usb_role_set(chip, USB_ROLE_HOST);

	/* set and clear vbus_int_clr[0] to clear interrupt */
	writel(BIT(0), chip->base + EUD_REG_VBUS_INT_CLR);
	writel(0, chip->base + EUD_REG_VBUS_INT_CLR);

	return IRQ_HANDLED;
}

static int eud_usb_role_switch_set(struct usb_role_switch *sw,
				   enum usb_role role)
{
	struct eud_chip *chip = usb_role_switch_get_drvdata(sw);

	return eud_usb_role_set(chip, role);
}

static int eud_uart_init(struct eud_chip *chip)
{
	struct uart_port *port;
	int ret;

	port = chip->port;
	port->line = 0;
	port->type = PORT_EUD_UART;
	port->dev = chip->dev;
	port->fifosize = MAX_FIFO_SIZE;
	port->iotype = SERIAL_IO_MEM;
	port->flags = UPF_BOOT_AUTOCONF;
	port->membase = chip->base;
	port->irq = chip->irq;
	port->ops = &eud_uart_ops;

	ret = uart_add_one_port(&eud_uart_driver, port);
	if (ret < 0) {
		dev_err(chip->dev, "failed to add uart port!\n");
		return ret;
	}

	return 0;
}

static int eud_probe(struct platform_device *pdev)
{
	struct eud_chip *chip;
	struct usb_role_switch_desc eud_role_switch = {NULL};
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;

	chip->usb2_phy = devm_phy_get(chip->dev, "usb2-phy");
	if (IS_ERR(chip->usb2_phy))
		return dev_err_probe(chip->dev, PTR_ERR(chip->usb2_phy),
				     "no usb2 phy configured\n");

	chip->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(chip->base))
		return PTR_ERR(chip->base);

	chip->mode_mgr = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(chip->mode_mgr))
		return PTR_ERR(chip->mode_mgr);

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0)
		return chip->irq;

	ret = devm_request_threaded_irq(&pdev->dev, chip->irq, handle_eud_irq,
			handle_eud_irq_thread, IRQF_ONESHOT, NULL, chip);
	if (ret)
		return dev_err_probe(chip->dev, ret, "failed to allocate irq\n");

	eud_role_switch.fwnode = dev_fwnode(chip->dev);
	eud_role_switch.set = eud_usb_role_switch_set;
	eud_role_switch.get = NULL;
	eud_role_switch.driver_data = chip;
	chip->role_sw = usb_role_switch_register(chip->dev, &eud_role_switch);

	if (IS_ERR(chip->role_sw))
		return dev_err_probe(chip->dev, PTR_ERR(chip->role_sw),
				"failed to register role switch\n");

	mutex_init(&chip->mutex);

	chip->port = &eud_port;

	ret = eud_uart_init(chip);
	if (ret < 0)
		return dev_err_probe(chip->dev, ret, "failed to init uart\n");

	enable_irq_wake(chip->irq);

	platform_set_drvdata(pdev, chip);

	enable_eud(chip);

	return 0;
}

static void eud_remove(struct platform_device *pdev)
{
	struct eud_chip *chip = platform_get_drvdata(pdev);

	if (chip->enabled)
		disable_eud(chip);

	if (chip->role_sw)
		usb_role_switch_unregister(chip->role_sw);

	device_init_wakeup(&pdev->dev, false);
	disable_irq_wake(chip->irq);
}

static const struct of_device_id eud_dt_match[] = {
	{ .compatible = "qcom,eud" },
	{ .compatible = "qcom,sm8450-eud" },
	{ .compatible = "qcom,sm8650-eud" },
	{ }
};
MODULE_DEVICE_TABLE(of, eud_dt_match);

static struct platform_driver eud_driver = {
	.probe	= eud_probe,
	.remove = eud_remove,
	.driver	= {
		.name = "qcom_eud",
		.dev_groups = eud_groups,
		.of_match_table = eud_dt_match,
	},
};

static int __init eud_init(void)
{
	int ret;

	ret = uart_register_driver(&eud_uart_driver);
	if (ret < 0)
		return ret;

	ret = platform_driver_register(&eud_driver);
	if (ret < 0)
		uart_unregister_driver(&eud_uart_driver);

	return ret;
}

static void __exit eud_exit(void)
{
	platform_driver_unregister(&eud_driver);
	uart_unregister_driver(&eud_uart_driver);
}

module_init(eud_init);
module_exit(eud_exit);

MODULE_DESCRIPTION("QTI EUD driver");
MODULE_LICENSE("GPL v2");
