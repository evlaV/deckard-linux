#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>

struct clk_fpga {
    struct clk_hw hw;
    struct pinctrl *pinctrl;
    struct pinctrl_state *active_state,*sleep_state;
    u32 fixed_rate;
    int nclks;
    struct clk_bulk_data *clks;
};

static inline struct clk_fpga *to_clk_fpga(struct clk_hw *hw)
{
    return container_of(hw, struct clk_fpga, hw);
}

static int clk_fpga_prepare(struct clk_hw *hw)
{
    struct clk_fpga *clk_fpga = to_clk_fpga(hw);
    int ret;
    ret = clk_bulk_prepare_enable(clk_fpga->nclks, clk_fpga->clks);
    if (ret < 0) {
        pr_err("failed to prepare/enable clocks %d\n", ret);
        return ret;
    }
    return 0;
}

static void clk_fpga_unprepare(struct clk_hw *hw)
{
    struct clk_fpga *clk_fpga = to_clk_fpga(hw);
    clk_bulk_disable_unprepare(clk_fpga->nclks, clk_fpga->clks);
}

static unsigned long clk_fpga_recalc_rate(struct clk_hw *hw,
                     unsigned long parent_rate)
{
    struct clk_fpga *clk_fpga = to_clk_fpga(hw);

    return clk_fpga->fixed_rate;
}

static int clk_fpga_get_duty_cycle(struct clk_hw *hw, struct clk_duty *duty)
{
    // always set to 50%
    duty->num = 1;
    duty->den = 2;

    return 0;
}

static const struct clk_ops clk_fpga_ops = {
    .prepare = clk_fpga_prepare,
    .unprepare = clk_fpga_unprepare,
    .recalc_rate = clk_fpga_recalc_rate,
    .get_duty_cycle = clk_fpga_get_duty_cycle,
};

static int clk_fpga_probe(struct platform_device *pdev)
{
    struct device_node *node = pdev->dev.of_node;
    struct clk_init_data init;
    struct clk_fpga *clk_fpga;
    const char *clk_name;
    int ret;

    struct device *dev = &pdev->dev;

    clk_fpga = devm_kzalloc(&pdev->dev, sizeof(*clk_fpga), GFP_KERNEL);
        if (!clk_fpga)
                return -ENOMEM;

    /* Clocks */
    ret = devm_clk_bulk_get_all(dev, &clk_fpga->clks);
    if (ret < 1) {
        dev_err(dev, "failed to get clocks %d\n", ret);
        return ret;
    }
    clk_fpga->nclks = ret;

    //clk_set_rate(clk_fpga->clks[0].clk, 50000000);
    clk_set_rate(clk_fpga->clks[0].clk, 33333333);

    clk_name = node->name;
    of_property_read_string(node, "clock-output-names", &clk_name);

    init.name = clk_name;
    init.ops = &clk_fpga_ops;
    init.flags = 0;
    init.num_parents = 0;

    clk_fpga->hw.init = &init;
    ret = devm_clk_hw_register(&pdev->dev, &clk_fpga->hw);

    if (ret)
        return ret;

    return of_clk_add_hw_provider(node, of_clk_hw_simple_get, &clk_fpga->hw);
}

static void clk_fpga_remove(struct platform_device *pdev)
{
    of_clk_del_provider(pdev->dev.of_node);

    return;
}

static const struct of_device_id clk_fpga_dt_ids[] = {
    { .compatible = "fpga-clock" },
    { }
};
MODULE_DEVICE_TABLE(of, clk_fpga_dt_ids);

static struct platform_driver clk_fpga_driver = {
    .probe = clk_fpga_probe,
    .remove = clk_fpga_remove,
    .driver = {
        .name = "fpga-clock",
        .of_match_table = clk_fpga_dt_ids,
    },
};

module_platform_driver(clk_fpga_driver);

MODULE_DESCRIPTION("FPGA clock driver");
MODULE_LICENSE("GPL");
