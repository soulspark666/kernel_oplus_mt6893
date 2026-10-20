// SPDX-License-Identifier: GPL-2.0-only
/*
 * MTK fuel-gauge HAL for the v2 charger framework (MT6893/Cupida).
 *
 * Production 21127 has NO external gauge (no bq27541 on any bus;
 * qcom,vooc_is_platform_gauge is set) — the battery data comes from the
 * MTK battery/fuel-gauge driver ("battery" power_supply). This HAL fronts
 * it as a v2 GAUGE IC so virtual_gauge/mms_gauge keep working.
 *
 * Only the funcs the framework actually pulls are registered (mms_gauge
 * update set + BATT_NUM/EXIST). Everything else returns -ENOTSUPP by
 * design. Units: mV / mA (signed) / % / decidegreeC-in, degreeC-out.
 */

#define pr_fmt(fmt) "[MTKFG]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/of.h>

#include <oplus_chg_module.h>
#include <oplus_chg.h>
#include <oplus_chg_ic.h>

struct mtk_fg_chip {
	struct device *dev;
	struct oplus_chg_ic_dev *ic_dev;
	struct power_supply *bat_psy;
};

static struct power_supply *mtk_fg_get_bat_psy(struct mtk_fg_chip *chip)
{
	if (chip->bat_psy && !IS_ERR(chip->bat_psy))
		return chip->bat_psy;
	/* The supply is registered by oplus_chg_gki.c once the gauge topic is
	 * up; until then this returns NULL and callers fall back to defaults. */
	chip->bat_psy = power_supply_get_by_name("battery");
	return chip->bat_psy;
}

static int mtk_fg_get_prop(struct mtk_fg_chip *chip,
			   enum power_supply_property psp, int *val)
{
	struct power_supply *psy;
	union power_supply_propval prop = { 0 };
	int rc;

	psy = mtk_fg_get_bat_psy(chip);
	if (!psy || IS_ERR(psy))
		return -ENODEV;
	rc = power_supply_get_property(psy, psp, &prop);
	if (rc < 0)
		return rc;
	*val = prop.intval;
	return 0;
}

static int mtk_fg_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int mtk_fg_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;
	ic_dev->online = false;
	return 0;
}

static int mtk_fg_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int mtk_fg_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[],
			   int len)
{
	return 0;
}

static int mtk_fg_get_vol(struct oplus_chg_ic_dev *ic_dev, int index, int *vol_mv)
{
	struct mtk_fg_chip *chip;
	int uv, rc;

	(void)index; /* single cell */
	if (!ic_dev || !vol_mv)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_VOLTAGE_NOW, &uv);
	if (rc < 0)
		return rc;
	*vol_mv = uv / 1000;
	return 0;
}

static int mtk_fg_get_max(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	/* single cell: max == pack voltage */
	return mtk_fg_get_vol(ic_dev, 0, vol_mv);
}

static int mtk_fg_get_min(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	/* single cell: min == pack voltage */
	return mtk_fg_get_vol(ic_dev, 0, vol_mv);
}

static int mtk_fg_get_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct mtk_fg_chip *chip;
	int ua, rc;

	if (!ic_dev || !curr_ma)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_CURRENT_NOW, &ua);
	if (rc < 0)
		return rc;
	*curr_ma = ua / 1000;
	return 0;
}

static int mtk_fg_get_temp(struct oplus_chg_ic_dev *ic_dev, int *temp)
{
	struct mtk_fg_chip *chip;
	int deci, rc;

	if (!ic_dev || !temp)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_TEMP, &deci);
	if (rc < 0)
		return rc;
	*temp = deci / 10;
	return 0;
}

static int mtk_fg_get_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	struct mtk_fg_chip *chip;
	int val, rc;

	if (!ic_dev || !soc)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (rc < 0)
		return rc;
	*soc = val;
	return 0;
}

static int mtk_fg_get_fcc(struct oplus_chg_ic_dev *ic_dev, int *fcc)
{
	struct mtk_fg_chip *chip;
	int uah, rc;

	if (!ic_dev || !fcc)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_CHARGE_FULL, &uah);
	if (rc < 0)
		return rc;
	*fcc = uah / 1000;
	return 0;
}

static int mtk_fg_get_cc(struct oplus_chg_ic_dev *ic_dev, int *cc)
{
	struct mtk_fg_chip *chip;
	int uah, rc;

	if (!ic_dev || !cc)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_CHARGE_COUNTER, &uah);
	if (rc < 0)
		return rc;
	*cc = uah / 1000;
	return 0;
}

static int mtk_fg_get_rm(struct oplus_chg_ic_dev *ic_dev, int *rm)
{
	int fcc, soc, rc;

	/* remaining = fcc * soc / 100 (no RM source on MTK FG) */
	rc = mtk_fg_get_fcc(ic_dev, &fcc);
	if (rc < 0)
		return rc;
	rc = mtk_fg_get_soc(ic_dev, &soc);
	if (rc < 0)
		return rc;
	*rm = fcc * soc / 100;
	return 0;
}

static int mtk_fg_get_soh(struct oplus_chg_ic_dev *ic_dev, int *soh)
{
	int fcc, rc, design;

	/* soh = fcc / design * 100; design from batt_capacity_mah? The IC
	 * has no DT context here, so report 100 when FCC reads OK and let
	 * the framework default (0) apply otherwise. Keep it simple and
	 * honest: derive from design 4500mAh (Cupida cell). */
	rc = mtk_fg_get_fcc(ic_dev, &fcc);
	if (rc < 0 || fcc <= 0)
		return rc < 0 ? rc : -EINVAL;
	design = 4500;
	*soh = fcc * 100 / design;
	if (*soh > 100)
		*soh = 100;
	return 0;
}

static int mtk_fg_get_exist(struct oplus_chg_ic_dev *ic_dev, bool *exist)
{
	struct mtk_fg_chip *chip;
	int val, rc;

	if (!ic_dev || !exist)
		return -ENODEV;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mtk_fg_get_prop(chip, POWER_SUPPLY_PROP_PRESENT, &val);
	if (rc < 0)
		return rc;
	*exist = !!val;
	return 0;
}

static int mtk_fg_get_batt_num(struct oplus_chg_ic_dev *ic_dev, int *num)
{
	if (!ic_dev || !num)
		return -ENODEV;
	*num = 1;
	return 0;
}

static void *mtk_fg_get_func(struct oplus_chg_ic_dev *ic_dev,
			     enum oplus_chg_ic_func func_id)
{
	void *func = NULL;

	if (!ic_dev->online && (func_id != OPLUS_IC_FUNC_INIT) &&
	    (func_id != OPLUS_IC_FUNC_EXIT)) {
		chg_err("%s is offline\n", ic_dev->name);
		return NULL;
	}

	switch (func_id) {
	case OPLUS_IC_FUNC_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_INIT,
					       mtk_fg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       mtk_fg_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					       mtk_fg_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       mtk_fg_smt_test);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_VOL,
					       mtk_fg_get_vol);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MAX,
					       mtk_fg_get_max);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_MIN,
					       mtk_fg_get_min);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CURR,
					       mtk_fg_get_curr);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_TEMP,
					       mtk_fg_get_temp);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOC,
					       mtk_fg_get_soc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_FCC,
					       mtk_fg_get_fcc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_CC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_CC,
					       mtk_fg_get_cc);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_RM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_RM,
					       mtk_fg_get_rm);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_SOH,
					       mtk_fg_get_soh);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_EXIST,
					       mtk_fg_get_exist);
		break;
	case OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GAUGE_GET_BATT_NUM,
					       mtk_fg_get_batt_num);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

static struct oplus_chg_ic_virq mtk_fg_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_OFFLINE },
	{ .virq_id = OPLUS_IC_VIRQ_RESUME },
};

static int mtk_fg_probe(struct platform_device *pdev)
{
	struct mtk_fg_chip *chip;
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	chip->dev = &pdev->dev;
	platform_set_drvdata(pdev, chip);

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		return rc;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		return rc;
	}

	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	snprintf(ic_cfg.manu_name, sizeof(ic_cfg.manu_name), "mtk-fg");
	snprintf(ic_cfg.fw_id, sizeof(ic_cfg.fw_id), "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = mtk_fg_get_func;
	ic_cfg.virq_data = mtk_fg_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(mtk_fg_virq_table);
	chip->ic_dev = devm_oplus_chg_ic_register(chip->dev, &ic_cfg);
	if (!chip->ic_dev) {
		chg_err("register %s error\n", node->name);
		return -ENODEV;
	}
	/* NOTE: do NOT defer on the 'battery' power_supply here. It is registered
	 * by oplus_chg_gki.c only after this IC's gauge topic comes up, so
	 * deferring deadlocks: no battery -> no topic -> no battery. The getters
	 * retry on their own (mtk_fg_get_bat_psy). */
	chg_info("register %s\n", node->name);

	return 0;
}

static int mtk_fg_remove(struct platform_device *pdev)
{
	struct mtk_fg_chip *chip = platform_get_drvdata(pdev);

	if (!chip)
		return 0;
	if (chip->ic_dev && chip->ic_dev->online) {
		chip->ic_dev->online = false;
		devm_oplus_chg_ic_unregister(&pdev->dev, chip->ic_dev);
	}
	return 0;
}

static const struct of_device_id mtk_fg_match[] = {
	{ .compatible = "oplus,mtk-fg" },
	{},
};
MODULE_DEVICE_TABLE(of, mtk_fg_match);

static struct platform_driver mtk_fg_driver = {
	.probe = mtk_fg_probe,
	.remove = mtk_fg_remove,
	.driver = {
		.name = "oplus-mtk-fg",
		.of_match_table = mtk_fg_match,
	},
};

int mtk_fg_driver_init(void)
{
	chg_debug("init start\n");
	return platform_driver_register(&mtk_fg_driver);
}

void mtk_fg_driver_exit(void)
{
	platform_driver_unregister(&mtk_fg_driver);
}
oplus_chg_module_register(mtk_fg_driver);
