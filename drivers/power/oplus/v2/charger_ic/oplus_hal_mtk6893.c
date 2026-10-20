// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2022 Oplus. All rights reserved.
 */

#define pr_fmt(fmt) "[MTK6893]([%s][%d]): " fmt, __func__, __LINE__

#include <linux/init.h>		/* For init/exit macros */
#include <linux/module.h>	/* For MODULE_ marcros  */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/kdev_t.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/power_supply.h>
#include <linux/pm_wakeup.h>
#include <linux/time.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/suspend.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/reboot.h>
#include <linux/mfd/mt6397/core.h>/* PMIC MFD core header */
#include <linux/regmap.h>
#include <linux/of_platform.h>

#include <asm/setup.h>

#include "oplus_hal_mtk6893.h"

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/iio/consumer.h>

#include <oplus_chg_module.h>
#include <oplus_chg_ic.h>
#include <oplus_mms.h>
#include <oplus_mms_wired.h>
#include <oplus_chg_comm.h>
#include <oplus_chg_vooc.h>
#include <oplus_chg_voter.h>
#include <tcpm.h>
#include <mt6360_pmu.h>
/* Minimal view of MTK's struct battery_data (mediatek/battery/
 * mtk_battery_internal.h) — just enough to publish charging state.
 * That header is too heavy to include here (gauge class deps). */
struct mtk_battery_data {
	struct power_supply_desc psd;
	struct power_supply *psy;
	int BAT_STATUS;
};
extern struct mtk_battery_data battery_main;
#include <mt-plat/v1/charger_type.h>
/* mt-plat/v1/mtk_charger.h shares the include guard with this file's own
 * header (both use __MTK_CHARGER_H); drop it so the real MTK definitions
 * (struct charger_consumer, CHARGER_NOTIFY_*, MAIN_CHARGER) land here.
 * mtk_battery.c dereferences struct charger_consumer, so both translation
 * units must agree on its layout. */
#undef __MTK_CHARGER_H
#undef __MTK_CHARGER_H__
#include <mt-plat/v1/mtk_charger.h>
#include <mt-plat/mtk_boot_common.h>
#include <oplus_mms_gauge.h>
#define POWER_SUPPLY_TYPE_USB_HVDCP 13

static struct mtk_charger *pinfo;

/* Charger-manager consumer list. MTK's charger_manager_get_by_name() and
 * register_charger_manager_notifier() live in mtk_charger.c, which the
 * OPLUS tree excludes, so mtk_battery.c was only seeing the weak
 * NULL-returning stubs in mtk_battery.c. That left gm.pbat_consumer NULL,
 * battery_callback() unregistered and battery_main.BAT_STATUS frozen at
 * DISCHARGING -- i.e. no charging indicator no matter what the charger
 * did. Provide the real (single-manager) implementation here. */
static DEFINE_MUTEX(consumer_mutex);
static LIST_HEAD(consumer_head);

struct charger_consumer *charger_manager_get_by_name(struct device *dev,
	const char *supply_name)
{
	struct charger_consumer *puser;

	(void)supply_name;

	if (!pinfo)
		return NULL;

	puser = kzalloc(sizeof(*puser), GFP_KERNEL);
	if (!puser)
		return NULL;

	mutex_lock(&consumer_mutex);
	puser->dev = dev;
	puser->cm = pinfo;
	list_add(&puser->list, &consumer_head);
	mutex_unlock(&consumer_mutex);

	return puser;
}
EXPORT_SYMBOL(charger_manager_get_by_name);

int register_charger_manager_notifier(struct charger_consumer *consumer,
	struct notifier_block *nb)
{
	int ret = 0;
	struct mtk_charger *info;

	if (!consumer || !nb)
		return -EINVAL;

	info = consumer->cm;

	mutex_lock(&consumer_mutex);
	if (info)
		ret = srcu_notifier_chain_register(&info->evt_nh, nb);
	else
		consumer->pnb = nb;
	mutex_unlock(&consumer_mutex);

	return ret;
}
EXPORT_SYMBOL(register_charger_manager_notifier);

int unregister_charger_manager_notifier(struct charger_consumer *consumer,
	struct notifier_block *nb)
{
	struct mtk_charger *info;

	if (!consumer || !nb)
		return -EINVAL;

	info = consumer->cm;

	if (info)
		return srcu_notifier_chain_unregister(&info->evt_nh, nb);

	return 0;
}

extern enum charger_type mt_get_charger_type(void);
extern int oplus_chg_track_check_wired_charging_break(int vbus_rising);
extern int mt6360_get_vbus_rising(void);
extern int mt6360_suspend_charger(bool suspend);
extern int mt6360_set_rechg_voltage(int rechg_mv);
extern int mt6360_reset_charger(void);
extern int mt6360_aicl_enable(bool enable);
extern int mt6360_set_register(u8 addr, u8 mask, u8 data);
extern int mt6360_enter_shipmode(void);
extern int mt6360_chg_enable_wdt(bool enable);
extern enum power_supply_type mt6360_get_hvdcp_type(void);
extern void mt6360_enable_hvdcp_detect(void);
extern void oplus_notify_hvdcp_detect_stat(void);
extern void oplus_set_hvdcp_flag_clear(void);
/* MT6360 DPDM switch codes (v1 oplus_chg_set_qc_config_forsvooc):
 * force 5V first, then 9V. No MT6375 5V/9V helpers on this platform. */
#define MT6360_HVDCP_FORCE_5V	0x15
#define MT6360_HVDCP_FORCE_9V	0x18
static int mt6360_set_hvdcp_to_5v(void)
{
	return mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F,
				   MT6360_HVDCP_FORCE_5V);
}
static int mt6360_set_hvdcp_to_9v(void)
{
	return mt6360_set_register(MT6360_PMU_DPDM_CTRL, 0x1F,
				   MT6360_HVDCP_FORCE_9V);
}
#endif

/* TODO V2 */
int oplus_chg_wake_update_work(void)
{
	return 0;
}
EXPORT_SYMBOL(oplus_chg_wake_update_work);

bool oplus_chg_check_chip_is_null(void)
{
	return true;
}
EXPORT_SYMBOL(oplus_chg_check_chip_is_null);

int oplus_chg_check_ui_soc(void)
{
	return true;
}
EXPORT_SYMBOL(oplus_chg_check_ui_soc);

int oplus_is_vooc_project(void)
{
	int vooc_project = 0;
	struct oplus_mms *vooc_topic;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	vooc_project = oplus_vooc_get_project(vooc_topic);

	chr_info("get vooc project = %d\n", vooc_project);
	return vooc_project;
}
EXPORT_SYMBOL(oplus_is_vooc_project);

int oplus_chg_show_vooc_logo_ornot(void)
{
	int vooc_online_true = 0;
	int online = 0;
	int online_keep = 0;
	struct oplus_mms *vooc_topic;
	union mms_msg_data data = { 0 };
	int rc;

	vooc_topic = oplus_mms_get_by_name("vooc");
	if (!vooc_topic)
		return 0;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_ONLINE, &data, false);
	if (!rc)
		online = data.intval;

	rc = oplus_mms_get_item_data(vooc_topic, VOOC_ITEM_ONLINE_KEEP, &data, false);
	if (!rc)
		online_keep = data.intval;

	vooc_online_true = online || online_keep;

	chr_info("get vooc online = %d\n", vooc_online_true);
	return vooc_online_true;
}
EXPORT_SYMBOL(oplus_chg_show_vooc_logo_ornot);

int oplus_get_prop_status(void)
{
	int prop_status = 0;
	struct oplus_mms *comm_topic;
	union mms_msg_data data = { 0 };
	int rc;

	comm_topic = oplus_mms_get_by_name("common");
	if (!comm_topic)
		return 0;

	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_BATT_STATUS, &data, true);
	if (!rc)
		prop_status = data.intval;

	chr_info("get prop_status = %d\n", prop_status);

	return prop_status;
}
EXPORT_SYMBOL(oplus_get_prop_status);

bool oplus_mt_get_vbus_status(void)
{
	struct mtk_charger *info = pinfo;

	if (!info) {
		pr_err("pinfo not found\n");
		return false;
	}

	if (!pinfo->tcpc)
		return mt6360_get_vbus_rising();

	if(tcpm_inquire_typec_attach_state(pinfo->tcpc) == TYPEC_ATTACHED_SRC)
		return false;

	if (pinfo->chrdet_state)
		return true;
	else if (mt6360_get_vbus_rising())
		return true;
	else
		return false;
}
EXPORT_SYMBOL(oplus_mt_get_vbus_status);

int oplus_chg_get_notify_flag(void)
{
	int notify_flag = 0;
	struct oplus_mms *comm_topic;
	union mms_msg_data data = { 0 };
	int rc;

	comm_topic = oplus_mms_get_by_name("common");
	if (!comm_topic)
		return 0;

	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_NOTIFY_FLAG, &data, false);
	if (!rc)
		notify_flag = data.intval;

	chr_info("get notify code is %d\n", notify_flag);
	return notify_flag;
}
EXPORT_SYMBOL(oplus_chg_get_notify_flag);

int oplus_chg_get_ui_soc(void)
{
	int ui_soc = 0;
	struct oplus_mms *comm_topic;
	union mms_msg_data data = { 0 };
	int rc;

	comm_topic = oplus_mms_get_by_name("common");
	if (!comm_topic)
		return 0;

	rc = oplus_mms_get_item_data(comm_topic, COMM_ITEM_UI_SOC, &data, false);
	if (!rc) {
		ui_soc = data.intval;
		if (ui_soc < 0) {
			chg_err("ui soc not ready, rc=%d\n", ui_soc);
			ui_soc = 0;
		}
	}

	chr_info("get ui soc = %d\n", ui_soc);
	return ui_soc;
}
EXPORT_SYMBOL(oplus_chg_get_ui_soc);

int oplus_chg_get_voocphy_support(void)
{
	return 0;
}
EXPORT_SYMBOL(oplus_chg_get_voocphy_support);

int oplus_force_get_subboard_temp(void)
{
	return 250;
}
EXPORT_SYMBOL(oplus_force_get_subboard_temp);

struct oplus_gauge_chip {
	int gauge_unused;
};

void oplus_gauge_init(struct oplus_gauge_chip *chip)
{
	return;
}
EXPORT_SYMBOL(oplus_gauge_init);
/*end TODO V2*/

/* v2-native battery source: the "gauge" MMS topic (bq27541-backed).
 * Falls back to safe defaults until the topic is up. Units: SOC %, VOL mV,
 * CURR mA, TEMP decidegreeC (Oplus convention), EXIST bool. */
static int mtk6893_mms_gauge_intval(u32 item_id, int fallback)
{
	struct oplus_mms *gauge_topic;
	union mms_msg_data data = { 0 };
	int rc;

	gauge_topic = oplus_mms_get_by_name("gauge");
	if (!gauge_topic)
		return fallback;

	rc = oplus_mms_get_item_data(gauge_topic, item_id, &data, false);
	if (rc < 0)
		return fallback;

	return data.intval;
}

int get_uisoc(struct mtk_charger *info)
{
	int ret;

	(void)info;
	ret = mtk6893_mms_gauge_intval(GAUGE_ITEM_SOC, 50);

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_voltage(struct mtk_charger *info)
{
	int ret;

	(void)info;
	/* max cell mV; v1 batt_volt thresholds (4100/4438) are cell-level */
	ret = mtk6893_mms_gauge_intval(GAUGE_ITEM_VOL_MAX, 3999);

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_temperature(struct mtk_charger *info)
{
	int ret = 0;

	(void)info;
	ret = mtk6893_mms_gauge_intval(GAUGE_ITEM_TEMP, 270) / 10;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_battery_current(struct mtk_charger *info)
{
	int ret = 0;

	(void)info;
	ret = mtk6893_mms_gauge_intval(GAUGE_ITEM_CURR, 0);

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

static int get_pmic_vbus(struct mtk_charger *info, int *vchr)
{
	union power_supply_propval prop;
	static struct power_supply *chg_psy;
	int ret;

	if (chg_psy == NULL)
		chg_psy = power_supply_get_by_name("mtk_charger_type");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		ret = -1;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	}
	*vchr = prop.intval;

	chr_debug("%s vbus:%d\n", __func__,
		prop.intval);
	return ret;
}

int get_vbus(struct mtk_charger *info)
{
	int ret = 0;
	int vchr = 0;

	if (info == NULL)
		return 0;
	ret = charger_dev_get_vbus(info->chg1_dev, &vchr);
	if (ret < 0) {
		ret = get_pmic_vbus(info, &vchr);
		if (ret < 0)
			chr_err("%s: get vbus failed: %d\n", __func__, ret);
	} else
		vchr /= 1000;

	return vchr;
}

int get_ibat(struct mtk_charger *info)
{
	int ret = 0;
	int ibat = 0;

	if (info == NULL)
		return -EINVAL;
	ret = charger_dev_get_ibat(info->chg1_dev, &ibat);
	if (ret < 0)
		chr_err("%s: get ibat failed: %d\n", __func__, ret);

	return ibat / 1000;
}

int get_ibus(struct mtk_charger *info)
{
	int ret = 0;
	int ibus = 0;

	if (info == NULL)
		return -EINVAL;
	ret = charger_dev_get_ibus(info->chg1_dev, &ibus);
	if (ret < 0)
		chr_err("%s: get ibus failed: %d\n", __func__, ret);

	return ibus / 1000;
}

bool is_battery_exist(struct mtk_charger *info)
{
	int ret = 0;

	(void)info;
	ret = mtk6893_mms_gauge_intval(GAUGE_ITEM_BATT_EXIST, 1);

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

bool is_charger_exist(struct mtk_charger *info)
{
	int ret;

	(void)info;
	/* MT6360 VBUS rising status; no "charger" power_supply on MT6893 */
	ret = oplus_mt_get_vbus_status();

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

/* MT6360-native charger type (v1 mt_power_supply_type_check).
 * No "charger" power_supply exists on MT6893; query the PMIC’s BC12
 * result directly (built-in link, same as v1 — no EXPORT needed). */
int get_charger_type(struct mtk_charger *info)
{
	int charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	(void)info;

	switch (mt_get_charger_type()) {
	case CHARGER_UNKNOWN:
		break;
	case STANDARD_HOST:
		charger_type = POWER_SUPPLY_TYPE_USB;
		break;
	case CHARGING_HOST:
		charger_type = POWER_SUPPLY_TYPE_USB_CDP;
		break;
	case NONSTANDARD_CHARGER:
	case STANDARD_CHARGER:
	case APPLE_2_1A_CHARGER:
	case APPLE_1_0A_CHARGER:
	case APPLE_0_5A_CHARGER:
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;
		break;
	default:
		break;
	}

	/* Neither the TCPC usbid IRQ nor BC1.2 ever reports attach on this
	 * board (mt6360 PMIC raises chrdet but the MT6360 TCPC stays silent),
	 * so mt_get_charger_type() is permanently CHARGER_UNKNOWN. VBUS is
	 * however live and correct (49xx mV when a brick is connected), and
	 * the MT6360 charger default-enables its FET when pwr_rdy is set.
	 * Fall back to "some supply is attached" so the charging algorithm,
	 * the charger-manager notifier and the UI state machine all run. */
	if (charger_type == POWER_SUPPLY_TYPE_UNKNOWN && pinfo != NULL &&
	    get_vbus(pinfo) > 4000)
		charger_type = POWER_SUPPLY_TYPE_USB_DCP;

	chr_debug("%s type:%d\n", __func__, charger_type);

	return charger_type;
}

int get_usb_type(struct mtk_charger *info)
{
	int charger_type;
	int usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;

	charger_type = get_charger_type(info);
	switch (charger_type) {
	case POWER_SUPPLY_TYPE_USB:
		usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		break;
	}

	chr_debug("%s usb_type:%d\n", __func__, usb_type);
	return usb_type;
}

int get_charger_temperature(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int tchg_min = 0, tchg_max = 0;

	if (info == NULL)
		return 0;

	ret = charger_dev_get_temperature(chg, &tchg_min, &tchg_max);
	if (ret < 0)
		chr_err("%s: get temperature failed: %d\n", __func__, ret);
	else
		ret = (tchg_max + tchg_min) / 2;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_charging_current(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int oldua = 0;

	if (info == NULL)
		return 0;
	ret = charger_dev_get_charging_current(chg, &oldua);
	if (ret < 0)
		chr_err("%s: get charging current failed: %d\n", __func__, ret);
	else
		ret = oldua;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_input_current(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int oldua = 0;

	if (info == NULL)
		return 0;
	ret = charger_dev_get_input_current(chg, &oldua);
	if (ret < 0)
		chr_err("%s: get input current failed: %d\n", __func__, ret);
	else
		ret = oldua;

	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

int get_charger_zcv(struct mtk_charger *info,
	struct charger_device *chg)
{
	int ret = 0;
	int zcv = 0;

	if (info == NULL)
		return 0;

	ret = charger_dev_get_zcv(chg, &zcv);
	if (ret < 0)
		chr_err("%s: get charger zcv failed: %d\n", __func__, ret);
	else
		ret = zcv;
	chr_debug("%s:%d\n", __func__,
		ret);
	return ret;
}

#define PMIC_RG_VCDT_HV_EN_ADDR		0xb88
#define PMIC_RG_VCDT_HV_EN_MASK		0x1
#define PMIC_RG_VCDT_HV_EN_SHIFT	11

static void pmic_set_register_value(struct regmap *map,
	unsigned int addr,
	unsigned int mask,
	unsigned int shift,
	unsigned int val)
{
	regmap_update_bits(map,
		addr,
		mask << shift,
		val << shift);
}

int disable_hw_ovp(struct mtk_charger *info, int en)
{
	struct device_node *pmic_node;
	struct platform_device *pmic_pdev;
	struct mt6397_chip *chip;
	struct regmap *regmap;

	pmic_node = of_parse_phandle(info->pdev->dev.of_node, "pmic", 0);
	if (!pmic_node) {
		chr_err("get pmic_node fail\n");
		return -1;
	}

	pmic_pdev = of_find_device_by_node(pmic_node);
	if (!pmic_pdev) {
		chr_err("get pmic_pdev fail\n");
		return -1;
	}
	chip = dev_get_drvdata(&(pmic_pdev->dev));

	if (!chip) {
		chr_err("get chip fail\n");
		return -1;
	}

	regmap = chip->regmap;

	pmic_set_register_value(regmap,
		PMIC_RG_VCDT_HV_EN_ADDR,
		PMIC_RG_VCDT_HV_EN_SHIFT,
		PMIC_RG_VCDT_HV_EN_MASK,
		en);

	return 0;
}

struct tag_bootmode {
	u32 size;
	u32 tag;
	u32 bootmode;
	u32 boottype;
};

#ifdef MODULE
static char __chg_cmdline[COMMAND_LINE_SIZE];
static char *chg_cmdline = __chg_cmdline;

const char *chg_get_cmd(void)
{
	struct device_node * of_chosen = NULL;
	char *bootargs = NULL;

	if (__chg_cmdline[0] != 0)
		return chg_cmdline;

	of_chosen = of_find_node_by_path("/chosen");
	if (of_chosen) {
		bootargs = (char *)of_get_property(
					of_chosen, "bootargs", NULL);
		if (!bootargs)
			chr_err("%s: failed to get bootargs\n", __func__);
		else {
			strcpy(__chg_cmdline, bootargs);
			chr_err("%s: bootargs: %s\n", __func__, bootargs);
		}
	} else
		chr_err("%s: failed to get /chosen \n", __func__);

	return chg_cmdline;
}

#else
const char *chg_get_cmd(void)
{
	return saved_command_line;
}
#endif

int chr_get_debug_level(void)
{
	struct power_supply *psy;
	static struct mtk_charger *info;
	int ret;

	if (info == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (psy == NULL)
			ret = CHRLOG_DEBUG_LEVEL;
		else {
			info =
			(struct mtk_charger *)power_supply_get_drvdata(psy);
			if (info == NULL)
				ret = CHRLOG_DEBUG_LEVEL;
			else
				ret = info->log_level;
		}
	} else
		ret = info->log_level;

	return ret;
}
EXPORT_SYMBOL(chr_get_debug_level);

static void oplus_wake_up_charger(struct mtk_charger *info)
{
	unsigned long flags;

	info->timer_cb_duration[2] = ktime_get_boottime();
	if (info == NULL)
		return;

	spin_lock_irqsave(&info->slock, flags);
	info->timer_cb_duration[3] = ktime_get_boottime();
	if (!info->charger_wakelock->active)
		__pm_stay_awake(info->charger_wakelock);
	info->timer_cb_duration[4] = ktime_get_boottime();
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	info->timer_cb_duration[5] = ktime_get_boottime();
	wake_up_interruptible(&info->wait_que);
}

/* MTK's _wake_up_charger() takes a struct charger_manager *; the v2 HAL
 * keeps its charger state in a struct mtk_charger, whose field offsets are
 * completely different. mtk_intf.c and mtk_switch_charging.c are built on
 * this tree and call _wake_up_charger() with a struct charger_manager *, so
 * the v2 function must not be the one they land on -- it would read
 * garbage out of the wrong offsets. The MTK-compatible version lives in
 * oplus_mtk_compat.c, which is the only TU able to include
 * mtk_charger_intf.h without clashing with the charger macros. */

bool is_disable_charger(struct mtk_charger *info)
{
	if (info == NULL)
		return true;

	if (info->disable_charger == true || IS_ENABLED(CONFIG_POWER_EXT))
		return true;
	else
		return false;
}

int _mtk_enable_charging(struct mtk_charger *info,
	bool en)
{
	chr_debug("%s en:%d\n", __func__, en);
	if (info->algo.enable_charging != NULL)
		return info->algo.enable_charging(info, en);
	return false;
}

int mtk_charger_notifier(struct mtk_charger *info, int event)
{
	return srcu_notifier_call_chain(&info->evt_nh, event, NULL);
}

static void mtk_charger_parse_dt(struct mtk_charger *info,
				struct device *dev)
{
	struct device_node *np = dev->of_node;
	u32 val = 0;
	struct device_node *boot_node = NULL;
	struct tag_bootmode *tag = NULL;

	boot_node = of_parse_phandle(dev->of_node, "bootmode", 0);
	if (!boot_node)
		chr_err("%s: failed to get boot mode phandle\n", __func__);
	else {
		tag = (struct tag_bootmode *)of_get_property(boot_node,
							"atag,boot", NULL);
		if (!tag)
			chr_err("%s: failed to get atag,boot\n", __func__);
		else {
			chr_err("%s: size:0x%x tag:0x%x bootmode:0x%x boottype:0x%x\n",
				__func__, tag->size, tag->tag,
				tag->bootmode, tag->boottype);
			info->bootmode = tag->bootmode;
			info->boottype = tag->boottype;
		}
	}

	if (of_property_read_string(np, "algorithm_name",
		&info->algorithm_name) < 0) {
		chr_err("%s: no algorithm_name name\n", __func__);
		info->algorithm_name = "Basic";
	}

#ifndef OPLUS_FEATURE_CHG_BASIC
	if (strcmp(info->algorithm_name, "Basic") == 0) {
		chr_err("found Basic\n");
		mtk_basic_charger_init(info);
	} else if (strcmp(info->algorithm_name, "Pulse") == 0) {
		chr_err("found Pulse\n");
		mtk_pulse_charger_init(info);
	}
#endif

	info->disable_charger = of_property_read_bool(np, "disable_charger");
	info->atm_enabled = of_property_read_bool(np, "atm_is_enabled");
	info->enable_sw_safety_timer =
			of_property_read_bool(np, "enable_sw_safety_timer");
	info->sw_safety_timer_setting = info->enable_sw_safety_timer;

	/* common */

	if (of_property_read_u32(np, "charger_configuration", &val) >= 0)
		info->config = val;
	else {
		chr_err("use default charger_configuration:%d\n",
			SINGLE_CHARGER);
		info->config = SINGLE_CHARGER;
	}

	if (of_property_read_u32(np, "battery_cv", &val) >= 0)
		info->data.battery_cv = val;
	else {
		chr_err("use default BATTERY_CV:%d\n", BATTERY_CV);
		info->data.battery_cv = BATTERY_CV;
	}

	if (of_property_read_u32(np, "max_charger_voltage", &val) >= 0)
		info->data.max_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MAX:%d\n", V_CHARGER_MAX);
		info->data.max_charger_voltage = V_CHARGER_MAX;
	}
	info->data.max_charger_voltage_setting = info->data.max_charger_voltage;

	if (of_property_read_u32(np, "min_charger_voltage", &val) >= 0)
		info->data.min_charger_voltage = val;
	else {
		chr_err("use default V_CHARGER_MIN:%d\n", V_CHARGER_MIN);
		info->data.min_charger_voltage = V_CHARGER_MIN;
	}
	info->enable_vbat_mon = of_property_read_bool(np, "enable_vbat_mon");
	chr_err("use 6pin bat, enable_vbat_mon:%d\n", info->enable_vbat_mon);

	/* sw jeita */
	info->enable_sw_jeita = of_property_read_bool(np, "enable_sw_jeita");
	if (of_property_read_u32(np, "jeita_temp_above_t4_cv", &val) >= 0)
		info->data.jeita_temp_above_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_ABOVE_T4_CV:%d\n",
			JEITA_TEMP_ABOVE_T4_CV);
		info->data.jeita_temp_above_t4_cv = JEITA_TEMP_ABOVE_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t3_to_t4_cv", &val) >= 0)
		info->data.jeita_temp_t3_to_t4_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T3_TO_T4_CV:%d\n",
			JEITA_TEMP_T3_TO_T4_CV);
		info->data.jeita_temp_t3_to_t4_cv = JEITA_TEMP_T3_TO_T4_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t2_to_t3_cv", &val) >= 0)
		info->data.jeita_temp_t2_to_t3_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T2_TO_T3_CV:%d\n",
			JEITA_TEMP_T2_TO_T3_CV);
		info->data.jeita_temp_t2_to_t3_cv = JEITA_TEMP_T2_TO_T3_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t1_to_t2_cv", &val) >= 0)
		info->data.jeita_temp_t1_to_t2_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T1_TO_T2_CV:%d\n",
			JEITA_TEMP_T1_TO_T2_CV);
		info->data.jeita_temp_t1_to_t2_cv = JEITA_TEMP_T1_TO_T2_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_t0_to_t1_cv", &val) >= 0)
		info->data.jeita_temp_t0_to_t1_cv = val;
	else {
		chr_err("use default JEITA_TEMP_T0_TO_T1_CV:%d\n",
			JEITA_TEMP_T0_TO_T1_CV);
		info->data.jeita_temp_t0_to_t1_cv = JEITA_TEMP_T0_TO_T1_CV;
	}

	if (of_property_read_u32(np, "jeita_temp_below_t0_cv", &val) >= 0)
		info->data.jeita_temp_below_t0_cv = val;
	else {
		chr_err("use default JEITA_TEMP_BELOW_T0_CV:%d\n",
			JEITA_TEMP_BELOW_T0_CV);
		info->data.jeita_temp_below_t0_cv = JEITA_TEMP_BELOW_T0_CV;
	}

	if (of_property_read_u32(np, "temp_t4_thres", &val) >= 0)
		info->data.temp_t4_thres = val;
	else {
		chr_err("use default TEMP_T4_THRES:%d\n",
			TEMP_T4_THRES);
		info->data.temp_t4_thres = TEMP_T4_THRES;
	}

	if (of_property_read_u32(np, "temp_t4_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t4_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T4_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T4_THRES_MINUS_X_DEGREE);
		info->data.temp_t4_thres_minus_x_degree =
					TEMP_T4_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t3_thres", &val) >= 0)
		info->data.temp_t3_thres = val;
	else {
		chr_err("use default TEMP_T3_THRES:%d\n",
			TEMP_T3_THRES);
		info->data.temp_t3_thres = TEMP_T3_THRES;
	}

	if (of_property_read_u32(np, "temp_t3_thres_minus_x_degree", &val) >= 0)
		info->data.temp_t3_thres_minus_x_degree = val;
	else {
		chr_err("use default TEMP_T3_THRES_MINUS_X_DEGREE:%d\n",
			TEMP_T3_THRES_MINUS_X_DEGREE);
		info->data.temp_t3_thres_minus_x_degree =
					TEMP_T3_THRES_MINUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t2_thres", &val) >= 0)
		info->data.temp_t2_thres = val;
	else {
		chr_err("use default TEMP_T2_THRES:%d\n",
			TEMP_T2_THRES);
		info->data.temp_t2_thres = TEMP_T2_THRES;
	}

	if (of_property_read_u32(np, "temp_t2_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t2_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T2_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T2_THRES_PLUS_X_DEGREE);
		info->data.temp_t2_thres_plus_x_degree =
					TEMP_T2_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t1_thres", &val) >= 0)
		info->data.temp_t1_thres = val;
	else {
		chr_err("use default TEMP_T1_THRES:%d\n",
			TEMP_T1_THRES);
		info->data.temp_t1_thres = TEMP_T1_THRES;
	}

	if (of_property_read_u32(np, "temp_t1_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t1_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T1_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T1_THRES_PLUS_X_DEGREE);
		info->data.temp_t1_thres_plus_x_degree =
					TEMP_T1_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_t0_thres", &val) >= 0)
		info->data.temp_t0_thres = val;
	else {
		chr_err("use default TEMP_T0_THRES:%d\n",
			TEMP_T0_THRES);
		info->data.temp_t0_thres = TEMP_T0_THRES;
	}

	if (of_property_read_u32(np, "temp_t0_thres_plus_x_degree", &val) >= 0)
		info->data.temp_t0_thres_plus_x_degree = val;
	else {
		chr_err("use default TEMP_T0_THRES_PLUS_X_DEGREE:%d\n",
			TEMP_T0_THRES_PLUS_X_DEGREE);
		info->data.temp_t0_thres_plus_x_degree =
					TEMP_T0_THRES_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "temp_neg_10_thres", &val) >= 0)
		info->data.temp_neg_10_thres = val;
	else {
		chr_err("use default TEMP_NEG_10_THRES:%d\n",
			TEMP_NEG_10_THRES);
		info->data.temp_neg_10_thres = TEMP_NEG_10_THRES;
	}

	/* battery temperature protection */
	info->thermal.sm = BAT_TEMP_NORMAL;
	info->thermal.enable_min_charge_temp =
		of_property_read_bool(np, "enable_min_charge_temp");

	if (of_property_read_u32(np, "min_charge_temp", &val) >= 0)
		info->thermal.min_charge_temp = val;
	else {
		chr_err("use default MIN_CHARGE_TEMP:%d\n",
			MIN_CHARGE_TEMP);
		info->thermal.min_charge_temp = MIN_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "min_charge_temp_plus_x_degree", &val)
		>= 0) {
		info->thermal.min_charge_temp_plus_x_degree = val;
	} else {
		chr_err("use default MIN_CHARGE_TEMP_PLUS_X_DEGREE:%d\n",
			MIN_CHARGE_TEMP_PLUS_X_DEGREE);
		info->thermal.min_charge_temp_plus_x_degree =
					MIN_CHARGE_TEMP_PLUS_X_DEGREE;
	}

	if (of_property_read_u32(np, "max_charge_temp", &val) >= 0)
		info->thermal.max_charge_temp = val;
	else {
		chr_err("use default MAX_CHARGE_TEMP:%d\n",
			MAX_CHARGE_TEMP);
		info->thermal.max_charge_temp = MAX_CHARGE_TEMP;
	}

	if (of_property_read_u32(np, "max_charge_temp_minus_x_degree", &val)
		>= 0) {
		info->thermal.max_charge_temp_minus_x_degree = val;
	} else {
		chr_err("use default MAX_CHARGE_TEMP_MINUS_X_DEGREE:%d\n",
			MAX_CHARGE_TEMP_MINUS_X_DEGREE);
		info->thermal.max_charge_temp_minus_x_degree =
					MAX_CHARGE_TEMP_MINUS_X_DEGREE;
	}

	/* charging current */
	if (of_property_read_u32(np, "usb_charger_current", &val) >= 0) {
		info->data.usb_charger_current = val;
	} else {
		chr_err("use default USB_CHARGER_CURRENT:%d\n",
			USB_CHARGER_CURRENT);
		info->data.usb_charger_current = USB_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_current", &val) >= 0) {
		info->data.ac_charger_current = val;
	} else {
		chr_err("use default AC_CHARGER_CURRENT:%d\n",
			AC_CHARGER_CURRENT);
		info->data.ac_charger_current = AC_CHARGER_CURRENT;
	}

	if (of_property_read_u32(np, "ac_charger_input_current", &val) >= 0)
		info->data.ac_charger_input_current = val;
	else {
		chr_err("use default AC_CHARGER_INPUT_CURRENT:%d\n",
			AC_CHARGER_INPUT_CURRENT);
		info->data.ac_charger_input_current = AC_CHARGER_INPUT_CURRENT;
	}

	if (of_property_read_u32(np, "charging_host_charger_current", &val)
		>= 0) {
		info->data.charging_host_charger_current = val;
	} else {
		chr_err("use default CHARGING_HOST_CHARGER_CURRENT:%d\n",
			CHARGING_HOST_CHARGER_CURRENT);
		info->data.charging_host_charger_current =
					CHARGING_HOST_CHARGER_CURRENT;
	}

	/* dynamic mivr */
	info->enable_dynamic_mivr =
			of_property_read_bool(np, "enable_dynamic_mivr");

	if (of_property_read_u32(np, "min_charger_voltage_1", &val) >= 0)
		info->data.min_charger_voltage_1 = val;
	else {
		chr_err("use default V_CHARGER_MIN_1: %d\n", V_CHARGER_MIN_1);
		info->data.min_charger_voltage_1 = V_CHARGER_MIN_1;
	}

	if (of_property_read_u32(np, "min_charger_voltage_2", &val) >= 0)
		info->data.min_charger_voltage_2 = val;
	else {
		chr_err("use default V_CHARGER_MIN_2: %d\n", V_CHARGER_MIN_2);
		info->data.min_charger_voltage_2 = V_CHARGER_MIN_2;
	}

	if (of_property_read_u32(np, "max_dmivr_charger_current", &val) >= 0)
		info->data.max_dmivr_charger_current = val;
	else {
		chr_err("use default MAX_DMIVR_CHARGER_CURRENT: %d\n",
			MAX_DMIVR_CHARGER_CURRENT);
		info->data.max_dmivr_charger_current =
					MAX_DMIVR_CHARGER_CURRENT;
	}
	/* fast charging algo support indicator */
	info->enable_fast_charging_indicator =
			of_property_read_bool(np, "enable_fast_charging_indicator");

	info->support_ntc_01c_precision = of_property_read_bool(np, "oplus,support_ntc_01c_precision");
	chr_debug("%s: support_ntc_01c_precision: %d\n", __func__, info->support_ntc_01c_precision);
}

static void mtk_charger_start_timer(struct mtk_charger *info)
{
	struct timespec64 end_time, time_now;
	ktime_t ktime, ktime_now;
	int ret = 0;

	/* If the timer was already set, cancel it */
	ret = alarm_try_to_cancel(&info->charger_timer);
	if (ret < 0) {
		chr_err("%s: callback was running, skip timer\n", __func__);
		return;
	}

	ktime_now = ktime_get_boottime();
	time_now = ktime_to_timespec64(ktime_now);
	end_time.tv_sec = time_now.tv_sec + info->polling_interval;
	end_time.tv_nsec = time_now.tv_nsec + 0;
	info->endtime = end_time;
	ktime = ktime_set(info->endtime.tv_sec, info->endtime.tv_nsec);

	chr_err("alarm timer start:%d, %ld %ld\n", ret,
		info->endtime.tv_sec, info->endtime.tv_nsec);
	alarm_start(&info->charger_timer, ktime);
}

static void check_battery_exist(struct mtk_charger *info)
{
	unsigned int i = 0;
	int count = 0;
	/*int boot_mode = get_boot_mode();*/

	if (is_disable_charger(info))
		return;

	for (i = 0; i < 3; i++) {
		if (is_battery_exist(info) == false)
			count++;
	}

#ifdef FIXME
	if (count >= 3) {
		if (boot_mode == META_BOOT || boot_mode == ADVMETA_BOOT ||
		    boot_mode == ATE_FACTORY_BOOT)
			chr_info("boot_mode = %d, bypass battery check\n",
				boot_mode);
		else {
			chr_err("battery doesn't exist, shutdown\n");
			orderly_poweroff(true);
		}
	}
#endif
}

static void check_dynamic_mivr(struct mtk_charger *info)
{
	int i = 0, ret = 0;
	int vbat = 0;
	bool is_fast_charge = false;
	struct chg_alg_device *alg = NULL;

	if (!info->enable_dynamic_mivr)
		return;

	/* NOTE: no chg_alg providers on this tree (4.19); info->alg stays
	 * NULL and PE algorithms are dormant, as in v1 builds where the
	 * stock manager never existed. */

	if (!is_fast_charge) {
		vbat = get_battery_voltage(info);
		if (vbat < info->data.min_charger_voltage_2 / 1000 - 200)
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage_2);
		else if (vbat < info->data.min_charger_voltage_1 / 1000 - 200)
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage_1);
		else
			charger_dev_set_mivr(info->chg1_dev,
				info->data.min_charger_voltage);
	}
}

/* sw jeita */
void do_sw_jeita_state_machine(struct mtk_charger *info)
{
	struct sw_jeita_data *sw_jeita;

	sw_jeita = &info->sw_jeita;
	sw_jeita->pre_sm = sw_jeita->sm;
	sw_jeita->charging = true;

	/* JEITA battery temp Standard */
	if (info->battery_temp >= info->data.temp_t4_thres) {
		chr_err("[SW_JEITA] Battery Over high Temperature(%d) !!\n",
			info->data.temp_t4_thres);

		sw_jeita->sm = TEMP_ABOVE_T4;
		sw_jeita->charging = false;
	} else if (info->battery_temp > info->data.temp_t3_thres) {
		/* control 45 degree to normal behavior */
		if ((sw_jeita->sm == TEMP_ABOVE_T4)
		    && (info->battery_temp
			>= info->data.temp_t4_thres_minus_x_degree)) {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
				info->data.temp_t4_thres_minus_x_degree,
				info->data.temp_t4_thres);

			sw_jeita->charging = false;
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t3_thres,
				info->data.temp_t4_thres);

			sw_jeita->sm = TEMP_T3_TO_T4;
		}
	} else if (info->battery_temp >= info->data.temp_t2_thres) {
		if (((sw_jeita->sm == TEMP_T3_TO_T4)
		     && (info->battery_temp
			 >= info->data.temp_t3_thres_minus_x_degree))
		    || ((sw_jeita->sm == TEMP_T1_TO_T2)
			&& (info->battery_temp
			    <= info->data.temp_t2_thres_plus_x_degree))) {
			chr_err("[SW_JEITA] Battery Temperature not recovery to normal temperature charging mode yet!!\n");
		} else {
			chr_err("[SW_JEITA] Battery Normal Temperature between %d and %d !!\n",
				info->data.temp_t2_thres,
				info->data.temp_t3_thres);
			sw_jeita->sm = TEMP_T2_TO_T3;
		}
	} else if (info->battery_temp >= info->data.temp_t1_thres) {
		if ((sw_jeita->sm == TEMP_T0_TO_T1
		     || sw_jeita->sm == TEMP_BELOW_T0)
		    && (info->battery_temp
			<= info->data.temp_t1_thres_plus_x_degree)) {
			if (sw_jeita->sm == TEMP_T0_TO_T1) {
				chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
					info->data.temp_t1_thres_plus_x_degree,
					info->data.temp_t2_thres);
			}
			if (sw_jeita->sm == TEMP_BELOW_T0) {
				chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
					info->data.temp_t1_thres,
					info->data.temp_t1_thres_plus_x_degree);
				sw_jeita->charging = false;
			}
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t1_thres,
				info->data.temp_t2_thres);

			sw_jeita->sm = TEMP_T1_TO_T2;
		}
	} else if (info->battery_temp >= info->data.temp_t0_thres) {
		if ((sw_jeita->sm == TEMP_BELOW_T0)
		    && (info->battery_temp
			<= info->data.temp_t0_thres_plus_x_degree)) {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d,not allow charging yet!!\n",
				info->data.temp_t0_thres,
				info->data.temp_t0_thres_plus_x_degree);

			sw_jeita->charging = false;
		} else {
			chr_err("[SW_JEITA] Battery Temperature between %d and %d !!\n",
				info->data.temp_t0_thres,
				info->data.temp_t1_thres);

			sw_jeita->sm = TEMP_T0_TO_T1;
		}
	} else {
		chr_err("[SW_JEITA] Battery below low Temperature(%d) !!\n",
			info->data.temp_t0_thres);
		sw_jeita->sm = TEMP_BELOW_T0;
		sw_jeita->charging = false;
	}

	/* set CV after temperature changed */
	/* In normal range, we adjust CV dynamically */
	if (sw_jeita->sm != TEMP_T2_TO_T3) {
		if (sw_jeita->sm == TEMP_ABOVE_T4)
			sw_jeita->cv = info->data.jeita_temp_above_t4_cv;
		else if (sw_jeita->sm == TEMP_T3_TO_T4)
			sw_jeita->cv = info->data.jeita_temp_t3_to_t4_cv;
		else if (sw_jeita->sm == TEMP_T2_TO_T3)
			sw_jeita->cv = 0;
		else if (sw_jeita->sm == TEMP_T1_TO_T2)
			sw_jeita->cv = info->data.jeita_temp_t1_to_t2_cv;
		else if (sw_jeita->sm == TEMP_T0_TO_T1)
			sw_jeita->cv = info->data.jeita_temp_t0_to_t1_cv;
		else if (sw_jeita->sm == TEMP_BELOW_T0)
			sw_jeita->cv = info->data.jeita_temp_below_t0_cv;
		else
			sw_jeita->cv = info->data.battery_cv;
	} else {
		sw_jeita->cv = 0;
	}

	chr_err("[SW_JEITA]preState:%d newState:%d tmp:%d cv:%d\n",
		sw_jeita->pre_sm, sw_jeita->sm, info->battery_temp,
		sw_jeita->cv);
}

static int mtk_chgstat_notify(struct mtk_charger *info)
{
	int ret = 0;
	char *env[2] = { "CHGSTAT=1", NULL };

	chr_err("%s: 0x%x\n", __func__, info->notify_code);
	ret = kobject_uevent_env(&info->pdev->dev.kobj, KOBJ_CHANGE, env);
	if (ret)
		chr_err("%s: kobject_uevent_fail, ret=%d", __func__, ret);

	return ret;
}

static void mtk_charger_set_algo_log_level(struct mtk_charger *info, int level)
{
	/* 4.19 chg_alg API has no ALG_LOG_LEVEL; per-algo log levels
	 * unsupported here. Kept as a stub for the two call sites. */
	(void)info;
	(void)level;
}

static ssize_t sw_jeita_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->enable_sw_jeita);
	return sprintf(buf, "%d\n", pinfo->enable_sw_jeita);
}

static ssize_t sw_jeita_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp == 0)
			pinfo->enable_sw_jeita = false;
		else
			pinfo->enable_sw_jeita = true;

	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(sw_jeita);
/* sw jeita end*/

static ssize_t chr_type_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->chr_type);
	return sprintf(buf, "%d\n", pinfo->chr_type);
}

static ssize_t chr_type_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0)
		pinfo->chr_type = temp;
	else
		chr_err("%s: format error!\n", __func__);

	return size;
}

static DEVICE_ATTR_RW(chr_type);

static ssize_t Pump_Express_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	int ret = 0, i = 0;
	bool is_ta_detected = false;
	struct mtk_charger *pinfo = dev->driver_data;
	struct chg_alg_device *alg = NULL;

	if (!pinfo) {
		chr_err("%s: pinfo is null\n", __func__);
		return sprintf(buf, "%d\n", is_ta_detected);
	}

	/* NOTE: no chg_alg providers; TA detect stays false. */
	chr_err("%s: idx = %d, detect = %d\n", __func__, i, is_ta_detected);
	return sprintf(buf, "%d\n", is_ta_detected);
}

static DEVICE_ATTR_RO(Pump_Express);

static ssize_t fast_chg_indicator_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->fast_charging_indicator);
	return sprintf(buf, "%d\n", pinfo->fast_charging_indicator);
}

static ssize_t fast_chg_indicator_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->fast_charging_indicator = temp;
	else
		chr_err("%s: format error!\n", __func__);

	if (pinfo->fast_charging_indicator > 0) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	oplus_wake_up_charger(pinfo);
	return size;
}

static DEVICE_ATTR_RW(fast_chg_indicator);

static ssize_t enable_meta_current_limit_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->enable_meta_current_limit);
	return sprintf(buf, "%d\n", pinfo->enable_meta_current_limit);
}

static ssize_t enable_meta_current_limit_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0)
		pinfo->enable_meta_current_limit = temp;
	else
		chr_err("%s: format error!\n", __func__);

	if (pinfo->enable_meta_current_limit > 0) {
		pinfo->log_level = CHRLOG_DEBUG_LEVEL;
		mtk_charger_set_algo_log_level(pinfo, pinfo->log_level);
	}

	oplus_wake_up_charger(pinfo);
	return size;
}

static DEVICE_ATTR_RW(enable_meta_current_limit);

static ssize_t vbat_mon_show(struct device *dev, struct device_attribute *attr,
					       char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_debug("%s: %d\n", __func__, pinfo->enable_vbat_mon);
	return sprintf(buf, "%d\n", pinfo->enable_vbat_mon);
}

static ssize_t vbat_mon_store(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int temp;

	if (kstrtouint(buf, 10, &temp) == 0) {
		if (temp == 0)
			pinfo->enable_vbat_mon = false;
		else
			pinfo->enable_vbat_mon = true;
	} else {
		chr_err("%s: format error!\n", __func__);
	}

	oplus_wake_up_charger(pinfo);
	return size;
}

static DEVICE_ATTR_RW(vbat_mon);

static ssize_t ADC_Charger_Voltage_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int vbus = get_vbus(pinfo); /* mV */

	chr_err("%s: %d\n", __func__, vbus);
	return sprintf(buf, "%d\n", vbus);
}

static DEVICE_ATTR_RO(ADC_Charger_Voltage);

static ssize_t ADC_Charging_Current_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int ibat = get_battery_current(pinfo); /* mA */

	chr_err("%s: %d\n", __func__, ibat);
	return sprintf(buf, "%d\n", ibat);
}

static DEVICE_ATTR_RO(ADC_Charging_Current);

static ssize_t input_current_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;
	int aicr = 0;

	aicr = pinfo->chg_data[CHG1_SETTING].thermal_input_current_limit;
	chr_err("%s: %d\n", __func__, aicr);
	return sprintf(buf, "%d\n", aicr);
}

static ssize_t input_current_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	struct charger_data *chg_data;
	signed int temp;

	chg_data = &pinfo->chg_data[CHG1_SETTING];
	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0)
			chg_data->thermal_input_current_limit = 0;
		else
			chg_data->thermal_input_current_limit = temp;
	} else {
		chr_err("%s: format error!\n", __func__);
	}
	return size;
}

static DEVICE_ATTR_RW(input_current);

static ssize_t charger_log_level_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_err("%s: %d\n", __func__, pinfo->log_level);
	return sprintf(buf, "%d\n", pinfo->log_level);
}

static ssize_t charger_log_level_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	signed int temp;

	if (kstrtoint(buf, 10, &temp) == 0) {
		if (temp < 0) {
			chr_err("val is invalid: %d\n", temp);
			temp = 0;
		}
		pinfo->log_level = temp;
		chr_err("log_level=%d\n", pinfo->log_level);

	} else {
		chr_err("format error!\n");
	}
	return size;
}

static DEVICE_ATTR_RW(charger_log_level);

static ssize_t BatteryNotify_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct mtk_charger *pinfo = dev->driver_data;

	chr_info("%s: 0x%x\n", __func__, pinfo->notify_code);

	return sprintf(buf, "%u\n", pinfo->notify_code);
}

static ssize_t BatteryNotify_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct mtk_charger *pinfo = dev->driver_data;
	unsigned int reg = 0;
	int ret = 0;

	if (buf != NULL && size != 0) {
		ret = kstrtouint(buf, 16, &reg);
		if (ret < 0) {
			chr_err("%s: failed, ret = %d\n", __func__, ret);
			return ret;
		}
		pinfo->notify_code = reg;
		chr_info("%s: store code=0x%x\n", __func__, pinfo->notify_code);
		mtk_chgstat_notify(pinfo);
	}
	return size;
}

static DEVICE_ATTR_RW(BatteryNotify);

/* procfs */
static int mtk_chg_set_cv_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;

	seq_printf(m, "%d\n", pinfo->data.battery_cv);
	return 0;
}

static int mtk_chg_set_cv_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_set_cv_show, PDE_DATA(node));
}

static ssize_t mtk_chg_set_cv_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int cv = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));
	struct power_supply *psy = NULL;
	union  power_supply_propval dynamic_cv;

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &cv);
	if (ret == 0) {
		if (cv >= BATTERY_CV) {
			info->data.battery_cv = BATTERY_CV;
			chr_info("%s: adjust charge voltage %dV too high, use default cv\n",
				  __func__, cv);
		} else {
			info->data.battery_cv = cv;
			chr_info("%s: adjust charge voltage = %dV\n", __func__, cv);
		}
		psy = power_supply_get_by_name("battery");
		if (!IS_ERR_OR_NULL(psy)) {
			dynamic_cv.intval = info->data.battery_cv;
			ret = power_supply_set_property(psy,
				POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE, &dynamic_cv);
			if (ret < 0)
				chr_err("set gauge cv fail\n");
		}
		return count;
	}

	chr_err("%s: bad argument\n", __func__);
	return count;
}

static const struct file_operations mtk_chg_set_cv_fops = {
	.open = mtk_chg_set_cv_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_set_cv_write,
};

static int mtk_chg_current_cmd_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;

	seq_printf(m, "%d %d\n", pinfo->usb_unlimited, pinfo->cmd_discharging);
	return 0;
}

static int mtk_chg_current_cmd_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_current_cmd_show, PDE_DATA(node));
}

static ssize_t mtk_chg_current_cmd_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0;
	char desc[32] = {0};
	int current_unlimited = 0;
	int cmd_discharging = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	if (sscanf(desc, "%d %d", &current_unlimited, &cmd_discharging) == 2) {
		info->usb_unlimited = current_unlimited;
		if (cmd_discharging == 1) {
			info->cmd_discharging = true;
			charger_dev_enable(info->chg1_dev, false);
			charger_dev_do_event(info->chg1_dev,
					EVENT_DISCHARGE, 0);
		} else if (cmd_discharging == 0) {
			info->cmd_discharging = false;
			charger_dev_enable(info->chg1_dev, true);
			charger_dev_do_event(info->chg1_dev,
					EVENT_RECHARGE, 0);
		}

		chr_info("%s: current_unlimited=%d, cmd_discharging=%d\n",
			__func__, current_unlimited, cmd_discharging);
		return count;
	}

	chr_err("bad argument, echo [usb_unlimited] [disable] > current_cmd\n");
	return count;
}

static const struct file_operations mtk_chg_current_cmd_fops = {
	.open = mtk_chg_current_cmd_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_current_cmd_write,
};

static int mtk_chg_en_power_path_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;
	bool power_path_en = true;

	charger_dev_is_powerpath_enabled(pinfo->chg1_dev, &power_path_en);
	seq_printf(m, "%d\n", power_path_en);

	return 0;
}

static int mtk_chg_en_power_path_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_en_power_path_show, PDE_DATA(node));
}

static ssize_t mtk_chg_en_power_path_write(struct file *file,
		const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &enable);
	if (ret == 0) {
		charger_dev_enable_powerpath(info->chg1_dev, enable);
		chr_info("%s: enable power path = %d\n", __func__, enable);
		return count;
	}

	chr_err("bad argument, echo [enable] > en_power_path\n");
	return count;
}

static const struct file_operations mtk_chg_en_power_path_fops = {
	.open = mtk_chg_en_power_path_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_en_power_path_write,
};

static int mtk_chg_en_safety_timer_show(struct seq_file *m, void *data)
{
	struct mtk_charger *pinfo = m->private;
	bool safety_timer_en = false;

	charger_dev_is_safety_timer_enabled(pinfo->chg1_dev, &safety_timer_en);
	seq_printf(m, "%d\n", safety_timer_en);

	return 0;
}

static int mtk_chg_en_safety_timer_open(struct inode *node, struct file *file)
{
	return single_open(file, mtk_chg_en_safety_timer_show, PDE_DATA(node));
}

static ssize_t mtk_chg_en_safety_timer_write(struct file *file,
	const char *buffer, size_t count, loff_t *data)
{
	int len = 0, ret = 0;
	char desc[32] = {0};
	unsigned int enable = 0;
	struct mtk_charger *info = PDE_DATA(file_inode(file));

	if (!info)
		return -EINVAL;
	if (count <= 0)
		return -EINVAL;

	len = (count < (sizeof(desc) - 1)) ? count : (sizeof(desc) - 1);
	if (copy_from_user(desc, buffer, len))
		return -EFAULT;

	desc[len] = '\0';

	ret = kstrtou32(desc, 10, &enable);
	if (ret == 0) {
		charger_dev_enable_safety_timer(info->chg1_dev, enable);
		info->safety_timer_cmd = (int)enable;
		chr_info("%s: enable safety timer = %d\n", __func__, enable);

		/* SW safety timer */
		if (info->sw_safety_timer_setting == true) {
			if (enable)
				info->enable_sw_safety_timer = true;
			else
				info->enable_sw_safety_timer = false;
		}

		return count;
	}

	chr_err("bad argument, echo [enable] > en_safety_timer\n");
	return count;
}

static const struct file_operations mtk_chg_en_safety_timer_fops = {
	.open = mtk_chg_en_safety_timer_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
	.write = mtk_chg_en_safety_timer_write,
};

void scd_ctrl_cmd_from_user(void *nl_data, struct sc_nl_msg_t *ret_msg)
{
	struct sc_nl_msg_t *msg;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return;

	msg = nl_data;
	ret_msg->sc_cmd = msg->sc_cmd;

	switch (msg->sc_cmd) {
	case SC_DAEMON_CMD_PRINT_LOG: {
		chr_err("[%s] %s", SC_TAG, &msg->sc_data[0]);
		}
	break;
	case SC_DAEMON_CMD_SET_DAEMON_PID: {
		memcpy(&info->sc.g_scd_pid, &msg->sc_data[0],
			sizeof(info->sc.g_scd_pid));
		chr_err("[%s][fr] SC_DAEMON_CMD_SET_DAEMON_PID = %d(first launch)\n",
			SC_TAG, info->sc.g_scd_pid);
		}
	break;
	case SC_DAEMON_CMD_SETTING: {
		struct scd_cmd_param_t_1 data;
			memcpy(&data, &msg->sc_data[0],
				sizeof(struct scd_cmd_param_t_1));

		chr_debug("[%s] rcv data:%d %d %d %d %d %d %d %d %d %d %d %d %d %d Ans:%d\n",
			SC_TAG,
			data.data[0],
			data.data[1],
			data.data[2],
			data.data[3],
			data.data[4],
			data.data[5],
			data.data[6],
			data.data[7],
			data.data[8],
			data.data[9],
			data.data[10],
			data.data[11],
			data.data[12],
			data.data[13],
			data.data[14]);

		info->sc.solution = data.data[SC_SOLUTION];
		if (data.data[SC_SOLUTION] == SC_DISABLE)
			info->sc.disable_charger = true;
		else if (data.data[SC_SOLUTION] == SC_REDUCE)
			info->sc.disable_charger = false;
		else
			info->sc.disable_charger = false;
	}
	break;
	default:
		chr_err("[%s] bad sc_DAEMON_CTRL_CMD_FROM_USER 0x%x\n", SC_TAG, msg->sc_cmd);
		break;
	}
}

static void sc_nl_send_to_user(u32 pid, int seq, struct sc_nl_msg_t *reply_msg)
{
	struct sk_buff *skb;
	struct nlmsghdr *nlh;
	/* int size=sizeof(struct fgd_nl_msg_t); */
	int size = reply_msg->sc_data_len + SCD_NL_MSG_T_HDR_LEN;
	int len = NLMSG_SPACE(size);
	void *data;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return;

	reply_msg->identity = SCD_NL_MAGIC;

	if (in_interrupt())
		skb = alloc_skb(len, GFP_ATOMIC);
	else
		skb = alloc_skb(len, GFP_KERNEL);

	if (!skb)
		return;

	nlh = nlmsg_put(skb, pid, seq, 0, size, 0);
	data = NLMSG_DATA(nlh);
	memcpy(data, reply_msg, size);
	NETLINK_CB(skb).portid = 0;	/* from kernel */
	NETLINK_CB(skb).dst_group = 0;	/* unicast */

	ret = netlink_unicast(info->sc.daemo_nl_sk, skb, pid, MSG_DONTWAIT);
	if (ret < 0) {
		chr_err("[Netlink] sc send failed %d\n", ret);
		return;
	}
}

static void chg_nl_data_handler(struct sk_buff *skb)
{
	u32 pid;
	kuid_t uid;
	int seq;
	void *data;
	struct nlmsghdr *nlh;
	struct sc_nl_msg_t *sc_msg, *sc_ret_msg;
	int size = 0;

	nlh = (struct nlmsghdr *)skb->data;
	pid = NETLINK_CREDS(skb)->pid;
	uid = NETLINK_CREDS(skb)->uid;
	seq = nlh->nlmsg_seq;

	data = NLMSG_DATA(nlh);

	sc_msg = (struct sc_nl_msg_t *)data;

	size = sc_msg->sc_ret_data_len + SCD_NL_MSG_T_HDR_LEN;

	if (size > (PAGE_SIZE << 1))
		sc_ret_msg = vmalloc(size);
	else {
		if (in_interrupt())
			sc_ret_msg = kmalloc(size, GFP_ATOMIC);
	else
		sc_ret_msg = kmalloc(size, GFP_KERNEL);
	}

	if (sc_ret_msg == NULL) {
		if (size > PAGE_SIZE)
			sc_ret_msg = vmalloc(size);

		if (sc_ret_msg == NULL)
			return;
	}

	memset(sc_ret_msg, 0, size);

	scd_ctrl_cmd_from_user(data, sc_ret_msg);
	sc_nl_send_to_user(pid, seq, sc_ret_msg);

	kvfree(sc_ret_msg);
}

void sc_init(struct smartcharging *sc)
{
	sc->enable = false;
	sc->battery_size = 3000;
	sc->start_time = 0;
	sc->end_time = 80000;
	sc->current_limit = 2000;
	sc->target_percentage = 80;
	sc->left_time_for_cv = 3600;
	sc->pre_ibat = -1;
}

static void mtk6893_sc_update(struct mtk_charger *info)
{
	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	info->sc.data.data[SC_VBAT] = get_battery_voltage(info);
	info->sc.data.data[SC_BAT_TMP] = get_battery_temperature(info);
	info->sc.data.data[SC_UISOC] = get_uisoc(info);
	/* info->sc.data.data[SC_SOC] = battery_get_soc(); */

	info->sc.data.data[SC_ENABLE] = info->sc.enable;
	info->sc.data.data[SC_BAT_SIZE] = info->sc.battery_size;
	info->sc.data.data[SC_START_TIME] = info->sc.start_time;
	info->sc.data.data[SC_END_TIME] = info->sc.end_time;
	info->sc.data.data[SC_IBAT_LIMIT] = info->sc.current_limit;
	info->sc.data.data[SC_TARGET_PERCENTAGE] = info->sc.target_percentage;
	info->sc.data.data[SC_LEFT_TIME_FOR_CV] = info->sc.left_time_for_cv;

	charger_dev_get_charging_current(info->chg1_dev, &info->sc.data.data[SC_IBAT_SETTING]);
	info->sc.data.data[SC_IBAT_SETTING] = info->sc.data.data[SC_IBAT_SETTING] / 1000;
	info->sc.data.data[SC_IBAT] = get_battery_current(info);
}

int wakeup_sc_algo_cmd(struct scd_cmd_param_t_1 *data, int subcmd, int para1)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (info->sc.g_scd_pid != 0) {
		struct sc_nl_msg_t *sc_msg;
		int size = SCD_NL_MSG_T_HDR_LEN + sizeof(struct scd_cmd_param_t_1);

		if (size > (PAGE_SIZE << 1))
			sc_msg = vmalloc(size);
		else {
			if (in_interrupt())
				sc_msg = kmalloc(size, GFP_ATOMIC);
			else
				sc_msg = kmalloc(size, GFP_KERNEL);
		}

		if (sc_msg == NULL) {
			if (size > PAGE_SIZE)
				sc_msg = vmalloc(size);

			if (sc_msg == NULL)
				return -1;
		}
		chr_debug(
			"[wakeup_fg_algo] malloc size=%d pid=%d\n",
			size, info->sc.g_scd_pid);
		memset(sc_msg, 0, size);
		sc_msg->sc_cmd = SC_DAEMON_CMD_NOTIFY_DAEMON;
		sc_msg->sc_subcmd = subcmd;
		sc_msg->sc_subcmd_para1 = para1;
		memcpy(sc_msg->sc_data, data, sizeof(struct scd_cmd_param_t_1));
		sc_msg->sc_data_len += sizeof(struct scd_cmd_param_t_1);
		sc_nl_send_to_user(info->sc.g_scd_pid, 0, sc_msg);

		kvfree(sc_msg);

		return 0;
	}
	chr_debug("pid is NULL\n");
	return -1;
}

static ssize_t enable_sc_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[enable smartcharging] : %d\n",
	info->sc.enable);

	return sprintf(buf, "%d\n", info->sc.enable);
}

static ssize_t enable_sc_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[enable smartcharging] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			chr_err(
				"[enable smartcharging] val is %d ??\n",
				(int)val);
			val = 0;
		}

		if (val == 0)
			info->sc.enable = false;
		else
			info->sc.enable = true;

		chr_err(
			"[enable smartcharging]enable smartcharging=%d\n",
			info->sc.enable);
	}
	return size;
}
static DEVICE_ATTR_RW(enable_sc);

static ssize_t sc_stime_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging stime] : %d\n",
	info->sc.start_time);

	return sprintf(buf, "%d\n", info->sc.start_time);
}

static ssize_t sc_stime_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging stime] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			chr_err(
				"[smartcharging stime] val is %d ??\n",
				(int)val);
			val = 0;
		}

		if (val >= 0)
			info->sc.start_time = val;

		chr_err(
			"[smartcharging stime]enable smartcharging=%d\n",
			info->sc.start_time);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_stime);

static ssize_t sc_etime_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging etime] : %d\n",
	info->sc.end_time);

	return sprintf(buf, "%d\n", info->sc.end_time);
}

static ssize_t sc_etime_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging etime] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			chr_err(
				"[smartcharging etime] val is %d ??\n",
				(int)val);
			val = 0;
		}

		if (val >= 0)
			info->sc.end_time = val;

		chr_err(
			"[smartcharging stime]enable smartcharging=%d\n",
			info->sc.end_time);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_etime);

static ssize_t sc_tuisoc_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging target uisoc] : %d\n",
	info->sc.target_percentage);

	return sprintf(buf, "%d\n", info->sc.target_percentage);
}

static ssize_t sc_tuisoc_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging tuisoc] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			chr_err(
				"[smartcharging tuisoc] val is %d ??\n",
				(int)val);
			val = 0;
		}

		if (val >= 0)
			info->sc.target_percentage = val;

		chr_err(
			"[smartcharging stime]tuisoc=%d\n",
			info->sc.target_percentage);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_tuisoc);

static ssize_t sc_ibat_limit_show(
	struct device *dev, struct device_attribute *attr,
					char *buf)
{
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	chr_err(
	"[smartcharging ibat limit] : %d\n",
	info->sc.current_limit);

	return sprintf(buf, "%d\n", info->sc.current_limit);
}

static ssize_t sc_ibat_limit_store(
	struct device *dev, struct device_attribute *attr,
					 const char *buf, size_t size)
{
	unsigned long val = 0;
	int ret;
	struct power_supply *chg_psy = NULL;
	struct mtk_charger *info = NULL;

	chg_psy = power_supply_get_by_name("mtk-master-charger");
	if (chg_psy == NULL || IS_ERR(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
		return -EINVAL;
	}
	info = (struct mtk_charger *)power_supply_get_drvdata(chg_psy);
	if (info == NULL)
		return -EINVAL;

	if (buf != NULL && size != 0) {
		chr_err("[smartcharging ibat limit] buf is %s\n", buf);
		ret = kstrtoul(buf, 10, &val);
		if (val < 0) {
			chr_err(
				"[smartcharging ibat limit] val is %d ??\n",
				(int)val);
			val = 0;
		}

		if (val >= 0)
			info->sc.current_limit = val;

		chr_err(
			"[smartcharging ibat limit]=%d\n",
			info->sc.current_limit);
	}
	return size;
}
static DEVICE_ATTR_RW(sc_ibat_limit);

int mtk_chg_enable_vbus_ovp(bool enable)
{
	static struct mtk_charger *pinfo;
	int ret = 0;
	u32 sw_ovp = 0;
	struct power_supply *psy;

	if (pinfo == NULL) {
		psy = power_supply_get_by_name("mtk-master-charger");
		if (psy == NULL) {
			chr_err("[%s]psy is not rdy\n", __func__);
			return -1;
		}

		pinfo = (struct mtk_charger *)power_supply_get_drvdata(psy);
		if (pinfo == NULL) {
			chr_err("[%s]mtk_gauge is not rdy\n", __func__);
			return -1;
		}
	}

	if (enable)
		sw_ovp = pinfo->data.max_charger_voltage_setting;
	else
		sw_ovp = 15000000;

	/* Enable/Disable SW OVP status */
	pinfo->data.max_charger_voltage = sw_ovp;

	disable_hw_ovp(pinfo, enable);

	chr_err("[%s] en:%d ovp:%d\n",
			    __func__, enable, sw_ovp);
	return ret;
}
EXPORT_SYMBOL(mtk_chg_enable_vbus_ovp);

/* return false if vbus is over max_charger_voltage */
static bool mtk_chg_check_vbus(struct mtk_charger *info)
{
	int vchr = 0;

	vchr = get_vbus(info) * 1000; /* uV */
	if (vchr > info->data.max_charger_voltage) {
		chr_err("%s: vbus(%d mV) > %d mV\n", __func__, vchr / 1000,
			info->data.max_charger_voltage / 1000);
		return false;
	}
	return true;
}

static void mtk_battery_notify_VCharger_check(struct mtk_charger *info)
{
#if defined(BATTERY_NOTIFY_CASE_0001_VCHARGER)
	int vchr = 0;

	vchr = get_vbus(info) * 1000; /* uV */
	if (vchr < info->data.max_charger_voltage)
		info->notify_code &= ~CHG_VBUS_OV_STATUS;
	else {
		info->notify_code |= CHG_VBUS_OV_STATUS;
		chr_err("[BATTERY] charger_vol(%d mV) > %d mV\n",
			vchr / 1000, info->data.max_charger_voltage / 1000);
		mtk_chgstat_notify(info);
	}
#endif
}

static void mtk_battery_notify_VBatTemp_check(struct mtk_charger *info)
{
#if defined(BATTERY_NOTIFY_CASE_0002_VBATTEMP)
	if (info->battery_temp >= info->thermal.max_charge_temp) {
		info->notify_code |= CHG_BAT_OT_STATUS;
		chr_err("[BATTERY] bat_temp(%d) out of range(too high)\n",
			info->battery_temp);
		mtk_chgstat_notify(info);
	} else {
		info->notify_code &= ~CHG_BAT_OT_STATUS;
	}

	if (info->enable_sw_jeita == true) {
		if (info->battery_temp < info->data.temp_neg_10_thres) {
			info->notify_code |= CHG_BAT_LT_STATUS;
			chr_err("bat_temp(%d) out of range(too low)\n",
				info->battery_temp);
			mtk_chgstat_notify(info);
		} else {
			info->notify_code &= ~CHG_BAT_LT_STATUS;
		}
	} else {
#ifdef BAT_LOW_TEMP_PROTECT_ENABLE
		if (info->battery_temp < info->thermal.min_charge_temp) {
			info->notify_code |= CHG_BAT_LT_STATUS;
			chr_err("bat_temp(%d) out of range(too low)\n",
				info->battery_temp);
			mtk_chgstat_notify(info);
		} else {
			info->notify_code &= ~CHG_BAT_LT_STATUS;
		}
#endif
	}
#endif
}

static void mtk_battery_notify_UI_test(struct mtk_charger *info)
{
	switch (info->notify_test_mode) {
	case 1:
		info->notify_code = CHG_VBUS_OV_STATUS;
		chr_debug("[%s] CASE_0001_VCHARGER\n", __func__);
		break;
	case 2:
		info->notify_code = CHG_BAT_OT_STATUS;
		chr_debug("[%s] CASE_0002_VBATTEMP\n", __func__);
		break;
	case 3:
		info->notify_code = CHG_OC_STATUS;
		chr_debug("[%s] CASE_0003_ICHARGING\n", __func__);
		break;
	case 4:
		info->notify_code = CHG_BAT_OV_STATUS;
		chr_debug("[%s] CASE_0004_VBAT\n", __func__);
		break;
	case 5:
		info->notify_code = CHG_ST_TMO_STATUS;
		chr_debug("[%s] CASE_0005_TOTAL_CHARGINGTIME\n", __func__);
		break;
	case 6:
		info->notify_code = CHG_BAT_LT_STATUS;
		chr_debug("[%s] CASE6: VBATTEMP_LOW\n", __func__);
		break;
	case 7:
		info->notify_code = CHG_TYPEC_WD_STATUS;
		chr_debug("[%s] CASE7: Moisture Detection\n", __func__);
		break;
	default:
		chr_debug("[%s] Unknown BN_TestMode Code: %x\n",
			__func__, info->notify_test_mode);
	}
	mtk_chgstat_notify(info);
}

static void mtk_battery_notify_check(struct mtk_charger *info)
{
	if (info->notify_test_mode == 0x0000) {
		mtk_battery_notify_VCharger_check(info);
		mtk_battery_notify_VBatTemp_check(info);
	} else {
		mtk_battery_notify_UI_test(info);
	}
}

static void mtk_chg_get_tchg(struct mtk_charger *info)
{
	int ret;
	int tchg_min = -127, tchg_max = -127;
	struct charger_data *pdata;
	bool en = false;

	pdata = &info->chg_data[CHG1_SETTING];
	ret = charger_dev_get_temperature(info->chg1_dev, &tchg_min, &tchg_max);
	if (ret < 0) {
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
	} else {
		pdata->junction_temp_min = tchg_min;
		pdata->junction_temp_max = tchg_max;
	}

	if (info->chg2_dev) {
		pdata = &info->chg_data[CHG2_SETTING];
		ret = charger_dev_get_temperature(info->chg2_dev,
			&tchg_min, &tchg_max);

		if (ret < 0) {
			pdata->junction_temp_min = -127;
			pdata->junction_temp_max = -127;
		} else {
			pdata->junction_temp_min = tchg_min;
			pdata->junction_temp_max = tchg_max;
		}
	}

	if (info->dvchg1_dev) {
		pdata = &info->chg_data[DVCHG1_SETTING];
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
		ret = charger_dev_is_enabled(info->dvchg1_dev, &en);
		if (ret >= 0 && en) {
			ret = charger_dev_get_adc(info->dvchg1_dev,
						  ADC_CHANNEL_TEMP_JC,
						  &tchg_min, &tchg_max);
			if (ret >= 0) {
				pdata->junction_temp_min = tchg_min;
				pdata->junction_temp_max = tchg_max;
			}
		}
	}

	if (info->dvchg2_dev) {
		pdata = &info->chg_data[DVCHG2_SETTING];
		pdata->junction_temp_min = -127;
		pdata->junction_temp_max = -127;
		ret = charger_dev_is_enabled(info->dvchg2_dev, &en);
		if (ret >= 0 && en) {
			ret = charger_dev_get_adc(info->dvchg2_dev,
						  ADC_CHANNEL_TEMP_JC,
						  &tchg_min, &tchg_max);
			if (ret >= 0) {
				pdata->junction_temp_min = tchg_min;
				pdata->junction_temp_max = tchg_max;
			}
		}
	}
}

static void charger_check_status(struct mtk_charger *info)
{
	bool charging = true;
	bool chg_dev_chgen = true;
	int temperature;
	struct battery_thermal_protection_data *thermal;
	int uisoc = 0;

	if (get_charger_type(info) == POWER_SUPPLY_TYPE_UNKNOWN)
		return;

	temperature = info->battery_temp;
	thermal = &info->thermal;
	uisoc = get_uisoc(info);

	if (info->enable_sw_jeita == true || info->enable_vbat_mon != true ||
	    info->batpro_done == true)
		chr_debug("vbat_mon disabled by jeita/vbat_mon/batpro\n");

	if (info->enable_sw_jeita == true) {
		do_sw_jeita_state_machine(info);
		if (info->sw_jeita.charging == false) {
			charging = false;
			goto stop_charging;
		}
	} else {
		if (thermal->enable_min_charge_temp) {
			if (temperature < thermal->min_charge_temp) {
				chr_err("Battery Under Temperature or NTC fail %d %d\n",
					temperature, thermal->min_charge_temp);
				thermal->sm = BAT_TEMP_LOW;
				charging = false;
				goto stop_charging;
			} else if (thermal->sm == BAT_TEMP_LOW) {
				if (temperature >=
				    thermal->min_charge_temp_plus_x_degree) {
					chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
					thermal->min_charge_temp,
					temperature,
					thermal->min_charge_temp_plus_x_degree);
					thermal->sm = BAT_TEMP_NORMAL;
				} else {
					charging = false;
					goto stop_charging;
				}
			}
		}

		if (temperature >= thermal->max_charge_temp) {
			chr_err("Battery over Temperature or NTC fail %d %d\n",
				temperature, thermal->max_charge_temp);
			thermal->sm = BAT_TEMP_HIGH;
			charging = false;
			goto stop_charging;
		} else if (thermal->sm == BAT_TEMP_HIGH) {
			if (temperature
			    < thermal->max_charge_temp_minus_x_degree) {
				chr_err("Battery Temperature raise from %d to %d(%d), allow charging!!\n",
				thermal->max_charge_temp,
				temperature,
				thermal->max_charge_temp_minus_x_degree);
				thermal->sm = BAT_TEMP_NORMAL;
			} else {
				charging = false;
				goto stop_charging;
			}
		}
	}

	mtk_chg_get_tchg(info);

	if (!mtk_chg_check_vbus(info)) {
		charging = false;
		goto stop_charging;
	}

	if (info->cmd_discharging)
		charging = false;
	if (info->safety_timeout)
		charging = false;
	if (info->vbusov_stat)
		charging = false;
	if (info->sc.disable_charger == true)
		charging = false;
stop_charging:
	mtk_battery_notify_check(info);

	if (charging && uisoc < 80 && info->batpro_done == true) {
		info->batpro_done = false;
		info->stop_6pin_re_en = false;
	}

	chr_err("tmp:%d (jeita:%d sm:%d cv:%d en:%d) (sm:%d) en:%d c:%d s:%d ov:%d sc:%d %d %d saf_cmd:%d bat_mon:%d %d\n",
		temperature, info->enable_sw_jeita, info->sw_jeita.sm,
		info->sw_jeita.cv, info->sw_jeita.charging, thermal->sm,
		charging, info->cmd_discharging, info->safety_timeout,
		info->vbusov_stat, info->sc.disable_charger,
		info->can_charging, charging, info->safety_timer_cmd,
		info->enable_vbat_mon, info->batpro_done);

	charger_dev_is_enabled(info->chg1_dev, &chg_dev_chgen);

	if (charging != info->can_charging)
		_mtk_enable_charging(info, charging);
	else if (charging == false && chg_dev_chgen == true)
		_mtk_enable_charging(info, charging);

	info->can_charging = charging;
}

static bool charger_init_algo(struct mtk_charger *info)
{
	struct chg_alg_device *alg;
	int idx = 0;

	/* NOTE: PE/PD algorithm objects do not exist on this tree (4.19
	 * chg_alg framework); the v2 policy layers own charge control.
	 * info->alg[] stays NULL. */
	for (idx = 0; idx < MAX_ALG_NO; idx++)
		info->alg[idx] = NULL;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev)
		chr_err("Found primary charger\n");
	else {
		chr_err("*** Error : can't find primary charger ***\n");
		return false;
	}

	chr_err("config is %d\n", info->config);
	if (info->config == DUAL_CHARGERS_IN_SERIES) {
		info->chg2_dev = get_charger_by_name("secondary_chg");
		if (info->chg2_dev)
			chr_err("Found secondary charger\n");
		else {
			chr_err("*** Error : can't find secondary charger ***\n");
			return false;
		}
	} else if (info->config != SINGLE_CHARGER &&
		   info->config != DUAL_CHARGERS_IN_PARALLEL) {
		/* NOTE: 4.19 enum charger_configuration has no DIVIDER variants;
		 * divider pump path is owned by the Oplus mp2650 driver. */
		chr_err("unsupported charger config %d\n", info->config);
		return false;
	}

	chr_err("register chg1 notifier %d %d\n",
		info->chg1_dev != NULL, info->algo.do_event != NULL);
	if (info->chg1_dev != NULL && info->algo.do_event != NULL) {
		chr_err("register chg1 notifier done\n");
		info->chg1_nb.notifier_call = info->algo.do_event;
		register_charger_device_notifier(info->chg1_dev,
						&info->chg1_nb);
		charger_dev_set_drvdata(info->chg1_dev, info);
	}

	chr_err("register dvchg chg1 notifier %d %d\n",
		info->dvchg1_dev != NULL, info->algo.do_dvchg1_event != NULL);
	if (info->dvchg1_dev != NULL && info->algo.do_dvchg1_event != NULL) {
		chr_err("register dvchg chg1 notifier done\n");
		info->dvchg1_nb.notifier_call = info->algo.do_dvchg1_event;
		register_charger_device_notifier(info->dvchg1_dev,
						&info->dvchg1_nb);
		charger_dev_set_drvdata(info->dvchg1_dev, info);
	}

	chr_err("register dvchg chg2 notifier %d %d\n",
		info->dvchg2_dev != NULL, info->algo.do_dvchg2_event != NULL);
	if (info->dvchg2_dev != NULL && info->algo.do_dvchg2_event != NULL) {
		chr_err("register dvchg chg2 notifier done\n");
		info->dvchg2_nb.notifier_call = info->algo.do_dvchg2_event;
		register_charger_device_notifier(info->dvchg2_dev,
						 &info->dvchg2_nb);
		charger_dev_set_drvdata(info->dvchg2_dev, info);
	}

	return true;
}

static int mtk_charger_plug_out(struct mtk_charger *info)
{
	struct charger_data *pdata1 = &info->chg_data[CHG1_SETTING];
	struct charger_data *pdata2 = &info->chg_data[CHG2_SETTING];
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	int i;

	chr_err("%s\n", __func__);
	info->chr_type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->charger_thread_polling = false;
	info->pd_reset = false;

	pdata1->disable_charging_count = 0;
	pdata1->input_current_limit_by_aicl = -1;
	pdata2->disable_charging_count = 0;

	notify.evt = EVT_PLUG_OUT;
	notify.value = 0;
	/* NOTE: no algo subscribers (see above). */
	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_PLUG_OUT, 0);
	charger_dev_set_input_current(info->chg1_dev, 100000);
	charger_dev_set_mivr(info->chg1_dev, info->data.min_charger_voltage);
	charger_dev_plug_out(info->chg1_dev);

	return 0;
}

static int mtk_charger_plug_in(struct mtk_charger *info,
				int chr_type)
{
	struct chg_alg_device *alg;
	struct chg_alg_notify notify;
	int i, vbat;

	chr_debug("%s\n",
		__func__);

	info->chr_type = chr_type;
	info->usb_type = get_usb_type(info);
	info->charger_thread_polling = true;

	info->can_charging = true;
	/* info->enable_dynamic_cv = true; */
	info->safety_timeout = false;
	info->vbusov_stat = false;
	info->old_cv = 0;
	info->stop_6pin_re_en = false;
	info->batpro_done = false;

	chr_err("mtk_is_charger_on plug in, type:%d\n", chr_type);

	vbat = get_battery_voltage(info);

	notify.evt = EVT_PLUG_IN;
	notify.value = 0;
	/* NOTE: no algo subscribers (see above). */
	memset(&info->sc.data, 0, sizeof(struct scd_cmd_param_t_1));
	info->sc.disable_in_this_plug = false;
	wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_PLUG_IN, 0);
	charger_dev_set_input_current(info->chg1_dev,
				info->data.usb_charger_current);
	charger_dev_plug_in(info->chg1_dev);

	return 0;
}


static bool mtk_is_charger_on(struct mtk_charger *info)
{
	int chr_type;

	chr_type = get_charger_type(info);
	if (chr_type == POWER_SUPPLY_TYPE_UNKNOWN) {
		if (info->chr_type != POWER_SUPPLY_TYPE_UNKNOWN) {
			mtk_charger_plug_out(info);
			mutex_lock(&info->cable_out_lock);
			info->cable_out_cnt = 0;
			mutex_unlock(&info->cable_out_lock);
		}
	} else {
		if (info->chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
			mtk_charger_plug_in(info, chr_type);
		else {
			info->chr_type = chr_type;
			info->usb_type = get_usb_type(info);
		}

		if (info->cable_out_cnt > 0) {
			mtk_charger_plug_out(info);
			mtk_charger_plug_in(info, chr_type);
			mutex_lock(&info->cable_out_lock);
			info->cable_out_cnt = 0;
			mutex_unlock(&info->cable_out_lock);
		}
	}

	if (chr_type == POWER_SUPPLY_TYPE_UNKNOWN)
		return false;

	return true;
}

#ifndef OPLUS_FEATURE_CHG_BASIC
static void charger_send_kpoc_uevent(struct mtk_charger *info)
{
	static bool first_time = true;
	ktime_t ktime_now;

	if (first_time) {
		info->uevent_time_check = ktime_get();
		first_time = false;
	} else {
		ktime_now = ktime_get();
		if ((ktime_ms_delta(ktime_now, info->uevent_time_check) / 1000) >= 60) {
			mtk_chgstat_notify(info);
			info->uevent_time_check = ktime_now;
		}
	}
}

static void kpoc_power_off_check(struct mtk_charger *info)
{
	unsigned int boot_mode = info->bootmode;
	int vbus = 0;

	/* 8 = KERNEL_POWER_OFF_CHARGING_BOOT */
	/* 9 = LOW_POWER_OFF_CHARGING_BOOT */
	if (boot_mode == 8 || boot_mode == 9) {
		vbus = get_vbus(info);
		if (vbus >= 0 && vbus < 2500 && !mtk_is_charger_on(info) && !info->pd_reset) {
			chr_err("Unplug Charger/USB in KPOC mode, vbus=%d, shutdown\n", vbus);
			while (1) {
				if (info->is_suspend == false) {
					chr_err("%s, not in suspend, shutdown\n", __func__);
					kernel_power_off();
				} else {
					chr_err("%s, suspend! cannot shutdown\n", __func__);
					msleep(20);
				}
			}
		}
		charger_send_kpoc_uevent(info);
	}
}
#endif

static void charger_status_check(struct mtk_charger *info)
{
	union power_supply_propval online, status;
	struct power_supply *chg_psy = NULL;
	int ret;
	bool charging = true;

	chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
	if (IS_ERR_OR_NULL(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &online);

		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_STATUS, &status);

		if (!online.intval)
			charging = false;
		else {
			if (status.intval == POWER_SUPPLY_STATUS_NOT_CHARGING)
				charging = false;
		}
	}
	if (!charging) {
		/* CC/BC1.2 attach detection is unreliable on this board, but
		 * VBUS present means a supply is attached (the hardware
		 * default-charges without SW enable). Gate the UI state on
		 * VBUS so the bolt shows while the notifier path is dead. */
		if (get_vbus(info) > 4000)
			charging = true;
	}
	if (charging != info->is_charging) {
		info->is_charging = charging;
		power_supply_changed(info->psy1);
		/* Drive the MTK charger-manager event chain. mtk_battery.c
		 * registers battery_callback() here (once
		 * charger_manager_get_by_name() stops returning NULL) and
		 * that callback is what sets BAT_STATUS / publishes the
		 * battery uevent healthd and the UI read. */
		mtk_charger_notifier(info, charging ?
			CHARGER_NOTIFY_START_CHARGING :
			CHARGER_NOTIFY_STOP_CHARGING);
	}

	/* Mirror charging state into the MTK battery supply for the UI
	 * (healthd/BatteryService). The MTK charger-manager event path that
	 * normally does this is not built (weak stubs), so publish directly. */
	if (battery_main.psy && !IS_ERR(battery_main.psy)) {
		int new_status = charging ? POWER_SUPPLY_STATUS_CHARGING :
					    POWER_SUPPLY_STATUS_DISCHARGING;
		if (battery_main.BAT_STATUS != new_status) {
			battery_main.BAT_STATUS = new_status;
			power_supply_changed(battery_main.psy);
		}
	}
}


static char *dump_charger_type(int chg_type, int usb_type)
{
	switch (chg_type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		return "none";
	case POWER_SUPPLY_TYPE_USB:
		if (usb_type == POWER_SUPPLY_USB_TYPE_SDP)
			return "usb";
		else
			return "nonstd";
	case POWER_SUPPLY_TYPE_USB_CDP:
		return "usb-h";
	case POWER_SUPPLY_TYPE_USB_DCP:
		return "std";
	/* case POWER_SUPPLY_TYPE_USB_FLOAT:
		return "nonstd"; */
	default:
		return "unknown";
	}
}

void oplus_usbtemp_set_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, true);
}

void oplus_usbtemp_recover_cc_open(void)
{
	if (pinfo == NULL || pinfo->tcpc == NULL)
		return;

	tcpm_typec_disable_function(pinfo->tcpc, false);
}

static int charger_routine_thread(void *arg)
{
	struct mtk_charger *info = arg;
	unsigned long flags;
	static bool is_module_init_done;
	bool is_charger_on;
	int ret;
	int vbat_min, vbat_max;
	u32 chg_cv = 0;

	while (1) {
		ret = wait_event_interruptible(info->wait_que,
			(info->charger_thread_timeout == true));
		if (ret < 0) {
			chr_err("%s: wait event been interrupted(%d)\n", __func__, ret);
			continue;
		}

		while (is_module_init_done == false) {
			if (charger_init_algo(info) == true)
				is_module_init_done = true;
			else {
				chr_err("charger_init fail\n");
				msleep(5000);
			}
		}

		mutex_lock(&info->charger_lock);
		spin_lock_irqsave(&info->slock, flags);
		if (!info->charger_wakelock->active)
			__pm_stay_awake(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);
		info->charger_thread_timeout = false;

		info->battery_temp = get_battery_temperature(info);
		ret = charger_dev_get_adc(info->chg1_dev,
			ADC_CHANNEL_VBAT, &vbat_min, &vbat_max);
		ret = charger_dev_get_constant_voltage(info->chg1_dev, &chg_cv);

		if (vbat_min != 0)
			vbat_min = vbat_min / 1000;

		chr_err("Vbat=%d vbats=%d vbus:%d ibus:%d I=%d T=%d uisoc:%d type:%s>%s pd:%d swchg_ibat:%d cv:%d\n",
			get_battery_voltage(info),
			vbat_min,
			get_vbus(info),
			get_ibus(info),
			get_battery_current(info),
			info->battery_temp,
			get_uisoc(info),
			dump_charger_type(info->chr_type, info->usb_type),
			dump_charger_type(get_charger_type(info), get_usb_type(info)),
			info->pd_type, get_ibat(info), chg_cv);

		is_charger_on = mtk_is_charger_on(info);

		if (info->charger_thread_polling == true)
			mtk_charger_start_timer(info);

		check_battery_exist(info);
		check_dynamic_mivr(info);
		charger_check_status(info);
		/* Publish the charging state unconditionally. It used to sit
		 * inside the "charging enabled" branch below, which only runs
		 * once the type is detected -- so with detection dead the UI
		 * never learned that current was flowing. */
		charger_status_check(info);
#ifndef OPLUS_FEATURE_CHG_BASIC
		kpoc_power_off_check(info);
#endif

		if (is_disable_charger(info) == false &&
			is_charger_on == true &&
			info->can_charging == true) {
			if (info->algo.do_algorithm)
				info->algo.do_algorithm(info);
			mtk6893_sc_update(info);
			wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_CHARGING, 0);
		} else {
			chr_debug("disable charging %d %d %d\n",
			    is_disable_charger(info), is_charger_on, info->can_charging);
			mtk6893_sc_update(info);
			wakeup_sc_algo_cmd(&info->sc.data, SC_EVENT_STOP_CHARGING, 0);
		}

		spin_lock_irqsave(&info->slock, flags);
		__pm_relax(info->charger_wakelock);
		spin_unlock_irqrestore(&info->slock, flags);
		chr_debug("%s end , %d\n",
			__func__, info->charger_thread_timeout);
		mutex_unlock(&info->charger_lock);
	}

	return 0;
}


#ifdef CONFIG_PM
static int charger_pm_event(struct notifier_block *notifier,
			unsigned long pm_event, void *unused)
{
	ktime_t ktime_now;
	struct timespec64 now;
	struct mtk_charger *info;

	info = container_of(notifier,
		struct mtk_charger, pm_notifier);

	switch (pm_event) {
	case PM_SUSPEND_PREPARE:
		info->is_suspend = true;
		chr_debug("%s: enter PM_SUSPEND_PREPARE\n", __func__);
		break;
	case PM_POST_SUSPEND:
		info->is_suspend = false;
		chr_debug("%s: enter PM_POST_SUSPEND\n", __func__);
		ktime_now = ktime_get_boottime();
		now = ktime_to_timespec64(ktime_now);

		if (timespec64_compare(&now, &info->endtime) >= 0 &&
			info->endtime.tv_sec != 0 &&
			info->endtime.tv_nsec != 0) {
			chr_err("%s: alarm timeout, wake up charger\n",
				__func__);
			__pm_relax(info->charger_wakelock);
			info->endtime.tv_sec = 0;
			info->endtime.tv_nsec = 0;
			oplus_wake_up_charger(info);
		}
		break;
	default:
		break;
	}
	return NOTIFY_DONE;
}
#endif /* CONFIG_PM */

static enum alarmtimer_restart
	mtk_charger_alarm_timer_func(struct alarm *alarm, ktime_t now)
{
	struct mtk_charger *info =
	container_of(alarm, struct mtk_charger, charger_timer);
	ktime_t *time_p = info->timer_cb_duration;

	info->timer_cb_duration[0] = ktime_get_boottime();
	if (info->is_suspend == false) {
		chr_debug("%s: not suspend, wake up charger\n", __func__);
		info->timer_cb_duration[1] = ktime_get_boottime();
		oplus_wake_up_charger(info);
		info->timer_cb_duration[6] = ktime_get_boottime();
	} else {
		chr_debug("%s: alarm timer timeout\n", __func__);
		__pm_stay_awake(info->charger_wakelock);
	}

	info->timer_cb_duration[7] = ktime_get_boottime();

	if (ktime_us_delta(time_p[7], time_p[0]) > 5000)
		chr_err("delta_t: %lld %lld %lld %lld %lld %lld %lld (%lld)\n",
			ktime_us_delta(time_p[1], time_p[0]),
			ktime_us_delta(time_p[2], time_p[1]),
			ktime_us_delta(time_p[3], time_p[2]),
			ktime_us_delta(time_p[4], time_p[3]),
			ktime_us_delta(time_p[5], time_p[4]),
			ktime_us_delta(time_p[6], time_p[5]),
			ktime_us_delta(time_p[7], time_p[6]),
			ktime_us_delta(time_p[7], time_p[0]));

	return ALARMTIMER_NORESTART;
}

static void mtk_charger_init_timer(struct mtk_charger *info)
{
	alarm_init(&info->charger_timer, ALARM_BOOTTIME,
			mtk_charger_alarm_timer_func);
	mtk_charger_start_timer(info);

#ifdef CONFIG_PM
	if (register_pm_notifier(&info->pm_notifier))
		chr_err("%s: register pm failed\n", __func__);
#endif /* CONFIG_PM */
}

static int mtk_charger_setup_files(struct platform_device *pdev)
{
	int ret = 0;
	struct proc_dir_entry *battery_dir = NULL, *entry = NULL;
	struct mtk_charger *info = platform_get_drvdata(pdev);

	ret = device_create_file(&(pdev->dev), &dev_attr_sw_jeita);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_chr_type);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_enable_meta_current_limit);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_fast_chg_indicator);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_vbat_mon);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_Pump_Express);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_ADC_Charger_Voltage);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_ADC_Charging_Current);
	if (ret)
		goto _out;
	ret = device_create_file(&(pdev->dev), &dev_attr_input_current);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_charger_log_level);
	if (ret)
		goto _out;

	/* Battery warning */
	ret = device_create_file(&(pdev->dev), &dev_attr_BatteryNotify);
	if (ret)
		goto _out;

	/* sysfs node */
	ret = device_create_file(&(pdev->dev), &dev_attr_enable_sc);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_stime);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_etime);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_tuisoc);
	if (ret)
		goto _out;

	ret = device_create_file(&(pdev->dev), &dev_attr_sc_ibat_limit);
	if (ret)
		goto _out;

	battery_dir = proc_mkdir("mtk_battery_cmd", NULL);
	if (!battery_dir) {
		chr_err("%s: mkdir /proc/mtk_battery_cmd failed\n", __func__);
		return -ENOMEM;
	}

	entry = proc_create_data("current_cmd", 0644, battery_dir,
			&mtk_chg_current_cmd_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("en_power_path", 0644, battery_dir,
			&mtk_chg_en_power_path_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("en_safety_timer", 0644, battery_dir,
			&mtk_chg_en_safety_timer_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}
	entry = proc_create_data("set_cv", 0644, battery_dir,
			&mtk_chg_set_cv_fops, info);
	if (!entry) {
		ret = -ENODEV;
		goto fail_procfs;
	}

	return 0;

fail_procfs:
	remove_proc_subtree("mtk_battery_cmd", NULL);
_out:
	return ret;
}

void mtk_charger_get_atm_mode(struct mtk_charger *info)
{
	char atm_str[64] = {0};
	char *ptr = NULL, *ptr_e = NULL;
	char keyword[] = "androidboot.atm=";
	int size = 0;

	ptr = strstr(chg_get_cmd(), keyword);
	if (ptr != 0) {
		ptr_e = strstr(ptr, " ");
		if (ptr_e == 0)
			goto end;

		size = ptr_e - (ptr + strlen(keyword));
		if (size <= 0)
			goto end;
		strncpy(atm_str, ptr + strlen(keyword), size);
		atm_str[size] = '\0';
		chr_err("%s: atm_str: %s\n", __func__, atm_str);

		if (!strncmp(atm_str, "enable", strlen("enable")))
			info->atm_enabled = true;
	}
end:
	chr_err("%s: atm_enabled = %d\n", __func__, info->atm_enabled);
}

static int psy_charger_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		return 1;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property charger_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int psy_charger_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct mtk_charger *info;
	struct charger_device *chg;
	struct charger_data *pdata;
	int ret = 0;
	struct chg_alg_device *alg = NULL;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);

	chr_err("%s psp:%d\n",
		__func__, psp);


	if (info->psy1 != NULL &&
		info->psy1 == psy)
		chg = info->chg1_dev;
	else if (info->psy2 != NULL &&
		info->psy2 == psy)
		chg = info->chg2_dev;
	else if (info->psy_dvchg1 != NULL && info->psy_dvchg1 == psy)
		chg = info->dvchg1_dev;
	else if (info->psy_dvchg2 != NULL && info->psy_dvchg2 == psy)
		chg = info->dvchg2_dev;
	else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		if (chg == info->dvchg1_dev) {
			/* NOTE: no PE5 algo object on this tree. */
			val->intval = false;
			break;
		}

		val->intval = is_charger_exist(info);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		if (chg != NULL)
			val->intval = true;
		else
			val->intval = false;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		val->intval = info->enable_hv_charging;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = get_vbus(info);
		break;
	case POWER_SUPPLY_PROP_TEMP:
#ifndef OPLUS_FEATURE_CHG_BASIC
		if (chg == info->chg1_dev)
			val->intval =
				info->chg_data[CHG1_SETTING].junction_temp_max * 10;
		else if (chg == info->chg2_dev)
			val->intval =
				info->chg_data[CHG2_SETTING].junction_temp_max * 10;
		else if (chg == info->dvchg1_dev) {
			pdata = &info->chg_data[DVCHG1_SETTING];
			val->intval = pdata->junction_temp_max;
		} else if (chg == info->dvchg2_dev) {
			pdata = &info->chg_data[DVCHG2_SETTING];
			val->intval = pdata->junction_temp_max;
		} else
			val->intval = -127;
#else
		val->intval = oplus_get_chargeric_temp();
#endif
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = get_charger_charging_current(info, chg);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = get_charger_input_current(info, chg);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = info->chr_type;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_BOOT:
		val->intval = get_charger_zcv(info, chg);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int psy_charger_set_property(struct power_supply *psy,
			enum power_supply_property psp,
			const union power_supply_propval *val)
{
	struct mtk_charger *info;
	int idx;

	chr_err("%s: prop:%d %d\n", __func__, psp, val->intval);

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);

	if (info->psy1 != NULL &&
		info->psy1 == psy)
		idx = CHG1_SETTING;
	else if (info->psy2 != NULL &&
		info->psy2 == psy)
		idx = CHG2_SETTING;
	else if (info->psy_dvchg1 != NULL && info->psy_dvchg1 == psy)
		idx = DVCHG1_SETTING;
	else if (info->psy_dvchg2 != NULL && info->psy_dvchg2 == psy)
		idx = DVCHG2_SETTING;
	else {
		chr_err("%s fail\n", __func__);
		return 0;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (val->intval > 0)
			info->enable_hv_charging = true;
		else
			info->enable_hv_charging = false;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		info->chg_data[idx].thermal_charging_current_limit =
			val->intval;
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		info->chg_data[idx].thermal_input_current_limit =
			val->intval;
		break;
	default:
		return -EINVAL;
	}
	oplus_wake_up_charger(info);

	return 0;
}

static void mtk_charger_external_power_changed(struct power_supply *psy)
{
	struct mtk_charger *info;
	union power_supply_propval prop, prop2;
	struct power_supply *chg_psy = NULL;
	int ret;

	info = (struct mtk_charger *)power_supply_get_drvdata(psy);
	chg_psy = info->chg_psy;

	if (IS_ERR_OR_NULL(chg_psy)) {
		pr_notice("%s Couldn't get chg_psy\n", __func__);
	chg_psy = devm_power_supply_get_by_phandle(&info->pdev->dev,
						       "charger");
		info->chg_psy = chg_psy;
	} else {
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_ONLINE, &prop);
		ret = power_supply_get_property(chg_psy,
			POWER_SUPPLY_PROP_USB_TYPE, &prop2);
	}

	pr_notice("%s event, name:%s online:%d type:%d vbus:%d\n", __func__,
		psy->desc->name, prop.intval, prop2.intval,
		get_vbus(info));

#ifdef OPLUS_FEATURE_CHG_BASIC
	pinfo = info;
	oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
#endif

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	oplus_set_hvdcp_flag_clear();
#endif

	oplus_wake_up_charger(info);
}

int oplus_pd_notify_call(struct notifier_block *notifier,
			unsigned long evt, void *val){
	struct mtk_charger *pinfo = NULL;

	chr_err("%ld\n", evt);

	if (!notifier)
		return NOTIFY_DONE;

	pinfo = container_of(notifier,
		struct mtk_charger, pd_nb);

	switch (evt) {
	case  MTK_PD_CONNECT_NONE:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify Detach\n");
		pinfo->pd_type = MTK_PD_CONNECT_NONE;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		mtk_chg_alg_notify_call(pinfo, EVT_PLUG_OUT, 0);
		/* reset PE40 */
		break;

	case MTK_PD_CONNECT_HARD_RESET:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify HardReset\n");
		pinfo->pd_type = MTK_PD_CONNECT_NONE;
		pinfo->pd_reset = true;
		mutex_unlock(&pinfo->pd_lock);
		mtk_chg_alg_notify_call(pinfo, EVT_PLUG_OUT, 0);
		oplus_wake_up_charger(pinfo);
		/* reset PE40 */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify fixe voltage ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* PD is ready */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK_PD30:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify PD30 ready\r\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_PD30;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* PD30 is ready */
		break;

	case MTK_PD_CONNECT_PE_READY_SNK_APDO:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify APDO Ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_APDO;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* PE40 is ready */
		oplus_wake_up_charger(pinfo);
		break;

	case MTK_PD_CONNECT_TYPEC_ONLY_SNK:
		mutex_lock(&pinfo->pd_lock);
		chr_err("PD Notify Type-C Ready\n");
		pinfo->pd_type = MTK_PD_CONNECT_TYPEC_ONLY_SNK;
		pinfo->pd_reset = false;
		mutex_unlock(&pinfo->pd_lock);
		/* type C is ready */
		oplus_wake_up_charger(pinfo);
		break;
	case MTK_TYPEC_WD_STATUS:
		chr_err("wd status = %d\n", *(bool *)val);
		pinfo->water_detected = *(bool *)val;
		if (pinfo->water_detected == true)
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
		else
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
		mtk_chgstat_notify(pinfo);
		break;
	}
	return NOTIFY_DONE;
}
EXPORT_SYMBOL(oplus_pd_notify_call);

/* Entry point used by oplus_mtk_compat.c, which provides MTK's
 * notify_adapter_event() signature to the built stock charger sources. */
void oplus_mtk6893_notify_adapter_event(unsigned long evt, void *val)
{
	if (!pinfo)
		return;

	oplus_pd_notify_call(&pinfo->pd_nb, evt, val);
}
EXPORT_SYMBOL(oplus_mtk6893_notify_adapter_event);

/* MTK's mtk_pd_adapter.c is built on this tree and calls
 * notify_adapter_event() with MTK's signature
 *	notify_adapter_event(enum adapter_type, enum adapter_event, void *)
 * The stock definition lives in mtk_charger.c, which the OPLUS tree
 * excludes, so the v2 HAL has to provide it -- but in a separate
 * translation unit (oplus_mtk_compat.c), because this file cannot include
 * mtk_charger_intf.h without clashing with the charger macros that are
 * already defined here.
 *
 * It must NOT be the same function as the v2 notifier callback above: with
 * a colliding name the linker bound mtk_pd_adapter.c's call to the v2
 * notifier callback, so an enum (MTK_PD_ADAPTER == 0) was passed where a
 * struct notifier_block * was expected. container_of() on that produced a
 * bogus pinfo and mutex_lock(&pinfo->pd_lock) data-aborted the kernel
 * (ramoops: "Internal error: Oops: 96000006", pc mutex_lock+0x18, Workqueue
 * type_c_port0 tcp_notify_func) as soon as the TCPC started delivering PD
 * events -- i.e. a boot loop. Keep the two apart. */

int chg_alg_event(struct notifier_block *notifier,
			unsigned long event, void *data)
{
	chr_err("evt:%ld\n", event);

	return NOTIFY_DONE;
}

static void mtk_tcpc_set_otg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct mtk_charger *chip;

	/* TCPC notifier is registered before the v2 ic_dev exists; early
	 * SOURCE_VBUS events must not deref a NULL ic_dev (was: boot panic). */
	if (!ic_dev)
		return;
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip)
		return;

	if (chip->otg_enable == en)
		return;
	chg_err("otg_enable = %d\n", en);
	chip->otg_enable = en;
	oplus_chg_ic_virq_trigger(chip->ic_dev, OPLUS_IC_VIRQ_OTG_ENABLE);
}

#define OPLUS_SVID 0x22D9
int oplus_get_adapter_svid(void)
{
	int i = 0, j = 0;
	uint32_t vdos[VDO_MAX_NR] = {0};
	struct tcpc_device *tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
	struct tcpm_svid_list svid_list = {0, {0}};

	if (tcpc_dev == NULL) {
		chg_err("tcpc_dev is null return\n");
		return -1;
	}

	tcpm_inquire_pd_partner_svids(tcpc_dev, &svid_list);
	for (i = 0; i < svid_list.cnt; i++) {
		chg_err("svid[%d] = 0x%x\n", i, svid_list.svids[i]);
		if (svid_list.svids[i] == OPLUS_SVID) {
			pinfo->pd_svooc = true;
			chg_err("match svid and this is oplus adapter\n");
			break;
		}
	}

	tcpm_inquire_pd_partner_inform(tcpc_dev, vdos);
	if ((vdos[0] & 0xFFFF) == OPLUS_SVID) {
		pinfo->pd_svooc = true;
		chg_err("match svid and this is oplus adapter 11\n");
	}

	oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_SVID);

	return 0;
}


static int pd_tcp_notifier_call(struct notifier_block *pnb,
				unsigned long event, void *data)
{
	struct tcp_notify *noti = data;
	struct mtk_charger *pinfo = NULL;
	int ret = 0;

	pinfo = container_of(pnb, struct mtk_charger, pd_nb);

	/* Some TCPC events carry no payload; the union below is only valid
	 * when data is present. */
	if (!noti)
		return NOTIFY_OK;

	pr_notice("PD charger event:%d %d\n", (int)event,
		(int)noti->pd_state.connected);

	switch (event) {
	case TCP_NOTIFY_SOURCE_VBUS:
		if (noti->vbus_state.mv) {
			mtk_tcpc_set_otg_enable(pinfo->ic_dev, true);
		} else {
			mtk_tcpc_set_otg_enable(pinfo->ic_dev, false);
		}
		break;

	case TCP_NOTIFY_PD_STATE:
		switch (noti->pd_state.connected) {
		case  PD_CONNECT_NONE:
			pinfo->pd_type = MTK_PD_CONNECT_NONE;
			chr_err("PD Notify Detach\n");
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;

		case PD_CONNECT_HARD_RESET:
			pinfo->pd_type = MTK_PD_CONNECT_NONE;
			chr_err("PD Notify HardReset\n");
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;

		case PD_CONNECT_PE_READY_SNK:
			pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK;
			chr_err("PD Notify fixe voltage ready\n");
			oplus_get_adapter_svid();
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;

		case PD_CONNECT_PE_READY_SNK_PD30:
			pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_PD30;
			chr_err("PD Notify PD30 ready\r\n");
			oplus_get_adapter_svid();
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;

		case PD_CONNECT_PE_READY_SNK_APDO:
			pinfo->pd_type = MTK_PD_CONNECT_PE_READY_SNK_APDO;
			chr_err("PD Notify APDO Ready\n");
			oplus_get_adapter_svid();
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;

		case PD_CONNECT_TYPEC_ONLY_SNK_DFT:
			/* fall-through */
		case PD_CONNECT_TYPEC_ONLY_SNK:
			pinfo->pd_type = MTK_PD_CONNECT_TYPEC_ONLY_SNK;
			chr_err("PD Notify Type-C Ready\n");
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_SVID);/* not support svid */
			oplus_chg_ic_virq_trigger(pinfo->ic_dev, OPLUS_IC_VIRQ_CHG_TYPE_CHANGE);
			break;
		}
		break;
	case TCP_NOTIFY_TYPEC_STATE:
		chr_err("old_state=%d, new_state=%d\n",
				noti->typec_state.old_state,
				noti->typec_state.new_state);
		if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SRC) {
			chr_err("adb she-C SRC plug in\n");
		} else if (noti->typec_state.old_state == TYPEC_UNATTACHED &&
			noti->typec_state.new_state == TYPEC_ATTACHED_SNK) {
			chr_err("Type-C SINK plug in\n");
		} else if ((noti->typec_state.old_state == TYPEC_ATTACHED_SRC ||
			noti->typec_state.old_state == TYPEC_ATTACHED_SNK) &&
			noti->typec_state.new_state == TYPEC_UNATTACHED) {
			chr_err("Type-C plug out\n");
			pinfo->pd_svooc = false;
		}
		break;
	case TCP_NOTIFY_WD_STATUS:
		chr_err("wd status = %d\n", (bool)noti->wd_status.water_detected);
		pinfo->water_detected = (bool)noti->wd_status.water_detected;
		if (pinfo->water_detected == true)
			pinfo->notify_code |= CHG_TYPEC_WD_STATUS;
		else
			pinfo->notify_code &= ~CHG_TYPEC_WD_STATUS;
		mtk_chgstat_notify(pinfo);
		break;
	/* NOTE: no TCP_NOTIFY_WD0_STATE/CHRDET_STATE on this TCPC. */
	}
	return ret;
}

#ifdef CONFIG_OPLUS_RTC_DET_SUPPORT
static int rtc_reset_check(void)
{
	int rc = 0;
	struct rtc_time tm;
	struct rtc_device *rtc;

	rtc = rtc_class_open(CONFIG_RTC_HCTOSYS_DEVICE);
	if (rtc == NULL) {
		pr_err("%s: unable to open rtc device (%s)\n",
			__FILE__, CONFIG_RTC_HCTOSYS_DEVICE);
		return 0;
	}

	rc = rtc_read_time(rtc, &tm);
	if (rc) {
		pr_err("Error reading rtc device (%s) : %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	rc = rtc_valid_tm(&tm);
	if (rc) {
		pr_err("Invalid RTC time (%s): %d\n",
			CONFIG_RTC_HCTOSYS_DEVICE, rc);
		goto close_time;
	}

	if ((tm.tm_year == 70) && (tm.tm_mon == 0) && (tm.tm_mday <= 1)) {
		chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  @@@ wday: %d, yday: %d, isdst: %d\n",
			tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
			tm.tm_wday, tm.tm_yday, tm.tm_isdst);
		rtc_class_close(rtc);
		return 1;
	}

	chg_debug(": Sec: %d, Min: %d, Hour: %d, Day: %d, Mon: %d, Year: %d  ###  wday: %d, yday: %d, isdst: %d\n",
		tm.tm_sec, tm.tm_min, tm.tm_hour, tm.tm_mday, tm.tm_mon, tm.tm_year,
		tm.tm_wday, tm.tm_yday, tm.tm_isdst);

close_time:
	rtc_class_close(rtc);
	return 0;
}
#endif /* CONFIG_OPLUS_RTC_DET_SUPPORT */

int mt_power_supply_type_check(void)
{
	int chr_type;
	union power_supply_propval prop;
	struct power_supply *chg_psy = NULL;

	chg_psy = devm_power_supply_get_by_phandle(&pinfo->pdev->dev, "charger");
	if (IS_ERR_OR_NULL(chg_psy)) {
		chr_err("%s Couldn't get chg_psy\n", __func__);
	} else {
		power_supply_get_property(chg_psy, POWER_SUPPLY_PROP_ONLINE, &prop);
	}

	chr_type = get_charger_type(pinfo);

	chr_err("charger_type[%d]\n", chr_type);

	return chr_type;
}
EXPORT_SYMBOL(mt_power_supply_type_check);

void mt_usb_connect(void)
{
	return;
}
EXPORT_SYMBOL(mt_usb_connect);

void mt_usb_disconnect(void)
{
	return;
}
EXPORT_SYMBOL(mt_usb_disconnect);

int get_rtc_spare_oplus_fg_value(void)
{
	return 0;
}
EXPORT_SYMBOL(get_rtc_spare_oplus_fg_value);

int set_rtc_spare_oplus_fg_value(int soc)
{
        return 0;
}
EXPORT_SYMBOL(set_rtc_spare_oplus_fg_value);


#ifdef OPLUS_FEATURE_CHG_BASIC

static int oplus_suspend_charger(bool suspend)
{
	struct votable *suspend_votable;
	int rc;

	suspend_votable = find_votable("WIRED_CHARGE_SUSPEND");
	if (!suspend_votable) {
		chg_err("WIRED_CHARGE_SUSPEND votable not found\n");
		return -EINVAL;
	}

	rc = vote(suspend_votable, PDQC_VOTER, suspend, 0, true);
	if (rc < 0)
		chg_err("%s charger error, rc=%d\n",
			suspend ? "suspend" : "unsuspend", rc);
	else
		chg_info("%s charger\n", suspend ? "suspend" : "unsuspend");

	return rc;
}

static int mtk_chg_init(struct oplus_chg_ic_dev *ic_dev)
{
	ic_dev->online = true;
	return 0;
}

static int mtk_chg_exit(struct oplus_chg_ic_dev *ic_dev)
{
	if (!ic_dev->online)
		return 0;

	ic_dev->online = false;
	return 0;
}

static int mtk_chg_reg_dump(struct oplus_chg_ic_dev *ic_dev)
{
	return 0;
}

static int  mtk_chg_smt_test(struct oplus_chg_ic_dev *ic_dev, char buf[], int len)
{
	return 0;
}

static int mtk_chg_input_present(struct oplus_chg_ic_dev *ic_dev, bool *present)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	if (oplus_mt_get_vbus_status() == true)
		*present = true;
	else
		*present = false;
	/*chg_info("mtk_chg_input_present *present = %d\n", *present);*/

	return 0;
}

static int mtk_chg_input_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	rc = mt6360_suspend_charger(suspend);
	if (rc < 0) {
		chg_err("suspend charger %d fail, rc=%d\n", suspend, rc);
		return rc;
	}
	chip->input_suspend_sts = suspend;

	return 0;
}

static int mtk_chg_input_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL || suspend == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*suspend = chip->input_suspend_sts;

	return 0;
}

static int mtk_chg_output_suspend(struct oplus_chg_ic_dev *ic_dev, bool suspend)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_enable(chip->chg1_dev, !suspend);
	if (rc < 0) {
		chg_err("enable charger %d fail, rc=%d\n", !suspend, rc);
		return rc;
	}
	chip->output_suspend_sts = suspend;

	return 0;
}

static int mtk_chg_output_is_suspend(struct oplus_chg_ic_dev *ic_dev, bool *suspend)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL || suspend == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*suspend = chip->output_suspend_sts;

	return 0;
}

static int mtk_chg_set_icl(struct oplus_chg_ic_dev *ic_dev, bool vooc_mode,
			   bool step, int icl_ma)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	/* direct set; AICL stepping policy runs via BUCK_AICL_RERUN */
	(void)vooc_mode;
	(void)step;
	rc = charger_dev_set_input_current(chip->chg1_dev, icl_ma * 1000);
	if (rc < 0) {
		chg_err("set input current %d fail, rc=%d\n", icl_ma, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_get_icl(struct oplus_chg_ic_dev *ic_dev, int *icl_ma)
{
	struct mtk_charger *chip;
	u32 icl_ua = 0;
	int rc;

	if (ic_dev == NULL || icl_ma == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_get_input_current(chip->chg1_dev, &icl_ua);
	if (rc < 0) {
		chg_err("get input current fail, rc=%d\n", rc);
		return rc;
	}
	*icl_ma = icl_ua / 1000;

	return 0;
}

static int mtk_chg_set_fcc(struct oplus_chg_ic_dev *ic_dev, int fcc_ma)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_set_charging_current(chip->chg1_dev, fcc_ma * 1000);
	if (rc < 0) {
		chg_err("set fcc %d fail, rc=%d\n", fcc_ma, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_set_fv(struct oplus_chg_ic_dev *ic_dev, int fv_mv)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_set_constant_voltage(chip->chg1_dev, fv_mv * 1000);
	if (rc < 0) {
		chg_err("set fv %d fail, rc=%d\n", fv_mv, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_set_iterm(struct oplus_chg_ic_dev *ic_dev, int iterm_ma)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_set_eoc_current(chip->chg1_dev, iterm_ma * 1000);
	if (rc < 0) {
		chg_err("set iterm %d fail, rc=%d\n", iterm_ma, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_set_rechg_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	rc = mt6360_set_rechg_voltage(vol_mv);
	if (rc < 0) {
		chg_err("set rechg %d fail, rc=%d\n", vol_mv, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_get_input_curr(struct oplus_chg_ic_dev *ic_dev, int *curr_ma)
{
	struct mtk_charger *chip;
	u32 ibus = 0;
	int rc;

	if (ic_dev == NULL || curr_ma == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_get_ibus(chip->chg1_dev, &ibus);
	if (rc < 0) {
		chg_err("get ibus fail, rc=%d\n", rc);
		return rc;
	}
	*curr_ma = ibus / 1000;

	return 0;
}

static int mtk_chg_get_input_vol(struct oplus_chg_ic_dev *ic_dev, int *vol_mv)
{
	if (ic_dev == NULL || vol_mv == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	*vol_mv = battery_meter_get_charger_voltage() / 1000;

	return 0;
}

static int mtk_chg_otg_boost_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_enable_otg(chip->chg1_dev, en);
	if (rc < 0) {
		chg_err("enable otg %d fail, rc=%d\n", en, rc);
		return rc;
	}
	chip->otg_enable = en;

	return 0;
}

static int mtk_chg_set_otg_boost_vol(struct oplus_chg_ic_dev *ic_dev, int vol_mv)
{
	/* TODO */
	return 0;
}

static int mtk_chg_set_otg_boost_curr_limit(struct oplus_chg_ic_dev *ic_dev, int curr_ma)
{
	/* TODO */
	return 0;
}

static int mtk_chg_aicl_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	rc = mt6360_aicl_enable(en);
	if (rc < 0) {
		chg_err("aicl enable %d fail, rc=%d\n", en, rc);
		return rc;
	}

	return rc;
}

static int mtk_chg_aicl_rerun(struct oplus_chg_ic_dev *ic_dev)
{
	struct mtk_charger *chip;
	u32 uA = 0;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_run_aicl(chip->chg1_dev, &uA);
	if (rc < 0) {
		chg_err("run aicl fail, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int mtk_chg_aicl_reset(struct oplus_chg_ic_dev *ic_dev)
{
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	rc = mt6360_aicl_enable(false);
	if (rc < 0)
		return rc;
	rc = mt6360_aicl_enable(true);
	if (rc < 0) {
		chg_err("aicl reset fail, rc=%d\n", rc);
		return rc;
	}

	return rc;
}

static int mtk_chg_get_cc_orientation(struct oplus_chg_ic_dev *ic_dev, int *orientation)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->tcpc) {
		chg_err("tcpc is NULL");
		return -ENODEV;
	}

	if (tcpm_inquire_typec_attach_state(chip->tcpc) != TYPEC_UNATTACHED)
		*orientation = (int)tcpm_inquire_cc_polarity(chip->tcpc) + 1;
	else
		*orientation = 0;

	if (*orientation != 0)
		chg_info("cc[%d]\n", *orientation);

	return 0;
}

static int mtk_chg_get_hw_detect(struct oplus_chg_ic_dev *ic_dev, int *detected)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*detected = chip->wd0_detect;

	return 0;
}

static bool oplus_pd_without_usb(void)
{
	struct tcpc_device *tcpc;

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return -ENODEV;
	}

	if (!tcpm_inquire_pd_connected(tcpc))
		return true;
	return (tcpm_inquire_dpm_flags(tcpc) &
			DPM_FLAGS_PARTNER_USB_COMM) ? false : true;
}

static int mtk_chg_get_charger_type(struct oplus_chg_ic_dev *ic_dev, int *type)
{
	struct mtk_charger *chip;
	bool online;
	int charger_type;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	online = oplus_mt_get_vbus_status();
	charger_type = get_charger_type(chip);
	if (!online)
		charger_type = POWER_SUPPLY_TYPE_UNKNOWN;

	chr_debug("online:%d type:%d\n", online, charger_type);

	switch (charger_type) {
	case POWER_SUPPLY_TYPE_UNKNOWN:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	case POWER_SUPPLY_TYPE_USB:
		*type = OPLUS_CHG_USB_TYPE_SDP;
		break;
	case POWER_SUPPLY_TYPE_USB_DCP:
		*type = OPLUS_CHG_USB_TYPE_DCP;
		break;
	case POWER_SUPPLY_TYPE_USB_CDP:
		*type = OPLUS_CHG_USB_TYPE_CDP;
		break;
	case POWER_SUPPLY_TYPE_USB_ACA:
		*type = OPLUS_CHG_USB_TYPE_ACA;
		break;
	case POWER_SUPPLY_TYPE_USB_TYPE_C:
		*type = OPLUS_CHG_USB_TYPE_C;
		break;
	case POWER_SUPPLY_TYPE_USB_PD:
		*type = OPLUS_CHG_USB_TYPE_PD;
		break;
	case POWER_SUPPLY_TYPE_USB_PD_DRP:
		*type = OPLUS_CHG_USB_TYPE_PD_DRP;
		break;
	case POWER_SUPPLY_TYPE_APPLE_BRICK_ID:
		*type = OPLUS_CHG_USB_TYPE_APPLE_BRICK_ID;
		break;
	default:
		*type = OPLUS_CHG_USB_TYPE_UNKNOWN;
		break;
	}

	if (chip->pd_type == MTK_PD_CONNECT_PE_READY_SNK ||
			chip->pd_type == MTK_PD_CONNECT_PE_READY_SNK_PD30||
			chip->pd_type == MTK_PD_CONNECT_PE_READY_SNK_APDO)
		*type = OPLUS_CHG_USB_TYPE_PD;

	if (*type != OPLUS_CHG_USB_TYPE_UNKNOWN && !oplus_pd_without_usb())
		*type = OPLUS_CHG_USB_TYPE_PD_SDP;

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (mt6360_get_hvdcp_type() == POWER_SUPPLY_TYPE_USB_HVDCP)
		*type = OPLUS_CHG_USB_TYPE_QC2;
#endif

	return 0;
}

static int mtk_chg_rerun_bc12(struct oplus_chg_ic_dev *ic_dev)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_enable_chg_type_det(chip->chg1_dev, true);
	if (rc < 0) {
		chg_err("rerun bc12 fail, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_qc_detect_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (en) {
		chg_err("enter chg enable qc!\n");
		mt6360_enable_hvdcp_detect();
	}
#else
	(void)en;
#endif
	return 0;
}

static int mtk_chg_shipmode_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	/* MT6360 shipping mode is one-way; en=false is a no-op */
	if (!en)
		return 0;

	rc = mt6360_enter_shipmode();
	if (rc < 0) {
		chg_err("enter shipmode fail, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

#define PDQC_BOOST_DELAY_MS	300
#define QC_BUCK_DELAY_MS	400
static int mtk_chg_set_qc_config(struct oplus_chg_ic_dev *ic_dev, enum oplus_chg_qc_version version, int vol_mv)
{
	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}

	switch (version) {
	case OPLUS_CHG_QC_2_0:
		if (vol_mv != 5000 && vol_mv != 9000) {
			chg_err("Unsupported qc voltage(=%d)\n", vol_mv);
			return -EINVAL;
		}
		if (vol_mv == 9000) {
			mt6360_set_hvdcp_to_5v();
			msleep(PDQC_BOOST_DELAY_MS);
			oplus_suspend_charger(true);
			mt6360_set_hvdcp_to_9v();
			oplus_notify_hvdcp_detect_stat();
			msleep(PDQC_BOOST_DELAY_MS);
			oplus_suspend_charger(false);
		} else if (vol_mv == 5000) {
			oplus_suspend_charger(true);
			mt6360_set_hvdcp_to_5v();
			msleep(QC_BUCK_DELAY_MS);
			oplus_suspend_charger(false);
		}
		break;
	case OPLUS_CHG_QC_3_0:
	default:
		chg_err("Unsupported qc version(=%d)\n", version);
		return -EINVAL;
	}

	return 0;
}

static int mtk_chg_set_pd_config(struct oplus_chg_ic_dev *ic_dev, u32 pdo)
{
	struct mtk_charger *chip;
	struct tcpc_device *tcpc;
	int vol_mv, curr_ma;
	int vbus_mv_t = 0;
	int ibus_ma_t = 0;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!tcpc) {
		chg_err("get type_c_port0 fail\n");
		return -ENODEV;
	}

	switch (PD_SRC_PDO_TYPE(pdo)) {
	case PD_SRC_PDO_TYPE_FIXED:
		vol_mv = PD_SRC_PDO_FIXED_VOLTAGE(pdo) * 50;
		curr_ma = PD_SRC_PDO_FIXED_MAX_CURR(pdo) * 10;
		if (curr_ma >= 2000)
			curr_ma = 2000;
		break;
	case PD_SRC_PDO_TYPE_BATTERY:
	case PD_SRC_PDO_TYPE_VARIABLE:
	case PD_SRC_PDO_TYPE_AUGMENTED:
	default:
		chg_err("Unsupported pdo type(=%d)\n", PD_SRC_PDO_TYPE(pdo));
		return -EINVAL;
	}

	oplus_suspend_charger(true);

	rc = tcpm_dpm_pd_request(tcpc, vol_mv, curr_ma, NULL);
	if (rc != TCPM_SUCCESS) {
		chg_err("tcpm_dpm_pd_request fail, rc=%d\n", rc);
		rc = -EINVAL;
		goto out;
	}
	rc = tcpm_inquire_pd_contract(tcpc, &vbus_mv_t, &ibus_ma_t);
	if (rc != TCPM_SUCCESS) {
		chg_err("inquire current vbus_mv and ibus_ma fail, rc=%d\n", rc);
		rc = -EINVAL;
		goto out;
	}

	msleep(PDQC_BOOST_DELAY_MS);
	chg_info("PD Default vbus_mv[%d], ibus_ma[%d]\n", vol_mv, curr_ma);
out:
	oplus_suspend_charger(false);
	return rc;
}

static int mtk_chg_get_shutdown_soc(struct oplus_chg_ic_dev *ic_dev, int *soc)
{
	/* TODO */
	return -ENOTSUPP;
}

static int mtk_chg_backup_soc(struct oplus_chg_ic_dev *ic_dev, int soc)
{
	/* TODO */
	return -ENOTSUPP;
}

static int mtk_chg_get_vbus_collapse_status(struct oplus_chg_ic_dev *ic_dev, bool *collapse)
{
	/* TODO */
	return -ENOTSUPP;
}

static int mtk_chg_get_typec_mode(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type *mode)
{
	struct mtk_charger *chip;
	uint8_t typec_role;

	if (!ic_dev || !pinfo || !pinfo->tcpc) {
		chg_err("[%s]: g_oplus_chip not ready!\n", __func__);
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	typec_role = tcpm_inquire_typec_role(pinfo->tcpc);

	switch(typec_role) {
	case TYPEC_ROLE_SNK:
		*mode = TYPEC_PORT_ROLE_SNK;
		break;
	case TYPEC_ROLE_SRC:
		*mode = TYPEC_PORT_ROLE_SRC;
		break;
	case TYPEC_ROLE_DRP:
		*mode = TYPEC_PORT_ROLE_DRP;
		break;
	case TYPEC_ROLE_TRY_SRC:
		*mode = TYPEC_PORT_ROLE_TRY_SRC;
		break;
	case TYPEC_ROLE_TRY_SNK:
		*mode = TYPEC_PORT_ROLE_TRY_SNK;
		break;
	case TYPEC_ROLE_UNKNOWN:
	case TYPEC_ROLE_NR:
	default:
		*mode = TYPEC_PORT_ROLE_INVALID;
		break;
	}
	return 0;
}

static int mtk_chg_set_typec_mode(struct oplus_chg_ic_dev *ic_dev,
					enum oplus_chg_typec_port_role_type mode)
{
	struct mtk_charger *chip;
	uint8_t typec_role;
	int rc = 0;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	if (!chip->tcpc)
		chip->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!chip->tcpc) {
		chg_err("get type_c_port0 fail\n");
		return -ENODEV;
	}

	switch(mode) {
	case TYPEC_PORT_ROLE_DRP:
		typec_role = TYPEC_ROLE_DRP;
		break;
	case TYPEC_PORT_ROLE_SNK:
		typec_role = TYPEC_ROLE_SNK;
		break;
	case TYPEC_PORT_ROLE_SRC:
		typec_role = TYPEC_ROLE_SRC;
		break;
	case TYPEC_PORT_ROLE_TRY_SNK:
		typec_role = TYPEC_ROLE_TRY_SNK;
		break;
	case TYPEC_PORT_ROLE_TRY_SRC:
		typec_role = TYPEC_ROLE_TRY_SRC;
		break;
	case TYPEC_PORT_ROLE_INVALID:
		typec_role = TYPEC_ROLE_UNKNOWN;
		break;
	case TYPEC_PORT_ROLE_DISABLE:
		oplus_usbtemp_set_cc_open();
		goto out;
	case TYPEC_PORT_ROLE_ENABLE:
		oplus_usbtemp_recover_cc_open();
		goto out;
	}

	rc = tcpm_typec_change_role_postpone(pinfo->tcpc, typec_role, true);
	if (rc < 0)
		chg_err("can't set typec_role=%d, rc=%d\n", mode, rc);
out:
	return rc;
}

static bool mtk_otg_switch_state;

static int mtk_chg_set_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	/* This is the USB *mux / redriver* enable, not the Type-C port role.
	 * The vendor writes it at boot:
	 *   /vendor/etc/init/hw/init.mt6893.rc:
	 *     on property:sys.boot_completed=1
	 *         write /sys/devices/virtual/oplus_chg/usb/otg_switch 1
	 * and stock handled it in extcon-mtk-usb.c::otg_switch_mode(), which
	 * only drives OTG_ID_GPIO_NUM and flips otg_switch_state.
	 *
	 * Calling tcpm_typec_change_role_postpone(TYPEC_ROLE_DRP) here -- which
	 * is what this port did -- put the TCPC into DRP ~19s into every boot.
	 * A DRP facing an A-plug source drives its own Rp and resolves to
	 * SRC/Try.SRC, so the port never reached TYPEC_ATTACHED_SNK: BC1.2
	 * never ran (usbid_evt IRQ count stayed 0), EXTCON_USB never rose and
	 * the adb gadget never enumerated on a PC.
	 *
	 * Keep the flag; do not touch the Type-C role. */
	mtk_otg_switch_state = en;
	chg_err("[OPLUS_CHG][%s]: otg switch[%d]\n", __func__, en);
	return 0;
}

static int mtk_chg_get_otg_switch_status(struct oplus_chg_ic_dev *ic_dev, bool *en)
{
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	*en = mtk_otg_switch_state;
	return 0;
}

static int mtk_chg_get_otg_enbale(struct oplus_chg_ic_dev *ic_dev, bool *enable)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*enable = chip->otg_enable;
	return 0;
}


static int mtk_chg_is_oplus_svid(struct oplus_chg_ic_dev *ic_dev, bool *oplus_svid)
{
	struct mtk_charger *chip;
	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	*oplus_svid = chip->pd_svooc;
	return 0;
}

static int mtk_chg_set_usbtemp_dischg_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	struct mtk_charger *chip;
	chip = oplus_chg_ic_get_drvdata(ic_dev);

	oplus_chg_set_dischg_enable(en);
	chg_err("set_usbtemp_dischg_enable=%d\n", en);

	return 0;
}

static int mtk_chg_wdt_enable(struct oplus_chg_ic_dev *ic_dev, bool en)
{
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}

	rc = mt6360_chg_enable_wdt(en);
	if (rc < 0) {
		chg_err("wdt enable %d fail, rc=%d\n", en, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_kick_wdt(struct oplus_chg_ic_dev *ic_dev)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_kick_wdt(chip->chg1_dev);
	if (rc < 0) {
		chg_err("kick wdt fail, rc=%d\n", rc);
		return rc;
	}

	return 0;
}

/* HW AICL point with 4400 <-> 4500 mV hysteresis on cell voltage
 * (v1 oplus_mt6360_set_aicl_point). vbatt_mv is max-cell mV. */
static int mtk_chg_set_aicl_point(struct oplus_chg_ic_dev *ic_dev, int vbatt_mv)
{
	struct mtk_charger *chip;
	static int hw_aicl_point = 4400;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	if (hw_aicl_point == 4400 && vbatt_mv > 4100)
		hw_aicl_point = 4500;
	else if (hw_aicl_point == 4500 && vbatt_mv <= 4100)
		hw_aicl_point = 4400;

	rc = charger_dev_set_mivr(chip->chg1_dev, hw_aicl_point * 1000);
	if (rc < 0) {
		chg_err("set aicl point %d fail, rc=%d\n", hw_aicl_point, rc);
		return rc;
	}

	return 0;
}

static int mtk_chg_set_vindpm(struct oplus_chg_ic_dev *ic_dev, int vindpm_mv)
{
	struct mtk_charger *chip;
	int rc;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	rc = charger_dev_set_mivr(chip->chg1_dev, vindpm_mv * 1000);
	if (rc < 0) {
		chg_err("set vindpm %d fail, rc=%d\n", vindpm_mv, rc);
		return rc;
	}

	return 0;
}

/* Init values from v1 oplus_mt6360_hardware_init (Cupida): float 4435 mV,
 * FCC 512 mA, iterm 150 mA, rechg 100 mV, MIVR 4400 mV, MT6360 init regs.
 * Never copy other-project numbers. */
static int mtk_chg_hardware_init(struct oplus_chg_ic_dev *ic_dev)
{
	struct mtk_charger *chip;

	if (ic_dev == NULL) {
		chg_err("oplus_chg_ic_dev is NULL");
		return -ENODEV;
	}
	chip = oplus_chg_ic_get_drvdata(ic_dev);
	if (!chip->chg1_dev) {
		chg_err("chg1_dev is NULL");
		return -ENODEV;
	}

	mt6360_reset_charger();

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT) {
		charger_dev_enable(chip->chg1_dev, false);
		charger_dev_set_constant_voltage(chip->chg1_dev, 4435 * 1000);
		msleep(100);
	}

	charger_dev_set_constant_voltage(chip->chg1_dev, 4435 * 1000);

	mt6360_set_register(0x1C, 0xFF, 0xF9);
	mt6360_set_register(0x18, 0xF, 0x4);

	charger_dev_set_charging_current(chip->chg1_dev, 512 * 1000);
	charger_dev_set_eoc_current(chip->chg1_dev, 150 * 1000);
	mt6360_set_rechg_voltage(100);
	charger_dev_set_mivr(chip->chg1_dev, 4400 * 1000);

	if (get_boot_mode() == META_BOOT || get_boot_mode() == FACTORY_BOOT
	    || get_boot_mode() == ADVMETA_BOOT ||
	    get_boot_mode() == ATE_FACTORY_BOOT) {
		mt6360_suspend_charger(true);
		charger_dev_enable(chip->chg1_dev, false);
	} else {
		mt6360_suspend_charger(false);
		charger_dev_enable(chip->chg1_dev, true);
	}

	mt6360_set_register(0x1D, 0x30, 0x10);

	return 0;
}

static void *oplus_chg_get_func(struct oplus_chg_ic_dev *ic_dev,
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
					       mtk_chg_init);
		break;
	case OPLUS_IC_FUNC_EXIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_EXIT,
					       mtk_chg_exit);
		break;
	case OPLUS_IC_FUNC_REG_DUMP:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_REG_DUMP,
					       mtk_chg_reg_dump);
		break;
	case OPLUS_IC_FUNC_SMT_TEST:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SMT_TEST,
					       mtk_chg_smt_test);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_PRESENT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_PRESENT,
					       mtk_chg_input_present);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_SUSPEND,
					       mtk_chg_input_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_INPUT_IS_SUSPEND,
							mtk_chg_input_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
							mtk_chg_output_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_OUTPUT_IS_SUSPEND,
							mtk_chg_output_is_suspend);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ICL,
					       mtk_chg_set_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_ICL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_ICL,
					       mtk_chg_get_icl);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FCC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FCC,
					       mtk_chg_set_fcc);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_FV:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_FV,
					       mtk_chg_set_fv);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_ITERM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_ITERM,
					       mtk_chg_set_iterm);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_RECHG_VOL,
					       mtk_chg_set_rechg_vol);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_CURR,
							mtk_chg_get_input_curr);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_INPUT_VOL,
					       mtk_chg_get_input_vol);
		break;
	case OPLUS_IC_FUNC_OTG_BOOST_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
					       mtk_chg_otg_boost_enable);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_VOL:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_VOL,
					       mtk_chg_set_otg_boost_vol);
		break;
	case OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_BOOST_CURR_LIMIT,
							mtk_chg_set_otg_boost_curr_limit);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_ENABLE,
					       mtk_chg_aicl_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RERUN:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RERUN,
					       mtk_chg_aicl_rerun);
		break;
	case OPLUS_IC_FUNC_BUCK_AICL_RESET:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_AICL_RESET,
					       mtk_chg_aicl_reset);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CC_ORIENTATION,
							mtk_chg_get_cc_orientation);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_HW_DETECT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_HW_DETECT,
					       mtk_chg_get_hw_detect);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_CHARGER_TYPE,
							mtk_chg_get_charger_type);
		break;
	case OPLUS_IC_FUNC_BUCK_RERUN_BC12:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_RERUN_BC12,
					       mtk_chg_rerun_bc12);
		break;
	case OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_QC_DETECT_ENABLE,
							mtk_chg_qc_detect_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SHIPMODE_ENABLE,
							mtk_chg_shipmode_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_QC_CONFIG,
					       mtk_chg_set_qc_config);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_PD_CONFIG,
					       mtk_chg_set_pd_config);
		break;
	case OPLUS_IC_FUNC_BUCK_WDT_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_WDT_ENABLE,
					       mtk_chg_wdt_enable);
		break;
	case OPLUS_IC_FUNC_BUCK_KICK_WDT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_KICK_WDT,
					       mtk_chg_kick_wdt);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_AICL_POINT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_AICL_POINT,
					       mtk_chg_set_aicl_point);
		break;
	case OPLUS_IC_FUNC_BUCK_SET_VINDPM:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_SET_VINDPM,
					       mtk_chg_set_vindpm);
		break;
	case OPLUS_IC_FUNC_BUCK_HARDWARE_INIT:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_HARDWARE_INIT,
					       mtk_chg_hardware_init);
		break;
	case OPLUS_IC_FUNC_GET_SHUTDOWN_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_SHUTDOWN_SOC,
					       mtk_chg_get_shutdown_soc);
		break;
	case OPLUS_IC_FUNC_BACKUP_SOC:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BACKUP_SOC,
					       mtk_chg_backup_soc);
		break;
	case OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_BUCK_GET_VBUS_COLLAPSE_STATUS,
			mtk_chg_get_vbus_collapse_status);
		break;
	case OPLUS_IC_FUNC_GET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_TYPEC_MODE,
					       mtk_chg_get_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_TYPEC_MODE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_TYPEC_MODE,
						   mtk_chg_set_typec_mode);
		break;
	case OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_OTG_SWITCH_STATUS, 
						   mtk_chg_set_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_SWITCH_STATUS, 
						   mtk_chg_get_otg_switch_status);
		break;
	case OPLUS_IC_FUNC_GET_OTG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_GET_OTG_ENABLE,
					       mtk_chg_get_otg_enbale);
		break;
	case OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_SET_USB_DISCHG_ENABLE,
					       mtk_chg_set_usbtemp_dischg_enable);
		break;
	case OPLUS_IC_FUNC_IS_OPLUS_SVID:
		func = OPLUS_CHG_IC_FUNC_CHECK(OPLUS_IC_FUNC_IS_OPLUS_SVID,
					       mtk_chg_is_oplus_svid);
		break;
	default:
		chg_err("this func(=%d) is not supported\n", func_id);
		func = NULL;
		break;
	}

	return func;
}

struct oplus_chg_ic_virq mtk_chg_virq_table[] = {
	{ .virq_id = OPLUS_IC_VIRQ_ERR },
	{ .virq_id = OPLUS_IC_VIRQ_CC_DETECT },
	{ .virq_id = OPLUS_IC_VIRQ_PLUGIN },
	{ .virq_id = OPLUS_IC_VIRQ_CC_CHANGED },
	{ .virq_id = OPLUS_IC_VIRQ_CHG_TYPE_CHANGE },
	{ .virq_id = OPLUS_IC_VIRQ_OTG_ENABLE },
	{ .virq_id = OPLUS_IC_VIRQ_SVID },
};
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
#define CHARGER_25C_VOLT	900
struct chtemperature {
	__s32 bts_temp;
	__s32 temperature_r;
};

static struct chtemperature charger_ic_temp_table[] = {
	{-40, 4251000}, {-39, 3962000}, {-38, 3695000}, {-37, 3447000}, {-36, 3218000}, {-35, 3005000},
	{-34, 2807000}, {-33, 2624000}, {-32, 2454000}, {-31, 2296000}, {-30, 2149000}, {-29, 2012000},
	{-28, 1885000}, {-27, 1767000}, {-26, 1656000}, {-25, 1554000}, {-24, 1458000}, {-23, 1369000},
	{-22, 1286000}, {-21, 1208000}, {-20, 1135000}, {-19, 1068000}, {-18, 1004000}, {-17, 945000 },
	{-16, 889600 }, {-15, 837800 }, {-14, 789300 }, {-13, 743900 }, {-12, 701300 }, {-11, 661500 },
	{-10, 624100 }, {-9, 589000  }, {-8, 556200  }, {-7, 525300  }, {-6, 496300  }, {-5, 469100  },
	{-4, 443500  }, {-3, 419500  }, {-2, 396900  }, {-1, 375600  }, {0, 355600   }, {1, 336800   },
	{2, 319100   }, {3, 302400   }, {4, 286700   }, {5, 271800   }, {6, 257800   }, {7, 244700   },
	{8, 232200   }, {9, 220500   }, {10, 209400  }, {11, 198900  }, {12, 189000  }, {13, 179700  },
	{14, 170900  }, {15, 162500  }, {16, 154600  }, {17, 147200  }, {18, 140100  }, {19, 133400  },
	{20, 127000  }, {21, 121000  }, {22, 115400  }, {23, 110000  }, {24, 104800  }, {25, 100000  },
	{26, 95400   }, {27, 91040   }, {28, 86900   }, {29, 82970   }, {30, 79230   }, {31, 75690   },
	{32, 72320   }, {33, 69120   }, {34, 66070   }, {35, 63180   }, {36, 60420   }, {37, 57810   },
	{38, 55310   }, {39, 52940   }, {40, 50680   }, {41, 48530   }, {42, 46490   }, {43, 44530   },
	{44, 42670   }, {45, 40900   }, {46, 39210   }, {47, 37600   }, {48, 36060   }, {49, 34600   },
	{50, 33190   }, {51, 31860   }, {52, 30580   }, {53, 29360   }, {54, 28200   }, {55, 27090   },
	{56, 26030   }, {57, 25010   }, {58, 24040   }, {59, 23110   }, {60, 22220   }, {61, 21370   },
	{62, 20560   }, {63, 19780   }, {64, 19040   }, {65, 18320   }, {66, 17640   }, {67, 16990   },
	{68, 16360   }, {69, 15760   }, {70, 15180   }, {71, 14630   }, {72, 14100   }, {73, 13600   },
	{74, 13110   }, {75, 12640   }, {76, 12190   }, {77, 11760   }, {78, 11350   }, {79, 10960   },
	{80, 10580   }, {81, 10210   }, {82, 9859    }, {83, 9522    }, {84, 9198    }, {85, 8887    },
	{86, 8587    }, {87, 8299    }, {88, 8022    }, {89, 7756    }, {90, 7500    }, {91, 7254    },
	{92, 7016    }, {93, 6788    }, {94, 6568    }, {95, 6357    }, {96, 6153    }, {97, 5957    },
	{98, 5768    }, {99, 5586    }, {100, 5410   }, {101, 5241   }, {102, 5078   }, {103, 4921   },
	{104, 4769   }, {105, 4623   }, {106, 4482   }, {107, 4346   }, {108, 4215   }, {109, 4088   },
	{110, 3965   }, {111, 3847   }, {112, 3733   }, {113, 3623   }, {114, 3517   }, {115, 3415   },
	{116, 3315   }, {117, 3220   }, {118, 3127   }, {119, 3038   }, {120, 2951   }, {121, 2868   },
	{122, 2787   }, {123, 2709   }, {124, 2633   }, {125, 2560   }
};

static __s16 oplus_ch_thermistor_conver_temp(__s32 res)
{
	int i = 0;
	int asize = 0;
	__s32 res1 = 0, res2 = 0;
	__s32 tap_value = -2000, tmp1 = 0, tmp2 = 0;

	asize = (sizeof(charger_ic_temp_table) / sizeof(struct chtemperature));

	if (res >= charger_ic_temp_table[0].temperature_r) {
		tap_value = -400;	/* min */
	} else if (res <= charger_ic_temp_table[asize - 1].temperature_r) {
		tap_value = 1250;	/* max */
	} else {
		res1 = charger_ic_temp_table[0].temperature_r;
		tmp1 = charger_ic_temp_table[0].bts_temp;

		for (i = 0; i < asize; i++) {
			if (res >= charger_ic_temp_table[i].temperature_r) {
				res2 = charger_ic_temp_table[i].temperature_r;
				tmp2 = charger_ic_temp_table[i].bts_temp;
				break;
			}
			res1 = charger_ic_temp_table[i].temperature_r;
			tmp1 = charger_ic_temp_table[i].bts_temp;
		}

		tap_value = (((res - res2) * tmp1) + ((res1 - res) * tmp2)) * 10 / (res1 - res2);
	}

	return tap_value;
}

static __s16 oplus_ts_ch_volt_to_temp(__u32 dwvolt)
{
	__s32 tres;
	__u64 dwvcrich = 0;
	__s32 chg_tmp = -100;
	__u64 dwvcrich2 = 0;
	const int g_tap_over_critical_low = 4251000;
	const int g_rap_pull_up_r = 100000; /* 100K, pull up resister */
	const int g_rap_pull_up_voltage = 1840;
	dwvcrich = ((__u64)g_tap_over_critical_low * (__u64)g_rap_pull_up_voltage);
	dwvcrich2 = (g_tap_over_critical_low + g_rap_pull_up_r);
	do_div(dwvcrich, dwvcrich2);

	if (dwvolt > ((__u32)dwvcrich)) {
		tres = g_tap_over_critical_low;
	} else {
		tres = (g_rap_pull_up_r * dwvolt) / (g_rap_pull_up_voltage - dwvolt);
	}

	/* convert register to temperature */
	chg_tmp = oplus_ch_thermistor_conver_temp(tres);

	return chg_tmp;
}

int oplus_get_chargeric_temp(void)
{
	int val = 0;
	int ret = 0, output;

	if (pinfo && pinfo->chargeric_temp_chan) {
		ret = iio_read_channel_processed(pinfo->chargeric_temp_chan, &val);
		if (ret< 0) {
			chg_err("read usb_temp_v_r_chan volt failed, rc=%d\n", ret);
			return ret;
		}
	}

	if (val <= 0) {
		val = CHARGER_25C_VOLT;
	}
	ret = val;
	output = oplus_ts_ch_volt_to_temp(ret);
	if (pinfo->support_ntc_01c_precision) {
		pinfo->chargeric_temp = output;
	} else {
		pinfo->chargeric_temp = output / 10;
	}

	return pinfo->chargeric_temp;
}

int oplus_mt6360_get_tchg(int *tchg_min,	int *tchg_max)
{
	if (pinfo != NULL) {
		oplus_get_chargeric_temp();
		*tchg_min = pinfo->chargeric_temp;
		*tchg_max = pinfo->chargeric_temp;
		return pinfo->chargeric_temp;
	}

	return -EBUSY;
}

bool oplus_tchg_01c_precision(void)
{
	if (!pinfo) {
		printk(KERN_ERR "[OPLUS_CHG][%s]: charger_data not ready!\n", __func__);
		return false;
	}
	return pinfo->support_ntc_01c_precision;
}
EXPORT_SYMBOL(oplus_tchg_01c_precision);
#endif


/* ---- v1-role compat providers ----
 * With OPLUS_CHG_V2, drivers/power/oplus/v1/ is unbuilt, but the MTK
 * charger framework (switch/linear/PE/type_det), the MT6360 PMIC driver,
 * and board drivers (camera, thermal, extcon) still call the entry points
 * the v1 MT6893 battery driver used to provide. They are provided here.
 * Grades of fidelity:
 *  (a) v2-owned path: routed to the real v2 mechanism;
 *  (b) dormant path: was already dormant in v1 builds (no stock
 *      charger_manager exists on this tree — stock mtk_charger.o is
 *      excluded by the OPLUS branch), so NULL-safe defaults preserve
 *      v1 behavior exactly;
 *  (c) stateless v1 bookkeeping: safe no-ops (v2 derives the state live).
 */
struct charger_manager;
struct charger_consumer;

void sc_select_charging_current(struct charger_manager *info,
				struct charger_data *pdata)
{
	/* (b) PE switch-charging has no manager/object on this tree. */
	(void)info;
	(void)pdata;
}

int mtk_get_dynamic_cv(struct charger_manager *info, unsigned int *cv)
{
	/* (b) PE switch-charging dormant; report the design CV. */
	(void)info;
	if (cv)
		*cv = BATTERY_CV;
	return 0;
}

bool mtk_is_TA_support_pd_pps(struct charger_manager *info)
{
	/* (b) No PE4/PE5 algorithm objects -> no PPS TA. */
	(void)info;
	return false;
}

bool is_dual_charger_supported(struct charger_manager *info)
{
	/* Single charger on Cupida (chg2 never bound). */
	(void)info;
	if (!pinfo)
		return false;
	return !!pinfo->chg2_dev;
}

bool is_typec_adapter(struct charger_manager *info)
{
	/* (b) Type-C-only SNK special path unused; PD covers it. */
	(void)info;
	return false;
}

int charger_enable_vbus_ovp(struct charger_manager *info, bool enable)
{
	/* v1 body, on our own data (struct layout is ours). */
	u32 sw_ovp;

	(void)info;
	if (!pinfo)
		return -ENODEV;
	if (enable)
		sw_ovp = pinfo->data.max_charger_voltage_setting;
	else
		sw_ovp = 15000000;
	pinfo->data.max_charger_voltage = sw_ovp;

	chr_err("[%s] en:%d ovp:%d\n", __func__, enable, sw_ovp);
	return 0;
}

int charger_manager_enable_chg_type_det(struct charger_consumer *consumer,
					bool en)
{
	struct charger_device *chg_dev = NULL;

	/* (a/b) No manager instance; this HAL owns the only charger. */
	(void)consumer;
	if (!pinfo || !pinfo->chg1_dev)
		return -ENODEV;
	chg_dev = pinfo->chg1_dev;

	chr_err("%s: chg1 is doing bc12\n", __func__);
	return charger_dev_enable_chg_type_det(chg_dev, en);
}

int charger_manager_notifier(struct charger_manager *info, int event)
{
	if (!info)
		return 0;
	/* struct charger_manager is opaque here; the single manager on this
	 * tree is this HAL's struct mtk_charger (see pinfo). */
	return mtk_charger_notifier((struct mtk_charger *)info, event);
}

void mtk_charger_int_handler(void)
{
	/* Trimmed v1 body: wake our charger thread on plug transitions.
	 * Step-charging/PE-notification belonged to the stock manager. */
	chr_err("%s\n", __func__);
	if (!pinfo)
		return;
	if (mt_get_charger_type() != CHARGER_UNKNOWN)
		oplus_wake_up_charger(pinfo);
}

int charger_manager_enable_power_path(struct charger_consumer *consumer,
				      int idx, bool en)
{
	/* Matches the old-gen OPLUS_FEATURE_CHG_BASIC behavior (return 0);
	 * power-path policy is owned by the TCPC/v2 layers here. */
	(void)consumer;
	(void)idx;
	(void)en;
	return 0;
}

void oplus_gauge_set_event(int event)
{
	/* (c) Fuel-gauge plug notification; the bq27541 v2 HAL polls via
	 * MMS, so there is no consumer for this event. */
	chr_err("[%s] notify event = %d (no-op)\n", __func__, event);
}

int oplus_get_chg_unwakelock(void)
{
	/* (c) v1 flag defaulted to 0 (always wakelock-managed). */
	return 0;
}

void oplus_set_hvdcp_flag_clear(void)
{
	/* (c) No HVDCP state flag exists in v2 (type is queried live). */
}

void oplus_chg_check_break(int vbus_rising)
{
	/* (a) v2 break detection lives in the track layer. */
	oplus_chg_track_check_wired_charging_break(vbus_rising);
}

void oplus_chg_clear_chargerid_info(void)
{
	/* (c) No v1-chip chargerid state in v2. */
}

void oplus_chg_set_charger_type_unknown(void)
{
	/* (c) v2 derives charger type live from the PMIC; no stale state. */
}

void oplus_chg_set_chargerid_switch_val(int value)
{
	/* Charger-ID switch (pio 192; 1 = rxtx, 0 = dpdm). v2 leaves the
	 * switch alone during operation; the IRQ unplug path asks for dpdm. */
	if (!pinfo || !gpio_is_valid(pinfo->chargerid_switch_gpio))
		return;
	if (value != 0 && value != 1)
		return;
	gpio_direction_output(pinfo->chargerid_switch_gpio, value);
}

int oplus_vooc_get_adapter_update_status(void)
{
	/* (c) No adapter-update tracking on this path (v2 FW updates run
	 * through the ASIC HAL); report NONE so IRQ takes normal handling. */
	return 0;
}

bool oplus_vooc_get_fastchg_to_normal(void)
{
	/* (c) v2 strategy owns derating internally. */
	return false;
}

bool oplus_vooc_get_fastchg_to_warm(void)
{
	/* (c) v2 strategy owns derating internally. */
	return false;
}

void oplus_vooc_reset_fastchg_after_usbout(void)
{
	/* (a) v2 core owns the usb-out state machine. */
}

void oplus_chg_set_otg_online(bool online)
{
	if (!pinfo)
		return;
	pinfo->otg_online = online;
}

void oplus_chg_track_record_chg_type_info(void)
{
	/* (a) v2 track records the type on CHG_TYPE_CHANGE virq (triggered
	 * by this HAL's PD notifier). */
}

bool is_vooc_support_single_batt_svooc(void)
{
	/* Live DT sets qcom,support-single-batt-svooc (single-cell SVOOC). */
	return true;
}

static int oplus_chg_camera_on;
void oplus_chg_set_camera_on(int val)
{
	/* (c) Sinked for future thermal policy; no v2 consumer yet. */
	oplus_chg_camera_on = val;
}

static bool oplus_chg_camera_status;
void oplus_chg_set_camera_status(bool val)
{
	/* (c) Sinked for future thermal policy; no v2 consumer yet. */
	oplus_chg_camera_status = val;
}

bool oplus_otgctl_by_buckboost(void)
{
	/* Live vbatt_num == 1 (v1 semantic: buckboost controls OTG only on
	 * dual-cell). OTG on this board goes through the MT6360 boost. */
	return false;
}

void oplus_otg_enable_by_buckboost(void)
{
	/* (a) Route through our own buck funcs (MT6360 boost). */
	if (!pinfo || !pinfo->ic_dev)
		return;
	oplus_chg_ic_func(pinfo->ic_dev, OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
			  true);
	oplus_chg_ic_func(pinfo->ic_dev, OPLUS_IC_FUNC_OTG_BOOST_ENABLE, true);
}

void oplus_otg_disable_by_buckboost(void)
{
	/* (a) Route through our own buck funcs (MT6360 boost). */
	if (!pinfo || !pinfo->ic_dev)
		return;
	oplus_chg_ic_func(pinfo->ic_dev, OPLUS_IC_FUNC_OTG_BOOST_ENABLE,
			  false);
	oplus_chg_ic_func(pinfo->ic_dev, OPLUS_IC_FUNC_BUCK_OUTPUT_SUSPEND,
			  false);
}

void vooc_enable_cp_for_otg(int en)
{
	/* Single-batt-only path in v1; dual-cell board -> no-op. */
	(void)en;
}

bool oplus_voocphy_get_bidirect_cp_support(void)
{
	/* No VOOCPHY on this port. */
	return false;
}

int oplus_chg_set_dischg_enable(bool en)
{
	/* USB-temp discharge path (pio 51, v1 qcom,dischg-gpio). */
	if (!pinfo || !gpio_is_valid(pinfo->dischg_gpio))
		return -ENODEV;
	gpio_direction_output(pinfo->dischg_gpio, en ? 1 : 0);
	chg_err("set dischg %d\n", en);
	return 0;
}

static int mtk_charger_probe(struct platform_device *pdev)
{
	struct mtk_charger *info = NULL;
	int i;
	char *name = NULL;
	struct netlink_kernel_cfg cfg = {
		.input = chg_nl_data_handler,
	};
#ifdef OPLUS_FEATURE_CHG_BASIC
	struct device_node *node = pdev->dev.of_node;
	struct oplus_chg_ic_cfg ic_cfg = { 0 };
	enum oplus_chg_ic_type ic_type;
	int ic_index;
	int rc = 0;
#endif

	chr_err("%s: 111 starts\n", __func__);

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (!tcpc_dev_get_by_name("type_c_port0")) {
		chr_err("%s get tcpc device type_c_port0 fail\n", __func__);
		return -EPROBE_DEFER;
	}

#endif /* OPLUS_FEATURE_CHG_BASIC */

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	platform_set_drvdata(pdev, info);
	info->pdev = pdev;

	mtk_charger_parse_dt(info, &pdev->dev);

#ifdef OPLUS_FEATURE_CHG_BASIC
	pinfo = info;

	info->chg1_dev = get_charger_by_name("primary_chg");
	if (info->chg1_dev) {
		chg_err("found primary charger [%s]\n",
			info->chg1_dev->props.alias_name);
	} else {
		chg_err("can't find primary charger!\n");
	}
#endif

	mutex_init(&info->cable_out_lock);
	mutex_init(&info->charger_lock);
	mutex_init(&info->pd_lock);

	name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s",
		"charger suspend wakelock");
	info->charger_wakelock =
		wakeup_source_register(NULL, name);
	spin_lock_init(&info->slock);

	init_waitqueue_head(&info->wait_que);
	info->polling_interval = CHARGING_INTERVAL;
	mtk_charger_init_timer(info);
#ifdef CONFIG_PM
	info->pm_notifier.notifier_call = charger_pm_event;
#endif /* CONFIG_PM */
	srcu_init_notifier_head(&info->evt_nh);
	mtk_charger_setup_files(pdev);
	mtk_charger_get_atm_mode(info);

	for (i = 0; i < CHGS_SETTING_MAX; i++) {
		info->chg_data[i].thermal_charging_current_limit = -1;
		info->chg_data[i].thermal_input_current_limit = -1;
		info->chg_data[i].input_current_limit_by_aicl = -1;
	}
	info->enable_hv_charging = true;

	info->psy_desc1.name = "mtk-master-charger";
	/* healthd maps ONLINE on a supply to BATTERY_PLUGGED_* purely by
	 * SUPPLY_TYPE. With UNKNOWN it reported "AC powered: false" even
	 * while ONLINE was already 1, so plugged stayed 0 and the status-bar
	 * bolt never appeared. A wall brick / USB supply is a Mains source as
	 * far as the framework is concerned. */
	info->psy_desc1.type = POWER_SUPPLY_TYPE_MAINS;
	info->psy_desc1.properties = charger_psy_properties;
	info->psy_desc1.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc1.get_property = psy_charger_get_property;
	info->psy_desc1.set_property = psy_charger_set_property;
	info->psy_desc1.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_desc1.external_power_changed =
		mtk_charger_external_power_changed;
	info->psy_cfg1.drv_data = info;
	info->psy1 = power_supply_register(&pdev->dev, &info->psy_desc1,
			&info->psy_cfg1);

	info->chg_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"charger");
	if (IS_ERR_OR_NULL(info->chg_psy))
		chr_err("%s: devm power fail to get chg_psy\n", __func__);

	info->bat_psy = devm_power_supply_get_by_phandle(&pdev->dev,
		"gauge");
	if (IS_ERR_OR_NULL(info->bat_psy))
		chr_err("%s: devm power fail to get bat_psy\n", __func__);

	if (IS_ERR(info->psy1))
		chr_err("register psy1 fail:%ld\n",
			PTR_ERR(info->psy1));

	info->psy_desc2.name = "mtk-slave-charger";
	info->psy_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_desc2.properties = charger_psy_properties;
	info->psy_desc2.num_properties = ARRAY_SIZE(charger_psy_properties);
	info->psy_desc2.get_property = psy_charger_get_property;
	info->psy_desc2.set_property = psy_charger_set_property;
	info->psy_desc2.property_is_writeable =
			psy_charger_property_is_writeable;
	info->psy_cfg2.drv_data = info;
	info->psy2 = power_supply_register(&pdev->dev, &info->psy_desc2,
			&info->psy_cfg2);

	if (IS_ERR(info->psy2))
		chr_err("register psy2 fail:%ld\n",
			PTR_ERR(info->psy2));

	info->psy_dvchg_desc1.name = "mtk-mst-div-charger";
	info->psy_dvchg_desc1.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_dvchg_desc1.properties = charger_psy_properties;
	info->psy_dvchg_desc1.num_properties =
		ARRAY_SIZE(charger_psy_properties);
	info->psy_dvchg_desc1.get_property = psy_charger_get_property;
	info->psy_dvchg_desc1.set_property = psy_charger_set_property;
	info->psy_dvchg_desc1.property_is_writeable =
		psy_charger_property_is_writeable;
	info->psy_dvchg_cfg1.drv_data = info;
	info->psy_dvchg1 = power_supply_register(&pdev->dev,
						 &info->psy_dvchg_desc1,
						 &info->psy_dvchg_cfg1);
	if (IS_ERR(info->psy_dvchg1))
		chr_err("register psy dvchg1 fail:%ld\n",
			PTR_ERR(info->psy_dvchg1));

	info->psy_dvchg_desc2.name = "mtk-slv-div-charger";
	info->psy_dvchg_desc2.type = POWER_SUPPLY_TYPE_UNKNOWN;
	info->psy_dvchg_desc2.properties = charger_psy_properties;
	info->psy_dvchg_desc2.num_properties =
		ARRAY_SIZE(charger_psy_properties);
	info->psy_dvchg_desc2.get_property = psy_charger_get_property;
	info->psy_dvchg_desc2.set_property = psy_charger_set_property;
	info->psy_dvchg_desc2.property_is_writeable =
		psy_charger_property_is_writeable;
	info->psy_dvchg_cfg2.drv_data = info;
	info->psy_dvchg2 = power_supply_register(&pdev->dev,
						 &info->psy_dvchg_desc2,
						 &info->psy_dvchg_cfg2);
	if (IS_ERR(info->psy_dvchg2))
		chr_err("register psy dvchg2 fail:%ld\n",
			PTR_ERR(info->psy_dvchg2));

	info->log_level = CHRLOG_ERROR_LEVEL;

	info->pd_adapter = get_adapter_by_name("pd_adapter");
	if (!info->pd_adapter)
		chr_err("No pd adapter found\n");
	else {
		info->pd_nb.notifier_call = oplus_pd_notify_call;
		register_adapter_device_notifier(info->pd_adapter,
						 &info->pd_nb);
	}

	info->sc.daemo_nl_sk = netlink_kernel_create(&init_net, NETLINK_CHG, &cfg);

	if (info->sc.daemo_nl_sk == NULL)
		chr_err("sc netlink_kernel_create error id:%d\n", NETLINK_CHG);
	else
		chr_err("sc_netlink_kernel_create success id:%d\n", NETLINK_CHG);
	sc_init(&info->sc);

	info->chg_alg_nb.notifier_call = chg_alg_event;

	info->fast_charging_indicator = 0;
	info->is_charging = false;
	info->safety_timer_cmd = -1;

	kthread_run(charger_routine_thread, info, "charger_thread");

#ifdef OPLUS_FEATURE_CHG_BASIC
	pinfo->tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (!pinfo->tcpc) {
		chr_err("%s get tcpc device type_c_port0 fail\n", __func__);
	} else {
		pinfo->pd_nb.notifier_call = pd_tcp_notifier_call;
		rc = register_tcp_dev_notifier(pinfo->tcpc, &pinfo->pd_nb,
					TCP_NOTIFY_TYPE_ALL);
		if (rc < 0) {
			pr_err("%s: register tcpc notifer fail\n", __func__);
			return -EINVAL;
		}
	}

	pinfo->chargeric_temp_chan = devm_iio_channel_get(&pdev->dev, "auxadc5-chargeric_temp");
	if (IS_ERR(pinfo->chargeric_temp_chan)) {
		chg_err("Couldn't get chargeric_temp_chan...\n");
		pinfo->chargeric_temp_chan = NULL;
	}

	/* compat-provider GPIOs (v1 qcom,* names; oplus,* preferred) */
	pinfo->chargerid_switch_gpio =
		of_get_named_gpio(node, "oplus,chargerid_switch-gpio", 0);
	if (!gpio_is_valid(pinfo->chargerid_switch_gpio))
		pinfo->chargerid_switch_gpio =
			of_get_named_gpio(node, "qcom,chargerid_switch-gpio",
					  0);
	if (gpio_is_valid(pinfo->chargerid_switch_gpio)) {
		rc = devm_gpio_request_one(&pdev->dev,
					   pinfo->chargerid_switch_gpio,
					   GPIOF_OUT_INIT_LOW, "chargerid_switch");
		if (rc < 0) {
			chg_err("request chargerid_switch gpio fail, rc=%d\n",
				rc);
			pinfo->chargerid_switch_gpio = -ENODEV;
		}
	}
	pinfo->dischg_gpio =
		of_get_named_gpio(node, "oplus,dischg-gpio", 0);
	if (!gpio_is_valid(pinfo->dischg_gpio))
		pinfo->dischg_gpio =
			of_get_named_gpio(node, "qcom,dischg-gpio", 0);
	if (gpio_is_valid(pinfo->dischg_gpio)) {
		rc = devm_gpio_request_one(&pdev->dev, pinfo->dischg_gpio,
					   GPIOF_OUT_INIT_LOW, "dischg");
		if (rc < 0) {
			chg_err("request dischg gpio fail, rc=%d\n", rc);
			pinfo->dischg_gpio = -ENODEV;
		}
	}

	rc = of_property_read_u32(node, "oplus,ic_type", &ic_type);
	if (rc < 0) {
		chg_err("can't get ic type, rc=%d\n", rc);
		goto reg_ic_err;
	}
	rc = of_property_read_u32(node, "oplus,ic_index", &ic_index);
	if (rc < 0) {
		chg_err("can't get ic index, rc=%d\n", rc);
		goto reg_ic_err;
	}
	ic_cfg.name = node->name;
	ic_cfg.index = ic_index;
	sprintf(ic_cfg.manu_name, "MTK6893");
	sprintf(ic_cfg.fw_id, "0x00");
	ic_cfg.type = ic_type;
	ic_cfg.get_func = oplus_chg_get_func;
	ic_cfg.virq_data = mtk_chg_virq_table;
	ic_cfg.virq_num = ARRAY_SIZE(mtk_chg_virq_table);
	pinfo->ic_dev = devm_oplus_chg_ic_register(&pinfo->pdev->dev, &ic_cfg);
	if (!pinfo->ic_dev) {
		rc = -ENODEV;
		chg_err("register %s error\n", node->name);
		goto reg_ic_err;
	}

reg_ic_err:
#endif

	return 0;
}

static int mtk_charger_remove(struct platform_device *dev)
{
	return 0;
}

static void mtk_charger_shutdown(struct platform_device *dev)
{
	struct mtk_charger *info = platform_get_drvdata(dev);
	/* NOTE: info->alg[] stays NULL; nothing to stop. */
}

/* Name must NOT be "charger": the stock MTK charger-manager platform
 * driver (.name = "charger") already owns that bus name, so this driver
 * never registers. Distinct name avoids the collision. */
static const struct of_device_id mtk_charger_of_match[] = {
	{.compatible = "mediatek,charger", },
	{},
};

MODULE_DEVICE_TABLE(of, mtk_charger_of_match);

struct platform_device mtk_charger_device = {
	.name = "oplus-chg-mtk6893",
	.id = -1,
};

static struct platform_driver mtk_charger_driver = {
	.probe = mtk_charger_probe,
	.remove = mtk_charger_remove,
	.shutdown = mtk_charger_shutdown,
	.driver = {
		   .name = "oplus-chg-mtk6893",
		   .of_match_table = mtk_charger_of_match,
	},
};

static int __init mtk_charger_init(void)
{
	return platform_driver_register(&mtk_charger_driver);
}
#ifndef OPLUS_FEATURE_CHG_BASIC
module_init(mtk_charger_init);
#endif

static void __exit mtk_charger_exit(void)
{
	platform_driver_unregister(&mtk_charger_driver);
}
#ifndef OPLUS_FEATURE_CHG_BASIC
module_exit(mtk_charger_exit);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
oplus_chg_module_register(mtk_charger);
#endif

MODULE_AUTHOR("wy.chuang <wy.chuang@mediatek.com>");
MODULE_DESCRIPTION("MTK Charger Driver");
MODULE_LICENSE("GPL");
