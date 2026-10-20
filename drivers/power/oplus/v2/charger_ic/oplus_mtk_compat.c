// SPDX-License-Identifier: GPL-2.0-only
/*
 * MTK charger-manager compatibility shims for the OPLUS v2 charger stack.
 *
 * The stock MTK charger manager (mtk_charger.c) is excluded from this
 * tree, but several stock charger sources that ARE built call into it:
 *
 *   mtk_intf.c            -> _wake_up_charger(struct charger_manager *)
 *   mtk_switch_charging.c -> _wake_up_charger(struct charger_manager *)
 *   mtk_pd_adapter.c      -> notify_adapter_event(enum adapter_type,
 *                                                  enum adapter_event,
 *                                                  void *)
 *
 * The v2 HAL has its own equivalents, but with v2 types:
 *   struct mtk_charger  instead of struct charger_manager
 *   a notifier callback instead of the adapter-event entry point
 *
 * If the v2 symbols are exported under the stock names, the stock callers
 * land on functions with the wrong signature. That is not a link error and
 * not a compile error -- it is a silent memory corruption at runtime. It
 * already cost one boot loop: mtk_pd_adapter.c passed MTK_PD_ADAPTER (== 0)
 * where the v2 notifier callback expected a struct notifier_block *, the
 * container_of() produced a bogus pinfo and mutex_lock(&pinfo->pd_lock)
 * data-aborted the kernel (ramoops: "Internal error: Oops: 96000006",
 * pc mutex_lock+0x18, Workqueue: type_c_port0 tcp_notify_func).
 *
 * So: keep the stock names here, with the stock signatures, and forward to
 * the v2 implementations. This is the only translation unit that can
 * include mtk_charger_intf.h, because that header redefines charger
 * macros/enums that the v2 HAL already has.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/pm.h>
#include <linux/wait.h>

#include "mtk_charger_intf.h"

struct mtk_charger;

extern void oplus_mtk6893_notify_adapter_event(unsigned long evt, void *val);

/* Stock signature: struct charger_manager *.
 *
 * There is no live stock charger manager on this tree, so this only has to
 * be safe, not functional: the stock callers either pass NULL or a manager
 * that nothing ever drives. */
void _wake_up_charger(struct charger_manager *info)
{
	unsigned long flags;

	if (info == NULL)
		return;

	spin_lock_irqsave(&info->slock, flags);
	if (info->charger_wakelock && !info->charger_wakelock->active)
		__pm_stay_awake(info->charger_wakelock);
	spin_unlock_irqrestore(&info->slock, flags);
	info->charger_thread_timeout = true;
	wake_up_interruptible(&info->wait_que);
}
EXPORT_SYMBOL(_wake_up_charger);

/* Stock signature: (enum adapter_type, enum adapter_event, void *). */
void notify_adapter_event(enum adapter_type type, enum adapter_event evt,
			void *val)
{
	if (type != MTK_PD_ADAPTER)
		return;

	oplus_mtk6893_notify_adapter_event((unsigned long)evt, val);
}
EXPORT_SYMBOL(notify_adapter_event);

/* The charger thermal zone (mtk_ts_charger_v1.c) polls this at ~2Hz. With
 * only the weak stub from that file it fails with -ENODEV and prints
 * "charger_manager_get_charger_temperature not found." every time, which
 * floods the kernel ring buffer and hides every other message. Implement it
 * against the v2 charger IC temperature so the zone gets a real reading. */
extern int oplus_get_chargeric_temp(void);

int charger_manager_get_charger_temperature(struct charger_consumer *consumer,
	int idx, int *tchg_min, int *tchg_max)
{
	int t;

	if (!consumer) {
		*tchg_min = -127;
		*tchg_max = -127;
		return -ENODEV;
	}

	t = oplus_get_chargeric_temp();

	/* Sanity check: the zone multiplies this by 1000 and feeds it to the
	 * thermal framework, so a bogus value could throttle the phone. */
	if (t <= -40 || t >= 150) {
		*tchg_min = -127;
		*tchg_max = -127;
		return -ENODEV;
	}

	*tchg_min = t;
	*tchg_max = t;

	return 0;
}
EXPORT_SYMBOL(charger_manager_get_charger_temperature);
