#!/usr/bin/env python3
"""WP-A done-gate (§3.5 of V2_CHARGER_PORT_PLAN.md): check the MT6893 v2
charger DTSI provides every oplus,* property the v2 framework reads and that
no phandle is left unresolved in the compiled overlay.

Usage (from kernel root, on branch port/v2-mt6893-dts):
  1. Build the v2-only overlay:
       make O=out18 ARCH=arm64 LLVM=1 ... mediatek/oplus6893_21127.dtbo
     (with the _v2 include swapped in — see oplus6893_21127.dts comment)
  2. Decompile: out18/scripts/dtc/dtc -I dtb -O dts \\
       out18/arch/arm64/boot/dts/mediatek/oplus6893_21127.dtbo -o /tmp/v2.dts
  3. Run: python3 arch/arm64/boot/dts/mediatek/check_v2_charge_dts.py /tmp/v2.dts

Exit 0 = gate passes. Exit 1 = missing props / unresolved phandles.
"""
import re
import sys

# Correctly absent on MT6893 (other SoCs / no hardware / template agrees):
#   hal_mt6375, hal_sm8350 (other PMICs), monitor (template has no node
#   either), display_panel (optional panel notifier), tcpc_name (no v2
#   reader), voocphy_support (no v2 reader), zy0603_soc_opt (no v2 reader).
OPTIONAL_ABSENT = {
    "asic_id",  # optional: falls back to oplus,asic_id-gpio strapping,
                # which IS wired (pio 56 on production). Either satisfies.
    "display_panel",
    "hal_mt6375",
    "hal_sm8350",
    "monitor",
    "tcpc_name",
    "voocphy_support",
    "zy0603_soc_opt",
    # No production-DT source (drivers fall back safely):
    # - abnormal_adapter_current: support_abnormal_adapter=false
    # - smart_chg_bcc_support: BCC paths skipped
    # - batt_num: served via GAUGE_GET_BATT_NUM func (=1, live vbatt_num)
    "abnormal_adapter_current",
    "smart_chg_bcc_support",
    "batt_num",
}

REQUIRED_47 = (
    "abnormal_adapter_current adc_info_name asic_ic asic_id "
    "batt_capacity_mah batt_num buck_ic buck_ic_connect "
    "buck_ic_current_ratio buck_ic_func_group chg_framework_v2 "
    "display_panel firmware_data gauge_ic gauge_ic_func_group "
    "general_strategy_data general_strategy_name hal_mt6375 hal_sm8350 "
    "ic_index ic_type main_charger mms_gauge mms_wired monitor "
    "smart_charging_screenoff smart_chg_bcc_support "
    "support_ntc_01c_precision support_vooc_by_normal_charger_path "
    "tcpc_name ui_soc_decimal_speedmin usbtemp_batttemp_gap virtual_asic "
    "virtual_buck virtual_gauge vooc vooc_curr_max vooc_data_width "
    "vooc_fw_update_newmethod vooc_ic vooc_project vooc_show_ui_soc_decimal "
    "vooc_strategy_data vooc_strategy_name voocphy_support wired "
    "zy0603_soc_opt"
).split()
assert len(REQUIRED_47) == 47, len(REQUIRED_47)


def main(path):
    dts = open(path).read()
    missing = [
        p
        for p in REQUIRED_47
        if p not in OPTIONAL_ABSENT
        and not re.search(r"\boplus," + re.escape(p) + r"\b", dts)
    ]
    bad_phandles = re.findall(r"<0x0>", dts)
    ok = True
    if missing:
        print("MISSING required oplus,* props:", missing)
        ok = False
    else:
        print(
            "props: %d/47 present (%d correctly absent: %s)"
            % (
                sum(
                    1
                    for p in REQUIRED_47
                    if re.search(r"\boplus," + re.escape(p) + r"\b", dts)
                ),
                len(OPTIONAL_ABSENT),
                sorted(OPTIONAL_ABSENT),
            )
        )
    if bad_phandles:
        print("UNRESOLVED phandles: %d" % len(bad_phandles))
        ok = False
    else:
        print("phandles: 0 unresolved")
    for node in (
        "chg_framework_v2",
        "oplus,virtual_asic",
        "oplus,virtual_buck",
        "oplus,virtual_gauge",
    ):
        if node not in dts:
            print("MISSING node/marker:", node)
            ok = False
    print("GATE:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1]))
