#!/usr/bin/env bash
# Snapshot every thermal / DVFS / reclaim / radio signal named in
# KERNEL_OPTIMIZATION_PROMPT.md section 9.
#
# Design rules (section 9.4 + C-7):
#   - Nothing here may assume a node exists. Every read is existence
#     checked and every failure is reported, never fatal.
#   - Nothing here writes to the device. It is a pure observer, so it
#     is safe to run at any point during a soak.
#   - One logical change per series: pass a label so every snapshot in
#     a series carries the same tag.
#
# Usage: snapshot.sh <label>
# Writes: out/snapshots/<label>-<UTC timestamp>.txt

set -u

LABEL="${1:-unnamed}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUTDIR="${OUTDIR:-$HERE/../../out/snapshots}"
mkdir -p "$OUTDIR"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
FILE="$OUTDIR/$LABEL-$STAMP.txt"

# Run a snippet as root on the device.
#
# The snippet is pushed to a file and executed rather than passed inline:
# `adb shell su -c '<multiline>'` mangles nested quotes, $(...) and
# newlines, which silently produced empty output for the topology loop
# during development. Pushing sidesteps all of that.
TMPL="$(mktemp -d)"
REMOTE="/data/local/tmp/longevity-snapshot.$$.sh"
trap 'rm -rf "$TMPL"' EXIT

sh_() {
	printf '%s\n' "$1" >"$TMPL/cmd.sh"
	# NB: no -q; not all adb builds accept it, and a silent push failure
	# here looks exactly like "the device reported nothing".
	adb push "$TMPL/cmd.sh" "$REMOTE" >/dev/null 2>&1 || return 0
	adb shell su -c "sh $REMOTE" 2>/dev/null
}

# Existence check, C-7: never assume a node is present.
have() { [ "$(sh_ "test -e '$1' && echo yes")" = "yes" ]; }

sec() { printf '\n===== %s =====\n' "$1" >>"$FILE"; }

{
printf 'label=%s utc=%s\n' "$LABEL" "$STAMP"
printf 'banner=%s\n' "$(sh_ 'cat /proc/version')"
} >>"$FILE"

sec "IDENTITY (section 0 re-verify)"
for p in ro.board.platform ro.hardware ro.soc.model ro.product.model ro.product.manufacturer; do
	printf '%-28s %s\n' "$p" "$(sh_ "getprop $p")" >>"$FILE"
done

sec "CPU TOPOLOGY (section 3; policy numbers are non-contiguous 0/4/7)"
sh_ 'for p in /sys/devices/system/cpu/cpufreq/policy*; do
	printf "%s cpus=%s gov=%s cur=%s min=%s max=%s\n" \
		"$p" "$(cat $p/related_cpus)" "$(cat $p/scaling_governor 2>/dev/null)" \
		"$(cat $p/scaling_cur_freq 2>/dev/null)" \
		"$(cat $p/scaling_min_freq 2>/dev/null)" \
		"$(cat $p/scaling_max_freq 2>/dev/null)"
done' >>"$FILE"

sec "SCHEDULER PLACEMENT (T-2.1 / T-2.2)"
printf 'eas/info:\n' >>"$FILE"
have /sys/devices/system/cpu/eas/info && sh_ 'cat /sys/devices/system/cpu/eas/info' >>"$FILE"
printf 'core_ctl isolation: %s\n' \
	"$(sh_ 'cat /proc/perfmgr/boost_ctrl/eas_ctrl/sched_isolated 2>/dev/null')" >>"$FILE"
printf 'cpuset top-app  : %s\n' "$(sh_ 'cat /dev/cpuset/top-app/cpus 2>/dev/null')" >>"$FILE"
printf 'cpuset foreground: %s\n' "$(sh_ 'cat /dev/cpuset/foreground/cpus 2>/dev/null')" >>"$FILE"
printf 'cpuset background: %s\n' "$(sh_ 'cat /dev/cpuset/background/cpus 2>/dev/null')" >>"$FILE"

sec "DVFS RESIDENCY (S-3)"
for p in 0 4 7; do
	d="/sys/devices/system/cpu/cpufreq/policy$p/stats"
	have "$d/time_in_state" || continue
	printf 'policy%s time_in_state (kHz jiffies):\n' "$p" >>"$FILE"
	sh_ "cat $d/time_in_state | sort -k2 -nr | head -12" >>"$FILE"
done

sec "DVFS GOVERNOR KNOBS (runtime-reversible, no reflash)"
for p in 0 4 7; do
	for k in target_loads up_rate_limit_us down_rate_limit_us; do
		n="/sys/devices/system/cpu/cpufreq/policy$p/schedutil/$k"
		have "$n" && printf 'policy%s/%-18s %s\n' "$p" "$k" "$(sh_ "cat $n")" >>"$FILE"
	done
done

sec "THERMAL (C-1: observe only, never write)"
# NOTE: dumpsys thermalservice reports mtktsAP twice - once under
# "Cached temperatures:" (frozen) and once under "Current temperatures
# from HAL:" (live). Both are printed so the two are never confused
# again; only the live one may be used for pass/fail.
printf 'CACHED (not for pass/fail):\n' >>"$FILE"
sh_ 'dumpsys thermalservice | sed -n "/Cached temperatures:/,/HAL Ready/p" | grep -E "mType=[0-9]+, mName="' >>"$FILE"
printf '\nLIVE from HAL (this is the one that matters):\n' >>"$FILE"
sh_ 'dumpsys thermalservice | sed -n "/Current temperatures from HAL:/,/Current cooling devices/p"' >>"$FILE"
printf 'thermal status line: %s\n' \
	"$(sh_ 'dumpsys thermalservice | grep -m1 "^Thermal Status:"')" >>"$FILE"
printf 'thermal thresholds:\n' >>"$FILE"
sh_ 'dumpsys thermalservice | grep -E "mName=mtktsAP.*mHotThrottling"' >>"$FILE"

sec "PPM POLICIES (C-1 audit: THERMAL/PWR_THRO must read enabled)"
if have /proc/ppm/policy_status; then
	sh_ 'cat /proc/ppm/policy_status' >>"$FILE"
	sh_ 'cat /proc/ppm/policy/thermal_limit /proc/ppm/policy/thermal_cur_power 2>/dev/null' >>"$FILE"
	sh_ 'cat /proc/ppm/dump_policy_list' >>"$FILE"
else
	printf 'ABSENT: /proc/ppm\n' >>"$FILE"
fi

sec "RECLAIM / SWAP (objective 5.3)"
printf 'page-cluster: %s\n' "$(sh_ 'cat /proc/sys/vm/page-cluster 2>/dev/null')" >>"$FILE"
printf 'swappiness  : %s   direct: %s\n' \
	"$(sh_ 'cat /proc/sys/vm/swappiness 2>/dev/null')" \
	"$(sh_ 'cat /proc/sys/vm/direct_swappiness 2>/dev/null')" >>"$FILE"
have /sys/kernel/mm/lru_gen/enabled \
	&& printf 'lru_gen enabled: %s\n' "$(sh_ 'cat /sys/kernel/mm/lru_gen/enabled')" >>"$FILE" \
	|| printf 'lru_gen: ABSENT (MGLRU not compiled in on this kernel)\n' >>"$FILE"
have /sys/block/zram0/disksize && {
	printf 'zram disksize: %s\n' "$(sh_ 'cat /sys/block/zram0/disksize')" >>"$FILE"
	printf 'zram mm_stat : %s\n' "$(sh_ 'cat /sys/block/zram0/mm_stat')" >>"$FILE"
}
sh_ 'grep -E "^(pgscan_|pgsteal_|pswpin|pswpout|pgmajfault|compact_|oom_kill)" /proc/vmstat' >>"$FILE"
have /proc/pressure/memory && sh_ 'cat /proc/pressure/memory' >>"$FILE"
sh_ 'grep -E "MemTotal|MemAvailable|SwapTotal|SwapFree" /proc/meminfo' >>"$FILE"

sec "RECLAIM SANITY (S-5)"
printf 'oom_kill counter: %s\n' "$(sh_ 'grep ^oom_kill /proc/vmstat')" >>"$FILE"
printf 'cgroup kills   : %s\n' "$(sh_ 'dumpsys deviceidle get deep | grep -i kill 2>/dev/null')" >>"$FILE"

sec "KERNEL ERROR LOG (S-6)"
sh_ 'dmesg | grep -icE "oops|bug:|WARNING:|Call trace|panic"' >/dev/null
printf 'dmesg error-class line count: %s\n' \
	"$(sh_ 'dmesg | grep -cE "oops|bug:|WARNING:|Call trace|panic"')" >>"$FILE"
printf 'oops_count: %s warn_count: %s\n' \
	"$(sh_ 'cat /sys/kernel/oops_count 2>/dev/null')" \
	"$(sh_ 'cat /sys/kernel/warn_count 2>/dev/null')" >>"$FILE"

sec "CRASH REPORTING AUDIT (C-5 must stay intact)"
for k in panic_on_oops panic_on_warn panic; do
	printf '%-18s %s\n' "$k" "$(sh_ "cat /proc/sys/kernel/$k 2>/dev/null")" >>"$FILE"
done

sec "NR BEARER (S-2 primary success signal)"
sh_ 'dumpsys telephony.registry | grep -oE "mBands = \[[0-9]+\]|mCellBandwidths=\[[0-9]*\]"' >>"$FILE"
sh_ 'dumpsys connectivity | grep -iE "MOBILE\[" | head -5' >>"$FILE"

printf '\nwrote %s\n' "$FILE"