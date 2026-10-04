#!/usr/bin/env bash
# Runtime A/B for page_cluster, plus the rollback/verification recipe.
#
# The kernel change in this branch is a build-time default, but
# /proc/sys/vm/page-cluster is a plain sysctl, so the change can be
# measured on the CURRENTLY FLASHED kernel before ever reflashing.
# That is the intended workflow: prove the effect at runtime first, then
# flash. It also means a bad image is never the first thing tested.
#
# Usage:
#   pagecluster-ab.sh status
#   pagecluster-ab.sh set 0 | set 3
#   pagecluster-ab.sh restore
#   pagecluster-ab.sh verify-susfs

set -u

SYSCTL=/proc/sys/vm/page-cluster
# The stock-OPPO value this branch removes, and the upstream value it
# restores (megs >= 16 -> 3 on this 7.3 GB device).
OPUS_DEFAULT=0
UPSTREAM_DEFAULT=3

sh_() { adb shell su -c "$1" 2>/dev/null; }
get() { sh_ "cat $SYSCTL 2>/dev/null"; }

case "${1:-status}" in
status)
	echo "current page-cluster : $(get)"
	echo "branch baseline (OPUS)  : $OPUS_DEFAULT"
	echo "branch change   (upstream): $UPSTREAM_DEFAULT"
	echo
	echo "swap read-ahead is effectively DISABLED while page-cluster == 0:"
	echo "  swapin_nr_pages() -> 1, so swap_cluster_readahead() and the"
	echo "  VMA swap readahead window are both no-ops."
	;;

set)
	want="${2:-}"
	case "$want" in
	0|3) ;;
	*) echo "refusing: only 0 or 3 are meaningful here (got '$want')" >&2; exit 2 ;;
	esac
	cur="$(get)"
	if [ "$cur" = "$want" ]; then
		echo "already $want, nothing to do"
		exit 0
	fi
	echo "setting page-cluster $cur -> $want"
	sh_ "echo $want > $SYSCTL" || { echo "write failed" >&2; exit 1; }
	echo "now: $(get)"
	[ "$want" = 0 ] && \
		echo "NOTE: 0 is the stock/OPUS behaviour this branch removes. Use it as the A side." || \
		echo "NOTE: $want is the branch's new default. Use it as the B side."
	;;

restore)
	echo "restoring stock behaviour for this device (page-cluster = $OPUS_DEFAULT)"
	sh_ "echo $OPUS_DEFAULT > $SYSCTL"
	echo "now: $(get)"
	;;

verify-susfs)
	# Sanity check that the SUSFS work in this branch is still wired up.
	# Read-only; does not exercise the hooks.
	echo "SUSFS markers present in the built kernel:"
	sh_ 'ls /proc/sys/kernel/*susfs* /sys/module/*susfs* 2>/dev/null'
	sh_ 'grep -icE "susfs" /proc/kallsyms 2>/dev/null | sed "s/^/kallsyms matches: /"'
	sh_ 'zcat /proc/config.gz 2>/dev/null | grep -iE "SUSFS|SU_KSU" | head'
	;;

*)
	echo "usage: $0 {status|set <0|3>|restore|verify-susfs}" >&2
	exit 2
	;;
esac