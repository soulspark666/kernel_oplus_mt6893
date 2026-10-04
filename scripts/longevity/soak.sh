#!/usr/bin/env bash
# Controlled-load thermal soak (section 9.1 / 9.2).
#
# Requirements this satisfies:
#   9.1 Controlled load   - a fixed, scripted, reproducible workload whose
#                           exact definition and duration are recorded.
#   9.2 Thermal equilibrium- soaks long enough to reach steady state
#                           (default 20 min) and reports a TIME SERIES with
#                           max AND time-in-band, not one number.
#   9.3 mtktsAP is the sensor that matters - battery temp lags and is
#                           recorded but never used for pass/fail.
#   9.4 One logical change per series - the label is stamped into every
#                           sample so a mixed series is impossible to fake.
#
# The workload is intentionally boring and CPU-only: it is a fixed number
# of single-threaded SHA-256 loops pinned to nothing, so it does not
# perturb cpusets, does not touch storage, and produces a repeatable
# amount of work. It is a proxy for "ordinary sustained app load" (the
# brief's section 4.3 finding), not a benchmark.
#
# Usage: soak.sh <label> [minutes] [parallel-jobs]

set -u

LABEL="${1:-soak}"
MINUTES="${2:-20}"
JOBS="${3:-4}"
INTERVAL="${INTERVAL:-30}"

HERE="$(cd "$(dirname "$0")" && pwd)"
OUTDIR="${OUTDIR:-$HERE/../../out/soak}"
mkdir -p "$OUTDIR"
STAMP="$(date -u +%Y%m%dT%H%M%SZ)"
CSV="$OUTDIR/$LABEL-$STAMP.csv"
META="$OUTDIR/$LABEL-$STAMP.meta"
TMPD="$(mktemp -d)"
trap 'rm -rf "$TMPD"' EXIT

# Threshold bands for mtktsAP, from section 9.3 / section 4.1.
LIGHT=39.0
MODERATE=43.0
SEVERE=44.0

adb shell su -c 'mkdir -p /data/local/tmp/soak' >/dev/null 2>&1

# Push snippets rather than passing them inline: `adb shell su -c '<many
# lines with quotes and $(...)>'` mangles them, which silently yields an
# empty sample. No -q; not every adb build accepts it.
TMPL="$(mktemp -d)"
REMOTE="/data/local/tmp/longevity-soak.$$.sh"
trap 'rm -rf "$TMPL"' EXIT

sh_() {
	printf '%s\n' "$1" >"$TMPL/cmd.sh"
	adb push "$TMPL/cmd.sh" "$REMOTE" >/dev/null 2>&1 || return 0
	adb shell su -c "sh $REMOTE" 2>/dev/null
}

# ---- the fixed workload -------------------------------------------------
# $1 = minutes to run. Burns a known, repeatable CPU amount on purpose so
# that two runs are comparable without depending on a benchmark binary.
workload() {
	local mins="$1"
	local end=$(( $(date +%s) + mins * 60 ))
	local n=0
	while [ "$(date +%s)" -lt "$end" ]; do
		# ~1 unit of deliberate, reproducible CPU work per iteration.
		for _ in $(seq 1 20000); do
			echo "$n$RANDOM$RANDOM$RANDOM" | sha256sum >/dev/null
		done
		n=$((n + 1))
	done
	echo "$n"
}

cat >"$META" <<EOF
label=$LABEL
started_utc=$STAMP
duration_minutes=$MINUTES
parallel_jobs=$JOBS
sample_interval_s=$INTERVAL
workload=sha256 loops, $JOBS concurrent single-threaded processes, CPU only
workload_repeatable=yes (same job count + duration => same delivered work)
battery_temperature_used_for_passfail=no (mtktsAP only, per section 9.3)
bands_light=$LIGHT bands_moderate=$MODERATE bands_severe=$SEVERE
S-1 target: mtktsAP stays below $MODERATE C for the whole soak
EOF

echo "soak '$LABEL' starting: ${MINUTES}m, $JOBS jobs, ${INTERVAL}s sampling"
echo "  series -> $CSV"

# Launch the load.
PIDS=""
for j in $(seq 1 "$JOBS"); do
	workload "$MINUTES" >"$TMPD/job$j.done" 2>&1 &
	PIDS="$PIDS $!"
done

# ---- sample loop --------------------------------------------------------
echo "elapsed_s,mtktsAP_C,mtktsbattery_C,thermal_status,p7_khz,p4_khz,p0_khz,isolated,pgscan_kswapd,pswpin,oom_kill" >"$CSV"

start=$(date +%s)
end=$((start + MINUTES * 60))
while [ "$(date +%s)" -lt "$end" ]; do
	el=$(( $(date +%s) - start ))
	# Single remote round trip: gather all samples in one shell so the
	# series stays time-aligned instead of drifting per-field.
	line=$(sh_ '
		# IMPORTANT: dumpsys thermalservice prints mtktsAP TWICE:
		#   under "Cached temperatures:"      <- frozen, NOT live
		#   under "Current temperatures from HAL:"  <- live
		# Taking the first match silently reports a stale value that never
		# moves, which looks exactly like a stable sensor. Parse ONLY the
		# "Current temperatures from HAL:" section.
		ts=$(dumpsys thermalservice | sed -n "/Current temperatures from HAL:/,/Current cooling devices/p")
		parsed=$(echo "$ts" | awk "
			/Temperature[{]/ {
				v=\"\"; t=\"\"; s=\"\"
				if (match(\$0, /mValue=[0-9.]+/)) v=substr(\$0, RSTART+7, RLENGTH-7)
				if (match(\$0, /mType=[0-9]+/))   t=substr(\$0, RSTART+6, RLENGTH-6)
				if (match(\$0, /mStatus=[0-9]+/)) s=substr(\$0, RSTART+8, RLENGTH-8)
				if (t==3 && ap==\"\")   { ap=v; apm=s }
				if (t==2 && batt==\"\") { batt=v }
			}
			END { printf \"%s,%s,%s\", ap, apm, batt }")
		IFS=, read -r apv apm bt <<EOFP
$parsed
EOFP
		[ -z "$apv" ] && apv=NA
		[ -z "$apm" ] && apm=NA
		[ -z "$bt" ]  && bt=NA
		p7=$(cat /sys/devices/system/cpu/cpufreq/policy7/scaling_cur_freq 2>/dev/null)
		p4=$(cat /sys/devices/system/cpu/cpufreq/policy4/scaling_cur_freq 2>/dev/null)
		p0=$(cat /sys/devices/system/cpu/cpufreq/policy0/scaling_cur_freq 2>/dev/null)
		iso=$(cat /proc/perfmgr/boost_ctrl/eas_ctrl/sched_isolated 2>/dev/null | grep -oE "0x[0-9a-f]+")
		vs=$(grep -E "^(pgscan_kswapd|pswpin|oom_kill) " /proc/vmstat | tr "\n" " ")
		echo "$apv,$bt,$apm,${p7:-NA},${p4:-NA},${p0:-NA},${iso:-NA},$vs"
	' | tr -d '\r')

	# normalise the trailing vmstat triple into columns
	scan=$(echo "$line" | grep -oE 'pgscan_kswapd [0-9]+' | awk '{print $2}')
	sin=$(echo  "$line" | grep -oE 'pswpin [0-9]+'      | awk '{print $2}')
	oom=$(echo  "$line" | grep -oE 'oom_kill [0-9]+'     | awk '{print $2}')
	IFS=',' read -r apv bt st p7 p4 p0 iso _ <<<"$line"
	echo "$el,${apv:-NA},${bt:-NA},${st:-NA},${p7:-NA},${p4:-NA},${p0:-NA},${iso:-NA},${scan:-NA},${sin:-NA},${oom:-NA}" >>"$CSV"

	printf '\r  t=%3ds  mtktsAP=%-7s batt=%-7s p7=%-8s iso=%-6s' \
		"$el" "${apv:-NA}" "${bt:-NA}" "${p7:-NA}" "${iso:-NA}"
	sleep "$INTERVAL"
done
echo

wait $PIDS 2>/dev/null

# ---- report: max AND time-in-band, per 9.2 ------------------------------
TOTAL_ITERS=$(cat "$TMPD"/job*.done 2>/dev/null | paste -sd+ | bc 2>/dev/null || echo NA)
{
	echo
	echo "===== SOAK RESULT: $LABEL ====="
	echo "samples            : $(($(wc -l <"$CSV") - 1))"
	echo "delivered work     : $TOTAL_ITERS workload iterations"
	echo "mtktsAP max        : $(awk -F, 'NR>1 && $2!="NA" {if($2+0>m)m=$2+0} END{print m+0}' "$CSV") C"
	echo "mtktsAP mean       : $(awk -F, 'NR>1 && $2!="NA" {s+=$2;n++} END{if(n)printf "%.2f",s/n; else print "NA"}' "$CSV") C"
	echo "samples >= $LIGHT   (LIGHT)     : $(awk -F, -v t=$LIGHT 'NR>1 && $2!="NA" && $2+0>=t' "$CSV" | wc -l)"
	echo "samples >= $MODERATE (MODERATE)  : $(awk -F, -v t=$MODERATE 'NR>1 && $2!="NA" && $2+0>=t' "$CSV" | wc -l)"
	echo "samples >= $SEVERE (SEVERE)   : $(awk -F, -v t=$SEVERE 'NR>1 && $2!="NA" && $2+0>=t' "$CSV" | wc -l)"
	echo "S-1 (mtktsAP < $MODERATE throughout): $(awk -F, -v t=$MODERATE 'NR>1 && $2!="NA" && $2+0>=t' "$CSV" | grep -q . && echo FAIL || echo PASS)"
	echo "p7 (prime) kHz max/mean       : $(awk -F, 'NR>1 && $5!="NA"{if($5+0>m)m=$5+0;s+=$5;n++} END{printf "%d/%.0f", m, (n?s/n:0)}' "$CSV")"
	echo "delta pgscan_kswapd over run   : $(awk -F, 'NR>1 && $9!="NA"{if(f=="")f=$9; l=$9} END{print (f!=""?l-f:"NA")}' "$CSV")"
	echo "delta pswpin over run          : $(awk -F, 'NR>1 && $10!="NA"{if(f=="")f=$10; l=$10} END{print (f!=""?l-f:"NA")}' "$CSV")"
	echo "delta oom_kill over run        : $(awk -F, 'NR>1 && $11!="NA"{if(f=="")f=$11; l=$11} END{print (f!=""?l-f:"NA")}' "$CSV")"
	echo
	echo "series: $CSV"
} | tee -a "$META"

printf '\npost-soak snapshot -> run: %s/snapshot.sh %s-post\n' "$HERE" "$LABEL"