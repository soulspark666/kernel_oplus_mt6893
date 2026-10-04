# Longevity measurement harness

Evidence-gathering for `KERNEL_OPTIMIZATION_PROMPT.md` §9. Every claim about this
device's thermal / DVFS / reclaim behaviour is supposed to be measured, not asserted —
§9 states "should improve" is a failure. These scripts produce the measurements.

All three are **read-only observers** except where they explicitly say otherwise.
They are safe to run at any point during a soak.

Nothing here assumes a node exists. Every read is existence-checked and every failure
is reported rather than fatal, because §6-C7 is explicit that this device lacks
`helio-dvfsrc`, `mtk-dvfsrc-devfreq`, EAS `enable`, MGLRU, and most devfreq nodes.

## `snapshot.sh <label>`

One-shot capture of every signal named in §9: identity re-verification (§0), CPU
topology, `eas/info` placement data, per-policy `time_in_state`, schedutil knobs,
`mtktsAP` + thresholds, PPM policy state, reclaim/swap counters, PSI, `dmesg` error
count, the C-5 crash-reporting audit, and the NR bearer (the §4.2 success signal).

```
./scripts/longevity/snapshot.sh baseline
```

Writes `out/snapshots/<label>-<UTC>.txt`.

## `soak.sh <label> [minutes] [jobs]`

Fixed, reproducible, CPU-only soak (default 20 min, 4 jobs) with a time series rather
than a single number. Pass/fail is judged **only** on `mtktsAP`, per §9.3 — battery
temperature lags and is recorded but never used.

The workload is a fixed number of single-threaded SHA-256 loops. It deliberately does
not touch storage, does not perturb cpusets, and delivers a repeatable amount of work,
so two runs are comparable without depending on a benchmark binary. It is a proxy for
the brief's §4.3 sustained app load, not a benchmark.

Reports max, mean, and **time-in-band** against LIGHT 39 / MODERATE 43 / SEVERE 44,
plus deltas for `pgscan_kswapd`, `pswpin` and `oom_kill` over the run, and an explicit
S-1 verdict.

```
./scripts/longevity/soak.sh baseline 20 4
```

Writes `out/soak/<label>-<UTC>.csv` and a matching `.meta` recording the exact workload
definition and duration, so runs stay comparable (§9.1).

## `pagecluster-ab.sh {status|set 0|set 3|restore|verify-susfs}`

The branch's one behavioural change is `page_cluster`. `/proc/sys/vm/page-cluster` is
a plain sysctl, so **the change can be measured on the currently-flashed kernel before
anything is reflashed.** That is the intended order: prove it at runtime, then flash.
It also means an untested image is never the first thing tried.

- `set 0` → A side: stock/OPUS behaviour this branch removes (read-ahead disabled)
- `set 3` → B side: the branch's new default (upstream, read-ahead clustering restored)
- `restore` → put the device back to stock without reflashing

## Notes for anyone extending this

- Snippets are **pushed** to the device and run with `sh`, not passed inline to
  `adb shell su -c '...'`. Inline multi-line snippets with nested quotes and `$(...)`
  get mangled and silently return empty. This cost real debugging time.
- Do not use `adb push -q`; not every adb build accepts it, and a rejected flag makes
  the push fail *silently*, which looks identical to "the device reported nothing".