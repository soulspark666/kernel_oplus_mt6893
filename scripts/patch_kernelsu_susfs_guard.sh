#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
#
# Remove the upstream stop-ship guard from the KernelSU submodule.
#
# KernelSU/kernel/tools/susfs_compat.mk ends with
#
#	$(info -- $(REPO_NAME): susfs has NOT update with latest KernelSU source)
#	$(info -- $(REPO_NAME): Please wait simonpunk update)
#	$(error Please wait susfs update.)
#
# which aborts the build unconditionally. It is the maintainers' way of
# saying "the newest KernelSU expects a susfs that has not been released
# yet". This tree now carries susfs v2.3.00 (the release that adds the
# inline hooks the newest KernelSU uses), so the guard's premise no
# longer holds.
#
# The submodule itself is left pristine -- the pointer stays on an
# upstream commit so that "git submodule update" keeps working -- the
# guard is stripped from the working tree right before the build.
#
# Usage: scripts/patch_kernelsu_susfs_guard.sh [kernel-tree]

set -eu

TREE="${1:-$(cd "$(dirname "$0")/.." && pwd)}"
GUARD_FILE="$TREE/KernelSU/kernel/tools/susfs_compat.mk"

[ -f "$GUARD_FILE" ] || {
	echo "patch_kernelsu_susfs_guard: $GUARD_FILE not found" >&2
	exit 1
}

if ! grep -q 'Please wait susfs update' "$GUARD_FILE"; then
	echo "patch_kernelsu_susfs_guard: guard already gone upstream, nothing to do"
	exit 0
fi

tmp="$(mktemp)"
trap 'rm -f "$tmp"' EXIT

# Drop the three trailing lines of the guard.
sed '/Please wait susfs update/d' "$GUARD_FILE" \
	| sed '/susfs has NOT update with latest KernelSU source/d' \
	| sed '/^\$(error Please wait susfs update\.)$/d' > "$tmp"

if grep -q 'Please wait susfs update' "$tmp"; then
	echo "patch_kernelsu_susfs_guard: failed to remove the guard" >&2
	exit 1
fi

cat "$tmp" > "$GUARD_FILE"
echo "patch_kernelsu_susfs_guard: removed the stale susfs stop-ship guard"
