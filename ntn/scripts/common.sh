#!/usr/bin/env bash
# Copyright (c) 2026 Jesse Chiu (Kuan-Po), University of Washington
#
# SPDX-License-Identifier: GPL-2.0-only
# Shared environment and safety helpers for the NTN shell runners.

set -euo pipefail

NTN_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
NS3_ROOT="$(cd -- "$NTN_SCRIPT_DIR/../.." && pwd)"
NS3_BUILD_DIR="${NS3_BUILD_DIR:-$NS3_ROOT/build}"
NTN_PYTHON="${PYTHON:-python3}"
NTN_TIME_BIN="${TIME_BIN:-$(type -P time || true)}"
export NTN_PYTHON

cd -- "$NS3_ROOT"
export LD_LIBRARY_PATH="$NS3_BUILD_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

ntn_find_binary() {
    local target="$1"
    local profile
    local candidate
    local example_dir="$NS3_BUILD_DIR/contrib/nr/examples"

    if [[ -n "${NTN_BIN:-}" ]]; then
        [[ -x "$NTN_BIN" ]] || {
            printf 'NTN_BIN is not executable: %s\n' "$NTN_BIN" >&2
            return 1
        }
        printf '%s\n' "$NTN_BIN"
        return
    fi

    for profile in optimized default release debug; do
        candidate="$(find "$example_dir" -maxdepth 1 -type f \
            -name "ns3.*-${target}-${profile}" -perm -u+x -print -quit 2>/dev/null || true)"
        if [[ -n "$candidate" ]]; then
            printf '%s\n' "$candidate"
            return
        fi
    done

    printf 'No built executable found for %s under %s\n' "$target" "$example_dir" >&2
    printf 'Build it with: ./ns3 build %s\n' "$target" >&2
    return 1
}

ntn_find_cmake() {
    local candidate

    if [[ -n "${CMAKE:-}" ]]; then
        candidate="$CMAKE"
        "$candidate" --version >/dev/null
        printf '%s\n' "$candidate"
        return
    fi

    while IFS= read -r candidate; do
        if "$candidate" --version >/dev/null 2>&1; then
            printf '%s\n' "$candidate"
            return
        fi
    done < <(type -aP cmake || true)

    printf 'No working cmake executable found; set CMAKE=/path/to/cmake\n' >&2
    return 1
}

ntn_reset_output_dir() {
    local requested="$1"
    local target

    target="$(realpath -m -- "$requested")"
    case "$target" in
        "$NS3_ROOT/results/"*|"$NS3_ROOT/ntn/results/"*) ;;
        *)
            printf 'Refusing to replace output outside a repository results directory: %s\n' \
                "$target" >&2
            return 1
            ;;
    esac

    rm -rf -- "$target"
    mkdir -p -- "$target"
}

ntn_run_timed() {
    local timing_file="$1"
    shift
    if [[ -n "$NTN_TIME_BIN" ]]; then
        "$NTN_TIME_BIN" -f 'wall %e s' -o "$timing_file" "$@"
    else
        "$@"
    fi
}

ntn_run_timed_verbose() {
    local timing_file="$1"
    shift
    if [[ -n "$NTN_TIME_BIN" ]]; then
        "$NTN_TIME_BIN" -v -o "$timing_file" "$@"
    else
        "$@"
    fi
}

ntn_enable_jemalloc() {
    if [[ -z "${JEMALLOC_LIB:-}" ]]; then
        return
    fi
    [[ -f "$JEMALLOC_LIB" ]] || {
        printf 'JEMALLOC_LIB does not exist: %s\n' "$JEMALLOC_LIB" >&2
        return 1
    }
    export LD_PRELOAD="$JEMALLOC_LIB${LD_PRELOAD:+:$LD_PRELOAD}"
}
