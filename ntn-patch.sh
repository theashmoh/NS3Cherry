#!/usr/bin/env bash
# ns-3-NTN patch kit (v2: ns-3.48 + 5G-LENA v5.1).
#
# Carries the NTN work that is not upstream onto a freshly cloned ns-3 + 5G-LENA,
# then runs the NTN test suites against it. Two patches plus the harness folder:
#
#   patches/ns-3-src-ntn.patch   ECEF-consistent geometry and acos guards in src/
#   patches/nr-ntn.patch         the NTN feature set in contrib/nr
#   ntn/                         scripts, README and orbit data, copied into <ns3-dir>/ntn
#
# User flow:
#   git clone --branch ns-3.48 https://gitlab.com/nsnam/ns-3-dev.git ns-3
#   git clone --branch v5.1 https://gitlab.com/cttc-lena/nr.git ns-3/contrib/nr
#   ./ntn-patch.sh apply ns-3
#   ./ntn-patch.sh test  ns-3
#
# Both examples build only when Eigen3 is found (e.g. apt install libeigen3-dev);
# leo-link-regen-RT also needs the Sionna RT Python packages (cppyy, sionna.rt, pybind11).
# test finds a Sionna virtual environment on its own: SIONNA_PYTHON, the active one,
# ./venv or ~/sionna-venv (ntn/SIONNA.md); without one the Sionna suite is skipped.
#
# Usage:
#   ./ntn-patch.sh export               regenerate both patches from the migrated trees
#   ./ntn-patch.sh check  <ns3-dir>     dry-run the apply, change nothing
#   ./ntn-patch.sh apply  <ns3-dir>     apply both patches and copy ntn/
#   ./ntn-patch.sh test   <ns3-dir>     build and run the NTN, kit and touched stock suites
#
# <ns3-dir> is an ns-3 checkout with 5G-LENA in contrib/nr. Bases are recorded in
# patches/BASES; override with NS3_BASE=<rev> NR_BASE=<rev> when the pull moves them.
# Export sources: SRC_NS3 (default ns348/) and SRC_NR (default its contrib/nr).
set -euo pipefail

HERE=$(cd "$(dirname "$0")" && pwd)
SRC_NS3=${SRC_NS3:-$HERE/ns348}
SRC_NR=${SRC_NR:-$SRC_NS3/contrib/nr}
OUT=${OUT:-$HERE/patches}
NS3_BASE=${NS3_BASE:-ns-3.48}
NR_BASE=${NR_BASE:-v5.1}
# The NTN suites run at EXTENSIVE. Then the other nr suites the kit adds or changes, and the
# stock suites of the ns-3 modules src/ patches: wifi-phy-ofdma drives MultiModelSpectrumChannel
# with back-to-back PPDU parts, so it catches a channel change that leaks onto terrestrial links.
NTN_SUITES="nr-ntn-propagation-timing nr-ntn-link-models nr-ntn-scheduler nr-ntn-random-access nr-ntn-timing-advance"
KIT_SUITES="nr-test-koffset nr-test-sched-harq nr-test-sched-reshape nr-test-harq nr-test-l2sm-eesm
  nr-rrc nr-fdd-ul-routing nr-lte-pattern-generation
  nr-rlc-am-transmitter nr-rlc-am-e2e nr-test-rlc-am-status"
STOCK_SUITES="angles uniform-planar-array-test three-gpp-propagation-loss-model
  three-gpp-ntn-propagation-loss-model three-gpp-channel spectrum-interference wifi-phy-ofdma
  multi-model-spectrum-channel leo-mobility
  sionna-rt-channel-model"

die() { echo "ntn-patch: $*" >&2; exit 1; }

# Both trees must be clone roots: --3way needs a repository, and inside some other repository
# git apply resolves paths from that repository's root and silently patches nothing.
require_clone() {
  local top
  top=$(git -C "$1" rev-parse --show-toplevel 2>/dev/null) ||
    die "$1 is not a git clone; clone it as in the user flow"
  [ "$top" = "$(cd "$1" && pwd -P)" ] ||
    die "$1 is inside another git repository ($top); clone it as in the user flow"
}

do_export() {
  # The patches are cut from HEAD, so uncommitted work would silently miss the kit.
  git -C "$SRC_NS3" diff --quiet HEAD -- src || die "uncommitted changes in $SRC_NS3/src; commit first"
  git -C "$SRC_NR" diff --quiet HEAD -- . || die "uncommitted changes in $SRC_NR; commit first"
  mkdir -p "$OUT"
  # src/ only: the ntn/ analysis scripts and the examples ride along with contrib/nr,
  # and nothing outside src/ has to be patched to build and test.
  git -C "$SRC_NS3" diff --binary "$NS3_BASE"..HEAD -- src > "$OUT/ns-3-src-ntn.patch"
  # The MR branches sit on cttc/master (v5.1 + 2 commits that touch only these three
  # files), so exclude them and the patch applies to the v5.1 tag users clone.
  git -C "$SRC_NR" diff --binary "$NR_BASE"..HEAD -- . ':!CHANGES.md' ':!RELEASE_NOTES.md' \
    ':!.gitlab-ci.yml' > "$OUT/nr-ntn.patch"
  {
    echo "# Bases these patches were cut against. Regenerate after every upstream pull."
    echo "generated:   $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "ns3_base:    $NS3_BASE ($(git -C "$SRC_NS3" rev-parse --short "$NS3_BASE^{commit}"))"
    echo "ns3_head:    $(git -C "$SRC_NS3" rev-parse --short HEAD) on $(git -C "$SRC_NS3" branch --show-current)"
    echo "nr_base:     $NR_BASE ($(git -C "$SRC_NR" rev-parse --short "$NR_BASE^{commit}"))"
    echo "nr_head:     $(git -C "$SRC_NR" rev-parse --short HEAD) on $(git -C "$SRC_NR" branch --show-current)"
  } > "$OUT/BASES"
  wc -l "$OUT"/*.patch
  cat "$OUT/BASES"
}

# $1 = tree, $2 = patch, $3 = git apply flags. git apply --3way --check exits 0 even when it
# reports conflicts, so the output decides too.
apply_one() {
  local out rc=0
  out=$(git -C "$1" apply $3 "$2" 2>&1) || rc=1
  [ -z "$out" ] || printf '%s\n' "$out"
  if grep -q 'with conflicts' <<<"$out"; then rc=1; fi
  return $rc
}

# $1 = target dir, $2 = --check or empty
do_apply() {
  local ns3=$1 mode=${2:-}
  [ -d "$ns3/src" ] || die "no src/ in $ns3"
  [ -d "$ns3/contrib/nr" ] || die "no contrib/nr in $ns3 - clone 5G-LENA there first"
  require_clone "$ns3"
  require_clone "$ns3/contrib/nr"
  # check runs the same --3way apply as apply, so an edit that would not match the index
  # fails here rather than halfway through apply.
  local flags="--3way" failed=0
  [ -n "$mode" ] && flags="--3way --check"
  echo "== src/          $flags"
  apply_one "$ns3" "$OUT/ns-3-src-ntn.patch" "$flags" || failed=1
  echo "== contrib/nr    $flags"
  apply_one "$ns3/contrib/nr" "$OUT/nr-ntn.patch" "$flags" || failed=1
  if [ -n "$mode" ]; then
    [ -d "$HERE/ntn" ] || echo "ntn-patch: warning, no ntn/ next to this script" >&2
    [ "$failed" -eq 0 ] || die "a patch does not apply; see the errors above"
    echo "both patches apply cleanly."
    return
  fi
  if [ -d "$HERE/ntn" ]; then
    echo "== ntn/          copy, keeping ntn/results"
    mkdir -p "$ns3/ntn"
    find "$ns3/ntn" -mindepth 1 -maxdepth 1 ! -name results -exec rm -rf {} +
    cp -r "$HERE/ntn/." "$ns3/ntn/"
  else
    echo "ntn-patch: no ntn/ next to this script; harness not copied" >&2
  fi
  [ "$failed" -eq 0 ] ||
    die "conflicts: resolve the <<<< markers and git add the files in the tree that reported them"
  echo "applied."
}

# The first interpreter that can import what ns-3's Sionna support needs.
find_sionna_python() {
  local py
  for py in ${SIONNA_PYTHON:-} ${VIRTUAL_ENV:+$VIRTUAL_ENV/bin/python} "$HERE/venv/bin/python" \
    "$HOME/sionna-venv/bin/python"; do
    [ -x "$py" ] && "$py" -c 'import sionna.rt, pybind11, cppyy' 2>/dev/null && { echo "$py"; return; }
  done
  return 0
}

do_test() {
  local ns3=$1
  local profile="${NS3_PROFILE:-default}"
  local cmake="${CMAKE:-cmake}" log="$ns3/ntn-patch-configure.log"
  local py venv inc lib site
  local sionna=()
  # ./ns3 turns asserts and logging on for these profiles; an explicit CMAKE_BUILD_TYPE skips
  # ns-3's own default, so the tests would run without their NS_ASSERT checks.
  local checks=()
  case "$profile" in default | debug) checks=(-DNS3_ASSERT=ON -DNS3_LOG=ON) ;; esac
  py=$(find_sionna_python)
  if [ -n "$py" ]; then
    # As in ntn/SIONNA.md: ns-3 asks CMake for the interpreter only, so pybind11 gets the
    # headers and library explicitly; the embedded interpreter is the system one, so the
    # runner sees the environment through PYTHONPATH.
    read -r inc lib site < <("$py" -c 'import sysconfig as s; p = s.get_paths()
print(p["include"], "python" + s.get_config_var("LDVERSION"), p["purelib"])')
    venv=$(dirname "$(dirname "$py")")
    sionna=(-DPython3_EXECUTABLE="$py" -DCMAKE_CXX_FLAGS="-isystem $inc"
      -DCMAKE_CXX_STANDARD_LIBRARIES="-l$lib")
    export PYTHONPATH="$site${PYTHONPATH:+:$PYTHONPATH}"
    echo "Sionna RT: $venv"
  fi
  # Prefer the ns3 wrapper; fall back to driving cmake directly where the wrapper is broken.
  # ponytail: Sionna always takes the cmake path, the wrapper has no way to pass it the flags.
  if [ -z "$py" ] &&
    (cd "$ns3" && ./ns3 configure --enable-tests --enable-examples -d "$profile") >"$log" 2>&1; then
    (cd "$ns3" && ./ns3 build nr-test test-runner)
  else
    [ -n "$py" ] || echo "ntn-patch: ./ns3 configure failed (log: $log); trying $cmake directly" >&2
    VIRTUAL_ENV=${py:+$venv} PATH=${py:+$venv/bin:}$PATH \
      "$cmake" -S "$ns3" -B "$ns3/cmake-cache" -DNS3_TESTS=ON -DNS3_EXAMPLES=ON \
      -DCMAKE_BUILD_TYPE="$profile" "${checks[@]}" "${sionna[@]}" >>"$log" 2>&1 ||
      { tail -20 "$log" >&2; die "configure failed; log: $log"; }
    # nr-test alone builds only the module's test library; test-runner is the binary.
    "$cmake" --build "$ns3/cmake-cache" --target nr-test test-runner -j"$(nproc)"
  fi
  # The runner carries the profile as a suffix, except release, which has none. Matching the
  # profile keeps a stale runner from another profile out of the run.
  local suffix="-$profile" runner
  [ "$profile" = release ] && suffix=""
  runner=$(ls "$ns3"/build/utils/ns3.*-test-runner"$suffix" 2>/dev/null | head -1 || true)
  [ -n "$runner" ] || die "no test runner for profile '$profile' in $ns3/build/utils"
  local listed failed=0 skipped=0 suite fullness
  listed=$("$runner" --list)
  for suite in $NTN_SUITES $KIT_SUITES $STOCK_SUITES; do
    if ! grep -qx -- "$suite" <<<"$listed"; then
      echo "skip $suite (not built)"
      skipped=$((skipped + 1))
      continue
    fi
    fullness=QUICK
    case " $NTN_SUITES " in *" $suite "*) fullness=EXTENSIVE ;; esac
    "$runner" --suite="$suite" --fullness="$fullness" --verbose || failed=$((failed + 1))
  done
  echo "suites failed: $failed, skipped (not built): $skipped"
  return $((failed > 0))
}

case "${1:-}" in
  export) do_export ;;
  check)  do_apply "${2:?usage: ntn-patch.sh check <ns3-dir>}" --check ;;
  apply)  do_apply "${2:?usage: ntn-patch.sh apply <ns3-dir>}" ;;
  test)   do_test  "${2:?usage: ntn-patch.sh test <ns3-dir>}" ;;
  *) sed -n '2,25p' "$0"; exit 1 ;;
esac
