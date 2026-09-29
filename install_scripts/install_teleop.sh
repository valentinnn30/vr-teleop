#!/usr/bin/env bash
# Create .venv_teleop — everything the Quest teleop manager needs.
#
# Works on the robot (Jetson/aarch64) and on a laptop for the off-robot replay
# loop; the manager has the same dependencies either way.
#
# Distilled from the GR00T fork's install_scripts/install_pico.sh, minus
# everything that belongs to other input paths: no XRoboToolkit/PXREARobotSDK
# (PICO), no isaacteleop/CloudXR, no mujoco sim extra, no unitree_sdk2_python
# (the Python DDS bindings are only needed by the sim2sim bridge — the Quest path
# reaches the robot through the C++ deploy binary over ZMQ).
#
# Usage:
#   bash install_scripts/install_teleop.sh
#   GROOT_REPO=/path/to/GR00T-WholeBodyControl bash install_scripts/install_teleop.sh
#
# What it does NOT install, and cannot:
#   * the BrainCo hand service C++ binary (needs unitree_sdk2 + CycloneDDS
#     system-wide; see the note printed at the end)
#   * anything on the GR00T side: the deploy container, the ONNX checkpoints
#     (python download_from_hf.py), or the USE_BRAINCO_HANDS build flag
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

# The external GR00T checkout supplying gear_sonic (robot kinematics for the
# manager's FK calibration). Kept as an env var rather than a pyproject path
# dependency, because PEP 508 URLs cannot interpolate environment variables — so
# a hardcoded file:// path would only ever be right on one machine.
GROOT_REPO="${GROOT_REPO:-$HOME/GR00T-WholeBodyControl}"
VENV="${VENV:-$REPO_ROOT/.venv_teleop}"

ARCH="$(uname -m)"
echo "[OK] Architecture: $ARCH"

if [ ! -f "$GROOT_REPO/gear_sonic/pyproject.toml" ]; then
    cat >&2 <<EOF
ERROR: no gear_sonic package at GROOT_REPO=$GROOT_REPO
       expected: $GROOT_REPO/gear_sonic/pyproject.toml

The teleop manager imports gear_sonic for the robot's kinematic chain (forward
kinematics during calibration). Clone GR00T-WholeBodyControl and either export
GROOT_REPO=<path> or place it at \$HOME/GR00T-WholeBodyControl.

That checkout also needs the BrainCo kinematic patch, so that
instantiate_g1_robot_model(hand_type="brainco") resolves:
  gear_sonic/data/robot_model/model_data/g1/with_brainco/  (URDF/XML; no meshes needed)
EOF
    exit 1
fi
echo "[OK] gear_sonic found at $GROOT_REPO/gear_sonic"

# ── 1. uv ────────────────────────────────────────────────────────────────────
if ! command -v uv &>/dev/null; then
    echo "[INFO] uv not found – installing via the official installer …"
    curl -LsSf https://astral.sh/uv/install.sh | sh
    if [ -f "$HOME/.local/bin/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.local/bin/env"
    elif [ -f "$HOME/.cargo/env" ]; then
        # shellcheck disable=SC1091
        source "$HOME/.cargo/env"
    else
        export PATH="$HOME/.local/bin:$PATH"
    fi
    if ! command -v uv &>/dev/null; then
        echo "[ERROR] uv installed but not on PATH. Add ~/.local/bin and re-run." >&2
        exit 1
    fi
fi
echo "[OK] uv $(uv --version)"

# ── 2. uv-managed Python 3.10 (ships the dev headers some deps build against) ─
echo "[INFO] Installing uv-managed Python 3.10 …"
uv python install 3.10
MANAGED_PY="$(uv python find --no-project 3.10)"
echo "[OK] Using Python: $MANAGED_PY"

# ── 3. Fresh venv ────────────────────────────────────────────────────────────
echo "[INFO] Removing old $(basename "$VENV") (if present) …"
rm -rf "$VENV"
echo "[INFO] Creating $(basename "$VENV") …"
uv venv "$VENV" --python "$MANAGED_PY" --prompt vr_teleop
# shellcheck disable=SC1091
source "$VENV/bin/activate"

# ── 4. gear_sonic[teleop] from the external checkout ─────────────────────────
# Brings pyzmq, msgpack, msgpack-numpy, pin (Pinocchio, for FK) and — on x86_64
# only — pyvista, on top of numpy/scipy/torch.
echo "[INFO] Installing gear_sonic[teleop] from $GROOT_REPO …"
uv pip install -e "$GROOT_REPO/gear_sonic[teleop]"

# ── 5. brainco-retargeting (MANO-21 landmarks -> BrainCo motor commands) ─────
BRAINCO_DIR="$REPO_ROOT/third_party/brainco-retargeting"
if [ ! -f "$BRAINCO_DIR/pyproject.toml" ]; then
    echo "[INFO] third_party/brainco-retargeting is empty — initialising submodule …"
    git -C "$REPO_ROOT" submodule update --init third_party/brainco-retargeting
fi
echo "[INFO] Installing brainco-retargeting[live] …"
uv pip install -e "$BRAINCO_DIR[live]"

# ── 6. This repo ─────────────────────────────────────────────────────────────
echo "[INFO] Installing vr-teleop (editable) …"
uv pip install -e "$REPO_ROOT"

# ── 7. Sanity check: the imports the manager actually makes at startup ────────
echo "[INFO] Verifying imports …"
python - <<'PY'
import importlib, sys
missing = []
for mod in ("msgpack", "zmq", "numpy", "scipy", "pinocchio",
            "gear_sonic.data.robot_model", "brainco_retargeting",
            "teleop_manager.quest_manager"):
    try:
        importlib.import_module(mod)
    except Exception as e:  # noqa: BLE001 - report, don't mask
        missing.append(f"  {mod}: {type(e).__name__}: {e}")
if missing:
    print("[ERROR] these imports failed:", *missing, sep="\n", file=sys.stderr)
    sys.exit(1)
print("[OK] all manager imports resolve")
PY

cat <<EOF

══════════════════════════════════════════════════════════════
  Setup complete. Activate with:

    source $(basename "$VENV")/bin/activate

  Still needed on the ROBOT, outside this venv:

  1. BrainCo hand service (C++). Needs unitree_sdk2 and CycloneDDS installed
     system-wide under /usr/local (it links them with a bare link_libraries,
     no find_package), plus libboost-all-dev libyaml-cpp-dev libfmt-dev
     libspdlog-dev:
       cd third_party/brainco_hand_service && mkdir -p build && cd build
       cmake .. && make -j6          # binaries land in ../bin/

  2. On the GR00T side ($GROOT_REPO):
       python download_from_hf.py    # ONNX checkpoints — NOT in git or LFS
       hand_config.hpp must have '#define USE_BRAINCO_HANDS 1', then rebuild

  Then:  ./scripts/launch_robot_side.sh
══════════════════════════════════════════════════════════════
EOF
