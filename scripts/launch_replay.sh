#!/usr/bin/env bash
# Off-robot replay launcher: iterate on retargeting and latency with no robot.
#
# Two capture formats, two entry points into the same pipeline:
#
#   --bag <file>   Replay a rosbag through the REAL relay container. The bag
#                  supplies the ROS topics the Quest endpoint would have
#                  published, so the relay, its msgpack encoding and the ZMQ hop
#                  are all exercised. This is the one that lets you attribute
#                  latency to the relay rather than assume.
#
#   --npz <file>   Replay an NPZ recording (quest_bridge/record_quest_data.py, or
#                  generate_mock_quest_data.py) straight into the manager. No
#                  container, no Docker — but it enters at the manager's input, so
#                  the relay is bypassed and its timestamps are media-relative.
#
# Both write a per-frame latency CSV by default (see --csv). Neither needs a
# deploy: the manager runs with --no-robot / --replay, which skip the ramp and
# the g1_debug feedback wait.
#
# WHAT THIS CANNOT MEASURE: the Quest -> robot hop. The Unity app sends zero ROS
# header stamps and the ROS-TCP-Endpoint fork restamps on arrival, so every
# timestamp here starts at or after the robot. A small age_recv_ms means the
# robot-side pipeline is healthy and the remaining lag is in the Quest link.
#
#   ./scripts/launch_replay.sh --bag bags/quest_20260929_120000.bag
#   ./scripts/launch_replay.sh --bag ... --loop
#   ./scripts/launch_replay.sh --npz data/quest/traj_001.npz
#   ./scripts/launch_replay.sh kill
#
#   --print / -n   show the exact command(s) without running.
set -euo pipefail

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

TELEOP_VENV="${TELEOP_VENV:-$REPO/.venv_teleop}"
SESSION="${SESSION:-g1_replay}"
ZMQ_PORT="${ZMQ_PORT:-5559}"
# Manager flags. --static-base by default so a replay never implies locomotion;
# no colon in the expansion, so MANAGER_EXTRA="" means "full motion".
MANAGER_EXTRA="${MANAGER_EXTRA---static-base}"

DRYRUN=0
BAG=""
NPZ=""
LOOP=0
CSV=""
MODE=""
while [ $# -gt 0 ]; do
    case "$1" in
        -n|--print) DRYRUN=1 ;;
        --bag) BAG="${2:?--bag needs a path}"; shift ;;
        --npz) NPZ="${2:?--npz needs a path}"; shift ;;
        --csv) CSV="${2:?--csv needs a path}"; shift ;;
        --loop) LOOP=1 ;;
        kill|relay|manager) MODE="$1" ;;
        -h|--help) MODE="help" ;;
        *) echo "ERROR: unknown argument: $1" >&2; MODE="help"; break ;;
    esac
    shift
done

usage() {
    cat >&2 <<EOF
Usage: $0 (--bag <file> [--loop] | --npz <file>) [--csv <path>] [--print]
       $0 kill

  --bag <file>   replay a rosbag through the real relay container (2 panes)
  --npz <file>   replay an NPZ into the manager directly (1 pane, no Docker)
  --loop         with --bag, loop the bag forever
  --csv <path>   latency trace output (default: outputs/latency_<timestamp>.csv)
  kill           kill the '$SESSION' tmux session

Neither mode needs a robot or a Quest. The Quest->robot hop is not measurable
from a recording — see the comment block at the top of this script.
EOF
}

case "$MODE" in
    help) usage; exit 2 ;;
    kill)
        printf '%q ' tmux kill-session -t "$SESSION"; echo
        [ "$DRYRUN" -eq 1 ] || tmux kill-session -t "$SESSION"
        exit 0
        ;;
esac

if [ -n "$BAG" ] && [ -n "$NPZ" ]; then
    echo "ERROR: --bag and --npz are mutually exclusive." >&2; exit 2
fi
if [ -z "$BAG" ] && [ -z "$NPZ" ]; then
    usage; exit 2
fi
if [ "$LOOP" -eq 1 ] && [ -z "$BAG" ]; then
    echo "ERROR: --loop only applies to --bag." >&2; exit 2
fi
if [ -z "$CSV" ]; then
    CSV="$REPO/outputs/latency_$(date +%Y%m%d_%H%M%S).csv"
fi
if [ "$DRYRUN" -eq 0 ]; then
    [ -n "$BAG" ] && [ ! -f "$BAG" ] && { echo "ERROR: no such bag: $BAG" >&2; exit 1; }
    [ -n "$NPZ" ] && [ ! -f "$NPZ" ] && { echo "ERROR: no such NPZ: $NPZ" >&2; exit 1; }
    if [ ! -x "$TELEOP_VENV/bin/python" ]; then
        echo "ERROR: no teleop venv at $TELEOP_VENV" >&2
        echo "       Create it: bash install_scripts/install_teleop.sh" >&2
        exit 1
    fi
    mkdir -p "$(dirname "$CSV")"
fi

# ---- command construction -------------------------------------------------
relay_cmd() {
    local c=(python3 "$REPO/quest_bridge/run_quest_relay.py" --play-bag "$BAG")
    [ "$LOOP" -eq 1 ] && c+=(--loop)
    printf '%q ' "${c[@]}"
}

manager_cmd() {
    local c=("$TELEOP_VENV/bin/python" -m teleop_manager.quest_manager
             --latency-csv "$CSV")
    if [ -n "$NPZ" ]; then
        # --replay implies offline; ReplaySource drives the frames itself.
        c+=(--replay "$NPZ")
    else
        # Live source with no deploy behind it: skip the ramp + feedback wait.
        c+=(--relay-host localhost --relay-port "$ZMQ_PORT" --no-robot)
    fi
    printf '%q ' "${c[@]}"
    # `if` rather than `[ ... ] && ...`: the latter returns non-zero when
    # MANAGER_EXTRA is empty, which under `set -e` aborts the caller.
    if [ -n "$MANAGER_EXTRA" ]; then
        # Unquoted on purpose: zero or more flags.
        # shellcheck disable=SC2086
        printf '%q ' $MANAGER_EXTRA
    fi
}

if [ "$DRYRUN" -eq 1 ]; then
    echo "# latency trace -> $CSV"
    [ -n "$BAG" ] && { echo -n "# [replay:relay]   "; relay_cmd; echo; }
    echo -n "# [replay:manager] "; manager_cmd; echo
    exit 0
fi

command -v tmux >/dev/null 2>&1 || { echo "ERROR: tmux not found" >&2; exit 1; }
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "tmux session '$SESSION' already exists." >&2
    echo "  attach: tmux attach -t $SESSION    kill: $SELF kill" >&2
    exit 1
fi

echo "[launch_replay] latency trace -> $CSV"
if [ -n "$NPZ" ]; then
    # NPZ needs no relay: one pane is the whole pipeline.
    tmux new-session -d -s "$SESSION" -n stack \
        "tmux select-pane -t \"\$TMUX_PANE\" -T manager; ( cd $(printf '%q' "$REPO") && $(manager_cmd) ); printf '\n[manager exited — press Enter] '; read"
else
    tmux new-session -d -s "$SESSION" -n stack \
        "tmux select-pane -t \"\$TMUX_PANE\" -T relay; ( $(relay_cmd) ); printf '\n[relay exited — press Enter] '; read"
    # Let the relay container come up and bind 5559 before the manager connects.
    # (A ZMQ SUB would reconnect anyway; this just keeps the logs readable.)
    tmux split-window -t "$SESSION":stack \
        "tmux select-pane -t \"\$TMUX_PANE\" -T manager; echo 'waiting 8s for the relay container...'; sleep 8; ( cd $(printf '%q' "$REPO") && $(manager_cmd) ); printf '\n[manager exited — press Enter] '; read"
    tmux select-layout -t "$SESSION":stack tiled
fi
tmux set-option -t "$SESSION" pane-border-status top
echo "[launch_replay] press 's' in the manager pane to calibrate and start."
if [ -n "${TMUX:-}" ]; then
    tmux switch-client -t "$SESSION"
else
    tmux attach -t "$SESSION"
fi
