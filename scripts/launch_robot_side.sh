#!/usr/bin/env bash
# Robot-side launcher (everything-on-robot topology).
#
# Brings up the four components of the teleop path on the Jetson, in one tmux
# window split into tiled panes, in startup order:
#
#     hand -> deploy -> relay -> manager
#
# Every ZMQ/DDS link is localhost on the robot. The Quest reaches the relay over
# whatever network you already have (shared WiFi works; there is no wired-link
# component here). The laptop's only job is `ssh <robot>; tmux attach -t g1_robot`
# to drive the manager keyboard.
#
# This repo owns the VR side only. The deploy pane drives an EXTERNAL GR00T
# checkout ($GROOT_REPO) as a black box over ZMQ 5556/5557 — see CLAUDE.md and
# docs/INTERFACE_CONTRACT.md.
#
#   ./scripts/launch_robot_side.sh            # tmux: hand+deploy+relay+manager
#   ./scripts/launch_robot_side.sh all        # same as above
#   ./scripts/launch_robot_side.sh kill       # tear the tmux session down
#
# Single-component mode (foreground in the current terminal; this is also what
# each tmux pane calls under the hood):
#
#   ./scripts/launch_robot_side.sh hand       # BrainCo hand service (first)
#   ./scripts/launch_robot_side.sh deploy     # GR00T g1_deploy_onnx_ref container
#   ./scripts/launch_robot_side.sh relay      # Quest ROS1->ZMQ relay container
#   ./scripts/launch_robot_side.sh manager    # Quest teleop manager (press s to start)
#
#   --print / -n   show the exact command(s) without running (safe to test).
set -euo pipefail

SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
REPO="${REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"

# ---------------------------------------------------------------------------
# Config — override by exporting before you call, e.g. ROBOT_IFACE=eth0
# ---------------------------------------------------------------------------
# External GR00T checkout: supplies the SONIC deploy binary/container and the
# gear_sonic Python package the manager imports for FK calibration. Not vendored,
# not a submodule — point this at wherever you cloned it.
GROOT_REPO="${GROOT_REPO:-$HOME/GR00T-WholeBodyControl}"
ZMQ_HOST="${ZMQ_HOST:-localhost}"            # deploy --zmq-host: manager is on the robot
DEPLOY_TARGET="${DEPLOY_TARGET:-real}"       # deploy.sh arg: 'real' auto-detects the iface
OUTPUT_TYPE="${OUTPUT_TYPE:-all}"            # 'all' or 'zmq' (zmq skips ROS2)
DEPLOY_EXTRA="${DEPLOY_EXTRA:-}"             # extra deploy.sh flags, e.g. --cp policy/sonic_v1_1/model
# -n for the hand service's DDS. Defaults to the Jetson's onboard NIC rather than
# letting the SDK pick: it MUST match the interface the deploy binary resolves, and
# leaving it to chance is the usual cause of "arms move, fingers dead". Set
# ROBOT_IFACE= (empty) to fall back to the hand service's own default.
ROBOT_IFACE="${ROBOT_IFACE-enP8p1s0}"
# Where the built BrainCo hand service binary lives. Prefer this repo's own
# submodule build, but fall back to the GR00T checkout's: the submodule is
# byte-identical at the same commit, and the GR00T robot setup already builds it
# there, so a robot provisioned per that README needs no second build.
if [ -z "${HAND_DIR:-}" ]; then
    _groot_hand="$GROOT_REPO/gear_sonic_deploy/thirdparty/brainco_hand_service/bin"
    if [ -x "$REPO/third_party/brainco_hand_service/bin/brainco_hand_server" ]; then
        HAND_DIR="$REPO/third_party/brainco_hand_service/bin"
    elif [ -x "$_groot_hand/brainco_hand_server" ]; then
        HAND_DIR="$_groot_hand"
    else
        HAND_DIR="$REPO/third_party/brainco_hand_service/bin"   # for the error message
    fi
fi
HAND_USE_SYSTEMD="${HAND_USE_SYSTEMD:-0}"    # 1 = systemctl restart brainco_hand.service
BAG_DIR="${BAG_DIR:-}"                       # set to RECORD a Quest rosbag while teleoperating
PLAY_BAG="${PLAY_BAG:-}"                     # set to DRIVE THE ROBOT from a bag instead of a
                                             # live Quest (no headset needed). Mutually
                                             # exclusive with BAG_DIR. The manager still runs
                                             # its normal two-press ramp against measured
                                             # joints, so this is the safe way to replay onto
                                             # hardware — see the note in usage().
BAG_LOOP="${BAG_LOOP:-}"                     # with PLAY_BAG, loop the bag forever
LATENCY_CSV="${LATENCY_CSV:-}"               # set to write a per-frame latency trace
# No colon in the expansion: MANAGER_EXTRA="" must mean "no flags / full motion",
# and ${VAR:-d} would silently re-apply the default for an empty value (also when
# each tmux pane re-evaluates this after env_prefix exports it).
MANAGER_EXTRA="${MANAGER_EXTRA---static-base}"   # arms/hands only by default: no walk, no
                                             # turn-in-place, no crouch. Relax deliberately,
                                             # e.g. MANAGER_EXTRA="--disable-walk" or ""
TELEOP_VENV="${TELEOP_VENV:-$REPO/.venv_teleop}"
SESSION="${SESSION:-g1_robot}"               # tmux session name

# Config vars propagated into each tmux window (so exported overrides survive).
CONFIG_VARS=(REPO GROOT_REPO ZMQ_HOST DEPLOY_TARGET OUTPUT_TYPE DEPLOY_EXTRA \
             ROBOT_IFACE HAND_DIR HAND_USE_SYSTEMD BAG_DIR PLAY_BAG BAG_LOOP \
             LATENCY_CSV MANAGER_EXTRA TELEOP_VENV SESSION)
DEFAULT_COMPONENTS=(hand deploy relay manager)

DRYRUN=0
POS=()
for arg in "$@"; do
    case "$arg" in
        -n|--print) DRYRUN=1 ;;
        *) POS+=("$arg") ;;
    esac
done
MODE="${POS[0]:-}"

# emit <cmd...>: print the shell-quoted command, then run it unless --print.
emit() {
    printf '%q ' "$@"; echo
    [ "$DRYRUN" -eq 1 ] || "$@"
}

# env_prefix: "VAR=val VAR2=val2 ..." for the current config, shell-quoted, so a
# tmux window's fresh shell inherits the same resolved config this process has.
env_prefix() {
    local v esc out=""
    for v in "${CONFIG_VARS[@]}"; do
        printf -v esc '%q' "${!v}"
        out+="$v=$esc "
    done
    printf '%s' "$out"
}

# delay_for <component>: seconds to wait before starting it (enforces order).
# ZMQ SUB reconnects on its own, so the manager binding 5556 after the deploy
# starts is harmless; this ordering is for readable logs and because the deploy
# blocks on its own [Y/n] gate anyway.
delay_for() {
    case "$1" in
        hand)    echo 0 ;;
        deploy)  echo 3 ;;
        relay)   echo 6 ;;
        manager) echo 10 ;;
        *)       echo 0 ;;
    esac
}

# require_groot: fail early and legibly rather than deep inside a tmux pane.
require_groot() {
    local runner="$GROOT_REPO/gear_sonic_deploy/docker/run-ros2-dev.sh"
    if [ ! -x "$runner" ]; then
        cat >&2 <<EOF
ERROR: GR00T checkout not usable at GROOT_REPO=$GROOT_REPO
       expected an executable: $runner

This repo does VR teleop only; the SONIC deploy binary lives in a separate
GR00T-WholeBodyControl checkout. Clone it and either export GROOT_REPO=<path>
or place it at \$HOME/GR00T-WholeBodyControl.
EOF
        exit 1
    fi
}

# run <venv-or-"-"> <workdir-or-"-"> <cmd...>: echo the resolved command; run it
# (foreground) unless --print. Activates the venv and cd's first when given.
run() {
    local venv="$1" workdir="$2"; shift 2
    echo "# [robot:$COMPONENT] ZMQ_HOST=$ZMQ_HOST GROOT_REPO=$GROOT_REPO"
    [ "$venv" != "-" ] && echo "source $venv/bin/activate"
    [ "$workdir" != "-" ] && echo "cd $workdir"
    printf '%q ' "$@"; echo   # shell-quoted so --print output is copy-paste-safe
    if [ "$DRYRUN" -eq 1 ]; then return 0; fi
    # shellcheck disable=SC1091
    [ "$venv" != "-" ] && source "$venv/bin/activate"
    [ "$workdir" != "-" ] && cd "$workdir"
    exec "$@"
}

# run_single <component>: launch one component in the foreground.
run_single() {
    COMPONENT="$1"
    case "$COMPONENT" in
        hand)
            if [ "$HAND_USE_SYSTEMD" -eq 1 ]; then
                echo "# BrainCo hand service via systemd (HAND_USE_SYSTEMD=1)"
                echo "sudo systemctl restart brainco_hand.service && systemctl status brainco_hand.service"
                [ "$DRYRUN" -eq 1 ] && exit 0
                sudo systemctl restart brainco_hand.service
                exec systemctl status brainco_hand.service
            fi
            # Manual launch (default). No -n unless ROBOT_IFACE is set — the hand
            # service's own default interface works. Needs unitree_sdk2 +
            # CycloneDDS installed system-wide; see install_scripts/.
            if [ "$DRYRUN" -eq 0 ] && [ ! -x "$HAND_DIR/brainco_hand_server" ]; then
                echo "ERROR: $HAND_DIR/brainco_hand_server not found or not executable." >&2
                echo "       Build it: cd $REPO/third_party/brainco_hand_service && \\" >&2
                echo "                 mkdir -p build && cd build && cmake .. && make -j6" >&2
                echo "       (needs unitree_sdk2 + CycloneDDS system-wide under /usr/local)" >&2
                echo "       Or reuse an existing build, e.g. the GR00T one:" >&2
                echo "         HAND_DIR=$GROOT_REPO/gear_sonic_deploy/thirdparty/brainco_hand_service/bin" >&2
                exit 1
            fi
            hand_cmd=(sudo ./brainco_hand_server)
            [ -n "$ROBOT_IFACE" ] && hand_cmd+=(-n "$ROBOT_IFACE")
            run - "$HAND_DIR" "${hand_cmd[@]}"
            ;;
        deploy)
            require_groot
            # Runs INSIDE the GR00T repo's ROS2 container, which drops into an
            # interactive shell after its banner. --host-net is mandatory on the
            # real robot: the deploy binary's DDS must reach the 192.168.123.x
            # interface, which the bridge default cannot carry, and in bridge mode
            # ZMQ 5556 is not even port-mapped.
            #
            # The container's cwd is already /workspace/g1_deploy, so the command
            # below needs no cd. In tmux a watcher waits for the container's ready
            # marker and types it in; outside tmux, paste it yourself. deploy.sh
            # then stops at its own [Y/n] prompt — the operator types y, which is
            # the last gate before the robot is commanded.
            deploy_cmd="source scripts/setup_env.sh && ./deploy.sh $DEPLOY_TARGET"
            deploy_cmd="$deploy_cmd --input-type zmq_manager"
            deploy_cmd="$deploy_cmd --zmq-host $ZMQ_HOST --output-type $OUTPUT_TYPE"
            [ -n "$DEPLOY_EXTRA" ] && deploy_cmd="$deploy_cmd $DEPLOY_EXTRA"
            echo "# [robot:deploy] container: \$GROOT_REPO/gear_sonic_deploy/docker/run-ros2-dev.sh --host-net"
            echo "# [robot:deploy] command inside container: $deploy_cmd"
            if [ "$DRYRUN" -eq 1 ]; then
                printf '%q ' "$GROOT_REPO/gear_sonic_deploy/docker/run-ros2-dev.sh" --host-net; echo
                return 0
            fi
            if [ -n "${TMUX_PANE:-}" ]; then
                (
                    # 'Relays active' is the container startup script's last line,
                    # right before it hands over to the interactive shell.
                    for _ in $(seq 1 1800); do
                        tmux display-message -p -t "$TMUX_PANE" '' >/dev/null 2>&1 || exit 0
                        if tmux capture-pane -p -t "$TMUX_PANE" 2>/dev/null \
                                | grep -q 'Relays active'; then
                            sleep 1
                            tmux send-keys -t "$TMUX_PANE" "$deploy_cmd" C-m
                            exit 0
                        fi
                        sleep 1
                    done
                    echo "[robot:deploy] container never became ready; run manually: $deploy_cmd" >&2
                ) &
            else
                echo "# Not inside tmux — paste the command above into the container shell."
            fi
            exec "$GROOT_REPO/gear_sonic_deploy/docker/run-ros2-dev.sh" --host-net
            ;;
        relay)
            # Quest relay in Docker with host networking (no NAT hop). The Quest
            # Unity app targets <this robot>:10000. No --camera-* flags, so the
            # container's optional ego-view image relay stays off.
            # No venv — run_quest_relay.py drives Docker via the system python3.
            relay_cmd=(python3 "$REPO/quest_bridge/run_quest_relay.py" --network-host)
            if [ -n "$PLAY_BAG" ] && [ -n "$BAG_DIR" ]; then
                echo "ERROR: PLAY_BAG and BAG_DIR are mutually exclusive (replay vs record)." >&2
                exit 2
            fi
            if [ -n "$PLAY_BAG" ]; then
                # Drive the stack from a recording: the bag supplies the ROS
                # topics the Quest endpoint would have, so relay/msgpack/ZMQ are
                # all still in the loop and the manager cannot tell it from live.
                relay_cmd+=(--play-bag "$PLAY_BAG")
                [ -n "$BAG_LOOP" ] && relay_cmd+=(--loop)
            elif [ -n "$BAG_DIR" ]; then
                relay_cmd+=(--record-bag "$BAG_DIR")
            fi
            run - - "${relay_cmd[@]}"
            ;;
        manager)
            # All sources local: relay (5559) + deploy feedback (5557) on the robot.
            # Attach over ssh (tmux) to drive the keyboard state machine.
            # shellcheck disable=SC2086
            mgr_cmd=("$TELEOP_VENV/bin/python" -m teleop_manager.quest_manager
                     --relay-host localhost --relay-port 5559
                     --port 5556
                     --feedback-host localhost --feedback-port 5557)
            [ -n "$LATENCY_CSV" ] && mgr_cmd+=(--latency-csv "$LATENCY_CSV")
            if [ "$DRYRUN" -eq 0 ] && [ ! -x "$TELEOP_VENV/bin/python" ]; then
                echo "ERROR: no teleop venv at $TELEOP_VENV" >&2
                echo "       Create it: bash install_scripts/install_teleop.sh" >&2
                exit 1
            fi
            # MANAGER_EXTRA is deliberately unquoted: it carries zero or more flags.
            run - "$REPO" "${mgr_cmd[@]}" $MANAGER_EXTRA
            ;;
        *)
            usage; exit 2 ;;
    esac
}

# launch_tmux <component...>: all components as tiled panes in one tmux window
# (so you see everything at once), started in order, then attach (or switch-client
# if already inside tmux).
launch_tmux() {
    local comps=("$@")
    command -v tmux >/dev/null 2>&1 || { echo "ERROR: tmux not found on this host" >&2; exit 1; }
    require_groot
    if [ "$DRYRUN" -eq 0 ] && tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "tmux session '$SESSION' already exists." >&2
        echo "  attach: tmux attach -t $SESSION    kill: $SELF kill" >&2
        exit 1
    fi
    local envp; envp="$(env_prefix)"
    local first=1 c d inner body win_cmd
    for c in "${comps[@]}"; do
        d="$(delay_for "$c")"
        # Run in a subshell so the component's `exec` replaces the subshell only,
        # leaving the pane's outer shell alive to show output after it exits.
        inner="$envp$(printf '%q' "$SELF") $c"
        body="( $inner ); printf '\n[%s exited — press Enter to close] ' $c; read"
        [ "$d" -gt 0 ] && body="echo 'waiting ${d}s for startup order...'; sleep $d; $body"
        # Each pane titles its own border from inside (via \$TMUX_PANE), so the
        # label is always correct regardless of pane creation/layout ordering.
        win_cmd="tmux select-pane -t \"\$TMUX_PANE\" -T $c; $body"
        if [ "$first" -eq 1 ]; then
            emit tmux new-session -d -s "$SESSION" -n stack "$win_cmd"
            first=0
        else
            # Re-tile after each split so panes stay evenly sized and none get
            # too small for the next split.
            emit tmux split-window -t "$SESSION":stack "$win_cmd"
            emit tmux select-layout -t "$SESSION":stack tiled
        fi
    done
    emit tmux set-option -t "$SESSION" pane-border-status top
    emit tmux select-layout -t "$SESSION":stack tiled
    if [ -n "${TMUX:-}" ]; then
        emit tmux switch-client -t "$SESSION"
    else
        emit tmux attach -t "$SESSION"
    fi
}

usage() {
    cat >&2 <<EOF
Usage: $0 [all|hand|deploy|relay|manager|kill] [--print]

Robot-side components (everything-on-robot topology):
  (no args)  start hand + deploy + relay + manager as tiled panes, then attach
  all        same as no args
  hand       BrainCo hand service (must be up first)            — single, foreground
  deploy     GR00T g1_deploy_onnx_ref in its container          — single, foreground
  relay      Quest relay (TCP 10000 + ZMQ 5559)                 — single, foreground
  manager    Quest teleop manager (binds 5556)                  — single, foreground
  kill       kill the '$SESSION' tmux session

Keys in the manager pane:
  s  1st: start + ramp to the calibration pose.  2nd: countdown -> teleop
  r  recalibrate     p  pause/resume     f  fingers on/off
  c / x  record episode start-stop / abort
  -  /  =  crouch / stand          q  stop

Drive the manager from the laptop with:  ssh <robot> ; tmux attach -t $SESSION
E-stop is 'o' in the deploy pane.

Resolved config: GROOT_REPO=$GROOT_REPO
                 ZMQ_HOST=$ZMQ_HOST  OUTPUT_TYPE=$OUTPUT_TYPE  DEPLOY_EXTRA='$DEPLOY_EXTRA'
                 ROBOT_IFACE=${ROBOT_IFACE:-<hand-service default>}  SESSION=$SESSION
                 MANAGER_EXTRA='$MANAGER_EXTRA'
                 BAG_DIR=${BAG_DIR:-<no recording>}  PLAY_BAG=${PLAY_BAG:-<live Quest>}
                 LATENCY_CSV=${LATENCY_CSV:-<none>}
Override any of these via env vars (see the config block at the top of this file).

Driving the robot from a recording (no headset):
  PLAY_BAG=~/bags/quest_20260929_120000.bag BAG_LOOP=1 $0
The manager is NOT put in --no-robot here: the deploy is real, so you still get
the normal two-press ramp from the robot's measured pose. Time the SECOND 's' to
a moment when the recording is at the operator's rest pose — calibration samples
whatever frame is playing, and rosbag play cannot be rewound by the manager.
Do NOT use the manager's --replay (NPZ) path on hardware: it skips the ramp.
EOF
}

case "$MODE" in
    hand|deploy|relay|manager)
        run_single "$MODE"
        ;;
    kill)
        emit tmux kill-session -t "$SESSION"
        ;;
    ""|all|tmux)
        launch_tmux "${DEFAULT_COMPONENTS[@]}"
        ;;
    *)
        usage; exit 2 ;;
esac
