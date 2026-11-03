#!/usr/bin/env bash
# Runs inside a dev container (via `devcontainer exec`) and checks that the
# environment a student gets is usable.
#
# Usage: smoke-test.sh <check>...
#   vnc          built-in VNC desktop on :4, noVNC on port 6080
#   host-x11     host X server reachable on $CI_EXPECT_DISPLAY (with cookie)
#   dri          /dev/dri passed through
#   auto-dri     linux-auto chose the DRI path
#   auto-nvidia  linux-auto chose the NVIDIA path
#   nvidia-stub  host NVIDIA files/libraries linked into the container
#   nvidia-gpu   real NVIDIA GPU visible (self-hosted only)
#   wsl          WSLg/Docker Desktop mounts and X server on :0
#   wsl-gpu      Mesa renders through d3d12 (self-hosted only)
set -uo pipefail

failures=0
log="$(mktemp)"

# check <description> <command> [warning]: with "warning", a failure is only reported.
check() {
    local desc="$1" cmd="$2" level="${3:-error}"
    if bash -c "$cmd" >"$log" 2>&1; then
        echo "PASS: $desc"
    else
        echo "::$level::FAIL: $desc"
        sed 's/^/    /' "$log"
        [ "$level" = warning ] || failures=$((failures + 1))
    fi
}

# Retries a command for up to a minute, for services started asynchronously.
retry() { for _ in $(seq 1 30); do bash -c "$1" && return 0; sleep 2; done; return 1; }
export -f retry

# Connects to $DISPLAY with whichever X client the image has.
x_connect() {
    if command -v xdpyinfo >/dev/null; then xdpyinfo >/dev/null
    elif command -v xset >/dev/null; then xset q >/dev/null
    elif command -v xprop >/dev/null; then xprop -root >/dev/null
    else
        echo "no X client tool in image; only checked the socket"
        [ -S "/tmp/.X11-unix/X${DISPLAY#*:}" ] || [ -L "/tmp/.X11-unix/X${DISPLAY#*:}" ]
    fi
}
export -f x_connect

renderer() { glxinfo -B 2>/dev/null | grep -i 'OpenGL renderer'; }
export -f renderer

# Common to every configuration.
check "remote user is ubuntu" '[ "$(id -un)" = ubuntu ]'
check "workspace is mounted" '[ -d .devcontainer ]'
check "host home mounted at ~/homedir" '[ -d ~/homedir ]'
check "postStartCommand ran config.sh (git user.name set)" '[ -n "$(git config --global user.name)" ]'
# /workspaces is only writable by ubuntu when VS Code clones into a volume.
check "bash history persisted under /workspaces" '[ -L ~/.bash_history ]' warning
check "ROS 2 environment works" 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 pkg prefix rclpy'

for c in "$@"; do
    case "$c" in
    vnc)
        check "DISPLAY is :4" '[ "$DISPLAY" = :4 ]'
        check "noVNC listening on 6080" 'retry "exec 3<>/dev/tcp/127.0.0.1/6080"'
        check "VNC X server reachable" 'retry x_connect'
        ;;
    host-x11)
        check "DISPLAY forwarded from host" '[ "$DISPLAY" = "$CI_EXPECT_DISPLAY" ]'
        check "host X socket linked into /tmp/.X11-unix" '[ -e "/tmp/.X11-unix/X${DISPLAY#*:}" ]'
        check "host X server reachable" 'x_connect'
        ;;
    dri)
        check "/dev/dri present" '[ -d /dev/dri ] && ls /dev/dri/card*'
        ;;
    auto-dri)
        check "/dev/dri linked from host" '[ -e /dev/dri ]'
        check "NVIDIA profile not installed" '[ ! -e /etc/profile.d/nvidia-gpu.sh ]'
        ;;
    auto-nvidia)
        check "NVIDIA profile installed" '[ -f /etc/profile.d/nvidia-gpu.sh ]'
        check "NVIDIA devices linked" '[ -e /dev/nvidiactl ] && [ -e /dev/nvidia0 ]'
        ;;
    nvidia-stub)
        check "nvidia-smi available" 'nvidia-smi'
        check "host NVIDIA libraries linked" '[ -L /usr/lib/x86_64-linux-gnu/libnvidia-cistub.so.1 ]'
        ;;
    nvidia-gpu)
        check "nvidia-smi sees a GPU" 'nvidia-smi -L | grep -q GPU'
        ;;
    wsl)
        check "DISPLAY is :0" '[ "$DISPLAY" = :0 ]'
        check "WAYLAND_DISPLAY set" '[ "$WAYLAND_DISPLAY" = wayland-0 ]'
        check "/dev/dxg present" '[ -e /dev/dxg ]'
        check "/usr/lib/wsl mounted" '[ -d /usr/lib/wsl/lib ]'
        check "/mnt/wslg mounted" '[ -e /mnt/wslg/PulseServer ]'
        check "WSLg X server reachable" 'x_connect'
        ;;
    wsl-gpu)
        check "OpenGL renders through d3d12" 'renderer | grep -qi d3d12'
        ;;
    *)
        echo "::error::Unknown check '$c'"
        failures=$((failures + 1))
        ;;
    esac
done

# Informational only: hosted runners fall back to software rendering.
command -v glxinfo >/dev/null && renderer || true

echo "$failures check(s) failed"
[ "$failures" -eq 0 ]
