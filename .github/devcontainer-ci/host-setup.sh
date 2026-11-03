#!/usr/bin/env bash
# Prepares a GitHub-hosted Ubuntu runner so that it looks enough like a
# student's machine for a given devcontainer.json to start.
#
# Usage: host-setup.sh <feature>...
#   x11          Xvfb on :99 with an MIT cookie in ~/.Xauthority (Linux desktop)
#   dri          /dev/dri via the vkms virtual GPU, or stub device nodes
#   nvidia-stub  fake NVIDIA devices, binaries, ICDs and libraries
#   wsl          fake Docker Desktop + WSLg layout (/dev/dxg, /usr/lib/wsl, wslg)
#
# Variables the dev container picks up via ${localEnv:...} are appended to
# $GITHUB_ENV so later `devcontainer up/exec` steps see them.
set -euxo pipefail

setenv() { echo "$1=$2" >> "${GITHUB_ENV:-/dev/null}"; export "$1=$2"; }

need_xvfb() {
    if ! command -v Xvfb >/dev/null || ! command -v xauth >/dev/null; then
        sudo apt-get update -q
        sudo apt-get install -yq xvfb xauth x11-utils
    fi
}

wait_for_socket() {
    for _ in $(seq 1 30); do
        [ -S "$1" ] && return 0
        sleep 1
    done
    echo "::error::X socket $1 did not appear"
    exit 1
}

# config.sh derives the git identity from USERNAME.
setenv USERNAME ci-user

for feature in "$@"; do
    case "$feature" in
    x11)
        need_xvfb
        # Use a cookie so the Xauthority plumbing is exercised, not bypassed.
        # `xauth add` ties it to this hostname, which differs inside the
        # container, so rewrite it as FamilyWild (ffff) as GDM/Xwayland do.
        touch "$RUNNER_TEMP/host.xauth" "$HOME/.Xauthority"
        xauth -f "$RUNNER_TEMP/host.xauth" add :99 . "$(mcookie)"
        xauth -f "$RUNNER_TEMP/host.xauth" nlist :99 | sed 's/^..../ffff/' \
            | xauth -f "$HOME/.Xauthority" nmerge -
        nohup Xvfb :99 -auth "$HOME/.Xauthority" -screen 0 1280x720x24 \
            > "$RUNNER_TEMP/xvfb.log" 2>&1 &
        wait_for_socket /tmp/.X11-unix/X99
        setenv DISPLAY :99
        setenv XAUTHORITY "$HOME/.Xauthority"
        ;;
    dri)
        # Prefer a real (virtual) DRM device; fall back to bare device nodes,
        # which are enough for `docker run --device /dev/dri` to succeed.
        sudo modprobe vkms || true
        if ! compgen -G "/dev/dri/card*" >/dev/null; then
            echo "::warning::vkms unavailable, using stub /dev/dri nodes"
            sudo mkdir -p /dev/dri
            sudo mknod -m 666 /dev/dri/card0 c 226 0
            sudo mknod -m 666 /dev/dri/renderD128 c 226 128
        fi
        ls -l /dev/dri
        ;;
    nvidia-stub)
        for dev in nvidiactl:255 nvidia0:0 nvidia-modeset:254; do
            sudo mknod -m 666 "/dev/${dev%%:*}" c 195 "${dev##*:}"
        done
        # nvidia-uvm has a dynamic major number; any free one will do here.
        sudo mknod -m 666 /dev/nvidia-uvm c 509 0
        sudo mknod -m 666 /dev/nvidia-uvm-tools c 509 1
        sudo install -m 755 /bin/true /usr/bin/nvidia-smi
        sudo install -m 755 /bin/true /usr/bin/nvidia-persistenced
        sudo mkdir -p /run/nvidia-persistenced /etc/OpenCL/vendors \
            /usr/share/glvnd/egl_vendor.d /usr/share/vulkan/icd.d
        sudo python3 -c 'import socket; socket.socket(socket.AF_UNIX).bind("/run/nvidia-persistenced/socket")'
        echo libnvidia-opencl.so.1 | sudo tee /etc/OpenCL/vendors/nvidia.icd
        echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
            | sudo tee /usr/share/glvnd/egl_vendor.d/10_nvidia.json
        echo '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0"}}' \
            | sudo tee /usr/share/vulkan/icd.d/nvidia_icd.json
        # Marker library: the setup scripts should symlink it into the container.
        sudo touch /usr/lib/x86_64-linux-gnu/libnvidia-cistub.so.1
        ;;
    wsl)
        need_xvfb
        # Docker Desktop exposes WSLg under /run/desktop/mnt/host/wslg.
        # WSLg's X server does not use cookies, hence -ac.
        sudo mknod -m 666 /dev/dxg c 10 250
        sudo mkdir -p /usr/lib/wsl/lib /run/desktop/mnt/host/wslg/.X11-unix
        sudo touch /run/desktop/mnt/host/wslg/PulseServer
        nohup Xvfb :0 -ac -screen 0 1280x720x24 > "$RUNNER_TEMP/xvfb.log" 2>&1 &
        wait_for_socket /tmp/.X11-unix/X0
        sudo mount --bind /tmp/.X11-unix /run/desktop/mnt/host/wslg/.X11-unix
        ;;
    *)
        echo "::error::Unknown host feature '$feature'"
        exit 1
        ;;
    esac
done
