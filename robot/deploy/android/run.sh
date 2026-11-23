#!/system/bin/sh
# 在 magisk 域里进入 chroot 并执行命令。
#
# 必须先有 root：这个脚本要被 `su -c` 调起，否则 SELinux 会挡住
# /dev/bus/usb（u:object_r:usb_device:s0 只有 magisk 域放行）。
#
# 用法：
#   su -c "/data/local/tmp/run.sh /opt/robot/host 'python3 main.py --port android'"

set -e

ROOT="${ROOT:-/data/local/linux}"

if [ "$(id -u)" != "0" ]; then
    echo "必须用 root 运行（su -c）" >&2
    exit 1
fi

if [ ! -x "$ROOT/bin/sh" ]; then
    echo "rootfs 不存在：$ROOT" >&2
    exit 1
fi

# 只在不重复的情况下挂载；重复 mount --bind 会叠加
mountpoint -q "$ROOT/dev"  || mount --bind /dev  "$ROOT/dev"
mountpoint -q "$ROOT/proc" || mount --bind /proc "$ROOT/proc"
mountpoint -q "$ROOT/sys"  || mount --bind /sys  "$ROOT/sys"

# Android 的 /etc 里没有这些，补一份，否则 apt 解析不了域名
[ -f "$ROOT/etc/resolv.conf" ] || \
    echo "nameserver 223.5.5.5" > "$ROOT/etc/resolv.conf"

WORKDIR="${1:-/opt/robot/host}"
shift 2>/dev/null || true
CMD="${*:-/bin/bash}"

exec chroot "$ROOT" /usr/bin/env -i \
    HOME=/root \
    TERM=xterm-256color \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 \
    /bin/bash -lc "cd '$WORKDIR' && $CMD"
