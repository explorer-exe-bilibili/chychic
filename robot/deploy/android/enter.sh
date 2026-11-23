#!/system/bin/sh
# 进 chroot 的交互式入口。
#
# 用法（在 MT 管理器的终端里）：
#     su
#     sh /data/local/tmp/enter.sh
#
# 注意三点：
#   1. 必须先 su —— chroot() 需要 CAP_SYS_CHROOT，
#      而且只有 magisk 域能读 /dev/bus/usb（SELinux 是 Enforcing）
#   2. /dev /proc /sys 要 bind 进去，否则里面看不到 USB、也没有 /proc
#   3. 退出时自动 umount，避免下次重复挂载叠加

ROOT="${ROOT:-/data/linux/ubuntu}"

if [ "$(id -u)" != "0" ]; then
    echo "需要 root：请先执行 su" >&2
    exit 1
fi

if [ ! -x "$ROOT/bin/bash" ]; then
    echo "rootfs 不可用：$ROOT" >&2
    echo "用 ROOT=/你的/路径 sh $0 指定" >&2
    exit 1
fi

# bind 之前先确认目录存在（rootfs 里 dev/proc/sys 一般都有，保险起见）
mkdir -p "$ROOT/dev" "$ROOT/proc" "$ROOT/sys" 2>/dev/null

mountpoint -q "$ROOT/dev"  || mount --bind /dev  "$ROOT/dev"
mountpoint -q "$ROOT/proc" || mount --bind /proc "$ROOT/proc"
mountpoint -q "$ROOT/sys"  || mount --bind /sys  "$ROOT/sys"

# chroot 里没有 Android 的 resolver 配置，apt 会解析不了域名
[ -s "$ROOT/etc/resolv.conf" ] || echo "nameserver 223.5.5.5" > "$ROOT/etc/resolv.conf"

chroot "$ROOT" /usr/bin/env -i \
    HOME=/root \
    TERM=xterm-256color \
    PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 \
    /bin/bash -l

# 退出后收拾干净，否则下次 mount --bind 会叠一层
umount "$ROOT/sys"  2>/dev/null
umount "$ROOT/proc" 2>/dev/null
umount "$ROOT/dev"  2>/dev/null
