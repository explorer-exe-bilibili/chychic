#!/system/bin/sh
# 显式清理 chroot 的 bind 挂载。
#
# ⚠️ 为什么单独做成一个脚本，而不是放在 enter.sh 退出时：
#    挂载是**全局**的，不归属某个进程。enter.sh 退出时如果 umount，会把
#    同时在用的另一个会话（比如 adb 里跑着的 python）的挂载一起拆掉，
#    对方表现是"设备突然消失"且不报任何错。
#
# ⚠️ 为什么需要清理：
#    挂载留着无害，但如果你之后 rm -rf 这个 rootfs 目录，会顺着 bind 挂载
#    删进 /dev、/proc 里 —— 那是灾难。所以删 rootfs 之前必须先跑这个脚本。
#
# 用法：su -c "sh cleanup.sh"
R="${ROOT:-/data/linux/ubuntu}"

if [ "$(id -u)" != "0" ]; then echo "需要 root" >&2; exit 1; fi

echo "== 当前挂载 =="
mount | grep "$R" || echo "  （没有该路径的挂载）"

echo "== 检查是否还有进程在用 =="
if command -v fuser >/dev/null 2>&1; then
    fuser -m "$R" 2>/dev/null && echo "  ⚠ 仍有进程引用，先让它们退出"
fi

for m in sys proc dev; do
    if mountpoint -q "$R/$m"; then
        if umount "$R/$m" 2>/dev/null; then
            echo "  已卸载 $m"
        else
            echo "  ✗ 卸载失败 $m（被占用？试用 umount -l，但会留下半断开状态）"
            echo "    正在使用该挂载的进程："
            fuser -m "$R/$m" 2>/dev/null
        fi
    else
        echo "  $m 未挂载，跳过"
    fi
done

echo "== 结果 =="
mount | grep "$R" || echo "  已清理干净"
