# 用 chroot 跑上位机（不用 Termux）

## 为什么不用 Termux

Termux 本质也是"用户态 Linux"，只是打包成 App。既然已经有 root，直接 chroot
更干净：

- **没有 proot 的 ptrace 开销**，OpenCV 跑得更快
- 完整 Debian/Ubuntu 用户空间，`apt` 装什么都行
- 不受 Timex 生命周期影响，不会被系统回收

设备侧实测（vivo V2072A / Android 13 / MTK 4.19.191）：

```
chroot / /system/bin/echo CHROOT_OK   → CHROOT_OK     toybox 自带 chroot
mount --bind A B && umount B          → BIND_OK       挂载可用
unshare -m true                       → UNSHARE_OK    命名空间可用
df -h /data                           → 100G 可用
```

## 一个前提：必须从 magisk 域启动

`/dev/bus/usb/*` 的标签是 `u:object_r:usb_device:s0`，SELinux 是 **Enforcing**。
实测只有 `u:r:magisk:s0` 能读：

```
adb shell su -c 'cat /dev/bus/usb/001/004'   # rc=0，能读
```

所以**所有入口都必须是 `su -c`**，包括开机自启。用 `adb shell chroot ...`
直接跑会得到一个"能跑但 pyusb 找不到设备"的诡异状态。

## 步骤

### 1. 拿 rootfs（Ubuntu Base arm64，26.4 MB）

```bash
# 清华镜像，国内快；三个源都实测可达
curl -O https://mirrors.tuna.tsinghua.edu.cn/ubuntu-cdimage/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-arm64.tar.gz
# 备选
# https://mirrors.ustc.edu.cn/ubuntu-cdimage/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-arm64.tar.gz
# https://cdimage.ubuntu.com/ubuntu-base/releases/22.04/release/ubuntu-base-22.04.5-base-arm64.tar.gz
```

### 2. 推到手机并解开

```bash
ROOT=/data/local/linux
adb push ubuntu-base-22.04.5-base-arm64.tar.gz /data/local/tmp/
adb shell su -c "mkdir -p $ROOT && tar -xzf /data/local/tmp/ubuntu-base-22.04.5-base-arm64.tar.gz -C $ROOT"
adb push robot/deploy/android/run.sh /data/local/tmp/run.sh
adb shell su -c "chmod 755 /data/local/tmp/run.sh"
```

### 3. 首次进去配环境

```bash
adb shell su -c "/data/local/tmp/run.sh /root '/bin/bash'"
```

进去之后（换成国内源，否则 apt 很慢）：

```bash
cat > /etc/apt/sources.list <<'EOF'
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports jammy main restricted universe multiverse
deb http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports jammy-updates main restricted universe multiverse
EOF

apt update
apt install -y python3 python3-numpy python3-opencv python3-usb usbutils
```

> 用 `apt install python3-usb` 装 pyusb，别 pip 装——系统包自带 libusb 依赖。

### 4. 验证能看见 CH340

```bash
lsusb                      # 应该能看到 1a86:7523
python3 -c "import usb.core; print(usb.core.find(idVendor=0x1a86, idProduct=0x7523))"
```

第二行**必须打印出一个 Device 对象**。如果打印 `None`：

| 现象 | 原因 |
|---|---|
| 报 `No backend available` | 没装 `libusb-1.0-0`，或 `/dev` 没 bind 进去 |
| 返回 `None` 但 `lsusb` 有 | 不是从 magisk 域进的 chroot |
| `lsusb` 也看不到 | OTG 没插好 / 线只供电不走数据 |

### 5. 部署代码

```bash
adb push robot/host /data/local/linux/opt/robot/host
adb shell su -c "/data/local/tmp/run.sh /opt/robot/host 'python3 android_ch340.py --selftest'"
```

`--selftest` 需要 TX-RX 短接。看到 `✓ 回环通过` 才算真的通。

### 6. 开局启动（比赛时不能连电脑）

用 Magisk 的开机脚本：

```bash
adb shell su -c "mkdir -p /data/adb/service.d"
adb shell su -c "cat > /data/adb/service.d/robot.sh <<'EOF'
#!/system/bin/sh
sleep 20
/data/local/tmp/run.sh /opt/robot/host 'python3 main.py --port android --camera http://127.0.0.1:8080/video' >> /data/local/tmp/robot.log 2>&1 &
EOF
chmod 755 /data/adb/service.d/robot.sh"
```

`sleep 20` 是等 Android 起完、Wi-Fi 连上（摄像头 App 要网络）。

## 摄像头

chroot 里拿不到 Android 的 Camera2 API，也拿不到 UVC（内核没编驱动）。
所以仍然是：**Android 侧的 App 开 MJPEG → chroot 里读 `127.0.0.1:8080`**。

chroot 和 Android 共享网络命名空间，`127.0.0.1` 是通的，不用担心。

## 和 Termux 的取舍

| | chroot | Termux |
|---|---|---|
| 性能 | 无额外开销 | proot 有 ptrace 开销 |
| 装包 | apt，要自己配源 | pkg，开箱即用 |
| 调试 | 靠 adb | 有终端，方便 |
| 开机自启 | service.d | 也要靠 service.d |
| 隔离 | 共享命名空间 | proot 更隔离 |

**如果只是想快点验证串口能不能通，Termux 更快上手。**
但既然你 root 都在、adb 也通，chroot 是更长期的选择——性能和省心都更好。

## 已知坑

1. **不要在 `/data/local/tmp` 里解压 rootfs**：那是 `shell_data_file` 标签，
   挂载和权限都可能出问题，用 `/data/local/linux`。
2. **mount --bind 会叠加**：重复执行会把 `/dev` 挂很多层，脚本里有 `mountpoint -q`
   守卫，别自己手敲。
3. **apt 会装 systemd 服务**，chroot 里没有 init，装完忽略那些报错即可。
4. **手机重启后 chroot 里的 mount 全没了**，必须靠 `run.sh` 重新挂，
   所以别指望手动 mount 一次就完事。
