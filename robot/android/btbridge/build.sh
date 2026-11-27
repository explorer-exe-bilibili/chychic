#!/bin/bash
# 不依赖 Gradle，直接用 Android SDK 的 build-tools 出 APK。
# 需要：ANDROID_SDK 或默认路径下的 build-tools + platforms。
set -e

cd "$(dirname "$0")"

SDK="${ANDROID_SDK:-$LOCALAPPDATA/Android/Sdk}"
BT="$(ls -d "$SDK"/build-tools/* 2>/dev/null | sort -V | tail -1)"
JAR="$(ls -d "$SDK"/platforms/*/android.jar 2>/dev/null | sort -V | tail -1)"
OUT=build

echo "build-tools : $BT"
echo "android.jar : $JAR"
[ -x "$BT/d8" ] || { echo "找不到 d8，检查 SDK 路径"; exit 1; }
[ -f "$JAR" ]  || { echo "找不到 android.jar"; exit 1; }

rm -rf "$OUT"
mkdir -p "$OUT/classes" "$OUT/dex"

echo "== 1) 编译 Java =="
# 优先用 --release 8；JDK 太新不支持时退回 -source/-target
if ! javac --release 8 -classpath "$JAR" -d "$OUT/classes" \
        $(find src -name '*.java') 2>"$OUT/javac.err"; then
    echo "  --release 8 失败，改用 -source 8 -target 8"
    javac -source 8 -target 8 -bootclasspath "$JAR" -classpath "$JAR" \
        -d "$OUT/classes" $(find src -name '*.java')
fi
find "$OUT/classes" -name '*.class' | wc -l | xargs echo "  生成 class 数:"

echo "== 2) 转 dex =="
"$BT/d8" --lib "$JAR" --min-api 26 --output "$OUT/dex" \
    $(find "$OUT/classes" -name '*.class')

echo "== 3) 打包资源清单 =="
"$BT/aapt2" link -o "$OUT/base.apk" -I "$JAR" \
    --manifest AndroidManifest.xml \
    --min-sdk-version 26 --target-sdk-version 33

echo "== 4) 把 classes.dex 塞进 APK =="
python - "$OUT/base.apk" "$OUT/dex/classes.dex" <<'PY'
import sys, zipfile
apk, dex = sys.argv[1], sys.argv[2]
with zipfile.ZipFile(apk, 'a', zipfile.ZIP_DEFLATED) as z:
    z.write(dex, 'classes.dex')
print("  dex 已加入")
PY

echo "== 5) 对齐 =="
"$BT/zipalign" -f 4 "$OUT/base.apk" "$OUT/aligned.apk"

echo "== 6) 签名 =="
# 密钥放在项目根目录，不要放进 build/ —— 否则清一次 build 签名就变了，
# 后果是装新版必须先卸载旧版，每次都要手动点确认。
if [ ! -f "debug.keystore" ]; then
    keytool -genkeypair -keystore "debug.keystore" -storepass android \
        -keypass android -alias androiddebugkey -keyalg RSA -validity 10000 \
        -dname "CN=Android Debug,O=Android,C=US" >/dev/null 2>&1
fi
"$BT/apksigner" sign --ks "debug.keystore" --ks-pass pass:android \
    --key-pass pass:android --out "$OUT/btbridge.apk" "$OUT/aligned.apk"

echo "== 完成 =="
ls -lh "$OUT/btbridge.apk"
