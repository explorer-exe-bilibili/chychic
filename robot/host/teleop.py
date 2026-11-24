#!/usr/bin/env python3
"""网页遥控：手机（或树莓派）在车上跑这个，笔记本浏览器打开就能开。

    python3 teleop.py --port android
    # 然后在笔记本浏览器里打开  http://<车上设备的IP>:8000

为什么做成网页而不是终端键盘：
    MT 管理器的终端对方向键支持很差，而且手机夹在车上根本看不见。
    网页方案不依赖任何第三方库（用标准库 http.server），笔记本/平板/另一台手机
    都能当遥控器。

安全设计：
    - 浏览器 20Hz 持续上报，超过 400ms 没有任何指令就自动停车（拔网线/关网页都安全）
    - 速度在服务端再限幅一次，前端改参数也超不过 --v-max / --w-max
    - Ctrl-C 或退出时先发急停，再切 IDLE，最后才关串口
"""

from __future__ import annotations

import argparse
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import link as link_mod
import params
import protocol as p
from mcu import MCU

#: 多久没收到浏览器指令就自动停车（秒）。
#: 前端是 20Hz 上报，但浏览器在页面不可见时会把定时器限到 1Hz，
#: 留 0.8s 才不会把"切后台"误判成"抖动"。真正的保护由页面在
#: visibilitychange 时主动停车承担。
CMD_TIMEOUT = 0.8

_state = {
    "v": 0.0,
    "w": 0.0,
    "t": 0.0,
    "estop": False,
}
_lock = threading.Lock()

#: 当前主控页面。多个页面同时打开时，只有最后一个操作过的那页能下发指令，
#: 否则两个 20Hz 的数据流会互相覆盖，车在两个速度之间来回跳。
_active = {"cid": "", "t": 0.0}

#: 另一页要接管，必须等当前页静默这么久（秒）
TAKEOVER_S = 0.5


PAGE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>遥控</title>
<style>
  body{margin:0;background:#111;color:#ddd;font-family:system-ui,sans-serif;
       display:flex;flex-direction:column;align-items:center;height:100vh;overflow:hidden}
  #pad{position:relative;width:min(78vw,70vh);aspect-ratio:1;margin-top:3vh;
       border-radius:50%;background:#1b1b1b;border:2px solid #333;touch-action:none}
  #knob{position:absolute;width:22%;aspect-ratio:1;border-radius:50%;
        background:#3a7bd5;left:39%;top:39%;transition:background .15s}
  #readout{margin-top:2vh;font-size:15px;color:#888;font-variant-numeric:tabular-nums}
  #hz{font-size:13px;color:#666;margin-top:6px;font-variant-numeric:tabular-nums}
  #row{display:flex;gap:12px;margin-top:2vh;align-items:center}
  button{background:#222;color:#ddd;border:1px solid #444;border-radius:8px;
         padding:10px 18px;font-size:16px}
  button:active{background:#333}
  #stop{background:#7a2020;border-color:#a33}
  .ok{color:#4caf50}.bad{color:#e05a5a}
</style>
</head>
<body>
  <div id="pad"><div id="knob"></div></div>
  <div id="readout">拖动摇杆，或用 W A S D / 方向键</div>
  <div id="hz">上报 0.0 Hz</div>
  <div id="row">
    <button id="stop">急停</button>
    <span id="link" class="bad">未连接</span>
  </div>
<script>
const pad=document.getElementById('pad'),knob=document.getElementById('knob'),
      out=document.getElementById('readout'),linkEl=document.getElementById('link');
let v=0,w=0,active=false,dead=false;
const MAXV=__MAXV__,MAXW=__MAXW__;

function setKnob(x,y){ // x,y in [-1,1]
  knob.style.left=(39+x*39)+'%'; knob.style.top=(39-y*39)+'%';
}
function apply(x,y){
  v=y*MAXV; w=-x*MAXW;
  setKnob(x,y);
  out.textContent=`前进 ${v.toFixed(2)} m/s   转向 ${w.toFixed(2)} rad/s`;
}
function center(){v=0;w=0;setKnob(0,0);out.textContent='已回中';}

function pos(e){
  const r=pad.getBoundingClientRect();
  const t=e.touches?e.touches[0]:e;
  let x=(t.clientX-r.left)/r.width*2-1;
  let y=-((t.clientY-r.top)/r.height*2-1);
  const m=Math.hypot(x,y); if(m>1){x/=m;y/=m;}
  return [x,y];
}
pad.addEventListener('pointerdown',e=>{active=true;dead=false;apply(...pos(e));pad.setPointerCapture(e.pointerId)});
pad.addEventListener('pointermove',e=>{if(active&&!dead)apply(...pos(e))});
pad.addEventListener('pointerup',()=>{active=false;center()});
pad.addEventListener('pointercancel',()=>{active=false;center()});

document.addEventListener('keydown',e=>{
  const k=e.key.toLowerCase();
  if(k==='w'||k==='arrowup'){v=MAXV;}
  else if(k==='s'||k==='arrowdown'){v=-MAXV;}
  else if(k==='a'||k==='arrowleft'){w=MAXW;}
  else if(k==='d'||k==='arrowright'){w=-MAXW;}
  else if(k===' '){dead=true;v=0;w=0;}
});
document.addEventListener('keyup',e=>{
  const k=e.key.toLowerCase();
  if(['w','s','a','d','arrowup','arrowdown','arrowleft','arrowright'].includes(k)){v=0;w=0;}
});

document.getElementById('stop').onclick=()=>{
  dead=true;v=0;w=0;center();
  fetch('/estop',{method:'POST'});
};

const hzEl=document.getElementById('hz');
// 每个页面一个随机 id：服务端据此仲裁，避免两个页面同时上报互相覆盖
const CID=Math.random().toString(36).slice(2,10);
let lastT=0, hz=0;

// 页面切到后台时浏览器会把定时器限到 1Hz。与其让车"动一下停一下"，
// 不如干净地停住；回到前台后需要重新推摇杆才继续。
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){
    dead=true; v=0; w=0; setKnob(0,0);
    hzEl.textContent='页面在后台，已主动停车';
    send();
  }else{
    dead=false;
    hzEl.textContent='已回前台，推动摇杆继续';
  }
});

async function send(){
  const now=performance.now();
  if(lastT && !document.hidden){
    hz = hz ? hz*0.8 + (1000/(now-lastT))*0.2 : 1000/(now-lastT);
    hzEl.textContent='上报 '+hz.toFixed(1)+' Hz'+(hz<10?'  ⚠ 被限频':'');
  }
  lastT=now;
  try{
    const r=await fetch('/cmd?v='+v+'&w='+w+'&dead='+(dead?1:0)+'&cid='+CID,{method:'POST'});
    const s=await r.json();
    if(s.ignored){linkEl.textContent='已被另一页接管';linkEl.className='bad';return;}
    if(s.error){linkEl.textContent='链路异常';linkEl.className='bad';}
    else{linkEl.textContent=s.online?'在线':'离线';linkEl.className=s.online?'ok':'bad';}
    if(!s.ok){linkEl.textContent='下位机无响应';linkEl.className='bad';}
  }catch(e){linkEl.textContent='断连';linkEl.className='bad';}
}
setInterval(send,50);
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    """极简 HTTP：一个页面 + 两个接口。故意不做鉴权——只在局域网内用。"""

    # BaseHTTPRequestHandler 默认是 HTTP/1.0，每个请求都会新建一条 TCP 连接。
    # 前端 20Hz 轮询 = 每秒 20 次三次握手，在省电 Wi-Fi 上会被放大成严重卡顿。
    # 改成 HTTP/1.1 后连接复用，握手一次就够。
    protocol_version = "HTTP/1.1"

    max_v = 0.4
    max_w = 1.5
    mcu: MCU | None = None

    def log_message(self, *args) -> None:  # 静音，别刷屏
        pass

    def _send(self, code: int, body: bytes, ctype: str = "text/plain; charset=utf-8") -> None:
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802
        if self.path in ("/", "/index.html"):
            html = PAGE.replace("__MAXV__", str(self.max_v)).replace("__MAXW__", str(self.max_w))
            self._send(200, html.encode("utf-8"), "text/html; charset=utf-8")
        elif self.path == "/state":
            st = self.mcu.state if self.mcu else None
            data = {
                "online": bool(self.mcu and self.mcu.is_online()),
                "mode": st.mode if st else -1,
                "lf_state": st.lf_state if st else -1,
                "v_l": st.v_l if st else 0,
                "v_r": st.v_r if st else 0,
                "error": _state.get("error", ""),
            }
            self._send(200, json.dumps(data).encode(), "application/json")
        else:
            self._send(404, b"not found")

    def do_POST(self) -> None:  # noqa: N802
        u = urlparse(self.path)
        q = parse_qs(u.query)

        if u.path == "/estop":
            with _lock:
                _state.update(v=0.0, w=0.0, t=time.monotonic(), estop=True)
            if self.mcu:
                self.mcu.estop()
            self._send(200, b'{"ok":true}')
            return

        if u.path == "/cmd":
            try:
                v = float(q.get("v", ["0"])[0])
                w = float(q.get("w", ["0"])[0])
                dead = q.get("dead", ["0"])[0] == "1"
                cid = q.get("cid", [""])[0][:16]
            except ValueError:
                self._send(400, b'{"ok":false}')
                return

            now = time.monotonic()

            # 单主控仲裁：避免两个页面同时 20Hz 上报互相覆盖
            with _lock:
                if cid and _active["cid"] and cid != _active["cid"]:
                    if now - _active["t"] < TAKEOVER_S:
                        self._send(
                            200,
                            json.dumps({"ok": True, "ignored": True, "online": bool(self.mcu and self.mcu.is_online())}).encode(),
                            "application/json",
                        )
                        return
                if cid:
                    _active["cid"] = cid
                _active["t"] = now

            # 服务端二次限幅：前端被改也超不过上限
            v = max(-self.max_v, min(self.max_v, v))
            w = max(-self.max_w, min(self.max_w, w))
            if dead:
                v = w = 0.0

            with _lock:
                _state["v"] = v
                _state["w"] = w
                _state["t"] = time.monotonic()
                # 只有用户真正操作（dead=0）才解除急停锁存；
                # 急停期间前端一直发 dead=1，锁存得以保持。
                if not dead:
                    _state["estop"] = False

            online = bool(self.mcu and self.mcu.is_online())
            self._send(
                200,
                json.dumps(
                    {"ok": online, "online": online, "error": _state.get("error", "")}
                ).encode(),
                "application/json",
            )
            return

        self._send(404, b"not found")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="android", help="串口：android / /dev/ttyUSB0 / sim")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--v-max", type=float, default=0.40, help="线速度上限 m/s")
    ap.add_argument("--w-max", type=float, default=1.50, help="角速度上限 rad/s")
    args = ap.parse_args()

    params.load_override()

    link = link_mod.open_link(args.port, args.baud)
    mcu = MCU(link, params.WHEEL_BASE)

    Handler.max_v = args.v_max
    Handler.max_w = args.w_max
    Handler.mcu = mcu

    httpd = ThreadingHTTPServer(("0.0.0.0", args.http_port), Handler)
    threading.Thread(target=httpd.serve_forever, daemon=True).start()

    mcu.set_mode(p.MODE_MANUAL)
    print(f"遥控已启动：http://<本机IP>:{args.http_port}")
    print(f"  线速度上限 {args.v_max} m/s，角速度上限 {args.w_max} rad/s")
    print("  Ctrl-C 退出")

    last_print = time.monotonic()
    errors = 0
    while True:
        try:
            now = time.monotonic()

            with _lock:
                fresh = (now - _state["t"]) < CMD_TIMEOUT
                latched = _state["estop"]
                v = _state["v"] if fresh else 0.0
                w = _state["w"] if fresh else 0.0

            if latched:
                # 急停锁定中：只发零速，不碰模式，MCU 停在 ESTOP
                link.send_vel(0.0, 0.0)
            else:
                # MANUAL 模式按设计就是 50Hz 持续下发，不做去重——
                # 去重会导致"从急停恢复后模式切不回来"
                mcu.manual(v, w)

            mcu.poll()
            mcu.keepalive()

            if now - last_print > 2.0:
                last_print = now
                st = mcu.state
                if st is not None:
                    print(f"  {st.describe()}", flush=True)
                elif not mcu.is_online():
                    print("  等下位机遥测中…（检查接线/波特率/是否已烧录）", flush=True)

            if errors:
                print(f"  链路已恢复（此前连续失败 {errors} 次）", flush=True)
                errors = 0
            _state["error"] = ""

            time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n退出中…", flush=True)
            break
        except Exception as exc:  # noqa: BLE001
            # 单次 USB/串口异常不能让整个进程退出：http.server 是 daemon 线程，
            # 主线程一死服务就没了，浏览器看到的就是"断连"。
            errors += 1
            msg = f"{type(exc).__name__}: {exc}"
            _state["error"] = msg
            print(f"  [异常 {errors}] {msg}", flush=True)
            if errors >= 40:
                print("连续失败过多，退出（检查和下位机的连线）", flush=True)
                break
            time.sleep(0.1)

    # 收尾：先急停、再切 IDLE，最后才关串口
    try:
        mcu.estop()
        mcu.stop()
    except Exception:  # noqa: BLE001
        pass
    httpd.shutdown()
    link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
