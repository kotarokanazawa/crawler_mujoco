"""Small dependency-free browser UI for live MuJoCo velocity commands."""

from __future__ import annotations

import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


HTML = r'''<!doctype html>
<html lang="ja"><head><meta charset="utf-8"><title>Crawler MuJoCo control</title>
<style>
body{font-family:sans-serif;max-width:620px;margin:32px auto;background:#1d2025;color:#eee}
.card{background:#292d34;padding:24px;border-radius:12px}label{display:block;margin:18px 0}
input[type=range]{width:100%}.value{float:right;font-variant-numeric:tabular-nums}
button{font-size:16px;padding:10px 18px;margin:8px 4px;border:0;border-radius:6px;cursor:pointer}
.stop{background:#b43b3b;color:white}.reset{background:#d28a31}.pause{background:#4b78b7;color:white}
</style></head><body><div class="card"><h2>Crawler velocity control</h2>
<label>linear.x [m/s] <span class="value" id="lv">0.300</span>
<input id="linear" type="range" min="-0.6" max="0.6" step="0.01" value="0.3"></label>
<label>angular.z [rad/s] <span class="value" id="av">0.000</span>
<input id="angular" type="range" min="-2.0" max="2.0" step="0.05" value="0"></label>
<button class="pause" id="pause">Pause</button><button class="reset" id="reset">Reset</button>
<button class="stop" id="stop">Stop</button><p id="status">connecting...</p></div>
<script>
const linear=document.querySelector('#linear'), angular=document.querySelector('#angular');
async function send(extra={}){document.querySelector('#lv').textContent=(+linear.value).toFixed(3);
document.querySelector('#av').textContent=(+angular.value).toFixed(3);
await fetch('/command',{method:'POST',headers:{'Content-Type':'application/json'},
body:JSON.stringify({linear:+linear.value,angular:+angular.value,...extra})});}
linear.oninput=()=>send(); angular.oninput=()=>send();
document.querySelector('#pause').onclick=()=>send({toggle_pause:true});
document.querySelector('#reset').onclick=()=>send({reset:true});
document.querySelector('#stop').onclick=()=>send({stop:true});
setInterval(async()=>{try{let s=await (await fetch('/status')).json();
document.querySelector('#status').textContent=`t=${s.time.toFixed(2)} s  x=${s.x.toFixed(3)} m  ${s.paused?'PAUSED':'RUNNING'}`;
document.querySelector('#pause').textContent=s.paused?'Resume':'Pause';}catch(e){}},250);
send();</script></body></html>'''


class ControlState:
    def __init__(self, linear: float = 0.3):
        self.lock = threading.Lock()
        self.linear = linear
        self.angular = 0.0
        self.paused = False
        self.running = True
        self.reset_requested = False
        self.time = 0.0
        self.x = 0.0

    def command(self) -> tuple[float, float, bool, bool, bool]:
        with self.lock:
            reset = self.reset_requested
            self.reset_requested = False
            return self.linear, self.angular, self.paused, self.running, reset

    def update_pose(self, sim_time: float, x: float) -> None:
        with self.lock:
            self.time, self.x = sim_time, x


def start_control_server(state: ControlState, host: str = "127.0.0.1",
                         port: int = 8765, open_browser: bool = True):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def reply(self, code: int, body: bytes, content_type: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/status":
                with state.lock:
                    payload = {"time": state.time, "x": state.x, "paused": state.paused,
                               "linear": state.linear, "angular": state.angular}
                self.reply(200, json.dumps(payload).encode(), "application/json")
            else:
                self.reply(200, HTML.encode(), "text/html; charset=utf-8")

        def do_POST(self):
            length = int(self.headers.get("Content-Length", "0"))
            payload = json.loads(self.rfile.read(length) or b"{}")
            with state.lock:
                state.linear = float(payload.get("linear", state.linear))
                state.angular = float(payload.get("angular", state.angular))
                if payload.get("toggle_pause"):
                    state.paused = not state.paused
                if payload.get("reset"):
                    state.reset_requested = True
                if payload.get("stop"):
                    state.running = False
            self.reply(204, b"", "text/plain")

    server = ThreadingHTTPServer((host, port), Handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    url = f"http://{host}:{port}"
    if open_browser:
        webbrowser.open(url)
    return server, url
