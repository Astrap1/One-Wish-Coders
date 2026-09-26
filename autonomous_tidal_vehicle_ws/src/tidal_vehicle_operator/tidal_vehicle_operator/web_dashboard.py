"""Small browser dashboard for the live tidal-corridor demonstration."""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


# This is deliberately dependency-free: the dashboard can run in the same ROS
# package as the scenario without needing a separate JavaScript build pipeline.
DASHBOARD_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tidal Corridor Dashboard</title>
<style>
:root{color-scheme:dark}*{box-sizing:border-box}body{margin:0;background:#070d1a;color:#eef3ff;font-family:Arial,sans-serif;background-image:radial-gradient(circle at 50% -20%,#173a56 0%,#070d1a 56%)}main{max-width:1280px;margin:auto;padding:24px}.head{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:16px}.eyebrow,.label{color:#63d9ff;font-size:12px;font-weight:700;letter-spacing:1.6px;text-transform:uppercase}.live,.sub,.legend{color:#839bb9;font-size:13px}.dot{color:#35e28b}h1{margin:6px 0 0;font-size:28px;letter-spacing:1px;text-transform:uppercase}.grid{display:grid;grid-template-columns:repeat(6,1fr);gap:14px}.card{position:relative;overflow:hidden;padding:16px;min-height:108px;background:linear-gradient(145deg,#111d36,#0c152a);border:1px solid #24496a;border-radius:12px;box-shadow:0 12px 32px rgba(0,0,0,.22)}.wide{grid-column:span 6}.span3{grid-column:span 3}.span2{grid-column:span 2}.value{margin-top:12px;font-size:29px;font-weight:700}.status{display:inline-block;margin-top:14px;padding:7px 12px;border:1px solid #38d6ff;border-radius:999px;background:#142d43;color:#78e5ff;font-weight:700;letter-spacing:1px}.map{display:block;width:100%;height:390px;margin-top:10px;background:#101a30;border:1px solid #214967;border-radius:8px}.tide{display:block;width:100%;height:180px;margin-top:10px;background:#101a30;border:1px solid #214967;border-radius:8px}.camera{display:block;width:min(100%,640px);aspect-ratio:16/9;object-fit:cover;margin:10px auto 0;background:#101a30;border:1px solid #214967;border-radius:8px}.legend{display:flex;flex-wrap:wrap;gap:14px;margin-top:8px}.blue{color:#42a5ff}.orange{color:#ffc857}.red{color:#f52222}.green{color:#35e28b}.map-legend{font-size:12px}.event-list{height:245px;overflow:auto;display:flex;flex-direction:column;gap:8px;margin-top:10px}.event-item{padding:8px 10px;border-left:2px solid #42d9ff;background:#101a30;color:#d7e6f7;font:12px ui-monospace,monospace}.event-time{color:#63d9ff;margin-right:8px}.remote-toggle,.remote-abort{display:block;margin:12px auto;padding:9px 16px;border:1px solid #48d8ff;border-radius:6px;background:#10243a;color:#8ceaff;font-weight:700;cursor:pointer}.remote-toggle.active{background:#1e5c71;color:#fff}.remote-abort{border-color:#bd5360;background:#351e2a;color:#ff9ba4}.remote-pad{width:190px;height:190px;margin:14px auto;position:relative;border:1px solid #2d5370;border-radius:50%;background:radial-gradient(circle,#152941 0 31%,transparent 32%),linear-gradient(45deg,transparent 49%,#2d5370 50%,transparent 51%),linear-gradient(-45deg,transparent 49%,#2d5370 50%,transparent 51%)}.remote-pad.disabled{opacity:.35;pointer-events:none}.remote-btn{position:absolute;width:42px;height:42px;border:1px solid #48d8ff;border-radius:50%;background:#10243a;color:#8ceaff;font-size:22px;cursor:pointer}.remote-up{top:8px;left:74px}.remote-down{bottom:8px;left:74px}.remote-left{left:8px;top:74px}.remote-right{right:8px;top:74px}.remote-stop{top:74px;left:74px;color:#ff8790;border-color:#bd5360;background:#351e2a;font-size:15px}.remote-status,.remote-note{text-align:center;color:#8da7c2;font-size:12px}.remote-note{color:#62809e;margin-top:6px}@media(max-width:800px){main{padding:16px}.grid{grid-template-columns:1fr}.card,.wide,.span2,.span3{grid-column:1 / -1}.map{height:280px}}
</style></head><body><main>
<header class="head"><div><div class="eyebrow">Tidal corridor / vehicle operations</div><h1>Vehicle dashboard</h1></div><div class="live"><span class="dot">●</span> <span id="connection">Connecting telemetry</span></div></header>
<section class="grid">
<article class="card wide"><div class="label">Terrain map &amp; mission route</div><canvas id="map" class="map"></canvas><div class="legend map-legend"><span style="color:#d7bb78">■ Firm shore</span><span style="color:#825b36">■ Mud</span><span style="color:#3987b7">■ Shallow water</span><span class="red">■ Physical obstacle</span><span class="green">● HOME</span><span style="color:#f6d94c">● DELIVERY</span><span class="blue">━ Planned path</span><span class="orange">━ Return path</span><span>━ Travelled</span></div></article>
<article class="card span3"><div class="label">Mission events</div><div id="mission-events" class="event-list"><div class="event-item">Waiting for mission events…</div></div></article>
<article class="card span3"><div class="label">Control mode</div><button class="remote-toggle" id="remote-toggle">Switch to remote control</button><button class="remote-toggle" id="dispatch-goal">Dispatch demo delivery</button><div class="remote-pad disabled" id="remote-pad"><button class="remote-btn remote-up" data-remote="forward">▲</button><button class="remote-btn remote-left" data-remote="left">◀</button><button class="remote-btn remote-stop" data-remote="stop">■</button><button class="remote-btn remote-right" data-remote="right">▶</button><button class="remote-btn remote-down" data-remote="reverse">▼</button></div><div class="remote-status" id="remote-status">Autonomous control active</div><div class="remote-note">Hold a direction to drive. Remote commands remain safety-capped.</div><button class="remote-abort" id="remote-abort">Abort mission and return home</button></article>
<article class="card span2"><div class="label">Safety status</div><div class="status" id="safety">—</div><div class="sub" id="reason">Awaiting telemetry.</div></article>
<article class="card span2"><div class="label">Current speed</div><div class="value" id="speed">—</div><div class="sub">Ground speed</div></article>
<article class="card span2"><div class="label">Vehicle coordinates</div><div class="value" id="coordinates">—</div><div class="sub">Map frame metres</div></article>
<article class="card span3"><div class="label">Energy / return margin</div><div class="value" id="energy">—</div><div class="sub" id="reserve">—</div></article>
<article class="card span3"><div class="label">Tide level</div><div class="value" id="tide-level">—</div><div class="sub" id="tide-state">—</div></article>
<article class="card span3"><div class="label">Tide level over time</div><canvas id="tide-chart" class="tide"></canvas></article>
<article class="card span3"><div class="label">Forward camera</div><img id="camera" class="camera" src="/api/camera.jpg" alt="Live forward camera view"><div class="sub" id="camera-status">Waiting for camera frames</div></article>
</section></main><script>
const E=id=>document.getElementById(id),N=v=>Number.isFinite(Number(v))?Number(v):0,T=[];let remoteMode=false;
const post=p=>fetch('/api/remote',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)}).then(r=>{if(!r.ok)throw Error('request rejected')});
function setRemote(enabled){remoteMode=!!enabled;E('remote-toggle').textContent=remoteMode?'Switch to autonomous control':'Switch to remote control';E('remote-toggle').classList.toggle('active',remoteMode);E('remote-pad').classList.toggle('disabled',!remoteMode);E('remote-status').textContent=remoteMode?'Remote control active · hold a direction to drive':'Autonomous control active'}
E('remote-toggle').onclick=()=>post({kind:'mode',enabled:!remoteMode}).catch(()=>E('remote-status').textContent='Control-mode request failed');
E('dispatch-goal').onclick=()=>post({kind:'goal',x:104.0,y:0.0}).then(()=>E('remote-status').textContent='Demo delivery goal dispatched').catch(()=>E('remote-status').textContent='Goal request failed');
const stop=()=>post({kind:'motion',action:'stop'}).catch(()=>{});document.querySelectorAll('[data-remote]').forEach(b=>{const a=b.dataset.remote;b.onpointerdown=e=>{e.preventDefault();if(!remoteMode)return;a==='stop'?stop():post({kind:'motion',action:a}).catch(()=>E('remote-status').textContent='Remote command failed')};['pointerup','pointerleave','pointercancel'].forEach(k=>b.addEventListener(k,stop))});window.addEventListener('blur',stop);E('remote-abort').onclick=()=>post({kind:'abort'}).then(()=>setRemote(false));
function terrainColor(v){return v<0?'#26334a':v>=90?'#f52222':v<=19?'#d7bb78':v<=50?'#825b36':'#3987b7'}
function drawMap(t){const c=E('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1,m=t.cost_map||{};c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.fillStyle='#101a30';x.fillRect(0,0,r.width,r.height);if(!m.width||!m.height||!m.data?.length){x.fillStyle='#839bb9';x.font='14px Arial';x.fillText('Waiting for the tidal-corridor terrain cost map',18,28);return}const res=N(m.resolution)||1,ox=N(m.origin_x),oy=N(m.origin_y),mw=N(m.width)*res,mh=N(m.height)*res,p={l:34,r:18,t:24,b:28},w=r.width-p.l-p.r,h=r.height-p.t-p.b,tx=q=>p.l+(q.x-ox)/mw*w,ty=q=>p.t+(oy+mh-q.y)/mh*h,cw=w/m.width,ch=h/m.height;for(let row=0;row<m.height;row++)for(let col=0;col<m.width;col++){const v=m.data[row*m.width+col],px=p.l+col*cw,py=p.t+(m.height-row-1)*ch;x.fillStyle=terrainColor(v);x.fillRect(px,py,Math.ceil(cw)+.5,Math.ceil(ch)+.5);if(v>=90){x.strokeStyle='#111827';x.lineWidth=1;x.strokeRect(px+1,py+1,Math.max(1,cw-2),Math.max(1,ch-2))}}x.strokeStyle='rgba(238,243,255,.55)';x.setLineDash([7,5]);x.lineWidth=1.5;x.beginPath();x.moveTo(tx({x:0}),ty({y:0}));x.lineTo(tx({x:104}),ty({y:0}));x.stroke();x.setLineDash([]);const route=(points,color,width)=>{if(!points?.length)return;x.strokeStyle=color;x.lineWidth=width;x.beginPath();points.forEach((q,i)=>i?x.lineTo(tx(q),ty(q)):x.moveTo(tx(q),ty(q)));x.stroke()};route(t.travelled_path,'#f4fbff',3);route(t.planned_path,'#42a5ff',3);route(t.return_path,'#ffc857',3);const marker=(q,label,fill)=>{x.fillStyle=fill;x.strokeStyle='#111827';x.lineWidth=2;x.beginPath();x.arc(tx(q),ty(q),7,0,Math.PI*2);x.fill();x.stroke();x.fillStyle='#101a30';x.font='700 11px Arial';x.fillText(label,tx(q)-13,ty(q)-11)};marker({x:0,y:0},'HOME','#35e28b');marker({x:104,y:0},'DELIVERY','#f6d94c');for(const [pos,label] of [[35,'BARRIER A'],[68,'BARRIER B']]){const l=tx({x:pos-2}),rr=tx({x:pos+2}),top=ty({y:4}),bottom=ty({y:-4});x.strokeStyle='#f6e84c';x.lineWidth=2;x.strokeRect(l,top,rr-l,bottom-top);x.fillStyle='#f6e84c';x.font='700 10px Arial';x.fillText(label,l-4,top-5)}x.fillStyle='#42a5ff';x.strokeStyle='#f4fbff';x.lineWidth=2;x.beginPath();x.arc(tx({x:N(t.vehicle_x),y:N(t.vehicle_y)}),ty({x:N(t.vehicle_x),y:N(t.vehicle_y)}),6,0,Math.PI*2);x.fill();x.stroke();x.fillStyle='#839bb9';x.font='10px Arial';x.fillText('map x (m)',r.width/2-22,r.height-7)}
function drawTide(){const c=E('tide-chart'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,r.width,r.height);if(!T.length)return;const lo=Math.min(...T),hi=Math.max(...T),span=Math.max(.2,hi-lo),pad=20;x.strokeStyle='#405171';x.strokeRect(pad,pad,r.width-2*pad,r.height-2*pad);x.strokeStyle='#42a5ff';x.lineWidth=3;x.beginPath();T.forEach((v,i)=>{const px=pad+i/Math.max(1,T.length-1)*(r.width-2*pad),py=pad+(hi-v)/span*(r.height-2*pad);i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()}
function events(items){E('mission-events').innerHTML=(items||[]).slice().reverse().map(q=>`<div class="event-item"><span class="event-time">${q.time||'--:--:--'}</span>${q.event||'Unknown event'}</div>`).join('')||'<div class="event-item">Waiting for mission events…</div>'}
async function refresh(){try{const d=await(await fetch('/api/state',{cache:'no-store'})).json(),t=d.telemetry||{},level=N(t.water_level_m);setRemote(d.remote_enabled);E('energy').textContent=N(d.battery_percent).toFixed(1)+'%';E('reserve').textContent='Return margin '+N(d.return_margin_percent).toFixed(1)+'%';E('speed').textContent=N(t.speed_mps).toFixed(2)+' m/s';E('coordinates').textContent=N(t.vehicle_x).toFixed(1)+', '+N(t.vehicle_y).toFixed(1);E('safety').textContent=d.return_required?'RETURN':(d.mission_state||'HOLD');E('reason').textContent=d.reason||'—';E('tide-level').textContent=level.toFixed(2)+' m';E('tide-state').textContent=(t.tide_state||'UNKNOWN')+' · risk '+(N(t.tide_risk)*100).toFixed(0)+'%';E('connection').textContent='Live telemetry';E('camera-status').textContent='Live camera stream';T.push(level);if(T.length>72)T.shift();drawMap(t);drawTide();events(t.mission_events)}catch(_){E('connection').textContent='Telemetry unavailable'}}setInterval(refresh,750);setInterval(()=>{E('camera').src='/api/camera.jpg?ts='+Date.now()},1000);refresh();
</script></body></html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "TidalVehicleDashboard/1.0"
    _state: dict[str, object] = {"mission_state": "HOLD", "effective_state": "HOLD", "return_required": False, "battery_percent": 0.0, "return_margin_percent": 0.0, "reason": "Awaiting telemetry.", "telemetry": {}}
    _camera_jpeg: bytes | None = None
    _remote_request_handler: Callable[[dict[str, Any]], bool] | None = None

    def do_GET(self) -> None:  # noqa: N802
        path = self.path.split("?", 1)[0]
        if path == "/api/camera.jpg":
            if self._camera_jpeg is None:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(self._camera_jpeg)
            return
        if path == "/api/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(json.dumps(self._state).encode("utf-8"))
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(DASHBOARD_PAGE.encode("utf-8"))

    def do_POST(self) -> None:  # noqa: N802
        if self.path != "/api/remote":
            self.send_error(404)
            return
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 512:
            self.send_error(400, "Invalid request length")
            return
        try:
            request = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.send_error(400, "Request must be JSON")
            return
        handler = self._remote_request_handler
        accepted = bool(handler(request)) if handler and isinstance(request, dict) else False
        self.send_response(202 if accepted else 400)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(json.dumps({"accepted": accepted}).encode("utf-8"))

    def log_message(self, format: str, *args: object) -> None:
        return


def update_dashboard_state(**kwargs: object) -> None:
    DashboardHandler._state = _json_safe(kwargs)


def update_camera_frame(frame: bytes) -> None:
    DashboardHandler._camera_jpeg = frame


def set_remote_request_handler(handler: Callable[[dict[str, Any]], bool] | None) -> None:
    """Install the ROS-node callback used by the local dashboard HTTP endpoint."""
    DashboardHandler._remote_request_handler = handler


def _json_safe(value: object) -> object:
    """Convert ROS sentinel floats into JSON values that browsers can parse."""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_json_safe(item) for item in value]
    return value


def serve_dashboard(host: str = "0.0.0.0", port: int = 8000) -> None:
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    print(f"Dashboard server running at http://{host}:{port}")
    server.serve_forever()
