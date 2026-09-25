"""Browser dashboard for the operator's small, live metric set."""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

DASHBOARD_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tidal Vehicle Dashboard</title>
<style>
:root{color-scheme:dark}body{margin:0;background:#0d1324;color:#eef3ff;font-family:Arial,sans-serif}main{max-width:1280px;margin:0 auto;padding:28px}.head{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:18px}.eyebrow{color:#55d7ff;font-size:11px;letter-spacing:2px}.live{color:#aeb9d3;font-size:13px}.dot{color:#35e28b}h1{margin:6px 0 0;font-size:30px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.card{background:#18223b;border:1px solid #2c3b5e;border-radius:12px;padding:18px;min-height:110px}.wide{grid-column:span 2}.label{color:#9eabc5;font-size:12px;text-transform:uppercase;letter-spacing:1px}.value{font-size:30px;font-weight:700;margin-top:12px}.sub{color:#aeb9d3;font-size:13px;margin-top:8px}.status{display:inline-block;padding:7px 12px;border-radius:999px;background:#35405e;color:#fff;font-weight:700;margin-top:14px}.map{height:330px;display:block;width:100%;background:#101a30;border-radius:8px}.tide{height:280px;display:block;width:100%;background:#101a30;border-radius:8px}.legend{display:flex;gap:16px;color:#aeb9d3;font-size:12px;margin-top:8px}.blue{color:#42a5ff}.orange{color:#ffc857}.red{color:#ff5d67}.ring-card{text-align:center}.ring{width:132px;height:132px;margin:10px auto 4px;position:relative}.ring svg{width:100%;height:100%;transform:rotate(-90deg)}.ring circle{fill:none;stroke-width:10}.ring .track{stroke:#2f3d5c}.ring .progress{stroke:#55d7cf;stroke-linecap:round;stroke-dasharray:339.3;stroke-dashoffset:339.3;transition:stroke-dashoffset .35s ease}.ring .reserve-progress{stroke:#ffc857}.ring-value{position:absolute;inset:0;display:grid;place-items:center;font-size:24px;font-weight:700}.coords{font-family:ui-monospace,monospace;font-size:18px;margin-top:12px}@media(max-width:800px){main{padding:16px}.grid{grid-template-columns:1fr}.wide{grid-column:span 1}}
</style></head><body><main>
<header class="head"><div><div class="eyebrow">TIDAL CORRIDOR / VEHICLE OPERATIONS</div><h1>Vehicle dashboard</h1></div><div class="live"><span class="dot">●</span> <span id="connection">Connecting telemetry</span></div></header>
<section class="grid">
<article class="card ring-card"><div class="label">Fuel</div><div class="ring"><svg viewBox="0 0 132 132"><circle class="track" cx="66" cy="66" r="54"/><circle class="progress" id="fuel-ring" cx="66" cy="66" r="54"/></svg><div class="ring-value" id="fuel">N/A</div></div><div class="sub">Vehicle fuel level</div></article>
<article class="card ring-card"><div class="label">Energy</div><div class="ring"><svg viewBox="0 0 132 132"><circle class="track" cx="66" cy="66" r="54"/><circle class="progress" id="energy-ring" cx="66" cy="66" r="54"/></svg><div class="ring-value" id="energy">—</div></div><div class="sub">Available battery energy</div></article>
<article class="card ring-card"><div class="label">Fuel &amp; energy reserve</div><div class="ring"><svg viewBox="0 0 132 132"><circle class="track" cx="66" cy="66" r="54"/><circle class="progress reserve-progress" id="reserve-ring" cx="66" cy="66" r="54"/></svg><div class="ring-value" id="reserve">—</div></div><div class="sub">Safety return reserve</div></article>
<article class="card"><div class="label">Current vehicle speed</div><div class="value" id="speed">—</div><div class="sub">Ground speed</div></article>
<article class="card"><div class="label">Vehicle coordinates</div><div class="coords" id="coordinates">—</div><div class="sub">Map frame</div></article>
<article class="card"><div class="label">Safety status</div><div class="status" id="safety">—</div><div class="sub" id="reason">Awaiting telemetry.</div></article>
<article class="card"><div class="label">Tide level</div><div class="value" id="tide-level">—</div><div class="sub" id="tide-state">—</div></article>
<article class="card wide"><div class="label">Tide level over time</div><canvas id="tide-chart" class="tide"></canvas><div class="legend"><span class="blue">━ Rising</span><span class="red">━ Falling</span></div></article>
<article class="card wide"><div class="label">Current location and path</div><canvas id="map" class="map"></canvas><div class="legend"><span class="blue">● Vehicle</span><span class="blue">━ Planned path</span><span class="orange">━ Return path</span></div></article>
</section></main><script>
const E=id=>document.getElementById(id),C=x=>Number.isFinite(Number(x))?Number(x):0,T=[];
function drawTide(){const c=E('tide-chart'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.scale(d,d);const w=r.width,h=r.height;x.clearRect(0,0,w,h);if(!T.length)return;const values=T.map(q=>q.level),lo=Math.min(...values),hi=Math.max(...values),span=Math.max(.2,hi-lo),left=34,right=12,top=16,bottom=28;x.strokeStyle='#405171';x.lineWidth=1;x.beginPath();x.moveTo(left,top);x.lineTo(left,h-bottom);x.lineTo(w-right,h-bottom);x.stroke();for(let i=1;i<T.length;i++){const a=T[i-1],b=T[i],xa=left+(i-1)/Math.max(1,T.length-1)*(w-left-right),xb=left+i/Math.max(1,T.length-1)*(w-left-right),ya=top+(hi-a.level)/span*(h-top-bottom),yb=top+(hi-b.level)/span*(h-top-bottom);x.strokeStyle=b.rising?'#42a5ff':'#ff5d67';x.lineWidth=3;x.beginPath();x.moveTo(xa,ya);x.lineTo(xb,yb);x.stroke()}x.fillStyle='#aeb9d3';x.font='11px Arial';x.fillText(hi.toFixed(2)+' m',left+6,top+8);x.fillText(lo.toFixed(2)+' m',left+6,h-bottom-6);x.fillText('Time',w/2-12,h-8)}
function drawMap(t){const c=E('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.scale(d,d);const w=r.width,h=r.height;x.clearRect(0,0,w,h);x.fillStyle='#101a30';x.fillRect(0,0,w,h);const paths=[{p:t.planned_path||[],color:'#42a5ff'},{p:t.return_path||[],color:'#ffc857'}].filter(a=>a.p.length);const all=paths.flatMap(a=>a.p).concat([{x:C(t.vehicle_x),y:C(t.vehicle_y)}]);if(!all.length)return;const xs=all.map(p=>p.x),ys=all.map(p=>p.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),sx=Math.max(maxX-minX,.01),sy=Math.max(maxY-minY,.01),scale=Math.min((w-48)/sx,(h-48)/sy),tx=p=>24+(p.x-minX)*scale,ty=p=>h-24-(p.y-minY)*scale;paths.forEach(a=>{x.strokeStyle=a.color;x.lineWidth=3;x.beginPath();a.p.forEach((p,i)=>i?x.lineTo(tx(p),ty(p)):x.moveTo(tx(p),ty(p)));x.stroke()});x.fillStyle='#fff';x.beginPath();x.arc(tx({x:C(t.vehicle_x),y:C(t.vehicle_y)}),6,0,Math.PI*2);x.fill()}
async function refresh(){try{const d=await(await fetch('/api/state')).json(),t=d.telemetry||{},fuel=t.fuel_percent,battery=C(d.battery_percent),reserve=C(d.return_margin_percent),speed=C(t.speed_mps),level=C(t.water_level_m),rate=C(t.tide_rate_m_per_minute),setRing=(id,value)=>{const el=E(id);if(el)el.style.strokeDashoffset=(339.3*(1-value/100)).toFixed(1)};E('fuel').textContent=fuel==null?'N/A':C(fuel).toFixed(1)+'%';E('energy').textContent=battery.toFixed(1)+'%';E('reserve').textContent=reserve.toFixed(1)+'%';setRing('fuel-ring',fuel==null?0:C(fuel));setRing('energy-ring',battery);setRing('reserve-ring',reserve);E('speed').textContent=speed.toFixed(2)+' m/s';E('coordinates').textContent=C(t.vehicle_x).toFixed(2)+', '+C(t.vehicle_y).toFixed(2);E('safety').textContent=d.return_required?'RETURN':(d.mission_state||'HOLD');E('reason').textContent=d.reason||'—';E('tide-level').textContent=level.toFixed(2)+' m';E('tide-state').textContent=(t.tide_state||'UNKNOWN')+' · risk '+C(t.tide_risk).toFixed(1)+'%';E('connection').textContent='Live telemetry';T.push({level,rising:rate>=0});if(T.length>72)T.shift();drawTide();drawMap(t)}catch(e){E('connection').textContent='Telemetry unavailable'}}setInterval(refresh,1000);refresh();
</script></body></html>"""

class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "TidalVehicleDashboard/1.0"
    _state = {"mission_state":"HOLD","effective_state":"HOLD","return_required":False,"battery_percent":0.0,"return_margin_percent":0.0,"reason":"Awaiting telemetry.","telemetry":{}}

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/api/state":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(json.dumps(self._state).encode("utf-8"))
            return
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.end_headers()
        self.wfile.write(DASHBOARD_PAGE.encode("utf-8"))

    def log_message(self, format: str, *args: object) -> None:
        return

def update_dashboard_state(**kwargs: object) -> None:
    DashboardHandler._state = dict(kwargs)

def serve_dashboard(host: str = "0.0.0.0", port: int = 8000) -> None:
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    print(f"Dashboard server running at http://{host}:{port}")
    server.serve_forever()
