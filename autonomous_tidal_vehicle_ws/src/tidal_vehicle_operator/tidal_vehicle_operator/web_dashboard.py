"""Simple browser dashboard for the operator summary."""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from .operator_dashboard import build_dashboard_payload

DASHBOARD_PAGE = """<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'><title>Mission Control</title><style>body{margin:0;background:#dce1eb;font-family:Arial;color:#f6f7ff}main{max-width:1350px;margin:7vh auto;padding:24px;background:#15113f;border-radius:10px}.head{display:flex;justify-content:space-between}.eye{color:#0ed0ee;font-size:11px;letter-spacing:2px}.live{color:#b7bad5;font-size:13px}.dot{color:#1fd567}h1{margin:5px 0 20px}.top,.bottom{display:grid;gap:14px}.top{grid-template-columns:1.4fr 1fr 1fr}.bottom{grid-template-columns:repeat(4,1fr);margin-top:14px}.p{background:#292958;border:1px solid #3c3d69;border-radius:9px;padding:16px}h2{font-size:15px;margin:0 0 14px}canvas{width:100%;height:175px;display:block}.metric{font-size:40px;margin:20px 0 5px}.muted{color:#b7bad5;font-size:12px}.row{display:flex;gap:8px;align-items:center;margin:18px 0}.row b{width:60px}.bar{background:#41426d;height:8px;flex:1;border-radius:6px}.bar i{display:block;height:100%;background:#0ed0ee;border-radius:inherit}.bar.green i{background:#1fd567}.list div{border-bottom:1px solid #454671;padding:10px 0;font-size:12px}.note{border-left:3px solid #ffc75d;padding-left:10px;font-size:12px;line-height:1.4}.route{margin:18px 0}.route span{display:flex;justify-content:space-between;font-size:12px;color:#b7bad5}@media(max-width:800px){main{margin:0;border-radius:0}.top,.bottom{grid-template-columns:1fr}}</style></head><body><main><header class='head'><div><div class='eye'>TIDAL CORRIDOR / VEHICLE OPERATIONS</div><h1>Mission control</h1></div><div class='live'><b class='dot'>o</b> <span id='connection'>Connecting telemetry</span></div></header><section class='top'><article class='p'><h2>Energy and return reserve</h2><canvas id='chart'></canvas><div class='muted'>Cyan: battery energy &nbsp; Green: return margin</div></article><article class='p'><h2>Mission readiness</h2><div class='row'><b>Safety</b><span class='bar'><i id='safe'></i></span><em id='safe-v'></em></div><div class='row'><b>Battery</b><span class='bar'><i id='bat'></i></span><em id='bat-v'></em></div><div class='row'><b>Reserve</b><span class='bar green'><i id='res'></i></span><em id='res-v'></em></div><div class='row'><b>Tide risk</b><span class='bar green'><i id='tide-bar'></i></span><em id='tide'></em></div><div class='row'><b>Mobility</b><span class='bar'><i id='mobility-bar'></i></span><em id='mobility'></em></div><div class='note' id='reason'>Awaiting telemetry.</div></article><article class='p list'><h2>Mission watch</h2><div>Safety state <b id='state'>-</b></div><div>Outbound route <b id='outbound'>0 points</b></div><div>Return route <b id='return-points'>0 points</b></div><div>Return reserve <b id='margin'>-</b></div></article></section><section class='bottom'><article class='p'><h2>Battery energy</h2><div class='metric' id='battery'>-</div><div class='muted'>Live vehicle-health estimate</div></article><article class='p'><h2>Safety state</h2><div class='metric' id='state-copy'>-</div><div class='muted' id='action'>Supervisor approval</div></article><article class='p'><h2>Energy gauge</h2><canvas id='gauge'></canvas><div class='metric' id='gauge-v'>-</div></article><article class='p'><h2>Route composition</h2><div class='route'><span>Outbound <b id='out-copy'>0 points</b></span><div class='bar'><i id='out-bar'></i></div></div><div class='route'><span>Return <b id='ret-copy'>0 points</b></span><div class='bar green'><i id='ret-bar'></i></div></div><div class='note' id='summary'>Loading...</div></article></section></main><script>const H=[],E=x=>document.getElementById(x),C=x=>Math.max(0,Math.min(100,Number(x)||0)),P=x=>C(x).toFixed(1)+'%';function cn(id){let c=E(id),r=c.getBoundingClientRect(),d=devicePixelRatio;c.width=r.width*d;c.height=r.height*d;let x=c.getContext('2d');x.scale(d,d);return[x,r.width,r.height]}function draw(){let a=cn('chart'),x=a[0],w=a[1],h=a[2];x.clearRect(0,0,w,h);[['b','#0ed0ee'],['m','#1fd567']].forEach(p=>{x.strokeStyle=p[1];x.lineWidth=3;x.beginPath();H.forEach((q,i)=>{let X=10+i/47*(w-20),Y=10+(100-q[p[0]])/100*(h-20);i?x.lineTo(X,Y):x.moveTo(X,Y)});x.stroke()})}function g(v){let a=cn('gauge'),x=a[0],w=a[1],h=a[2];x.clearRect(0,0,w,h);x.lineWidth=16;x.strokeStyle='#41426d';x.beginPath();x.arc(w/2,h-5,Math.min(w*.35,h*.75),Math.PI,2*Math.PI);x.stroke();x.strokeStyle='#1fd567';x.beginPath();x.arc(w/2,h-5,Math.min(w*.35,h*.75),Math.PI,Math.PI+Math.PI*v/100);x.stroke()}async function R(){try{let d=await(await fetch('/api/state')).json(),b=C(d.battery_percent),m=C(d.return_margin_percent),s=d.return_required?'RETURN':d.mission_state;[['battery',P(b)],['margin',P(m)],['state',s],['state-copy',s],['gauge-v',P(b)],['outbound',d.planned_route_points+' points'],['return-points',d.return_route_points+' points'],['out-copy',d.planned_route_points+' points'],['ret-copy',d.return_route_points+' points'],['reason',d.reason],['summary',d.summary]].forEach(q=>E(q[0]).textContent=q[1]);[['safe',d.return_required?0:100],['bat',b],['res',m],['out-bar',100*d.planned_route_points/Math.max(1,d.planned_route_points,d.return_route_points)],['ret-bar',100*d.return_route_points/Math.max(1,d.planned_route_points,d.return_route_points)]].forEach(q=>E(q[0]).style.width=q[1]+'%');E('safe-v').textContent=d.return_required?'0%':'100%';E('bat-v').textContent=Math.round(b)+'%';E('res-v').textContent=Math.round(m)+'%';E('action').textContent=d.return_required?'Controlled return required':'Supervisor approval';E('connection').textContent='Live telemetry';H.push({b:b,m:m});if(H.length>48)H.shift();draw();g(b)}catch(e){E('connection').textContent='Telemetry unavailable'}}setInterval(R,1000);R()</script></body></html>"""


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve a tiny HTML dashboard that reflects the last mission summary."""

    server_version = "TidalVehicleDashboard/1.0"
    _state = {
        "mission_state": "HOLD",
        "return_required": False,
        "planned_route_points": 0,
        "return_route_points": 0,
        "battery_percent": 0.0,
        "return_margin_percent": 0.0,
        "reason": "Awaiting telemetry.",
        "summary": "Mission state: HOLD. Battery 0.0%, return margin 0.0%. Outbound route points=0, return route points=0. Reason: Awaiting telemetry.",
    }

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
        self.wfile.write(DASHBOARD_PAGE.encode('utf-8'))
        return

        page = """
        <!doctype html>
        <html>
        <head>
          <meta charset="utf-8" />
          <title>Tidal Vehicle Dashboard</title>
          <style>
            body { font-family: Arial, sans-serif; background: #0b1020; color: #eef6ff; margin: 0; padding: 24px; }
            .card { max-width: 960px; margin: 0 auto; background: #111a2d; border-radius: 12px; padding: 24px; box-shadow: 0 10px 25px rgba(0,0,0,0.25); }
            h1 { margin-top: 0; }
            ul { line-height: 1.8; }
            .ok { color: #71f0a8; }
            .warn { color: #ffd166; }
          </style>
        </head>
        <body>
          <div class="card">
            <h1>Tidal Vehicle Operator Dashboard</h1>
            <p id="summary">Loading...</p>
            <ul>
              <li><strong>State:</strong> <span id="state">-</span></li>
              <li><strong>Battery:</strong> <span id="battery">-</span></li>
              <li><strong>Return margin:</strong> <span id="margin">-</span></li>
              <li><strong>Outbound points:</strong> <span id="outbound">-</span></li>
              <li><strong>Return points:</strong> <span id="return-points">-</span></li>
            </ul>
          </div>
          <script>
            async function refresh() {
              const response = await fetch('/api/state');
              const data = await response.json();
              document.getElementById('summary').textContent = data.summary;
              document.getElementById('state').textContent = data.mission_state;
              document.getElementById('battery').textContent = data.battery_percent + '%';
              document.getElementById('margin').textContent = data.return_margin_percent + '%';
              document.getElementById('outbound').textContent = data.planned_route_points;
              document.getElementById('return-points').textContent = data.return_route_points;
            }
            setInterval(refresh, 1000);
            refresh();
          </script>
        </body>
        </html>
        """
        self.wfile.write(page.encode("utf-8"))

    def log_message(self, format: str, *args: object) -> None:
        return


def update_dashboard_state(**kwargs: object) -> None:
    DashboardHandler._state = build_dashboard_payload(**kwargs)


def serve_dashboard(host: str = "127.0.0.1", port: int = 8000) -> None:
    server = ThreadingHTTPServer((host, port), DashboardHandler)
    print(f"Dashboard server running at http://{host}:{port}")
    server.serve_forever()
