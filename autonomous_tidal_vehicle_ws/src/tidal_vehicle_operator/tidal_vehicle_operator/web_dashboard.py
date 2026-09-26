"""Browser dashboard for the operator's small, live metric set."""

from __future__ import annotations

import json
import math
from collections.abc import Callable
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

DASHBOARD_PAGE = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Tidal Vehicle Dashboard</title>
<style>
:root{color-scheme:dark}body{margin:0;background:#0d1324;color:#eef3ff;font-family:Arial,sans-serif}main{max-width:1280px;margin:0 auto;padding:28px}.head{display:flex;justify-content:space-between;align-items:flex-start;margin-bottom:18px}.eyebrow{color:#55d7ff;font-size:11px;letter-spacing:2px}.live{color:#aeb9d3;font-size:13px}.dot{color:#35e28b}h1{margin:6px 0 0;font-size:30px}.grid{display:grid;grid-template-columns:repeat(3,1fr);gap:14px}.card{background:#18223b;border:1px solid #2c3b5e;border-radius:12px;padding:18px;min-height:110px}.wide{grid-column:span 2}.label{color:#9eabc5;font-size:12px;text-transform:uppercase;letter-spacing:1px}.value{font-size:30px;font-weight:700;margin-top:12px}.sub{color:#aeb9d3;font-size:13px;margin-top:8px}.status{display:inline-block;padding:7px 12px;border-radius:999px;background:#35405e;color:#fff;font-weight:700;margin-top:14px}.map{height:330px;display:block;width:100%;background:#101a30;border-radius:8px}.tide{height:280px;display:block;width:100%;background:#101a30;border-radius:8px}.camera{display:block;width:100%;aspect-ratio:16/9;object-fit:cover;background:#101a30;border-radius:8px}.legend{display:flex;gap:16px;color:#aeb9d3;font-size:12px;margin-top:8px}.blue{color:#42a5ff}.orange{color:#ffc857}.red{color:#ff5d67}.ring-card{text-align:center}.ring{width:132px;height:132px;margin:10px auto 4px;position:relative}.ring svg{width:100%;height:100%;transform:rotate(-90deg)}.ring circle{fill:none;stroke-width:10}.ring .track{stroke:#2f3d5c}.ring .progress{stroke:#55d7cf;stroke-linecap:round;stroke-dasharray:339.3;stroke-dashoffset:339.3;transition:stroke-dashoffset .35s ease}.ring .reserve-progress{stroke:#ffc857}.ring-value{position:absolute;inset:0;display:grid;place-items:center;font-size:24px;font-weight:700}.coords{font-family:ui-monospace,monospace;font-size:18px;margin-top:12px}@media(max-width:800px){main{padding:16px}.grid{grid-template-columns:1fr}.wide{grid-column:span 1}}
</style></head><body><main>
<header class="head"><div><div class="eyebrow">TIDAL CORRIDOR / VEHICLE OPERATIONS</div><h1>Vehicle dashboard</h1></div><div class="live"><span class="dot">●</span> <span id="connection">Connecting telemetry</span></div></header>
<article class="card wide"><div class="label">Forward camera</div><img id="camera" class="camera" src="/api/camera.jpg" alt="Live forward camera view"><div class="sub" id="camera-status">Waiting for camera frames</div></article>
<section class="grid">
<article class="card ring-card"><div class="label">Fuel</div><div class="ring"><svg viewBox="0 0 132 132"><circle class="track" cx="66" cy="66" r="54"/><circle class="progress" id="fuel-ring" cx="66" cy="66" r="54"/></svg><div class="ring-value" id="fuel">N/A</div></div><div class="sub">Vehicle fuel level</div></article>
<article class="card ring-card"><div class="label">Energy</div><div class="ring"><svg viewBox="0 0 132 132"><circle class="track" cx="66" cy="66" r="54"/><circle class="progress" id="energy-ring" cx="66" cy="66" r="54"/></svg><div class="ring-value" id="energy">—</div></div><div class="sub">Available battery energy</div></article>
<article class="card ring-card"><div class="label">Fuel &amp; energy reserve</div><div class="ring"><svg viewBox="0 0 132 132"><circle class="track" cx="66" cy="66" r="54"/><circle class="progress reserve-progress" id="reserve-ring" cx="66" cy="66" r="54"/></svg><div class="ring-value" id="reserve">—</div></div><div class="sub">Safety return reserve</div></article>
<article class="card"><div class="label">Current vehicle speed</div><div class="value" id="speed">—</div><div class="sub">Ground speed</div></article>
<article class="card"><div class="label">Vehicle coordinates</div><div class="coords" id="coordinates">—</div><div class="sub">Map frame</div></article>
<article class="card"><div class="label">Safety status</div><div class="status" id="safety">—</div><div class="sub" id="reason">Awaiting telemetry.</div></article>
<article class="card"><div class="label">Tide level</div><div class="value" id="tide-level">—</div><div class="sub" id="tide-state">—</div></article>
<article class="card wide"><div class="label">Tide level over time</div><canvas id="tide-chart" class="tide"></canvas><div class="legend"><span class="blue">━ Rising</span><span class="red">━ Falling</span></div></article>
<article class="card wide"><div class="label">Gazebo top-down map &amp; route</div><canvas id="map" class="map"></canvas><div class="legend"><span class="blue">● Vehicle</span><span class="blue">━ Planned path</span><span class="orange">━ Return path</span><span>━ Travelled</span><span>terrain background</span></div></article>
</section></main><script>
const E=id=>document.getElementById(id),C=x=>Number.isFinite(Number(x))?Number(x):0,T=[];
function drawTide(){const c=E('tide-chart'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.scale(d,d);const w=r.width,h=r.height;x.clearRect(0,0,w,h);if(!T.length)return;const values=T.map(q=>q.level),lo=Math.min(...values),hi=Math.max(...values),span=Math.max(.2,hi-lo),left=34,right=12,top=16,bottom=28;x.strokeStyle='#405171';x.lineWidth=1;x.beginPath();x.moveTo(left,top);x.lineTo(left,h-bottom);x.lineTo(w-right,h-bottom);x.stroke();for(let i=1;i<T.length;i++){const a=T[i-1],b=T[i],xa=left+(i-1)/Math.max(1,T.length-1)*(w-left-right),xb=left+i/Math.max(1,T.length-1)*(w-left-right),ya=top+(hi-a.level)/span*(h-top-bottom),yb=top+(hi-b.level)/span*(h-top-bottom);x.strokeStyle=b.rising?'#42a5ff':'#ff5d67';x.lineWidth=3;x.beginPath();x.moveTo(xa,ya);x.lineTo(xb,yb);x.stroke()}x.fillStyle='#aeb9d3';x.font='11px Arial';x.fillText(hi.toFixed(2)+' m',left+6,top+8);x.fillText(lo.toFixed(2)+' m',left+6,h-bottom-6);x.fillText('Time',w/2-12,h-8)}
function drawMap(t){const c=E('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.scale(d,d);const w=r.width,h=r.height;x.clearRect(0,0,w,h);x.fillStyle='#101a30';x.fillRect(0,0,w,h);const paths=[{p:t.planned_path||[],color:'#42a5ff'},{p:t.return_path||[],color:'#ffc857'}].filter(a=>a.p.length);const all=paths.flatMap(a=>a.p).concat([{x:C(t.vehicle_x),y:C(t.vehicle_y)}]);if(!all.length)return;const xs=all.map(p=>p.x),ys=all.map(p=>p.y),minX=Math.min(...xs),maxX=Math.max(...xs),minY=Math.min(...ys),maxY=Math.max(...ys),sx=Math.max(maxX-minX,.01),sy=Math.max(maxY-minY,.01),scale=Math.min((w-48)/sx,(h-48)/sy),tx=p=>24+(p.x-minX)*scale,ty=p=>h-24-(p.y-minY)*scale;paths.forEach(a=>{x.strokeStyle=a.color;x.lineWidth=3;x.beginPath();a.p.forEach((p,i)=>i?x.lineTo(tx(p),ty(p)):x.moveTo(tx(p),ty(p)));x.stroke()});x.fillStyle='#fff';x.beginPath();x.arc(tx({x:C(t.vehicle_x),y:C(t.vehicle_y)}),6,0,Math.PI*2);x.fill()}
function drawTopDownMap(t){const c=E('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1,m=t.cost_map||{};c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,r.width,r.height);x.fillStyle='#101a30';x.fillRect(0,0,r.width,r.height);const originX=Number(m.origin_x)||0,originY=Number(m.origin_y)||0,res=Number(m.resolution)||1,mapW=(Number(m.width)||0)*res,mapH=(Number(m.height)||0)*res;const paths=[{p:t.planned_path||[],color:'#42a5ff'},{p:t.return_path||[],color:'#ffc857'}].filter(a=>a.p.length);const all=paths.flatMap(a=>a.p).concat([{x:C(t.vehicle_x),y:C(t.vehicle_y)}]);let minX=originX,minY=originY,maxX=originX+mapW,maxY=originY+mapH;if(!mapW||!mapH){if(!all.length){x.fillStyle='#839bb9';x.font='12px Arial';x.fillText('Waiting for Gazebo map and route data',16,24);return}const xs=all.map(p=>p.x),ys=all.map(p=>p.y);minX=Math.min(...xs);maxX=Math.max(...xs);minY=Math.min(...ys);maxY=Math.max(...ys)}const sx=Math.max(maxX-minX,.01),sy=Math.max(maxY-minY,.01),scale=Math.min((r.width-48)/sx,(r.height-48)/sy),tx=p=>24+(p.x-minX)*scale,ty=p=>r.height-24-(p.y-minY)*scale;if(m.width&&m.height&&m.data?.length){const cellW=scale*res,cellH=scale*res;for(let row=0;row<m.height;row++)for(let col=0;col<m.width;col++){const v=m.data[row*m.width+col];x.fillStyle=v<0?'#26334a':v>=90?'#c74652':`hsl(${190-(Math.min(v,89)/89)*150},65%,${28+(1-Math.min(v,89)/89)*28}%)`;x.fillRect(tx({x:originX+col*res,y:originY+row*res+res}),ty({x:originX+col*res,y:originY+row*res+res}),Math.ceil(cellW)+1,Math.ceil(cellH)+1)}}paths.forEach(a=>{x.strokeStyle=a.color;x.lineWidth=3;x.beginPath();a.p.forEach((p,i)=>i?x.lineTo(tx(p),ty(p)):x.moveTo(tx(p),ty(p)));x.stroke()});x.fillStyle='#fff';x.beginPath();x.arc(tx({x:C(t.vehicle_x),y:C(t.vehicle_y)}),6,0,Math.PI*2);x.fill()}
function drawMap(t){drawTopDownMap(t)}
function drawTopDownMap(t){const c=E('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1,m=t.cost_map||{};c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,r.width,r.height);x.fillStyle='#101a30';x.fillRect(0,0,r.width,r.height);const ox=Number(m.origin_x)||0,oy=Number(m.origin_y)||0,res=Number(m.resolution)||1,mw=(Number(m.width)||0)*res,mh=(Number(m.height)||0)*res,paths=[{p:t.planned_path||[],color:'#42a5ff',dots:true},{p:t.return_path||[],color:'#ffc857',dots:true},{p:t.travelled_path||[],color:'#f4fbff',dots:false}].filter(a=>a.p.length),all=paths.flatMap(a=>a.p).concat([{x:C(t.vehicle_x),y:C(t.vehicle_y)}]);let minX=ox,minY=oy,maxX=ox+mw,maxY=oy+mh;if(!mw||!mh){if(!all.length){x.fillStyle='#839bb9';x.font='12px Arial';x.fillText('Waiting for Gazebo map and route data',16,24);return}const xs=all.map(p=>p.x),ys=all.map(p=>p.y);minX=Math.min(...xs);maxX=Math.max(...xs);minY=Math.min(...ys);maxY=Math.max(...ys)}const sx=Math.max(maxX-minX,.01),sy=Math.max(maxY-minY,.01),scale=Math.min((r.width-48)/sx,(r.height-48)/sy),tx=p=>24+(p.x-minX)*scale,ty=p=>r.height-24-(p.y-minY)*scale;if(m.width&&m.height&&m.data?.length){const cw=scale*res,ch=scale*res;for(let row=0;row<m.height;row++)for(let col=0;col<m.width;col++){const v=m.data[row*m.width+col];x.fillStyle=v<0?'#26334a':v>=90?'#c74652':`hsl(${190-(Math.min(v,89)/89)*150},65%,${28+(1-Math.min(v,89)/89)*28}%)`;x.fillRect(tx({x:ox+col*res,y:oy+row*res+res}),ty({x:ox+col*res,y:oy+row*res+res}),Math.ceil(cw)+1,Math.ceil(ch)+1)}}paths.forEach(a=>{x.strokeStyle=a.color;x.lineWidth=a.dots?3:4;x.beginPath();a.p.forEach((p,i)=>i?x.lineTo(tx(p),ty(p)):x.moveTo(tx(p),ty(p)));x.stroke();if(a.dots){x.fillStyle=a.color;a.p.forEach(p=>{x.beginPath();x.arc(tx(p),ty(p),2.5,0,Math.PI*2);x.fill()})}});x.fillStyle='#fff';x.beginPath();x.arc(tx({x:C(t.vehicle_x),y:C(t.vehicle_y)}),6,0,Math.PI*2);x.fill()}
function drawCostMap(t){const c=E('cost-map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1,m=t.cost_map||{};c.width=r.width*d;c.height=r.height*d;const x=c.getContext('2d');x.setTransform(d,0,0,d,0,0);x.clearRect(0,0,r.width,r.height);if(!m.width||!m.height||!m.data?.length){x.fillStyle='#839bb9';x.font='12px Arial';x.fillText('Waiting for /terrain_costmap',16,24);return}const res=C(m.resolution)||1,ox=C(m.origin_x),oy=C(m.origin_y),mapW=C(m.width)*res,mapH=C(m.height)*res,pad={l:30,r:14,t:16,b:20},w=r.width-pad.l-pad.r,h=r.height-pad.t-pad.b,tx=p=>pad.l+(p.x-ox)/mapW*w,ty=p=>pad.t+(oy+mapH-p.y)/mapH*h,cw=w/m.width,ch=h/m.height;x.fillStyle='#101a30';x.fillRect(0,0,r.width,r.height);for(let row=0;row<m.height;row++)for(let col=0;col<m.width;col++){const v=m.data[row*m.width+col],px=pad.l+col*cw,py=pad.t+(m.height-row-1)*ch;x.fillStyle=terrainColor(v);x.fillRect(px,py,Math.ceil(cw)+.5,Math.ceil(ch)+.5);if(v>=90){x.strokeStyle='#111827';x.lineWidth=1;x.strokeRect(px+1,py+1,Math.max(1,cw-2),Math.max(1,ch-2))}}x.strokeStyle='rgba(238,243,255,.20)';x.setLineDash([5,4]);x.lineWidth=1.5;x.beginPath();x.moveTo(tx({x:0}),ty({y:0}));x.lineTo(tx({x:104}),ty({y:0}));x.stroke();x.setLineDash([]);const marker=(p,label,fill)=>{x.fillStyle=fill;x.strokeStyle='#111827';x.lineWidth=2;x.beginPath();x.arc(tx(p),ty(p),7,0,Math.PI*2);x.fill();x.stroke();x.fillStyle='#f4fbff';x.font='700 11px Arial';x.fillText(label,tx(p)-12,ty(p)-11)};marker({x:0,y:0},'HOME','#35e28b');marker({x:104,y:0},'DELIVERY','#f6d94c');const barrier=(xPos,label)=>{const left=tx({x:xPos-2}),right=tx({x:xPos+2}),top=ty({y:4}),bottom=ty({y:-4});x.strokeStyle='#f6e84c';x.lineWidth=2;x.strokeRect(left,top,right-left,bottom-top);x.fillStyle='#f6e84c';x.font='700 10px Arial';x.fillText(label,left-4,top-5)};barrier(35,'BARRIER A');barrier(68,'BARRIER B');x.fillStyle='#839bb9';x.font='10px Arial';x.fillText('map x (m)',r.width/2-22,r.height-5)}
function renderEvents(events){const el=E('mission-events');if(!el)return;el.innerHTML=(events||[]).slice().reverse().map(item=>`<div class="event-item"><span class="event-time">${item.time||'--:--:--'}</span>${item.event||'Unknown event'}</div>`).join('')||'<div class="event-item">Waiting for mission events…</div>'}
async function refresh(){try{const d=await(await fetch('/api/state')).json(),t=d.telemetry||{},fuel=t.fuel_percent,battery=C(d.battery_percent),reserve=C(d.return_margin_percent),speed=C(t.speed_mps),level=C(t.water_level_m),rate=C(t.tide_rate_m_per_minute),setRing=(id,value)=>{const el=E(id);if(el)el.style.strokeDashoffset=(339.3*(1-value/100)).toFixed(1)};setRemoteMode(!!d.remote_enabled);E('fuel').textContent=fuel==null?'N/A':C(fuel).toFixed(1)+'%';E('energy').textContent=battery.toFixed(1)+'%';E('reserve').textContent=reserve.toFixed(1)+'%';setRing('fuel-ring',fuel==null?0:C(fuel));setRing('energy-ring',battery);setRing('reserve-ring',reserve);E('speed').textContent=speed.toFixed(2)+' m/s';E('coordinates').textContent=C(t.vehicle_x).toFixed(2)+', '+C(t.vehicle_y).toFixed(2);E('safety').textContent=d.return_required?'RETURN':(d.mission_state||'HOLD');E('reason').textContent=d.reason||'—';E('tide-level').textContent=level.toFixed(2)+' m';E('tide-state').textContent=(t.tide_state||'UNKNOWN')+' · risk '+C(t.tide_risk).toFixed(1)+'%';E('connection').textContent='Live telemetry';E('camera-status').textContent='Live camera stream';T.push({level,rising:rate>=0});if(T.length>72)T.shift();drawTide();drawMap(t);drawCostMap(t);renderEvents(t.mission_events)}catch(e){E('connection').textContent='Telemetry unavailable'}}setInterval(refresh,1000);setInterval(()=>{const camera=E('camera');if(camera)camera.src='/api/camera.jpg?ts='+Date.now()},1000);refresh();
</script></body></html>"""

DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    ".camera{display:block;width:100%;aspect-ratio:16/9;object-fit:cover;background:#101a30;border-radius:8px}",
    ".camera{display:block;width:min(100%,640px);aspect-ratio:16/9;object-fit:cover;background:#101a30;border-radius:8px;margin:0 auto}",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "</style>",
    """.tech-only{display:none}.card{position:relative;overflow:hidden;background:linear-gradient(145deg,#111d36 0%,#0c152a 100%);border:1px solid #24496a;box-shadow:0 0 0 1px rgba(63,213,255,.04),0 12px 32px rgba(0,0,0,.25)}.card:before{content:'';position:absolute;inset:0;pointer-events:none;background:linear-gradient(115deg,rgba(63,213,255,.06),transparent 35%,rgba(210,73,255,.04));opacity:.9}.card>*{position:relative}.label{color:#63d9ff;font-weight:700;letter-spacing:1.6px}.sub,.live,.legend{color:#839bb9}.value{color:#f4fbff;text-shadow:0 0 12px rgba(74,221,255,.2)}.status{background:#142d43;border:1px solid #38d6ff;color:#78e5ff;box-shadow:0 0 14px rgba(56,214,255,.18);letter-spacing:1px}.eyebrow{color:#63d9ff;text-shadow:0 0 10px rgba(99,217,255,.55)}h1{letter-spacing:1px;text-transform:uppercase}.ring .track{stroke:#1c3853}.ring .progress{stroke:#63e6dc;filter:drop-shadow(0 0 5px rgba(99,230,220,.55))}.ring .reserve-progress{stroke:#ffc857;filter:drop-shadow(0 0 5px rgba(255,200,87,.45))}.map,.tide,.camera{border:1px solid #214967;box-shadow:inset 0 0 28px rgba(0,0,0,.35)}body{background-color:#070d1a;background-image:linear-gradient(rgba(40,112,150,.055) 1px,transparent 1px),linear-gradient(90deg,rgba(40,112,150,.055) 1px,transparent 1px),radial-gradient(circle at 50% -20%,#173a56 0%,#070d1a 56%);background-size:32px 32px,32px 32px,100% 100%}main{position:relative}main:after{content:'';position:fixed;inset:0;pointer-events:none;background:repeating-linear-gradient(0deg,rgba(255,255,255,.012) 0,rgba(255,255,255,.012) 1px,transparent 1px,transparent 4px);mix-blend-mode:screen;opacity:.25}.dot{ text-shadow:0 0 9px #35e28b}</style>""",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "</style>",
    ".remote-card{grid-column:1 / -1}.remote-toggle,.remote-abort{display:block;margin:12px auto;padding:9px 16px;border-radius:6px;border:1px solid #48d8ff;background:#10243a;color:#8ceaff;font-weight:700;cursor:pointer}.remote-toggle.active{background:#1e5c71;color:#fff}.remote-abort{border-color:#bd5360;background:#351e2a;color:#ff9ba4}.remote-pad{width:190px;height:190px;margin:14px auto;position:relative;border:1px solid #2d5370;border-radius:50%;background:radial-gradient(circle,#152941 0 31%,transparent 32%),linear-gradient(45deg,transparent 49%,#2d5370 50%,transparent 51%),linear-gradient(-45deg,transparent 49%,#2d5370 50%,transparent 51%);box-shadow:0 0 24px rgba(65,213,255,.1)}.remote-pad.disabled{opacity:.35;pointer-events:none}.remote-btn{position:absolute;width:42px;height:42px;border-radius:50%;border:1px solid #48d8ff;background:#10243a;color:#8ceaff;font-size:22px;cursor:pointer;box-shadow:0 0 10px rgba(72,216,255,.18)}.remote-btn:hover{background:#1d4c68;box-shadow:0 0 16px rgba(72,216,255,.45)}.remote-btn:active{transform:scale(.94)}.remote-up{top:8px;left:74px}.remote-down{bottom:8px;left:74px}.remote-left{left:8px;top:74px}.remote-right{right:8px;top:74px}.remote-stop{top:74px;left:74px;width:42px;height:42px;color:#ff8790;border-color:#bd5360;background:#351e2a;font-size:15px}.remote-status{text-align:center;color:#8da7c2;font-size:12px}.remote-note{text-align:center;color:#62809e;font-size:11px;margin-top:6px}</style>",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "grid-template-columns:repeat(3,1fr)",
    "grid-template-columns:repeat(6,1fr)",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "</style>",
    ".costmap{height:280px;display:block;width:100%;background:#101a30;border-radius:8px}.event-list{height:280px;overflow:auto;display:flex;flex-direction:column;gap:8px}.event-item{padding:8px 10px;border-left:2px solid #42d9ff;background:#101a30;color:#d7e6f7;font:12px ui-monospace,monospace}.event-time{color:#63d9ff;margin-right:8px}</style>",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    ".grid{grid-template-columns:1fr}.wide{grid-column:span 1}",
    ".grid{grid-template-columns:1fr}.grid .card{grid-column:1 / -1!important}",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "</main><script>",
    """</main><script>
const grid=document.querySelector('.grid'),cameraCard=document.querySelector('#camera').closest('.card'),remoteCard=document.querySelector('.remote-card'),costMapCard=document.querySelector('#cost-map').closest('.card'),eventsCard=document.querySelector('#mission-events').closest('.card');
const cards=[...grid.children],by=id=>document.querySelector(id).closest('.card');
const order=[by('#safety'),by('#speed'),by('#coordinates'),by('#fuel'),by('#energy'),by('#reserve'),by('#tide-level'),by('#tide-chart'),costMapCard,eventsCard,by('#map'),cameraCard,remoteCard];
order.forEach(card=>grid.appendChild(card));
order.forEach((card,index)=>{card.style.gridColumn=index===0?'1 / -1':(index===7||index===8?'span 4':(index===1||index===2?'span 3':'span 2'))});
""",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "</section></main><script>",
    """<article class="card remote-card"><div class="label">Control mode</div><button class="remote-toggle" id="remote-toggle">Switch to remote control</button><button class="remote-toggle" id="dispatch-goal">Dispatch demo delivery</button><div class="remote-pad disabled" id="remote-pad"><button class="remote-btn remote-up" data-remote="forward" aria-label="Forward">▲</button><button class="remote-btn remote-left" data-remote="left" aria-label="Left">◀</button><button class="remote-btn remote-stop" data-remote="stop" aria-label="Stop">■</button><button class="remote-btn remote-right" data-remote="right" aria-label="Right">▶</button><button class="remote-btn remote-down" data-remote="reverse" aria-label="Reverse">▼</button></div><div class="remote-status" id="remote-status">Autonomous control active</div><div class="remote-note">Hold a direction to drive. Remote commands remain safety-capped.</div><button class="remote-abort" id="remote-abort">Abort mission and return home</button></article></section></main><script>""",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "const E=id=>document.getElementById(id),C=x=>Number.isFinite(Number(x))?Number(x):0,T=[];",
    """const E=id=>document.getElementById(id),C=x=>Number.isFinite(Number(x))?Number(x):0,T=[],terrainColor=v=>v<0?'#26334a':v>=90?'#f52222':v<=19?'#d7bb78':v<=50?'#825b36':'#3987b7';let remoteMode=false;const postRemote=payload=>fetch('/api/remote',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(payload)}).then(r=>{if(!r.ok)throw new Error('request rejected')});const setRemoteMode=enabled=>{remoteMode=!!enabled;const toggle=E('remote-toggle'),pad=E('remote-pad'),status=E('remote-status');toggle.textContent=remoteMode?'Switch to autonomous control':'Switch to remote control';toggle.classList.toggle('active',remoteMode);pad.classList.toggle('disabled',!remoteMode);status.textContent=remoteMode?'Remote control active · hold a direction to drive':'Autonomous control active'};E('remote-toggle').addEventListener('click',()=>postRemote({kind:'mode',enabled:!remoteMode}).catch(()=>E('remote-status').textContent='Control-mode request failed'));E('dispatch-goal').addEventListener('click',()=>postRemote({kind:'goal',x:14.0,y:0.0}).then(()=>E('remote-status').textContent='Demo delivery goal dispatched').catch(()=>E('remote-status').textContent='Goal request failed'));const stopRemote=()=>postRemote({kind:'motion',action:'stop'}).catch(()=>{});document.querySelectorAll('[data-remote]').forEach(button=>{const action=button.dataset.remote;button.addEventListener('pointerdown',event=>{event.preventDefault();if(!remoteMode)return;if(action==='stop'){stopRemote();return}postRemote({kind:'motion',action}).catch(()=>E('remote-status').textContent='Remote command failed')});['pointerup','pointerleave','pointercancel'].forEach(event=>button.addEventListener(event,stopRemote))});window.addEventListener('blur',stopRemote);E('remote-abort').addEventListener('click',()=>postRemote({kind:'abort'}).then(()=>setRemoteMode(false)).catch(()=>E('remote-status').textContent='Abort request failed'));window.addEventListener('keydown',event=>{if(event.repeat||!remoteMode)return;const action={ArrowUp:'forward',ArrowDown:'reverse',ArrowLeft:'left',ArrowRight:'right'}[event.key];if(action){event.preventDefault();postRemote({kind:'motion',action})}});window.addEventListener('keyup',event=>{if(['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(event.key))stopRemote()});""",
)
# The tidal-corridor mission starts at HOME on the western bank (0, 0) and
# delivers to the marked firm eastern bank at (104, 0), across the tidal valley.
DASHBOARD_PAGE = DASHBOARD_PAGE.replace("x:14.0,y:0.0", "x:104.0,y:0.0")
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    "v<0?'#26334a':v>=90?'#c74652':`hsl(${190-(Math.min(v,89)/89)*150},65%,${28+(1-Math.min(v,89)/89)*28}%)`",
    "terrainColor(v)",
)
DASHBOARD_PAGE = DASHBOARD_PAGE.replace(
    '<article class="card remote-card">',
    '<article class="card"><div class="label">Terrain cost map</div><canvas id="cost-map" class="costmap"></canvas><div class="legend"><span style="color:#d7bb78">■ Firm: 10</span><span style="color:#825b36">■ Mud: 45</span><span style="color:#3987b7">■ Shallow water: 55</span><span class="red">■ Physical obstacle: 100</span></div></article><article class="card"><div class="label">Mission events</div><div id="mission-events" class="event-list"><div class="event-item">Waiting for mission events…</div></div></article><article class="card remote-card">',
)


class DashboardHandler(BaseHTTPRequestHandler):
    server_version = "TidalVehicleDashboard/1.0"
    _state = {"mission_state":"HOLD","effective_state":"HOLD","return_required":False,"battery_percent":0.0,"return_margin_percent":0.0,"reason":"Awaiting telemetry.","telemetry":{}}
    _camera_jpeg: bytes | None = None
    _remote_request_handler: Callable[[dict[str, Any]], bool] | None = None

    def do_GET(self) -> None:  # noqa: N802
        if self.path.split("?", 1)[0] == "/api/camera.jpg":
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
        if self.path == "/api/state":
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
