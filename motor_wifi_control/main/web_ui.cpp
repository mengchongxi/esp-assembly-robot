/*
 * web_ui.cpp – 多关节控制 HTML 页面
 * 使用 C++11 raw string literal 保留 HTML/CSS/JS 原始格式。
 */
#include "web_ui.h"
#include <cstring>

static const char HTML_PAGE[] = R"rawhtml(<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robot Arm Control</title>
  <style>
    *{box-sizing:border-box;margin:0;padding:0}
    body{font-family:'Segoe UI',sans-serif;background:#1a1a2e;color:#e0e0e0;
         display:flex;flex-direction:column;align-items:center;padding:12px;min-height:100vh}
    h1{font-size:1.4rem;margin:8px 0;color:#00d4ff;letter-spacing:2px}

    .top-bar{width:100%;max-width:1200px;margin-bottom:12px}
    .estop{width:100%;padding:14px;background:#dc2626;color:#fff;border:none;border-radius:12px;
           font-size:1.3rem;font-weight:bold;cursor:pointer;
           box-shadow:0 4px 16px rgba(220,38,38,.5);text-transform:uppercase}
    .estop:active{background:#991b1b;transform:scale(.97)}

    .columns{display:flex;gap:12px;width:100%;max-width:1200px;align-items:flex-start}
    .col{flex:1;min-width:0}

    .card{background:#16213e;border-radius:12px;padding:16px;width:100%;
          margin-bottom:10px;box-shadow:0 2px 12px rgba(0,0,0,.3)}
    .card h2{font-size:.85rem;color:#a0a0c0;margin-bottom:10px;text-transform:uppercase;letter-spacing:1px}

    .joint-row{display:flex;align-items:center;gap:8px;margin-bottom:8px;flex-wrap:wrap}
    .joint-label{font-size:.8rem;color:#a0a0c0;min-width:28px}
    .angle-val{font-size:1.6rem;font-weight:bold;color:#4ade80;min-width:80px;text-align:right}
    .target-input{width:72px;padding:8px;background:#0f3460;border:1px solid #1a4a8a;
                  border-radius:8px;color:#00d4ff;font-size:1rem;text-align:center;outline:none}
    .target-input:focus{border-color:#00d4ff}

    .btn-sm{padding:8px 12px;border:none;border-radius:8px;font-size:.85rem;font-weight:bold;
            cursor:pointer;transition:opacity .15s}
    .btn-sm:active{opacity:.7;transform:scale(.95)}
    .btn-go{background:#0f3460;color:#00d4ff}
    .btn-zero{background:#1e3a5f;color:#facc15}

    .param-row{display:flex;gap:6px;align-items:center;margin-bottom:6px;flex-wrap:wrap}
    .param-row label{font-size:.75rem;color:#a0a0c0;min-width:26px}
    .param-row input{width:58px;padding:6px;background:#0f3460;border:1px solid #1a4a8a;
                     border-radius:6px;color:#00d4ff;font-size:.85rem;text-align:center;outline:none}
    .param-row input:focus{border-color:#00d4ff}
    .btn-apply{padding:6px 14px;background:#065f46;color:#34d399;border:none;border-radius:6px;
               font-size:.8rem;cursor:pointer;font-weight:bold}
    .btn-apply:active{opacity:.7}

    .status-bar{font-size:.7rem;color:#666;margin-top:8px;text-align:center}

    @media(max-width:900px){
      .columns{flex-direction:column}
    }
  </style>
</head>
<body>
  <h1>&#129302; Robot Arm Control</h1>

  <div class="top-bar">
    <button class="estop" onclick="eStop()">&#9888; EMERGENCY STOP</button>
  </div>

  <div class="columns">
    <!-- 左列：关节控制 -->
    <div class="col">
      <div class="card">
        <h2>&#127918; Joint Control</h2>
        <div id="joints"></div>
      </div>
    </div>

    <!-- 中列：PD 调参 -->
    <div class="col">
      <div class="card">
        <h2>&#9881; PD Tuning</h2>
        <div id="pd-params"></div>
      </div>
    </div>

    <!-- 右列：限位设定 -->
    <div class="col">
      <div class="card">
        <h2>&#128207; Joint Limits</h2>
        <div id="limit-params"></div>
      </div>
    </div>
  </div>

  <div class="status-bar" id="statusBar">Connecting...</div>

  <script>
    var N=5;

    function buildUI(){
      var jh='';
      for(var i=0;i<N;i++){
        jh+='<div class="joint-row">'
          +'<span class="joint-label">J'+(i+1)+'</span>'
          +'<span class="angle-val" id="cur'+i+'">--</span>'
          +'<input class="target-input" type="number" id="tgt'+i+'" step="0.5" value="0">'
          +'<button class="btn-sm btn-go" onclick="goJoint('+i+')">GO</button>'
          +'<button class="btn-sm btn-zero" onclick="setZero('+i+')">Zero</button>'
          +'</div>';
      }
      document.getElementById('joints').innerHTML=jh;

      var ph='';
      for(var i=0;i<N;i++){
        ph+='<div style="margin-bottom:10px"><span class="joint-label" style="font-weight:bold">J'+(i+1)+'</span>'
          +'<div class="param-row">'
          +'<label>Kp</label><input id="kp'+i+'" type="number" step="0.1" value="2.0">'
          +'<label>Kd</label><input id="kd'+i+'" type="number" step="0.1" value="0.5">'
          +'<label>Ki</label><input id="ki'+i+'" type="number" step="0.01" value="0">'
          +'</div><div class="param-row">'
          +'<label>DZ</label><input id="dz'+i+'" type="number" step="0.1" value="1.0">'
          +'<label>mP</label><input id="mp'+i+'" type="number" step="1" value="20">'
          +'<button class="btn-apply" onclick="setPD('+i+')">Apply</button>'
          +'</div></div>';
      }
      document.getElementById('pd-params').innerHTML=ph;

      var lh='';
      for(var i=0;i<N;i++){
        lh+='<div class="param-row">'
          +'<span class="joint-label">J'+(i+1)+'</span>'
          +'<label>Min</label><input id="lmin'+i+'" type="number" value="-360">'
          +'<label>Max</label><input id="lmax'+i+'" type="number" value="360">'
          +'<button class="btn-apply" onclick="setLimits('+i+')">Set</button>'
          +'</div>';
      }
      document.getElementById('limit-params').innerHTML=lh;
    }

    function eStop(){
      fetch('/stop').then(function(){
        document.getElementById('statusBar').innerText='EMERGENCY STOP!';
      });
    }

    function goJoint(i){
      var t=document.getElementById('tgt'+i).value;
      fetch('/joint?id='+i+'&target='+t);
    }

    function setZero(i){
      if(confirm('Set joint '+(i+1)+' current position as zero?'))
        fetch('/zero?id='+i);
    }

    function setPD(i){
      var kp=document.getElementById('kp'+i).value;
      var kd=document.getElementById('kd'+i).value;
      var ki=document.getElementById('ki'+i).value;
      var dz=document.getElementById('dz'+i).value;
      var mp=document.getElementById('mp'+i).value;
      fetch('/pd?id='+i+'&kp='+kp+'&kd='+kd+'&ki='+ki+'&dead='+dz+'&min_pwm='+mp);
    }

    function setLimits(i){
      var mn=document.getElementById('lmin'+i).value;
      var mx=document.getElementById('lmax'+i).value;
      fetch('/limits?id='+i+'&min='+mn+'&max='+mx);
    }

    function updateStatus(d){
      if(!d.joints)return;
      for(var i=0;i<d.joints.length;i++){
        var j=d.joints[i];
        var el=document.getElementById('cur'+j.id);
        if(el) el.innerText=j.current.toFixed(1)+'\u00b0';
      }
      document.getElementById('statusBar').innerText='Connected \u2022 '+new Date().toLocaleTimeString();
    }

    function pollStatus(){
      fetch('/status')
        .then(function(r){return r.json();})
        .then(updateStatus)
        .catch(function(){
          document.getElementById('statusBar').innerText='Disconnected';
        });
    }

    buildUI();
    setInterval(pollStatus,200);
    pollStatus();
  </script>
</body>
</html>
)rawhtml";

extern "C" esp_err_t web_ui_send_html(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, HTML_PAGE, strlen(HTML_PAGE));
}
