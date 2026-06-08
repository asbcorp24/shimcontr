#include "web_ui.h"

#include <WiFi.h>
#include <WebServer.h>

static WebServer server(80);
static WebUiGetJsonFn gGetStatus = nullptr;
static WebUiGetJsonFn gGetRules = nullptr;
static WebUiSetJsonFn gSetRules = nullptr;
static WebUiActionFn gSaveRules = nullptr;
static WebUiActionFn gLoadRules = nullptr;
static const char* gApSsid = nullptr;
static const char* gApPass = nullptr;
static bool gWebEnabled = false;
static bool gServerStarted = false;

static const char* INDEX_HTML = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Rules UI</title>
  <style>
    :root{--bg:#f1f5f9;--card:#fff;--line:#dbe3ee;--txt:#0f172a;--muted:#475569;--pri:#0ea5e9;--ok:#16a34a;--warn:#dc2626}
    body{font-family:Segoe UI,Tahoma,sans-serif;margin:0;background:var(--bg);color:var(--txt)}
    .wrap{max-width:1180px;margin:20px auto;padding:0 12px}
    .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:12px}
    h1,h2,h3{margin:0 0 8px}
    .muted{color:var(--muted);font-size:14px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
    button{padding:8px 12px;border:0;border-radius:10px;background:var(--pri);color:#fff;cursor:pointer}
    button.gray{background:#64748b}
    button.green{background:var(--ok)}
    button.red{background:var(--warn)}
    button.small{padding:6px 9px;font-size:12px}
    select,input{padding:7px 8px;border:1px solid var(--line);border-radius:8px}
    pre{background:#0b1220;color:#dbeafe;padding:10px;border-radius:8px;overflow:auto}
    table{width:100%;border-collapse:collapse;font-size:14px}
    th,td{border-bottom:1px solid var(--line);padding:6px;text-align:left;vertical-align:top}
    .chip{padding:4px 8px;border-radius:999px;background:#e2e8f0;font-size:12px}
    .preset{border:1px solid var(--line);border-radius:10px;padding:10px}
    .preset p{margin:6px 0;font-size:13px;color:var(--muted)}
    @media (max-width:900px){.grid{grid-template-columns:1fr}}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Настройка правил ESP32</h1>
      <div class="muted">Редактор правил без ручного JSON. Выберите канал, добавьте правила, примените их на устройство и сохраните в память.</div>
      <div class="row" style="margin-top:10px">
        <button onclick="refreshStatus()">Обновить статус</button>
        <button onclick="fetchRules()">Загрузить правила</button>
        <button class="green" onclick="applyRules()">Применить на устройство</button>
        <button class="gray" onclick="saveRules()">Сохранить в NVS</button>
        <button class="gray" onclick="loadRulesFromNvs()">Загрузить из NVS</button>
        <button class="gray" onclick="location.href='/help'">Help / Инструкция</button>
        <button class="gray" onclick="location.href='/wiring'">Схемы / Подключение</button>
        <span id="msg" class="chip">Готово</span>
      </div>
    </div>

    <div class="grid">
      <div class="card">
        <div class="row">
          <label>Канал:</label>
          <select id="channelSel" onchange="renderRules()"></select>
          <button onclick="addRule()">Добавить правило</button>
        </div>
        <table style="margin-top:10px">
          <thead>
            <tr><th>#</th><th>Режим</th><th>Условие</th><th>A</th><th>B</th><th>OutA</th><th>OutB</th><th>BTS%</th><th>Цель</th><th></th></tr>
          </thead>
          <tbody id="rulesBody"></tbody>
        </table>
      </div>

      <div class="card">
        <h3>Готовые пресеты</h3>
        <div class="muted">Кнопки ниже добавляют готовые правила в текущий выбранный канал. После вставки можно сразу подправить пороги, цель и диапазоны.</div>
        <div class="grid" style="margin-top:10px">
          <div class="preset">
            <h3>Relay вверх</h3>
            <p>Включает реле только выше порога.</p>
            <button class="small" onclick="presetRelayHigh()">Вставить</button>
          </div>
          <div class="preset">
            <h3>Relay центр</h3>
            <p>Включает реле только в окне 1450..1550.</p>
            <button class="small" onclick="presetRelayCenter()">Вставить</button>
          </div>
          <div class="preset">
            <h3>PCA 1:1</h3>
            <p>Полный ход 1000..2000 в такой же выходной диапазон.</p>
            <button class="small" onclick="presetPcaLinear()">Вставить</button>
          </div>
          <div class="preset">
            <h3>PCA растянуть</h3>
            <p>Узкий вход 1200..1600 растягивается до 1000..2000.</p>
            <button class="small" onclick="presetPcaStretch()">Вставить</button>
          </div>
          <div class="preset">
            <h3>BTS вперед 100%</h3>
            <p>Если условие выполнилось, на выход уйдет +100%.</p>
            <button class="small" onclick="presetBtsForward100()">Вставить</button>
          </div>
          <div class="preset">
            <h3>BTS назад 100%</h3>
            <p>Если условие выполнилось, на выход уйдет -100%.</p>
            <button class="small" onclick="presetBtsReverse100()">Вставить</button>
          </div>
          <div class="preset">
            <h3>BTS стоп</h3>
            <p>Если условие выполнилось, на выход уйдет 0%.</p>
            <button class="small" onclick="presetBtsStop()">Вставить</button>
          </div>
          <div class="preset">
            <h3>BTS вперед / стоп / назад</h3>
            <p>Сразу вставляет 3 фиксированных правила на один BTS: верх, центр, низ.</p>
            <button class="small" onclick="presetBtsFwdStopRev()">Вставить</button>
          </div>
          <div class="preset">
            <h3>BLDC PWM + DIR</h3>
            <p>Вставляет набор для BLDC: газ через PCA, направление через Relay.</p>
            <button class="small" onclick="presetBldcPwmDir()">Вставить</button>
          </div>
          <div class="preset">
            <h3>Polarity по каналу</h3>
            <p>Переключение реле полярности по ходу канала.</p>
            <button class="small" onclick="presetPolarityBasic()">Вставить</button>
          </div>
        </div>
      </div>
    </div>

    <div class="card">
      <h3>Таблица целей и физических выходов</h3>
      <div class="muted">Эта таблица помогает быстро понять, на какой реальный пин или канал уйдет правило.</div>
      <table style="margin-top:10px">
        <thead><tr><th>Цель</th><th>Тип</th><th>Физический канал / пин</th><th>Назначение</th></tr></thead>
        <tbody>
          <tr><td>PCA 0..15</td><td>PCA9685</td><td>I2C модуль PCA9685, каналы CH0..CH15</td><td>Плавный выход ШИМ / сервосигнал</td></tr>
          <tr><td>BTS 0</td><td>BTS7960</td><td>RPWM GPIO25, LPWM GPIO26</td><td>Мотор 0</td></tr>
          <tr><td>BTS 1</td><td>BTS7960</td><td>RPWM GPIO27, LPWM GPIO14</td><td>Мотор 1</td></tr>
          <tr><td>BTS 2</td><td>BTS7960</td><td>RPWM GPIO12, LPWM GPIO13</td><td>Мотор 2</td></tr>
          <tr><td>BTS 3</td><td>BTS7960</td><td>RPWM GPIO32, LPWM GPIO33</td><td>Мотор 3</td></tr>
          <tr><td>Relay 0</td><td>MCP23017</td><td>A4, pin 4</td><td>Обычное реле 0</td></tr>
          <tr><td>Relay 1</td><td>MCP23017</td><td>A3, pin 3</td><td>Обычное реле 1</td></tr>
          <tr><td>Relay 2</td><td>MCP23017</td><td>A2, pin 2</td><td>Обычное реле 2</td></tr>
          <tr><td>Relay 3</td><td>MCP23017</td><td>A1, pin 1</td><td>Обычное реле 3</td></tr>
          <tr><td>Polarity 0</td><td>MCP23017</td><td>ON B2 pin 10, DIR B3 pin 11</td><td>Реле полярности 0</td></tr>
          <tr><td>Polarity 1</td><td>MCP23017</td><td>ON B4 pin 12, DIR B5 pin 13</td><td>Реле полярности 1</td></tr>
          <tr><td>Polarity 2</td><td>MCP23017</td><td>ON B6 pin 14, DIR B7 pin 15</td><td>Реле полярности 2</td></tr>
        </tbody>
      </table>
    </div>

    <div class="card">
      <div class="row" style="justify-content:space-between">
        <h3 style="margin:0">Статус</h3>
        <label class="row" style="gap:6px">
          <input id="autoRefresh" type="checkbox" checked onchange="toggleAuto()">
          Автообновление 1000мс
        </label>
      </div>
      <div class="row" style="margin-top:10px">
        <div style="flex:1;min-width:320px">
          <div class="muted" style="margin-bottom:6px">Входы</div>
          <table>
            <thead><tr><th>Канал</th><th>PWM, мкс</th><th>Возраст, мс</th><th>Сигнал</th></tr></thead>
            <tbody id="inputsBody"></tbody>
          </table>
        </div>
        <div style="flex:1;min-width:320px">
          <div class="muted" style="margin-bottom:6px">Выходы</div>
          <table>
            <thead><tr><th>Выход</th><th>Active</th><th>v01</th><th>vs</th><th>Итог</th></tr></thead>
            <tbody id="outputsBody"></tbody>
          </table>
        </div>
      </div>
    </div>

    <div class="card">
      <details>
        <summary>Показать JSON</summary>
        <pre id="status">...</pre>
        <pre id="jsonView"></pre>
      </details>
    </div>
  </div>
<script>
const MODE = ["Relay","Polarity","Engine","BTS"];
const COND = ["ANY","Меньше","Больше","Между"];
let cfg = { rules: [[],[],[],[],[],[],[],[]] };
let autoTimer = null;
let lastStatus = null;
let statusRequestInFlight = false;

function msg(t){ document.getElementById('msg').textContent = t; }
function opt(list,v){ return list.map((x,i)=>`<option value="${i}" ${i===v?'selected':''}>${x}</option>`).join(''); }
function targetLabel(t){
  t = Number(t);
  if (t <= 15) return `PCA ${t}`;
  if (t <= 19) return `BTS ${t-16}`;
  if (t <= 23) return `Relay ${t-20}`;
  return `Polarity ${t-24}`;
}
function targetOptions(v){
  let s = "";
  for(let i=0;i<=26;i++){
    s += `<option value="${i}" ${i===Number(v)?'selected':''}>${targetLabel(i)}</option>`;
  }
  return s;
}
function modeForTarget(t){
  t = Number(t);
  if (t <= 15) return 2;
  if (t <= 19) return 3;
  if (t <= 23) return 0;
  return 1;
}
function isPcaTarget(t){ return Number(t) <= 15; }
function isBtsTarget(t){ t = Number(t); return t >= 16 && t <= 19; }
function currentChannel(){ return Number(document.getElementById('channelSel').value||0); }
function currentInputUs(ch){
  if (!lastStatus || !lastStatus.inputs || !lastStatus.inputs[ch]) return null;
  return Number(lastStatus.inputs[ch].us);
}
function mapRangeClamped(x, inA, inB, outA, outB){
  if (Math.abs(inB - inA) < 0.001) return outA;
  let t = (x - inA) / (inB - inA);
  t = Math.max(0, Math.min(1, t));
  return outA + (outB - outA) * t;
}
function sanitizeRule(r){
  const tgt = Math.min(26, Math.max(0, Number(r.tgt||0)));
  return {
    type: Math.min(2, Math.max(0, Number(r.type||0))),
    cond: Math.min(3, Math.max(0, Number(r.cond||0))),
    a: Math.min(2100, Math.max(900, Number(r.a||1500))),
    b: Math.min(2500, Math.max(900, Number(r.b||2000))),
    outA: Math.min(4095, Math.max(0, Number(r.outA||1000))),
    outB: Math.min(4095, Math.max(0, Number(r.outB||2000))),
    bts: Math.min(100, Math.max(-100, Number((r.bts !== undefined ? r.bts : r.btsPct) || 0))),
    tgt: tgt,
    mode: modeForTarget(tgt)
  };
}
function makeRule(p){
  return sanitizeRule(Object.assign({cond:0,a:1500,b:2000,outA:1000,outB:2000,bts:0,tgt:0,mode:2}, p||{}));
}
function pushRule(rule){
  const ch = currentChannel();
  cfg.rules[ch].push(makeRule(rule));
}
function updateHints(){
  const ch = currentChannel();
  const arr = cfg.rules[ch] || [];
  arr.forEach((r,idx)=>{
    const pca = isPcaTarget(r.tgt);
    const bts = isBtsTarget(r.tgt);
    if (!pca && !bts) return;
    const el = document.getElementById(`hint-${ch}-${idx}`);
    if (!el) return;
    const curUs = currentInputUs(ch);
    let hintText = "";
    if (pca) {
      hintText = `A..B ${r.a}..${r.b} -> OutA..OutB ${r.outA}..${r.outB}`;
      if (curUs !== null && curUs >= 900 && curUs <= 2100) {
        const mapped = Math.round(mapRangeClamped(curUs, Number(r.a), Number(r.b), Number(r.outA), Number(r.outB)));
        hintText += ` | ${curUs} -> ${mapped}`;
      }
    } else if (bts) {
      hintText = `При выполнении условия -> BTS ${r.bts}%`;
    }
    el.textContent = hintText;
  });
}
function initChannels(){
  const s = document.getElementById('channelSel');
  s.innerHTML = "";
  for(let i=0;i<8;i++){
    const o = document.createElement('option');
    o.value = i;
    o.textContent = `Канал ${i+1}`;
    s.appendChild(o);
  }
}
function renderRules(){
  const ch = currentChannel();
  const body = document.getElementById('rulesBody');
  body.innerHTML = "";
  const arr = cfg.rules[ch] || [];
  arr.forEach((r,idx)=>{
    const pca = isPcaTarget(r.tgt);
    const bts = isBtsTarget(r.tgt);
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${idx+1}</td>
      <td><select onchange="upd(${ch},${idx},'mode',this.value)" disabled>${opt(MODE, Number(r.mode))}</select></td>
      <td><select onchange="upd(${ch},${idx},'cond',this.value)">${opt(COND, Number(r.cond))}</select></td>
      <td><input type="number" min="900" max="2100" value="${r.a}" onchange="upd(${ch},${idx},'a',this.value)"></td>
      <td><input type="number" min="900" max="2500" value="${r.b}" onchange="upd(${ch},${idx},'b',this.value)"></td>
      <td><input type="number" min="0" max="4095" value="${r.outA}" ${pca ? "" : "disabled"} onchange="upd(${ch},${idx},'outA',this.value)"></td>
      <td><input type="number" min="0" max="4095" value="${r.outB}" ${pca ? "" : "disabled"} onchange="upd(${ch},${idx},'outB',this.value)"></td>
      <td><input type="number" min="-100" max="100" value="${r.bts}" ${bts ? "" : "disabled"} onchange="upd(${ch},${idx},'bts',this.value)"></td>
      <td><select onchange="upd(${ch},${idx},'tgt',this.value)">${targetOptions(r.tgt)}</select></td>
      <td>
        <button class="gray small" onclick="moveRule(${ch},${idx},-1)">↑</button>
        <button class="gray small" onclick="moveRule(${ch},${idx},1)">↓</button>
        <button class="red small" onclick="delRule(${ch},${idx})">Удалить</button>
      </td>`;
    body.appendChild(tr);
    if (pca || bts) {
      const hint = document.createElement('tr');
      hint.innerHTML = `<td id="hint-${ch}-${idx}" colspan="10" style="color:#475569;font-size:12px;padding-top:0"></td>`;
      body.appendChild(hint);
    }
  });
  document.getElementById('jsonView').textContent = JSON.stringify(cfg, null, 2);
  updateHints();
}
function upd(ch,idx,key,val){
  cfg.rules[ch][idx][key] = Number(val);
  if (key === 'tgt') cfg.rules[ch][idx].mode = modeForTarget(Number(val));
  cfg.rules[ch][idx] = sanitizeRule(cfg.rules[ch][idx]);
  renderRules();
}
function addRule(){
  pushRule({});
  renderRules();
}
function delRule(ch,idx){
  cfg.rules[ch].splice(idx,1);
  renderRules();
}
function moveRule(ch, idx, dir){
  const arr = cfg.rules[ch];
  const ni = idx + dir;
  if (ni < 0 || ni >= arr.length) return;
  const t = arr[idx];
  arr[idx] = arr[ni];
  arr[ni] = t;
  renderRules();
}
function presetRelayHigh(){ pushRule({cond:2,a:1700,tgt:20}); renderRules(); msg("Добавлен пресет Relay вверх"); }
function presetRelayCenter(){ pushRule({cond:3,a:1450,b:1550,tgt:20}); renderRules(); msg("Добавлен пресет Relay центр"); }
function presetPcaLinear(){ pushRule({cond:0,a:1000,b:2000,outA:1000,outB:2000,tgt:0}); renderRules(); msg("Добавлен пресет PCA 1:1"); }
function presetPcaStretch(){ pushRule({cond:0,a:1200,b:1600,outA:1000,outB:2000,tgt:0}); renderRules(); msg("Добавлен пресет PCA растянуть"); }
function presetBtsForward100(){ pushRule({cond:2,a:1700,bts:100,tgt:16}); renderRules(); msg("Добавлен пресет BTS вперед 100%"); }
function presetBtsReverse100(){ pushRule({cond:1,a:1300,bts:-100,tgt:16}); renderRules(); msg("Добавлен пресет BTS назад 100%"); }
function presetBtsStop(){ pushRule({cond:3,a:1400,b:1600,bts:0,tgt:16}); renderRules(); msg("Добавлен пресет BTS стоп"); }
function presetBtsFwdStopRev(){
  pushRule({cond:2,a:1700,bts:100,tgt:16});
  pushRule({cond:3,a:1400,b:1600,bts:0,tgt:16});
  pushRule({cond:1,a:1300,bts:-100,tgt:16});
  renderRules();
  msg("Добавлен пресет BTS вперед / стоп / назад");
}
function presetBldcPwmDir(){
  pushRule({cond:2,a:1600,b:2000,outA:1000,outB:2000,tgt:0});
  pushRule({cond:1,a:1400,b:1000,outA:1000,outB:2000,tgt:0});
  pushRule({cond:2,a:1600,tgt:20});
  pushRule({cond:1,a:1400,tgt:20});
  renderRules();
  msg("Добавлен пресет BLDC PWM + DIR");
}
function presetPolarityBasic(){ pushRule({cond:0,tgt:24}); renderRules(); msg("Добавлен пресет Polarity"); }
async function refreshStatus(){
  if (statusRequestInFlight) return;
  statusRequestInFlight = true;
  try{
    const r = await fetch('/api/status', { cache: 'no-store' });
    const txt = await r.text();
    document.getElementById('status').textContent = txt;
    const st = JSON.parse(txt);
    lastStatus = st;
    renderStatusTables(st);
    updateHints();
  }catch(e){}
  finally{
    statusRequestInFlight = false;
  }
}
function renderStatusTables(st){
  const inBody = document.getElementById('inputsBody');
  const outBody = document.getElementById('outputsBody');
  inBody.innerHTML = "";
  outBody.innerHTML = "";
  (st.inputs || []).forEach(i=>{
    const ok = Number(i.ageMs) < 120 && Number(i.us) >= 900 && Number(i.us) <= 2100;
    const tr = document.createElement('tr');
    tr.innerHTML = `<td>${Number(i.ch)+1}</td><td>${i.us}</td><td>${i.ageMs}</td><td>${ok ? "OK":"LOSS"}</td>`;
    inBody.appendChild(tr);
  });
  (st.outputs || []).forEach(o=>{
    if (Number(o.idx) > 26) return;
    const tr = document.createElement('tr');
    let eff = "-";
    if (Number(o.idx) <= 15 && o.pcaUs !== undefined) eff = `${o.pcaUs} us / ${o.pca12}`;
    tr.innerHTML = `<td>${targetLabel(o.idx)}</td><td>${o.active ? "1":"0"}</td><td>${Number(o.v01).toFixed(2)}</td><td>${Number(o.vs).toFixed(2)}</td><td>${eff}</td>`;
    outBody.appendChild(tr);
  });
}
function toggleAuto(){
  const on = document.getElementById('autoRefresh').checked;
  if (autoTimer) { clearInterval(autoTimer); autoTimer = null; }
  if (on) autoTimer = setInterval(refreshStatus, 1000);
}
async function fetchRules(){
  const r = await fetch('/api/rules');
  const t = await r.text();
  cfg = JSON.parse(t);
  for(let ch=0; ch<8; ch++) cfg.rules[ch] = (cfg.rules[ch]||[]).map(sanitizeRule);
  renderRules();
  msg("Правила загружены");
}
async function applyRules(){
  const r = await fetch('/api/rules',{
    method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(cfg)
  });
  msg(await r.text());
}
async function saveRules(){
  const r = await fetch('/api/save',{method:'POST'});
  msg(await r.text());
}
async function loadRulesFromNvs(){
  const r = await fetch('/api/load',{method:'POST'});
  msg(await r.text());
  await fetchRules();
}
initChannels();
refreshStatus();
fetchRules();
toggleAuto();
</script>
</body>
</html>
)HTML";

static const char* HELP_HTML = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Help</title>
  <style>
    body{font-family:Segoe UI,Tahoma,sans-serif;margin:0;background:#f1f5f9;color:#0f172a}
    .wrap{max-width:980px;margin:20px auto;padding:0 12px}
    .card{background:#fff;border:1px solid #dbe3ee;border-radius:12px;padding:14px;margin-bottom:12px}
    h1,h2,h3{margin:0 0 10px}
    p{margin:8px 0;line-height:1.45}
    code{background:#e2e8f0;padding:2px 6px;border-radius:6px}
    .btn{display:inline-block;padding:8px 12px;background:#0ea5e9;color:#fff;text-decoration:none;border-radius:10px}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Инструкция по настройке правил</h1>
      <a class="btn" href="/">Назад в настройки</a>
      <a class="btn" href="/wiring" style="margin-left:8px">Схемы подключения</a>
    </div>
    <div class="card">
      <h2>Что делает прибор</h2>
      <p>ESP32 читает 8 входных каналов приемника. Для каждого канала можно создать несколько правил. Если условие правила выполняется, правило управляет выбранным выходом.</p>
      <p>Поддерживаются выходы: <code>PCA 0..15</code>, <code>BTS 0..3</code>, <code>Relay 0..3</code>, <code>Polarity 0..2</code>.</p>
      <p>Если несколько правил пишут в один и тот же выход, побеждает последнее сработавшее правило в списке.</p>
    </div>
    <div class="card">
      <h2>Поля правила</h2>
      <p><b>Режим:</b> ставится автоматически по цели.</p>
      <p><b>Условие:</b> <code>ANY</code>, <code>Меньше A</code>, <code>Больше A</code>, <code>Между A и B</code>.</p>
      <p><b>A/B:</b> входной диапазон PWM. Обычно <code>1000</code> минимум, <code>1500</code> центр, <code>2000</code> максимум.</p>
      <p><b>OutA/OutB:</b> используются для PCA. Диапазон входа <code>A..B</code> преобразуется в выход <code>OutA..OutB</code>.</p>
      <p><b>BTS%:</b> используется для BTS. Если условие выполнилось, на выход уходит именно это фиксированное значение от <code>-100</code> до <code>100</code>.</p>
      <p><b>Цель:</b> <code>0..15 PCA</code>, <code>16..19 BTS</code>, <code>20..23 Relay</code>, <code>24..26 Polarity</code>.</p>
    </div>
    <div class="card">
      <h2>Как настраивать</h2>
      <p>1. Выберите входной канал.</p>
      <p>2. Добавьте правило вручную или вставьте готовый пресет.</p>
      <p>3. Выберите цель и проверьте, что режим подставился правильно.</p>
      <p>4. Настройте условие, пороги A/B и значения OutA/OutB или BTS%.</p>
      <p>5. Нажмите "Применить на устройство".</p>
      <p>6. Проверьте результат в блоке "Статус".</p>
      <p>7. Если все правильно, нажмите "Сохранить в NVS".</p>
    </div>
    <div class="card">
      <h2>Relay</h2>
      <p>Relay это дискретный выход: включено или выключено.</p>
      <p><b>Примеры:</b></p>
      <p>Верх стика: cond=Больше, A=1700, tgt=Relay 0.</p>
      <p>Низ стика: cond=Меньше, A=1300, tgt=Relay 1.</p>
      <p>Центр стика: cond=Между, A=1450, B=1550, tgt=Relay 2.</p>
    </div>
    <div class="card">
      <h2>PCA</h2>
      <p>PCA используется как плавный ШИМ или сервосигнал. Здесь важны поля <code>A</code>, <code>B</code>, <code>OutA</code>, <code>OutB</code>.</p>
      <p><b>Примеры:</b></p>
      <p>Полный диапазон 1:1: cond=ANY, A=1000, B=2000, OutA=1000, OutB=2000, tgt=PCA 0.</p>
      <p>Растянуть узкий вход: cond=ANY, A=1200, B=1600, OutA=1000, OutB=2000, tgt=PCA 1.</p>
      <p>Сжать широкий вход: cond=ANY, A=1000, B=2000, OutA=1300, OutB=1700, tgt=PCA 2.</p>
      <p>Инверсия: cond=ANY, A=1000, B=2000, OutA=2000, OutB=1000, tgt=PCA 3.</p>
    </div>
    <div class="card">
      <h2>BTS</h2>
      <p>BTS используется для управления мотором через BTS7960. Если условие выполнилось, на выход уходит фиксированное значение BTS%.</p>
      <p><b>Как выбирать BTS%:</b> <code>100</code> это вперед на 100%, <code>-100</code> это назад на 100%, <code>0</code> это стоп.</p>
      <p><b>Примеры:</b></p>
      <p>Вперед: cond=Больше, A=1700, BTS%=100, tgt=BTS 0.</p>
      <p>Назад: cond=Меньше, A=1300, BTS%=-100, tgt=BTS 0.</p>
      <p>Стоп: cond=Между, A=1400, B=1600, BTS%=0, tgt=BTS 0.</p>
      <p>Для набора вперед / стоп / назад используйте три отдельных правила на один BTS.</p>
    </div>
    <div class="card">
      <h2>Polarity</h2>
      <p>Polarity это выход для реле полярности. Он удобен там, где нужно менять направление или полярность.</p>
      <p><b>Примеры:</b></p>
      <p>По всему ходу: cond=ANY, tgt=Polarity 0.</p>
      <p>Только сверху: cond=Больше, A=1600, tgt=Polarity 1.</p>
      <p>Только снизу: cond=Меньше, A=1400, tgt=Polarity 2.</p>
    </div>
    <div class="card">
      <h2>BLDC через PWM + DIR</h2>
      <p>Если у BLDC-контроллера есть вход газа <code>PWM</code> и отдельный вход направления <code>DIR</code>, удобно использовать <code>PCA</code> для газа и <code>Relay</code> или <code>Polarity</code> для направления.</p>
      <p>Газ вперед: cond=Больше, A=1600, B=2000, OutA=1000, OutB=2000, tgt=PCA 0.</p>
      <p>Газ назад: cond=Меньше, A=1400, B=1000, OutA=1000, OutB=2000, tgt=PCA 0.</p>
      <p>Направление вперед: cond=Больше, A=1600, tgt=Relay 0.</p>
      <p>Направление назад: cond=Меньше, A=1400, tgt=Relay 0.</p>
      <p>Безопаснее делать центральную стоп-зону, а направление менять только за ее пределами.</p>
    </div>
    <div class="card">
      <h2>Таблица целей</h2>
      <p>PCA 0..15: каналы PCA9685 CH0..CH15.</p>
      <p>BTS 0: GPIO25 и GPIO26.</p>
      <p>BTS 1: GPIO27 и GPIO14.</p>
      <p>BTS 2: GPIO12 и GPIO13.</p>
      <p>BTS 3: GPIO32 и GPIO33.</p>
      <p>Relay 0..3: MCP23017 A4, A3, A2, A1.</p>
      <p>Polarity 0..2: MCP23017 B2/B3, B4/B5, B6/B7.</p>
    </div>
  </div>
</body>
</html>
)HTML";

static const char* WIRING_HTML = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 Wiring</title>
  <style>
    body{font-family:Segoe UI,Tahoma,sans-serif;margin:0;background:#f1f5f9;color:#0f172a}
    .wrap{max-width:980px;margin:20px auto;padding:0 12px}
    .card{background:#fff;border:1px solid #dbe3ee;border-radius:12px;padding:14px;margin-bottom:12px}
    h1,h2{margin:0 0 10px}
    p{margin:8px 0;line-height:1.45}
    code{background:#e2e8f0;padding:2px 6px;border-radius:6px}
    .btn{display:inline-block;padding:8px 12px;background:#0ea5e9;color:#fff;text-decoration:none;border-radius:10px}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Схемы подключения</h1>
      <a class="btn" href="/">Назад в настройки</a>
      <a class="btn" href="/help" style="margin-left:8px">Help / Инструкция</a>
    </div>
    <div class="card">
      <h2>Общие линии ESP32</h2>
      <p><b>I2C:</b> <code>SDA GPIO21</code>, <code>SCL GPIO22</code>.</p>
      <p><b>Входы приемника:</b> CH1 GPIO16, CH2 GPIO17, CH3 GPIO18, CH4 GPIO19, CH5 GPIO23, CH6 GPIO4, CH7 GPIO5, CH8 GPIO34.</p>
    </div>
    <div class="card">
      <h2>BTS7960</h2>
      <p><b>BTS 0:</b> RPWM GPIO25, LPWM GPIO26.</p>
      <p><b>BTS 1:</b> RPWM GPIO27, LPWM GPIO14.</p>
      <p><b>BTS 2:</b> RPWM GPIO12, LPWM GPIO13.</p>
      <p><b>BTS 3:</b> RPWM GPIO32, LPWM GPIO33.</p>
      <p>Подключение: входы RPWM и LPWM драйвера подключаются к указанным GPIO ESP32. Земля ESP32 и драйвера должна быть общей.</p>
    </div>
    <div class="card">
      <h2>BLDC через PWM + DIR</h2>
      <p><b>PWM газа:</b> берите с любого канала <code>PCA 0..15</code> на модуле PCA9685.</p>
      <p><b>DIR направления:</b> если контроллеру нужен логический вход 0/1, используйте любой <code>Relay 0..3</code>.</p>
      <p><b>Если нужна смена полярности:</b> используйте <code>Polarity 0..2</code>.</p>
      <p>Практически: PCA дает газ, Relay или Polarity выбирает направление.</p>
    </div>
    <div class="card">
      <h2>Relay</h2>
      <p><b>Relay 0:</b> MCP23017 A4, pin 4.</p>
      <p><b>Relay 1:</b> MCP23017 A3, pin 3.</p>
      <p><b>Relay 2:</b> MCP23017 A2, pin 2.</p>
      <p><b>Relay 3:</b> MCP23017 A1, pin 1.</p>
      <p>Это обычные дискретные выходы. Используйте их для разрешения, света, насоса, клапана, DIR-входа и других задач 0/1.</p>
    </div>
    <div class="card">
      <h2>Polarity</h2>
      <p><b>Polarity 0:</b> ON B2 pin 10, DIR B3 pin 11.</p>
      <p><b>Polarity 1:</b> ON B4 pin 12, DIR B5 pin 13.</p>
      <p><b>Polarity 2:</b> ON B6 pin 14, DIR B7 pin 15.</p>
      <p>Здесь два сигнала на каждый канал: один включает выход, второй задает направление или полярность.</p>
    </div>
    <div class="card">
      <h2>PCA9685</h2>
      <p>Каналы <code>PCA 0..15</code> находятся на модуле PCA9685 по I2C адресу <code>0x40</code>.</p>
      <p>Используйте их как плавный ШИМ, газ, сервосигнал или другой управляемый выход.</p>
    </div>
  </div>
</body>
</html>
)HTML";

static void handleIndex() {
  server.send(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleHelp() {
  server.send(200, "text/html; charset=utf-8", HELP_HTML);
}

static void handleWiring() {
  server.send(200, "text/html; charset=utf-8", WIRING_HTML);
}

static void handleStatus() {
  if (!gGetStatus) {
    server.send(500, "text/plain; charset=utf-8", "status callback is null");
    return;
  }
  server.send(200, "application/json; charset=utf-8", gGetStatus());
}

static void handleGetRules() {
  if (!gGetRules) {
    server.send(500, "text/plain; charset=utf-8", "rules callback is null");
    return;
  }
  server.send(200, "application/json; charset=utf-8", gGetRules());
}

static void handleSetRules() {
  if (!gSetRules) {
    server.send(500, "text/plain; charset=utf-8", "set callback is null");
    return;
  }
  String body = server.arg("plain");
  bool ok = gSetRules(body);
  server.send(ok ? 200 : 400, "text/plain; charset=utf-8", ok ? "OK" : "Invalid JSON");
}

static void handleSave() {
  if (gSaveRules) gSaveRules();
  server.send(200, "text/plain; charset=utf-8", "Saved");
}

static void handleLoad() {
  if (gLoadRules) gLoadRules();
  server.send(200, "text/plain; charset=utf-8", "Loaded");
}

static void webTask(void*) {
  for (;;) {
    if (gWebEnabled && gServerStarted) {
      server.handleClient();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void webStartInternal() {
  if (!gApSsid || !gApPass || gWebEnabled) return;
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(gApSsid, gApPass);
  gWebEnabled = true;
  Serial.printf("[WEB] AP SSID: %s\n", gApSsid);
  Serial.printf("[WEB] AP IP: %s\n", WiFi.softAPIP().toString().c_str());
}

static void webStopInternal() {
  if (!gWebEnabled) return;
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  gWebEnabled = false;
  Serial.println("[WEB] AP disabled");
}

void webUiBegin(
  const char* apSsid,
  const char* apPass,
  bool enabled,
  WebUiGetJsonFn getStatusJson,
  WebUiGetJsonFn getRulesJson,
  WebUiSetJsonFn setRulesJson,
  WebUiActionFn saveRulesFn,
  WebUiActionFn loadRulesFn
) {
  gApSsid = apSsid;
  gApPass = apPass;
  gGetStatus = getStatusJson;
  gGetRules = getRulesJson;
  gSetRules = setRulesJson;
  gSaveRules = saveRulesFn;
  gLoadRules = loadRulesFn;

  server.on("/", HTTP_GET, handleIndex);
  server.on("/help", HTTP_GET, handleHelp);
  server.on("/wiring", HTTP_GET, handleWiring);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/rules", HTTP_GET, handleGetRules);
  server.on("/api/rules", HTTP_POST, handleSetRules);
  server.on("/api/save", HTTP_POST, handleSave);
  server.on("/api/load", HTTP_POST, handleLoad);
  server.begin();
  gServerStarted = true;
  if (enabled) {
    webStartInternal();
  } else {
    Serial.println("[WEB] AP disabled by preference");
  }

  xTaskCreatePinnedToCore(webTask, "WEB", 4096, nullptr, 1, nullptr, 1);
}

void webUiSetEnabled(bool enabled) {
  if (enabled) webStartInternal();
  else webStopInternal();
}

