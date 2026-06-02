#include "web_ui.h"

#include <WiFi.h>
#include <WebServer.h>

static WebServer server(80);
static WebUiGetJsonFn gGetStatus = nullptr;
static WebUiGetJsonFn gGetRules = nullptr;
static WebUiSetJsonFn gSetRules = nullptr;
static WebUiActionFn gSaveRules = nullptr;
static WebUiActionFn gLoadRules = nullptr;

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
    .wrap{max-width:1100px;margin:20px auto;padding:0 12px}
    .card{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px;margin-bottom:12px}
    h1{margin:0 0 8px}
    .muted{color:var(--muted);font-size:14px}
    .row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
    button{padding:8px 12px;border:0;border-radius:10px;background:var(--pri);color:#fff;cursor:pointer}
    button.gray{background:#64748b}
    button.green{background:var(--ok)}
    button.red{background:var(--warn)}
    select,input{padding:7px 8px;border:1px solid var(--line);border-radius:8px}
    pre{background:#0b1220;color:#dbeafe;padding:10px;border-radius:8px;overflow:auto}
    table{width:100%;border-collapse:collapse;font-size:14px}
    th,td{border-bottom:1px solid var(--line);padding:6px;text-align:left}
    .chip{padding:4px 8px;border-radius:999px;background:#e2e8f0;font-size:12px}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Настройка правил ESP32</h1>
      <div class="muted">Редактор правил без ручного JSON. Выберите канал, добавьте правила, примените, затем сохраните в память.</div>
      <div class="row" style="margin-top:10px">
        <button onclick="refreshStatus()">Обновить статус</button>
        <button onclick="fetchRules()">Загрузить правила</button>
        <button class="green" onclick="applyRules()">Применить на устройство</button>
        <button class="gray" onclick="saveRules()">Сохранить в NVS</button>
        <button class="gray" onclick="loadRulesFromNvs()">Загрузить из NVS</button>
        <button class="gray" onclick="location.href='/help'">Help / Инструкция</button>
        <span id="msg" class="chip">Готово</span>
      </div>
    </div>

    <div class="card">
      <div class="row">
        <label>Канал:</label>
        <select id="channelSel" onchange="renderRules()"></select>
        <button onclick="addRule()">Добавить правило</button>
      </div>
      <table style="margin-top:10px">
        <thead>
          <tr><th>#</th><th>Режим</th><th>Условие</th><th>A</th><th>B</th><th>OutA</th><th>OutB</th><th>Цель</th><th></th></tr>
        </thead>
        <tbody id="rulesBody"></tbody>
      </table>
    </div>

    <div class="card">
      <div class="row" style="justify-content:space-between">
        <h3 style="margin:0">Статус (онлайн)</h3>
        <label class="row" style="gap:6px">
          <input id="autoRefresh" type="checkbox" checked onchange="toggleAuto()">
          Автообновление 300мс
        </label>
      </div>
      <div class="row" style="margin-top:10px">
        <div style="flex:1;min-width:320px">
          <div class="muted" style="margin-bottom:6px">Входы (каналы)</div>
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
  if (t <= 15) return 2;   // Engine
  if (t <= 19) return 3;   // BTS
  if (t <= 23) return 0;   // Relay
  return 1;                // Polarity
}
function isPcaTarget(t){
  return Number(t) <= 15;
}
function mapRangeClamped(x, inA, inB, outA, outB){
  if (Math.abs(inB - inA) < 0.001) return outA;
  let t = (x - inA) / (inB - inA);
  t = Math.max(0, Math.min(1, t));
  return outA + (outB - outA) * t;
}
function currentInputUs(ch){
  if (!lastStatus || !lastStatus.inputs || !lastStatus.inputs[ch]) return null;
  return Number(lastStatus.inputs[ch].us);
}
function updatePcaHints(){
  const ch = Number(document.getElementById('channelSel').value||0);
  const arr = cfg.rules[ch] || [];
  arr.forEach((r,idx)=>{
    if (!isPcaTarget(r.tgt)) return;
    const el = document.getElementById(`hint-${ch}-${idx}`);
    if (!el) return;
    const curUs = currentInputUs(ch);
    let hintText = `A..B ${r.a}..${r.b} -> OutA..OutB ${r.outA}..${r.outB}`;
    if (curUs !== null && curUs >= 900 && curUs <= 2100) {
      const mapped = Math.round(mapRangeClamped(curUs, Number(r.a), Number(r.b), Number(r.outA), Number(r.outB)));
      hintText += ` | ${curUs} -> ${mapped}`;
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

function sanitizeRule(r){
  const tgt = Math.min(26, Math.max(0, Number(r.tgt||0)));
  const safeMode = modeForTarget(tgt);
  return {
    type: Math.min(2, Math.max(0, Number(r.type||0))),
    cond: Math.min(3, Math.max(0, Number(r.cond||0))),
    a: Math.min(2100, Math.max(900, Number(r.a||1500))),
    b: Math.min(2500, Math.max(900, Number(r.b||2000))),
    outA: Math.min(4095, Math.max(0, Number(r.outA||1000))),
    outB: Math.min(4095, Math.max(0, Number(r.outB||2000))),
    tgt: tgt,
    mode: safeMode
  };
}

function renderRules(){
  const ch = Number(document.getElementById('channelSel').value||0);
  const body = document.getElementById('rulesBody');
  body.innerHTML = "";
  const arr = cfg.rules[ch] || [];
  arr.forEach((r,idx)=>{
    const pca = isPcaTarget(r.tgt);
    const tr = document.createElement('tr');
    tr.innerHTML = `
      <td>${idx+1}</td>
      <td><select onchange="upd(${ch},${idx},'mode',this.value)" disabled>${opt(MODE, Number(r.mode))}</select></td>
      <td><select onchange="upd(${ch},${idx},'cond',this.value)">${opt(COND, Number(r.cond))}</select></td>
      <td><input type="number" min="900" max="2100" value="${r.a}" onchange="upd(${ch},${idx},'a',this.value)"></td>
      <td><input type="number" min="900" max="2500" value="${r.b}" onchange="upd(${ch},${idx},'b',this.value)"></td>
      <td><input type="number" min="0" max="4095" value="${r.outA}" ${pca ? "" : "disabled"} onchange="upd(${ch},${idx},'outA',this.value)"></td>
      <td><input type="number" min="0" max="4095" value="${r.outB}" ${pca ? "" : "disabled"} onchange="upd(${ch},${idx},'outB',this.value)"></td>
      <td><select onchange="upd(${ch},${idx},'tgt',this.value)">${targetOptions(r.tgt)}</select></td>
      <td>
        <button class="gray" onclick="moveRule(${ch},${idx},-1)">↑</button>
        <button class="gray" onclick="moveRule(${ch},${idx},1)">↓</button>
        <button class="red" onclick="delRule(${ch},${idx})">Удалить</button>
      </td>`;
    body.appendChild(tr);
    if (pca) {
      const hint = document.createElement('tr');
      const curUs = currentInputUs(ch);
      let hintText = `A..B ${r.a}..${r.b} -> OutA..OutB ${r.outA}..${r.outB}`;
      if (curUs !== null && curUs >= 900 && curUs <= 2100) {
        const mapped = Math.round(mapRangeClamped(curUs, Number(r.a), Number(r.b), Number(r.outA), Number(r.outB)));
        hintText += ` | ${curUs} -> ${mapped}`;
      }
      hint.innerHTML = `<td id="hint-${ch}-${idx}" colspan="9" style="color:#475569;font-size:12px;padding-top:0">${hintText}</td>`;
      body.appendChild(hint);
    }
  });
  document.getElementById('jsonView').textContent = JSON.stringify(cfg, null, 2);
}

function upd(ch,idx,key,val){
  cfg.rules[ch][idx][key] = Number(val);
  if (key === 'tgt') {
    cfg.rules[ch][idx].mode = modeForTarget(Number(val));
  }
  cfg.rules[ch][idx] = sanitizeRule(cfg.rules[ch][idx]);
  renderRules();
}

function addRule(){
  const ch = Number(document.getElementById('channelSel').value||0);
  cfg.rules[ch].push({cond:0, a:1500, b:2000, outA:1000, outB:2000, tgt:0, mode:2});
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
    updatePcaHints();
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
    if (Number(o.idx) <= 15 && o.pcaUs !== undefined) {
      eff = `${o.pcaUs} us / ${o.pca12}`;
    }
    tr.innerHTML = `<td>${targetLabel(o.idx)}</td><td>${o.active ? "1":"0"}</td><td>${Number(o.v01).toFixed(2)}</td><td>${Number(o.vs).toFixed(2)}</td><td>${eff}</td>`;
    outBody.appendChild(tr);
  });
}

function toggleAuto(){
  const on = document.getElementById('autoRefresh').checked;
  if (autoTimer) {
    clearInterval(autoTimer);
    autoTimer = null;
  }
  if (on) {
    autoTimer = setInterval(refreshStatus, 1000);
  }
}

async function fetchRules(){
  const r = await fetch('/api/rules');
  const t = await r.text();
  cfg = JSON.parse(t);
  for(let ch=0; ch<8; ch++){
    cfg.rules[ch] = (cfg.rules[ch]||[]).map(sanitizeRule);
  }
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
    .wrap{max-width:900px;margin:20px auto;padding:0 12px}
    .card{background:#fff;border:1px solid #dbe3ee;border-radius:12px;padding:14px;margin-bottom:12px}
    h1,h2{margin:0 0 10px}
    p{margin:8px 0}
    code{background:#e2e8f0;padding:2px 6px;border-radius:6px}
    .btn{display:inline-block;padding:8px 12px;background:#0ea5e9;color:#fff;text-decoration:none;border-radius:10px}
  </style>
</head>
<body>
  <div class="wrap">
    <div class="card">
      <h1>Инструкция по настройке правил</h1>
      <a class="btn" href="/">Назад в настройки</a>
    </div>
    <div class="card">
      <h2>Поля правила</h2>
      <p><b>Режим (mode):</b> ставится автоматически по цели и вручную не выбирается: Relay / Polarity / Engine / BTS.</p>
      <p><b>Условие (cond):</b> <code>0 ANY</code>, <code>1 Меньше A</code>, <code>2 Больше A</code>, <code>3 Между A и B</code>.</p>
      <p><b>A/B:</b> входной диапазон PWM в микросекундах. Для PCA именно диапазон <code>A..B</code> будет растягиваться или сжиматься.</p>
      <p><b>OutA/OutB:</b> выходной диапазон для PCA. Пример: вход <code>1200..1600</code> можно растянуть в выход <code>1000..2000</code>.</p>
      <p><b>Цель (tgt):</b> <code>0..15 PCA</code>, <code>16..19 BTS</code>, <code>20..23 Relay</code>, <code>24..26 Polarity</code>.</p>
    </div>
    <div class="card">
      <h2>Как настраивать</h2>
      <p>1. Выберите канал.</p>
      <p>2. Нажмите «Добавить правило».</p>
      <p>3. Выберите условие и диапазон A/B.</p>
      <p>4. Если цель PCA, задайте OutA/OutB.</p>
      <p>5. Выберите цель (выход).</p>
      <p>6. Нажмите «Применить на устройство».</p>
      <p>7. Если результат подходит, нажмите «Сохранить в NVS».</p>
    </div>
    <div class="card">
      <h2>Приоритет</h2>
      <p>Если несколько правил пишут в один выход, используется последнее сработавшее правило в списке.</p>
      <p>Меняйте порядок кнопками <b>↑</b> и <b>↓</b>.</p>
    </div>
    <div class="card">
      <h2>Примеры</h2>
      <p><b>Реле по верхнему положению стика:</b> cond=Больше, A=1700, tgt=Relay 0..3.</p>
      <p><b>BTS мотор от стика:</b> cond=ANY, tgt=BTS 0..3.</p>
      <p><b>PCA ШИМ от стика:</b> cond=ANY, A=1000, B=2000, OutA=1000, OutB=2000, tgt=PCA 0..15.</p>
      <p><b>Реле полярности:</b> cond=ANY, tgt=Polarity 0..2.</p>
    </div>
    <div class="card">
      <h2>Большой набор готовых настроек</h2>
      <p><b>1. Relay ON только вверху:</b> cond=Больше, A=1700, tgt=Relay 0 (20).</p>
      <p><b>2. Relay ON только внизу:</b> cond=Меньше, A=1300, tgt=Relay 1 (21).</p>
      <p><b>3. Relay ON только в центре:</b> cond=Между, A=1450, B=1550, tgt=Relay 2 (22).</p>
      <p><b>4. Relay как кнопка с широким порогом:</b> cond=Больше, A=1600, tgt=Relay 3 (23).</p>

      <p><b>5. BTS прямой привод стиком:</b> cond=ANY, tgt=BTS 0 (16).</p>
      <p><b>6. BTS только в рабочем окне:</b> cond=Между, A=1200, B=1800, tgt=BTS 1 (17).</p>
      <p><b>7. BTS мягкая зона старта:</b> cond=Между, A=1400, B=2000, tgt=BTS 2 (18).</p>
      <p><b>8. BTS реверс только внизу:</b> cond=Меньше, A=1480, tgt=BTS 3 (19).</p>

      <p><b>9. PCA полный диапазон 1:1:</b> cond=ANY, A=1000, B=2000, OutA=1000, OutB=2000, tgt=PCA 0.</p>
      <p><b>10. PCA растянуть узкий вход:</b> cond=ANY, A=1200, B=1600, OutA=1000, OutB=2000, tgt=PCA 1.</p>
      <p><b>11. PCA сжать широкий вход:</b> cond=ANY, A=1000, B=2000, OutA=1300, OutB=1700, tgt=PCA 2.</p>
      <p><b>12. PCA инверсия:</b> cond=ANY, A=1000, B=2000, OutA=2000, OutB=1000, tgt=PCA 3.</p>
      <p><b>13. PCA только верхняя зона:</b> cond=Больше, A=1500, B=2000, OutA=1000, OutB=2000, tgt=PCA 4.</p>
      <p><b>14. PCA только центр:</b> cond=Между, A=1400, B=1600, OutA=1000, OutB=2000, tgt=PCA 5.</p>
      <p><b>15. PCA очень узкая зона:</b> cond=Между, A=1700, B=1800, OutA=1000, OutB=2000, tgt=PCA 6.</p>
      <p><b>16. PCA мягкий диапазон:</b> cond=ANY, A=1100, B=1900, OutA=1200, OutB=1800, tgt=PCA 7.</p>

      <p><b>17. Polarity по стику:</b> cond=ANY, tgt=Polarity 0 (24).</p>
      <p><b>18. Polarity только при верхнем уровне:</b> cond=Больше, A=1600, tgt=Polarity 1 (25).</p>
      <p><b>19. Polarity только при нижнем уровне:</b> cond=Меньше, A=1400, tgt=Polarity 2 (26).</p>

      <p><b>18. Два правила на один Relay (логика окна):</b></p>
      <p>Правило A: cond=Больше, A=1600, tgt=Relay 0 (20).</p>
      <p>Правило B: cond=Меньше, A=1400, tgt=Relay 0 (20).</p>
      <p>Итог: в центре OFF, по краям ON (зависит от порядка).</p>

      <p><b>19. Два правила на один BTS (ограничение):</b></p>
      <p>Основное: cond=ANY, tgt=BTS 0 (16).</p>
      <p>Ограничение ниже списком: cond=Между, A=1300, B=1700, tgt=BTS 0 (16).</p>
      <p>Итог: за пределами окна BTS отключается/неактивен.</p>

      <p><b>20. Аварийный стоп для BTS по отдельному каналу:</b></p>
      <p>Канал газа: cond=ANY, tgt=BTS 1 (17).</p>
      <p>Канал стопа (правило ниже приоритетом): cond=Больше, A=1700, tgt=BTS 1 (17).</p>
      <p>Итог: при активации стоп-канала последнее правило перехватывает выход.</p>

      <p><b>21. Реле включается только при “валидном” сигнале:</b> cond=Между, A=1000, B=2000, tgt=Relay 2 (22).</p>
      <p><b>22. Реле только на максимуме:</b> cond=Больше, A=1900, tgt=Relay 3 (23).</p>
      <p><b>23. PCA только в минимумах:</b> cond=Меньше, A=1100, tgt=PCA 6.</p>
      <p><b>24. PCA только в максимумах:</b> cond=Больше, A=1900, tgt=PCA 7.</p>
      <p><b>25. BTS “мертвая зона” вокруг центра:</b> cond=Меньше, A=1450, tgt=BTS 2 (18) и отдельное правило cond=Больше, A=1550, tgt=BTS 2 (18).</p>

      <p><b>26. Несколько PCA от одного канала:</b> cond=ANY, tgt=PCA 8, 9, 10 (три отдельных правила).</p>
      <p><b>27. Один канал включает сразу 2 реле:</b> cond=Больше, A=1700, tgt=Relay 0 и Relay 1.</p>
      <p><b>28. Один канал управляет Relay + PCA:</b> cond=ANY, tgt=Relay 0 и отдельное cond=ANY, tgt=PCA 11.</p>
      <p><b>29. Один канал управляет Polarity + Relay:</b> cond=ANY, tgt=Polarity 0 и cond=Больше, A=1650, tgt=Relay 2.</p>
      <p><b>30. Служебный тест выхода:</b> cond=ANY, tgt=любой выход, подвигайте канал и смотрите таблицу “Статус”.</p>
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
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void webUiBegin(
  const char* apSsid,
  const char* apPass,
  WebUiGetJsonFn getStatusJson,
  WebUiGetJsonFn getRulesJson,
  WebUiSetJsonFn setRulesJson,
  WebUiActionFn saveRulesFn,
  WebUiActionFn loadRulesFn
) {
  gGetStatus = getStatusJson;
  gGetRules = getRulesJson;
  gSetRules = setRulesJson;
  gSaveRules = saveRulesFn;
  gLoadRules = loadRulesFn;

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(apSsid, apPass);

  server.on("/", HTTP_GET, handleIndex);
  server.on("/help", HTTP_GET, handleHelp);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/rules", HTTP_GET, handleGetRules);
  server.on("/api/rules", HTTP_POST, handleSetRules);
  server.on("/api/save", HTTP_POST, handleSave);
  server.on("/api/load", HTTP_POST, handleLoad);
  server.begin();

  Serial.printf("[WEB] AP SSID: %s\n", apSsid);
  Serial.printf("[WEB] AP IP: %s\n", WiFi.softAPIP().toString().c_str());

  xTaskCreatePinnedToCore(webTask, "WEB", 4096, nullptr, 1, nullptr, 1);
}
