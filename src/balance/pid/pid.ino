#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <DShotRMT.h>
#include <Preferences.h>

static const int MAX_SAMPLES = 300; // niet te hoog om geheugen te besparen (vermenigvuldigen met de log_interval om te komen aan de maximale totale meet periode)

struct LogSample {
  float angle;
  float p;
  float i;
  float d;
  float pidOut;
  float err;
  uint16_t m1;
  uint16_t m2;
};

static LogSample logBuf[MAX_SAMPLES];
static int       logCount = 0;   // huidig aantal opgeslagen samples
static bool      testRunning = false;  // true tussen Start en Stop
static bool      testDataReady = false; // true nadat Stop gedrukt is

// ────────────────────────────────────────────────────────────
//  IMU
// ────────────────────────────────────────────────────────────
const int MPU1 = 0x68;
#define I2C_SDA 21
#define I2C_SCL 22

float pitch = 0.0f;
float roll  = 0.0f;

float absolutePitchRaw = 0.0f;
float absoluteRollRaw  = 0.0f;
bool  firstLoop        = true;

float balance_offset = 0.0f;
float roll_offset    = 0.0f;

unsigned long lastTime = 0;
const unsigned long interval = 1;   // ms tussen IMU-reads
const unsigned long LOG_INTERVAL = 50; //ms logging (niet op 10ms zetten maar minsten 50ms anders te veel lag)

Preferences preferences;

// ────────────────────────────────────────────────────────────
//  WiFi
// ────────────────────────────────────────────────────────────

//ADD CREDENTIALS
//static const char* WIFI_SSID = "";
//static const char* WIFI_PASS = "";


WebServer server(80);
String ipAddress = "niet verbonden";

// ────────────────────────────────────────────────────────────
//  PID
// ────────────────────────────────────────────────────────────
float Kp = 10.0f;
float Ki = 0.1f;
float Kd = 1.0f;

float targetAngle = 0.0f;
float error_val   = 0.0f;
float lastError   = 0.0f;
float integral    = 0.0f;
float derivative  = 0.0f;
float pTerm       = 0.0f;
float iTerm       = 0.0f;
float dTerm       = 0.0f;
float pidOutput   = 0.0f;
unsigned long lastPIDTime = 0;

bool pidEnabled    = true;
bool motorsEnabled = false;

float motorBase = 100.0f;
float motorTrim = 0.0f;

// ────────────────────────────────────────────────────────────
//  DSHOT
// ────────────────────────────────────────────────────────────
static constexpr gpio_num_t MOTOR1_PIN = GPIO_NUM_18;
static constexpr gpio_num_t MOTOR2_PIN = GPIO_NUM_19;

DShotRMT motor1(MOTOR1_PIN, DSHOT600, false);
DShotRMT motor2(MOTOR2_PIN, DSHOT600, false);

uint16_t motor1Raw = 0;
uint16_t motor2Raw = 0;

// ────────────────────────────────────────────────────────────
//  IMU hulpfuncties  
// ────────────────────────────────────────────────────────────
void setupMPU(int address) {
  Wire.beginTransmission(address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
}

void readRawData(int address,
    int16_t &acX, int16_t &acY, int16_t &acZ,
    int16_t &gyX, int16_t &gyY, int16_t &gyZ) {
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(address, 14, true);
  acX = Wire.read() << 8 | Wire.read();
  acY = Wire.read() << 8 | Wire.read();
  acZ = Wire.read() << 8 | Wire.read();
  int16_t tmp = Wire.read() << 8 | Wire.read();
  gyX = Wire.read() << 8 | Wire.read();
  gyY = Wire.read() << 8 | Wire.read();
  gyZ = Wire.read() << 8 | Wire.read();
}

// ────────────────────────────────────────────────────────────
//  Kalibratie 
// ────────────────────────────────────────────────────────────
void voerKalibratieUit() {
  float somPitchRaw = 0, somRollRaw = 0;
  for (int i = 0; i < 100; i++) {
    int16_t acX1, acY1, acZ1, gyX1, gyY1, gyZ1;
    readRawData(MPU1, acX1, acY1, acZ1, gyX1, gyY1, gyZ1);
    somPitchRaw += (atan2(-(float)acX1,
        sqrt(pow((float)acY1, 2) + pow((float)acZ1, 2))) * 180.0) / PI;
    somRollRaw  += (atan2((float)acY1, (float)acZ1) * 180.0) / PI;
    delay(10);
  }
  balance_offset = somPitchRaw / 100;
  roll_offset    = somRollRaw  / 100;
  preferences.putFloat("balOff",  balance_offset);
  preferences.putFloat("rollOff", roll_offset);
  firstLoop = true;
}

void wisKalibratie() {
  balance_offset = 0.0f;
  roll_offset    = 0.0f;
  preferences.putFloat("balOff",  0.0f);
  preferences.putFloat("rollOff", 0.0f);
}

// ────────────────────────────────────────────────────────────
//  Motor helper 
// ────────────────────────────────────────────────────────────
uint16_t throttleFromValue(float value, float minVal) {
  if (value < minVal)             value = minVal;
  if (value > DSHOT_THROTTLE_MAX) value = DSHOT_THROTTLE_MAX;
  return (uint16_t)value;
}

// ────────────────────────────────────────────────────────────
//  PID 
// ────────────────────────────────────────────────────────────
void computePID(float currentAngle, float dt) {
  if (dt <= 0.0f || dt > 2.0f) return;  // sla slechte dt over
  error_val  = targetAngle - currentAngle;
  pTerm      = Kp * error_val;

  integral  += error_val * dt;
  // anti-windup: begrens integraal
  integral   = constrain(integral, -500.0f/max(Ki,0.001f), 500.0f/max(Ki,0.001f));
  iTerm      = Ki * integral;

  derivative = (error_val - lastError) / dt;
  dTerm      = Kd * derivative;

  pidOutput  = pTerm + iTerm + dTerm;
  lastError  = error_val;
}

// ────────────────────────────────────────────────────────────
//  HTML-pagina
// ────────────────────────────────────────────────────────────
String htmlPage() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Balans Controller</title>
<style>
* { box-sizing: border-box; margin: 0; padding: 0; }
body { font-family: Arial, sans-serif; background: #111; color: #eee; padding: 12px; }
h1  { font-size: 18px; margin-bottom: 10px; color: #fff; }
h2  { font-size: 14px; color: #aaa; margin: 14px 0 6px; text-transform: uppercase; letter-spacing: 1px; }
.card { background: #1c1c1c; border-radius: 10px; padding: 12px; margin-bottom: 10px; }
.row  { display: flex; gap: 8px; margin-bottom: 10px; }
.row .card { flex: 1; margin-bottom: 0; text-align: center; }
.label  { color: #aaa; font-size: 12px; margin-bottom: 3px; }
.angle  { font-size: 42px; font-weight: bold; text-align: center; padding: 8px 0; }
.angle.ok     { color: #4caf50; }
.angle.warn   { color: #ff9800; }
.angle.danger { color: #ff5252; }
.status { font-size: 20px; font-weight: bold; }
.on  { color: #4caf50; }
.off { color: #ff5252; }
.btn-row { display: flex; gap: 6px; flex-wrap: wrap; margin-top: 8px; }
button { padding: 9px 14px; border: none; border-radius: 8px; font-size: 14px; cursor: pointer; }
.onBtn    { background: #4caf50; color: #fff; }
.offBtn   { background: #ff5252; color: #fff; }
.calBtn   { background: #9c27b0; color: #fff; }
.resetBtn { background: #555; color: #fff; }
.startBtn { background: #2196f3; color: #fff; flex: 1; font-size: 16px; font-weight: bold; padding: 12px; }
.stopBtn  { background: #ff5252; color: #fff; flex: 1; font-size: 16px; font-weight: bold; padding: 12px; }
label  { font-size: 12px; color: #aaa; display: block; margin-top: 6px; margin-bottom: 2px; }
input[type=number] { width: 100%; padding: 7px 9px; border-radius: 8px; border: 1px solid #444; background: #222; color: #eee; font-size: 14px; }
.save-btn { background: #2196f3; color: #fff; width: 100%; margin-top: 10px; padding: 10px; font-size: 15px; border-radius: 8px; border: none; cursor: pointer; }
.trim-bar-track  { position: relative; height: 7px; background: #333; border-radius: 4px; margin: 6px 0; }
.trim-bar-fill   { position: absolute; top: 0; height: 7px; border-radius: 4px; transition: all 0.15s; }
.trim-center     { position: absolute; left: 50%; top: -2px; width: 2px; height: 11px; background: #555; }
.trim-labels     { display: flex; justify-content: space-between; font-size: 10px; color: #666; }

/* ── POST-TEST sectie ── */
#postSection { display: none; }
.graph-toggles { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 8px; }
.toggle-btn { padding: 5px 11px; border-radius: 6px; font-size: 12px; cursor: pointer; border: 1.5px solid transparent; opacity: 0.4; transition: opacity 0.2s; }
.toggle-btn.active { opacity: 1; border-color: currentColor; }

/* Grafiek canvas */
.chart-wrap  { position: relative; margin-bottom: 6px; }
canvas.chart { display: block; width: 100%; border-radius: 6px; background: #0d0d14; }

/* Slider */
.slider-wrap { margin: 4px 0 10px; }
#timeSlider  { width: 100%; accent-color: #2196f3; }
.slider-readout { background: #1c1c1c; border-radius: 8px; padding: 8px 10px; font-size: 12px; margin-top: 4px; }
.readout-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(140px, 1fr)); gap: 4px 12px; }
.readout-row  { display: flex; justify-content: space-between; }
.readout-key  { color: #888; }
.readout-val  { font-weight: bold; }

/* Fiets animatie */
#bikeCanvas { display: block; width: 100%; border-radius: 8px; background: #0d0d14; }
</style>
</head>
<body>
<h1>Balans Controller</h1>

<!-- Live hoek -->
<div class="card">
  <div class="label">Hoek (Pitch)</div>
  <div class="angle" id="pitch">-</div>
</div>

<!-- Kalibratie offset tonen -->
<div class="card" id="calCard" style="display:none;">
  <div class="label">Gekalibreerde hoek (offset)</div>
  <div class="status" id="calOffset" style="font-size:20px;">-</div>
  <div style="font-size:11px; color:#666; margin-top:4px;">Target hoek = 0° t.o.v. deze positie</div>
</div>

<!-- Motor + PID status -->
<div class="row">
  <div class="card">
    <div class="label">Motoren</div>
    <div class="status" id="motorsStatus">-</div>
  </div>
  <div class="card">
    <div class="label">PID</div>
    <div class="status" id="pidStatus">-</div>
    <div class="btn-row" style="justify-content:center; margin-top:8px;">
      <button class="onBtn"  onclick="setPid(1)">AAN</button>
      <button class="offBtn" onclick="setPid(0)">UIT</button>
    </div>
  </div>
</div>


<!-- START / STOP -->
<div class="card">
  <div class="label" id="testStatusLabel">Test niet actief</div>
  <div class="btn-row" style="margin-top:6px;">
    <button class="startBtn" onclick="startTest()">MOTOR AAN</button>
    <button class="stopBtn"  onclick="stopTest()">MOTOR UIT</button>
  </div>
</div>

<!-- PID instellingen -->
<div class="card">
  <h2>PID Instellingen</h2>
  <label>Target hoek (°)</label>
  <input type="number" id="target" step="0.1" oninput="queueLive()">
  <label>Kp</label>
  <input type="number" id="kp" step="0.1" oninput="queueLive()">
  <label>Ki</label>
  <input type="number" id="ki" step="0.01" oninput="queueLive()">
  <label>Kd</label>
  <input type="number" id="kd" step="0.1" oninput="queueLive()">
  <label>Basis toerental (idle, 48-500)</label>
  <input type="number" id="motorBase" step="1" oninput="queueLive()">
  <label>Motor Trim (-200 tot +200)</label>
  <input type="number" id="motorTrim" step="1" min="-200" max="200" oninput="queueLive(); updateTrimBar()">
  <div style="margin-top:6px;">
    <div class="trim-bar-track">
      <div class="trim-center"></div>
      <div class="trim-bar-fill" id="trimBarFill" style="left:50%;width:0%"></div>
    </div>
    <div class="trim-labels"><span>Motor 2</span><span>Gelijk</span><span>Motor 1</span></div>
  </div>
  <button class="save-btn" onclick="savePid()">Opslaan</button>
</div>

<!-- Kalibratie -->
<div class="card">
  <h2>Kalibratie</h2>
  <div class="btn-row" style="margin-top:4px;">
    <button class="calBtn"   onclick="calibrateImu()">Kalibreer (houd rechtop)</button>
    <button class="resetBtn" onclick="resetCal()">Wis kalibratie</button>
  </div>
</div>

<!-- ══════════════════════════════════════════
     POST-TEST SECTIE
     Verborgen totdat Stop gedrukt is.
     Toont: grafieken, slider + readout, animatie
══════════════════════════════════════════ -->
<div id="postSection">
  <div class="card">
    <h2>Testresultaten</h2>
    <div id="testMeta" style="font-size:12px; color:#888; margin-bottom:8px;"></div>

    <!-- Toggle knoppen -->
    <div class="graph-toggles" id="graphToggles"></div>

    <!-- Tijdslider -->
    <div class="slider-wrap">
      <input type="range" id="timeSlider" min="0" max="100" value="0" oninput="onSlider()">
    </div>
    <!-- Readout voor slider positie -->
    <div class="slider-readout">
      <div style="font-size:11px; color:#666; margin-bottom:4px;">Waarden op geselecteerd moment:</div>
      <div class="readout-grid" id="sliderReadout"></div>
    </div>

    <!-- Grafiek canvassen worden hier ingevuld door JS -->
    <div id="chartsContainer" style="margin-top:10px;"></div>

    <!-- Fiets animatie -->
    <h2 style="margin-top:14px;">Fiets animatie (playback)</h2>
    <div style="margin-bottom:6px;">
      <button onclick="playAnim()" style="background:#2196f3; color:#fff; border-radius:6px; padding:6px 14px; border:none; cursor:pointer; font-size:13px;">▶ Afspelen</button>
      <button onclick="pauseAnim()" style="background:#555; color:#fff; border-radius:6px; padding:6px 14px; border:none; cursor:pointer; font-size:13px; margin-left:6px;">⏸ Pauze</button>
      <button onclick="resetAnim()" style="background:#333; color:#eee; border-radius:6px; padding:6px 12px; border:none; cursor:pointer; font-size:13px; margin-left:6px;">⏮ Reset</button>
      <span id="animTime" style="font-size:12px; color:#888; margin-left:10px;"></span>
    </div>
    <canvas id="bikeCanvas" height="180"></canvas>
  </div>
</div>

<script>
// ════════════════════════════════════════════
//  Globals
// ════════════════════════════════════════════
let testData   = null;   // opgeslagen na stop: {dt, samples:[{angle,p,i,d,pidOut,err,m1,m2}]}
let liveTimer  = null;
let animFrame  = null;
let animIdx    = 0;
let animPlaying = false;

// Grafiek configuratie
const CHANNELS = [
  { key: 'angle', label: 'Hoek (°)',        color: '#ef5350', dynamic: true  },
  { key: 'm1',    label: 'Motor 1 DSHOT',   color: '#42a5f5', dynamic: false },
  { key: 'm2',    label: 'Motor 2 DSHOT',   color: '#ef9a9a', dynamic: false },
  { key: 'p',     label: 'P-term',          color: '#66bb6a', dynamic: true  },
  { key: 'i',     label: 'I-term',          color: '#ffa726', dynamic: true  },
  { key: 'd',     label: 'D-term',          color: '#ab47bc', dynamic: true  },
];
const DSHOT_MAX = 2047;
let channelVisible = {};
CHANNELS.forEach(c => channelVisible[c.key] = true);

// ════════════════════════════════════════════
//  Live data pollen (250ms)
// ════════════════════════════════════════════
async function refreshData() {
  try {
    const data = await (await fetch('/data')).json();
    const p  = data.pitch.toFixed(1);
    const el = document.getElementById('pitch');
    el.textContent = p + '°';
    el.className = 'angle ' + (Math.abs(data.pitch) < 3 ? 'ok' : Math.abs(data.pitch) < 10 ? 'warn' : 'danger');

    const mEl = document.getElementById('motorsStatus');
    mEl.textContent  = data.motors_enabled ? 'AAN' : 'UIT';
    mEl.className    = 'status ' + (data.motors_enabled ? 'on' : 'off');

    const pEl = document.getElementById('pidStatus');
    pEl.textContent  = data.pid_enabled ? 'AAN' : 'UIT';
    pEl.className    = 'status ' + (data.pid_enabled ? 'on' : 'off');

    const tEl = document.getElementById('testStatusLabel');
    tEl.textContent  = data.test_running
      ? ('Test loopt… ' + data.log_count + ' samples opgeslagen')
      : (data.test_data_ready ? 'Test klaar — zie resultaten hieronder' : 'Test niet actief');

    updateInputIfNotFocused('target',    data.target_angle);
    updateInputIfNotFocused('kp',        data.kp);
    updateInputIfNotFocused('ki',        data.ki);
    updateInputIfNotFocused('kd',        data.kd);
    updateInputIfNotFocused('motorBase', data.motor_base);
    updateInputIfNotFocused('motorTrim', data.motor_trim);
    if (document.activeElement !== document.getElementById('motorTrim')) updateTrimBar();

    const offset = data.balance_offset;
    const calCard = document.getElementById('calCard');
    const calEl   = document.getElementById('calOffset');
    if (offset !== 0) {
        calCard.style.display = '';
        calEl.textContent = offset.toFixed(2) + '°';
    } else {
        calCard.style.display = 'none';
    }

  } catch(e) {}
}

function updateInputIfNotFocused(id, value) {
  const el = document.getElementById(id);
  if (document.activeElement !== el) el.value = value;
}

function updateTrimBar() {
  const val     = parseFloat(document.getElementById('motorTrim').value) || 0;
  const clamped = Math.max(-200, Math.min(200, val));
  const fill    = document.getElementById('trimBarFill');
  if (clamped >= 0) {
    fill.style.left  = '50%';
    fill.style.width = (clamped / 200 * 50) + '%';
    fill.style.background = '#4caf50';
  } else {
    const w = (-clamped / 200 * 50);
    fill.style.left  = (50 - w) + '%';
    fill.style.width = w + '%';
    fill.style.background = '#ff9800';
  }
}

// ════════════════════════════════════════════
//  Start / Stop test
// ════════════════════════════════════════════
async function startTest() {
  await fetch('/startTest');
}

async function stopTest() {
  await fetch('/stopTest');
  // Haal testdata op na korte vertraging
  setTimeout(fetchTestData, 400);
}

async function fetchTestData() {
  try {
    const resp = await fetch('/testdata');
    const json = await resp.json();
    if (!json || !json.samples || json.samples.length === 0) return;
    testData = json;
    buildPostSection();
    document.getElementById('postSection').style.display = 'block';
    document.getElementById('postSection').scrollIntoView({ behavior: 'smooth' });
  } catch(e) { console.error('testdata ophalen mislukt', e); }
}

// ════════════════════════════════════════════
//  Post-test UI opbouwen
// ════════════════════════════════════════════
function buildPostSection() {
  const n   = testData.samples.length;
  const dt  = testData.dt_ms / 1000.0;
  const dur = (n * dt).toFixed(1);

  document.getElementById('testMeta').textContent =
    n + ' samples  ·  ' + dur + ' s  ·  interval ' + testData.dt_ms + ' ms';

  // Slider bereik
  const slider = document.getElementById('timeSlider');
  slider.max   = n - 1;
  slider.value = 0;

  // Toggle knoppen
  const tDiv = document.getElementById('graphToggles');
  tDiv.innerHTML = '';
  CHANNELS.forEach(c => {
    const btn = document.createElement('button');
    btn.className   = 'toggle-btn active';
    btn.textContent = c.label;
    btn.style.color = c.color;
    btn.dataset.key = c.key;
    btn.onclick = () => {
      channelVisible[c.key] = !channelVisible[c.key];
      btn.classList.toggle('active', channelVisible[c.key]);
      renderAllCharts();
    };
    tDiv.appendChild(btn);
  });

  // Canvas per kanaal
  const cDiv = document.getElementById('chartsContainer');
  cDiv.innerHTML = '';
  CHANNELS.forEach(c => {
    const wrap = document.createElement('div');
    wrap.className   = 'chart-wrap';
    wrap.id          = 'wrap_' + c.key;
    const cv = document.createElement('canvas');
    cv.id     = 'chart_' + c.key;
    cv.height = 110;
    cv.className = 'chart';
    wrap.appendChild(cv);
    cDiv.appendChild(wrap);
  });

  renderAllCharts();
  updateSliderReadout(0);
  drawBikeFrame(0);
}

// ════════════════════════════════════════════
//  Grafieken tekenen
// ════════════════════════════════════════════
function getValues(key) {
  return testData.samples.map(s => s[key]);
}

function dynamicRange(vals, minHalf) {
  let peak = minHalf;
  vals.forEach(v => { if (Math.abs(v) > peak) peak = Math.abs(v); });
  const half = Math.max(minHalf, peak * 1.3);
  return { yMin: -half, yMax: half };
}

function renderChart(ch, sliderIdx) {
  const cv = document.getElementById('chart_' + ch.key);
  if (!cv) return;
  const wrap = document.getElementById('wrap_' + ch.key);
  wrap.style.display = channelVisible[ch.key] ? '' : 'none';
  if (!channelVisible[ch.key]) return;

  const ctx  = cv.getContext('2d');
  const W    = cv.clientWidth || 320;
  cv.width   = W;
  const H    = cv.height;

  const vals    = getValues(ch.key);
  const n       = vals.length;
  const ML = 38, MR = 6, MT = 14, MB = 18;
  const gW = W - ML - MR;
  const gH = H - MT - MB;

  // Y bereik
  let yMin, yMax;
  if (!ch.dynamic) {
    yMin = 0; yMax = DSHOT_MAX;
  } else {
    const r = dynamicRange(vals, 1);
    yMin = r.yMin; yMax = r.yMax;
  }
  const yRange = yMax - yMin || 1;

  const tx = i  => ML + (i / (n - 1)) * gW;
  const ty = v  => MT + gH - (v - yMin) / yRange * gH;

  // Achtergrond
  ctx.fillStyle = '#0d0d14';
  ctx.fillRect(0, 0, W, H);

  // Grid lijnen
  ctx.strokeStyle = '#1e1e2a';
  ctx.lineWidth   = 0.5;
  for (let t = 0; t <= 4; t++) {
    const v  = yMin + t * yRange / 4;
    const py = ty(v);
    ctx.beginPath(); ctx.moveTo(ML, py); ctx.lineTo(ML + gW, py); ctx.stroke();
    ctx.fillStyle = '#555'; ctx.font = '9px Arial';
    ctx.fillText(v.toFixed(ch.dynamic ? 1 : 0), 0, py + 3);
  }

  // Nullijn
  if (yMin < 0 && yMax > 0) {
    const py = ty(0);
    ctx.strokeStyle = '#333';
    ctx.lineWidth   = 1;
    ctx.beginPath(); ctx.moveTo(ML, py); ctx.lineTo(ML + gW, py); ctx.stroke();
  }

  // Datalijn
  if (n > 1) {
    ctx.strokeStyle = ch.color;
    ctx.lineWidth   = 1.5;
    ctx.beginPath();
    ctx.moveTo(tx(0), ty(vals[0]));
    for (let i = 1; i < n; i++) ctx.lineTo(tx(i), ty(vals[i]));
    ctx.stroke();
  }

  // Slider verticale lijn
  if (sliderIdx !== undefined && n > 1) {
    const sx = tx(sliderIdx);
    ctx.strokeStyle = 'rgba(255,255,255,0.5)';
    ctx.lineWidth   = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(sx, MT); ctx.lineTo(sx, MT + gH); ctx.stroke();
    ctx.setLineDash([]);

    // Punt op de lijn
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.arc(sx, ty(vals[sliderIdx]), 3.5, 0, Math.PI * 2);
    ctx.fill();
  }

  // Label
  ctx.fillStyle = '#ccc'; ctx.font = 'bold 10px Arial';
  ctx.fillText(ch.label, ML + 4, MT + 11);

  // Border
  ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
  ctx.strokeRect(ML, MT, gW, gH);
}

function renderAllCharts(sliderIdx) {
  if (!testData) return;
  if (sliderIdx === undefined)
    sliderIdx = parseInt(document.getElementById('timeSlider').value);
  CHANNELS.forEach(c => renderChart(c, sliderIdx));
}

// ════════════════════════════════════════════
//  Tijdslider
// ════════════════════════════════════════════
function onSlider() {
  const idx = parseInt(document.getElementById('timeSlider').value);
  renderAllCharts(idx);
  updateSliderReadout(idx);
  drawBikeFrame(idx);
  animIdx = idx;
}

function updateSliderReadout(idx) {
  if (!testData) return;
  const s  = testData.samples[idx];
  const t  = (idx * testData.dt_ms / 1000.0).toFixed(2);
  const items = [
    { k: 'Tijd',      v: t + ' s'          },
    { k: 'Hoek',      v: s.angle.toFixed(2) + ' °' },
    { k: 'Error',     v: s.err.toFixed(2)   },
    { k: 'PID out',   v: s.pidOut.toFixed(1) },
    { k: 'P-term',    v: s.p.toFixed(2)     },
    { k: 'I-term',    v: s.i.toFixed(2)     },
    { k: 'D-term',    v: s.d.toFixed(2)     },
    { k: 'Motor 1',   v: s.m1               },
    { k: 'Motor 2',   v: s.m2               },
  ];
  const div = document.getElementById('sliderReadout');
  div.innerHTML = items.map(it =>
    `<div class="readout-row">
       <span class="readout-key">${it.k}</span>
       <span class="readout-val">${it.v}</span>
     </div>`).join('');
}

// ════════════════════════════════════════════
//  Fiets animatie
// ════════════════════════════════════════════
function drawBikeFrame(idx) {
  if (!testData) return;
  const cv  = document.getElementById('bikeCanvas');
  const ctx = cv.getContext('2d');
  const W   = cv.clientWidth || 300;
  cv.width  = W;
  const H   = cv.height;

  ctx.fillStyle = '#0d0d14';
  ctx.fillRect(0, 0, W, H);

  const s     = testData.samples[idx];
  const theta = s.angle * Math.PI / 180.0;

  const pivX  = W / 2;
  const pivY  = H * 0.82;
  const bikeH = H * 0.65;

  // Grondlijn
  ctx.strokeStyle = '#333';
  ctx.lineWidth   = 1.5;
  ctx.beginPath();
  ctx.moveTo(pivX - 80, pivY);
  ctx.lineTo(pivX + 80, pivY);
  ctx.stroke();

  // Referentielijn verticaal
  ctx.strokeStyle = '#1e1e2a';
  ctx.lineWidth   = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath();
  ctx.moveTo(pivX, pivY - bikeH * 1.1);
  ctx.lineTo(pivX, pivY);
  ctx.stroke();
  ctx.setLineDash([]);

  // Fiets lijn
  const tipX = pivX + Math.sin(theta) * bikeH;
  const tipY = pivY - Math.cos(theta) * bikeH;
  ctx.strokeStyle = '#ccc';
  ctx.lineWidth   = 5;
  ctx.lineCap     = 'round';
  ctx.beginPath();
  ctx.moveTo(pivX, pivY);
  ctx.lineTo(tipX, tipY);
  ctx.stroke();

  // Pivot
  ctx.fillStyle = '#fff';
  ctx.beginPath();
  ctx.arc(pivX, pivY, 5, 0, Math.PI * 2);
  ctx.fill();

  // Top
  const topColor = Math.abs(s.angle) > 15 ? '#ff5252' : Math.abs(s.angle) > 5 ? '#ff9800' : '#4caf50';
  ctx.fillStyle = topColor;
  ctx.beginPath();
  ctx.arc(tipX, tipY, 8, 0, Math.PI * 2);
  ctx.fill();

  // Hoek tekst
  ctx.fillStyle = '#aaa';
  ctx.font      = '12px Arial';
  ctx.fillText(s.angle.toFixed(1) + '°', pivX - 80, pivY - bikeH * 1.05);

  // Tijd tekst
  const t = (idx * testData.dt_ms / 1000.0).toFixed(2);
  ctx.fillStyle = '#555';
  ctx.font      = '11px Arial';
  ctx.fillText('t = ' + t + ' s', pivX + 30, pivY - bikeH * 1.05);
  document.getElementById('animTime').textContent = 't = ' + t + ' s  |  hoek = ' + s.angle.toFixed(1) + '°';
}

function playAnim() {
  if (!testData) return;
  animPlaying = true;
  stepAnim();
}
function pauseAnim() { animPlaying = false; if (animFrame) cancelAnimationFrame(animFrame); }
function resetAnim() { pauseAnim(); animIdx = 0; document.getElementById('timeSlider').value = 0; onSlider(); }

function stepAnim() {
  if (!animPlaying || !testData) return;
  if (animIdx >= testData.samples.length - 1) { animPlaying = false; return; }
  animIdx++;
  document.getElementById('timeSlider').value = animIdx;
  drawBikeFrame(animIdx);
  updateSliderReadout(animIdx);
  renderAllCharts(animIdx);
  // Afspelen op echte snelheid: wacht dt_ms milliseconden
  animFrame = setTimeout(stepAnim, testData.dt_ms);
}

// ════════════════════════════════════════════
//  PID / motor sturing (ongewijzigd)
// ════════════════════════════════════════════
async function sendLive() {
  const t  = document.getElementById('target').value;
  const p  = document.getElementById('kp').value;
  const i  = document.getElementById('ki').value;
  const d  = document.getElementById('kd').value;
  const b  = document.getElementById('motorBase').value;
  const tr = document.getElementById('motorTrim').value;
  await fetch('/setPid?target='+encodeURIComponent(t)+'&kp='+encodeURIComponent(p)+'&ki='+encodeURIComponent(i)+'&kd='+encodeURIComponent(d)+'&base='+encodeURIComponent(b)+'&trim='+encodeURIComponent(tr));
}
function queueLive() { if (liveTimer) clearTimeout(liveTimer); liveTimer = setTimeout(sendLive, 250); }
async function savePid()       { await sendLive(); refreshData(); }
async function setPid(on)      { await fetch('/setPidEnable?active='+on); refreshData(); }

async function calibrateImu() {
  if (confirm('Houd de fiets perfect rechtop. OK om te kalibreren!')) {
    await fetch('/calibrate');
    alert('Kalibratie opgeslagen!');
    refreshData();
  }
}
async function resetCal() {
  if (confirm('Kalibratie wissen?')) { await fetch('/resetCal'); refreshData(); }
}

// Herrender grafieken bij resize
window.addEventListener('resize', () => renderAllCharts());

setInterval(refreshData, 250);
refreshData();
</script>
</body>
</html>
)rawliteral";
}

// ────────────────────────────────────────────────────────────
//  Web handlers
// ────────────────────────────────────────────────────────────
void handleRoot()  { server.send(200, "text/html", htmlPage()); }

void handleData() {
  String json = "{";
  json += "\"pitch\":"         + String(pitch,       2) + ",";
  json += "\"target_angle\":"  + String(targetAngle, 2) + ",";
  json += "\"kp\":"            + String(Kp,          3) + ",";
  json += "\"ki\":"            + String(Ki,          3) + ",";
  json += "\"kd\":"            + String(Kd,          3) + ",";
  json += "\"motor_base\":"    + String(motorBase,   1) + ",";
  json += "\"motor_trim\":"    + String(motorTrim,   1) + ",";
  json += "\"pid_enabled\":"   + String(pidEnabled    ? "true" : "false") + ",";
  json += "\"motors_enabled\":" + String(motorsEnabled ? "true" : "false") + ",";
  json += "\"test_running\":"  + String(testRunning    ? "true" : "false") + ",";
  json += "\"test_data_ready\":" + String(testDataReady ? "true" : "false") + ",";
  json += "\"log_count\":"      + String(logCount)        + ",";
  json += "\"balance_offset\":" + String(balance_offset, 2);
  json += "}";
  server.send(200, "application/json", json);
}

void handleStartTest() {
  logCount       = 0;
  testRunning    = true;
  testDataReady  = false;
  motorsEnabled  = true;
  integral       = 0.0f;
  lastError      = 0.0f;
  pidOutput      = 0.0f;
  server.send(200, "text/plain", "OK");
}

void handleStopTest() {
  testRunning    = false;
  testDataReady  = true;
  motorsEnabled  = false;
  motor1.sendThrottle(0);
  motor2.sendThrottle(0);
  server.send(200, "text/plain", "OK");
}


void handleTestData() {
  if (!testDataReady || logCount == 0) {
    server.send(200, "application/json", "{\"samples\":[]}");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");

  String hdr = "{\"dt_ms\":" + String(LOG_INTERVAL) + ",\"samples\":[";
  server.sendContent(hdr);

  const int CHUNK = 50;
  for (int i = 0; i < logCount; i++) {
    const LogSample& s = logBuf[i];
    char buf[128];
    snprintf(buf, sizeof(buf),
      "%s{\"angle\":%.2f,\"p\":%.2f,\"i\":%.2f,\"d\":%.2f,"
      "\"pidOut\":%.1f,\"err\":%.2f,\"m1\":%u,\"m2\":%u}",
      (i > 0 ? "," : ""),
      s.angle, s.p, s.i, s.d,
      s.pidOut, s.err, (unsigned)s.m1, (unsigned)s.m2
    );
    server.sendContent(buf);

    if ((i + 1) % CHUNK == 0) delay(1);
  }

  server.sendContent("]}");
  server.sendContent("");  // einde chunked
}

void handleSetPid() {
  if (server.hasArg("target"))  targetAngle = server.arg("target").toFloat();
  if (server.hasArg("kp"))      Kp          = server.arg("kp").toFloat();
  if (server.hasArg("ki"))      Ki          = server.arg("ki").toFloat();
  if (server.hasArg("kd"))      Kd          = server.arg("kd").toFloat();
  if (server.hasArg("base"))    motorBase   = server.arg("base").toFloat();
  if (server.hasArg("trim")) {
    motorTrim = server.arg("trim").toFloat();
    if (motorTrim >  200.0f) motorTrim =  200.0f;
    if (motorTrim < -200.0f) motorTrim = -200.0f;
  }
  server.send(200, "text/plain", "OK");
}

void handleSetPidEnable() {
  if (server.hasArg("active")) pidEnabled = (server.arg("active") == "1");
  if (!pidEnabled) {
    integral = 0.0f; lastError = 0.0f; pidOutput = 0.0f;
    motor1Raw = 0; motor2Raw = 0;
    motor1.sendThrottle(0); motor2.sendThrottle(0);
  }
  server.send(200, "text/plain", "OK");
}

void handleSetMotors() {
  if (server.hasArg("active")) motorsEnabled = (server.arg("active") == "1");
  if (!motorsEnabled) { motor1.sendThrottle(0); motor2.sendThrottle(0); }
  server.send(200, "text/plain", "OK");
}

void handleCalibrate()      { voerKalibratieUit(); server.send(200, "text/plain", "OK"); }
void handleResetCalibrate() { wisKalibratie();      server.send(200, "text/plain", "OK"); }

// ────────────────────────────────────────────────────────────
//  Setup
// ────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(1500);

  preferences.begin("imu_cal", false);
  balance_offset = preferences.getFloat("balOff",  0.0f);
  roll_offset    = preferences.getFloat("rollOff", 0.0f);

  Wire.begin(I2C_SDA, I2C_SCL);
  setupMPU(MPU1);
  delay(100);

  lastTime    = millis();
  lastPIDTime = millis();

  motor1.begin(); motor2.begin();
  motor1.sendCommand(DSHOT_CMD_MOTOR_STOP);
  motor2.sendCommand(DSHOT_CMD_MOTOR_STOP);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000)
    delay(500);

  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString();
    Serial.print("IP: "); Serial.println(ipAddress);

    server.on("/",          handleRoot);
    server.on("/data",      handleData);
    server.on("/setPid",    handleSetPid);
    server.on("/setPidEnable", handleSetPidEnable);
    server.on("/setMotors", handleSetMotors);
    server.on("/calibrate", handleCalibrate);
    server.on("/resetCal",  handleResetCalibrate);

    // NIEUW: test-endpoints
    server.on("/startTest", handleStartTest);
    server.on("/stopTest",  handleStopTest);
    server.on("/testdata",  handleTestData);

    server.begin();
  }
}

// ────────────────────────────────────────────────────────────
//  Loop 
// ────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();

  unsigned long currentMillis = millis();
  if (currentMillis - lastTime >= interval) {
    float dt = (currentMillis - lastTime) / 1000.0f;
    lastTime = currentMillis;

    // ── IMU lezen  ───────────────────────────────
    int16_t acX1, acY1, acZ1, gyX1, gyY1, gyZ1;
    readRawData(MPU1, acX1, acY1, acZ1, gyX1, gyY1, gyZ1);

    float avgAcX = (float)acX1;
    float avgAcY = (float)acY1;
    float avgAcZ = (float)acZ1;
    float avgGyX = (float)gyX1;
    float avgGyY = (float)gyY1;

    avgAcZ = -avgAcZ;
    avgAcX = -avgAcX;
    avgGyY = -avgGyY;

    float accPitch  = (atan2(-avgAcX, sqrt(pow(avgAcY, 2) + pow(avgAcZ, 2))) * 180.0) / PI;
    float accRoll   = (atan2(avgAcY, avgAcZ) * 180.0) / PI;
    float gyroXrate = avgGyX / 131.0;
    float gyroYrate = avgGyY / 131.0;

    if (firstLoop) {
      absolutePitchRaw = accPitch;
      absoluteRollRaw  = accRoll;
      firstLoop        = false;
    } else {
      float accMagnitude = sqrt(pow(avgAcX, 2) + pow(avgAcY, 2) + pow(avgAcZ, 2));
      float accMagG      = accMagnitude / 16384.0f;
      float alpha = (fabs(accMagG - 1.0f) < 0.15f) ? 0.02f : 0.0f;
      absolutePitchRaw = (1.0f - alpha) * (absolutePitchRaw + gyroYrate * dt) + alpha * accPitch;
      absoluteRollRaw  = (1.0f - alpha) * (absoluteRollRaw  + gyroXrate * dt) + alpha * accRoll;
    }

    pitch = absolutePitchRaw - balance_offset;
    roll  = absoluteRollRaw  - roll_offset;

    // ── PID berekenen  ───────────────────────────
    if (pidEnabled) {
      computePID(pitch, dt);
    } else {
      pidOutput = 0.0f;
    }

    // ── Motor sturing  ───────────────────────────
motor1Raw = 0;
motor2Raw = 0;

if (pidEnabled && motorsEnabled) {
    float absOut = fabs(pidOutput);
    // Begrens zodat motorBase + absOut nooit boven DSHOT_MAX komt
    absOut = constrain(absOut, 0.0f, (float)(DSHOT_THROTTLE_MAX - (int)motorBase));
 

    if (pidOutput >= 0.0f) {
        motor1Raw = throttleFromValue(motorBase + absOut + motorTrim, motorBase);
        motor2Raw = throttleFromValue(motorBase          - motorTrim, motorBase);
    } else {
        motor1Raw = throttleFromValue(motorBase          + motorTrim, motorBase);
        motor2Raw = throttleFromValue(motorBase + absOut - motorTrim, motorBase);
    }

}

    motor1.sendThrottle(motor1Raw);
    motor2.sendThrottle(motor2Raw);


static unsigned long lastLogTime = 0;

if (testRunning && logCount < MAX_SAMPLES) {
  unsigned long now = millis();

  if (now - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = now;

    logBuf[logCount].angle  = pitch;
    logBuf[logCount].p      = pTerm;
    logBuf[logCount].i      = iTerm;
    logBuf[logCount].d      = dTerm;
    logBuf[logCount].pidOut = pidOutput;
    logBuf[logCount].err    = error_val;
    logBuf[logCount].m1     = motor1Raw;
    logBuf[logCount].m2     = motor2Raw;

    logCount++;
  }
}
  }
}
