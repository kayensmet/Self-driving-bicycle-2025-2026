#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <DShotRMT.h>
#include <Preferences.h>
#include <AutoLQRStatic.h>

// ────────────────────────────────────────────────────────────
//  Data-logging voor post-test grafieken
// ────────────────────────────────────────────────────────────
static const int MAX_SAMPLES = 300;

struct LogSample {
  float angle;
  float rate;
  float err;
  float controlOut;
  uint16_t m1;
  uint16_t m2;
};

static LogSample logBuf[MAX_SAMPLES];
static int       logCount      = 0;
static bool      testRunning   = false;
static bool      testDataReady = false;

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
const unsigned long interval    = 10;   // ms tussen IMU-reads
const unsigned long LOG_INTERVAL = 50;   // ms tussen log-samples

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
//  LQR / controller tuning
// ────────────────────────────────────────────────────────────
static constexpr int LQR_STATE_SIZE   = 2;
static constexpr int LQR_CONTROL_SIZE = 1;

AutoLQRStatic<LQR_STATE_SIZE, LQR_CONTROL_SIZE> lqr;

float targetAngle    = 0.0f;
float qAngle         = 10.0f;
float qRate          = 1.0f;
float rControl       = 0.1f;
float modelDamping   = 0.98f;
float modelInputScale = 1.0f;
float controlScale   = 35.0f;

float lqrState[2]   = {0.0f, 0.0f};
float lqrControl[1] = {0.0f};
float lqrK[2]       = {0.0f, 0.0f};
bool  lqrEnabled    = true;
bool  lqrReady      = false;
bool motorsEnabled = false;

float controlOutput = 0.0f;
float angleRateFilt  = 0.0f;
const float RATE_ALPHA = 0.20f;

float motorBase = 100.0f;
float motorTrim = 0.0f;

float A[4] = {1.0f, 0.01f, 0.0f, 0.98f};
float B[2] = {0.00005f, 0.01f};
float Q[4] = {10.0f, 0.0f, 0.0f, 1.0f};
float R[1] = {0.1f};

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
  int16_t tmp = Wire.read() << 8 | Wire.read(); // temperatuur, niet gebruikt
  (void)tmp;
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
  balance_offset = somPitchRaw / 100.0f;
  roll_offset    = somRollRaw  / 100.0f;
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
//  LQR setup
// ────────────────────────────────────────────────────────────
void updateLqrMatrices() {
  const float dt = interval / 1000.0f;

  A[0] = 1.0f;
  A[1] = dt;
  A[2] = 0.0f;
  A[3] = constrain(modelDamping, 0.0f, 1.0f);

  B[0] = 0.5f * dt * dt * modelInputScale;
  B[1] = dt * modelInputScale;

  Q[0] = max(qAngle, 0.0f);
  Q[1] = 0.0f;
  Q[2] = 0.0f;
  Q[3] = max(qRate, 0.0f);

  R[0] = max(rControl, 0.0001f);
}

bool rebuildLqr() {
  updateLqrMatrices();

  bool ok = true;
  ok = ok && lqr.setStateMatrix(A);
  ok = ok && lqr.setInputMatrix(B);
  ok = ok && lqr.setCostMatrices(Q, R);
  ok = ok && lqr.computeGains();

  lqrReady = ok;
  if (lqrReady) {
    lqr.exportGains(lqrK);
  } else {
    lqrK[0] = 0.0f;
    lqrK[1] = 0.0f;
  }
  return lqrReady;
}

void computeLQR(float currentAngle, float currentRate) {
  const float angleError = targetAngle - currentAngle;
  const float rateError  = 0.0f - currentRate;

  lqrState[0] = angleError;
  lqrState[1] = rateError;

  lqr.updateState(lqrState);
  lqr.calculateControl(lqrControl);

  controlOutput = lqrControl[0] * controlScale;
}

// ────────────────────────────────────────────────────────────
//  HTML pagina
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
#postSection { display: none; }
.graph-toggles { display: flex; flex-wrap: wrap; gap: 6px; margin-bottom: 8px; }
.toggle-btn { padding: 5px 11px; border-radius: 6px; font-size: 12px; cursor: pointer; border: 1.5px solid transparent; opacity: 0.4; transition: opacity 0.2s; }
.toggle-btn.active { opacity: 1; border-color: currentColor; }
.chart-wrap  { position: relative; margin-bottom: 6px; }
canvas.chart { display: block; width: 100%; border-radius: 6px; background: #0d0d14; }
.slider-wrap { margin: 4px 0 10px; }
#timeSlider  { width: 100%; accent-color: #2196f3; }
.slider-readout { background: #1c1c1c; border-radius: 8px; padding: 8px 10px; font-size: 12px; margin-top: 4px; }
.readout-grid { display: grid; grid-template-columns: repeat(auto-fill, minmax(140px, 1fr)); gap: 4px 12px; }
.readout-row  { display: flex; justify-content: space-between; }
.readout-key  { color: #888; }
.readout-val  { font-weight: bold; }
#bikeCanvas { display: block; width: 100%; border-radius: 8px; background: #0d0d14; }
.small { font-size: 12px; color: #888; margin-top: 4px; }
</style>
</head>
<body>
<h1>Balans Controller</h1>

<div class="card">
  <div class="label">Hoek (Pitch)</div>
  <div class="angle" id="pitch">-</div>
</div>

<div class="card" id="calCard" style="display:none;">
  <div class="label">Gekalibreerde hoek (offset)</div>
  <div class="status" id="calOffset" style="font-size:20px;">-</div>
  <div style="font-size:11px; color:#666; margin-top:4px;">Target hoek = 0° t.o.v. deze positie</div>
</div>

<div class="row">
  <div class="card">
    <div class="label">Motoren</div>
    <div class="status" id="motorsStatus">-</div>
  </div>
  <div class="card">
    <div class="label">LQR</div>
    <div class="status" id="lqrStatus">-</div>
    <div id="gainText" class="small">K = -</div>
    <div class="btn-row" style="justify-content:center; margin-top:8px;">
      <button class="onBtn"  onclick="setControl(1)">AAN</button>
      <button class="offBtn" onclick="setControl(0)">UIT</button>
    </div>
  </div>
</div>

<div class="card">
  <div class="label" id="testStatusLabel">Test niet actief</div>
  <div class="btn-row" style="margin-top:6px;">
    <button class="startBtn" onclick="startTest()">MOTOR AAN</button>
    <button class="stopBtn"  onclick="stopTest()">MOTOR UIT</button>
  </div>
</div>

<div class="card">
  <h2>LQR Instellingen</h2>
  <label>Target hoek (°)</label>
  <input type="number" id="target" step="0.1" oninput="queueLive()">
  <label>Q angle</label>
  <input type="number" id="qAngle" step="0.1" oninput="queueLive()">
  <label>Q rate</label>
  <input type="number" id="qRate" step="0.1" oninput="queueLive()">
  <label>R control</label>
  <input type="number" id="rControl" step="0.01" oninput="queueLive()">
  <label>Model damping (0.0 - 1.0)</label>
  <input type="number" id="modelDamping" step="0.01" min="0" max="1" oninput="queueLive()">
  <label>Input scale model</label>
  <input type="number" id="modelInputScale" step="0.01" oninput="queueLive()">
  <label>Control scale naar throttle</label>
  <input type="number" id="controlScale" step="0.1" oninput="queueLive()">
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
  <button class="save-btn" onclick="saveLqr()">Opslaan & herbereken</button>
</div>

<div class="card">
  <h2>Kalibratie</h2>
  <div class="btn-row" style="margin-top:4px;">
    <button class="calBtn"   onclick="calibrateImu()">Kalibreer (houd rechtop)</button>
    <button class="resetBtn" onclick="resetCal()">Wis kalibratie</button>
  </div>
</div>

<div id="postSection">
  <div class="card">
    <h2>Testresultaten</h2>
    <div id="testMeta" style="font-size:12px; color:#888; margin-bottom:8px;"></div>
    <div class="graph-toggles" id="graphToggles"></div>
    <div class="slider-wrap">
      <input type="range" id="timeSlider" min="0" max="100" value="0" oninput="onSlider()">
    </div>
    <div class="slider-readout">
      <div style="font-size:11px; color:#666; margin-bottom:4px;">Waarden op geselecteerd moment:</div>
      <div class="readout-grid" id="sliderReadout"></div>
    </div>
    <div id="chartsContainer" style="margin-top:10px;"></div>
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
let testData    = null;
let liveTimer   = null;
let animFrame   = null;
let animIdx     = 0;
let animPlaying = false;

const CHANNELS = [
  { key: 'angle',  label: 'Hoek (°)',          color: '#ef5350', dynamic: true  },
  { key: 'rate',   label: 'Hoeksnelheid (°/s)', color: '#42a5f5', dynamic: true  },
  { key: 'err',    label: 'Hoekfout (°)',       color: '#66bb6a', dynamic: true  },
  { key: 'ctrl',   label: 'LQR output',         color: '#ab47bc', dynamic: true  },
  { key: 'm1',     label: 'Motor 1 DSHOT',      color: '#ffa726', dynamic: false },
  { key: 'm2',     label: 'Motor 2 DSHOT',      color: '#90caf9', dynamic: false },
];
const DSHOT_MAX = 2047;
let channelVisible = {};
CHANNELS.forEach(c => channelVisible[c.key] = true);

async function refreshData() {
  try {
    const data = await (await fetch('/data')).json();
    const el = document.getElementById('pitch');
    el.textContent = data.pitch.toFixed(1) + '°';
    el.className = 'angle ' + (Math.abs(data.pitch) < 3 ? 'ok' : Math.abs(data.pitch) < 10 ? 'warn' : 'danger');

    const mEl = document.getElementById('motorsStatus');
    mEl.textContent = data.motors_enabled ? 'AAN' : 'UIT';
    mEl.className   = 'status ' + (data.motors_enabled ? 'on' : 'off');

    const lEl = document.getElementById('lqrStatus');
    if (data.lqr_enabled) {
      lEl.textContent = data.lqr_ready ? 'AAN' : 'AAN (geen gains)';
      lEl.className = 'status ' + (data.lqr_ready ? 'on' : 'off');
    } else {
      lEl.textContent = 'UIT';
      lEl.className = 'status off';
    }

    document.getElementById('gainText').textContent =
      'K = [' + data.k1.toFixed(2) + ', ' + data.k2.toFixed(2) + ']';

    document.getElementById('testStatusLabel').textContent = data.test_running
      ? ('Test loopt… ' + data.log_count + ' samples opgeslagen')
      : (data.test_data_ready ? 'Test klaar — zie resultaten hieronder' : 'Test niet actief');

    updateInputIfNotFocused('target',          data.target_angle);
    updateInputIfNotFocused('qAngle',          data.q_angle);
    updateInputIfNotFocused('qRate',           data.q_rate);
    updateInputIfNotFocused('rControl',        data.r_control);
    updateInputIfNotFocused('modelDamping',    data.model_damping);
    updateInputIfNotFocused('modelInputScale', data.model_input_scale);
    updateInputIfNotFocused('controlScale',    data.control_scale);
    updateInputIfNotFocused('motorBase',       data.motor_base);
    updateInputIfNotFocused('motorTrim',       data.motor_trim);
    if (document.activeElement !== document.getElementById('motorTrim')) updateTrimBar();

    const calCard = document.getElementById('calCard');
    const calEl   = document.getElementById('calOffset');
    if (data.balance_offset !== 0) {
      calCard.style.display = '';
      calEl.textContent = data.balance_offset.toFixed(2) + '°';
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

async function startTest() { await fetch('/startTest'); }
async function stopTest()  {
  await fetch('/stopTest');
  setTimeout(fetchTestData, 400);
}

async function fetchTestData() {
  try {
    const json = await (await fetch('/testdata')).json();
    if (!json || !json.samples || json.samples.length === 0) return;
    testData = json;
    buildPostSection();
    document.getElementById('postSection').style.display = 'block';
    document.getElementById('postSection').scrollIntoView({ behavior: 'smooth' });
  } catch(e) { console.error('testdata ophalen mislukt', e); }
}

function buildPostSection() {
  const n  = testData.samples.length;
  const dt = testData.dt_ms / 1000.0;
  document.getElementById('testMeta').textContent =
    n + ' samples  ·  ' + (n * dt).toFixed(1) + ' s  ·  interval ' + testData.dt_ms + ' ms';

  const slider = document.getElementById('timeSlider');
  slider.max   = n - 1;
  slider.value = 0;

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

  const cDiv = document.getElementById('chartsContainer');
  cDiv.innerHTML = '';
  CHANNELS.forEach(c => {
    const wrap = document.createElement('div');
    wrap.className = 'chart-wrap';
    wrap.id        = 'wrap_' + c.key;
    const cv = document.createElement('canvas');
    cv.id        = 'chart_' + c.key;
    cv.height    = 110;
    cv.className = 'chart';
    wrap.appendChild(cv);
    cDiv.appendChild(wrap);
  });

  renderAllCharts();
  updateSliderReadout(0);
  drawBikeFrame(0);
}

function getValues(key) { return testData.samples.map(s => s[key]); }

function dynamicRange(vals, minHalf) {
  let peak = minHalf;
  vals.forEach(v => { if (Math.abs(v) > peak) peak = Math.abs(v); });
  return { yMin: -(peak * 1.3), yMax: peak * 1.3 };
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
  const vals = getValues(ch.key);
  const n    = vals.length;
  const ML = 38, MR = 6, MT = 14, MB = 18;
  const gW = W - ML - MR, gH = H - MT - MB;

  let yMin, yMax;
  if (!ch.dynamic) { yMin = 0; yMax = DSHOT_MAX; }
  else { const r = dynamicRange(vals, 1); yMin = r.yMin; yMax = r.yMax; }
  const yRange = yMax - yMin || 1;

  const tx = i => ML + (i / (n - 1)) * gW;
  const ty = v => MT + gH - (v - yMin) / yRange * gH;

  ctx.fillStyle = '#0d0d14';
  ctx.fillRect(0, 0, W, H);

  ctx.strokeStyle = '#1e1e2a'; ctx.lineWidth = 0.5;
  for (let t = 0; t <= 4; t++) {
    const v = yMin + t * yRange / 4;
    const py = ty(v);
    ctx.beginPath(); ctx.moveTo(ML, py); ctx.lineTo(ML + gW, py); ctx.stroke();
    ctx.fillStyle = '#555'; ctx.font = '9px Arial';
    ctx.fillText(v.toFixed(ch.dynamic ? 1 : 0), 0, py + 3);
  }

  if (yMin < 0 && yMax > 0) {
    const py = ty(0);
    ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
    ctx.beginPath(); ctx.moveTo(ML, py); ctx.lineTo(ML + gW, py); ctx.stroke();
  }

  if (n > 1) {
    ctx.strokeStyle = ch.color; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(tx(0), ty(vals[0]));
    for (let i = 1; i < n; i++) ctx.lineTo(tx(i), ty(vals[i]));
    ctx.stroke();
  }

  if (sliderIdx !== undefined && n > 1) {
    const sx = tx(sliderIdx);
    ctx.strokeStyle = 'rgba(255,255,255,0.5)'; ctx.lineWidth = 1;
    ctx.setLineDash([3, 3]);
    ctx.beginPath(); ctx.moveTo(sx, MT); ctx.lineTo(sx, MT + gH); ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = '#fff';
    ctx.beginPath(); ctx.arc(sx, ty(vals[sliderIdx]), 3.5, 0, Math.PI * 2); ctx.fill();
  }

  ctx.fillStyle = '#ccc'; ctx.font = 'bold 10px Arial';
  ctx.fillText(ch.label, ML + 4, MT + 11);
  ctx.strokeStyle = '#333'; ctx.lineWidth = 1;
  ctx.strokeRect(ML, MT, gW, gH);
}

function renderAllCharts(sliderIdx) {
  if (!testData) return;
  if (sliderIdx === undefined) sliderIdx = parseInt(document.getElementById('timeSlider').value);
  CHANNELS.forEach(c => renderChart(c, sliderIdx));
}

function onSlider() {
  const idx = parseInt(document.getElementById('timeSlider').value);
  renderAllCharts(idx);
  updateSliderReadout(idx);
  drawBikeFrame(idx);
  animIdx = idx;
}

function updateSliderReadout(idx) {
  if (!testData) return;
  const s = testData.samples[idx];
  const t = (idx * testData.dt_ms / 1000.0).toFixed(2);
  const items = [
    { k: 'Tijd',     v: t + ' s' },
    { k: 'Hoek',     v: s.angle.toFixed(2) + ' °' },
    { k: 'Rate',     v: s.rate.toFixed(2) + ' °/s' },
    { k: 'Error',    v: s.err.toFixed(2) },
    { k: 'Control',  v: s.ctrl.toFixed(2) },
    { k: 'Motor 1',  v: s.m1 },
    { k: 'Motor 2',  v: s.m2 },
  ];
  document.getElementById('sliderReadout').innerHTML = items.map(it =>
    `<div class="readout-row"><span class="readout-key">${it.k}</span><span class="readout-val">${it.v}</span></div>`
  ).join('');
}

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
  const pivX  = W / 2, pivY = H * 0.82, bikeH = H * 0.65;

  ctx.strokeStyle = '#333'; ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(pivX - 80, pivY); ctx.lineTo(pivX + 80, pivY); ctx.stroke();

  ctx.strokeStyle = '#1e1e2a'; ctx.lineWidth = 1;
  ctx.setLineDash([4, 4]);
  ctx.beginPath(); ctx.moveTo(pivX, pivY - bikeH * 1.1); ctx.lineTo(pivX, pivY); ctx.stroke();
  ctx.setLineDash([]);

  const tipX = pivX + Math.sin(theta) * bikeH;
  const tipY = pivY - Math.cos(theta) * bikeH;
  ctx.strokeStyle = '#ccc'; ctx.lineWidth = 5; ctx.lineCap = 'round';
  ctx.beginPath(); ctx.moveTo(pivX, pivY); ctx.lineTo(tipX, tipY); ctx.stroke();

  ctx.fillStyle = '#fff';
  ctx.beginPath(); ctx.arc(pivX, pivY, 5, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = Math.abs(s.angle) > 15 ? '#ff5252' : Math.abs(s.angle) > 5 ? '#ff9800' : '#4caf50';
  ctx.beginPath(); ctx.arc(tipX, tipY, 8, 0, Math.PI * 2); ctx.fill();

  ctx.fillStyle = '#aaa'; ctx.font = '12px Arial';
  ctx.fillText(s.angle.toFixed(1) + '°', pivX - 80, pivY - bikeH * 1.05);
  ctx.fillStyle = '#555'; ctx.font = '11px Arial';
  ctx.fillText('t = ' + (idx * testData.dt_ms / 1000.0).toFixed(2) + ' s', pivX + 30, pivY - bikeH * 1.05);
  document.getElementById('animTime').textContent =
    't = ' + (idx * testData.dt_ms / 1000.0).toFixed(2) + ' s  |  hoek = ' + s.angle.toFixed(1) + '°';
}

function playAnim()  { if (!testData) return; animPlaying = true; stepAnim(); }
function pauseAnim() { animPlaying = false; if (animFrame) clearTimeout(animFrame); }
function resetAnim() { pauseAnim(); animIdx = 0; document.getElementById('timeSlider').value = 0; onSlider(); }

function stepAnim() {
  if (!animPlaying || !testData) return;
  if (animIdx >= testData.samples.length - 1) { animPlaying = false; return; }
  animIdx++;
  document.getElementById('timeSlider').value = animIdx;
  drawBikeFrame(animIdx);
  updateSliderReadout(animIdx);
  renderAllCharts(animIdx);
  animFrame = setTimeout(stepAnim, testData.dt_ms);
}

async function sendLive() {
  const t  = document.getElementById('target').value;
  const qa = document.getElementById('qAngle').value;
  const qr = document.getElementById('qRate').value;
  const rc = document.getElementById('rControl').value;
  const md = document.getElementById('modelDamping').value;
  const mi = document.getElementById('modelInputScale').value;
  const cs = document.getElementById('controlScale').value;
  const b  = document.getElementById('motorBase').value;
  const tr = document.getElementById('motorTrim').value;

  await fetch('/setLqr?target=' + encodeURIComponent(t)
    + '&qAngle=' + encodeURIComponent(qa)
    + '&qRate=' + encodeURIComponent(qr)
    + '&rControl=' + encodeURIComponent(rc)
    + '&modelDamping=' + encodeURIComponent(md)
    + '&modelInputScale=' + encodeURIComponent(mi)
    + '&controlScale=' + encodeURIComponent(cs)
    + '&base=' + encodeURIComponent(b)
    + '&trim=' + encodeURIComponent(tr));
}
function queueLive() { if (liveTimer) clearTimeout(liveTimer); liveTimer = setTimeout(sendLive, 250); }
async function saveLqr()  { await sendLive(); refreshData(); }
async function setControl(on) { await fetch('/setControlEnable?active=' + on); refreshData(); }

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
void handleRoot() { server.send(200, "text/html", htmlPage()); }

void handleData() {
  String json = "{";
  json += "\"pitch\":"            + String(pitch,             2) + ",";
  json += "\"target_angle\":"     + String(targetAngle,       2) + ",";
  json += "\"q_angle\":"          + String(qAngle,            3) + ",";
  json += "\"q_rate\":"           + String(qRate,             3) + ",";
  json += "\"r_control\":"        + String(rControl,          3) + ",";
  json += "\"model_damping\":"    + String(modelDamping,      3) + ",";
  json += "\"model_input_scale\":" + String(modelInputScale,   3) + ",";
  json += "\"control_scale\":"    + String(controlScale,      3) + ",";
  json += "\"motor_base\":"       + String(motorBase,         1) + ",";
  json += "\"motor_trim\":"       + String(motorTrim,         1) + ",";
  json += "\"lqr_enabled\":"      + String(lqrEnabled    ? "true" : "false") + ",";
  json += "\"lqr_ready\":"        + String(lqrReady      ? "true" : "false") + ",";
  json += "\"motors_enabled\":"   + String(motorsEnabled ? "true" : "false") + ",";
  json += "\"test_running\":"     + String(testRunning   ? "true" : "false") + ",";
  json += "\"test_data_ready\":"  + String(testDataReady ? "true" : "false") + ",";
  json += "\"log_count\":"        + String(logCount) + ",";
  json += "\"balance_offset\":"   + String(balance_offset,   2) + ",";
  json += "\"k1\":"               + String(lqrK[0],          3) + ",";
  json += "\"k2\":"               + String(lqrK[1],          3);
  json += "}";
  server.send(200, "application/json", json);
}

void handleStartTest() {
  logCount      = 0;
  testRunning   = true;
  testDataReady = false;
  motorsEnabled = true;
  controlOutput = 0.0f;
  lqrControl[0] = 0.0f;
  angleRateFilt = 0.0f;
  server.send(200, "text/plain", "OK");
}

void handleStopTest() {
  testRunning   = false;
  testDataReady = true;
  motorsEnabled = false;
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
  for (int i = 0; i < logCount; i++) {
    const LogSample& s = logBuf[i];
    char buf[128];
    snprintf(buf, sizeof(buf),
      "%s{\"angle\":%.2f,\"rate\":%.2f,\"err\":%.2f,\"ctrl\":%.2f,\"m1\":%u,\"m2\":%u}",
      (i > 0 ? "," : ""),
      s.angle, s.rate, s.err, s.controlOut, (unsigned)s.m1, (unsigned)s.m2);
    server.sendContent(buf);
    if ((i + 1) % 50 == 0) delay(1);
  }
  server.sendContent("]}");
  server.sendContent("");
}

void saveLqrSettingsToPrefs() {
  preferences.putFloat("tgt",  targetAngle);
  preferences.putFloat("qA",   qAngle);
  preferences.putFloat("qR",   qRate);
  preferences.putFloat("rC",   rControl);
  preferences.putFloat("damp", modelDamping);
  preferences.putFloat("inSc", modelInputScale);
  preferences.putFloat("cSc",  controlScale);
  preferences.putFloat("base", motorBase);
  preferences.putFloat("trim", motorTrim);
}

void handleSetLqr() {
  if (server.hasArg("target"))         targetAngle     = server.arg("target").toFloat();
  if (server.hasArg("qAngle"))         qAngle         = server.arg("qAngle").toFloat();
  if (server.hasArg("qRate"))          qRate          = server.arg("qRate").toFloat();
  if (server.hasArg("rControl"))       rControl       = server.arg("rControl").toFloat();
  if (server.hasArg("modelDamping"))   modelDamping   = server.arg("modelDamping").toFloat();
  if (server.hasArg("modelInputScale")) modelInputScale = server.arg("modelInputScale").toFloat();
  if (server.hasArg("controlScale"))    controlScale   = server.arg("controlScale").toFloat();
  if (server.hasArg("base"))           motorBase      = server.arg("base").toFloat();
  if (server.hasArg("trim")) {
    motorTrim = server.arg("trim").toFloat();
    motorTrim = constrain(motorTrim, -200.0f, 200.0f);
  }

  modelDamping    = constrain(modelDamping, 0.0f, 1.0f);
  modelInputScale = max(modelInputScale, 0.0001f);
  controlScale    = max(controlScale, 0.0f);
  motorBase       = constrain(motorBase, 48.0f, 500.0f);

  saveLqrSettingsToPrefs();
  rebuildLqr();
  server.send(200, "text/plain", "OK");
}

void handleSetControlEnable() {
  if (server.hasArg("active")) lqrEnabled = (server.arg("active") == "1");
  if (!lqrEnabled) {
    controlOutput = 0.0f;
    lqrControl[0] = 0.0f;
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
  balance_offset  = preferences.getFloat("balOff", 0.0f);
  roll_offset     = preferences.getFloat("rollOff", 0.0f);

  targetAngle     = preferences.getFloat("tgt", 0.0f);
  qAngle          = preferences.getFloat("qA", 10.0f);
  qRate           = preferences.getFloat("qR", 1.0f);
  rControl        = preferences.getFloat("rC", 0.1f);
  modelDamping    = preferences.getFloat("damp", 0.98f);
  modelInputScale = preferences.getFloat("inSc", 1.0f);
  controlScale    = preferences.getFloat("cSc", 35.0f);
  motorBase       = preferences.getFloat("base", 100.0f);
  motorTrim       = preferences.getFloat("trim", 0.0f);

  Wire.begin(I2C_SDA, I2C_SCL);
  setupMPU(MPU1);
  delay(100);

  lastTime = millis();

  motor1.begin();
  motor2.begin();

  updateLqrMatrices();
  rebuildLqr();

  // Stuur nul-throttle tijdens WiFi connect zodat ESCs armed blijven
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 15000) {
    motor1.sendThrottle(0);
    motor2.sendThrottle(0);
    delay(20);
  }

  // Extra arm-puls: 500ms nul throttle na verbinding
  unsigned long armStart = millis();
  while (millis() - armStart < 500) {
    motor1.sendThrottle(0);
    motor2.sendThrottle(0);
    delay(1);
  }

  if (WiFi.status() == WL_CONNECTED) {
    ipAddress = WiFi.localIP().toString();
    Serial.print("IP: "); Serial.println(ipAddress);
  }

  server.on("/",             handleRoot);
  server.on("/data",         handleData);
  server.on("/setLqr",       handleSetLqr);
  server.on("/setPid",       handleSetLqr);            // backward compatibility
  server.on("/setControlEnable", handleSetControlEnable);
  server.on("/setPidEnable",  handleSetControlEnable);  // backward compatibility
  server.on("/setMotors",     handleSetMotors);
  server.on("/calibrate",     handleCalibrate);
  server.on("/resetCal",      handleResetCalibrate);
  server.on("/startTest",     handleStartTest);
  server.on("/stopTest",      handleStopTest);
  server.on("/testdata",      handleTestData);
  server.begin();
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

    // ── IMU lezen ────────────────────────────────────────────
    int16_t acX1, acY1, acZ1, gyX1, gyY1, gyZ1;
    readRawData(MPU1, acX1, acY1, acZ1, gyX1, gyY1, gyZ1);

    float avgAcX = -(float)acX1;
    float avgAcY =  (float)acY1;
    float avgAcZ = -(float)acZ1;
    float avgGyX =  (float)gyX1;
    float avgGyY = -(float)gyY1;

    float accPitch  = (atan2(-avgAcX, sqrt(pow(avgAcY, 2) + pow(avgAcZ, 2))) * 180.0) / PI;
    float accRoll   = (atan2(avgAcY, avgAcZ) * 180.0) / PI;
    float gyroXrate = avgGyX / 131.0f;
    float gyroYrate = avgGyY / 131.0f;

    if (firstLoop) {
      absolutePitchRaw = accPitch;
      absoluteRollRaw  = accRoll;
      angleRateFilt    = gyroYrate;
      firstLoop        = false;
    } else {
      float accMagG = sqrt(pow(avgAcX, 2) + pow(avgAcY, 2) + pow(avgAcZ, 2)) / 16384.0f;
      float alpha   = (fabs(accMagG - 1.0f) < 0.15f) ? 0.02f : 0.0f;
      absolutePitchRaw = (1.0f - alpha) * (absolutePitchRaw + gyroYrate * dt) + alpha * accPitch;
      absoluteRollRaw  = (1.0f - alpha) * (absoluteRollRaw  + gyroXrate * dt) + alpha * accRoll;
      angleRateFilt    = RATE_ALPHA * gyroYrate + (1.0f - RATE_ALPHA) * angleRateFilt;
    }

    pitch = absolutePitchRaw - balance_offset;
    roll  = absoluteRollRaw  - roll_offset;

    // ── LQR berekenen ────────────────────────────────────────
    if (lqrEnabled && lqrReady) {
      computeLQR(pitch, angleRateFilt);
    } else {
      controlOutput = 0.0f;
      lqrControl[0] = 0.0f;
    }

    // ── Motor sturing ────────────────────────────────────────
    motor1Raw = 0;
    motor2Raw = 0;

    if (lqrEnabled && motorsEnabled && lqrReady) {
      float maxDelta = (float)DSHOT_THROTTLE_MAX - motorBase;
      float absOut   = constrain(fabs(controlOutput), 0.0f, maxDelta);

      if (controlOutput >= 0.0f) {
        motor1Raw = throttleFromValue(motorBase + absOut + motorTrim, motorBase);
        motor2Raw = throttleFromValue(motorBase           - motorTrim, motorBase);
      } else {
        motor1Raw = throttleFromValue(motorBase           + motorTrim, motorBase);
        motor2Raw = throttleFromValue(motorBase + absOut  - motorTrim, motorBase);
      }
    }

    // ALTIJD throttle sturen zodat ESCs armed blijven, ook als motorsEnabled=false
    motor1.sendThrottle(motor1Raw);
    motor2.sendThrottle(motor2Raw);

    // ── Logging ──────────────────────────────────────────────
    static unsigned long lastLogTime = 0;
    if (testRunning && logCount < MAX_SAMPLES) {
      unsigned long now = millis();
      if (now - lastLogTime >= LOG_INTERVAL) {
        lastLogTime = now;
        logBuf[logCount] = { pitch, angleRateFilt, targetAngle - pitch, controlOutput, motor1Raw, motor2Raw };
        logCount++;
      }
    }
  }
}
