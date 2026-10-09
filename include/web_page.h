#pragma once
#include <Arduino.h>

const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Pan/Tilt Camera</title>
<style>
  * { box-sizing:border-box; }
  body { font-family: sans-serif; font-size:18px; text-align: center; background:#111; color:#eee; margin:0; padding:16px; overflow-x:hidden; }
  h2 { margin: 8px 0 16px; }
  .status { background:#222; border-radius:10px; padding:10px; margin-bottom:20px; display:inline-block; min-width:220px; font-size:24px; }
  .status div { padding:2px 0; }
  .status div:first-child { font-size:24px; }
  .pad { display:grid; grid-template-columns: 70px 70px 70px; grid-template-rows: 70px 70px 70px; gap:8px; justify-content:center; margin-bottom:20px; }
  button { font-size:22px; border:none; border-radius:10px; background:#2d6cdf; color:#fff; touch-action:none; user-select:none; }
  button:active { background:#1a4fb0; }
  button:disabled { opacity:0.5; }
  #home, #saveHome, #clearHome { font-size:18px; border-radius:10px; }
  #home { background:#d94f2b; padding:12px 24px; }
  #saveHome { background:#2b9e4f; padding:10px 18px; margin-left:8px; }
  #clearHome { background:#c0392b; padding:10px 18px; margin-left:8px; }
  .b1{grid-column:2;grid-row:1;} .b2{grid-column:1;grid-row:2;} .b3{grid-column:3;grid-row:2;} .b4{grid-column:2;grid-row:3;}
  .pad button { background:#c0392b; }
  .pad button.arrived { background:#2b9e4f; }
  .presets { margin-top:20px; display:flex; gap:8px; justify-content:center; align-items:center; flex-wrap:wrap; }
  .presets input { width:56px; font-size:18px; padding:8px; border-radius:8px; border:none; text-align:center; }
  .presets button { font-size:16px; padding:10px 16px; }
  #goAzimuthBtn { background:#2d6cdf; }
  #goAzimuthBtn.driving { background:#c0392b; }
  #goAzimuthBtn.arrived { background:#2b9e4f; }
  .panel { background:#222; border-radius:10px; padding:14px; margin-top:20px; display:inline-block; min-width:260px; text-align:left; }
  .panel h3 { margin:0 0 10px; font-size:14px; color:#9ab; text-align:center; }
  .speedRow { display:flex; align-items:center; gap:10px; }
  .speedRow input[type=range] { flex:1; }
  .speedRow span { min-width:28px; text-align:right; }
    .stepRow { display:flex; align-items:center; gap:8px; margin-top:8px; }
    .stepRow label { flex:1; }
    .stepRow input { width:64px; font-size:16px; padding:6px; border-radius:6px; border:none; text-align:right; }
  .presetRow { display:flex; align-items:center; gap:8px; margin-top:8px; }
  .presetRow > span { flex:1; }
  .presetInfo { display:block; font-size:11px; color:#9ab; }
  .presetRow button { font-size:14px; padding:8px 12px; }
  .presetRow button.save { background:#6b2fa0; }
  .calRow { display:flex; gap:8px; justify-content:center; margin-top:4px; flex-wrap:wrap; }
  .calRow button { font-size:14px; padding:10px 14px; }
  .gotoBtn.driving { background:#c0392b; }
  .gotoBtn.arrived { background:#2b9e4f; }
  .limitAxis { margin-top:10px; }
  .limitAxis .lbl { font-size:13px; color:#9ab; margin-bottom:4px; }
  @media (max-width:600px) {
    body { padding:10px; }
    h2 { font-size:22px; margin:6px 0 12px; }
    .status, .panel { width:100%; min-width:0; }
    .status { margin-bottom:14px; }
    .pad { grid-template-columns:repeat(3, minmax(54px, 72px)); grid-template-rows:repeat(3, 64px); gap:6px; margin-bottom:14px; }
    .pad button { font-size:20px; }
    #home, #saveHome, #clearHome { margin:4px 2px; }
    .presets { margin-top:14px; }
    .presets input { width:72px; }
    .panel { padding:12px; margin-top:14px; }
    .calRow button { flex:1 1 120px; }
    .limitAxis .row span { min-width:0; }
  }
</style>
</head>
<body>
<h2>Pan / Tilt Camera</h2>
<div class="status">
  <div>Azimut: <span id="heading">--</span> deg<span id="noHome"></span></div>
  <div>Pan: <span id="pan">--</span> deg</div>
  <div>Motor Pan: <span id="rawPan">--</span> deg</div>
  <div>Tilt: <span id="tiltRel">--</span> deg (0 = horizontal)</div>
</div>
<div class="pad">
  <button class="b1" data-axis="tilt" data-dir="1">&#9650;</button>
  <button class="b2" data-axis="pan" data-dir="-1">&#9664;</button>
  <button class="b3" data-axis="pan" data-dir="1">&#9654;</button>
  <button class="b4" data-axis="tilt" data-dir="-1">&#9660;</button>
</div>
<div><button id="home">HOME</button><button id="saveHome">Save Home</button><button id="clearHome">Clear Home</button></div>
<div class="presets">
  <input type="number" id="targetAzimuth" min="0" max="359" value="0">
  <button id="goAzimuthBtn">Go to Azimut</button>
</div>

<div class="panel">
  <h3>JOG SPEED</h3>
  <div class="speedRow">
    <input type="range" id="speedSlider" min="1" max="63" value="63">
    <span id="speedVal">63</span>
  </div>
  <div class="stepRow">
    <label for="panStepInput">Pan step (deg)</label>
    <input type="number" id="panStepInput" min="0.1" max="20" step="0.1" value="2">
  </div>
  <div class="stepRow">
    <label for="tiltStepInput">Tilt step (deg)</label>
    <input type="number" id="tiltStepInput" min="0.1" max="20" step="0.1" value="1">
  </div>
</div>

<div class="panel">
  <h3>POSITION PRESETS</h3>
  <div id="presetRows"></div>
</div>

<div class="panel">
  <h3>POSITION TARGETS</h3>
  <div class="calRow">
    <button class="gotoBtn" id="goToPanMidBtn">Pan Mid (175&deg;)</button>
    <button class="gotoBtn" id="goToTiltZeroBtn">Tilt Zero (29&deg;)</button>
  </div>
  <div class="limitAxis">
    <div class="lbl">Pan limits: <span id="panLimits">min -- / max --</span></div>
    <div class="lbl">Tilt limits: <span id="tiltLimits">min -- / max --</span></div>
  </div>
</div>

<script>
// Each click moves a fixed step (see PELCO_STEP_DEG in config.h). Red by
// default/while moving, green once the motor has reached the new position -
// the request blocks server-side for the step's duration, so its resolution
// IS the "reached" signal (button disabled meanwhile to prevent piling up
// presses).
document.querySelectorAll('button[data-axis]').forEach(btn => {
  const axis = btn.dataset.axis, dir = btn.dataset.dir;
  btn.addEventListener('click', () => {
    btn.disabled = true;
    btn.classList.remove('arrived');
    fetch(`/move?axis=${axis}&dir=${dir}`)
      .then(() => { btn.classList.add('arrived'); })
      .catch(()=>{})
      .finally(() => { btn.disabled = false; });
  });
});
document.getElementById('home').addEventListener('click', () => fetch('/home').catch(()=>{}));
document.getElementById('saveHome').addEventListener('click', () => {
  if (confirm('Save the current position and heading as Home?')) {
    fetch('/saveHome').catch(()=>{});
  }
});
document.getElementById('clearHome').addEventListener('click', () => {
  if (confirm('Clear the saved Home? Azimut will go back to live compass tracking.')) {
    fetch('/clearHome').catch(()=>{});
  }
});

// Red while the head is auto-driving toward the target, green once
// /status reports it arrived (see autoDrive in the JSON, driven by
// updateAutoDrive() on the device).
let azDriving = false;
const goAzimuthBtn = document.getElementById('goAzimuthBtn');
goAzimuthBtn.addEventListener('click', () => {
  const t = document.getElementById('targetAzimuth').value;
  goAzimuthBtn.classList.remove('arrived');
  goAzimuthBtn.classList.add('driving');
  azDriving = true;
  fetch(`/goAzimuth?target=${t}`).then(async r => {
    const msg = await r.text();
    if (!r.ok) {
      azDriving = false;
      goAzimuthBtn.classList.remove('driving');
      alert('Go to Azimut failed: ' + msg);
    } else if (msg !== 'ok') {
      alert(msg);
    }
  }).catch(()=>{});
});

// Jog speed (Pelco speed byte 0-63, sent with every move/step command -
// see jogSpeed in main.cpp). Only pushed on 'change' (release/blur), not
// every 'input' tick, to avoid flooding the RS485-adjacent web server with
// requests while dragging.
const speedSlider = document.getElementById('speedSlider');
const speedVal = document.getElementById('speedVal');
speedSlider.addEventListener('input', () => { speedVal.textContent = speedSlider.value; });
speedSlider.addEventListener('change', () => {
  fetch(`/setSpeed?value=${speedSlider.value}`).catch(()=>{});
});

const panStepInput = document.getElementById('panStepInput');
const tiltStepInput = document.getElementById('tiltStepInput');
function saveSteps() {
  fetch(`/setSteps?pan=${panStepInput.value}&tilt=${tiltStepInput.value}`).catch(()=>{});
}
panStepInput.addEventListener('change', saveSteps);
tiltStepInput.addEventListener('change', saveSteps);

// Three user position presets (preset numbers 2-4; preset 1 is HOME - see
// PELCO_USER_PRESET_BASE/COUNT in config.h). Save asks for confirmation
// since it overwrites whatever was there before, matching Save Home.
const PRESET_COUNT = 3, PRESET_BASE = 2;
const presetRows = document.getElementById('presetRows');
for (let i = 0; i < PRESET_COUNT; i++) {
  const num = PRESET_BASE + i;
  const row = document.createElement('div');
  row.className = 'presetRow';
  row.innerHTML = `<span>Preset ${i + 1} <span class="presetInfo" id="presetInfo${i}">(--)</span></span>` +
    `<button class="save" data-num="${num}">Save</button>` +
    `<button class="go" data-num="${num}">Go</button>`;
  presetRows.appendChild(row);
}
presetRows.addEventListener('click', (e) => {
  const btn = e.target.closest('button[data-num]');
  if (!btn) return;
  const num = btn.dataset.num;
  if (btn.classList.contains('save')) {
    if (confirm(`Save the current position as Preset ${num - PRESET_BASE + 1}?`)) {
      fetch(`/presetSet?num=${num}`).catch(()=>{});
    }
  } else {
    fetch(`/presetGo?num=${num}`).catch(()=>{});
  }
});

// Fixed position-target buttons: drive straight to a known raw motor
// position (see PAN_MID_TARGET_DEG/TILT_ZERO_TARGET_DEG in config.h). Red
// while driving, green once /status reports arrival (goToPan/goToTilt).
let goToPanDriving = false, goToTiltDriving = false;
const goToPanMidBtn = document.getElementById('goToPanMidBtn');
const goToTiltZeroBtn = document.getElementById('goToTiltZeroBtn');
goToPanMidBtn.addEventListener('click', () => {
  goToPanMidBtn.classList.remove('arrived');
  goToPanMidBtn.classList.add('driving');
  goToPanDriving = true;
  fetch('/goToPanMid').catch(()=>{});
});
goToTiltZeroBtn.addEventListener('click', () => {
  goToTiltZeroBtn.classList.remove('arrived');
  goToTiltZeroBtn.classList.add('driving');
  goToTiltDriving = true;
  fetch('/goToTiltZero').catch(()=>{});
});

// Fixed limits from config.h (PAN_LIMIT_*/TILT_LIMIT_*), shown read-only.
function fmtLimits(min, max) {
  return `min ${min.toFixed(1)} / max ${max.toFixed(1)}`;
}

// Tilt is shown relative to TILT_ZERO_TARGET_DEG (horizontal) - explicit
// sign so + (forward/up) vs - (backward/down) from level is unambiguous.
function fmtSigned(v) {
  return (v >= 0 ? '+' : '') + v.toFixed(1);
}

function fmtPreset(p) {
  const az = p && p.azOk ? p.az.toFixed(1) + '°' : '--';
  const tilt = p && p.tiltOk ? fmtSigned(p.tilt) + '°' : '--';
  return `(Az ${az}, Tilt ${tilt})`;
}

function poll() {
  fetch('/status').then(r => r.json()).then(s => {
    document.getElementById('heading').textContent = s.heading.toFixed(1);
    document.getElementById('noHome').textContent = s.homeSet ? '' : ' (no home saved)';
    document.getElementById('pan').textContent = s.posKnown ? s.pan.toFixed(1) : '?? (no home saved)';
    document.getElementById('rawPan').textContent = s.rawPanOk ? s.rawPan.toFixed(1) : '??';
    document.getElementById('tiltRel').textContent = s.tiltRelOk ? fmtSigned(s.tiltRel) : '??';
    if (azDriving && !s.autoDrive) {
      azDriving = false;
      goAzimuthBtn.classList.remove('driving');
      goAzimuthBtn.classList.add('arrived');
    }
    if (goToPanDriving && !s.goToPan) {
      goToPanDriving = false;
      goToPanMidBtn.classList.remove('driving');
      goToPanMidBtn.classList.add('arrived');
    }
    if (goToTiltDriving && !s.goToTilt) {
      goToTiltDriving = false;
      goToTiltZeroBtn.classList.remove('driving');
      goToTiltZeroBtn.classList.add('arrived');
    }

    if (document.activeElement !== speedSlider) {
        if (document.activeElement !== panStepInput) panStepInput.value = s.panStep;
        if (document.activeElement !== tiltStepInput) tiltStepInput.value = s.tiltStep;
      speedSlider.value = s.speed;
      speedVal.textContent = s.speed;
    }

    document.getElementById('panLimits').textContent = fmtLimits(s.panMin, s.panMax);
    document.getElementById('tiltLimits').textContent = fmtLimits(s.tiltMin, s.tiltMax);

    if (s.presets) {
      s.presets.forEach((p, i) => {
        const el = document.getElementById(`presetInfo${i}`);
        if (el) el.textContent = fmtPreset(p);
      });
    }
  }).catch(()=>{});
}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
)HTMLPAGE";
