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
  body { font-family: sans-serif; text-align: center; background:#111; color:#eee; margin:0; padding:16px; }
  h2 { margin: 8px 0 16px; }
  .status { background:#222; border-radius:10px; padding:10px; margin-bottom:20px; display:inline-block; min-width:220px; }
  .status div { padding:2px 0; }
  .pad { display:grid; grid-template-columns: 70px 70px 70px; grid-template-rows: 70px 70px 70px; gap:8px; justify-content:center; margin-bottom:20px; }
  button { font-size:22px; border:none; border-radius:10px; background:#2d6cdf; color:#fff; touch-action:none; user-select:none; }
  button:active { background:#1a4fb0; }
  button:disabled { opacity:0.5; }
  #home { background:#d94f2b; font-size:16px; padding:12px 24px; border-radius:10px; }
  #saveHome { background:#6b2fa0; font-size:14px; padding:10px 18px; border-radius:10px; margin-left:8px; }
  #clearHome { background:#444; font-size:13px; padding:8px 14px; border-radius:10px; margin-left:8px; }
  .b1{grid-column:2;grid-row:1;} .b2{grid-column:1;grid-row:2;} .b3{grid-column:3;grid-row:2;} .b4{grid-column:2;grid-row:3;}
  .pad button { background:#c0392b; }
  .pad button.arrived { background:#2b9e4f; }
  .presets { margin-top:20px; display:flex; gap:8px; justify-content:center; align-items:center; }
  .presets input { width:56px; font-size:18px; padding:8px; border-radius:8px; border:none; text-align:center; }
  .presets button { font-size:16px; padding:10px 16px; }
  #goAzimuthBtn { background:#2d6cdf; }
  #goAzimuthBtn.driving { background:#c0392b; }
  #goAzimuthBtn.arrived { background:#2b9e4f; }
</style>
</head>
<body>
<h2>Pan / Tilt Camera</h2>
<div class="status">
  <div>Azimut: <span id="heading">--</span> deg<span id="noHome"></span></div>
  <div>Pan: <span id="pan">--</span> deg</div>
  <div>Tilt: <span id="tilt">--</span> deg</div>
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
  fetch(`/goAzimuth?target=${t}`).catch(()=>{});
});

function poll() {
  fetch('/status').then(r => r.json()).then(s => {
    document.getElementById('heading').textContent = s.heading.toFixed(1);
    document.getElementById('noHome').textContent = s.homeSet ? '' : ' (no home saved)';
    document.getElementById('pan').textContent = s.posKnown ? s.pan.toFixed(1) : '?? (no home saved)';
    document.getElementById('tilt').textContent = s.posKnown ? s.tilt.toFixed(1) : '??';
    if (azDriving && !s.autoDrive) {
      azDriving = false;
      goAzimuthBtn.classList.remove('driving');
      goAzimuthBtn.classList.add('arrived');
    }
  }).catch(()=>{});
}
setInterval(poll, 500);
poll();
</script>
</body>
</html>
)HTMLPAGE";
