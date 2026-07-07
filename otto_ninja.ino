// ================================================================
// OTTO NINJA — DUAL MODE  v3  (Web App Edition)
//
// Control via browser — no app needed.
// Phone connects to "OTTO NINJA" WiFi, opens 192.168.4.1
//
// Joystick        → Walk / Roll movement
// Button WAVE (A) → WAVE: stand on left leg, wave right leg 2x
// Button CIRC (B) → CIRCLE: stand on left leg, left foot spins
// Button WALK (Y) → Switch to Walk mode
// Button ROLL (X) → Switch to Roll mode
// Push Button D7  → EXPLORE mode (autonomous obstacle avoidance)
// ================================================================


// ================================================================
// LIBRARIES
// ================================================================
#include <ESP8266WiFi.h>
#include <ESPAsyncWebServer.h>   // install: ESPAsyncWebServer by lacamera
#include <ESPAsyncTCP.h>         // install: ESPAsyncTCP by dvarrel
#include <ArduinoJson.h>         // install: ArduinoJson by Benoit Blanchon
#include <Servo.h>

#define WIFI_SSID     "OTTO NINJA"
#define WIFI_PASSWORD "12345678"

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


// ================================================================
// CONTROL STATE  (replaces RemoteXY struct)
// Updated by WebSocket messages from phone browser
// ================================================================
int8_t  J_x      = 0;
int8_t  J_y      = 0;
uint8_t button_A = 0;   // WAVE
uint8_t button_B = 0;   // CIRCLE
uint8_t button_X = 0;   // Switch to Roll mode
uint8_t button_Y = 0;   // Switch to Walk mode


// ================================================================
// PIN DEFINITIONS
// ================================================================
#define SERVO_LEFT_FOOT_PIN   5    // D1
#define SERVO_LEFT_LEG_PIN    4    // D2
#define SERVO_RIGHT_FOOT_PIN  0    // D3
#define SERVO_RIGHT_LEG_PIN   2    // D4

#define PUSH_BUTTON_PIN      13    // D7 — HIGH = pressed
#define TRIG_PIN             14    // D5 — ultrasonic trigger
#define ECHO_PIN             12    // D6 — ultrasonic echo


// ================================================================
// SERVO OBJECTS
// ================================================================
Servo leftFoot;
Servo leftLeg;
Servo rightFoot;
Servo rightLeg;


// ================================================================
// CALIBRATION
// ================================================================
const int LA0 = 87;
const int RA0 = 100;

const int RI  = 85;
const int WI  = 35;
const int WSI = 60;

const int FOOT_STOP_L = 90;
const int FOOT_STOP_R = 90;

int LA1, RA1, LATL, RATL, LATR, RATR;

const int LFFWRS = 20;
const int RFFWRS = 20;
const int LFBWRS = 20;
const int RFBWRS = 20;


// ================================================================
// FOOT SERVO SPEED LIMIT
// Continuous-rotation foot servos draw more current and run hotter
// the further their write value sits from the stop point (90).
// FOOT_SPEED_LIMIT caps that deviation so no command — from any
// mode (walk, roll, explore, circle) — can push a foot servo past
// a safe sustained speed, even during long (10+ min) run sessions.
// Lower this further if you still want things gentler; it scales
// every foot-driven movement in the firmware (walk drive, rolling,
// the post-obstacle turn, circle spin, roll-mode joystick).
// ================================================================
const int FOOT_SPEED_LIMIT = 25;
const int FOOT_WRITE_MIN   = FOOT_STOP_L - FOOT_SPEED_LIMIT;
const int FOOT_WRITE_MAX   = FOOT_STOP_L + FOOT_SPEED_LIMIT;

int safeFootWrite(int v)
{
  return constrain(v, FOOT_WRITE_MIN, FOOT_WRITE_MAX);
}


// ================================================================
// LEG SERVO EASING
// Leg (position) servos used to jump straight to their target angle
// — e.g. standing-to-rolling and rolling-to-standing snapped in one
// step. easeLegsTo() ramps both leg servos to a target over a fixed
// duration instead, so transitions are gradual rather than a full
// speed slam on the gears. Used for one-shot transitions (mode
// switches, explore start/stop). Do NOT call this from a path that
// runs every loop() iteration (e.g. idle/hold states) — it blocks.
// ================================================================
const int LEG_EASE_MS = 200;

void easeLegsTo(int targetL, int targetR)
{
  int startL = leftLeg.read();
  int startR = rightLeg.read();
  const int steps = 15;
  for (int i = 1; i <= steps; i++) {
    leftLeg.write(map(i, 0, steps, startL, targetL));
    rightLeg.write(map(i, 0, steps, startR, targetR));
    delay(LEG_EASE_MS / steps);
  }
}


// ================================================================
// SEQUENCE TIMING
// ================================================================
#define OBSTACLE_DISTANCE_CM   10
#define EXPLORE_ROLL_ENTRY_MS  600
#define EXPLORE_TURN_MS        550
#define ROLL_FWD_SPEED         40
#define ROLL_TURN_ANGLE        135
#define EXPLORE_CLEAR_MS       800

#define WAVE_TILT_MS           500
#define WAVE_UP_MS             500
#define WAVE_DOWN_MS           500
#define WAVE_REPEATS             2
#define WAVE_LEG_LIFT           15

#define CIRCLE_DURATION_MS    4000
#define CIRCLE_FOOT_SPEED       20

// Walk gait cadence — how fast steps happen in Walk mode.
// Raise these further to slow walking down even more; lower them
// to speed it back up. All four scale together in walkForward()
// and walkBackward() (was 250/150/200/300 — felt fast/rushed).
#define WALK_TILT_MS    500   // time to tilt one leg into swing position
#define WALK_OVERLAP_MS 10   // how early the next leg starts moving
// Foot drive pulse duration — split per leg since the left and right
// foot drives sometimes need different timing (e.g. servo mismatch,
// asymmetric torque/speed).
#define WALK_DRIVE_MS_L 300   // left foot drive pulse duration
#define WALK_DRIVE_MS_R 250   // right foot drive pulse duration
#define WALK_RETURN_MS  500   // time to ease legs back to neutral

// The foot-drive push used to jump straight from stop to full speed
// the instant the drive phase started — that instant full-torque
// step is what causes the harsh "clunk" sound on the first push.
// WALK_DRIVE_RAMP_MS ramps the foot speed smoothly from stop up to
// full drive speed over this many ms at the start of each drive
// phase (same soft-start feel as the transform/roll speed), then
// holds full speed for the rest of the push. Keep it well under
// WALK_DRIVE_MS_L / WALK_DRIVE_MS_R. Raise it for a gentler/slower
// ramp, lower it for a snappier push.
#define WALK_DRIVE_RAMP_MS 120


// ================================================================
// FOOT SERVO HELPERS
// ================================================================
// Per-foot attach state (instead of one shared flag) so each foot can be
// attached/detached independently — this lets the walking gait attach a
// foot only for its own drive window (WALK_DRIVE_MS_L / WALK_DRIVE_MS_R)
// instead of leaving both feet powered for the whole gait cycle.
bool rightFootAttached = false;
bool leftFootAttached  = false;

void rightFootOn()
{
  if (!rightFootAttached) {
    rightFoot.attach(SERVO_RIGHT_FOOT_PIN, 544, 2400);
    rightFootAttached = true;
  }
}

void rightFootOff()
{
  if (rightFootAttached) {
    rightFoot.write(FOOT_STOP_R);
    rightFoot.detach();
    rightFootAttached = false;
  }
}

void leftFootOn()
{
  if (!leftFootAttached) {
    leftFoot.attach(SERVO_LEFT_FOOT_PIN, 544, 2400);
    leftFootAttached = true;
  }
}

void leftFootOff()
{
  if (leftFootAttached) {
    leftFoot.write(FOOT_STOP_L);
    leftFoot.detach();
    leftFootAttached = false;
  }
}

// Attach / stop-and-detach BOTH feet together — used by roll mode and by
// the obstacle/turn sequences that still want the old "both feet at once"
// behavior. Built on top of the per-foot helpers above so the attach state
// always stays consistent no matter which path (walk gait or these) last
// touched a foot.
void feetAttach()
{
  rightFootOn();
  leftFootOn();
}

void feetStop()
{
  if (rightFootAttached) rightFoot.write(FOOT_STOP_R);
  if (leftFootAttached)  leftFoot.write(FOOT_STOP_L);
  if (rightFootAttached || leftFootAttached) delay(30);
  rightFootOff();
  leftFootOff();
}


// ================================================================
// STATE VARIABLES
// ================================================================
int  ModeCounter = 0;
unsigned long walkCycleStart = 0;

bool          seqActive         = false;
int           seqPhase          = 0;
unsigned long seqPhaseStart     = 0;
bool          clearingAfterTurn = false;
unsigned long clearStart        = 0;

bool prevButtonState = false;
bool buttonEdge      = false;

bool          rxActionActive = false;
int           rxActionType   = 0;
int           rxPhase        = 0;
unsigned long rxPhaseStart   = 0;
int           rxWaveCount    = 0;

// Edge detection for ALL buttons
uint8_t prevBtnA = 0;
uint8_t prevBtnB = 0;
uint8_t prevBtnX = 0;
uint8_t prevBtnY = 0;


// ================================================================
// HTML CONTROL PAGE
// ================================================================
const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no">
<title>OTTO NINJA</title>
<style>
  * { margin:0; padding:0; box-sizing:border-box; -webkit-tap-highlight-color:transparent; user-select:none; }
  body {
    background:#1a1a1a;
    display:flex;
    flex-direction:column;
    align-items:center;
    height:100vh;
    overflow:hidden;
    font-family:Arial,sans-serif;
  }
  #header {
    width:100%;
    background:#111;
    color:#fff;
    text-align:center;
    padding:10px 0 6px;
    font-size:18px;
    font-weight:bold;
    letter-spacing:3px;
    border-bottom:2px solid #c0392b;
  }
  #status {
    font-size:11px;
    color:#888;
    margin-top:2px;
    letter-spacing:1px;
  }
  #controls {
    display:flex;
    flex:1;
    width:100%;
    align-items:center;
    justify-content:space-around;
    padding:16px 10px;
  }

  /* JOYSTICK */
  #joy-area {
    position:relative;
    width:180px;
    height:180px;
    flex-shrink:0;
    touch-action:none;
  }
  #joy-base {
    position:absolute;
    inset:0;
    border-radius:50%;
    border:3px solid #c0392b;
    background:rgba(192,57,43,0.08);
  }
  #joy-base::before, #joy-base::after {
    content:'';
    position:absolute;
    background:rgba(255,255,255,0.08);
  }
  #joy-base::before { left:50%; top:15%; width:1px; height:70%; transform:translateX(-50%); }
  #joy-base::after  { top:50%; left:15%; height:1px; width:70%; transform:translateY(-50%); }
  #joy-knob {
    position:absolute;
    width:64px;
    height:64px;
    border-radius:50%;
    background:radial-gradient(circle at 38% 38%, #e74c3c, #8e1a0e);
    box-shadow:0 4px 15px rgba(192,57,43,0.6);
    top:50%; left:50%;
    transform:translate(-50%,-50%);
  }

  /* BUTTONS */
  #btn-grid {
    display:grid;
    grid-template-columns:1fr 1fr;
    gap:12px;
    flex-shrink:0;
  }
  .ctrl-btn {
    width:90px;
    height:78px;
    border-radius:8px;
    border:none;
    background:#c0392b;
    display:flex;
    align-items:center;
    justify-content:center;
    cursor:pointer;
    box-shadow:0 4px 10px rgba(0,0,0,0.5);
    touch-action:none;
    font-size:14px;
    font-weight:bold;
    color:#fff;
    letter-spacing:1px;
    transition:background 0.08s, transform 0.08s;
  }
  .ctrl-btn.pressed {
    background:#8e1a0e;
    transform:scale(0.93);
  }

  /* MODE BAR */
  #mode-bar {
    width:100%;
    display:flex;
    border-top:2px solid #333;
  }
  .mode-btn {
    flex:1;
    padding:13px 0;
    border:none;
    font-size:13px;
    font-weight:bold;
    letter-spacing:2px;
    cursor:pointer;
    background:#222;
    color:#666;
    touch-action:manipulation;
  }
  .mode-btn.active { background:#c0392b; color:#fff; }
  .mode-btn:first-child { border-right:1px solid #333; }
</style>
</head>
<body>

<div id="header">
  OTTO NINJA
  <div id="status">CONNECTING...</div>
</div>

<div id="controls">
  <!-- JOYSTICK -->
  <div id="joy-area">
    <div id="joy-base"></div>
    <div id="joy-knob"></div>
  </div>

  <!-- ACTION BUTTONS — no emojis, plain text only -->
  <div id="btn-grid">
    <button class="ctrl-btn" id="btnA">WAVE</button>
    <button class="ctrl-btn" id="btnB">CIRCLE</button>
    <button class="ctrl-btn" id="btnY">STAND</button>
    <button class="ctrl-btn" id="btnX">TRANSFORM</button>
  </div>
</div>

<div id="mode-bar">
  <button class="mode-btn active" id="modeWalk">WALK MODE</button>
  <button class="mode-btn"        id="modeRoll">ROLL MODE</button>
</div>

<script>
// ── WebSocket ─────────────────────────────────────────────────
const statusEl = document.getElementById('status');
let ws;
let state = { jx:0, jy:0, A:0, B:0, X:0, Y:0 };
let sendInterval;

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen = () => {
    statusEl.textContent = 'CONNECTED';
    statusEl.style.color = '#2ecc71';
    sendInterval = setInterval(sendState, 50);
  };
  ws.onclose = () => {
    statusEl.textContent = 'DISCONNECTED - retrying...';
    statusEl.style.color = '#e74c3c';
    clearInterval(sendInterval);
    setTimeout(connect, 1500);
  };
  ws.onerror = () => ws.close();
}

function sendState() {
  if (ws && ws.readyState === 1)
    ws.send(JSON.stringify(state));
}

connect();

// ── JOYSTICK ──────────────────────────────────────────────────
const joyArea = document.getElementById('joy-area');
const joyKnob = document.getElementById('joy-knob');
const RADIUS  = 75;
let joyId = null;

function joyMove(cx, cy) {
  const r  = joyArea.getBoundingClientRect();
  const ox = r.left + r.width  / 2;
  const oy = r.top  + r.height / 2;
  let dx = cx - ox;
  let dy = cy - oy;
  const d = Math.sqrt(dx*dx + dy*dy);
  if (d > RADIUS) { dx = dx/d*RADIUS; dy = dy/d*RADIUS; }
  joyKnob.style.transform = 'translate(calc(-50% + ' + dx + 'px), calc(-50% + ' + dy + 'px))';
  state.jx = Math.round( dx / RADIUS * 100);
  state.jy = Math.round(-dy / RADIUS * 100);
}

function joyReset() {
  joyKnob.style.transition = 'transform 0.12s';
  joyKnob.style.transform  = 'translate(-50%,-50%)';
  setTimeout(function(){ joyKnob.style.transition = ''; }, 120);
  state.jx = 0; state.jy = 0;
  joyId = null;
}

joyArea.addEventListener('touchstart', function(e) {
  e.preventDefault();
  joyId = e.changedTouches[0].identifier;
  joyMove(e.changedTouches[0].clientX, e.changedTouches[0].clientY);
}, { passive:false });

joyArea.addEventListener('touchmove', function(e) {
  e.preventDefault();
  for (var i=0; i<e.changedTouches.length; i++)
    if (e.changedTouches[i].identifier === joyId)
      joyMove(e.changedTouches[i].clientX, e.changedTouches[i].clientY);
}, { passive:false });

joyArea.addEventListener('touchend',    function(e){ e.preventDefault(); joyReset(); }, { passive:false });
joyArea.addEventListener('touchcancel', function(e){ e.preventDefault(); joyReset(); }, { passive:false });

var mouseDragging = false;
joyArea.addEventListener('mousedown', function(e){ mouseDragging=true; joyMove(e.clientX,e.clientY); });
window.addEventListener('mousemove',  function(e){ if(mouseDragging) joyMove(e.clientX,e.clientY); });
window.addEventListener('mouseup',    function(){ if(mouseDragging){ mouseDragging=false; joyReset(); } });

// ── ACTION BUTTONS ────────────────────────────────────────────
// On tap: immediately set key=1, send it right away (don't wait for 50ms interval),
// then hold high for 350ms so the firmware edge-detect has plenty of time to catch it.
// This means a quick single tap always works — no need to hold the button.

var btnTimers = {};

function setupBtn(id, key, onPress) {
  var el = document.getElementById('btn' + id);

  function press() {
    // Cancel any pending release timer
    if (btnTimers[key]) { clearTimeout(btnTimers[key]); btnTimers[key] = null; }
    state[key] = 1;
    // Send immediately — don't wait for the 50ms interval
    if (ws && ws.readyState === 1) ws.send(JSON.stringify(state));
    el.classList.add('pressed');
    if (onPress) onPress();
  }

  function release() {
    // Hold high for 350ms after finger lifts to guarantee firmware catches the edge
    btnTimers[key] = setTimeout(function(){
      state[key] = 0;
      btnTimers[key] = null;
    }, 350);
    el.classList.remove('pressed');
  }

  el.addEventListener('touchstart',  function(e){ e.preventDefault(); press(); },   { passive:false });
  el.addEventListener('touchend',    function(e){ e.preventDefault(); release(); }, { passive:false });
  el.addEventListener('touchcancel', function(e){ e.preventDefault(); release(); }, { passive:false });
  el.addEventListener('mousedown',   function(){ press(); });
  el.addEventListener('mouseup',     function(){ release(); });
  el.addEventListener('mouseleave',  function(){ release(); });
}

setupBtn('A', 'A');  // WAVE
setupBtn('B', 'B');  // CIRCLE

setupBtn('Y', 'Y', function(){  // STAND (Walk mode)
  document.getElementById('modeWalk').classList.add('active');
  document.getElementById('modeRoll').classList.remove('active');
});

setupBtn('X', 'X', function(){  // TRANSFORM (Roll mode)
  document.getElementById('modeRoll').classList.add('active');
  document.getElementById('modeWalk').classList.remove('active');
});

// Prevent page scroll on touch
document.addEventListener('touchmove', function(e){ e.preventDefault(); }, { passive:false });
</script>
</body>
</html>
)rawliteral";


// ================================================================
// WEBSOCKET EVENT HANDLER
// ================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  if (type == WS_EVT_DATA)
  {
    StaticJsonDocument<128> doc;
    DeserializationError err = deserializeJson(doc, data, len);
    if (!err)
    {
      J_x      = doc["jx"] | 0;
      J_y      = doc["jy"] | 0;
      button_A = doc["A"]  | 0;
      button_B = doc["B"]  | 0;
      button_X = doc["X"]  | 0;
      button_Y = doc["Y"]  | 0;
    }
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    J_x = 0; J_y = 0;
    button_A = button_B = button_X = button_Y = 0;
    Serial.println(">>> Phone disconnected — all controls zeroed");
  }
}


// ================================================================
// HELPER — ULTRASONIC DISTANCE
// ================================================================
long readDistanceCM()
{
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return 999;
  return dur * 0.034 / 2;
}


// ================================================================
// HELPERS
// ================================================================
void standNeutral()
{
  leftLeg.write(LA0);
  rightLeg.write(RA0);
  feetStop();
}

void rollNeutral()
{
  leftLeg.write(LA1);
  rightLeg.write(RA1);
  feetStop();
}


// ================================================================
// WALK FORWARD GAIT
// ================================================================
// Each half-cycle:
//   Swing phase  — leg tilts to swing position (with overlap for smooth handoff)
//   Drive phase  — foot drives forward
//   Return phase — both legs slide gently back to neutral (no snap/jump)
// ================================================================
void walkForward()
{
  const int Interval     = WALK_TILT_MS;     // time to tilt one leg
  const int Overlap      = WALK_OVERLAP_MS;  // how early the next leg starts moving
  const int DriveR       = WALK_DRIVE_MS_R;  // right foot drive pulse duration
  const int DriveL       = WALK_DRIVE_MS_L;  // left foot drive pulse duration
  const int ReturnTime   = WALK_RETURN_MS;   // time to ease legs back to neutral (prevents jump)
  const int FullCycle    = (Interval*2) + DriveR + ReturnTime +
                           (Interval*2) + DriveL + ReturnTime;

  if (millis() > walkCycleStart + FullCycle) walkCycleStart = millis();
  long e = millis() - walkCycleStart;

  // ── Half 1: right foot drives ─────────────────────────────────
  // P1: left leg swings to LATR
  // P1→P2: right leg sweeps from neutral to RATR (overlapping)
  // P2→P3: right foot drives forward
  // P3→P4: both legs ease back to neutral smoothly

  const long P1 = Interval;
  const long P2 = P1 + Interval;
  const long P3 = P2 + DriveR;
  const long P4 = P3 + ReturnTime;

  // ── Half 2: left foot drives ──────────────────────────────────
  const long P5 = P4 + Interval;
  const long P6 = P5 + Interval;
  const long P7 = P6 + DriveL;
  // P7 → FullCycle: ease back to neutral

  // Ease into the tilt over Interval — was an instant leftLeg.write(LATR)
  // jump, which snapped the leg servo to full deflection the instant the
  // phase started (same kind of hard-torque snap that easeLegsTo() avoids
  // during transform). Ramping it here removes that jolt before the foot
  // ever starts driving.
  if (e <= P1) {
    leftLeg.write(map(e, 0, P1, LA0, LATR));
    rightLeg.write(RA0);
  }
  if (e >= P1 - Overlap && e <= P2) {
    leftLeg.write(LATR);
    rightLeg.write(map(e, P1 - Overlap, P2, RA0, RATR));
  }
  // Right foot is attached exactly when its drive window opens (P2) and
  // detached the instant it closes (P3) — it no longer sits attached
  // (and potentially creeping) through the swing/return phases too.
  if (e > P2 && e <= P3) {
    rightFootOn();
    long rampEnd = min((long)(P2 + WALK_DRIVE_RAMP_MS), P3);
    if (e <= rampEnd) {
      rightFoot.write(safeFootWrite(map(e, P2, rampEnd, FOOT_STOP_R, FOOT_STOP_R - RFFWRS)));
    } else {
      rightFoot.write(safeFootWrite(FOOT_STOP_R - RFFWRS));
    }
  }
  // Ease both legs back to neutral over ReturnTime — no snap
  if (e > P3 && e <= P4) {
    rightFootOff();
    leftLeg.write(map(e, P3, P4, LATR, LA0));
    rightLeg.write(map(e, P3, P4, RATR, RA0));
  }

  // Same fix mirrored for the second half — rightLeg was snapping to
  // RATL instantly instead of easing into the tilt.
  if (e > P4 && e <= P5) {
    rightLeg.write(map(e, P4, P5, RA0, RATL));
    leftLeg.write(LA0);
  }
  if (e >= P5 - Overlap && e <= P6) {
    rightLeg.write(RATL);
    leftLeg.write(map(e, P5 - Overlap, P6, LA0, LATL));
  }
  // Left foot: same idea — attach only for its own DriveL window.
  if (e > P6 && e <= P7) {
    leftFootOn();
    long rampEnd = min((long)(P6 + WALK_DRIVE_RAMP_MS), P7);
    if (e <= rampEnd) {
      leftFoot.write(safeFootWrite(map(e, P6, rampEnd, FOOT_STOP_L, FOOT_STOP_L + LFFWRS)));
    } else {
      leftFoot.write(safeFootWrite(FOOT_STOP_L + LFFWRS));
    }
  }
  // Ease both legs back to neutral over remaining time — no snap
  if (e > P7 && e <= FullCycle) {
    leftFootOff();
    leftLeg.write(map(e, P7, FullCycle, LATL, LA0));
    rightLeg.write(map(e, P7, FullCycle, RATL, RA0));
  }
}


// ================================================================
// WALK BACKWARD GAIT  — mirror of walkForward
// ================================================================
// Foot drives BACKWARD (reversed spin direction).
// Leg swing sequence is identical to forward — same smooth overlap
// and same eased return to neutral so no jumping.
// ================================================================
void walkBackward()
{
  const int Interval   = WALK_TILT_MS;
  const int Overlap    = WALK_OVERLAP_MS;
  const int DriveR     = WALK_DRIVE_MS_R;
  const int DriveL     = WALK_DRIVE_MS_L;
  const int ReturnTime = WALK_RETURN_MS;
  const int FullCycle  = (Interval*2) + DriveR + ReturnTime +
                         (Interval*2) + DriveL + ReturnTime;

  if (millis() > walkCycleStart + FullCycle) walkCycleStart = millis();
  long e = millis() - walkCycleStart;

  const long P1 = Interval;
  const long P2 = P1 + Interval;
  const long P3 = P2 + DriveR;
  const long P4 = P3 + ReturnTime;
  const long P5 = P4 + Interval;
  const long P6 = P5 + Interval;
  const long P7 = P6 + DriveL;

  // Half 1: right foot drives backward
  if (e <= P1) {
    leftLeg.write(map(e, 0, P1, LA0, LATR));
    rightLeg.write(RA0);
  }
  if (e >= P1 - Overlap && e <= P2) {
    leftLeg.write(LATR);
    rightLeg.write(map(e, P1 - Overlap, P2, RA0, RATR));
  }
  if (e > P2 && e <= P3) {
    rightFootOn();
    long rampEnd = min((long)(P2 + WALK_DRIVE_RAMP_MS), P3);
    if (e <= rampEnd) {
      rightFoot.write(safeFootWrite(map(e, P2, rampEnd, FOOT_STOP_R, FOOT_STOP_R + RFBWRS)));  // reversed: + instead of -
    } else {
      rightFoot.write(safeFootWrite(FOOT_STOP_R + RFBWRS));   // reversed: + instead of -
    }
  }
  if (e > P3 && e <= P4) {
    rightFootOff();
    leftLeg.write(map(e, P3, P4, LATR, LA0));
    rightLeg.write(map(e, P3, P4, RATR, RA0));
  }

  // Half 2: left foot drives backward
  if (e > P4 && e <= P5) {
    rightLeg.write(map(e, P4, P5, RA0, RATL));
    leftLeg.write(LA0);
  }
  if (e >= P5 - Overlap && e <= P6) {
    rightLeg.write(RATL);
    leftLeg.write(map(e, P5 - Overlap, P6, LA0, LATL));
  }
  if (e > P6 && e <= P7) {
    leftFootOn();
    long rampEnd = min((long)(P6 + WALK_DRIVE_RAMP_MS), P7);
    if (e <= rampEnd) {
      leftFoot.write(safeFootWrite(map(e, P6, rampEnd, FOOT_STOP_L, FOOT_STOP_L - LFBWRS)));   // reversed: - instead of +
    } else {
      leftFoot.write(safeFootWrite(FOOT_STOP_L - LFBWRS));    // reversed: - instead of +
    }
  }
  if (e > P7 && e <= FullCycle) {
    leftFootOff();
    leftLeg.write(map(e, P7, FullCycle, LATL, LA0));
    rightLeg.write(map(e, P7, FullCycle, RATL, RA0));
  }
}


// ================================================================
// SETUP
// ================================================================
void setup()
{
  leftLeg.attach(SERVO_LEFT_LEG_PIN,   544, 2400);
  rightLeg.attach(SERVO_RIGHT_LEG_PIN, 544, 2400);

  LA1  = LA0 + RI;
  RA1  = RA0 - RI;
  LATL = LA0 + WI;
  RATL = RA0 + WSI;
  LATR = LA0 - WSI;
  RATR = RA0 - WI - 8;

  pinMode(PUSH_BUTTON_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Serial.begin(250000);

  WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", CONTROL_PAGE);
  });

  server.begin();
  Serial.println("Web server started — open 192.168.4.1 in phone browser");

  leftLeg.write(LA0);
  rightLeg.write(RA0);
  delay(500);

  Serial.println("OTTO NINJA v3 (Web) ready.");
}


// ================================================================
// MAIN LOOP
// ================================================================
void loop()
{
  ws.cleanupClients();

  // Push button edge detection
  bool currBtn    = (digitalRead(PUSH_BUTTON_PIN) == HIGH);
  buttonEdge      = (currBtn && !prevButtonState);
  prevButtonState = currBtn;

  // Web button edge detection — ALL four buttons
  bool btnAPressed = (button_A && !prevBtnA);
  bool btnBPressed = (button_B && !prevBtnB);
  bool btnXPressed = (button_X && !prevBtnX);
  bool btnYPressed = (button_Y && !prevBtnY);
  prevBtnA = button_A;
  prevBtnB = button_B;
  prevBtnX = button_X;
  prevBtnY = button_Y;


  // ================================================================
  // PUSH BUTTON — START EXPLORE
  // ================================================================
  if (buttonEdge && !seqActive && !rxActionActive)
  {
    seqActive     = true;
    seqPhase      = 1;
    seqPhaseStart = millis();
    Serial.println(">>> EXPLORE started");
    feetStop();
  }


  // ================================================================
  // EXPLORE STATE MACHINE
  // ================================================================
  if (seqActive)
  {
    unsigned long elapsed = millis() - seqPhaseStart;

    if (seqPhase == 1)
    {
      unsigned long t = constrain(elapsed, 0, (unsigned long)EXPLORE_ROLL_ENTRY_MS);
      leftLeg.write(map(t, 0, EXPLORE_ROLL_ENTRY_MS, LA0, LA1));
      rightLeg.write(map(t, 0, EXPLORE_ROLL_ENTRY_MS, RA0, RA1));
      if (elapsed >= EXPLORE_ROLL_ENTRY_MS) {
        seqPhase = 2; seqPhaseStart = millis();
        Serial.println(">>> Rolling forward");
      }
    }
    else if (seqPhase == 2)
    {
      leftLeg.write(LA1); rightLeg.write(RA1);
      feetAttach();
      leftFoot.write(safeFootWrite(FOOT_STOP_L + ROLL_FWD_SPEED));
      rightFoot.write(safeFootWrite(FOOT_STOP_R - ROLL_FWD_SPEED));

      if (clearingAfterTurn && (millis() - clearStart >= EXPLORE_CLEAR_MS))
        clearingAfterTurn = false;

      if (!clearingAfterTurn) {
        long dist = readDistanceCM();
        if (dist > 0 && dist < OBSTACLE_DISTANCE_CM) {
          Serial.print(">>> Obstacle at "); Serial.print(dist); Serial.println(" cm");
          feetStop(); delay(150);
          seqPhase = 3; seqPhaseStart = millis();
        }
      }

      if (buttonEdge) {
        feetStop(); easeLegsTo(LA0, RA0);
        seqActive = false; seqPhase = 0; clearingAfterTurn = false;
        Serial.println(">>> EXPLORE stopped");
        return;
      }
    }
    else if (seqPhase == 3)
    {
      leftLeg.write(LA1); rightLeg.write(RA1);
      feetAttach();
      leftFoot.write(safeFootWrite(ROLL_TURN_ANGLE));
      rightFoot.write(safeFootWrite(ROLL_TURN_ANGLE));

      if (elapsed >= EXPLORE_TURN_MS) {
        feetStop(); delay(150);
        clearingAfterTurn = true; clearStart = millis();
        seqPhase = 2; seqPhaseStart = millis();
        Serial.println(">>> 180 done — rolling again");
      }

      if (buttonEdge) {
        feetStop(); easeLegsTo(LA0, RA0);
        seqActive = false; seqPhase = 0; clearingAfterTurn = false;
        return;
      }
    }

    return;
  }


  // ================================================================
  // WAVE (A) and CIRCLE (B) — Walk mode only
  // ================================================================
  if (!rxActionActive && ModeCounter == 0)
  {
    if (btnAPressed) {
      rxActionActive = true; rxActionType = 1;
      rxPhase = 1; rxPhaseStart = millis(); rxWaveCount = 0;
      Serial.println(">>> WAVE started");
    }
    else if (btnBPressed) {
      rxActionActive = true; rxActionType = 2;
      rxPhase = 1; rxPhaseStart = millis();
      Serial.println(">>> CIRCLE started");
    }
  }

  if (rxActionActive)
  {
    unsigned long elapsed = millis() - rxPhaseStart;

    if (rxActionType == 1)  // WAVE
    {
      // Phase 1: shift weight onto left leg, right leg lifts off ground
      if (rxPhase == 1) {
        unsigned long t = constrain(elapsed, 0, (unsigned long)WAVE_TILT_MS);
        leftLeg.write(map(t, 0, WAVE_TILT_MS, LA0, LA0 + WI));
        rightLeg.write(map(t, 0, WAVE_TILT_MS, RA0, RA0 + WSI));
        feetStop();
        if (elapsed >= WAVE_TILT_MS) { rxPhase=2; rxPhaseStart=millis(); }
      }
      // Phase 2: wave UP — right leg swings to upper angle
      else if (rxPhase == 2) {
        leftLeg.write(LA0 + WI);
        rightLeg.write(90);   // upper wave position (absolute, safe 40-80 range)
        if (elapsed >= WAVE_UP_MS) { rxPhase=3; rxPhaseStart=millis(); }
      }
      // Phase 3: wave DOWN — right leg swings to lower angle
      else if (rxPhase == 3) {
        leftLeg.write(LA0 + WI);
        rightLeg.write(130);   // lower wave position (absolute, safe 40-80 range)
        if (elapsed >= WAVE_DOWN_MS) {
          rxWaveCount++;
          if (rxWaveCount < WAVE_REPEATS) { rxPhase=2; rxPhaseStart=millis(); }
          else                            { rxPhase=4; rxPhaseStart=millis(); }
        }
      }
      // Phase 4: return to neutral
      else if (rxPhase == 4) {
        feetStop();
        unsigned long t = constrain(elapsed, 0, 700UL);
        leftLeg.write(map(t, 0, 700, LA0 + WI, LA0));
        rightLeg.write(map(t, 0, 700, 130, RA0));   // 130 = last "wave down" angle
        if (elapsed >= 700) {
          rxActionActive=false; rxPhase=0; rxWaveCount=0;
          Serial.println(">>> WAVE done");
        }
      }
    }

    else if (rxActionType == 2)  // CIRCLE
    {
      if (rxPhase == 1) {
        const unsigned long CIRCLE_ENTRY_MS = 300;
        unsigned long t = constrain(elapsed, 0, CIRCLE_ENTRY_MS);
        rightLeg.write(map(t, 0, CIRCLE_ENTRY_MS, RA0, RA0 + WSI));
        leftLeg.write(map(t, 0, CIRCLE_ENTRY_MS, LA0, LA0 + WI));
        feetAttach();
        leftFoot.write(safeFootWrite(FOOT_STOP_L + CIRCLE_FOOT_SPEED));
        rightFoot.write(FOOT_STOP_R);
        if (elapsed >= CIRCLE_DURATION_MS) { rxPhase=2; rxPhaseStart=millis(); Serial.println(">>> CIRCLE returning"); }
      }
      else if (rxPhase == 2) {
        feetStop();
        unsigned long t = constrain(elapsed, 0, 600UL);
        leftLeg.write(map(t, 0, 600, LA0 + WI, LA0));
        rightLeg.write(map(t, 0, 600, RA0 + WSI, RA0));
        if (elapsed >= 600) { rxActionActive=false; rxPhase=0; Serial.println(">>> CIRCLE done"); }
      }
    }

    return;
  }


  // ================================================================
  // MODE SWITCH — edge triggered (fixed: was level triggered before)
  // ================================================================
  if (btnXPressed)
  {
    ModeCounter = 1;
    feetStop();
    easeLegsTo(LA1, RA1);
    Serial.println(">>> ROLL mode");
  }
  if (btnYPressed)
  {
    ModeCounter = 0;
    feetStop();
    easeLegsTo(LA0, RA0);
    Serial.println(">>> WALK mode");
  }


  // ================================================================
  // JOYSTICK CONTROL
  // ================================================================
  bool joystickIdle = (J_x >= -10 && J_x <= 10 && J_y >= -10 && J_y <= 10);

  if (ModeCounter == 0)  // Walk mode
  {
    if (joystickIdle) { standNeutral(); return; }

    if (J_y > 0) { walkForward(); }

    if (J_y < 0) { walkBackward(); }
  }

  if (ModeCounter == 1)  // Roll mode
  {
    if (joystickIdle) { feetStop(); return; }

    feetAttach();
    int LWS = map(J_y, 100, -100, FOOT_STOP_L+45, FOOT_STOP_L-45);
    int RWS = map(J_y, 100, -100, FOOT_STOP_R-45, FOOT_STOP_R+45);
    // Turn bias — zero when the stick is centered (J_x = 0), so it no longer
    // skews forward vs backward speed. The old map() ranges (45→0 and 0→-45)
    // gave a +22.5 / -22.5 bias even at center, which is what made forward
    // rolling faster than backward rolling.
    int turnBias = map(J_x, -100, 100, -22, 22);
    leftFoot.write(safeFootWrite(LWS + turnBias));
    rightFoot.write(safeFootWrite(RWS + turnBias));
  }
}
