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
const int LA0 = 65;
const int RA0 = 105;

const int RI  = 70;
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


// ================================================================
// FOOT SERVO HELPERS
// ================================================================
bool feetAttached = false;

void feetAttach()
{
  if (!feetAttached) {
    leftFoot.attach(SERVO_LEFT_FOOT_PIN,  544, 2400);
    rightFoot.attach(SERVO_RIGHT_FOOT_PIN, 544, 2400);
    feetAttached = true;
  }
}

void feetStop()
{
  if (feetAttached) {
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
    delay(30);
    leftFoot.detach();
    rightFoot.detach();
    feetAttached = false;
  }
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
  feetAttach();

  const int Interval     = 250;   // time to tilt one leg
  const int Overlap      = 150;   // how early the next leg starts moving
  const int Drive        = 200;   // foot drive pulse duration
  const int ReturnTime   = 300;   // time to ease legs back to neutral (prevents jump)
  const int FullCycle    = (Interval*2) + Drive + ReturnTime +
                           (Interval*2) + Drive + ReturnTime;

  if (millis() > walkCycleStart + FullCycle) walkCycleStart = millis();
  long e = millis() - walkCycleStart;

  // ── Half 1: right foot drives ─────────────────────────────────
  // P1: left leg swings to LATR
  // P1→P2: right leg sweeps from neutral to RATR (overlapping)
  // P2→P3: right foot drives forward
  // P3→P4: both legs ease back to neutral smoothly

  const long P1 = Interval;
  const long P2 = P1 + Interval;
  const long P3 = P2 + Drive;
  const long P4 = P3 + ReturnTime;

  // ── Half 2: left foot drives ──────────────────────────────────
  const long P5 = P4 + Interval;
  const long P6 = P5 + Interval;
  const long P7 = P6 + Drive;
  // P7 → FullCycle: ease back to neutral

  if (e <= P1) {
    leftLeg.write(LATR);
    rightLeg.write(RA0);
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
  }
  if (e >= P1 - Overlap && e <= P2) {
    leftLeg.write(LATR);
    rightLeg.write(map(e, P1 - Overlap, P2, RA0, RATR));
  }
  if (e > P2 && e <= P3) {
    rightFoot.write(FOOT_STOP_R - RFFWRS);
  }
  // Ease both legs back to neutral over ReturnTime — no snap
  if (e > P3 && e <= P4) {
    rightFoot.write(FOOT_STOP_R);
    leftLeg.write(map(e, P3, P4, LATR, LA0));
    rightLeg.write(map(e, P3, P4, RATR, RA0));
  }

  if (e > P4 && e <= P5) {
    rightLeg.write(RATL);
    leftLeg.write(LA0);
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
  }
  if (e >= P5 - Overlap && e <= P6) {
    rightLeg.write(RATL);
    leftLeg.write(map(e, P5 - Overlap, P6, LA0, LATL));
  }
  if (e > P6 && e <= P7) {
    leftFoot.write(FOOT_STOP_L + LFFWRS);
  }
  // Ease both legs back to neutral over remaining time — no snap
  if (e > P7 && e <= FullCycle) {
    leftFoot.write(FOOT_STOP_L);
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
  feetAttach();

  const int Interval   = 250;
  const int Overlap    = 150;
  const int Drive      = 200;
  const int ReturnTime = 300;
  const int FullCycle  = (Interval*2) + Drive + ReturnTime +
                         (Interval*2) + Drive + ReturnTime;

  if (millis() > walkCycleStart + FullCycle) walkCycleStart = millis();
  long e = millis() - walkCycleStart;

  const long P1 = Interval;
  const long P2 = P1 + Interval;
  const long P3 = P2 + Drive;
  const long P4 = P3 + ReturnTime;
  const long P5 = P4 + Interval;
  const long P6 = P5 + Interval;
  const long P7 = P6 + Drive;

  // Half 1: right foot drives backward
  if (e <= P1) {
    leftLeg.write(LATR);
    rightLeg.write(RA0);
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
  }
  if (e >= P1 - Overlap && e <= P2) {
    leftLeg.write(LATR);
    rightLeg.write(map(e, P1 - Overlap, P2, RA0, RATR));
  }
  if (e > P2 && e <= P3) {
    rightFoot.write(FOOT_STOP_R + RFBWRS);   // reversed: + instead of -
  }
  if (e > P3 && e <= P4) {
    rightFoot.write(FOOT_STOP_R);
    leftLeg.write(map(e, P3, P4, LATR, LA0));
    rightLeg.write(map(e, P3, P4, RATR, RA0));
  }

  // Half 2: left foot drives backward
  if (e > P4 && e <= P5) {
    rightLeg.write(RATL);
    leftLeg.write(LA0);
    leftFoot.write(FOOT_STOP_L);
    rightFoot.write(FOOT_STOP_R);
  }
  if (e >= P5 - Overlap && e <= P6) {
    rightLeg.write(RATL);
    leftLeg.write(map(e, P5 - Overlap, P6, LA0, LATL));
  }
  if (e > P6 && e <= P7) {
    leftFoot.write(FOOT_STOP_L - LFBWRS);    // reversed: - instead of +
  }
  if (e > P7 && e <= FullCycle) {
    leftFoot.write(FOOT_STOP_L);
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
    leftLeg.write(LA1);
    rightLeg.write(RA1);
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
      leftLeg.write(LA1); rightLeg.write(RA1);
      if (elapsed >= EXPLORE_ROLL_ENTRY_MS) {
        seqPhase = 2; seqPhaseStart = millis();
        Serial.println(">>> Rolling forward");
      }
    }
    else if (seqPhase == 2)
    {
      leftLeg.write(LA1); rightLeg.write(RA1);
      feetAttach();
      leftFoot.write(FOOT_STOP_L + ROLL_FWD_SPEED);
      rightFoot.write(FOOT_STOP_R - ROLL_FWD_SPEED);

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
        feetStop(); delay(300); standNeutral();
        seqActive = false; seqPhase = 0; clearingAfterTurn = false;
        Serial.println(">>> EXPLORE stopped");
        return;
      }
    }
    else if (seqPhase == 3)
    {
      leftLeg.write(LA1); rightLeg.write(RA1);
      feetAttach();
      leftFoot.write(ROLL_TURN_ANGLE);
      rightFoot.write(ROLL_TURN_ANGLE);

      if (elapsed >= EXPLORE_TURN_MS) {
        feetStop(); delay(150);
        clearingAfterTurn = true; clearStart = millis();
        seqPhase = 2; seqPhaseStart = millis();
        Serial.println(">>> 180 done — rolling again");
      }

      if (buttonEdge) {
        feetStop(); delay(300); standNeutral();
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
        leftLeg.write(LA0 + WI);
        rightLeg.write(RA0 + WSI);
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
        leftLeg.write(LA0);
        rightLeg.write(RA0);
        if (elapsed >= 700) {
          rxActionActive=false; rxPhase=0; rxWaveCount=0;
          Serial.println(">>> WAVE done");
        }
      }
    }

    else if (rxActionType == 2)  // CIRCLE
    {
      if (rxPhase == 1) {
        rightLeg.write(RA0 + WSI); delay(40); leftLeg.write(LA0 + WI);
        feetAttach();
        leftFoot.write(FOOT_STOP_L + CIRCLE_FOOT_SPEED);
        rightFoot.write(FOOT_STOP_R);
        if (elapsed >= CIRCLE_DURATION_MS) { rxPhase=2; rxPhaseStart=millis(); Serial.println(">>> CIRCLE returning"); }
      }
      else if (rxPhase == 2) {
        standNeutral();
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
    leftLeg.write(LA1); rightLeg.write(RA1);
    feetStop();
    Serial.println(">>> ROLL mode");
  }
  if (btnYPressed)
  {
    ModeCounter = 0;
    standNeutral();
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
    int LWD = map(J_x, 100, -100,  45,  0);
    int RWD = map(J_x, 100, -100,   0,-45);
    leftFoot.write(LWS + LWD);
    rightFoot.write(RWS + RWD);
  }
}
