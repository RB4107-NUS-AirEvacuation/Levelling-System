// =====================================================================
//  RB4107 Airstairs Carrier – levelling, standalone (no laptop needed)
//  ESP32 WROOM-32E + GY-BNO08X (BNO085, mounted upside down on the platform)
//  + DRV8871 + actuator.
//
//  Power up -> IMU starts -> holds roll 178.8 deg (pitch -6.2 if the actuator
//  turns out to tilt pitch) continuously. No commands needed.
//  If no direction is saved in flash it nudges the actuator once to learn it.
//  The ±180 deg roll flip is handled, so 178.8 and -179.5 are 1.7 deg apart.
//
//  Laptop optional, for debugging (Serial 115200): e / r jog, x stop,
//  t back to the hardcoded target, s hold the current angle instead, h help.
//
//  Wiring: BNO085 SDA-21 SCL-22 INT-27 RST-26 (0x4B)
//          DRV8871 IN1-32 (PWM = retract) IN2-33 (PWM = extend), 12 V on PWR+/-
//  Board "ESP32 Dev Module", ESP32 core 3.x. No libraries.
// =====================================================================

#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ---------------- pins ----------------
#define I2C_SDA   21
#define I2C_SCL   22
#define IMU_ADDR  0x4B
#define IMU_INT   27
#define IMU_RST   26
#define MOT_IN1   32      // PWM here = retract (as tested)
#define MOT_IN2   33      // PWM here = extend  (as tested)

// ---------------- fixed settings ----------------
#define PWM_FREQ        20000   // 20 kHz, above hearing
#define PWM_BITS        8       // duty 0-255
#define LOOP_MS         10      // control loop period
#define IMU_PERIOD_US   10000   // game rotation vector at 100 Hz
#define IMU_TIMEOUT_MS  200     // no reading for this long = pause motor
#define TELEMETRY_MS    50
#define FILTER_ALPHA    0.12f   // roll smoothing, about 80 ms
#define SLEW_PER_LOOP   8       // max PWM change per 10 ms
#define CONFIRM_MS      150     // error must last this long before moving
#define CAL_PWM         255     // full speed for the calibration sweep (ramped up)
#define CAL_MIN_MS      2000    // always drive at least this long toward an end
#define CAL_TIMEOUT_MS  9000    // longest a calibration move may take
#define CAL_STABLE_MS   1000    // angle unchanged this long = reached the end
#define END_MS          700     // (control) driving this long with no change = end of travel
#define END_MOVE_DEG    0.2f
#define WARN_EVERY_MS   2000

// ---------------- power-up target (set on the rig with angle_test, 17 Nov) ----------------
#define TARGET_ROLL     178.8f  // roll at the level position
#define TARGET_PITCH    -6.2f   // pitch at the same position (used if the actuator tilts pitch)
#define LEARN_MS        1000    // auto-learn nudge length if no direction is saved

// ---------------- tuning (from the code; change live with commands) ----------------
float kp     = 15.0f;   // PWM per degree of error
float ki     = 2.0f;    // PWM per degree-second
float db     = 1.0f;    // start correcting above this error (deg)
float dbExit = 0.3f;    // stop correcting below this error (deg)
int   minPwm = 110;     // smallest PWM that still moves the actuator

// ---------------- learned direction (saved in flash) ----------------
struct Cal {
  int   ok       = 0;      // 1 once a jog has taught the direction
  int   dir      = 0;      // raw motor sign that RAISES the roll correction (+1/-1)
  float rollRet  = 0;      // roll at full retract (base still)
  float rollExt  = 0;      // roll at full extend
  int   travelMs = 0;      // full stroke time (from c)
  int   axis     = 0;      // 0 = roll, 1 = pitch (picked from a jog)
} cal;
Preferences prefs;
#define PREFS_NS "levelcal2"   // new name: old calibration (pre-unwrap) is ignored

// ---------------- state ----------------
void startHold();

// Shortest signed difference, so 178.8 vs -179.5 is 1.7 deg, not 358
float wrap180(float x) { x = fmodf(x + 180.0f, 360.0f); if (x < 0) x += 360.0f; return x - 180.0f; }

float pitchDeg = 0, rollDeg = 0;   // continuous (unwrapped) angles, any mounting
uint32_t lastImuMs = 0;
float filtRoll = 0; bool filtInit = false;   // smoothed angle on the chosen axis
float filtR = 0, filtP = 0;                  // smoothed roll and pitch

bool  running = false;     // true after s, until x
float refRoll = 0;         // the roll recorded with s
int   pwmNow = 0;          // controller output, signed (before cal.dir)
float integ = 0;
bool  active = false;      // outside the deadband and correcting
uint32_t outsideSince = 0;
int   endBlock = 0;        // raw direction blocked at end of travel (0 = none)
uint32_t endStart = 0; float endRoll0 = 0; int endSign = 0;
uint32_t lastLoop = 0, lastTele = 0;
int   jogDir = 0;          // +1 retracting, -1 extending, 0 not jogging
uint32_t jogUntil = 0;
int   jogPwm = 255;        // jog power
int   jogNow = 0;          // ramped jog output (same ramp as the controller)
int   lastJogRaw = 0;      // motor sign of the current jog
float jogR0 = 0, jogP0 = 0;

void warn(int id, const char *msg) {
  static uint32_t last[4] = {0, 0, 0, 0};
  if (millis() - last[id] < WARN_EVERY_MS) return;
  last[id] = millis();
  Serial.printf("[WARN] %s\n", msg);
}

// =====================================================================
//  BNO085 over SHTP (I²C), no library
// =====================================================================
uint8_t buf[128];
uint8_t seqOut[6] = {0};

bool waitInt(uint32_t ms) {
  uint32_t t0 = millis();
  while (digitalRead(IMU_INT) && millis() - t0 < ms) delay(1);
  return !digitalRead(IMU_INT);
}

// Reads one SHTP packet into buf. Returns bytes read, 0 if empty, -1 on error.
int readPacket() {
  if (Wire.requestFrom(IMU_ADDR, 4) != 4) return -1;
  uint8_t h[4];
  for (int i = 0; i < 4; i++) h[i] = Wire.read();
  uint16_t len = (h[0] | (h[1] << 8)) & 0x7FFF;
  if (len == 0) return 0;
  int want = len > 128 ? 128 : len;
  int got = Wire.requestFrom(IMU_ADDR, want);
  for (int i = 0; i < got; i++) buf[i] = Wire.read();
  return got;
}

bool sendPacket(uint8_t ch, const uint8_t *data, uint8_t n) {
  uint16_t len = n + 4;
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(len & 0xFF);
  Wire.write(len >> 8);
  Wire.write(ch);
  Wire.write(seqOut[ch]++);
  Wire.write(data, n);
  return Wire.endTransmission() == 0;
}

// Set Feature command (0xFD) on the control channel
bool enableReport(uint8_t reportId, uint32_t intervalUs) {
  uint8_t cmd[17] = {0};
  cmd[0] = 0xFD;
  cmd[1] = reportId;
  for (int b = 0; b < 4; b++) cmd[5 + b] = (intervalUs >> (8 * b)) & 0xFF;
  return sendPacket(2, cmd, 17);
}

// Reset, clear start-up packets, turn on the game rotation vector (0x08):
// gyro + accelerometer fused on-chip, no magnetometer, so the motor can't disturb it.
bool imuStart() {
  digitalWrite(IMU_RST, LOW);
  delay(10);
  digitalWrite(IMU_RST, HIGH);
  if (!waitInt(2000)) {
    Serial.println("[IMU] No INT after reset – check RST (26) and INT (27)");
    return false;
  }
  while (waitInt(300)) {
    if (readPacket() < 0) break;
  }
  if (!enableReport(0x08, IMU_PERIOD_US)) {
    Serial.println("[IMU] Enable command not acknowledged");
    return false;
  }
  lastImuMs = millis();
  Serial.println("[IMU] Running – game rotation vector at 100 Hz");
  return true;
}

void handleQuat(int16_t qi16, int16_t qj16, int16_t qk16, int16_t qr16) {
  const float s = 1.0f / 16384.0f;          // Q14
  float x = qi16 * s, y = qj16 * s, z = qk16 * s, w = qr16 * s;
  float rRaw = atan2f(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * RAD_TO_DEG;
  float sp = 2 * (w * y - z * x);
  sp = sp > 1 ? 1 : (sp < -1 ? -1 : sp);
  float pRaw = asinf(sp) * RAD_TO_DEG;

  // Unwrap: an upside-down IMU sits near ±180° roll, where the raw value
  // jumps between +180 and -180. Track the change instead, so the angle is
  // continuous and the filter and error maths stay valid.
  static bool init = false;
  static float lastR = 0, lastP = 0;
  if (!init) { rollDeg = rRaw; pitchDeg = pRaw; init = true; }
  else {
    float dr = rRaw - lastR; while (dr > 180) dr -= 360; while (dr < -180) dr += 360;
    float dp = pRaw - lastP; while (dp > 180) dp -= 360; while (dp < -180) dp += 360;
    rollDeg += dr; pitchDeg += dp;
  }
  lastR = rRaw; lastP = pRaw;
  lastImuMs = millis();
}

// Read every packet the IMU has waiting (bounded so the loop stays on time)
void imuService() {
  for (int n = 0; n < 4 && !digitalRead(IMU_INT); n++) {
    int got = readPacket();
    if (got < 5) return;
    uint8_t ch = buf[2];

    if (ch == 1 && buf[4] == 0x01) {         // chip reset itself
      Serial.println("[IMU] Reset detected, re-enabling");
      enableReport(0x08, IMU_PERIOD_US);
      continue;
    }
    if (ch != 3) continue;                   // only sensor reports matter

    int i = 4;
    while (i < got) {
      uint8_t id = buf[i];
      if (id == 0xFB || id == 0xFA) {        // timestamp records
        i += 5;
      } else if (id == 0x08 && i + 12 <= got) {  // game rotation vector
        int16_t qi = buf[i + 4]  | (buf[i + 5]  << 8);
        int16_t qj = buf[i + 6]  | (buf[i + 7]  << 8);
        int16_t qk = buf[i + 8]  | (buf[i + 9]  << 8);
        int16_t qr = buf[i + 10] | (buf[i + 11] << 8);
        handleQuat(qi, qj, qk, qr);
        i += 12;
      } else {
        break;                               // unknown record, skip the rest
      }
    }
  }
}


bool imuFresh() { return millis() - lastImuMs < IMU_TIMEOUT_MS; }

// Smoothed roll, updated every time we service the IMU
void updateFilter() {
  float a = cal.axis == 0 ? rollDeg : pitchDeg;
  if (!filtInit) { filtRoll = a; filtR = rollDeg; filtP = pitchDeg; filtInit = true; return; }
  filtRoll += FILTER_ALPHA * (a - filtRoll);
  filtR += FILTER_ALPHA * (rollDeg - filtR);
  filtP += FILTER_ALPHA * (pitchDeg - filtP);
}

// Service the IMU and keep the filter running at ~100 Hz
void tick() {
  imuService();
  static uint32_t last = 0;
  if (millis() - last >= LOOP_MS) { last = millis(); updateFilter(); }
}

void waitMs(uint32_t ms) { uint32_t t0 = millis(); while (millis() - t0 < ms) { tick(); delay(1); } }

// =====================================================================
//  Actuator: raw > 0 = IN1 PWM = retract, raw < 0 = IN2 PWM = extend
// =====================================================================
void motorRaw(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0)      { ledcWrite(MOT_IN1, pwm); ledcWrite(MOT_IN2, 0); }
  else if (pwm < 0) { ledcWrite(MOT_IN1, 0);   ledcWrite(MOT_IN2, -pwm); }
  else              { ledcWrite(MOT_IN1, 255); ledcWrite(MOT_IN2, 255); }  // brake
}
void motorStop() { pwmNow = 0; motorRaw(0); }

// Ramp toward 'raw' and resend it every 10 ms, the same way the hold loop
// drives the motor (a single jump to full power did not move it on the rig).
int rampNow = 0;
void rampStep(int raw) {
  if (raw > rampNow + SLEW_PER_LOOP) rampNow += SLEW_PER_LOOP;
  else if (raw < rampNow - SLEW_PER_LOOP) rampNow -= SLEW_PER_LOOP;
  else rampNow = raw;
  motorRaw(rampNow);
}
void driveRamped(int raw, uint32_t ms) {
  rampNow = 0;
  uint32_t t0 = millis(), last = 0;
  while (millis() - t0 < ms) {
    tick(); delay(1);
    if (millis() - last >= LOOP_MS) { last = millis(); rampStep(raw); }
  }
}

// Drive one way until neither roll nor pitch changes for CAL_STABLE_MS
// (at least CAL_MIN_MS, at most CAL_TIMEOUT_MS). Returns ms spent moving.
uint32_t runToEnd(int raw) {
  uint32_t t0 = millis(), lastMove = millis(), last = 0;
  float lr = filtR, lp = filtP;
  rampNow = 0;
  while (millis() - t0 < CAL_TIMEOUT_MS) {
    tick(); delay(1);
    if (millis() - last >= LOOP_MS) { last = millis(); rampStep(raw); }
    if (fabsf(filtR - lr) > END_MOVE_DEG || fabsf(filtP - lp) > END_MOVE_DEG) {
      lr = filtR; lp = filtP; lastMove = millis();
    }
    if (millis() - t0 > CAL_MIN_MS && millis() - lastMove > CAL_STABLE_MS) break;
  }
  motorStop(); rampNow = 0;
  uint32_t moving = lastMove - t0;
  waitMs(400);
  return moving;
}


// =====================================================================
//  Saved direction (learned from jogs)
// =====================================================================
void saveCal() {
  prefs.begin(PREFS_NS, false);
  prefs.putInt("ok", cal.ok);
  prefs.putInt("dir", cal.dir);
  prefs.putFloat("rret", cal.rollRet);
  prefs.putFloat("rext", cal.rollExt);
  prefs.putInt("tms", cal.travelMs);
  prefs.putInt("axis", cal.axis);
  prefs.end();
}
void loadCal() {
  prefs.begin(PREFS_NS, true);
  cal.ok       = prefs.getInt("ok", 0);
  cal.dir      = prefs.getInt("dir", 0);
  cal.rollRet  = prefs.getFloat("rret", 0);
  cal.rollExt  = prefs.getFloat("rext", 0);
  cal.travelMs = prefs.getInt("tms", 0);
  cal.axis     = prefs.getInt("axis", 0);
  prefs.end();
}
void printCal() {
  if (!cal.ok) { Serial.println("[DIR] Not learned yet – jog with e or r once (needs > 1 deg of movement)"); return; }
  Serial.printf("[DIR] axis %s, dir %d (learned from a jog)\n", cal.axis == 0 ? "roll" : "pitch", cal.dir);
}

void calibrate() {
  bool was = running; running = false; motorStop();
  if (!imuFresh()) { Serial.println("[CAL] No IMU reading"); return; }
  Serial.println("[CAL] Hold the base still. Retracting fully...");
  runToEnd(+CAL_PWM);
  float r1 = filtR, p1 = filtP;
  Serial.printf("[CAL] Full retract: roll %.2f, pitch %.2f. Extending fully...\n", r1, p1);
  uint32_t t = runToEnd(-CAL_PWM);
  float r2 = filtR, p2 = filtP;
  Serial.printf("[CAL] Full extend:  roll %.2f, pitch %.2f (moved for %lu ms)\n", r2, p2, (unsigned long)t);

  float dr = r2 - r1, dp = p2 - p1;
  Serial.printf("[CAL] Change over the stroke: roll %+.2f, pitch %+.2f deg\n", dr, dp);
  if (fabsf(dr) < 2.0f && fabsf(dp) < 2.0f) {
    Serial.println("[CAL] Neither axis moved 2 deg. Check: IMU on the part the actuator tilts,");
    Serial.println("      actuator linked to it, motor powered (12 V). Not saved.");
    running = was;
    return;
  }
  // Use whichever axis the actuator actually moves
  cal.axis = fabsf(dr) >= fabsf(dp) ? 0 : 1;
  float range = cal.axis == 0 ? dr : dp;
  cal.ok = 1;
  cal.rollRet = cal.axis == 0 ? r1 : p1;
  cal.rollExt = cal.axis == 0 ? r2 : p2;
  cal.travelMs = t;
  // Extending (raw < 0) changes the angle by 'range'. To RAISE the angle we
  // need raw < 0 if range > 0, so the raw sign for "raise" is -sign(range).
  cal.dir = range > 0 ? -1 : +1;
  saveCal();
  filtInit = false;                          // restart the filter on the chosen axis
  Serial.printf("[CAL] Using %s. ", cal.axis == 0 ? "roll" : "pitch");
  printCal();

  // Park in the middle of the stroke
  driveRamped(+CAL_PWM, t / 2);
  motorStop();
  waitMs(300);
  Serial.println("[CAL] Done. Set the platform where you want it and type s to record it.");
  running = false;
}


// =====================================================================
//  Control loop (every 10 ms)
// =====================================================================
void controlStep() {
  float dt = LOOP_MS / 1000.0f;
  if (!imuFresh()) {
    motorStop(); active = false; integ = 0;
    warn(0, "No IMU reading – motor paused until it's back");
    return;
  }

  float err = wrap180(refRoll - filtRoll);   // + means the roll must go up
  float aerr = fabsf(err);

  // Deadband: start once the error has stayed above db for CONFIRM_MS,
  // stop once it is back inside dbExit
  if (aerr > db) { if (!outsideSince) outsideSince = millis(); }
  else outsideSince = 0;
  if (!active && outsideSince && millis() - outsideSince >= CONFIRM_MS) active = true;
  else if (active && aerr < dbExit) { active = false; integ = 0; }

  int target = 0;
  if (active) {
    integ += err * dt;
    float iMax = ki > 0 ? 150.0f / ki : 0;
    integ = constrain(integ, -iMax, iMax);
    float u = kp * err + ki * integ;
    float mag = fabsf(u);
    if (mag > 255) mag = 255;
    if (mag < minPwm) mag = minPwm;
    target = (u >= 0 ? 1 : -1) * (int)mag;
  }

  // Slew limit
  if (target > pwmNow + SLEW_PER_LOOP) pwmNow += SLEW_PER_LOOP;
  else if (target < pwmNow - SLEW_PER_LOOP) pwmNow -= SLEW_PER_LOOP;
  else pwmNow = target;

  int raw = cal.dir * pwmNow;                // positive pwmNow raises the roll

  // End of travel: if we keep driving one way and the roll doesn't move,
  // stop pushing that way until the controller asks for the other direction
  int rawSign = raw > 0 ? 1 : (raw < 0 ? -1 : 0);
  if (endBlock != 0 && rawSign == -endBlock) endBlock = 0;   // reversing clears it
  if (rawSign != 0 && rawSign == endBlock) raw = 0;          // blocked direction
  if (abs(raw) >= minPwm) {
    if (rawSign != endSign) { endSign = rawSign; endStart = millis(); endRoll0 = filtRoll; }
    if (fabsf(filtRoll - endRoll0) > END_MOVE_DEG) { endStart = millis(); endRoll0 = filtRoll; }
    if (millis() - endStart > END_MS) {
      endBlock = rawSign;
      warn(1, rawSign > 0 ? "Actuator at full retract – can't go further" : "Actuator at full extend – can't go further");
    }
  } else {
    endSign = 0;
  }

  motorRaw(raw);
}

// =====================================================================
//  Commands
// =====================================================================
void printHelp() {
  Serial.println(
    "\nCommands:\n"
    "  c          optional calibrate: full retract -> full extend -> park mid-stroke\n"
    "  t          hold the hardcoded target again (this is what runs at power-up)\n"
    "  s          record the current roll and start holding it (continuous)\n"
    "  x          stop (holding or jogging)\n"
    "  v 200      jog power 0-255 (default 255)\n"
    "  e / r      extend / retract until x (10 s max); e 1.5 = for 1.5 s\n"
    "  kp <v>  ki <v>  db <v>  dbx <v>  minpwm <v>   tune live\n"
    "  p          print learned direction and tuning     h   help");
}

// Stop a jog and learn from it which axis the actuator tilts and which motor
// direction raises the angle. This replaces the old full-stroke calibration.
void endJog(const char *why) {
  jogDir = 0; jogNow = 0; motorStop();
  float dR = filtR - jogR0, dP = filtP - jogP0;
  Serial.printf("[JOG] %s at %.2f deg (roll %+.2f, pitch %+.2f)\n", why, filtRoll, dR, dP);
  if (fabsf(dR) < 1.0f && fabsf(dP) < 1.0f) {
    Serial.println("[JOG] Moved < 1 deg. Check the 12 V supply and that it isn't at an end.");
    return;
  }
  int axis = fabsf(dP) > fabsf(dR) ? 1 : 0;
  float d = axis ? dP : dR;
  int dir = lastJogRaw * (d > 0 ? 1 : -1);   // motor sign that raises the angle
  if (!cal.ok || axis != cal.axis || dir != cal.dir) {
    if (axis != cal.axis) filtInit = false;
    cal.ok = 1; cal.axis = axis; cal.dir = dir;
    saveCal();
    printCal();
  }
}

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;
  String cmd = line, arg = "";
  int sp = line.indexOf(' ');
  if (sp > 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); }
  float v = arg.toFloat();
  Serial.printf("[CMD] %s\n", line.c_str());
  if (jogDir != 0) { endJog("stopped"); if (cmd == "x") return; }

  if (cmd == "c") {
    calibrate();
  } else if (cmd == "s") {
    if (!cal.ok) { Serial.println("Type c, or jog with e / r once, so it learns which way is up"); return; }
    if (!imuFresh()) { Serial.println("No IMU reading"); return; }
    float sum = 0; int n = 0;                // average 0.5 s for a clean reference
    uint32_t t0 = millis();
    while (millis() - t0 < 500) { tick(); delay(1); sum += filtRoll; n++; }
    refRoll = sum / n;
    pwmNow = 0; integ = 0; active = false; outsideSince = 0; endBlock = 0; endSign = 0;
    running = true;
    Serial.printf("[HOLD] Recorded roll %.2f deg. Holding it now.\n", refRoll);
  } else if (cmd == "t") {
    if (!cal.ok) { Serial.println("Direction not learned yet"); return; }
    startHold();
  } else if (cmd == "x") {
    running = false; motorStop(); active = false; integ = 0;
    Serial.println("[STOPPED] type s to record and hold again");
  } else if (cmd == "e" || cmd == "r") {
    // Jog: runs until x (or any other command), or for the seconds given
    // ("e 1.5"), with a 10 s safety limit. Stops holding while you jog.
    if (running) { running = false; active = false; integ = 0; Serial.println("[HOLD OFF] jogging"); }
    float secs = arg.length() ? v : 10.0f;
    secs = constrain(secs, 0.05f, 10.0f);
    jogDir = (cmd == "e") ? -1 : +1;           // IN2 (raw < 0) extends on this wiring
    jogUntil = millis() + (uint32_t)(secs * 1000);
    jogR0 = filtR; jogP0 = filtP;
    lastJogRaw = jogDir; jogNow = 0;            // ramped up in loop(), like the controller
    Serial.printf("%s (type x to stop%s)\n", cmd == "e" ? "Extending" : "Retracting",
                  arg.length() ? "" : ", 10 s max");
  }
  else if (cmd == "v")      { jogPwm = constrain((int)v, 0, 255); Serial.printf("Jog power %d\n", jogPwm); return; }
  else if (cmd == "kp")     { kp = v; }
  else if (cmd == "ki")     { ki = v; integ = 0; }
  else if (cmd == "db")     { db = v; }
  else if (cmd == "dbx")    { dbExit = v; }
  else if (cmd == "minpwm") { minPwm = constrain((int)v, 0, 255); }
  else if (cmd == "p")      { printCal(); }
  else if (cmd == "h")      { printHelp(); return; }
  else { Serial.println("Unknown command, type h"); return; }
  if (cmd == "kp" || cmd == "ki" || cmd == "db" || cmd == "dbx" || cmd == "minpwm" || cmd == "p")
    Serial.printf("kp=%.1f ki=%.1f db=%.2f dbx=%.2f minpwm=%d\n", kp, ki, db, dbExit, minPwm);
}

// =====================================================================
// No direction saved: nudge extend then retract and learn from the movement
bool autoLearn() {
  Serial.println("[AUTO] No saved direction. Nudging the actuator to learn it...");
  jogDir = -1; lastJogRaw = -1; jogR0 = filtR; jogP0 = filtP;
  driveRamped(-jogPwm, LEARN_MS);
  endJog("auto-learn");
  driveRamped(+jogPwm, LEARN_MS);             // go back to where it started
  motorStop();
  waitMs(500);
  return cal.ok;
}

void startHold() {
  refRoll = cal.axis == 0 ? TARGET_ROLL : TARGET_PITCH;
  pwmNow = 0; integ = 0; active = false; outsideSince = 0; endBlock = 0; endSign = 0;
  running = true;
  Serial.printf("[HOLD] Holding %s at %.2f deg (hardcoded target)\n", cal.axis == 0 ? "roll" : "pitch", refRoll);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[RB4107] Levelling controller (roll, platform IMU)");

  ledcAttach(MOT_IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(MOT_IN2, PWM_FREQ, PWM_BITS);
  motorStop();

  pinMode(IMU_INT, INPUT_PULLUP);
  pinMode(IMU_RST, OUTPUT);
  digitalWrite(IMU_RST, HIGH);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(1000);

  loadCal();
  printHelp();
  while (!imuStart()) delay(1000);
  waitMs(500);
  printCal();
  for (int i = 0; i < 3 && !cal.ok; i++) autoLearn();
  waitMs(500);                                 // let the filter settle
  if (cal.ok) startHold();
  else Serial.println("[WARN] Couldn't learn the direction (actuator didn't move 1 deg). Check 12 V, then reset.");
}

void loop() {
  tick();

  static uint32_t lastRetry = 0;
  if (!imuFresh() && millis() - lastRetry > 2000) { lastRetry = millis(); imuStart(); }

  static String line;
  static uint32_t lastChar = 0;
  while (Serial.available()) {
    char c = Serial.read();
    lastChar = millis();
    if (c == '\n' || c == '\r') { handleCommand(line); line = ""; }
    else line += c;
  }
  if (line.length() > 0 && millis() - lastChar > 150) { handleCommand(line); line = ""; }

  if (jogDir != 0 && (int32_t)(millis() - jogUntil) >= 0) endJog("done");

  if (millis() - lastLoop >= LOOP_MS) {
    lastLoop = millis();
    if (running) controlStep();
    else if (jogDir != 0) {                     // ramp and refresh every 10 ms, same as holding
      int target = jogDir * jogPwm;
      if (target > jogNow + SLEW_PER_LOOP) jogNow += SLEW_PER_LOOP;
      else if (target < jogNow - SLEW_PER_LOOP) jogNow -= SLEW_PER_LOOP;
      else jogNow = target;
      motorRaw(jogNow);
    }
  }

  if (millis() - lastTele >= TELEMETRY_MS) {
    lastTele = millis();
    if (running)
      Serial.printf("angle:%.2f,target:%.2f,error:%.2f,pwm:%d\n", filtRoll, refRoll, wrap180(refRoll - filtRoll), pwmNow);
    else
      Serial.printf("roll:%.2f,pitch:%.2f  [%s]\n", filtR, filtP, cal.ok ? "set with e / r, x to stop, then s to hold" : "type c, or jog with e / r first");
  }
}
