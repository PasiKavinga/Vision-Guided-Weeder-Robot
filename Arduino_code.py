// weeder_esp32_primitives_fixed.ino
// Keep pin assignments unchanged. Uses AccelStepper DRIVER mode (STEP, DIR).
// Implements primitives: MOVE x y, LOWER, LIFT_CM <cm>, GRIP_CLOSE, GRIP_OPEN,
// GRIPTEST, JOG, SET_SP*, HOME. Replies with "BUSY" at start and "DONE"/"OK"/"ERROR".

#include <AccelStepper.h>

// --- Pin assignments (final) --- (unchanged)
#define X_DIR 25
#define X_STEP 26
#define Y_DIR 27
#define Y_STEP 14
#define Z_DIR 32
#define Z_STEP 33
#define G_DIR 18
#define G_STEP 19
#define EN_PIN 23     // Common EN (LOW = enabled)
#define LIMIT_PIN 21  // limit switch for Z (pressed -> LOW)
// -----------------------------------------

// If your motors are physically swapped, set this to 1 to swap X/Y mapping server-side.
// Default 0 (do not swap). You can flip this if you find X/Y reversed.
#define SWAP_AXES 0

// AccelStepper(driver, stepPin, dirPin)
AccelStepper stepX(AccelStepper::DRIVER, X_STEP, X_DIR);
AccelStepper stepY(AccelStepper::DRIVER, Y_STEP, Y_DIR);
AccelStepper stepZ(AccelStepper::DRIVER, Z_STEP, Z_DIR);
AccelStepper stepG(AccelStepper::DRIVER, G_STEP, G_DIR);

// Calibration params (can be updated via SET_SPX etc)
volatile float steps_per_cm_x = 200.0;
volatile float steps_per_cm_y = 200.0;
volatile float steps_per_cm_z = 200.0;
volatile int grip_close_steps = -100;      // steps for ~180deg (adjust)
volatile float lift_cm_after_grip = 5.0; // default lift

const long z_lower_safety_steps = 200000; // safety upper bound for lowering
const int limit_debounce_ms = 40;

String inBuffer = "";

void setup() {
  Serial.begin(115200);
  delay(100);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, LOW); // enable A4988 drivers
  pinMode(LIMIT_PIN, INPUT_PULLUP); // pressed -> LOW

  // conservative speeds and accelerations (tune as needed)
  stepX.setMaxSpeed(400.0); stepX.setAcceleration(200.0);
  stepY.setMaxSpeed(400.0); stepY.setAcceleration(200.0);
  stepZ.setMaxSpeed(200.0); stepZ.setAcceleration(100.0);
  stepG.setMaxSpeed(200.0); stepG.setAcceleration(100.0);

  stepX.setCurrentPosition(0);
  stepY.setCurrentPosition(0);
  stepZ.setCurrentPosition(0);
  stepG.setCurrentPosition(0);

  // Ready
  Serial.println("READY");
}

// helpers
void moveXToCm(float x_cm) {
  long tx = (long)round(x_cm * steps_per_cm_x);
  stepX.moveTo(tx);
  while (stepX.distanceToGo() != 0) stepX.run();
}

void moveYToCm(float y_cm) {
  long ty = (long)round(y_cm * steps_per_cm_y);
  stepY.moveTo(ty);
  while (stepY.distanceToGo() != 0) stepY.run();
}

bool lowerUntilLimit() {
  long startPos = stepZ.currentPosition();
  stepZ.setSpeed(-200);  // try -200 for faster down, adjust if too fast
  while (digitalRead(LIMIT_PIN) == HIGH) {
    stepZ.runSpeed();
    long delta = abs(stepZ.currentPosition() - startPos);
    if (delta > z_lower_safety_steps) {
      Serial.println("[SAFETY] Z lower timeout");
      stepZ.stop();
      return false;
    }
  }
  delay(limit_debounce_ms);
  return (digitalRead(LIMIT_PIN) == LOW);
}


void moveZUpCm(float cm) {
  long steps = (long)round(cm * steps_per_cm_z);
  long target = stepZ.currentPosition() + steps;
  stepZ.moveTo(target);
  while (stepZ.distanceToGo() != 0) stepZ.run();
}

void gripClose() {
  long target = stepG.currentPosition() + grip_close_steps;
  stepG.moveTo(target);
  while (stepG.distanceToGo() != 0) stepG.run();
}

void gripOpen() {
  long target = stepG.currentPosition() - grip_close_steps;
  stepG.moveTo(target);
  while (stepG.distanceToGo() != 0) stepG.run();
}

void jogX(float cm) { long steps = (long)round(cm * steps_per_cm_x); stepX.move(steps); while (stepX.distanceToGo() != 0) stepX.run(); }
void jogY(float cm) { long steps = (long)round(cm * steps_per_cm_y); stepY.move(steps); while (stepY.distanceToGo() != 0) stepY.run(); }
void jogZ(float cm) { long steps = (long)round(cm * steps_per_cm_z); stepZ.move(steps); while (stepZ.distanceToGo() != 0) stepZ.run(); }

// Full pick (fallback device-side)
bool executePickSequence(float x_cm, float y_cm) {
  // Respect SWAP_AXES if compiled that way
  if (SWAP_AXES) {
    float tmp = x_cm; x_cm = y_cm; y_cm = tmp;
  }

  moveXToCm(x_cm);
  moveYToCm(y_cm);

  bool pressed = lowerUntilLimit();
  if (!pressed) {
    Serial.println("[ERR] LIMIT_NOT_PRESSED");
    return false;
  }

  gripClose();
  moveZUpCm(lift_cm_after_grip);
  gripOpen();
  return true;
}

// -------------------- Command processing --------------------
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  // Full PICK (GOTO)
  if (cmd.startsWith("GOTO")) {
    float x=0,y=0;
    int sp = cmd.indexOf(' ');
    if (sp > 0) {
      String rest = cmd.substring(sp+1);
      int sp2 = rest.indexOf(' ');
      if (sp2 > 0) {
        x = rest.substring(0, sp2).toFloat();
        y = rest.substring(sp2+1).toFloat();
        Serial.println("BUSY");
        bool ok = executePickSequence(x,y);
        if (ok) Serial.println("DONE"); else Serial.println("ERROR");
        return;
      }
    }
    Serial.println("ERROR");
    return;
  }

  // MOVE x y -> absolute (cm)
  if (cmd.startsWith("MOVE")) {
    // Accept both "MOVE <x> <y>" and "MOVE X <val> Y <val>"
    // First try simple two value parse
    float x=0,y=0;
    bool parsed = false;
    // attempt simple parse
    {
      int sp = cmd.indexOf(' ');
      if (sp > 0) {
        String rest = cmd.substring(sp+1);
        int sp2 = rest.indexOf(' ');
        if (sp2 > 0) {
          x = rest.substring(0, sp2).toFloat();
          y = rest.substring(sp2+1).toFloat();
          parsed = true;
        }
      }
    }
    // if failed, try labeled form "MOVE X <val> Y <val>"
    if (!parsed) {
      // naive extraction
      int xi = cmd.indexOf('X');
      int yi = cmd.indexOf('Y');
      if (xi >= 0 && yi >= 0) {
        // X may be "X " followed by number
        int spx = cmd.indexOf(' ', xi);
        int spy = cmd.indexOf(' ', yi);
        if (spx >= 0 && spy >= 0) {
          String sx = cmd.substring(spx+1, (spy>spx?spy:cmd.length()));
          sx.trim();
          String sy = cmd.substring(spy+1);
          sy.trim();

          x = sx.toFloat(); y = sy.toFloat();
          parsed = true;
        }
      }
    }

    if (parsed) {
      Serial.println("BUSY");
      if (SWAP_AXES) {
        float tmp = x; x = y; y = tmp;
      }
      moveXToCm(x);
      moveYToCm(y);
      Serial.println("DONE");
      return;
    }

    Serial.println("ERROR");
    return;
  }

  // LOWER (or synonym Z_DOWN)
  if (cmd == "LOWER" || cmd == "Z_DOWN") {
    Serial.println("BUSY");
    bool ok = lowerUntilLimit();
    if (ok) Serial.println("DONE"); else Serial.println("ERROR");
    return;
  }

  // LIFT_CM <cm> (or Z_UP synonym, if given without value use lift_cm_after_grip)
  if (cmd.startsWith("LIFT_CM") || cmd.startsWith("Z_UP")) {
    if (cmd.startsWith("LIFT_CM")) {
      int sp = cmd.indexOf(' ');
      if (sp > 0) {
        float cm = cmd.substring(sp+1).toFloat();
        Serial.println("BUSY");
        moveZUpCm(cm);
        Serial.println("DONE");
        return;
      } else { Serial.println("ERROR"); return; }
    } else { // Z_UP alone -> use configured lift_cm_after_grip
      Serial.println("BUSY");
      moveZUpCm(lift_cm_after_grip);
      Serial.println("DONE");
      return;
    }
  }

  // GRIP_CLOSE / GRIP_OPEN
  if (cmd == "GRIP_CLOSE") {
    Serial.println("BUSY");
    gripClose();
    Serial.println("DONE");
    return;
  }
  if (cmd == "GRIP_OPEN") {
    Serial.println("BUSY");
    gripOpen();
    Serial.println("DONE");
    return;
  }

  if (cmd == "GRIPTEST") {
    Serial.println("BUSY");
    gripClose();
    delay(120);
    gripOpen();
    Serial.println("DONE");
    return;
  }

  // JOG
  if (cmd.startsWith("JOG")) {
    int sp = cmd.indexOf(' ');
    if (sp > 0) {
      String rest = cmd.substring(sp+1);
      int sp2 = rest.indexOf(' ');
      if (sp2 > 0) {
        String axis = rest.substring(0, sp2);
        float val = rest.substring(sp2+1).toFloat();
        if (axis == "X") { jogX(val); Serial.println("OK"); return; }
        if (axis == "Y") { jogY(val); Serial.println("OK"); return; }
        if (axis == "Z") { jogZ(val); Serial.println("OK"); return; }
      }
    }
    Serial.println("ERROR");
    return;
  }

  // SET params
  if (cmd.startsWith("SET_SPX")) { steps_per_cm_x = cmd.substring(7).toFloat(); Serial.println("OK"); return; }
  if (cmd.startsWith("SET_SPY")) { steps_per_cm_y = cmd.substring(7).toFloat(); Serial.println("OK"); return; }
  if (cmd.startsWith("SET_SPZ")) { steps_per_cm_z = cmd.substring(7).toFloat(); Serial.println("OK"); return; }
  if (cmd.startsWith("SET_GSP")) { grip_close_steps = cmd.substring(7).toInt(); Serial.println("OK"); return; }
  if (cmd.startsWith("SET_LIFT")) { lift_cm_after_grip = cmd.substring(9).toFloat(); Serial.println("OK"); return; }

  if (cmd == "HOME") {
    // No hardware homing sensors implemented — just zero the internal positions.
    stepX.setCurrentPosition(0);
    stepY.setCurrentPosition(0);
    stepZ.setCurrentPosition(0);
    stepG.setCurrentPosition(0);
    Serial.println("OK");
    return;
  }

  Serial.println("UNKNOWN");
}

void loop() {
  // Read serial into buffer line-by-line
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inBuffer.length() > 0) {
        processCommand(inBuffer);
        inBuffer = "";
      }
    } else {
      inBuffer += c;
    }
  }

  // Allow steppers to run towards targets
  stepX.run();
  stepY.run();
  stepZ.run();
  stepG.run();
}
