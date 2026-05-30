#include <Servo.h>

Servo rackServo;

// ---------------- Tunable setup ----------------
const int SERVO_PIN = 3;

// Continuous-rotation servo values. If the rack moves the wrong way, swap
// LEFT_SPEED and RIGHT_SPEED. If it creeps at rest, tune STOP_SPEED around 90.
const int STOP_SPEED = 90;
const int LEFT_SPEED = 0;
const int RIGHT_SPEED = 180;

// Detach after stopping so a slightly-off STOP_SPEED cannot make a continuous
// servo drift back. Set false if your setup needs a continuous stop pulse.
const bool DETACH_WHEN_STOPPED = true;

// No homing/sensors: these are dead-reckoned time positions in milliseconds.
// Position 1 is all the way left. Position 5 is all the way right.
// Startup assumes the rack is physically at position 3.
//
// These are software positions, not sensor readings. Command numbers are
// 1-based, while this table is 0-based:
//   command 1 -> index 0 -> drill/CO2a
//   command 2 -> index 1 -> CO2b
//   command 3 -> index 2 -> startup position
const long POSITION_MS[5] = {
  0,      // 1: drill/CO2a
  1780,   // 2: CO2b
  2600,   // 3: startup position
  4200,   // 4
  7100    // 5
};
const int START_POSITION = 3;
const char* POSITION_LABELS[5] = {
  "drill/CO2a",
  "CO2b",
  "startup",
  "position 4",
  "position 5"
};
// ------------------------------------------------

enum Mode {
  MODE_STOPPED,
  MODE_TIMED_MOVE,
  MODE_MANUAL_JOG
};

Mode mode = MODE_STOPPED;
int currentPosition = START_POSITION;
int targetPosition = START_POSITION;
long currentPos = POSITION_MS[START_POSITION - 1];
long targetPos = POSITION_MS[START_POSITION - 1];
unsigned long moveStartMs = 0;
unsigned long moveDurationMs = 0;
unsigned long lastJogCommandMs = 0;
int moveDirection = 0;

const unsigned long JOG_TIMEOUT_MS = 250;

void updatePositionEstimate() {
  if (mode != MODE_TIMED_MOVE) {
    return;
  }

  unsigned long elapsedMs = millis() - moveStartMs;
  if (elapsedMs > moveDurationMs) {
    elapsedMs = moveDurationMs;
  }

  currentPos += moveDirection * (long)elapsedMs;
  moveStartMs = millis();
  moveDurationMs -= elapsedMs;
}

void ensureServoAttached() {
  if (!rackServo.attached()) {
    rackServo.attach(SERVO_PIN);
  }
}

void stopServo() {
  updatePositionEstimate();

  if (rackServo.attached()) {
    rackServo.write(STOP_SPEED);
    if (DETACH_WHEN_STOPPED) {
      delay(20);
      rackServo.detach();
    }
  }

  mode = MODE_STOPPED;
  moveDirection = 0;
}

void resetSoftwarePosition() {
  stopServo();

  currentPosition = START_POSITION;
  targetPosition = START_POSITION;
  currentPos = POSITION_MS[START_POSITION - 1];
  targetPos = POSITION_MS[START_POSITION - 1];

  Serial.print("Reset software position to ");
  Serial.print(currentPosition);
  printPositionLabel(currentPosition);
  Serial.println();
}

void printPositionLabel(int position) {
  Serial.print(" (");
  Serial.print(POSITION_LABELS[position - 1]);
  Serial.print(")");
}

void finishTimedMove() {
  currentPosition = targetPosition;
  currentPos = targetPos;
  mode = MODE_STOPPED;
  moveDirection = 0;
  stopServo();

  Serial.print("Reached position ");
  Serial.print(currentPosition);
  printPositionLabel(currentPosition);
  Serial.println();
}

void startTimedMove(int newTargetPosition) {
  if (newTargetPosition < 1 || newTargetPosition > 5) {
    Serial.println("Invalid position. Use 1, 2, 3, 4, or 5.");
    return;
  }

  updatePositionEstimate();

  if (mode == MODE_STOPPED && newTargetPosition == currentPosition) {
    stopServo();
    Serial.print("Already at position ");
    Serial.print(currentPosition);
    printPositionLabel(currentPosition);
    Serial.println();
    return;
  }

  targetPosition = newTargetPosition;
  targetPos = POSITION_MS[targetPosition - 1];

  long delta = targetPos - currentPos;
  if (delta == 0) {
    finishTimedMove();
    return;
  }

  moveDirection = (delta < 0) ? -1 : 1;
  moveDurationMs = abs(delta);
  moveStartMs = millis();
  mode = MODE_TIMED_MOVE;

  ensureServoAttached();
  rackServo.write((moveDirection < 0) ? LEFT_SPEED : RIGHT_SPEED);

  Serial.print("Moving from position ");
  Serial.print(currentPosition);
  printPositionLabel(currentPosition);
  Serial.print(" to ");
  Serial.print(targetPosition);
  printPositionLabel(targetPosition);
  Serial.print(" for ");
  Serial.print(moveDurationMs);
  Serial.println(" ms.");
}

void startManualJog(int direction) {
  if (mode == MODE_MANUAL_JOG && moveDirection == direction) {
    lastJogCommandMs = millis();
    return;
  }

  stopServo();

  moveDirection = direction;
  mode = MODE_MANUAL_JOG;
  lastJogCommandMs = millis();

  ensureServoAttached();
  rackServo.write((moveDirection < 0) ? LEFT_SPEED : RIGHT_SPEED);

  Serial.print("Manual jog ");
  Serial.println((moveDirection < 0) ? "left." : "right.");
}

void handleCommand(char command) {
  switch (command) {
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
      startTimedMove(command - '0');
      break;
    case 'S':
    case 's':
    case ' ':
      stopServo();
      Serial.println("Stopped.");
      break;
    case 'L':
    case 'l':
    case '-':
      startManualJog(-1);
      break;
    case 'R':
    case 'r':
      resetSoftwarePosition();
      break;
    case '+':
    case '=':
      startManualJog(1);
      break;
    default:
      Serial.println("Unknown command. Use 1, 2, 3, 4, 5, L, +, R, or S.");
      break;
  }
}

void setup() {
  Serial.begin(115200);

  currentPosition = START_POSITION;
  targetPosition = START_POSITION;
  currentPos = POSITION_MS[START_POSITION - 1];
  targetPos = POSITION_MS[START_POSITION - 1];

  Serial.println("Arduino rack servo controller ready.");
  Serial.println("Startup assumes current software position is 3.");
  Serial.println("Commands: 1 drill/CO2a, 2 CO2b, 3 start, 4, 5, L/- jog left, +/= jog right, R reset, S stop.");
}

void loop() {
  while (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '\n' || command == '\r') {
      continue;
    }
    handleCommand(command);
  }

  unsigned long now = millis();

  if (mode == MODE_TIMED_MOVE && now - moveStartMs >= moveDurationMs) {
    finishTimedMove();
  }

  if (mode == MODE_MANUAL_JOG && now - lastJogCommandMs > JOG_TIMEOUT_MS) {
    stopServo();
    Serial.println("Manual jog timeout stop.");
  }
}
