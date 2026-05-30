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
// Position 1 is all the way left. Position 4 is all the way right.
// Startup assumes the rack is physically at position 2.
//
// Tune these however you want. With the defaults, each neighboring position is
// 1000 ms apart:
//   1 <-> 2 = 1000 ms
//   2 <-> 3 = 1000 ms
//   3 <-> 4 = 1000 ms
const long POSITION_MS[4] = {
  0,      // 1: all the way left
  2200,   // 2: startup position
  3200,   // 3
  4200    // 4: all the way right
};
const int START_POSITION = 2;
// ------------------------------------------------

enum Mode {
  MODE_STOPPED,
  MODE_TIMED_MOVE
};

Mode mode = MODE_STOPPED;
int currentPosition = START_POSITION;
int targetPosition = START_POSITION;
long currentPos = POSITION_MS[START_POSITION - 1];
long targetPos = POSITION_MS[START_POSITION - 1];
unsigned long moveStartMs = 0;
unsigned long moveDurationMs = 0;
int moveDirection = 0;

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

void finishTimedMove() {
  currentPosition = targetPosition;
  currentPos = targetPos;
  mode = MODE_STOPPED;
  moveDirection = 0;
  stopServo();

  Serial.print("Reached position ");
  Serial.println(currentPosition);
}

void startTimedMove(int newTargetPosition) {
  if (newTargetPosition < 1 || newTargetPosition > 4) {
    Serial.println("Invalid position. Use 1, 2, 3, or 4.");
    return;
  }

  updatePositionEstimate();

  if (mode == MODE_STOPPED && newTargetPosition == currentPosition) {
    stopServo();
    Serial.print("Already at position ");
    Serial.println(currentPosition);
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
  Serial.print(" to ");
  Serial.print(targetPosition);
  Serial.print(" for ");
  Serial.print(moveDurationMs);
  Serial.println(" ms.");
}

void handleCommand(char command) {
  switch (command) {
    case '1':
    case '2':
    case '3':
    case '4':
      startTimedMove(command - '0');
      break;
    case 'S':
    case 's':
    case ' ':
      stopServo();
      Serial.println("Stopped.");
      break;
    default:
      Serial.println("Unknown command. Use 1, 2, 3, 4, or S.");
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
  Serial.println("Startup assumes current software position is 2.");
  Serial.println("Commands: 1 left end, 2 start, 3, 4 right end, S stop.");
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
}
