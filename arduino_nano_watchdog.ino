// OPTIROUTE Arduino Nano + L298N
// ENA=5, IN1=6, IN2=7, ENB=11, IN3=9, IN4=10
// Accepts: forward, backward, left, right, stop, speed 0..255
// Watchdog stops motors if the Raspberry Pi stops refreshing movement commands.

const int ENA = 5;
const int IN1 = 6;
const int IN2 = 7;
const int ENB = 11;
const int IN3 = 9;
const int IN4 = 10;

int motorSpeed = 150;
unsigned long lastCommandMs = 0;
const unsigned long WATCHDOG_MS = 1000;

void setMotors(int leftSpeed, int rightSpeed) {
  analogWrite(ENA, constrain(abs(leftSpeed), 0, 255));
  analogWrite(ENB, constrain(abs(rightSpeed), 0, 255));
  digitalWrite(IN1, leftSpeed > 0);
  digitalWrite(IN2, leftSpeed < 0);
  digitalWrite(IN3, rightSpeed > 0);
  digitalWrite(IN4, rightSpeed < 0);
}

void stopCar() { setMotors(0, 0); }
void forwardCar() { setMotors(motorSpeed, motorSpeed); }
void backwardCar() { setMotors(-motorSpeed, -motorSpeed); }
void leftCar() { setMotors(-motorSpeed, motorSpeed); }
void rightCar() { setMotors(motorSpeed, -motorSpeed); }

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(30);
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopCar();
  lastCommandMs = millis();
  Serial.println("CAR READY");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toLowerCase();
    lastCommandMs = millis();

    if (command == "forward") {
      forwardCar(); Serial.println("Moving Forward");
    } else if (command == "backward") {
      backwardCar(); Serial.println("Moving Backward");
    } else if (command == "left") {
      leftCar(); Serial.println("Turning Left");
    } else if (command == "right") {
      rightCar(); Serial.println("Turning Right");
    } else if (command == "stop") {
      stopCar(); Serial.println("Car Stopped");
    } else if (command.startsWith("speed ")) {
      int value = command.substring(6).toInt();
      motorSpeed = constrain(value, 0, 255);
      Serial.println("Speed Updated");
    } else {
      stopCar(); Serial.println("Unknown Command");
    }
  }

  // If Pi crashes, disconnects, or stops refreshing motion, stop the motors.
  if (millis() - lastCommandMs > WATCHDOG_MS) {
    stopCar();
  }
}
