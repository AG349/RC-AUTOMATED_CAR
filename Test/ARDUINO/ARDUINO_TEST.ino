#include <Servo.h>

// Motor pins (L298N) - Same as before
#define IN1 2    // Left front
#define IN2 3
#define IN3 4    // Left rear
#define IN4 A3   // Left rear (moved)
#define IN5 A4   // Right front (moved)
#define IN6 7
#define IN7 8    // Right rear
#define IN8 A5   // Right rear (moved)
#define ENA1 10  // L298N1 ENA (Left Front speed)
#define ENB1 11  // L298N1 ENB (Left Rear speed)
#define ENA2 5   // L298N2 ENA (Right Front speed)
#define ENB2 6   // L298N2 ENB (Right Rear speed)

// Sensor pins
#define TRIG 12
#define ECHO 13
#define SERVO_PIN A0
#define IR_LEFT A1
#define IR_RIGHT A2

Servo servo;
int servoAngle = 90;

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino Car Test Mode. Type commands: F (forward), B (back), L (left), R (right), S (stop), A (autonomous), U (ultrasonic scan)");
  
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(IN5, OUTPUT); pinMode(IN6, OUTPUT); pinMode(IN7, OUTPUT); pinMode(IN8, OUTPUT);
  pinMode(ENA1, OUTPUT); pinMode(ENB1, OUTPUT); pinMode(ENA2, OUTPUT); pinMode(ENB2, OUTPUT);
  
  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);
  pinMode(IR_LEFT, INPUT); pinMode(IR_RIGHT, INPUT);
  servo.attach(SERVO_PIN);
  servo.write(servoAngle);
  
  stopMotors();
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("Command received: ");
    Serial.println(cmd);
    
    if (cmd == 'F') moveForward();
    else if (cmd == 'B') moveBackward();
    else if (cmd == 'L') turnLeft();
    else if (cmd == 'R') turnRight();
    else if (cmd == 'S') stopMotors();
    else if (cmd == 'A') autonomousMode();
    else if (cmd == 'U') ultrasonicScan();
  }
  
  delay(100);
}

void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
  analogWrite(ENA1, 150); analogWrite(ENB1, 150);
  analogWrite(ENA2, 150); analogWrite(ENB2, 150);
}

void moveBackward() {
  Serial.println("Moving backward");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
  analogWrite(ENA1, 150); analogWrite(ENB1, 150);
  analogWrite(ENA2, 150); analogWrite(ENB2, 150);
}

void turnLeft() {
  Serial.println("Turning left");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
  analogWrite(ENA1, 100); analogWrite(ENB1, 100);
  analogWrite(ENA2, 100); analogWrite(ENB2, 100);
}

void turnRight() {
  Serial.println("Turning right");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
  analogWrite(ENA1, 100); analogWrite(ENB1, 100);
  analogWrite(ENA2, 100); analogWrite(ENB2, 100);
}

void stopMotors() {
  Serial.println("Stopping motors");
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW); digitalWrite(IN6, LOW);
  digitalWrite(IN7, LOW); digitalWrite(IN8, LOW);
  analogWrite(ENA1, 0); analogWrite(ENB1, 0);
  analogWrite(ENA2, 0); analogWrite(ENB2, 0);
}

void autonomousMode() {
  Serial.println("Autonomous mode: Scanning...");
  long distance = getUltrasonicDistance();
  Serial.print("Front distance: ");
  Serial.println(distance);
  if (distance < 20) {
    stopMotors();
    servo.write(0); delay(500); long leftDist = getUltrasonicDistance();
    Serial.print("Left distance: ");
    Serial.println(leftDist);
    servo.write(180); delay(500); long rightDist = getUltrasonicDistance();
    Serial.print("Right distance: ");
    Serial.println(rightDist);
    servo.write(90);
    if (leftDist > rightDist) turnLeft(); else turnRight();
  } else {
    int irLeft = digitalRead(IR_LEFT);
    int irRight = digitalRead(IR_RIGHT);
    Serial.print("IR Left: ");
    Serial.print(irLeft);
    Serial.print(" IR Right: ");
    Serial.println(irRight);
    if (irLeft == LOW) turnRight();
    else if (irRight == LOW) turnLeft();
    else moveForward();
  }
}

void ultrasonicScan() {
  Serial.println("Ultrasonic scan");
  servo.write(0); delay(500); long leftDist = getUltrasonicDistance();
  Serial.print("Left: ");
  Serial.println(leftDist);
  servo.write(90); delay(500); long centerDist = getUltrasonicDistance();
  Serial.print("Center: ");
  Serial.println(centerDist);
  servo.write(180); delay(500); long rightDist = getUltrasonicDistance();
  Serial.print("Right: ");
  Serial.println(rightDist);
  servo.write(90);
}

long getUltrasonicDistance() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2;
}