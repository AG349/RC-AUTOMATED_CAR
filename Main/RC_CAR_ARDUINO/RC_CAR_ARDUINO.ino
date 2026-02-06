#include <WiFiS3.h>
#include <WiFiUdp.h>
#include <Servo.h>

// WiFi AP settings
const char* ssid = "CarControl";
const char* password = "mypassword";
WiFiUDP udp;
const int udpPort = 4210;

// Motor pins (L298N)
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
unsigned long lastCommandTime = 0;
const unsigned long timeout = 10000;

void setup() {
  Serial.begin(115200);
  WiFi.beginAP(ssid, password);
  udp.begin(udpPort);
  delay(2000);
  Serial.println("WiFi AP started. IP: 192.168.4.1");
  
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
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char command = udp.read();  // Read single character to avoid corruption
    lastCommandTime = millis();
    Serial.print("Received: ");
    Serial.println(command);  // Should now show single letters like 'F'
    
    if (command == 'F') moveForward();
    else if (command == 'B') moveBackward();
    else if (command == 'L') turnLeft();
    else if (command == 'R') turnRight();
    else stopMotors();
  }
  
  if (millis() - lastCommandTime > timeout) {
    autonomousMode();
  }
  
  delay(100);
}

void moveForward() {
  Serial.println("Moving forward");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
  analogWrite(ENA1, 255); analogWrite(ENB1, 255);
  analogWrite(ENA2, 255); analogWrite(ENB2, 255);
}

void moveBackward() {
  Serial.println("Moving backward");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
  analogWrite(ENA1, 255); analogWrite(ENB1, 255);
  analogWrite(ENA2, 255); analogWrite(ENB2, 255);
}

void turnLeft() {
  Serial.println("Turning left");
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  digitalWrite(IN5, HIGH); digitalWrite(IN6, LOW);
  digitalWrite(IN7, HIGH); digitalWrite(IN8, LOW);
  analogWrite(ENA1, 200); analogWrite(ENB1, 200);
  analogWrite(ENA2, 200); analogWrite(ENB2, 200);
}

void turnRight() {
  Serial.println("Turning right");
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  digitalWrite(IN5, LOW); digitalWrite(IN6, HIGH);
  digitalWrite(IN7, LOW); digitalWrite(IN8, HIGH);
  analogWrite(ENA1, 200); analogWrite(ENB1, 200);
  analogWrite(ENA2, 200); analogWrite(ENB2, 200);
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
  Serial.println("Autonomous mode");
  long distance = getUltrasonicDistance();
  if (distance < 20) {
    stopMotors();
    servo.write(0); delay(500); long leftDist = getUltrasonicDistance();
    servo.write(180); delay(500); long rightDist = getUltrasonicDistance();
    servo.write(90);
    if (leftDist > rightDist) turnLeft(); else turnRight();
  } else {
    if (digitalRead(IR_LEFT) == LOW) turnRight();
    else if (digitalRead(IR_RIGHT) == LOW) turnLeft();
    else moveForward();
  }
}

long getUltrasonicDistance() {
  digitalWrite(TRIG, LOW); delayMicroseconds(2);
  digitalWrite(TRIG, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duration = pulseIn(ECHO, HIGH);
  return duration * 0.034 / 2;
}