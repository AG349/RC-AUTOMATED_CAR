#include <WiFi.h>
#include <WiFiUdp.h>
#include <MPU6050.h>
#include <Wire.h>

MPU6050 mpu;

const char* ssid = "CarControl";
const char* password = "mypassword";  // Match Arduino password
WiFiUDP udp;
const char* serverIP = "192.168.4.1";
const int udpPort = 4210;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("Connected to Car WiFi");
  udp.begin(udpPort);
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  Serial.print("AX: ");
  Serial.print(ax);
  Serial.print(" AY: ");
  Serial.println(ay);  // Debug: Print tilt values
  
  String command = "S";
  if (ax < -5000) command = "F";
  else if (ax > 5000) command = "B";
  else if (ay > 5000) command = "R";
  else if (ay < -5000) command = "L";
  
  Serial.println("Sending Command: " + command);  // Debug: Show sent command
  
  udp.beginPacket(serverIP, udpPort);
  udp.print(command);
  udp.endPacket();
  
  delay(200);
}