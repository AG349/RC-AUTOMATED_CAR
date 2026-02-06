🚗 RC-Automated Car using ESP32 & Arduino UNO

An RC car that doesn’t panic when it loses connection 😄
This project uses ESP32 as a wireless transmitter and Arduino UNO as a receiver, with a smart fallback system that enables autonomous driving using sensors when the wireless connection is lost.

✨ Key Features
📡 Wireless Control using ESP32
🔁 Fail-Safe Mode: Automatically switches to autonomous mode if connection is lost
🤖 Autonomous Navigation using sensors (Ultrasonic + Servo)
🎮 Gesture / Motion Control using MPU-6050
🧪 Separate testing codes to verify hardware before running the main program

🧠 How the System Works
ESP32 (Transmitter)
    Reads motion data from the MPU-6050
    Sends control commands wirelessly
Arduino UNO (Receiver)
    Controls motors and sensors
    Listens for ESP32 commands
    If wireless connection drops → switches to autonomous mode

📁 Project Structure
RC-AUTOMATED_CAR/
│
├── Test/
│   ├── Arduino_Test/
│   │   └── Sensor & Motor Testing Code
│   │
│   └── ESP32_MPU6050_Test/
│       └── MPU-6050 Gesture Testing Code
│
├── Main_Code/
│   ├── Arduino_Main_Code
│   └── ESP32_Main_Code
│
└── Connection_Details.xlsx

🧪 Testing Before Running Main Code (Very Important!)
🔧 1. Arduino UNO – Motor & Sensor Test
    Upload the test code from the Arduino Test folder.

Open Serial Monitor in Arduino IDE and use the following commands:

Command	Action
"F"	Move Forward
"B"	Move Backward
"R"	Turn Right
"L"	Turn Left
"S"	Stop
"U"	Ultrasonic Sensor / Servo Rotation
"A"	Autonomous Mode (Sensor-Based Movement)

✅ If all commands work correctly, your Arduino connections are fine.
❌ If something doesn’t work here, fix it before proceeding, or the main code may fail.

📐 2. ESP32 – MPU-6050 Test
    Upload the code from ESP32_MPU6050_Test folder.

Calibration Steps:
    Place the MPU-6050 on a flat surface
    Wait for calibration to complete
    Open Serial Monitor

Tilt Test Output:
Movement	Expected Output
Forward Tilt     "F"
Backward Tilt    "B"
Right Tilt	     "R"
Left Tilt	       "L"
Flat Surface	   "S"

❌ If values don’t change:
    Recheck wiring
    Check I²C connections
    The MPU-6050 module might be faulty

🚀 Running the Main Project
    Make sure both tests pass successfully
    
Upload the Main Code to:
    ESP32 (Transmitter)
    Arduino UNO (Receiver)

Follow wiring details from Components Connection.xlsx

Power on the car and enjoy 🎉

🛡️ Safety Logic (Fail-Safe Mode)
    If ESP32 connection is lost or interrupted
    Arduino automatically activates sensor-based autonomous driving
    Prevents uncontrolled movement or crashes

📌 Requirements
    ESP32
    Arduino UNO
    MPU-6050
    Ultrasonic Sensor
    Servo Motor
    Motor Driver
    DC Motors
    Power Supply
    Jumper Wires
