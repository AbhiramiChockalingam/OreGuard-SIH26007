# OreGuard-SIH26007
An intelligent safety system for mine vehicles that helps detect hazards and improve driver awareness in foggy and low-visibility conditions.
Intelligent Safety System for Mine Vehicles

Real-time hazard detection and driver alerts for mine vehicles in fog, dust, and low-visibility conditions.

Overview

Fog and dust in mines reduce visibility and cause collisions with people, vehicles, and obstacles. This system enhances camera feeds, detects hazards, estimates distance, and warns the driver before an accident can happen.

Features
🌫️ Fog/haze removal for clearer camera feeds
🎯 Real-time detection of persons, vehicles, and obstacles
📏 Distance estimation using sensors and/or camera
🚨 Multi-level alerts (Safe / Caution / Danger) with buzzer, display, and voice
📝 Event logging for safety analysis
Tech Stack

Python · OpenCV · YOLO (PyTorch) · Arduino / Raspberry Pi · Ultrasonic / RADAR sensors

Project Structure
├── src/          # Source code (detection, dehazing, alerts)
├── models/       # Trained weights
├── data/         # Sample data
├── hardware/     # Arduino sensor code
├── config.yaml   # Thresholds and settings
└── requirements.txt
Installation
bash
git clone https://github.com/<your-username>/mine-vehicle-safety-system.git
cd mine-vehicle-safety-system
pip install -r requirements.txt
Usage
bash
python src/main.py --source 0 --dehaze
Alert Levels
Level	Distance	Action
🟢 Safe	> 20 m	None
🟡 Caution	10–20 m	On-screen warning
🔴 Danger	< 10 m	Buzzer and voice alert
Team

Member 1 · Member 2 · Member 3 · Member 4

License
