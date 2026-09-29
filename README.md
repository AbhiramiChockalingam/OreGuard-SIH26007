# OreGuard-SIH26007
An intelligent safety system for mine vehicles that helps detect hazards and improve driver awareness in foggy and low-visibility conditions.
Intelligent Safety System for Mine Vehicles

Real-time hazard detection and driver alerts for mine vehicles in fog, dust, and low-visibility conditions.

The proposed system combines Radar, MPU6500, LoRa, DGPS, ESP32, IoT technology, and a centralized monitoring server to provide real-time information about vehicle position, vehicle condition, nearby vehicles/obstacles, and potential collision situations.

📌 Problem Statement

Open-cast mining operations involve large haul trucks, dumpers, excavators, and other heavy vehicles working simultaneously in the same mining area.

During:

🌫️ Fog
💨 Dust
🌧️ Heavy rain
🌙 Low-visibility conditions
⛰️ Blind corners
🚧 Restricted-visibility zones

Drivers may have difficulty detecting nearby vehicles, obstacles, and hazardous vehicle conditions.

This can increase the possibility of:

Vehicle-to-vehicle collisions
Collision with obstacles
Unsafe vehicle movement
Excessive vehicle tilt
Delayed driver response
Accidents in blind corners and low-visibility areas

The proposed system provides an additional vehicle safety and centralized monitoring layer for mining operations.

💡 Proposed Solution

The system consists of two major levels:

1. Vehicle-Level Safety System

Each mining vehicle is equipped with:

ESP32
Radar sensor
MPU6500
DGPS tracker
LoRa communication module

The vehicle continuously monitors:

Nearby vehicles and obstacles
Distance
Relative movement
Vehicle acceleration
Vehicle motion
Vehicle tilt
Vehicle safety condition
GPS/DGPS position

The information is transmitted from the vehicle to the mine infrastructure using LoRa communication.
<img width="1020" height="768" alt="image" src="https://github.com/user-attachments/assets/56465ed1-fcec-4a83-9d8b-8b26bb8d8559" />


2. Infrastructure-Level Monitoring System

Infrastructure nodes installed at suitable locations within the mining area receive information from vehicles through LoRa.

The infrastructure forwards the collected information to the central server using IoT technology.

The server provides a real-time monitoring interface showing:

Vehicle locations
Vehicle IDs
Vehicle movement
Vehicle safety status
Dangerous vehicles
Collision warnings
Vehicle condition
Other relevant telemetry

This allows the mine control/monitoring system to maintain a real-time overview of vehicle operations.

🏗️ Overall System Architecture
                    MINING VEHICLE
                         │
          ┌──────────────┼──────────────┐
          │              │              │
          ▼              ▼              ▼
       RADAR          MPU6500          DGPS
          │              │              │
          │              │              │
          └──────────────┼──────────────┘
                         │
                         ▼
                       ESP32
                         │
                         ▼
                      LoRa
                         │
                         │
                         ▼
              ┌─────────────────────┐
              │   MINE             │
              │ INFRASTRUCTURE     │
              │                     │
              │ LoRa Receiver       │
              └─────────┬───────────┘
                        │
                        │ IoT
                        ▼
              ┌─────────────────────┐
              │   CENTRAL SERVER    │
              │                     │
              │ Database            │
              │ Processing          │
              │ Monitoring          │
              └─────────┬───────────┘
                        │
                        ▼
              ┌─────────────────────┐
              │ REAL-TIME DASHBOARD │
              │                     │
              │ Vehicle Position    │
              │ Vehicle Condition   │
              │ Danger Alerts       │
              │ Collision Alerts    │
              └─────────────────────┘
🚛 Vehicle-to-Infrastructure Architecture

Each vehicle acts as a mobile sensing node.

Vehicle 1 ─────┐
               │
Vehicle 2 ─────┤
               │
Vehicle 3 ─────┤──► LoRa ──► Infrastructure
               │
Vehicle 4 ─────┤
               │
Vehicle N ─────┘

The infrastructure receives the information transmitted by vehicles and forwards it to the server through the available IoT communication network.

📡 Communication Architecture

The final system replaces the prototype's ESP-NOW communication with LoRa-based vehicle-to-infrastructure communication.

Prototype
Vehicle ESP32
     │
     ▼
  ESP-NOW
     │
     ▼
Other ESP32
     │
     ▼
Dashboard
Intended Real-Time System
Vehicle
   │
   ▼
LoRa Module
   │
   ▼
Mine Infrastructure
   │
   ▼
IoT Communication
   │
   ▼
Central Server
   │
   ▼
Monitoring Dashboard

LoRa is intended to provide long-range communication between mining vehicles and infrastructure nodes without requiring conventional Wi-Fi coverage throughout the mining area.

📡 LoRa Communication

LoRa modules will be used for transmitting vehicle information to infrastructure nodes.

The vehicle can periodically transmit information such as:

Vehicle ID
DGPS Position
Speed
Acceleration
Direction
Tilt Status
Vehicle Condition
Obstacle Information
Collision Risk
Danger Status
Timestamp

The infrastructure receives these packets and forwards the information to the server through an IoT communication layer.

🌐 IoT-Based Server Monitoring

The infrastructure acts as the communication bridge between the mining vehicles and the central server.

VEHICLE
   │
   │ LoRa
   ▼
INFRASTRUCTURE
   │
   │ IoT
   ▼
SERVER
   │
   ▼
DATABASE
   │
   ▼
REAL-TIME DASHBOARD

The server can maintain information about all vehicles operating within the mining area.

The dashboard can display:

Vehicle ID
Vehicle position
Vehicle status
Vehicle movement
Vehicle condition
Collision warning
Danger status
Last received data
Communication status
📍 DGPS-Based Vehicle Tracking

Each mining vehicle will be equipped with a DGPS tracker.

The DGPS system provides more accurate positioning than conventional GPS and allows the infrastructure/server to determine the real-time position of vehicles within the mining area.

The dashboard can display vehicles on a digital mine map.

Example:

                 MINE MAP

        ┌─────────────────────────────┐
        │                             │
        │      🚛 Vehicle 01          │
        │             │               │
        │             │               │
        │       🚛 Vehicle 02         │
        │                             │
        │                    🚛 V03   │
        │                             │
        │   ⚠ Vehicle 04              │
        │                             │
        └─────────────────────────────┘

The system can associate each DGPS position with the corresponding vehicle ID and safety condition.

🚨 Real-Time Vehicle Condition Monitoring

The monitoring system does not only display vehicle location.

It also displays the condition of each vehicle.

For example:

┌─────────────────────────────────────┐
│ Vehicle ID: TRUCK-04                │
│                                     │
│ Position: 11.1234, 79.1234          │
│ Speed: 18 km/h                      │
│                                     │
│ Vehicle Condition: ⚠ DANGER         │
│                                     │
│ Excessive Tilt: YES                 │
│ Obstacle Detected: YES              │
│ Collision Risk: HIGH                │
└─────────────────────────────────────┘

The system can therefore provide both:

Location Information
WHERE is the vehicle?

and

Safety Information
WHAT is happening to the vehicle?
📡 Radar-Based Detection
Replacement for HC-SR04 and RSSI

In the prototype, two different approaches are used for testing:

HC-SR04

Used for:

Distance measurement
Object detection
Closing-speed calculation
TTC calculation
RSSI

Used as a prototype method for estimating communication-based distance between ESP32 nodes.

However, these are not intended to be the final sensing mechanisms.

For the real-time implementation:

HC-SR04 ──────┐
              ├──► REPLACED BY RADAR
RSSI ─────────┘

The intended system will use Radar as the primary means of detecting nearby vehicles/objects and determining relevant range information.

📡 Why Radar?

Mining environments can contain:

Dense dust
Fog
Rain
Low visibility
Poor lighting
Night-time operation

Radar can operate independently of visible-light conditions and is therefore being considered for the real-time vehicle sensing system.

The final radar selection would depend on required:

Detection range
Field of view
Target classification capability
Accuracy
Update rate
Environmental protection
Mining-site operating requirements
🔄 Prototype vs Real-Time Implementation

One of the key design principles of this project is separating the proof-of-concept hardware from the intended deployment hardware.

Function	Prototype	Real-Time Implementation
Obstacle/Vehicle Detection	HC-SR04	Radar
Distance Estimation	HC-SR04	Radar
Communication	ESP-NOW	LoRa
Communication-based Distance	RSSI	Radar-based sensing
Vehicle Position	Prototype/optional	DGPS
Vehicle Condition	MPU6500	MPU6500 + additional vehicle data
Monitoring	Tkinter Dashboard	IoT Server Dashboard
Important

The current prototype uses HC-SR04 and ESP-NOW because they are practical for demonstrating the core functionality.

The intended real-time architecture replaces:

HC-SR04 → Radar
ESP-NOW → LoRa
RSSI-based distance → Radar
Local Dashboard → IoT Server Dashboard
🔧 Hardware Components
Component	Purpose
ESP32	Vehicle-side processing
MPU6500	Acceleration, gyroscope and vehicle motion monitoring
Radar Sensor	Real-time obstacle/vehicle detection
LoRa Module	Long-range vehicle-to-infrastructure communication
DGPS Tracker	Accurate vehicle positioning
Infrastructure LoRa Receiver	Receives vehicle telemetry
Server	Centralized data processing and storage
IoT Gateway/Communication	Infrastructure-to-server communication
Prototype Hardware
Component	Purpose
HC-SR04	Prototype obstacle detection
ESP-NOW	Prototype vehicle-to-vehicle communication
RSSI	Prototype communication-distance estimation
Tkinter	Prototype monitoring dashboard
📐 MPU6500

The MPU6500 is used to monitor vehicle motion.

It provides:

Accelerometer

Measures acceleration along:

X-axis
Y-axis
Z-axis

The overall acceleration magnitude can be calculated as:

Net Acceleration = √(Ax² + Ay² + Az²)
Gyroscope

Measures angular velocity around:

X-axis
Y-axis
Z-axis

This information can be used to identify abnormal vehicle movement and excessive tilting.

📏 HC-SR04 – Prototype Only

The HC-SR04 is currently used only for prototype testing.

It validates:

Object detection
Distance measurement
Closing-speed estimation
Time-to-Collision calculation
Collision warning logic

It will not be the intended sensing technology for the final mining implementation.

The final implementation is planned to use radar.

⏱️ Time to Collision (TTC)

Time to Collision can be used to estimate the remaining time before a potential collision under the current relative-motion conditions.

A simplified calculation is:

TTC = Distance / Closing Speed

The prototype uses configurable TTC warning levels.

Safe
   ↓
Caution
   ↓
Warning
   ↓
Critical

In the real-time implementation, radar measurements can provide the distance and relative-motion information required by the collision-warning algorithm.

🧠 Vehicle Safety Processing

The vehicle-side system can process multiple parameters.

                 SENSOR DATA
                      │
       ┌──────────────┼──────────────┐
       │              │              │
       ▼              ▼              ▼
     Radar         MPU6500          DGPS
       │              │              │
       ▼              ▼              ▼
 Object/Vehicle    Motion/Tilt     Position
 Detection         Monitoring      Tracking
       │              │              │
       └──────────────┼──────────────┘
                      │
                      ▼
               Safety Processing
                      │
          ┌───────────┼───────────┐
          │           │           │
          ▼           ▼           ▼
       Collision     Vehicle     Position
        Risk         Condition   Information
          │           │           │
          └───────────┼───────────┘
                      │
                      ▼
                    LoRa
                      │
                      ▼
               Infrastructure
🚨 Danger Detection

The system can identify potentially dangerous conditions such as:

Excessive vehicle tilt
Nearby vehicle/obstacle
Rapid closing movement
Low TTC
Abnormal vehicle motion
Other configured safety conditions

The danger status is transmitted along with the vehicle's position.

Therefore, the server can display:

Vehicle Location + Vehicle Condition

instead of displaying location alone.
