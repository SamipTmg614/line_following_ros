# Line Following Robot with micro-ROS and ROS 2

A simple line-following robot built using **ESP32**, **IR sensors**, and **micro-ROS** to learn the fundamentals of the ROS 2 ecosystem. The robot follows a black line using IR sensors while communicating with ROS 2 through micro-ROS running on the ESP32.

This project was created as a hands-on learning platform for understanding:

- ROS 2 Nodes
- Topics
- Publishers and Subscribers
- micro-ROS on ESP32
- Embedded-to-ROS communication
- Differential drive robot control

micro-ROS enables microcontrollers such as the ESP32 to participate directly in a ROS 2 network by publishing and subscribing to ROS topics. :contentReference[oaicite:0]{index=0}

---

## Project Overview

The robot uses IR sensors to detect the line and controls two DC motors through a motor driver.

The ESP32:

- Reads IR sensor values
- Determines steering direction
- Controls motor speeds
- Publishes sensor and robot state information to ROS 2 using micro-ROS
- Receives commands from ROS 2 (optional future extension)

This project focuses on learning ROS communication concepts rather than implementing advanced control algorithms.

---

## Hardware Used

- ESP32 Development Board
- IR Line Tracking Sensor Module
- L298N / Similar Motor Driver
- 2 × DC Gear Motors
- Robot Chassis
- Battery Pack
- Jumper Wires

---

## Software Stack

| Component | Purpose |
|------------|----------|
| ESP32 | Robot Controller |
| Arduino IDE / PlatformIO | Firmware Development |
| micro-ROS | ROS 2 Communication |
| ROS 2 | Robot Middleware |
| Ubuntu | ROS Development Environment |

---

## ROS 2 Concepts Learned

This project demonstrates the following ROS 2 concepts:

### Nodes

- ESP32 micro-ROS Node
- ROS 2 Monitoring Node

### Topics

Example topics:

/ir_sensor
/line_state
/cmd_vel





Publishers

The ESP32 publishes:

IR sensor readings
Robot state
Debug information
Subscribers

The ESP32 can subscribe to:

Velocity commands
Start/Stop commands
Configuration parameters

System Architecture
+-------------------+
|   IR Sensors      |
+---------+---------+
          |
          v
+-------------------+
|      ESP32        |
|   micro-ROS Node  |
+---------+---------+
          |
          |
   ROS 2 Topics
          |
          v
+-------------------+
|    ROS 2 PC       |
|  Subscriber Node  |
+-------------------+
