# autonomous-rover
Central Flask backend for the Autonomous Rover. Runs on Raspberry Pi 5 to control motors via Arduino, stream LiDAR maps (ROS2 + Hector SLAM), and serve ESP32-CAM video to the Flutter app. Handles mode switching, real-time telemetry, and system commands,acting as the rover’s intelligent command hub.


🛰️ app.py — The Central Intelligence of the Autonomous Rover

This is the core backend controller that unifies the rover’s entire ecosystem — motor control, LiDAR mapping, video streaming, and communication with the Flutter app. Built on Flask and running on the Raspberry Pi 5, app.py acts as the mission control hub where every subsystem reports, coordinates, and executes.

⚙️ Features

Unified Flask Server: Handles REST endpoints for remote commands and data exchange.

Motor Control Interface: Communicates with Arduino UNO via serial for precise drive commands.

LiDAR Map Streaming: Integrates ROS2 + Hector SLAM for real-time 2D environment mapping.

ESP32-CAM Video Feed: Serves live video stream from onboard camera.

System Monitor: Displays battery voltage and motor speed data in real time.

Modular Design: Built for future upgrades — SLAM visualization, object detection, and autonomous navigation logic.

🧠 Tech Stack

Backend: Flask (Python 3)

Hardware: Raspberry Pi 5, Arduino UNO, RPLiDAR A1, ESP32-CAM

Communication: HTTP + Serial + ROS2 Topics

Frontend: Flutter mobile app (Remote Control & Data Dashboard)

🚀 Summary

Think of app.py as the rover’s Jarvis — the brain that listens, processes, and commands the machine in real time. Every byte of data, every wheel rotation, every obstacle avoidance routine — it all flows through this file.
