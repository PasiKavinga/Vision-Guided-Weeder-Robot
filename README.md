Vision-Guided Autonomous Weed Removal Robot
Deep Learning + Mechatronics–Powered Precision Weeding System

This repository contains all documentation, source code, hardware specifications, and experimental results for the Vision-Guided Autonomous Weeder Robot, developed as an academic research project.
The system integrates a YOLOv8 deep learning model, a Python-based controller, and an ESP32-driven 3-axis linear actuator mechanism to automatically detect, localize, and remove weeds.

- Project Overview

Weeds significantly impact crop productivity and require large labor inputs in Sri Lanka. This project introduces a low-cost automated weeding solution combining machine vision and mechatronic control to reduce dependency on manual labor and chemical herbicides.

The prototype robot can:

- Detect weeds in real time using a YOLOv8 model
- Convert pixel coordinates into physical X–Y positions
- Move a robotic arm using lead-screw linear actuators
- Lower a gripper using Z-axis limit switch feedback
- Physically uproot weeds and return to home position
- Operate in manual mode or fully automated mode

- System Architecture
- 
✔ Vision System
Model: YOLOv8 (Ultralytics)
- Training images: 500+ custom images + Kaggle dataset
- Evaluation: Precision, recall, confusion matrix
- Best performance: 93% detection accuracy

✔ Control Software
Language: Python
Libraries: Ultralytics YOLO, OpenCV, PySerial, PySimpleGUI
Tasks:
Model inference
GUI for operation & calibration
Serial command generation

✔ Embedded Firmware
Microcontroller: ESP32
Drivers: A4988 (X, Y, Z, gripper)
Implements movement primitives:
MOVE x y
LOWER
LIFT_CM
GRIP_CLOSE
GRIP_OPEN

- Key Results
- 
🔹 Travel Accuracy
Linear correlation between commanded and actual travel:
Y = 0.7931X – 0.061
Integrated into controller for improved precision.
🔹 Movement Speed
Actuator speed stabilized at ~1.08 cm/s for longer movements.
Smooth, repeatable motion using lead-screw mechanism.
🔹 Detection Accuracy
Precision–Confidence curve shows high precision at ≥0.5 confidence
Confusion matrix:
93% true weed detection rate
7% false negative
100% background precision
🔹 Weed Removal Performance
40 weeds tested
11 successfully removed
27.5% removal success
Average cycle: 42.96 s/weed

- Dataset

1488 training images
244 validation
244 testing
Fully annotated using LabelImg
Stored in YOLO format (train/, val/, labels/)

- Hardware Overview
Chassis: Steel frame, 4-wheel base
Actuators: 3 × NEMA17 + lead screws
Gripper: Rack-and-pinion mechanism
Limit switch: Z-axis ground detection
Power: 12V/5A + USB 5V for ESP32
Drivers: A4988 with tuned current limit

- Novelty of the System

This project is the first Sri Lankan low-cost, vision-guided robotic weeder built using custom datasets, real-time YOLOv8 detection, and a locally manufactured mechanical platform. It combines AI, mechatronics, and automation into an affordable prototype tailored for smallholder farms.

- Potential for Commercialization

With improvements such as stronger actuators, refined calibration, and field-grade durability, this system can evolve into a low-cost agricultural robot suitable for vegetable fields, nurseries, and organic farms — addressing labor shortages and reducing herbicide usage.

- Contribution to Human & Environmental Benefit

The system can:

Reduce farmer exposure to chemical herbicides
Minimize labor requirements for manual weeding
Lower soil contamination and protect biodiversity
Offer an affordable precision-agriculture tool for small farmers

- Scientific Principles Used
- 
Deep Learning (YOLOv8): Object detection, localization
Camera Calibration: Pixel-to-cm mapping
Kinematics: Linear actuator motion translation
Closed-Loop Control: Limit-switch–based Z-axis feedback
Mechatronics Integration: Sensing → Control → Actuation


@Copyright to Pasindu Kavinga
