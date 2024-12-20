# Raspberry Pi Autonomous Vehicle Robot

This project is a Raspberry Pi-powered autonomous vehicle robot designed for lane following and dynamic command response using AprilTags. The robot leverages dual cameras for advanced functionality and multiple motors for smooth navigation, making it suitable for high-performance tasks.

## Features

- **Lane Following**: A camera detects lane markings and adjusts the robot's movement to stay within the lane.
- **AprilTag Detection**: A second camera scans for AprilTags and triggers dynamic actions:
  - **Stop**: Halts the robot upon detecting a specific tag.
  - **Slow Down**: Reduces speed based on the tag ID.
- **Dual Power Source**: Powered by two independent battery packs to ensure optimal performance for motors and processing.

---

## Hardware Components

- **Raspberry Pi 4**: Central processing unit for the robot.
- **Motors**: Four motors for enhanced maneuverability and power.
- **Cameras**: 
  - One for lane detection.
  - One for AprilTag recognition.
- **Motor Driver**: Cytron motor driver for efficient motor control.
- **Battery Packs**: Two battery packs, each holding four 18650 batteries, providing ample power for prolonged operation.

---

## Software

- **Python**: Primary programming language for the robot's functionality.
- **OpenCV**: For image processing tasks such as lane detection and AprilTag recognition.
- **AprilTag Library**: For decoding and responding to AprilTags.

---

## How It Works

### Lane Following:
1. The lane detection camera captures a real-time video feed.
2. Image processing algorithms identify lane markings.
3. The robot adjusts its movements to stay within the lane boundaries.

### AprilTag Detection:
1. The second camera scans the environment for AprilTags.
2. Upon detecting a tag, the system decodes its ID.
3. The corresponding action (e.g., stopping or slowing down) is executed.

---

## Getting Started

To replicate or build upon this project:
1. Set up the hardware components as listed above.
2. Install the necessary Python libraries:
   ```bash
   pip install opencv-python apriltag
