# Cube and Beam Control System

This repository contains the implementation of a vision-based control system for a sliding cube on a rail, actuated by a Universal Robots UR3e collaborative robot. It is an industrial adaptation of the classic "Ball and Beam" control problem.

## System Architecture

The project utilizes a Master/Slave architecture communicating via Ethernet TCP/IP:

1. **Master (Raspberry Pi & Camera):** Runs a Python server (`server.py`) that captures the video feed, detects the cube's position using an ArUco marker, calculates the position error, and computes the required compensation angle using a Proportional-Derivative (PD) controller.
2. **Slave (UR3e Robot):** Operates as a pure actuator. A lightweight Polyscope script listens to the TCP socket and applies the requested angle to its Wrist 3 joint using the `servoj` command for real-time position tracking.

## Hardware Requirements

* Universal Robots UR3e
* Raspberry Pi (or PC) with an Ethernet port
* USB Webcam or Pi Camera
* Custom rail mounted on the UR3e end-effector
* Sliding cube with an ArUco Marker (Dictionary: `DICT_4X4_50`)

## Software Dependencies

The vision server requires Python 3. Install the required libraries:

```bash
pip install opencv-contrib-python numpy