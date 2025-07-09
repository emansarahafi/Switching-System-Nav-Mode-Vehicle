# Switching System for Teleoperated Vehicle Navigation Modes

This project implements a sophisticated switching system for a teleoperated vehicle, allowing for dynamic transitions between manual and autonomous navigation modes. It leverages the CARLA simulator for realistic vehicle dynamics and sensor data, with a MATLAB-based control and diagnostics dashboard. The communication between the CARLA environment (managed by a Python script) and the MATLAB controller is handled via UDP.

## System Architecture

The system consists of two primary components:

1.  **CARLA Python Bridge (`carla_matlab_bridge.py`):** This script connects to a running CARLA server. It spawns the ego vehicle, attaches a suite of sensors (cameras, LiDAR, IMU, GNSS), collects and packages sensor data, and sends it to the MATLAB application via UDP. It also listens for control commands from MATLAB to drive the vehicle in autonomous mode.

2.  **MATLAB Diagnostics & Control (`carla_udp_receiver.m`):** This MATLAB application provides a comprehensive dashboard for real-time monitoring. It receives and decodes the data stream from CARLA, visualizes sensor feeds, runs the core decision-making logic, and sends control commands back to the Python bridge.

The decision to switch between modes is handled by a fuzzy logic system ([`fuzzy_bbna.m`](fuzzy_bbna.m)) and a state machine ([`ModeController.m`](ModeController.m)) within MATLAB, which considers factors like system health, driver readiness, and environmental complexity.

## Features

*   **Real-time Diagnostics Dashboard:** A MATLAB UI displays:
    *   Multiple camera feeds (Front, Rear, Left, Right, Interior).
    *   3D LiDAR point cloud visualization.
    *   Live plotting of IMU (accelerometer) data.
    *   GNSS trajectory mapping.
    *   Vehicle speed gauge.
    *   System status indicators (Control Mode, System Health, Latency).
*   **Dynamic Mode Switching:**
    *   **Manual Mode:** The vehicle is controlled by the user via keyboard input in the CARLA window.
    *   **Autonomous Mode:** MATLAB takes control based on its internal logic.
    *   **Safe Mode:** A fault-tolerant mode triggered by the FDIR system in case of critical failures.
*   **Fault Detection, Isolation, and Recovery (FDIR):** A background timer ([`run_fdir_cycle.m`](run_fdir_cycle.m)) continuously monitors the system for anomalies and can force the vehicle into a safe state.
*   **Fuzzy Logic Control:** A fuzzy inference system ([`fuzzy_bbna.m`](fuzzy_bbna.m)) assesses the desirability of switching to autonomous mode based on various inputs.
*   **Data Logging:** The system logs collision and lane invasion events to CSV files in the `data/logs/` directory.
*   **Image Recording:** On-demand saving of camera images to the `data/images/` directory.

## How to Run

1.  **Start CARLA:** Launch the CARLA simulator (version 0.9.13+ recommended).
2.  **Run the Python Bridge:** Execute the main simulation script.
    ```sh
    python carla_matlab_bridge.py
    ```
3.  **Run the MATLAB Dashboard:** Open MATLAB and run the receiver script.
    ```matlab
    carla_udp_receiver
    ```

## Key Files

*   [`carla_matlab_bridge.py`](carla_matlab_bridge.py): The main Python script that interfaces with CARLA and communicates with MATLAB.
*   [`carla_udp_receiver.m`](carla_udp_receiver.m): The main MATLAB script that runs the UI, processes data, and implements control logic.
*   [`ModeController.m`](ModeController.m): Contains the state machine for managing transitions between `MANUAL`, `AUTOPILOT`, and `AWAITING_CONFIRMATION` states.
*   [`fuzzy_bbna.m`](fuzzy_bbna.m): Implements the fuzzy logic system for decision-making.
*   [`run_fdir_cycle.m`](run_fdir_cycle.m): The script for the periodic FDIR checks.
*   [`get_safe.m`](get_safe.m): A utility function in MATLAB to safely access fields in a struct.

## Controls (in CARLA window)

*   **W/S:** Throttle / Brake
*   **A/D:** Steer Left / Right
*   **Space:** Hand Brake
*   **M:** Toggle MATLAB Control Link (Enable/Disable autonomous mode)
*   **L:** Unlock Safe Mode
*   **Y:** Confirm Remote Handover request from MATLAB
*   **R:** Toggle Image Recording
*   **Ctrl+R:** Toggle full simulation recording
*   **H:** Toggle Help Menu
