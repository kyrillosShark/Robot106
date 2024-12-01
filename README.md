# Autonomous Robot Control System

![Robot Image](path/to/robot_image.jpg)
*Figure 1: The Autonomous Robot*

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation and Setup](#installation-and-setup)
- [Usage Instructions](#usage-instructions)
- [Components](#components)
  - [Central Script (`central_script.py`)](#central-script-central_scriptpy)
  - [Face Tracking (`face_tracking.py`)](#face-tracking-face_trackingpy)
  - [IMU Integration (`IMU.py`)](#imu-integration-imupy)
  - [Auto Navigation (`auto_navigation.py`)](#auto-navigation-auto_navigationpy)
- [Project Structure](#project-structure)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This project is an autonomous robot control system designed for navigation, object tracking, and obstacle avoidance. It integrates various sensors and technologies to enable the robot to operate in multiple modes, including manual control, autonomous navigation, and face tracking.

The system is built using Python and utilizes MQTT for inter-process communication, Flask for the web dashboard, OpenCV for computer vision tasks, and Leaflet.js for interactive mapping on the web interface.


*Figure 2: The Control Dashboard*
<img width="1439" alt="Screenshot 2024-11-26 at 7 34 24 PM" src="https://github.com/user-attachments/assets/9e6d130e-6159-4074-8b02-d2a96e86977b">

## Features

- **Multiple Modes of Operation**:
  - **Basic Movement**: Manual control through a web interface.
  - **Auto-Navigation**: Autonomous path following using GPS waypoints.
  - **Face Tracking**: The robot detects and follows human faces.
  - **Road Tracking**: Follows predefined roads or paths.
- **Real-Time Monitoring**:
  - Live video feed from the robot's camera.
  - Real-time GPS tracking on an interactive map.
  - Display of sensor data such as IMU readings.
- **Dynamic Control Adjustments**:
  - Adjustable PID controller parameters for precise control.
  - Face detection settings to modify tracking behavior.
- **Safety Mechanisms**:
  - Immediate emergency stop functionality.
  - Manual override controls.
- **Extensible Architecture**:
  - Modular codebase allowing easy addition of new features.
  - Separate processes for different functionalities to optimize performance.

## System Architecture

### Overview

The system is designed with a modular architecture, separating concerns into different processes and components that communicate via queues and MQTT. This ensures scalability, maintainability, and efficient resource utilization.

### Components

- **Central Script (`central_script.py`)**:
  - Acts as the orchestrator for the entire system.
  - Manages MQTT communication with the robot's hardware.
  - Runs the Flask web server providing the control dashboard.
  - Handles different operational modes and process management.
- **Face Tracking Process (`face_tracking.py`)**:
  - Dedicated to detecting and tracking human faces using OpenCV.
  - Implements PID control to adjust the robot's movement based on face position and size.
  - Communicates with the central script via queues.
- **Auto Navigation Process (`auto_navigation.py`)**:
  - Handles autonomous navigation to specified waypoints.
  - Utilizes GPS data and IMU readings to calculate heading and steering angles.
  - Implements Pure Pursuit algorithm for path following.
- **IMU Integration (`IMU.py`)**:
  - Interfaces with the BerryIMU sensor module.
  - Provides functions to read accelerometer, gyroscope, and magnetometer data.
  - Calculates the robot's heading based on sensor data.
- **Web Dashboard**:
  - Built with Flask and rendered using HTML, CSS, and JavaScript.
  - Provides controls for mode selection, movement commands, and PID adjustments.
  - Displays live video feed and interactive map with the robot's position.

### Communication Flow

- **MQTT**:
  - Used for communication between the central script and the robot's hardware.
  - Topics are defined for commands, camera feed, detections, and IMU data.
- **Inter-Process Communication**:
  - Queues and events are used for communication between the central script and subprocesses (face tracking, auto navigation).
  - Commands and data are passed efficiently without blocking the main thread.

## Hardware Requirements

- **Computing Unit**: Raspberry Pi 3 or newer (or compatible SBC with GPIO and camera support).
- **Camera Module**: Raspberry Pi Camera Module or compatible.
- **GPS Module**: For location tracking and navigation.
- **IMU Sensor**: BerryIMU v1/v2/v3 for orientation data.
- **Motors and Controllers**: DC motors with motor drivers capable of PWM control.
- **Power Supply**: Adequate power supply for the Raspberry Pi and peripherals.
- **Connectivity**: Wi-Fi dongle or Ethernet for network communication.

## Software Requirements

- **Operating System**: Raspbian Buster or newer recommended.
- **Python 3.7 or higher**
- **Python Libraries**:
  - `opencv-python`
  - `numpy`
  - `paho-mqtt`
  - `flask`
  - `gpsd-py3`
  - `smbus`
  - `gps3`
  - `logging`
  - `multiprocessing`
  - `threading`
  - `time`
  - `math`
  - `json`
  - `base64`
- **MQTT Broker**:
  - Mosquitto MQTT broker installed on the network.
  - Alternatively, the broker can be installed on the Raspberry Pi.

## Installation and Setup

### 1. Prepare the Raspberry Pi

- Install Raspbian OS on your Raspberry Pi.
- Update the system packages:

  ```bash
  sudo apt-get update
  sudo apt-get upgrade
  ```

- Enable the camera module:

  ```bash
  sudo raspi-config
  ```

  - Navigate to `Interfacing Options` -> `Camera` -> Enable.

- Install required system packages:

  ```bash
  sudo apt-get install python3-pip python3-smbus i2c-tools gpsd gpsd-clients
  ```

### 2. Clone the Repository

```bash
git clone https://github.com/yourusername/robot-control-system.git
cd robot-control-system
```

### 3. Install Python Dependencies

```bash
pip3 install -r requirements.txt
```

### 4. Configure Hardware

- **Camera**: Connect the camera module to the Raspberry Pi's camera interface.
- **IMU Sensor**: Wire the BerryIMU according to the manufacturer's instructions.
- **GPS Module**: Connect the GPS module via serial or USB port.
- **Motors and Controllers**: Connect the motor drivers to the Raspberry Pi's GPIO pins.

### 5. Configure MQTT Broker

- Install Mosquitto broker on the Raspberry Pi or another machine on the network:

  ```bash
  sudo apt-get install mosquitto mosquitto-clients
  ```

- Edit the `MQTT_SERVER` variable in `central_script.py` to point to the MQTT broker's IP address.

### 6. Test Sensors

- Verify that the IMU and GPS modules are functioning correctly using test scripts or tools like `i2cdetect` and `gpsmon`.

### 7. Run the Application

```bash
python3 central_script.py
```

- The application will start the Flask web server and other necessary processes.
- Access the web dashboard by navigating to `http://<raspberry_pi_ip_address>:5000` in a web browser.

## Usage Instructions

### Web Dashboard Overview

- **Video Feed**: Displays the real-time camera feed from the robot.
- **Mode Selection**: Choose between Basic Movement, Auto-Navigation, Face Tracking, and Road Tracking.
- **Controls**:
  - **Movement Controls**: Buttons for manual control (Forward, Backward, Left, Right, Stop).
  - **Emergency Controls**: E-Stop to immediately halt the robot; Resume to continue operation.
- **Face Detection Settings**:
  - Adjust the desired face area and center offset to modify tracking behavior.
- **PID Controller Settings**:
  - Modify `Kp`, `Ki`, and `Kd` parameters for the PID controller and update them in real-time.

### Operating Modes

#### 1. Basic Movement

- Use the movement control buttons to manually navigate the robot.

#### 2. Auto-Navigation

- Select "Auto-Navigation" mode.
- Use the map to draw a path:
  - Click on the map to create waypoints.
  - Use the drawing tools to create lines or polygons representing the desired path.
- Click "Send Command" to upload the coordinates to the robot.
- The robot will autonomously navigate the specified path.

#### 3. Face Tracking

- Select "Face Tracking" mode.
- The robot will automatically detect faces in its field of view and adjust its movement to follow.
- Adjust the face detection settings and PID parameters for optimal performance.

#### 4. Road Tracking

- Select "Road Tracking" mode.
- Similar to auto-navigation but designed to follow predefined road paths.

### Emergency Stop

- Press the "E-Stop" button to immediately stop all robot movements.
- Press "Resume" to continue operation after resolving any issues.

### Adjusting PID Parameters

- Navigate to the PID Controller section.
- Enter new values for `Kp`, `Ki`, and `Kd`.
- Click "Update PID" to apply the new parameters.

## Components

### Central Script (`central_script.py`)

The central hub of the application, responsible for:

- Managing MQTT communication with the robot hardware.
- Running the Flask web server for the dashboard.
- Handling user inputs and commands from the web interface.
- Starting and stopping subprocesses for different modes.
- Reading sensor data from the IMU and GPS modules.

### Face Tracking (`face_tracking.py`)

Handles face detection and tracking using OpenCV:

- Processes video frames to detect faces using Haar cascades or deep learning models.
- Calculates errors based on the position and size of detected faces.
- Implements PID control to adjust the robot's movement commands.
- Communicates with the central script via queues.

### IMU Integration (`IMU.py`)

Interfaces with the BerryIMU sensor:

- Reads accelerometer, gyroscope, and magnetometer data.
- Calculates the robot's heading and orientation.
- Provides functions for sensor initialization and data retrieval.

### Auto Navigation (`auto_navigation.py`)

Manages autonomous navigation to waypoints:

- Uses GPS data to determine the robot's current position.
- Calculates target headings and steering angles.
- Implements path planning algorithms like Pure Pursuit.
- Sends movement commands based on calculated trajectories.

## Project Structure

```plaintext
robot-control-system/
├── central_script.py          # Main application script
├── face_tracking.py           # Face tracking process
├── auto_navigation.py         # Auto navigation process
├── IMU.py                     # IMU sensor interface
├── templates/
│   └── index.html             # Web dashboard HTML template
├── static/
│   ├── css/
│   │   └── styles.css         # CSS stylesheets
│   └── js/
│       └── main.js            # JavaScript files
├── requirements.txt           # Python dependencies
├── README.md                  # Project documentation
├── LICENSE                    # License file
```

## Contributing

We welcome contributions from the community!

### How to Contribute

1. **Fork the Repository**:

   Click the "Fork" button on the top right to create a personal copy of the repository.

2. **Clone Your Fork**:

   ```bash
   git clone https://github.com/yourusername/robot-control-system.git
   cd robot-control-system
   ```

3. **Create a Branch**:

   ```bash
   git checkout -b feature-or-bugfix-name
   ```

4. **Make Your Changes**:

   - Add new features or fix bugs.
   - Ensure your code follows the project's style guidelines.
   - Write tests if applicable.

5. **Commit and Push**:

   ```bash
   git add .
   git commit -m "Description of your changes"
   git push origin feature-or-bugfix-name
   ```

6. **Submit a Pull Request**:

   - Go to the original repository.
   - Click on "New Pull Request".
   - Provide a clear description of your changes.

### Code Style Guidelines

- Use descriptive variable and function names.
- Write comments where necessary to explain complex logic.
- Follow PEP 8 style guidelines for Python code.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more information.

