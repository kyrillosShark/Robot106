# central_script.py
import cv2
import numpy as np
import paho.mqtt.client as mqtt
import threading
import time
import json
import math
import sys
import base64
import gpsd
from flask import Flask, Response, jsonify, request, render_template_string
import logging
from multiprocessing import Process, Queue, Event
from face_tracking import face_tracking_process  # Ensure this module is available and updated
from auto_navigation import auto_navigation_process  # Import the auto_navigation_process
import IMU  # Importing the IMU module

# MQTT Configuration
MQTT_SERVER = "192.168.1.120"  # Update if different
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "robot/control"
MQTT_TOPIC_DETECTIONS = "robot/detections"
MQTT_TOPIC_CAMERA = "robot/camera"
MQTT_TOPIC_IMU = "imu/data"

app = Flask(__name__)

# Global variables
latest_detection = None
latest_camera_frame = None
output_frame = None
lock = threading.Lock()
e_stop_active = False  # E-Stop state

# GPS and heading data
current_lat, current_lon = None, None
robot_heading = 0.0
gps_data = []
gps_data_lock = threading.Lock()

# PID Controller Parameters
w, h = 640, 480  # Frame dimensions for visualization (can be adjusted)
center = w // 2

# Configuration Flags
ENABLE_FRAME_FLIP = True  # Set to False to disable frame flipping
INVERT_YAW_CONTROL = False  # Set to True if robot moves opposite to desired direction

# Mode Control
current_mode = 'basic_movement'  # Default mode

# Queues for inter-process communication
command_queue = Queue()
detection_queue = Queue()
camera_frame_queue = Queue()
gps_data_queue = Queue()
imu_queue = Queue()
movement_command_queue = Queue()  # New queue for movement commands

# Events to control processes
stop_event = Event()

# MQTT setup and functions
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"Connected to MQTT server {MQTT_SERVER} successfully.")
        client.subscribe([(MQTT_TOPIC_DETECTIONS, 0), (MQTT_TOPIC_CAMERA, 0)])
    else:
        print(f"Failed to connect to MQTT server, return code {rc}")

def on_message(client, userdata, msg):
    try:
        if msg.topic == MQTT_TOPIC_DETECTIONS:
            detection_data = json.loads(msg.payload.decode())
            detection_queue.put(detection_data)
        elif msg.topic == MQTT_TOPIC_CAMERA:
            camera_data = json.loads(msg.payload.decode())
            image_b64 = camera_data.get('image', '')
            if image_b64:
                image_bytes = base64.b64decode(image_b64)
                np_arr = np.frombuffer(image_bytes, np.uint8)
                image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                if image is not None:
                    camera_frame_queue.put(image)
                else:
                    print("Failed to decode camera image.")
            else:
                print("No image data found in camera message.")
    except Exception as e:
        print(f"Error handling message on topic {msg.topic}: {e}")

# Initialize MQTT Client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(MQTT_SERVER, MQTT_PORT, 60)
client.loop_start()

# Initialize IMU
IMU.detectIMU()
if IMU.BerryIMUversion == 99:
    print("No BerryIMU found... exiting")
    sys.exit()
IMU.initIMU()

# Connect to GPSD
gpsd.connect()

def calculate_heading():
    """
    Calculate the robot's heading using IMU readings.
    Returns the heading in degrees.
    """
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    # Normalize accelerometer raw values.
    acc_magnitude = math.sqrt(ACCx ** 2 + ACCy ** 2 + ACCz ** 2)
    if acc_magnitude == 0:
        print("Error: Accelerometer magnitude is zero.")
        return 0
    accXnorm = ACCx / acc_magnitude
    accYnorm = ACCy / acc_magnitude

    pitch = math.asin(accXnorm)
    if math.cos(pitch) == 0:
        roll = 0
    else:
        roll = -math.asin(accYnorm / math.cos(pitch))

    # Tilt compensation
    magXcomp = MAGx * math.cos(pitch) + MAGz * math.sin(pitch)
    magYcomp = (MAGx * math.sin(roll) * math.sin(pitch) +
               MAGy * math.cos(roll) -
               MAGz * math.sin(roll) * math.cos(pitch))

    heading = math.degrees(math.atan2(magYcomp, magXcomp))
    if heading < 0:
        heading += 360

    return heading

def receive_gps_data():
    global current_lat, current_lon
    try:
        packet = gpsd.get_current()
        if packet.mode >= 2:
            current_lat = packet.lat
            current_lon = packet.lon
            return current_lat, current_lon
        else:
            return None, None
    except Exception as e:
        print(f"Error retrieving GPS data: {e}")
        return None, None

def normalize_angle(angle):
    angle = angle % 360
    if angle > 180:
        angle -= 360
    return angle

def calculate_target_heading(current_lat, current_lon, target_lat, target_lon):
    delta_lon = math.radians(target_lon - current_lon)
    lat1 = math.radians(current_lat)
    lat2 = math.radians(target_lat)
    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))
    initial_bearing = math.atan2(x, y)
    initial_bearing = math.degrees(initial_bearing)
    compass_bearing = (initial_bearing + 360) % 360
    return compass_bearing

def find_lookahead_point(current_lat, current_lon, waypoints, lookahead_distance):
    # Convert degrees to radians for calculation
    current_pos = np.array([math.radians(current_lat), math.radians(current_lon)])

    for i in range(len(waypoints) - 1):
        start_wp = np.array([math.radians(waypoints[i][0]), math.radians(waypoints[i][1])])
        end_wp = np.array([math.radians(waypoints[i+1][0]), math.radians(waypoints[i+1][1])])

        # Calculate the vector from the current position to the waypoints
        d = end_wp - start_wp
        f = start_wp - current_pos
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - lookahead_distance**2
        discriminant = b**2 - 4 * a * c

        if discriminant < 0:
            continue  # No intersection

        discriminant = math.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)

        if 0 <= t1 <= 1:
            intersection = start_wp + t1 * d
            return [math.degrees(intersection[0]), math.degrees(intersection[1])]
        if 0 <= t2 <= 1:
            intersection = start_wp + t2 * d
            return [math.degrees(intersection[0]), math.degrees(intersection[1])]

    return None  # No valid lookahead point found

def calculate_steering_angle(current_lat, current_lon, current_heading, lookahead_point):
    # Calculate the angle between the robot's heading and the lookahead point
    target_heading = calculate_target_heading(current_lat, current_lon, lookahead_point[0], lookahead_point[1])
    steering_angle = normalize_angle(target_heading - current_heading)
    return steering_angle

def road_tracking_process(command_queue, movement_command_queue, stop_event):
    """
    Process for road tracking navigation mode.
    """
    gps_data = []
    path_coordinates = []
    waypoints = []
    e_stop_active = False

    while not stop_event.is_set():
        # Handle commands from the main process
        while not command_queue.empty():
            cmd, data = command_queue.get()
            if cmd == 'set_waypoints':
                waypoints = [(point['lat'], point['lng']) for point in data]
                print("Waypoints updated in road tracking process.")
            elif cmd == 'estop':
                e_stop_active = True
                print("E-Stop activated in road tracking process.")
            elif cmd == 'undo_estop':
                e_stop_active = False
                print("E-Stop deactivated in road tracking process.")
            elif cmd == 'stop':
                print("Stopping road tracking process.")
                stop_event.set()
                break

        if e_stop_active or not waypoints:
            time.sleep(0.1)
            continue

        current_lat, current_lon = receive_gps_data()
        if current_lat is not None and current_lon is not None:
            heading = calculate_heading()
            gps_data.append({
                'GPS_Lat': current_lat,
                'GPS_Lon': current_lon,
                'Heading': heading
            })
            path_coordinates.append([current_lat, current_lon])

            # Pure Pursuit logic
            lookahead_distance = 0.0001  # Adjust as necessary (in degrees)
            lookahead_point = find_lookahead_point(current_lat, current_lon, waypoints, lookahead_distance)

            if lookahead_point is None:
                print("Reached the end of the path.")
                movement_command_queue.put("64 64")  # Stop command
                break

            # Calculate steering angle to the lookahead point
            steering_angle = calculate_steering_angle(current_lat, current_lon, heading, lookahead_point)

            # Determine command based on the steering angle
            if steering_angle > 5:
                side_side_command = 126  # Turn left
            elif steering_angle < -5:
                side_side_command = 0    # Turn right
            else:
                side_side_command = 64   # Straight

            front_back_command = 100  # Move forward
            command_string = f"{front_back_command} {side_side_command}"
            movement_command_queue.put(command_string)
            print(f"Command: {command_string} | Steering Angle: {steering_angle}")
        else:
            print("No valid GPS data received.")

        time.sleep(0.1)

def main_loop():
    global latest_detection, latest_camera_frame
    global output_frame, lock, e_stop_active
    global current_lat, current_lon, robot_heading, gps_data
    global current_mode, auto_nav_proc, road_track_proc, stop_event

    # Initialize last_img with a black image
    last_img = np.zeros((h, w, 3), dtype=np.uint8)

    try:
        while True:
            # Update last_img if a new frame is available
            while not camera_frame_queue.empty():
                last_img = camera_frame_queue.get()

            # Use the last available frame
            img = last_img.copy()

            # Optionally flip the frame
            if ENABLE_FRAME_FLIP:
                img = cv2.flip(img, 1)  # Flip the image horizontally

            # Display mode on the frame
            cv2.putText(img, f"Mode: {current_mode}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Optionally display IMU heading
            imu_heading = calculate_heading()
            cv2.putText(img, f"IMU Heading: {imu_heading:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Optionally display E-Stop status
            if e_stop_active:
                cv2.putText(img, "E-STOP ACTIVE!", (10, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # Update GPS data
            current_lat, current_lon = receive_gps_data()
            if current_lat is not None and current_lon is not None:
                with gps_data_lock:
                    gps_data.append({
                        'GPS_Lat': current_lat,
                        'GPS_Lon': current_lon,
                        'Heading': imu_heading
                    })

            # Check for e-stop activation
            if e_stop_active:
                print("E-Stop is active. Stopping the robot.")
                front_back_command = 64  # Stop
                side_side_command = 64   # Neutral steering
                command_string = f"{front_back_command} {side_side_command}"
                client.publish(MQTT_TOPIC_COMMAND, command_string)
                with lock:
                    output_frame = img.copy()
                time.sleep(0.1)
                continue  # Skip the rest of the loop

            # Handle movement commands
            if current_mode == 'face_tracking':
                while not movement_command_queue.empty():
                    command_string = movement_command_queue.get()
                    client.publish(MQTT_TOPIC_COMMAND, command_string)
            elif current_mode == 'auto_navigation':
                # Handle movement commands from auto_navigation_process
                while not movement_command_queue.empty():
                    command_string = movement_command_queue.get()
                    client.publish(MQTT_TOPIC_COMMAND, command_string)
            elif current_mode == 'road_tracking':
                # Handle movement commands from road_tracking_process
                while not movement_command_queue.empty():
                    command_string = movement_command_queue.get()
                    client.publish(MQTT_TOPIC_COMMAND, command_string)
            elif current_mode == 'basic_movement':
                # Manual movement commands are handled via Flask routes
                pass
            else:
                # Unknown mode; stop the robot for safety
                front_back_command = 64  # Stop
                side_side_command = 64   # Neutral steering
                command_string = f"{front_back_command} {side_side_command}"
                print("Unknown mode. Stopping the robot.")
                client.publish(MQTT_TOPIC_COMMAND, command_string)

            # Acquire lock to set the global output frame
            with lock:
                output_frame = img.copy()

            # Small sleep to reduce CPU usage
            time.sleep(0.05)

    except Exception as e:
        print(f"An error occurred: {e}")

    finally:
        # Cleanup
        front_back_command = 64  # Stop
        side_side_command = 64   # Neutral steering
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        client.loop_stop()
        client.disconnect()
        if auto_nav_proc is not None:
            stop_event.set()
            auto_nav_proc.join()
        if road_track_proc is not None:
            stop_event.set()
            road_track_proc.join()

# [Rest of the code remains the same, including Flask routes]

def generate():
    global output_frame, lock
    # Continuously generate frames
    while True:
        with lock:
            if output_frame is None:
                continue
            # Encode the frame in JPEG format
            (flag, encoded_image) = cv2.imencode(".jpg", output_frame)
            if not flag:
                continue
        # Yield the output frame in byte format
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
              bytearray(encoded_image) + b'\r\n')
        time.sleep(0.05)  # Adjust to control frame rate

# [HTML content omitted for brevity; same as before]
html_content='''
<!DOCTYPE html>
<html>
<head>
    <title>Autonomous Robot Control Dashboard</title>
    <meta charset="utf-8" />
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <!-- Leaflet CSS -->
    <link rel="stylesheet" href="https://unpkg.com/leaflet@1.7.1/dist/leaflet.css" />
    <!-- Leaflet Draw CSS -->
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.css" />
    <!-- Leaflet Rotated Marker CSS -->
    <link rel="stylesheet" href="https://rawcdn.githack.com/bbecquet/Leaflet.RotatedMarker/master/leaflet.rotatedMarker.css" />
    <style>
        :root {
            --primary-color: #ecf0f1;
            --secondary-color: #34495e;
            --accent-color: #e67e22;
            --danger-color: #e74c3c;
            --success-color: #2ecc71;
            --text-color: #ecf0f1;
            --bg-color: #2c3e50;
            --button-padding: 8px 16px;
            --font-size: 14px;
            --control-gap: 10px;
        }

        * {
            margin: 0;
            padding: 0;
            box-sizing: border-box;
        }

        body {
            font-family: "Segoe UI", Tahoma, Geneva, Verdana, sans-serif;
            background-color: var(--bg-color);
            color: var(--text-color);
            overflow: hidden;
        }

        .dashboard {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            grid-gap: 10px;
            padding: 10px;
            height: 100vh;
            width: 100vw;
        }

        .panel {
            background: var(--secondary-color);
            border-radius: 8px;
            box-shadow: 0 2px 4px rgba(0, 0, 0, 0.3);
            padding: 10px;
            display: flex;
            flex-direction: column;
            overflow: hidden;
        }

        .control-panel {
            grid-column: span 1;
            display: flex;
            flex-direction: column;
            gap: 10px;
        }

        .maps-container {
            grid-column: span 2;
            display: grid;
            grid-template-rows: 1fr auto;
            gap: 10px;
        }

        .maps-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 10px;
        }

        .header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 10px;
            border-bottom: 1px solid var(--primary-color);
        }

        .title {
            font-size: 20px;
            font-weight: 600;
            color: var(--primary-color);
        }

        .status {
            padding: 4px 8px;
            border-radius: 15px;
            background: var(--success-color);
            color: white;
            font-size: 12px;
        }

        #video {
            width: 100%;
            height: 180px;
            border-radius: 6px;
            object-fit: cover;
            margin: 10px 0;
        }

        .controls-grid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 10px;
        }

        .control-group {
            background: var(--bg-color);
            padding: 10px;
            border-radius: 6px;
            display: flex;
            flex-direction: column;
            gap: 8px;
        }

        .button-group {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }

        button {
            background: var(--accent-color);
            color: white;
            border: none;
            padding: var(--button-padding);
            border-radius: 4px;
            cursor: pointer;
            font-weight: 500;
            font-size: var(--font-size);
            transition: transform 0.1s, background 0.3s;
            flex: 1 1 30%;
            text-align: center;
        }

        button:hover {
            background: #d35400;
            transform: translateY(-1px);
        }

        button.danger {
            background: var(--danger-color);
        }

        button.danger:hover {
            background: #c0392b;
        }

        input[type="number"], input[type="text"] {
            width: 60px;
            padding: 6px;
            border: 1px solid #555;
            border-radius: 4px;
            margin: 0 5px;
            background-color: var(--secondary-color);
            color: var(--text-color);
            font-size: var(--font-size);
            text-align: center;
        }

        input[type="number"]::placeholder, input[type="text"]::placeholder {
            color: var(--text-color);
        }

        #map {
            height: 400px;
            border-radius: 6px;
            overflow: hidden;
        }

        .input-group {
            display: flex;
            align-items: center;
            justify-content: space-between;
        }

        .input-group label {
            flex: 1;
            color: var(--primary-color);
            font-size: var(--font-size);
        }

        .input-group input {
            flex: 1;
            max-width: 80px;
        }

        .update-pid {
            background: var(--accent-color);
            color: white;
            border: none;
            padding: 6px 12px;
            border-radius: 4px;
            cursor: pointer;
            font-weight: 500;
            font-size: var(--font-size);
            transition: background 0.3s;
            align-self: flex-end;
        }

        .update-pid:hover {
            background: #d35400;
        }

        @media (max-width: 1200px) {
            .dashboard {
                grid-template-columns: 1fr;
            }

            .maps-container {
                grid-column: span 1;
            }

            .maps-grid {
                grid-template-columns: 1fr;
            }

            #video {
                height: 150px;
            }

            .panel {
                height: auto;
            }

            #map {
                height: 300px;
            }
        }
    </style>
</head>
<body>
    <div class="dashboard">
        <div class="control-panel">
            <div class="panel">
                <div class="header">
                    <h1 class="title">Robot Control</h1>
                    <span class="status">Online</span>
                </div>
                <img id="video" src="/video_feed" alt="Robot Camera Feed">
                <div class="controls-grid">
                    <!-- Face Detection Settings -->
                    <div class="control-group">
                        <h3>Face Detection</h3>
                        <div class="input-group">
                            <label for="desiredFaceArea">Face Area:</label>
                            <input type="number" id="desiredFaceArea" value="5000" min="1000" max="10000">
                            <button onclick="increaseFaceArea()">+</button>
                            <button onclick="decreaseFaceArea()">-</button>
                        </div>
                        <div class="input-group">
                            <label for="centerOffset">Center Offset:</label>
                            <input type="number" id="centerOffset" value="0" min="-320" max="320">
                            <button onclick="moveCenterLeft()">&#8592;</button>
                            <button onclick="moveCenterRight()">&#8594;</button>
                        </div>
                    </div>
                    <!-- PID Controller Settings -->
                    <div class="control-group">
                        <h3>PID Controller</h3>
                        <div class="input-group">
                            <label for="kp">Kp:</label>
                            <input type="number" id="kp" value="0.5" step="0.01">
                        </div>
                        <div class="input-group">
                            <label for="ki">Ki:</label>
                            <input type="number" id="ki" value="0.0001" step="0.0001">
                        </div>
                        <div class="input-group">
                            <label for="kd">Kd:</label>
                            <input type="number" id="kd" value="0.25" step="0.01">
                        </div>
                        <button class="update-pid" onclick="updatePID()">Update PID</button>
                    </div>
                </div>
            </div>
        </div>
        <div class="maps-container">
            <div class="maps-grid">
                <div class="panel">
                    <div class="header">
                        <h2 class="title">Location Tracking</h2>
                    </div>
                    <div id="map"></div>
                </div>
                <!-- Replaced Path Planning panel with Mode panel -->
                <div class="panel">
                    <div class="header">
                        <h2 class="title">Mode</h2>
                    </div>
                    <div class="control-group">
                        <div class="button-group">
                            <button onclick="setMode('basic_movement')">Basic Movement</button>
                            <button onclick="setMode('auto_navigation')">Auto-Navigation</button>
                            <button onclick="setMode('face_tracking')">Face Tracking</button>
                            <!-- Add this button in the Mode selection section -->
<button onclick="setMode('road_tracking')">Road Tracking</button>

                        </div>
                    </div>
                    <!-- Auto-Navigation Controls -->
                    <div class="control-group" id="autoNavControls" style="display: none;">
                        <h3>Auto-Navigation Controls</h3>
                        <div class="button-group">
                            <button id="sendButton">Send Command</button>
                            <button id="estopButton" class="danger">E-Stop</button>
                            <button id="undoEstopButton">Undo E-Stop</button>
                        </div>
                    </div>
                </div>
            </div>
            <div class="panel">
                <div class="controls-grid">
                    <!-- Emergency Controls -->
                    <div class="control-group">
                        <h3>Emergency Controls</h3>
                        <div class="button-group">
                            <button class="danger" onclick="sendEStop()">E-Stop</button>
                            <button onclick="undoEStop()">Resume</button>
                        </div>
                    </div>
                    <!-- Robot Movement Controls -->
                    <div class="control-group">
                        <h3>Movement Controls</h3>
                        <div class="button-group">
                            <button onclick="moveForward()">Forward</button>
                            <button onclick="moveBackward()">Backward</button>
                            <button onclick="moveLeft()">Left</button>
                            <button onclick="moveRight()">Right</button>
                            <button onclick="stopRobot()">Stop</button>
                        </div>
                    </div>
                </div>
            </div>
        </div>
    </div>

    <!-- Scripts -->
    <!-- Leaflet JS -->
    <script src="https://unpkg.com/leaflet@1.7.1/dist/leaflet.js"></script>
    <!-- Leaflet Draw JS -->
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet.draw/1.0.4/leaflet.draw.js"></script>
    <!-- Leaflet Rotated Marker JS -->
    <script src="https://rawcdn.githack.com/bbecquet/Leaflet.RotatedMarker/master/leaflet.rotatedMarker.js"></script>
    <script>
        var map;
        var robotMarker;
        var robotIcon;
        var drawnItems;
        var drawControl;
        var pathPolyline = null;
        var plannedPath = null;
        var gpsData = [];

        // Emergency Controls
        function sendEStop() {
            fetch('/estop', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                alert(data.status);
            })
            .catch(error => console.error('Error:', error));
        }

        function undoEStop() {
            fetch('/undo_estop', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                alert(data.status);
            })
            .catch(error => console.error('Error:', error));
        }

        // Robot Movement Controls
        function moveForward() {
            fetch('/move_forward', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                console.log('Robot moving forward');
            })
            .catch(error => console.error('Error:', error));
        }

        function moveBackward() {
            fetch('/move_backward', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                console.log('Robot moving backward');
            })
            .catch(error => console.error('Error:', error));
        }

        function moveLeft() {
            fetch('/move_left', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                console.log('Robot moving left');
            })
            .catch(error => console.error('Error:', error));
        }

        function moveRight() {
            fetch('/move_right', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                console.log('Robot moving right');
            })
            .catch(error => console.error('Error:', error));
        }

        function stopRobot() {
            fetch('/stop_robot', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                console.log('Robot stopped');
            })
            .catch(error => console.error('Error:', error));
        }

        // Face Detection Settings
        function increaseFaceArea() {
            fetch('/increase_face_area', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                document.getElementById('desiredFaceArea').value = data.desired_face_area;
            })
            .catch(error => console.error('Error:', error));
        }

        function decreaseFaceArea() {
            fetch('/decrease_face_area', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                document.getElementById('desiredFaceArea').value = data.desired_face_area;
            })
            .catch(error => console.error('Error:', error));
        }

        function moveCenterLeft() {
            fetch('/move_center_left', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                document.getElementById('centerOffset').value = data.center_offset;
            })
            .catch(error => console.error('Error:', error));
        }

        function moveCenterRight() {
            fetch('/move_center_right', { method: 'POST' })
            .then(response => response.json())
            .then(data => {
                document.getElementById('centerOffset').value = data.center_offset;
            })
            .catch(error => console.error('Error:', error));
        }

        // PID Controller Settings
        function updatePID() {
            var kp = parseFloat(document.getElementById('kp').value);
            var ki = parseFloat(document.getElementById('ki').value);
            var kd = parseFloat(document.getElementById('kd').value);

            var data = { kp: kp, ki: ki, kd: kd };

            fetch('/update_pid', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify(data)
            })
            .then(response => response.json())
            .then(data => {
                alert('PID parameters updated');
            })
            .catch(error => console.error('Error:', error));
        }

        // Mode Selection
        function setMode(mode) {
    fetch('/set_mode', {
        method: 'POST',
        headers: {'Content-Type': 'application/json'},
        body: JSON.stringify({ mode: mode })
    })
    .then(response => response.json())
    .then(data => {
        alert('Mode set to ' + mode.replace('_', ' '));
        if (mode === 'auto_navigation' || mode === 'road_tracking') {
            document.getElementById('autoNavControls').style.display = 'block';
            initializeDrawing();
        } else {
            document.getElementById('autoNavControls').style.display = 'none';
            removeDrawing();
        }
    })
    .catch(error => console.error('Error:', error));
}

        function initializeDrawing() {
            drawnItems = new L.FeatureGroup();
            map.addLayer(drawnItems);

            drawControl = new L.Control.Draw({
                edit: {
                    featureGroup: drawnItems
                }
            });
            map.addControl(drawControl);

            map.on(L.Draw.Event.CREATED, function (event) {
                var layer = event.layer;
                drawnItems.addLayer(layer);
            });

            document.getElementById('sendButton').onclick = function() {
                var coordinates = [];
                drawnItems.eachLayer(function(layer) {
                    if (layer instanceof L.Polyline || layer instanceof L.Polygon) {
                        var latLngs = layer.getLatLngs();
                        if (latLngs.length > 0 && Array.isArray(latLngs[0])) {
                            latLngs.forEach(function(latlngArray) {
                                latlngArray.forEach(function(latlng) {
                                    coordinates.push({ lat: latlng.lat, lng: latlng.lng });
                                });
                            });
                        } else {
                            latLngs.forEach(function(latlng) {
                                coordinates.push({ lat: latlng.lat, lng: latlng.lng });
                            });
                        }
                    } else if (layer instanceof L.Marker) {
                        var latlng = layer.getLatLng();
                        coordinates.push({ lat: latlng.lat, lng: latlng.lng });
                    }
                });

                fetch('/send_coordinates', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify({ coordinates: coordinates })
                })
                .then(response => response.json())
                .then(data => {
                    console.log(data);
                    alert("Coordinates sent to the server");
                    if (plannedPath) {
                        map.removeLayer(plannedPath);
                    }
                    plannedPath = L.polyline(coordinates.map(c => [c.lat, c.lng]), {color: 'green'}).addTo(map);
                })
                .catch(error => console.error('Error:', error));
            };

            document.getElementById('estopButton').onclick = function() {
                sendEStop();
            };

            document.getElementById('undoEstopButton').onclick = function() {
                undoEStop();
            };
        }

        function removeDrawing() {
            if (drawControl) {
                map.removeControl(drawControl);
                drawControl = null;
            }
            if (drawnItems) {
                map.removeLayer(drawnItems);
                drawnItems = null;
            }
            if (plannedPath) {
                map.removeLayer(plannedPath);
                plannedPath = null;
            }
            if (pathPolyline) {
                map.removeLayer(pathPolyline);
                pathPolyline = null;
            }
        }

        // Initialize the map
        map = L.map('map').setView([0, 0], 2);

        // Add OpenStreetMap tile layer
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '&copy; OpenStreetMap contributors'
        }).addTo(map);

        // Robot marker with rotation
        robotIcon = L.icon({
            iconUrl: 'https://raw.githubusercontent.com/pointhi/leaflet-color-markers/master/img/marker-icon-red.png',
            iconSize: [25, 41],
            iconAnchor: [12, 41],
        });

        robotMarker = L.marker([0, 0], {
            icon: robotIcon,
            rotationAngle: 0
        }).addTo(map);

        // Fetch initial GPS position
        fetch('/initial_gps')
            .then(response => response.json())
            .then(data => {
                var initialLat = data.lat || 0;
                var initialLon = data.lon || 0;
                map.setView([initialLat, initialLon], 18);
                robotMarker.setLatLng([initialLat, initialLon]);
            });

        // Update robot position and heading periodically
        setInterval(function() {
            fetch('/get_gps_data')
                .then(response => response.json())
                .then(data => {
                    if (data && data.length > 0) {
                        gpsData = data;
                        var gpsCoordinates = data.map(function(point) {
                            return [point.Estimated_Lat || point.GPS_Lat, point.Estimated_Lon || point.GPS_Lon];
                        });

                        var latestPosition = gpsCoordinates[gpsCoordinates.length - 1];
                        robotMarker.setLatLng(latestPosition);

                        var heading = data[data.length - 1].Estimated_Theta || data[data.length - 1].Heading || 0;
                        robotMarker.setRotationAngle(heading);

                        if (pathPolyline) {
                            pathPolyline.setLatLngs(gpsCoordinates);
                        } else {
                            pathPolyline = L.polyline(gpsCoordinates, {color: 'blue'}).addTo(map);
                        }
                    }
                })
                .catch(error => console.error('Error fetching GPS data:', error));
        }, 1000);
    </script>
</body>
</html>
'''

# Define Flask route functions here
@app.route("/")
def index():
    return render_template_string(html_content)

@app.route("/video_feed")
def video_feed():
    return Response(generate(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/estop", methods=['POST'])
def estop():
    global e_stop_active, stop_event
    e_stop_active = True
    print("E-Stop activated!")
    front_back_command = 64  # Stop
    side_side_command = 64   # Neutral steering
    command_string = f"{front_back_command} {side_side_command}"
    client.publish(MQTT_TOPIC_COMMAND, command_string)
    # Send estop command to processes
    command_queue.put(('estop', None))
    stop_event.set()
    return jsonify({"status": "E-Stop activated"})

@app.route("/undo_estop", methods=['POST'])
def undo_estop():
    global e_stop_active, stop_event
    e_stop_active = False
    stop_event.clear()
    command_queue.put(('undo_estop', None))
    print("E-Stop deactivated!")
    return jsonify({"status": "E-Stop deactivated"})

@app.route("/increase_face_area", methods=['POST'])
def increase_face_area():
    # Handle in face_tracking.py via command_queue
    command_queue.put(('increase_face_area', None))
    return jsonify({"status": "Face area increased"})

@app.route("/decrease_face_area", methods=['POST'])
def decrease_face_area():
    # Handle in face_tracking.py via command_queue
    command_queue.put(('decrease_face_area', None))
    return jsonify({"status": "Face area decreased"})

@app.route("/move_center_left", methods=['POST'])
def move_center_left():
    # Handle in face_tracking.py via command_queue
    command_queue.put(('move_center_left', None))
    return jsonify({"status": "Center moved left"})

@app.route("/move_center_right", methods=['POST'])
def move_center_right():
    # Handle in face_tracking.py via command_queue
    command_queue.put(('move_center_right', None))
    return jsonify({"status": "Center moved right"})

@app.route('/get_gps_data', methods=['GET'])
def get_gps_data_route():
    with gps_data_lock:
        if gps_data:
            sanitized_gps_data = [{
                'GPS_Lat': float(item.get('GPS_Lat', 0)),
                'GPS_Lon': float(item.get('GPS_Lon', 0)),
                'Heading': float(item.get('Heading', 0)),
                'Estimated_Lat': float(item.get('Estimated_Lat', 0)),
                'Estimated_Lon': float(item.get('Estimated_Lon', 0)),
                'Estimated_Theta': float(item.get('Estimated_Theta', 0))
            } for item in gps_data]
        else:
            sanitized_gps_data = []
    return jsonify(sanitized_gps_data)

@app.route('/initial_gps', methods=['GET'])
def initial_gps():
    lat, lon = receive_gps_data()
    if lat is not None and lon is not None:
        return jsonify({"lat": lat, "lon": lon})
    else:
        print("Initial GPS data unavailable.")
        return jsonify({"lat": 0.0, "lon": 0.0})


road_track_proc = None

@app.route("/set_mode", methods=['POST'])
def set_mode():
    global current_mode, stop_event, auto_nav_proc, road_track_proc
    data = request.get_json()
    mode = data.get('mode', 'basic_movement')
    if mode in ['basic_movement', 'auto_navigation', 'face_tracking', 'road_tracking']:
        if current_mode != mode:
            # Handle mode change
            stop_event.set()  # Stop any existing process
            time.sleep(0.5)   # Wait for processes to stop
            stop_event.clear()

            # Stop the road_tracking.py process if it's running
            if road_track_proc is not None and road_track_proc.poll() is None:
                road_track_proc.terminate()
                road_track_proc.wait()
                road_track_proc = None

            if mode == 'auto_navigation':
                auto_nav_proc = Process(target=auto_navigation_process,
                                        args=(command_queue, movement_command_queue, stop_event))
                auto_nav_proc.start()
                print("Started auto-navigation process.")
            elif mode == 'road_tracking':
                # Start road_tracking.py as a subprocess
                road_track_proc = subprocess.Popen(['python3', 'road_tracking.py'])
                print("Started road tracking process.")
            elif mode == 'face_tracking':
                face_track_proc = Process(target=face_tracking_process,
                                          args=(command_queue, detection_queue, movement_command_queue))
                face_track_proc.start()
                print("Started face tracking process.")
            else:
                # No additional process needed for basic_movement
                pass
        current_mode = mode
        print(f"Mode set to {current_mode}")
        return jsonify({"status": f"Mode set to {current_mode}"})
    else:
        print("Invalid mode selected")
        return jsonify({"status": "Invalid mode selected"}), 400
        
@app.route("/move_forward", methods=['POST'])
def move_forward():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 126  # Max forward
        side_side_command = 64    # Neutral steering
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Moving forward"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_backward", methods=['POST'])
def move_backward():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 0   # Max backward
        side_side_command = 64    # Neutral steering
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Moving backward"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_left", methods=['POST'])
def move_left():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 64  # Stop
        side_side_command = 126    # Max left
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Turning left"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/move_right", methods=['POST'])
def move_right():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 64  # Stop
        side_side_command = 0   # Max right
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Turning right"})
    else:
        return jsonify({"status": "Cannot move in current mode"}), 400

@app.route("/stop_robot", methods=['POST'])
def stop_robot():
    global current_mode
    if current_mode == 'basic_movement':
        front_back_command = 64   # Stop
        side_side_command = 64    # Neutral steering
        command_string = f"{front_back_command} {side_side_command}"
        client.publish(MQTT_TOPIC_COMMAND, command_string)
        return jsonify({"status": "Robot stopped"})
    else:
        return jsonify({"status": "Cannot stop in current mode"}), 400

@app.route("/update_pid", methods=['POST'])
def update_pid():
    data = request.get_json()
    kp = data.get('kp')
    ki = data.get('ki')
    kd = data.get('kd')

    if kp is not None and ki is not None and kd is not None:
        # Send PID update to face_tracking process
        command_queue.put(('update_pid', (kp, ki, kd)))
        print(f"Updated PID parameters: Kp={kp}, Ki={ki}, Kd={kd}")
        return jsonify({"status": "PID parameters updated"})
    else:
        return jsonify({"status": "Invalid PID parameters"}), 400

@app.route('/send_coordinates', methods=['POST'])
def receive_coordinates():
    data = request.get_json()
    coordinates = data.get('coordinates', [])
    if coordinates:
        command_queue.put(('set_waypoints', coordinates))
        print("Coordinates received and sent to road_tracking_process.")
        return jsonify({"status": "Coordinates received"})
    else:
        return jsonify({"status": "No coordinates received"}), 400

if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s')
    auto_nav_proc = None
    road_track_proc = None
    # Start the main loop thread
    t = threading.Thread(target=main_loop)
    t.daemon = True
    t.start()
    # Start face tracking process
    face_track_proc = Process(target=face_tracking_process,
                              args=(command_queue, detection_queue, movement_command_queue))
    face_track_proc.start()
    # Run the Flask app
    app.run(host='0.0.0.0', port=5000)
