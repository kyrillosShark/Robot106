# road_tracking.py

import paho.mqtt.client as mqtt
import json
import time
import threading
import math
import logging

# MQTT Configuration
MQTT_SERVER = "192.168.1.120"  # Update if different
MQTT_PORT = 1883
MQTT_TOPIC_COMMAND = "robot/control"
MQTT_TOPIC_DETECTIONS = "robot/detections"
MQTT_TOPIC_NAVIGATION_MODE = "robot/navigation_mode"

# PID Controller Parameters
pid = {'kp': 0.5, 'ki': 0.0, 'kd': 0.1}
previous_error = 0
integral = 0

# Variables to store the latest lines data
latest_lines = None
lines_lock = threading.Lock()

# Frame dimensions (update if necessary)
FRAME_WIDTH = 640

# Flag to indicate whether the process should stop
stop_flag = False

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s:%(message)s')

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info(f"Connected to MQTT server {MQTT_SERVER} successfully.")
        # Subscribe to detections topic
        client.subscribe(MQTT_TOPIC_DETECTIONS)
    else:
        logging.error(f"Failed to connect to MQTT server, return code {rc}")

def on_message(client, userdata, msg):
    global latest_lines
    try:
        if msg.topic == MQTT_TOPIC_DETECTIONS:
            payload = json.loads(msg.payload.decode())
            if payload.get('navigation_mode') == 'road_centering':
                detections = payload.get('detections', {})
                lines = detections.get('lines', [])
                with lines_lock:
                    latest_lines = lines
    except Exception as e:
        logging.error(f"Error handling message on topic {msg.topic}: {e}")

def calculate_middle_point(lines, frame_width):
    if len(lines) < 2:
        return None

    # Extract left and right lines based on their positions
    sorted_lines = sorted(lines, key=lambda l: l['x1'])
    left_line = sorted_lines[0]
    right_line = sorted_lines[-1]

    # Calculate middle x position between the two lines at the bottom of the frame
    y_middle = FRAME_WIDTH  # Assuming y at the bottom of the frame
    x_left = (left_line['x1'] + left_line['x2']) / 2
    x_right = (right_line['x1'] + right_line['x2']) / 2
    x_middle = (x_left + x_right) / 2

    return x_middle

def pid_control(target, current):
    global previous_error, integral
    error = target - current
    integral += error
    derivative = error - previous_error
    previous_error = error

    steering_adjust = pid['kp'] * error + pid['ki'] * integral + pid['kd'] * derivative
    return steering_adjust

def road_tracking_process():
    global stop_flag, latest_lines

    # Initialize MQTT Client
    client_mqtt = mqtt.Client()
    client_mqtt.on_connect = on_connect
    client_mqtt.on_message = on_message
    client_mqtt.connect(MQTT_SERVER, MQTT_PORT, 60)
    client_mqtt.loop_start()

    try:
        while not stop_flag:
            with lines_lock:
                lines = latest_lines

            if lines:
                # Perform PID control based on lines data
                x_middle = calculate_middle_point(lines, FRAME_WIDTH)
                if x_middle is not None:
                    frame_center = FRAME_WIDTH / 2
                    steering_adjust = pid_control(frame_center, x_middle)

                    # Map steering adjustment to robot command
                    max_steering_command = 62  # Adjust based on your robot's capabilities
                    steering_command = int(64 + (steering_adjust / frame_center) * max_steering_command)
                    steering_command = max(0, min(126, steering_command))  # Ensure command is within 0-126

                    front_back_command = 126  # Move forward
                    command_string = f"{front_back_command} {steering_command}"

                    # Publish the control command
                    client_mqtt.publish(MQTT_TOPIC_COMMAND, command_string)
                    logging.debug(f"Published control command: {command_string}")
                else:
                    logging.info("Insufficient lines data for PID control.")
            else:
                logging.info("No lines data received yet.")

            time.sleep(0.1)
    except Exception as e:
        logging.error(f"An error occurred in road_tracking_process: {e}")
    finally:
        client_mqtt.loop_stop()
        client_mqtt.disconnect()

def main():
    global stop_flag
    road_tracking_thread = threading.Thread(target=road_tracking_process)
    road_tracking_thread.start()

    try:
        while not stop_flag:
            time.sleep(1)
    except KeyboardInterrupt:
        stop_flag = True
        road_tracking_thread.join()

if __name__ == "__main__":
    main()