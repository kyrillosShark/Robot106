



# auto_navigation.py

import numpy as np
import time
import math
from pyproj import Proj
import IMU
import gpsd
import logging
import sys

dt = 0.1  # Time step in seconds

# Process noise covariance matrix Q
Q = np.diag([0.05, 0.05, 0.02, 0.02, 0.005, 0.005, 0.002])

# Measurement noise covariance matrix R
R = np.diag([10.0, 10.0, 0.1])

proj_utm = None

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

def initialize_projection(ref_lat, ref_lon):
    global proj_utm
    # Determine the UTM zone based on reference longitude
    zone_number = int((ref_lon + 180) / 6) + 1
    hemisphere = 'north' if ref_lat >= 0 else 'south'
    proj_utm = Proj(proj='utm', zone=zone_number, ellps='WGS84', hemisphere=hemisphere)

def latlon_to_xy(lat, lon):
    x, y = proj_utm(lon, lat)
    return x, y

def xy_to_latlon(x, y):
    lon, lat = proj_utm(x, y, inverse=True)
    return lat, lon

def pt_to_pt_distance(pt1, pt2):
    return math.hypot(pt2[0] - pt1[0], pt2[1] - pt1[1])

def sgn(num):
    return 1 if num >= 0 else -1

def line_circle_intersection(current_pos, pt1, pt2, look_ahead_distance):
    x1 = pt1[0] - current_pos[0]
    y1 = pt1[1] - current_pos[1]
    x2 = pt2[0] - current_pos[0]
    y2 = pt2[1] - current_pos[1]

    dx = x2 - x1
    dy = y2 - y1
    dr = math.hypot(dx, dy)
    D = x1 * y2 - x2 * y1
    discriminant = (look_ahead_distance ** 2) * (dr ** 2) - D ** 2

    intersections = []
    if discriminant >= 0:
        sqrt_discriminant = math.sqrt(discriminant)
        sign_dy = sgn(dy) if dy != 0 else 1
        for sign in [1, -1]:
            x = (D * dy + sign * sign_dy * dx * sqrt_discriminant) / (dr ** 2)
            y = (-D * dx + abs(dy) * sqrt_discriminant * sign) / (dr ** 2)
            x += current_pos[0]
            y += current_pos[1]
            min_x = min(pt1[0], pt2[0]) - 1e-6
            max_x = max(pt1[0], pt2[0]) + 1e-6
            min_y = min(pt1[1], pt2[1]) - 1e-6
            max_y = max(pt1[1], pt2[1]) + 1e-6
            if min_x <= x <= max_x and min_y <= y <= max_y:
                intersections.append([x, y])

    return intersections

def find_goal_point(path, current_pos, look_ahead_distance, last_found_index):
    for i in range(last_found_index, len(path) - 1):
        pt1 = path[i]
        pt2 = path[i + 1]
        intersections = line_circle_intersection(current_pos, pt1, pt2, look_ahead_distance)
        if intersections:
            goal_point = min(intersections, key=lambda pt: pt_to_pt_distance(pt, pt2))
            return goal_point, i
    return path[-1], len(path) - 1

def find_min_angle(abs_target_angle, current_heading):
    min_angle = abs_target_angle - current_heading
    if min_angle > 180:
        min_angle -= 360
    elif min_angle < -180:
        min_angle += 360
    return min_angle

def get_accelerometer_data():
    # Read accelerometer data
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()

    # Convert raw accelerometer data to m/s^2 (assuming proper scaling)
    # Adjust the scaling_factor based on your accelerometer's sensitivity
    scaling_factor = 0.000598550415  # Example scaling factor for a ±2g accelerometer
    accel_x = ACCx * scaling_factor
    accel_y = ACCy * scaling_factor

    return [accel_x, accel_y]

def initialize_ekf(initial_x, initial_y, initial_theta):
    global x_est, P_est
    logging.info(f"Initializing EKF with initial_x: {initial_x}, initial_y: {initial_y}, initial_theta: {initial_theta}")
    if initial_x is None or initial_y is None or initial_theta is None:
        logging.error("Initial EKF parameters are not properly initialized.")
        return
    x_est = np.array([
        [initial_x],
        [initial_y],
        [0.],
        [0.],
        [0.],
        [0.],
        [math.radians(initial_theta)]
    ], dtype=float)
    P_est = np.eye(7) * 500.
    logging.info(f"EKF Initialized with State: {x_est.flatten()} and Covariance: \n{P_est}")

def ekf_predict(x_est, P_est, accel_data):
    # State transition matrix F
    F = np.array([
        [1, 0, dt, 0, 0.5 * dt ** 2, 0, 0],
        [0, 1, 0, dt, 0, 0.5 * dt ** 2, 0],
        [0, 0, 1, 0, dt, 0, 0],
        [0, 0, 0, 1, 0, dt, 0],
        [0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1]
    ])

    # Control input (acceleration)
    u = np.array([[0], [0], [0], [0], [accel_data[0]], [accel_data[1]], [0]])

    x_pred = F @ x_est + u * dt
    P_pred = F @ P_est @ F.T + Q

    return x_pred, P_pred

def ekf_update(x_pred, P_pred, z_meas):
    # Measurement matrix H
    H = np.array([
        [1, 0, 0, 0, 0, 0, 0],  # x
        [0, 1, 0, 0, 0, 0, 0],  # y
        [0, 0, 0, 0, 0, 0, 1]   # theta
    ])

    y = z_meas - H @ x_pred
    y[2, 0] = ((y[2, 0] + np.pi) % (2 * np.pi)) - np.pi  # Normalize angle

    S = H @ P_pred @ H.T + R

    try:
        K = P_pred @ H.T @ np.linalg.inv(S)
    except np.linalg.LinAlgError:
        logging.error("Singular matrix encountered while calculating Kalman Gain.")
        K = np.zeros_like(P_pred @ H.T)

    x_est_new = x_pred + K @ y
    P_est_new = (np.eye(len(x_est)) - K @ H) @ P_pred

    # Ensure covariance matrix remains positive definite
    if not np.all(np.linalg.eigvals(P_est_new) > 0):
        logging.error("Covariance matrix is not positive definite after update!")
        P_est_new = P_pred

    return x_est_new, P_est_new
def auto_navigation_process(command_queue, movement_command_queue, stop_event):
        global proj_utm
        x_est = None
        P_est = None

        waypoints = []
        ref_lat, ref_lon = None, None
        last_found_index = 0
        robot_stopped = False  # Flag to track if the robot has been commanded to stop
 
        while not stop_event.is_set():
            # Handle commands from the command_queue
            while not command_queue.empty():
                command = command_queue.get()
                if command[0] == 'set_waypoints':
                    coordinates = command[1]
                    waypoints = [(point['lat'], point['lng']) for point in coordinates]
                    if waypoints:
                        ref_lat, ref_lon = waypoints[0]
                        initialize_projection(ref_lat, ref_lon)
                        initial_x, initial_y = latlon_to_xy(ref_lat, ref_lon)
                        initial_theta = calculate_heading()
                        initialize_ekf(initial_x, initial_y, initial_theta)
                        last_found_index = 0
                        robot_stopped = False  # Reset the stop flag
                        logging.info("Waypoints set for auto-navigation.")
                    else:
                        logging.error("No waypoints received.")
                elif command[0] == 'estop':
                    stop_event.set()
                    # Send stop command back to main process
                    front_back_command = 64  # Stop
                    side_side_command = 64    # Neutral steering
                    command_string = f"{front_back_command} {side_side_command}"
                    movement_command_queue.put(command_string)
                    logging.info("E-Stop activated. Stopping the robot.")
                    return

            if not waypoints or x_est is None:
                # If the robot hasn't been stopped yet, send a stop command
                if not robot_stopped:
                    front_back_command = 64  # Stop
                    side_side_command = 64   # Neutral steering
                    command_string = f"{front_back_command} {side_side_command}"
                    movement_command_queue.put(command_string)
                    robot_stopped = True  # Set the flag to prevent repeated stop commands
                    logging.info("No waypoints available. Stopping the robot.")
                time.sleep(0.1)
                continue

            # Build the path in UTM coordinates
            path = []
            for lat, lon in waypoints:
                x_wp, y_wp = latlon_to_xy(lat, lon)
                path.append([x_wp, y_wp])

            # Calculate average segment length between waypoints
            segment_lengths = []
            for i in range(len(path) - 1):
                length = pt_to_pt_distance(path[i], path[i + 1])
                segment_lengths.append(length)

            # Adjust look ahead distance based on path characteristics
            if segment_lengths:
                avg_segment_length = sum(segment_lengths) / len(segment_lengths)
                min_look_ahead = 2.0  # Minimum look ahead distance
                max_look_ahead = 1000.0  # Maximum look ahead distance
                look_ahead_distance = min(max(avg_segment_length * 0.3, min_look_ahead), max_look_ahead)
            else:
                look_ahead_distance = 5.0  # Default value if no segments

            # Receive current GPS data
            current_lat, current_lon = receive_gps_data()
            if current_lat is None or current_lon is None:
                logging.warning("No GPS data available.")
                time.sleep(dt)
                continue

            current_x, current_y = latlon_to_xy(current_lat, current_lon)
            current_heading = calculate_heading()
            current_heading_rad = math.radians(current_heading)

            # Get accelerometer data
            accel_data = get_accelerometer_data()

            # Predict step with accelerometer data
            x_pred, P_pred = ekf_predict(x_est, P_est, accel_data)

            # Measurement update with GPS and heading
            z_meas = np.array([
                [current_x],
                [current_y],
                [current_heading_rad]
            ])

            x_est, P_est = ekf_update(x_pred, P_pred, z_meas)
            estimated_x = x_est[0, 0]
        estimated_y = x_est[1, 0]
        estimated_theta = x_est[6, 0]
        estimated_heading = math.degrees(estimated_theta) % 360

        if estimated_heading < 0:
            estimated_heading += 360

        estimated_pos = [estimated_x, estimated_y]
        goal_point, last_found_index = find_goal_point(path, estimated_pos, look_ahead_distance, last_found_index)

        delta_x = goal_point[0] - estimated_x
        delta_y = goal_point[1] - estimated_y
        abs_target_angle_rad = math.atan2(delta_y, delta_x)
        abs_target_angle = math.degrees(abs_target_angle_rad) % 360

        heading_error = find_min_angle(abs_target_angle, estimated_heading)

        # Calculate steering angle based on heading error
        max_steering_angle = 30.0  # Maximum steering angle in degrees
        Kp_steering = 1.0  # Proportional gain for steering

        steering_angle = Kp_steering * heading_error

        # Limit steering angle to maximum values
        steering_angle = max(-max_steering_angle, min(max_steering_angle, steering_angle))

        # Compute desired speed based on distance to goal
        distance_to_goal = pt_to_pt_distance(estimated_pos, path[-1])
        if distance_to_goal < 1.0:
            desired_speed = 0.0  # Stop
        else:
            desired_speed = 1.0  # Fixed forward speed (can be adjusted as needed)

        # Map desired speed to front-back command (0-126, with 64 as stop)
        if desired_speed > 0:
            front_back_command = 126  # Forward
        elif desired_speed < 0:
            front_back_command = 0    # Backward
        else:
            front_back_command = 64   # Stop

        # Map steering angle to side-side command (0-126, with 64 as center)
        side_side_command = int(64 + (steering_angle / max_steering_angle) * 62)
        side_side_command = max(0, min(126, side_side_command))

        # Send the command via movement_command_queue
        command_string = f"{front_back_command} {side_side_command}"
        movement_command_queue.put(command_string)
        logging.info(f"Published command: {command_string}")

        time.sleep(dt)

    # Cleanup
    # Send stop command when the process ends
        front_back_command = 64  # Stop
        side_side_command = 64   # Neutral steering
        command_string = f"{front_back_command} {side_side_command}"
        movement_command_queue.put(command_string)