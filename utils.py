"""
This module provides utility functions for drone operation using pymavlink.
"""

import sys
import math
import logging
from time import sleep, time
from pymavlink import mavutil
from typing import Tuple, Optional


logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')


def get_drone_info(master) -> dict:
    """
    Get basic information about the drone including version, capabilities, and status.
    
    Parameters:
        master: mavutil connection object.
        
    Returns:
        dict: Dictionary containing drone information.
    """
    info = {
        'autopilot_type': 'Unknown',
        'firmware_version': 'Unknown',
        'vehicle_type': 'Unknown',
        'system_id': master.target_system,
        'component_id': master.target_component,
        'flight_mode': 'Unknown',
        'armed': False,
        'capabilities': []
    }
    
    try:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg:
            autopilot_types = {
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC: "Generic",
                mavutil.mavlink.MAV_AUTOPILOT_RESERVED: "Reserved",
                mavutil.mavlink.MAV_AUTOPILOT_SLUGS: "SLUGS",
                mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA: "ArduPilot",
                mavutil.mavlink.MAV_AUTOPILOT_OPENPILOT: "OpenPilot",
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY: "Generic Waypoints Only",
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY: "Generic Waypoints with Simple Navigation",
                mavutil.mavlink.MAV_AUTOPILOT_GENERIC_MISSION_FULL: "Generic Mission Full",
                mavutil.mavlink.MAV_AUTOPILOT_INVALID: "Invalid",
                mavutil.mavlink.MAV_AUTOPILOT_PPZ: "PPZ",
                mavutil.mavlink.MAV_AUTOPILOT_UDB: "UDB",
                mavutil.mavlink.MAV_AUTOPILOT_FP: "FlexiPilot",
                mavutil.mavlink.MAV_AUTOPILOT_PX4: "PX4",
                mavutil.mavlink.MAV_AUTOPILOT_SMACCMPILOT: "SMACCMPilot",
                mavutil.mavlink.MAV_AUTOPILOT_AUTOQUAD: "AutoQuad",
                mavutil.mavlink.MAV_AUTOPILOT_ARMAZILA: "Armazila",
                mavutil.mavlink.MAV_AUTOPILOT_AEROB: "Aerob",
                mavutil.mavlink.MAV_AUTOPILOT_ASLUAV: "ASLUAV",
                mavutil.mavlink.MAV_AUTOPILOT_SMARTAP: "SmartAP",
                mavutil.mavlink.MAV_AUTOPILOT_AIRRAILS: "AirRails"
            }
            info['autopilot_type'] = autopilot_types.get(msg.autopilot, "Unknown")
            
            vehicle_types = {
                mavutil.mavlink.MAV_TYPE_GENERIC: "Generic",
                mavutil.mavlink.MAV_TYPE_FIXED_WING: "Fixed Wing",
                mavutil.mavlink.MAV_TYPE_QUADROTOR: "Quadrotor",
                mavutil.mavlink.MAV_TYPE_COAXIAL: "Coaxial",
                mavutil.mavlink.MAV_TYPE_HELICOPTER: "Helicopter",
                mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER: "Antenna Tracker",
                mavutil.mavlink.MAV_TYPE_GCS: "Ground Control Station",
                mavutil.mavlink.MAV_TYPE_AIRSHIP: "Airship",
                mavutil.mavlink.MAV_TYPE_FREE_BALLOON: "Free Balloon",
                mavutil.mavlink.MAV_TYPE_ROCKET: "Rocket",
                mavutil.mavlink.MAV_TYPE_GROUND_ROVER: "Ground Rover",
                mavutil.mavlink.MAV_TYPE_SURFACE_BOAT: "Surface Boat",
                mavutil.mavlink.MAV_TYPE_SUBMARINE: "Submarine",
                mavutil.mavlink.MAV_TYPE_HEXAROTOR: "Hexarotor",
                mavutil.mavlink.MAV_TYPE_OCTOROTOR: "Octorotor",
                mavutil.mavlink.MAV_TYPE_TRICOPTER: "Tricopter",
                mavutil.mavlink.MAV_TYPE_FLAPPING_WING: "Flapping Wing",
                mavutil.mavlink.MAV_TYPE_KITE: "Kite",
                mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER: "Onboard Companion Controller",
                mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR: "VTOL Duorotor",
                mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR: "VTOL Quadrotor",
                mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR: "VTOL Tiltrotor",
                mavutil.mavlink.MAV_TYPE_VTOL_RESERVED2: "VTOL Reserved",
                mavutil.mavlink.MAV_TYPE_VTOL_RESERVED3: "VTOL Reserved",
                mavutil.mavlink.MAV_TYPE_VTOL_RESERVED4: "VTOL Reserved",
                mavutil.mavlink.MAV_TYPE_VTOL_RESERVED5: "VTOL Reserved",
                mavutil.mavlink.MAV_TYPE_GIMBAL: "Gimbal",
                mavutil.mavlink.MAV_TYPE_ADSB: "ADSB",
                mavutil.mavlink.MAV_TYPE_PARAFOIL: "Parafoil"
            }
            info['vehicle_type'] = vehicle_types.get(msg.type, "Unknown")
            
            base_mode = msg.base_mode
            info['armed'] = bool(base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
            info['flight_mode'] = get_current_mode(master)
            
            capabilities = []
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT:
                capabilities.append("Mission Float")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT:
                capabilities.append("Param Float")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_MISSION_INT:
                capabilities.append("Mission Int")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_COMMAND_INT:
                capabilities.append("Command Int")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_PARAM_UNION:
                capabilities.append("Param Union")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_FTP:
                capabilities.append("FTP")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET:
                capabilities.append("Set Attitude Target")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED:
                capabilities.append("Set Position Target Local NED")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT:
                capabilities.append("Set Position Target Global Int")
            if msg.capabilities & mavutil.mavlink.MAV_PROTOCOL_CAPABILITY_TERRAIN:
                capabilities.append("Terrain")
            info['capabilities'] = capabilities
                
        msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
        if msg:
            version = ".".join([str((msg.flight_sw_version >> 24) & 0xFF), 
                              str((msg.flight_sw_version >> 16) & 0xFF),
                              str((msg.flight_sw_version >> 8) & 0xFF),
                              str(msg.flight_sw_version & 0xFF)])
            info['firmware_version'] = version
            
            #If there's a product name/version available (vendor-specific)
            if hasattr(msg, 'product_id') and msg.product_id:
                info['product_id'] = msg.product_id
            if hasattr(msg, 'vendor_id') and msg.vendor_id:
                info['vendor_id'] = msg.vendor_id
                        
        try:
            lat, lon = get_current_location(master)
            info['latitude'] = lat
            info['longitude'] = lon
        except:
            info['latitude'] = 'Unavailable'
            info['longitude'] = 'Unavailable'
            
        battery = check_battery(master)
        if battery:
            voltage, remaining = battery
            info['battery_voltage'] = f"{voltage:.2f}V"
            info['battery_remaining'] = f"{remaining}%"
        
        logging.info(f"Drone info retrieved: {info}")
        
        return info
        
    except Exception as e:
        logging.error(f"Error getting drone information: {e}")
        return info
    
def wait_for_heartbeat(master):
    """
    Wait for a heartbeat message to establish communication with the drone.
    """
    logging.info("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
    logging.info("Received heartbeat")


def xy_to_latlon(x: float, y: float, origin_lat: float, origin_lon: float) -> Tuple[float, float]:
    """
    Convert local (x, y) offsets in meters to global latitude and longitude.
    
    Parameters:
        x (float): Eastward displacement in meters.
        y (float): Northward displacement in meters.
        origin_lat (float): Origin latitude in degrees.
        origin_lon (float): Origin longitude in degrees.
        
    Returns:
        Tuple[float, float]: Converted (latitude, longitude) in degrees.
    """
    R = 6371000
    origin_lat_rad = math.radians(origin_lat)
    origin_lon_rad = math.radians(origin_lon)
    lat_rad = origin_lat_rad + (y / R)
    lon_rad = origin_lon_rad + (x / (R * math.cos(origin_lat_rad)))
    return math.degrees(lat_rad), math.degrees(lon_rad)


def get_gps_fix(master):
    """
    Wait for and return a valid GLOBAL_POSITION_INT message.
    
    Returns:
        message: A GLOBAL_POSITION_INT message with valid GPS data.
    """
    logging.info("Waiting for GPS fix...")
    while True:
        gps_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if gps_msg and gps_msg.lat != 0 and gps_msg.lon != 0:
            logging.info("GPS fix acquired")
            return gps_msg
        
def get_current_location(master) -> Tuple[float, float]:
    """
    Retrieves the current GPS location from the GLOBAL_POSITION_INT message.
    
    Returns:
        Tuple[float, float]: (latitude, longitude) in degrees.
    """
    gps_msg = get_gps_fix(master)
    lat = gps_msg.lat / 1e7
    lon = gps_msg.lon / 1e7
    logging.info(f"Current location acquired: lat={lat}, lon={lon}")
    return lat, lon

def arm_vehicle(master):
    """
    Send arm command to the drone.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)
    logging.info("Arming vehicle...")
    sleep(2)
    logging.info("Armed!")


def disarm_vehicle(master):
    """
    Disarm the drone.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0, 0, 0, 0, 0, 0, 0)
    logging.info("Disarming vehicle...")
    sleep(2)
    logging.info("Disarmed!")


def set_mode(master, mode: str):
    """
    Set the flight mode of the drone.
    
    Parameters:
        master: mavutil connection object.
        mode (str): Desired flight mode (e.g. 'GUIDED', 'AUTO', 'RTL').
    """
    mapping = master.mode_mapping()
    if mode not in mapping:
        logging.error(f'Unknown mode: {mode}')
        logging.error(f'Available modes: {list(mapping.keys())}')
        sys.exit(1)
    mode_id = mapping[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    logging.info(f"Switched to {mode} mode")
    sleep(1)

def get_current_mode(master) -> str:
    """
    Get the current flight mode of the drone.
    
    Parameters:
        master: mavutil connection object.
        
    Returns:
        str: Current flight mode, or 'Unknown' if not available.
    """
    try:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        
        if msg:
            mode_id = msg.custom_mode
            mode_mapping = master.mode_mapping()
            
            for mode_name, mode_value in mode_mapping.items():
                if mode_value == mode_id:
                    logging.info(f"Current flight mode: {mode_name}")
                    return mode_name
                    
            logging.warning(f"Unknown mode ID: {mode_id}")
            return f"Unknown({mode_id})"
        else:
            logging.warning("Failed to receive heartbeat for mode query")
            return "Unknown"
            
    except Exception as e:
        logging.error(f"Error getting flight mode: {e}")
        return "Unknown"

def set_home_location(master, lat: float, lon: float, alt: float):
    """
    Set the home location of the drone.
    
    Parameters:
        master: mavutil connection object.
        lat (float): Home latitude in degrees.
        lon (float): Home longitude in degrees.
        alt (float): Home altitude in meters.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,
        0,
        1, 0, 0, 0, lat, lon, alt)
    logging.info(f"Home location set: lat={lat}, lon={lon}, alt={alt}")

def takeoff(master, altitude: float):
    """
    Send takeoff command.
    
    Parameters:
        master: mavutil connection object.
        altitude (float): Desired altitude in meters.
    """
    takeoff_msg = master.mav.command_long_encode(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude)
    master.mav.send(takeoff_msg)
    logging.info(f"Taking off to {altitude} meter altitude")
    sleep(5)


def land(master):
    """
    Send landing command to the drone.
    
    Uses the MAV_CMD_NAV_LAND command.
    """
    land_msg = master.mav.command_long_encode(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0)
    master.mav.send(land_msg)
    logging.info("Landing initiated")


def send_waypoint_global(master, lat: float, lon: float, altitude: float):
    """
    Send a waypoint to the drone.
    
    Parameters:
        master: mavutil connection object.
        lat (float): Latitude of the waypoint in degrees.
        lon (float): Longitude of the waypoint in degrees.
        altitude (float): Altitude of the waypoint in meters.
    """
    master.mav.mission_item_int_send(
        master.target_system,            # System ID
        master.target_component,         # Component ID
        0,                               # Sequence number
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        2,                               # Current (guided mode request)
        0,                               # Autocontinue
        0, 0, 0, 0,                      # Params p1-p4
        int(lat * 1e7),                  # Latitude
        int(lon * 1e7),                  # Longitude
        altitude,                        # Altitude (meters)
        mavutil.mavlink.MAV_MISSION_TYPE_MISSION
    )
    logging.info(f"Waypoint sent: lat={lat}, lon={lon}, alt={altitude}")

def send_waypoint_local(master, x: float, y: float, z: float):
    """
    Send a waypoint to the drone in local NED frame.
    
    Parameters:
        master: mavutil connection object.
        x (float): Eastward displacement in meters.
        y (float): Northward displacement in meters.
        z (float): Altitude in meters.
    """

    origin_lat, origin_lon = get_current_location(master)
    lat, lon = xy_to_latlon(x, y, origin_lat, origin_lon)
    send_waypoint_global(master, lat, lon, z)
    logging.info(f"Local waypoint sent: x={x}, y={y}, z={z}")


def get_current_location(master) -> Tuple[float, float]:
    """
    Retrieves the current GPS location from the GLOBAL_POSITION_INT message.
    
    Returns:
        Tuple[float, float]: (latitude, longitude) in degrees.
    """
    gps_msg = get_gps_fix(master)
    lat = gps_msg.lat / 1e7
    lon = gps_msg.lon / 1e7
    logging.info(f"Current location acquired: lat={lat}, lon={lon}")
    return lat, lon


def return_to_launch(master):
    """
    Return drone to launch location.
    """
    set_mode(master, "RTL")
    logging.info("Returning to launch location")


def set_yaw(master, yaw_angle: float, yaw_speed: float = 20, direction: int = 1, relative: int = 0):
    """
    Set drone yaw (heading).
    
    Parameters:
        master: mavutil connection object.
        yaw_angle (float): Desired yaw angle in degrees.
        yaw_speed (float): Speed of yaw in degrees/sec.
        direction (int): 1 for clockwise, -1 for anti-clockwise.
        relative (int): 1 means relative to current heading.
    """
    master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, 
        0,
        yaw_angle, yaw_speed, direction, relative, 0, 0, 0)
    logging.info(f"Setting yaw to {yaw_angle} degrees with speed {yaw_speed}")


def set_velocity(master, vx: float, vy: float, vz: float):
    """
    Set drone velocity in local NED frame.
    
    Parameters:
        master: mavutil connection object.
        vx (float): Velocity in X direction (m/s).
        vy (float): Velocity in Y direction (m/s).
        vz (float): Velocity in Z direction (m/s); positive downward.
    """
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111,
        0, 0, 0, vx, vy, vz, 0, 0, 0, 0, 0)
    logging.info(f"Setting velocity: vx={vx}, vy={vy}, vz={vz}")


def check_battery(master) -> Optional[Tuple[float, int]]:
    """
    Check battery status.
    
    Returns:
        Optional[Tuple[float, int]]: Battery voltage in V and remaining battery percentage, if available.
    """
    msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if msg:
        voltage = msg.voltage_battery / 1000 if msg.voltage_battery > 0 else 0
        remaining = msg.battery_remaining
        logging.info(f"Battery: {voltage}V, {remaining}% remaining")
        return voltage, remaining
    else:
        logging.warning("Battery status not available")
        return None


def get_attitude(master) -> Optional[Tuple[float, float, float]]:
    """
    Retrieve the current attitude (roll, pitch, yaw) from the drone.
    
    Returns:
        Optional[Tuple[float, float, float]]: Attitude in radians, or None if not available.
    """
    msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=5)
    if msg:
        logging.info(f"Attitude acquired: roll={msg.roll}, pitch={msg.pitch}, yaw={msg.yaw}")
        return msg.roll, msg.pitch, msg.yaw
    else:
        logging.warning("Attitude message not available")
        return None
    
def haversine(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance between two points on the Earth (specified in decimal degrees).
    """
    R = 6371000
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi/2)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1 - a))

def wait_for_waypoint(master, target_lat, target_lon, tolerance=2, timeout=60):
    """
    Wait until the drone reaches the target waypoint within a specified tolerance.
    
    Parameters:
        master: mavutil connection object.
        target_lat (float): Target latitude in degrees.
        target_lon (float): Target longitude in degrees.
        tolerance (float): Distance tolerance in meters.
        timeout (int): Timeout duration in seconds.
        
    Returns:
        bool: True if waypoint reached within tolerance, else False.
    """
    start_time = time()
    while time() - start_time < timeout:
        current_lat, current_lon = get_current_location(master)
        distance = haversine(current_lat, current_lon, target_lat, target_lon)
        print(f"Distance to waypoint: {distance:.2f} meters")
        if distance <= tolerance:
            print("Waypoint reached.")
            return True
        sleep(1)
    print("Timeout reached before getting to the waypoint.")
    return False

def set_message_interval(master, message_id: int, interval_us: int) -> bool:
    """
    Set a message to be streamed at a specified interval.
    
    Example:
        To set the battery status message interval to 1 second (1000000 microseconds):
        
            set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 1000000)
    
    Parameters:
        master: mavutil connection object.
        message_id (int): ID of the MAVLink message to stream.
        interval_us (int): Stream interval in microseconds.
        
    Returns:
        bool: True if the command was accepted, False otherwise.
    """
    message = master.mav.command_long_encode(
        master.target_system,                  # Target system ID
        master.target_component,               # Target component ID
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # Command to set message interval
        0,                                     # Confirmation
        message_id,                            # param1: Message ID to stream
        interval_us,                           # param2: Interval in microseconds
        0, 0, 0, 0, 0)                         # Unused parameters
    master.mav.send(message)
    logging.info(f"Setting message interval for message_id {message_id} to {interval_us} microseconds")
    response = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        logging.info("Message interval set: command accepted")
        return True
    else:
        logging.error("Failed to set message interval")
        return False