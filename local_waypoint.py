import time
from pymavlink import mavutil
import utils

def main():
    master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)
    utils.wait_for_heartbeat(master)
    utils.set_mode(master, 'GUIDED')

    origin_lat, origin_lon = utils.get_current_location(master)
    print(f"Origin location: lat={origin_lat}, lon={origin_lon}")

    utils.arm_vehicle(master)
    altitude = float(input("Enter desired takeoff altitude in meters: "))
    
    utils.takeoff(master, altitude)
    time.sleep(5)
    
    x_offset = float(input("Enter the x offset (eastward) in meters: "))
    y_offset = float(input("Enter the y offset (northward) in meters: "))
    
    utils.send_waypoint_local(master, x_offset, y_offset, altitude)
    
    target_lat, target_lon = utils.xy_to_latlon(x_offset, y_offset, origin_lat, origin_lon)
    print(f"Target waypoint: lat={target_lat}, lon={target_lon}")
    
    if utils.wait_for_waypoint(master, target_lat, target_lon, tolerance=2, timeout=60):
        print("Local waypoint reached successfully.")
    else:
        print("Failed to reach local waypoint within timeout.")

if __name__ == '__main__':
    main()