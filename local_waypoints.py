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
    
    origin_lat, origin_lon = utils.get_current_location(master)
    print(f"Origin location: lat={origin_lat}, lon={origin_lon}")
    
    local_waypoints = [
        (5, 0),    #5 m east
        (5, 5),    #5 m east and 5 m north
        (0, 5),    #5 m north
        (-5, 5),   #5 m west and 5 m north
        (-5, 0)    #5 m west
    ]
    
    for idx, (x_offset, y_offset) in enumerate(local_waypoints):
        print(f"Sending local waypoint {idx + 1}: x offset = {x_offset} m, y offset = {y_offset} m")
        
        utils.send_waypoint_local(master, x_offset, y_offset, altitude)
        
        target_lat, target_lon = utils.xy_to_latlon(x_offset, y_offset, origin_lat, origin_lon)
        print(f"Target waypoint {idx + 1}: lat={target_lat}, lon={target_lon}")
        
        if utils.wait_for_waypoint(master, target_lat, target_lon, tolerance=2, timeout=60):
            print(f"Waypoint {idx + 1} reached successfully.")
        else:
            print(f"Failed to reach waypoint {idx + 1} within timeout.")
        
        time.sleep(2)

if __name__ == '__main__':
    main()