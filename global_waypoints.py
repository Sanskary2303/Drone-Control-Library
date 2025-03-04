import time
from pymavlink import mavutil
import utils

def main():
    master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)
    
    utils.wait_for_heartbeat(master)
    utils.set_mode(master, 'GUIDED')
    utils.arm_vehicle(master)
    
    current_lat, current_lon = utils.get_current_location(master)
    print(f"Current location: lat={current_lat}, lon={current_lon}")
    
    # Define waypoints: (latitude, longitude, altitude)
    waypoints = [
        (current_lat, current_lon, 5),                        
        (current_lat + 0.0001, current_lon, 5),                  # North displacement
        (current_lat + 0.0001, current_lon + 0.0001, 5),         # Northeast displacement
        (current_lat, current_lon + 0.0001, 5)                   # East displacement
    ]
    
    for idx, (lat, lon, alt) in enumerate(waypoints):
        print(f"Sending waypoint {idx + 1}: lat={lat}, lon={lon}, alt={alt}")
        utils.send_waypoint_global(master, lat, lon, alt)
        
        if utils.wait_for_waypoint(master, lat, lon, tolerance=2, timeout=60):
            print(f"Waypoint {idx + 1} reached.")
        else:
            print(f"Failed to reach waypoint {idx + 1}.")
            
        #Optional pause between waypoints
        time.sleep(2)

if __name__ == '__main__':
    main()