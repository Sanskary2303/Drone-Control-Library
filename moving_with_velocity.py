import time
from pymavlink import mavutil
import utils

def main():
    master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)
    utils.wait_for_heartbeat(master)
    
    utils.set_mode(master, 'GUIDED')
    initial_lat, initial_lon = utils.get_current_location(master)
    print(f"Initial position: lat={initial_lat}, lon={initial_lon}")
        
    utils.arm_vehicle(master)
    

    altitude = float(input("Enter desired takeoff altitude in meters: "))
    utils.takeoff(master, altitude)
    
    print("Waiting for the drone to reach target altitude...")
    time.sleep(10)
    
    try:
        print("Moving NORTH at 2 m/s for 5 seconds")
        utils.set_velocity(master, 0, 2, 0)  # vx=0 (East), vy=2 (North), vz=0
        time.sleep(5)
        
        utils.set_velocity(master, 0, 0, 0)
        time.sleep(2)
        
        print("Moving EAST at 2 m/s for 5 seconds")
        utils.set_velocity(master, 2, 0, 0)  # vx=2 (East), vy=0, vz=0
        time.sleep(5)
        
        #Stop
        utils.set_velocity(master, 0, 0, 0)
        time.sleep(2)
        
        #Move diagonally (Northeast) at 1.4 m/s in each direction (~2 m/s combined)
        print("Moving NORTHEAST at 1.4 m/s in each direction for 5 seconds")
        utils.set_velocity(master, 1.4, 1.4, 0)  # vx=1.4 (East), vy=1.4 (North), vz=0
        time.sleep(5)
        
        # Stop
        utils.set_velocity(master, 0, 0, 0)
        time.sleep(2)
        
        final_lat, final_lon = utils.get_current_location(master)
        print(f"Final position: lat={final_lat}, lon={final_lon}")
        
        print("Returning to launch position...")
        utils.return_to_launch(master)
        time.sleep(10)
        
    except KeyboardInterrupt:
        print("\nUser interrupted! Stopping and landing the drone...")
        utils.set_velocity(master, 0, 0, 0)
        time.sleep(1)
        
        utils.land(master)
        
if __name__ == '__main__':
    main()