from pymavlink import mavutil
import utils

def main():
    master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)
    
    utils.wait_for_heartbeat(master)
    
    utils.set_mode(master, 'GUIDED')
    
    lat, lon = utils.get_current_location(master)
    print(f"Current location: lat={lat}, lon={lon}")
    
    utils.arm_vehicle(master)

if __name__ == '__main__':
    main()


