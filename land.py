import time
from pymavlink import mavutil
import utils

def main():
    master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)
    
    utils.wait_for_heartbeat(master)
    
    utils.land(master)
    
    time.sleep(5)

if __name__ == '__main__':
    main()