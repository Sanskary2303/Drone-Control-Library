import time
from pymavlink import mavutil
import utils

def main():
    altitude = float(input("Enter the desired altitude in meters: "))

    master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)

    utils.wait_for_heartbeat(master)

    utils.set_mode(master, 'GUIDED')

    current_lat, current_lon = utils.get_current_location(master)
    print(f"Current location: lat={current_lat}, lon={current_lon}")

    utils.arm_vehicle(master)

    utils.takeoff(master, altitude)

    time.sleep(5)

if __name__ == '__main__':
    main()