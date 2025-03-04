import time
import logging
from pymavlink import mavutil
import utils

def main():

    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
    logger = logging.getLogger(__name__)

    print("Choose connection method:")
    print("1. USB (e.g., /dev/ttyACM0)")
    print("2. Radio telemetry (e.g., /dev/ttyUSB0)")
    print("3. WiFi telemetry (e.g., udpin:127.0.0.1:14550)")
    
    choice = input("Enter choice (1-3): ")
    
    if choice == '1':
        device = '/dev/ttyACM0'
        master = mavutil.mavlink_connection(device=device, baud=57600)
        logger.info(f"Connecting via USB at {device}")
    elif choice == '2':
        device = '/dev/ttyUSB0'
        master = mavutil.mavlink_connection(device=device, baud=57600)
        logger.info(f"Connecting via radio telemetry at {device}")
    elif choice == '3':
        device = 'udpin:127.0.0.1:14550'
        master = mavutil.mavlink_connection(device)
        logger.info(f"Connecting via WiFi telemetry at {device}")
    else:
        logger.error("Invalid choice. Defaulting to USB connection.")
        master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)

    try:
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        print("Heartbeat sent")

        utils.wait_for_heartbeat(master)
        logger.info(f"Connection established with system {master.target_system}, component {master.target_component}")

        try:
            #Request battery status at 2 Hz (every 500000 ms)
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_BATTERY_STATUS, 500000)
            
            # Request position updates at 2 Hz
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000)
            
            # Request attitude updates at 5 Hz
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 200000)
            
            print("\nConnection test successful! Monitoring vehicle state...")
            print("Press Ctrl+C to exit\n")
            
            while True:
                try:
                    lat, lon = utils.get_current_location(master)
                    print(f"Current position: lat={lat:.6f}, lon={lon:.6f}")
                except Exception as e:
                    print(f"Could not get location: {e}")
                
                try:
                    battery = utils.check_battery(master)
                    if battery:
                        voltage, percentage = battery
                        print(f"Battery: {voltage:.2f}V, {percentage}% remaining")
                except Exception as e:
                    print(f"Could not get battery status: {e}")
                
                try:
                    attitude = utils.get_attitude(master)
                    if attitude:
                        roll, pitch, yaw = attitude
                        print(f"Attitude: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
                except Exception as e:
                    print(f"Could not get attitude data: {e}")
                
                print("-" * 50)
                time.sleep(2)
                
        except KeyboardInterrupt:
            print("\nConnection monitoring stopped by user")
            
    except Exception as e:
        logger.error(f"Failed to establish connection: {e}")
        
if __name__ == '__main__':
    main()