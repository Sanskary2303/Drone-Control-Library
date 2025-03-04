import time
import logging
import argparse
from pymavlink import mavutil
import utils

def main():
    parser = argparse.ArgumentParser(description='Monitor and interact with a Pixhawk device')
    parser.add_argument('--device', default='/dev/ttyACM0', help='Serial device or connection string (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=57600, help='Baud rate (default: 57600)')
    parser.add_argument('--mode', default='monitor', choices=['monitor', 'info', 'raw'], 
                        help='Operation mode: monitor (filtered info), info (vehicle information), raw (all messages)')
    
    args = parser.parse_args()
    
    logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
    logger = logging.getLogger(__name__)
    
    logger.info(f"Connecting to {args.device} at {args.baud} baud...")
    
    try:
        if args.device.startswith(('udpin:', 'udpout:', 'tcp:')):
            master = mavutil.mavlink_connection(args.device)
        else:
            master = mavutil.mavlink_connection(device=args.device, baud=args.baud)
            
        utils.wait_for_heartbeat(master)
        logger.info(f"Connected to system {master.target_system}, component {master.target_component}")
        
        if args.mode == 'info':
            info = utils.get_drone_info(master)
            
            # Display formatted information
            print("\n===== Drone Information =====")
            print(f"Vehicle Type:     {info['vehicle_type']}")
            print(f"Autopilot:        {info['autopilot_type']}")
            print(f"Firmware Version: {info['firmware_version']}")
            print(f"System ID:        {info['system_id']}")
            print(f"Component ID:     {info['component_id']}")
            print(f"Armed:            {'Yes' if info['armed'] else 'No'}")
            print(f"Mode:             {info['flight_mode']}")
            
            if 'latitude' in info and info['latitude'] != 'Unavailable':
                print(f"Location:         {info['latitude']}, {info['longitude']}")
            else:
                print("Location:         Unavailable")
                
            if 'battery_voltage' in info:
                print(f"Battery:          {info['battery_voltage']} ({info['battery_remaining']})")
            
            if 'capabilities' in info and info['capabilities']:
                print("\nCapabilities:")
                for capability in info['capabilities']:
                    print(f"- {capability}")
                    
        elif args.mode == 'monitor':
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1000000)  # 1 Hz
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000)  # 2 Hz
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 200000)  # 5 Hz
            utils.set_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000)  # 1 Hz
            
            print("\n===== Monitoring Vehicle Status =====")
            print("Press Ctrl+C to exit\n")
            
            while True:
                try:
                    mode = utils.get_current_mode(master)
                    print(f"Mode: {mode}")
                    
                    try:
                        lat, lon = utils.get_current_location(master)
                        print(f"Position: Lat={lat:.7f}, Lon={lon:.7f}")
                    except:
                        print("Position: Not available")
                    
                    battery = utils.check_battery(master)
                    if battery:
                        voltage, percentage = battery
                        print(f"Battery: {voltage:.2f}V ({percentage}%)")
                    
                    attitude = utils.get_attitude(master)
                    if attitude:
                        roll, pitch, yaw = attitude
                        print(f"Attitude: Roll={roll:.2f}, Pitch={pitch:.2f}, Yaw={yaw:.2f}")
                    
                    print("-" * 40)
                    time.sleep(1)
                    
                except KeyboardInterrupt:
                    print("\nExiting...")
                    break
                except Exception as e:
                    logger.error(f"Error in monitor mode: {e}")
                    time.sleep(1)
                    
        else:  # Raw mode
            print("\n===== Raw MAVLink Messages =====")
            print("Press Ctrl+C to exit\n")
            
            # Display all incoming messages
            while True:
                try:
                    msg = master.recv_match(blocking=True, timeout=1.0)
                    if msg:
                        # Filter out high-frequency messages to avoid flooding
                        if msg.get_type() not in ["BAD_DATA", "TIMESYNC", "SYSTEM_TIME", "PARAM_VALUE"]:
                            print(f"{msg.get_type()}: {msg.to_dict()}")
                except KeyboardInterrupt:
                    print("\nExiting...")
                    break
                except Exception as e:
                    logger.error(f"Error in raw mode: {e}")
    
    except Exception as e:
        logger.error(f"Error connecting to {args.device}: {e}")
        
if __name__ == "__main__":
    main()