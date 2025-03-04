# Drone Control Library

A comprehensive Python library for controlling drones using the MAVLink protocol. This project provides a collection of utility functions and scripts for interacting with flight controllers like Pixhawk.

## Features

- Multiple connection methods (USB, radio telemetry, WiFi)
- Vehicle information retrieval and monitoring
- Basic flight operations (arm, takeoff, land, RTL)
- Waypoint navigation (global and local coordinates)
- Velocity-based movement control
- Real-time attitude and battery monitoring
- Comprehensive utility functions for MAVLink operations

## Installation

1. Clone this repository:
    ```bash
    git clone https://github.com/yourusername/offboard.git
    cd offboard
    ```

2. Install required dependencies:
    ```bash
    pip install pymavlink
    ```

## Usage

### Connection

Connect to your drone using different methods:

```bash
python check_connection.py
```

This will prompt you to select a connection method (USB, radio telemetry, or WiFi).

### Vehicle Information

Get detailed information about your drone:

```bash
python pixhawk.py --mode info
```

### Flight Operations

#### Arming the vehicle

```bash
python arming.py
```

#### Taking off

```bash
python takeoff.py
```

#### Landing

```bash
python land.py
```

#### Moving with velocity commands

```bash
python moving_with_velocity.py
```

#### Navigating to local waypoints

```bash
python local_waypoints.py
```

## File Descriptions

- `utils.py`: Core utility functions for MAVLink operations
- `pixhawk.py`: Main interface for monitoring and interacting with Pixhawk devices
- `check_connection.py`: Utility to test different connection methods
- `arming.py`: Simple script to arm the vehicle
- `takeoff.py`: Performs a guided takeoff to a specified altitude
- `land.py`: Commands the vehicle to land
- `moving_with_velocity.py`: Demonstrates controlling the vehicle using velocity commands
- `local_waypoints.py`: Example of navigating to waypoints using local coordinates

## Connection Methods

The library supports three main connection methods to establish communication with your drone:

### 1. USB Connection

Direct connection via USB cable to the flight controller:

```python
# Connect to Pixhawk via USB
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
```

- **Windows**: Use `'COMx'` format (e.g., `'COM3'`)
- **Linux/macOS**: Use `/dev/ttyACMx` or `/dev/ttyUSBx` format

### 2. Radio Telemetry

Connect using SiK radio telemetry modules:

```python
# Connect via telemetry radio
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
```

Common baud rates: 57600 (default), 115200

### 3. WiFi Telemetry

Connect over WiFi using UDP or TCP:

```python
# Standard WiFi connection (default IP/port)
master = mavutil.mavlink_connection('udp:192.168.4.1:14550')

# Custom IP and port
master = mavutil.mavlink_connection('udp:10.0.0.100:5760')

# Connect to simulator
master = mavutil.mavlink_connection('udpin:localhost:14550')
```

#### WiFi Connection Details

- **Protocol formats**:
    - `udp:<ip>:<port>` - UDP outgoing connection
    - `udpin:<ip>:<port>` - UDP incoming connection
    - `tcp:<ip>:<port>` - TCP connection

- **Common scenarios**:
    - Default ESP8266/ESP32 connection: `udp:192.168.4.1:14550`
    - Custom network: Replace with your drone's actual IP address
    - Multiple drones: Use different ports (e.g., 14550, 14551, etc.)

**Example using variables:**

```python
ip_address = '192.168.4.1'  # Replace with your drone's IP
port = 14550               # Default MAVLink port

# Create the connection
master = mavutil.mavlink_connection(f'udp:{ip_address}:{port}')

# Wait for the connection to be established
master.wait_heartbeat()
print("Connection successful!")
```

For all connection methods, always verify the connection by waiting for heartbeat messages before sending commands.

## Advanced Usage

### Monitoring Vehicle Status

```bash
python pixhawk.py --mode monitor
```

This provides a real-time display of important vehicle metrics including position, battery status, and attitude.

### Raw MAVLink Message Monitoring

```bash
python pixhawk.py --mode raw
```

This displays all incoming MAVLink messages for debugging or development purposes.

## Examples

### Simple Take Off and Land Sequence

```python
import time
from pymavlink import mavutil
import utils

# Connect to the vehicle
master = mavutil.mavlink_connection(device='/dev/ttyACM0', baud=57600)
utils.wait_for_heartbeat(master)

# Set mode to GUIDED
utils.set_mode(master, 'GUIDED')

# Arm the vehicle
utils.arm_vehicle(master)

# Take off to 5 meters
utils.takeoff(master, 5)
time.sleep(10)  # Wait to reach altitude

# Land
utils.land(master)
```

## Requirements

- Python 3.6+
- pymavlink library
- A MAVLink-compatible flight controller (e.g., Pixhawk)

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## License

This project is licensed under the MIT License - see the LICENSE file for details.