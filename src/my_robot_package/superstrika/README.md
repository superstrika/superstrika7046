# Superstrika Robot Motor Control System

This package provides ROS 2 nodes for controlling a 4-wheel omnidirectional robot and a servo using Raspberry Pi GPIO pins, now utilizing the `gpiozero` library for all motor and servo control. The system supports both direct motor speed control and vector-based movement control, as well as LCD screen display via I2C.

## Features

- Control 4 DC motors using Raspberry Pi GPIO pins via `gpiozero`
- Support for both direct speed control and vector-based movement
- PWM-based motor control with configurable range (0-1024)
- Servo motor control using `gpiozero`
- LCD screen display via I2C (PCF8574)
- ROS 2 integration for easy communication with other nodes
- Omnidirectional movement support with rotation control

## Hardware Requirements

- Raspberry Pi (with GPIO access)
- 4 DC motors with motor drivers
- Motor driver connections to GPIO pins:
  - Motor 1: Pins 2, 3
  - Motor 2: Pins 17, 27
  - Motor 3: Pins 14, 15
  - Motor 4: Pins 23, 24
- Servo motor connected to a GPIO pin
- I2C LCD screen (PCF8574, default address 0x27)

## Python Dependencies

- ROS 2 (rclpy)
- gpiozero (for DC and servo motors)
- RPLCD (for LCD screen control)
- Python 3

## ROS 2 Topics

### Subscribed Topics
- `motorSpeed` (`Float32MultiArray`): Direct motor speed control
  - Array of 4 values representing speed for each motor (-100 to 100)
- `vectorMovement` (`Float32MultiArray`): Vector-based movement control
  - [angle (deg), magnitude (0-1), rotation]
- `servoAngle` (`Float32`): Servo motor angle control
  - Value represents the desired angle for the servo motor
- `printScreen` (`StringMultiArray`): LCD screen content control
  - Array of strings representing lines to display on the LCD screen

### Published Topics
- `motorSpeed` (`Float32MultiArray`): Current motor speeds
  - Array of 4 values representing current speed of each motor

## Usage

### Quick Topic Publish Commands

Publish direct motor speeds:
```bash
ros2 topic pub /motorSpeed std_msgs/Float32MultiArray '{data: [50.0, 50.0, 50.0, 50.0]}'
```

Publish vector movement:
```bash
ros2 topic pub /vectorMovement std_msgs/Float32MultiArray '{data: [45.0, 0.5, 0.0]}'
```

Publish servo angle:
```bash
ros2 topic pub /servoAngle std_msgs/Float32 '{data: 90.0}'
```

Publish LCD screen lines:
```bash
ros2 topic pub /printScreen std_msgs/StringMultiArray '{data: ["Line 1", "Line 2"]}'
```

---

### Direct Motor Control
To control motors directly, publish to the `motorSpeed` topic with an array of 4 values:
```python
from std_msgs.msg import Float32MultiArray
# Example: Set all motors to 50% speed
speed_msg = Float32MultiArray()
speed_msg.data = [50.0, 50.0, 50.0, 50.0]
# Publish to 'motorSpeed'
```

### Vector Movement Control
For vector-based movement, publish to the `vectorMovement` topic:
```python
from std_msgs.msg import Float32MultiArray
# Example: Move at 45 degrees with 0.5 magnitude and no rotation
vector_msg = Float32MultiArray()
vector_msg.data = [45.0, 0.5, 0.0]
# Publish to 'vectorMovement'
```

### Servo Motor Control
To control the servo motor, publish to the `servoAngle` topic:
```python
from std_msgs.msg import Float32
# Example: Set servo to 90 degrees
angle_msg = Float32()
angle_msg.data = 90.0
# Publish to 'servoAngle'
```

### LCD Screen Display
To display content on the LCD screen, publish to the `printScreen` topic:
```python
from std_msgs.msg import StringMultiArray
# Example: Display two lines of text
screen_msg = StringMultiArray()
screen_msg.data = ["Line 1", "Line 2"]
# Publish to 'printScreen'
```

## Motor Speed Calculation

The system uses the following formulas for vector-based movement:
- vx = magnitude * cos(angle - 45°)
- vy = magnitude * sin(angle - 45°)
- Wheel speeds are calculated as:
  - Wheel 1: vy + rotation
  - Wheel 2: vx - rotation
  - Wheel 3: vx + rotation
  - Wheel 4: vy - rotation

## Visual Speed Display

The system includes a visual display of motor speeds using ASCII art and ANSI colors. The `printSpeed` function creates a wheel layout display showing the current speed of each motor:

```
         WHEEL SPEEDS
        ┌───────────────────┐
        │  [1]         [2]  │
        │ (050)       (050) │
        │    \       /      │
        │     \     /       │
        │     /     \       │         
        │    /       \      │
        │ (050)       (050) │
        │  [4]         [3]  │
        └───────────────────┘
```

The display features:
- Color-coded speed values (green)
- Wheel numbers in brackets
- Visual representation of wheel positions
- Real-time updates of motor speeds

## Installation

1. Ensure you have ROS 2 installed on your Raspberry Pi
2. Install Python dependencies:
   ```bash
   pip install gpiozero RPLCD
   ```
3. Clone this package into your ROS 2 workspace
4. Build the package using colcon:
   ```bash
   colcon build --packages-select my_robot_package
   ```

## Running the Nodes

To start the motor control node:
```bash
ros2 run my_robot_package motors
```

To start the servo control node:
```bash
ros2 run my_robot_package servo
```

To start the LCD screen node:
```bash
ros2 run my_robot_package screen
```

## Node Details

### Motor Node (`motorNode.py`)
- Uses `gpiozero.PWMLED` for PWM control of each motor pin.
- Motor pins are set as follows:
  - Motor 1: (2, 3)
  - Motor 2: (17, 27)
  - Motor 3: (14, 15)
  - Motor 4: (23, 24)
- Accepts speed values in the range -100 to 100, mapped to PWM (0-1024).
- Supports both direct speed and vector-based movement.

### Servo Node (`servoNode.py`)
- Uses `gpiozero.PWMLED` for servo PWM control.
- Accepts angle values (e.g., 0-180 degrees).
- Example initialization: `ServoMotor(servoPin)`

### LCD Screen Node (`screenNode.py`)
- Uses `RPLCD.i2c.CharLCD` for I2C LCD control.
- Default I2C address: 0x27
- Default size: 20x4 characters
- Subscribes to `printScreen` topic for display updates.

## Additional Notes

- All GPIO and PWM control is now handled via the `gpiozero` library for simplicity and reliability.
- Ensure your user is in the `gpio` group for hardware access on Raspberry Pi.
- For I2C LCD, enable I2C on your Raspberry Pi (`raspi-config`).

## Contributing

This project is maintained by:
- [Noam Ron](https://github.com/NoamRon1)
- [Itamar Hoter Ishay](https://github.com/ItamarHoter)
