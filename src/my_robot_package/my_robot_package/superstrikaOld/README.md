# Superstrika Robot Motor Control System

This package provides ROS 2 nodes for controlling a 4-wheel omnidirectional robot using Raspberry Pi GPIO pins. The system supports both direct motor speed control and vector-based movement control.

## Features

- Control 4 DC motors using Raspberry Pi GPIO pins
- Support for both direct speed control and vector-based movement
- PWM-based motor control with configurable range (0-1024)
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

## ROS 2 Topics

### Subscribed Topics
- `motorSpeed` (Float32MultiArray): Direct motor speed control
  - Array of 4 values representing speed for each motor (-100 to 100)
- `vectorMovement` (Float32MultiArray): Vector-based movement control
  - First value: Angle in degrees
  - Second value: Magnitude (0-1)
  - Third value: Rotation value

### Published Topics
- `motorSpeed` (Float32MultiArray): Current motor speeds
  - Array of 4 values representing current speed of each motor

## Usage

### Direct Motor Control
To control motors directly, publish to the `motorSpeed` topic with an array of 4 values:
```python
# Example: Set all motors to 50% speed
speed_msg = Float32MultiArray()
speed_msg.data = [50.0, 50.0, 50.0, 50.0]
```

### Vector Movement Control
For vector-based movement, publish to the `vectorMovement` topic:
```python
# Example: Move at 45 degrees with 0.5 magnitude and no rotation
vector_msg = Float32MultiArray()
vector_msg.data = [45.0, 0.5, 0.0]
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
2. Clone this package into your ROS 2 workspace
3. Build the package using colcon:
```bash
colcon build --packages-select my_robot_package
```

## Running the Node

To start the motor control node:
```bash
ros2 run my_robot_package motors
```

## Dependencies

- ROS 2
- RPi.GPIO
- Python 3
- rclpy

## Contributing

This project is maintained by:
- [Noam Ron](https://github.com/NoamRon1)
- [Itamar Hoter Ishay](https://github.com/ItamarHoter)

## Servo Motor Control

The system includes servo motor control capabilities through the `servoNode.py` and `servo.py` files.

### Hardware Requirements
- Servo motor connected to a GPIO pin
- Raspberry Pi GPIO access

### ROS 2 Topics

#### Subscribed Topics
- `servoAngle` (Float32): Servo motor angle control
  - Value represents the desired angle for the servo motor

### Usage

To control the servo motor, publish to the `servoAngle` topic:
```python
# Example: Set servo to 90 degrees
angle_msg = Float32()
angle_msg.data = 90.0
```

## LCD Screen Display

The system includes an LCD screen display functionality through the `screenNode.py` file.

### Hardware Requirements
- I2C LCD screen (PCF8574)
- Raspberry Pi with I2C enabled

### ROS 2 Topics

#### Subscribed Topics
- `printScreen` (StringMultiArray): LCD screen content control
  - Array of strings representing lines to display on the LCD screen

### Usage

To display content on the LCD screen, publish to the `printScreen` topic:
```python
# Example: Display two lines of text
screen_msg = StringMultiArray()
screen_msg.data = ["Line 1", "Line 2"]
```

### LCD Screen Configuration
- Default I2C address: 0x27
- Default size: 20x4 characters
- Supports auto line breaks
- Backlight enabled by default

## Running Additional Nodes

To start the servo control node:
```bash
ros2 run my_robot_package servo
```

To start the LCD screen node:
```bash
ros2 run my_robot_package screen
```

## Additional Dependencies

- RPLCD (for LCD screen control)
- I2C tools (for LCD screen communication)
