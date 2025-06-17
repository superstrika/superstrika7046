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
