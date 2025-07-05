# Import the necessary libraries
from gpiozero import Servo
from time import sleep

# --- Configuration ---
# Set the GPIO pin you are using. You mentioned you're using pin 14.
# Physical pin 8 is GPIO 14.
SERVO_PIN = 14

# --- Advanced Configuration (Pulse Widths) ---
# Most servos operate on a pulse width between 1ms and 2ms.
# 1ms (0.001s) is typically the minimum angle (0 degrees).
# 1.5ms (0.0015s) is the middle angle (90 degrees).
# 2ms (0.002s) is the maximum angle (180 degrees).
#
# However, some servos (especially continuous rotation ones or clones)
# have different ranges. The gpiozero library defaults may not work perfectly.
#
# We will create a Servo object and tell it NOT to move initially.
# By setting initial_value=None, the servo will not be sent any signal
# until we explicitly set its value in the loop. This prevents it from
# spinning immediately when the script starts.
servo = Servo(
    SERVO_PIN,
    initial_value=None,  # <-- This is the crucial fix
    min_pulse_width=1/1000,
    max_pulse_width=2/1000
)

# --- Calibration Script ---
# This script will help you find the exact values for your servo.
# Run the script and enter values between -1.0 and 1.0 at the prompt.
#
# For a STANDARD SERVO:
# - Find the value that makes it go to 0 degrees (usually around -1.0).
# - Find the value that makes it go to 180 degrees (usually around 1.0).
#
# For a CONTINUOUS ROTATION SERVO:
# - Find the value that makes the servo STOP. This is the most important value. It's usually very close to 0.
# - Values below the 'stop' point will make it spin in one direction.
# - Values above the 'stop' point will make it spin in the other direction.

try:
    print("--- Servo Calibration Script ---")
    print(f"Servo connected to GPIO pin {SERVO_PIN}")
    print("The servo should be stopped now.")
    print("Enter a value between -1.0 and 1.0 to set the servo position/speed.")
    print("Find the value that makes the servo stop spinning (try 0, 0.01, -0.01, etc.).")
    print("Press Ctrl+C to exit.")

    while True:
        # Get user input from the terminal
        user_input = input("Enter servo value (-1.0 to 1.0): ")

        try:
            # Convert the input string to a floating-point number
            value = float(user_input)

            # Ensure the value is within the valid range
            if -1.0 <= value <= 1.0:
                print(f"Setting servo to {value}")
                servo.value = value
            else:
                print("Invalid input. Please enter a number between -1.0 and 1.0.")

        except ValueError:
            print("Invalid input. Please enter a number.")

except KeyboardInterrupt:
    print("\nProgram stopped by user.")
finally:
    # It's good practice to release the GPIO resources.
    # Detach the servo to stop sending pulses before exiting.
    if servo.value is not None:
        servo.detach()
    print("GPIO cleanup complete. Servo control finished.")
