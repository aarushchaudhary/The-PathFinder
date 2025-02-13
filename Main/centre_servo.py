import pigpio
import time

# Define GPIO pins for the servos
servo1_pin = 12
servo2_pin = 13

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    exit()

# Set GPIO pins as outputs
pi.set_mode(servo1_pin, pigpio.OUTPUT)
pi.set_mode(servo2_pin, pigpio.OUTPUT)

# Function to set servo position (pulse width in microseconds)
def set_servo_position(pin, pulse_width):
    pi.set_servo_pulsewidth(pin, pulse_width)

# Center the servos (typically around 1500 microseconds)
center_pulse_width = 1500

# Set the servos to the center position
set_servo_position(servo1_pin, center_pulse_width)
set_servo_position(servo2_pin, center_pulse_width)

print("Servos centered.")

# Keep the servos at the center position for a while (optional)
time.sleep(2)  # Adjust the duration as needed

# You can add code here to control the servos later if required.
# For example, to move a servo:
# set_servo_position(servo1_pin, 1700) # Example: Move servo1 to a new position

# To prevent jitter you can set the pulsewidth to 0 to stop sending pulses after you are done
# pi.set_servo_pulsewidth(servo1_pin, 0)
# pi.set_servo_pulsewidth(servo2_pin, 0)

# Stop pigpio and release resources (important to do this when finished)
pi.stop()

print("pigpio stopped. Program finished.")