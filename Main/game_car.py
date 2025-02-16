import pigpio
import pygame
import sys
import cv2
import numpy as np

# Initialize pygame and joystick
pygame.init()
pygame.joystick.init()

# Check for joystick
if pygame.joystick.get_count() == 0:
    print("No joystick detected. Please connect an Xbox controller.")
    sys.exit()

# Initialize the first joystick
joystick = pygame.joystick.Joystick(0)
joystick.init()

# Open a pygame window to capture events
screen_width, screen_height = 640, 480  # Adjust to match your webcam resolution
screen = pygame.display.set_mode((screen_width, screen_height))
pygame.display.set_caption("Robot Control with Camera Feed")

# Initialize pigpio
pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("Failed to connect to pigpio daemon")

# Motor control pins (using BCM numbering)
PINS = {
    'ENA': 25,  # PWM control for Motor A
    'IN1': 27,  # Direction control for Motor A
    'IN2': 22,  # Direction control for Motor A
    'IN3': 23,  # Direction control for Motor B
    'IN4': 24,  # Direction control for Motor B
    'ENB': 17   # PWM control for Motor B
}

# Servo control pins
PAN_PIN = 12   # Servo controlling pan movement
TILT_PIN = 13  # Servo controlling tilt movement

# Initialize all motor pins
for pin in PINS.values():
    pi.set_mode(pin, pigpio.OUTPUT)
    pi.write(pin, 0)  # Initialize all pins to LOW

# Initialize servo pins
pi.set_mode(PAN_PIN, pigpio.OUTPUT)
pi.set_mode(TILT_PIN, pigpio.OUTPUT)
# Set servos to initial position (centered)
pi.set_servo_pulsewidth(PAN_PIN, 1500)
pi.set_servo_pulsewidth(TILT_PIN, 1500)

# Set PWM frequency for motors
pi.set_PWM_frequency(PINS['ENA'], 1000)  # 1kHz frequency for smoother control
pi.set_PWM_frequency(PINS['ENB'], 1000)

# Initialize webcam capture
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, screen_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, screen_height)

if not cap.isOpened():
    print("Cannot open camera")
    sys.exit()

# Function to set motor speed
def set_speed(speed_percent):
    # Clamp speed to 0-100%
    speed_percent = max(0, min(100, speed_percent))
    # Convert percentage to PWM duty cycle (0-255)
    pwm_value = int(speed_percent * 2.55)
    pi.set_PWM_dutycycle(PINS['ENA'], pwm_value)
    pi.set_PWM_dutycycle(PINS['ENB'], pwm_value)

# Movement functions for the robot
def move_forward():
    pi.write(PINS['IN1'], 0)
    pi.write(PINS['IN2'], 1)
    pi.write(PINS['IN3'], 0)
    pi.write(PINS['IN4'], 1)

def move_backward():
    pi.write(PINS['IN1'], 1)
    pi.write(PINS['IN2'], 0)
    pi.write(PINS['IN3'], 1)
    pi.write(PINS['IN4'], 0)

def turn_left():
    pi.write(PINS['IN1'], 0)
    pi.write(PINS['IN2'], 1)
    pi.write(PINS['IN3'], 1)
    pi.write(PINS['IN4'], 0)

def turn_right():
    pi.write(PINS['IN1'], 1)
    pi.write(PINS['IN2'], 0)
    pi.write(PINS['IN3'], 0)
    pi.write(PINS['IN4'], 1)

def stop_motors():
    pi.write(PINS['IN1'], 0)
    pi.write(PINS['IN2'], 0)
    pi.write(PINS['IN3'], 0)
    pi.write(PINS['IN4'], 0)

# Function to set servo angle based on joystick input
def set_servo_angle(axis_value, servo_pin):
    # Invert axis_value for pan control if necessary
    if servo_pin == PAN_PIN:
        axis_value = -axis_value  # Invert to correct servo direction

    # Convert joystick axis (-1 to 1) to pulse width (1000 to 2000 microseconds)
    # Adjust pulse width range if necessary for your servos
    pulse_width = 1500 + (axis_value * 500)  # Center at 1500µs, range ±500µs
    # Clamp pulse width to valid range
    pulse_width = max(1000, min(2000, pulse_width))
    pi.set_servo_pulsewidth(servo_pin, pulse_width)

# Cleanup function
def cleanup():
    stop_motors()
    set_speed(0)
    # Stop servos
    pi.set_servo_pulsewidth(PAN_PIN, 0)
    pi.set_servo_pulsewidth(TILT_PIN, 0)
    # Release the camera
    cap.release()
    pi.stop()
    pygame.quit()

# Main control loop
try:
    clock = pygame.time.Clock()
    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt

        # Get joystick axes values
        # Left joystick for pan and tilt
        left_stick_x = joystick.get_axis(0)   # Left stick horizontal axis
        left_stick_y = -joystick.get_axis(1)  # Left stick vertical axis (invert if necessary)

        # Right joystick for movement
        right_stick_x = joystick.get_axis(3)    # Right stick horizontal axis
        right_stick_y = -joystick.get_axis(4)   # Invert Y-axis for correct direction

        # Apply a deadzone to joysticks to prevent drift
        deadzone = 0.2
        # Left joystick deadzone
        if abs(left_stick_x) < deadzone:
            left_stick_x = 0
        if abs(left_stick_y) < deadzone:
            left_stick_y = 0
        # Right joystick deadzone
        if abs(right_stick_x) < deadzone:
            right_stick_x = 0
        if abs(right_stick_y) < deadzone:
            right_stick_y = 0

        # Get RT trigger value
        rt_trigger = joystick.get_axis(5)  # Axis 5 for RT trigger (adjust if necessary)

        # Normalize RT value to 0-100%
        # The RT axis typically ranges from -1.0 (not pressed) to 1.0 (fully pressed)
        rt_value = ((-rt_trigger + 1) / 2) * 100  # Invert and scale

        # Set speed based on RT trigger
        set_speed(rt_value)

        # Determine movement based on right joystick
        if right_stick_y > deadzone:
            move_forward()
        elif right_stick_y < -deadzone:
            move_backward()
        elif right_stick_x < -deadzone:
            turn_left()
        elif right_stick_x > deadzone:
            turn_right()
        else:
            stop_motors()

        # Control servos with left joystick
        set_servo_angle(left_stick_x, PAN_PIN)   # Pan control
        set_servo_angle(left_stick_y, TILT_PIN)  # Tilt control

        # Capture frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            continue

        # Convert the image from OpenCV BGR format to Pygame Surface
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame = np.fliplr(frame)  # Flip horizontally if necessary
        frame = np.rot90(frame)   # Rotate if necessary

        # Create a Pygame surface from the image
        frame_surface = pygame.surfarray.make_surface(frame)

        # Blit the frame to the screen
        screen.blit(frame_surface, (0, 0))

        # Update the display
        pygame.display.update()

        # Limit the loop to 60 frames per second
        clock.tick(60)

except KeyboardInterrupt:
    print("Exiting program...")

finally:
    cleanup()
