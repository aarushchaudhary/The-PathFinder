import cv2
import mediapipe as mp
import pigpio
import time

class FaceTrackingRobot:
    def __init__(self):
        # Initialize MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        self.face_detection = self.mp_face_detection.FaceDetection(min_detection_confidence=0.5)
        
        # Initialize pigpio
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Failed to connect to pigpio daemon. Run 'sudo pigpiod' first.")

        # Servo pins
        self.PAN_PIN = 12   # Adjust these pins based on your wiring
        self.TILT_PIN = 13

        # Motor pins
        self.motor_pins = {
            'ENA': 25,  # Left motor enable
            'IN1': 27,  # Left motor input 1
            'IN2': 22,  # Left motor input 2
            'ENB': 17,  # Right motor enable
            'IN3': 23,  # Right motor input 1
            'IN4': 24   # Right motor input 2
        }

        # Servo parameters
        self.pan_current = 1500   # Starting positions (microseconds)
        self.tilt_current = 1500
        self.SERVO_MIN = 500      # Minimum pulse width
        self.SERVO_MAX = 2500     # Maximum pulse width
        self.STEP = 20            # How much to move per adjustment

        # Movement thresholds
        self.center_threshold = 50    # Pixels within center zone
        self.far_threshold = 120      # Face width below which to move forward
        self.close_threshold = 200    # Face width above which to move backward

        self.setup_hardware()

    def setup_hardware(self):
        # Setup servo pins
        self.pi.set_mode(self.PAN_PIN, pigpio.OUTPUT)
        self.pi.set_mode(self.TILT_PIN, pigpio.OUTPUT)

        # Center servos
        self.pi.set_servo_pulsewidth(self.PAN_PIN, self.pan_current)
        self.pi.set_servo_pulsewidth(self.TILT_PIN, self.tilt_current)

        # Setup motor pins
        for pin in self.motor_pins.values():
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)  # Initialize all pins to LOW

        # Initialize PWM on motor enable pins
        self.pi.set_PWM_frequency(self.motor_pins['ENA'], 1000)  # Set frequency to 1kHz
        self.pi.set_PWM_frequency(self.motor_pins['ENB'], 1000)

    def move_servo(self, servo_pin, current_pos, delta):
        """Moves servo by delta amount, respecting limits."""
        new_pos = current_pos + delta
        new_pos = max(self.SERVO_MIN, min(self.SERVO_MAX, new_pos))
        self.pi.set_servo_pulsewidth(servo_pin, new_pos)
        return new_pos

    def move_forward(self, speed=128):
        """Sets motors to move forward."""
        # Left motor forward (IN1=0, IN2=1)
        self.pi.write(self.motor_pins['IN1'], 0)
        self.pi.write(self.motor_pins['IN2'], 1)
        # Right motor forward (IN3=0, IN4=1)
        self.pi.write(self.motor_pins['IN3'], 0)
        self.pi.write(self.motor_pins['IN4'], 1)
        # Set speed
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def move_backward(self, speed=128):
        """Sets motors to move backward."""
        # Left motor backward (IN1=1, IN2=0)
        self.pi.write(self.motor_pins['IN1'], 1)
        self.pi.write(self.motor_pins['IN2'], 0)
        # Right motor backward (IN3=1, IN4=0)
        self.pi.write(self.motor_pins['IN3'], 1)
        self.pi.write(self.motor_pins['IN4'], 0)
        # Set speed
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def turn_left(self, speed=128):
        """Turns robot left by moving left motor forward and right motor backward."""
        # Left motor forward (IN1=0, IN2=1)
        self.pi.write(self.motor_pins['IN1'], 0)
        self.pi.write(self.motor_pins['IN2'], 1)
        # Right motor backward (IN3=1, IN4=0)
        self.pi.write(self.motor_pins['IN3'], 1)
        self.pi.write(self.motor_pins['IN4'], 0)
        # Set speed
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def turn_right(self, speed=128):
        """Turns robot right by moving right motor forward and left motor backward."""
        # Left motor backward (IN1=1, IN2=0)
        self.pi.write(self.motor_pins['IN1'], 1)
        self.pi.write(self.motor_pins['IN2'], 0)
        # Right motor forward (IN3=0, IN4=1)
        self.pi.write(self.motor_pins['IN3'], 0)
        self.pi.write(self.motor_pins['IN4'], 1)
        # Set speed
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def stop_motors(self):
        """Stops both motors."""
        # Stop left motor
        self.pi.write(self.motor_pins['IN1'], 0)
        self.pi.write(self.motor_pins['IN2'], 0)
        # Stop right motor
        self.pi.write(self.motor_pins['IN3'], 0)
        self.pi.write(self.motor_pins['IN4'], 0)
        # Disable motor speed
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], 0)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], 0)

    def track_face(self, frame):
        """Process frame, adjust servos, and control motors to track and follow face."""
        # Convert to RGB for MediaPipe
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, _ = frame.shape
        frame_center_x = w // 2
        frame_center_y = h // 2

        # Detect faces
        results = self.face_detection.process(rgb_frame)

        if results.detections:
            # Get first detected face
            detection = results.detections[0]
            bbox = detection.location_data.relative_bounding_box

            # Convert relative coordinates to pixel coordinates
            face_x = int((bbox.xmin + bbox.width / 2) * w)
            face_y = int((bbox.ymin + bbox.height / 2) * h)
            x = int(bbox.xmin * w)
            y = int(bbox.ymin * h)
            width = int(bbox.width * w)
            height = int(bbox.height * h)

            # Draw face box and center point (for debugging)
            cv2.rectangle(frame, (x, y), (x + width, y + height), (0, 255, 0), 2)
            cv2.circle(frame, (face_x, face_y), 5, (0, 0, 255), -1)
            cv2.circle(frame, (frame_center_x, frame_center_y), 5, (255, 0, 0), -1)

            # Calculate error from center
            error_x = frame_center_x - face_x  # Positive if face is left of center
            error_y = frame_center_y - face_y  # Positive if face is above center

            # Move servos based on error
            if abs(error_x) > 20:  # Pan adjustment with deadzone
                delta_x = self.STEP if error_x > 0 else -self.STEP
                self.pan_current = self.move_servo(self.PAN_PIN, self.pan_current, delta_x)

            if abs(error_y) > 20:  # Tilt adjustment with deadzone
                delta_y = self.STEP if error_y < 0 else -self.STEP  # Inverted due to coordinate system
                self.tilt_current = self.move_servo(self.TILT_PIN, self.tilt_current, delta_y)

            # Draw error lines (for debugging)
            cv2.line(frame, (frame_center_x, frame_center_y), (face_x, face_y), (0, 255, 255), 2)

            # Motor control logic
            # Horizontal movement (turning)
            if abs(error_x) > self.center_threshold:
                # Calculate proportional speed based on error
                max_error = frame_center_x
                turn_speed = int((abs(error_x) / max_error) * 255)
                turn_speed = min(max(turn_speed, 50), 255)  # Clamp speed between 50 and 255

                if error_x > 0:
                    self.turn_left(turn_speed)
                    print(f"Turning left at speed {turn_speed}")
                else:
                    self.turn_right(turn_speed)
                    print(f"Turning right at speed {turn_speed}")
            else:
                # Face is centered horizontally
                # Movement based on face size
                if width < self.far_threshold:
                    # Face is small; move forward
                    forward_speed = 128  # Adjust speed as needed
                    self.move_forward(forward_speed)
                    print(f"Moving forward at speed {forward_speed}")
                elif width > self.close_threshold:
                    # Face is large; move backward
                    backward_speed = 128  # Adjust speed as needed
                    self.move_backward(backward_speed)
                    print(f"Moving backward at speed {backward_speed}")
                else:
                    # Face is within acceptable range; stop
                    self.stop_motors()
                    print("Face is at optimal distance. Stopping motors.")

            # Add text showing servo positions (for debugging)
            cv2.putText(frame, f"Pan: {self.pan_current}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Tilt: {self.tilt_current}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # No face detected; stop motors
            self.stop_motors()
            print("No face detected. Stopping motors.")

        return frame

    def run(self):
        print("Initializing camera...")
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        if not cap.isOpened():
            raise RuntimeError("Failed to open camera")

        print("Starting face tracking. Press 'q' to quit.")

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Failed to read frame")
                    break

                # Process frame and track face
                processed_frame = self.track_face(frame)

                # Display frame
                cv2.imshow('Face Tracking Debug', processed_frame)

                # Break loop if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Small delay to prevent overwhelming the servos
                time.sleep(0.01)

        finally:
            print("Cleaning up...")
            cap.release()
            cv2.destroyAllWindows()
            self.cleanup()

    def cleanup(self):
        # Center servos
        self.pi.set_servo_pulsewidth(self.PAN_PIN, 1500)
        self.pi.set_servo_pulsewidth(self.TILT_PIN, 1500)
        time.sleep(0.5)  # Give servos time to center

        # Stop motors
        self.stop_motors()

        # Cleanup GPIO
        self.pi.stop()

if __name__ == "__main__":
    try:
        robot = FaceTrackingRobot()
        robot.run()
    except Exception as e:
        print(f"Error: {e}")
