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

        # Only using tilt servo now
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

        # Servo parameters (only for tilt)
        self.tilt_current = 1500
        self.SERVO_MIN = 500
        self.SERVO_MAX = 2500
        self.STEP = 20

        # Movement thresholds
        self.center_threshold = 50    # Pixels within center zone
        self.far_threshold = 120      # Face width below which to move forward
        self.close_threshold = 200    # Face width above which to move backward
        
        # Turn speed parameters
        self.MIN_TURN_SPEED = 50
        self.MAX_TURN_SPEED = 255

        self.setup_hardware()

    def setup_hardware(self):
        # Setup tilt servo pin
        self.pi.set_mode(self.TILT_PIN, pigpio.OUTPUT)

        # Center tilt servo
        self.pi.set_servo_pulsewidth(self.TILT_PIN, self.tilt_current)

        # Setup motor pins
        for pin in self.motor_pins.values():
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)  # Initialize all pins to LOW

        # Initialize PWM on motor enable pins
        self.pi.set_PWM_frequency(self.motor_pins['ENA'], 1000)
        self.pi.set_PWM_frequency(self.motor_pins['ENB'], 1000)

    def move_servo(self, servo_pin, current_pos, delta):
        """Moves servo by delta amount, respecting limits."""
        new_pos = current_pos + delta
        new_pos = max(self.SERVO_MIN, min(self.SERVO_MAX, new_pos))
        self.pi.set_servo_pulsewidth(servo_pin, new_pos)
        return new_pos

    def move_forward(self, speed=128):
        """Sets motors to move forward."""
        self.pi.write(self.motor_pins['IN1'], 0)
        self.pi.write(self.motor_pins['IN2'], 1)
        self.pi.write(self.motor_pins['IN3'], 0)
        self.pi.write(self.motor_pins['IN4'], 1)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def move_backward(self, speed=128):
        """Sets motors to move backward."""
        self.pi.write(self.motor_pins['IN1'], 1)
        self.pi.write(self.motor_pins['IN2'], 0)
        self.pi.write(self.motor_pins['IN3'], 1)
        self.pi.write(self.motor_pins['IN4'], 0)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def turn_left(self, speed=128):
        """Turns robot left."""
        self.pi.write(self.motor_pins['IN1'], 0)
        self.pi.write(self.motor_pins['IN2'], 1)
        self.pi.write(self.motor_pins['IN3'], 1)
        self.pi.write(self.motor_pins['IN4'], 0)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def turn_right(self, speed=128):
        """Turns robot right."""
        self.pi.write(self.motor_pins['IN1'], 1)
        self.pi.write(self.motor_pins['IN2'], 0)
        self.pi.write(self.motor_pins['IN3'], 0)
        self.pi.write(self.motor_pins['IN4'], 1)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], speed)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], speed)

    def stop_motors(self):
        """Stops both motors."""
        self.pi.write(self.motor_pins['IN1'], 0)
        self.pi.write(self.motor_pins['IN2'], 0)
        self.pi.write(self.motor_pins['IN3'], 0)
        self.pi.write(self.motor_pins['IN4'], 0)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENA'], 0)
        self.pi.set_PWM_dutycycle(self.motor_pins['ENB'], 0)

    def calculate_turn_speed(self, error, frame_width):
        """Calculate proportional turn speed based on error."""
        # Normalize error to percentage of frame width
        error_percentage = abs(error) / (frame_width / 2)
        # Calculate speed with minimum threshold
        speed = int(error_percentage * self.MAX_TURN_SPEED)
        return max(min(speed, self.MAX_TURN_SPEED), self.MIN_TURN_SPEED)

    def track_face(self, frame):
        """Process frame and control robot to track and follow face."""
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

            # Calculate errors
            error_x = frame_center_x - face_x
            error_y = frame_center_y - face_y

            # Vertical tracking with tilt servo
            if abs(error_y) > 20:
                delta_y = self.STEP if error_y < 0 else -self.STEP
                self.tilt_current = self.move_servo(self.TILT_PIN, self.tilt_current, delta_y)

            # Draw error lines (for debugging)
            cv2.line(frame, (frame_center_x, frame_center_y), (face_x, face_y), (0, 255, 255), 2)

            # Robot movement control
            if abs(error_x) > self.center_threshold:
                # Calculate turn speed based on error
                turn_speed = self.calculate_turn_speed(error_x, w)
                
                if error_x > 0:  # Face is to the left
                    self.turn_left(turn_speed)
                    print(f"Turning left at speed {turn_speed}")
                else:  # Face is to the right
                    self.turn_right(turn_speed)
                    print(f"Turning right at speed {turn_speed}")
            else:
                # Face is centered horizontally, handle forward/backward movement
                if width < self.far_threshold:
                    self.move_forward(128)
                    print("Moving forward")
                elif width > self.close_threshold:
                    self.move_backward(128)
                    print("Moving backward")
                else:
                    self.stop_motors()
                    print("Face is at optimal distance")

            # Add debug information
            cv2.putText(frame, f"Tilt: {self.tilt_current}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"Error X: {error_x}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            self.stop_motors()
            print("No face detected")

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

                processed_frame = self.track_face(frame)
                cv2.imshow('Face Tracking Debug', processed_frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                time.sleep(0.01)

        finally:
            print("Cleaning up...")
            cap.release()
            cv2.destroyAllWindows()
            self.cleanup()

    def cleanup(self):
        # Center tilt servo
        self.pi.set_servo_pulsewidth(self.TILT_PIN, 1500)
        time.sleep(0.5)

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