import cv2
import mediapipe as mp
import time
import pigpio
from statistics import mean

class PanTiltFaceTracker:
    def __init__(self, min_detection_confidence=0.5):
        # Initialize MediaPipe Face Detection
        self.mp_face_detection = mp.solutions.face_detection
        self.mp_draw = mp.solutions.drawing_utils
        self.face_detection = self.mp_face_detection.FaceDetection(
            min_detection_confidence=min_detection_confidence
        )
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Initialize servo control
        self.pi = pigpio.pi()
        self.PAN_PIN = 12
        self.TILT_PIN = 13
        
        # Servo parameters
        self.pan_position = 1500  # Center position (microseconds)
        self.tilt_position = 1500
        self.MIN_PULSE = 500     # Minimum pulse width
        self.MAX_PULSE = 2500    # Maximum pulse width
        self.STEP = 10          # Movement step size
        
        # Initialize servo positions
        self.pi.set_servo_pulsewidth(self.PAN_PIN, self.pan_position)
        self.pi.set_servo_pulsewidth(self.TILT_PIN, self.tilt_position)
        
        # PID control parameters
        self.prev_pan_error = 0
        self.prev_tilt_error = 0
        self.pan_integral = 0
        self.tilt_integral = 0
        
        # Camera frame center
        self.CENTER_X = 320  # Half of 640
        self.CENTER_Y = 240  # Half of 480
        
    def update_servo_positions(self, face_center_x, face_center_y):
        # PID constants
        KP = 0.1  # Proportional gain
        KI = 0.01 # Integral gain
        KD = 0.05 # Derivative gain
        
        # Calculate errors
        pan_error = self.CENTER_X - face_center_x
        tilt_error = self.CENTER_Y - face_center_y
        
        # Update integrals
        self.pan_integral += pan_error
        self.tilt_integral += tilt_error
        
        # Calculate derivatives
        pan_derivative = pan_error - self.prev_pan_error
        tilt_derivative = tilt_error - self.prev_tilt_error
        
        # Calculate PID outputs
        pan_adjustment = (KP * pan_error + 
                        KI * self.pan_integral + 
                        KD * pan_derivative)
        tilt_adjustment = (KP * tilt_error + 
                         KI * self.tilt_integral + 
                         KD * tilt_derivative)
        
        # Update positions
        self.pan_position += int(pan_adjustment)
        self.tilt_position -= int(tilt_adjustment)  # Inverted for natural movement
        
        # Constrain positions
        self.pan_position = max(self.MIN_PULSE, min(self.MAX_PULSE, self.pan_position))
        self.tilt_position = max(self.MIN_PULSE, min(self.MAX_PULSE, self.tilt_position))
        
        # Update servos
        self.pi.set_servo_pulsewidth(self.PAN_PIN, self.pan_position)
        self.pi.set_servo_pulsewidth(self.TILT_PIN, self.tilt_position)
        
        # Store errors for derivative calculation
        self.prev_pan_error = pan_error
        self.prev_tilt_error = tilt_error
        
    def process_frame(self, frame):
        # Convert BGR to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame and detect faces
        results = self.face_detection.process(rgb_frame)
        
        face_centers = []
        
        # Draw face detections
        if results.detections:
            for detection in results.detections:
                # Get bounding box coordinates
                bboxC = detection.location_data.relative_bounding_box
                ih, iw, _ = frame.shape
                bbox = int(bboxC.xmin * iw), int(bboxC.ymin * ih), \
                       int(bboxC.width * iw), int(bboxC.height * ih)
                
                # Calculate face center
                face_center_x = bbox[0] + bbox[2] // 2
                face_center_y = bbox[1] + bbox[3] // 2
                face_centers.append((face_center_x, face_center_y))
                
                # Draw bounding box
                cv2.rectangle(frame, bbox, (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(frame, (face_center_x, face_center_y), 5, (0, 0, 255), -1)
                
                # Draw confidence score
                cv2.putText(frame, 
                           f'{int(detection.score[0] * 100)}%',
                           (bbox[0], bbox[1] - 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 
                           1, (0, 255, 0), 2)
                
                # Draw landmarks
                self.mp_draw.draw_detection(frame, detection)
        
        # If faces detected, update servo positions based on the average face position
        if face_centers:
            avg_x = mean(x for x, _ in face_centers)
            avg_y = mean(y for _, y in face_centers)
            self.update_servo_positions(avg_x, avg_y)
            
            # Draw target lines
            cv2.line(frame, (int(avg_x), 0), (int(avg_x), 480), (255, 0, 0), 1)
            cv2.line(frame, (0, int(avg_y)), (640, int(avg_y)), (255, 0, 0), 1)
        
        return frame, results.detections if results.detections else []
    
    def run(self):
        prev_frame_time = 0
        
        try:
            while True:
                # Capture frame
                success, frame = self.cap.read()
                if not success:
                    print("Failed to grab frame")
                    break
                
                # Process frame
                frame, detections = self.process_frame(frame)
                
                # Calculate and display FPS
                current_time = time.time()
                fps = 1 / (current_time - prev_frame_time)
                prev_frame_time = current_time
                
                # Display FPS and servo positions
                cv2.putText(frame, f'FPS: {int(fps)}', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                           1, (0, 255, 0), 2)
                cv2.putText(frame, f'Faces: {len(detections)}',
                           (10, 70), cv2.FONT_HERSHEY_SIMPLEX,
                           1, (0, 255, 0), 2)
                cv2.putText(frame, f'Pan: {self.pan_position}',
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX,
                           1, (0, 255, 0), 2)
                cv2.putText(frame, f'Tilt: {self.tilt_position}',
                           (10, 150), cv2.FONT_HERSHEY_SIMPLEX,
                           1, (0, 255, 0), 2)
                
                # Display frame
                cv2.imshow('Face Tracking', frame)
                
                # Break loop on 'q' press
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
        finally:
            # Clean up
            self.cap.release()
            cv2.destroyAllWindows()
            self.pi.stop()

if __name__ == "__main__":
    # Create and run tracker
    tracker = PanTiltFaceTracker(min_detection_confidence=0.7)
    tracker.run()