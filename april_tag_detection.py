# tag 0, 2, 3 movements
# uses ultrasonic sensor

# show  sequence movement on screen

# updated with tag 1 actions. When tag 1 is detected, it slows down for 10 seconds
import cv2
from pupil_apriltags import Detector
import numpy as np
import logging
import time
from MotorModule_FINAL import Motor
import Mock.GPIO as GPIO

class UltrasonicSensor:
    def __init__(self, trig_pin=14, echo_pin=15):
        self.trig_pin = trig_pin
        self.echo_pin = echo_pin
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.trig_pin, GPIO.OUT)
        GPIO.setup(self.echo_pin, GPIO.IN)
        
    def get_distance(self):
        # Send trigger pulse
        GPIO.output(self.trig_pin, False)
        time.sleep(0.00001)  # 10 microseconds
        GPIO.output(self.trig_pin, True)
        time.sleep(0.00001)
        GPIO.output(self.trig_pin, False)
        
        # Get the time readings
        while GPIO.input(self.echo_pin) == 0:
            pulse_start = time.time()
        
        while GPIO.input(self.echo_pin) == 1:
            pulse_end = time.time()
            
        try:
            pulse_duration = pulse_end - pulse_start
            # Distance = Speed of sound (34300 cm/s) * time / 2 (round trip)
            distance = (pulse_duration * 34300) / 2
            return round(distance, 2)
        except UnboundLocalError:
            return None
        
        
     
     
     
     
     
     
     
        
# tag handler and functions for each tag
################################################################        
        
class AprilTagDetector_2: # to be called in the main class
    def __init__(self):
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.tag_actions = {
            0: "Railway level crossing: Monitor for obstacles",
            1: "Watch out for pedestrians: Wait for pedestrians to pass",
            2: "Traffic light intersection: Follow traffic light signals",
            3: "Double lane obstacle: Perform avoidance sequence",
        }
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def detect_tags(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        
        # Draw tag information on frame
        for tag in tags:
            # Draw tag outline
            corners = tag.corners
            corners = np.int32(corners)
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)
            
            # Draw tag center
            center = np.int32(tag.center)
            cv2.circle(frame, tuple(center), 5, (0, 0, 255), -1)
            
            # Draw tag ID
            cv2.putText(frame, f"ID: {tag.tag_id}", 
                       (center[0] - 10, center[1] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Draw action text
            if tag.tag_id in self.tag_actions:
                action = self.tag_actions[tag.tag_id]
                cv2.putText(frame, f"Action: {action}",
                           (10, 30 * (tag.tag_id + 1)),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        return tags

class TagHandler:
    def __init__(self, motor):
        self.motor = motor
        self.ultrasonic = UltrasonicSensor()
        self.color_detector = ColorDetector()
        self.movement_sequence = MovementSequence(motor)
        self.waiting_obstacle = False
        self.wait_start_time = 0
        self.in_pedestrian_zone = False
        self.pedestrian_zone_start_time = 0
        self.normal_speed = 0.20
        
    def draw_overlay(self, frame, text, y_position, color=(255, 255, 0)):
        """Helper method to draw semi-transparent overlay with text"""
        # Create overlay for better text visibility
        overlay = frame.copy()
        text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
        cv2.rectangle(overlay, 
                     (5, y_position - 20), 
                     (15 + text_size[0], y_position + 10),
                     (0, 0, 0), 
                     -1)
        cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
        cv2.putText(frame, text, (10, y_position),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    
    def handle_railway_crossing(self, frame):
        """Handle Tag 0 - Railway crossing with obstacle detection"""
        if not self.waiting_obstacle:
            distance = self.ultrasonic.get_distance()
            if distance is not None:
                self.draw_overlay(frame, f"Distance: {distance}cm", 380)
                if distance <= 20:
                    self.motor.stop()
                    self.waiting_obstacle = True
                    self.wait_start_time = time.time()
                    return True
        
        if self.waiting_obstacle:
            remaining_time = 15 - (time.time() - self.wait_start_time)
            if remaining_time > 0:
                self.draw_overlay(frame, f"Waiting: {remaining_time:.1f}s", 460, (0, 0, 255))
            else:
                self.waiting_obstacle = False
                
        return False
    
    def handle_pedestrian_zone(self, frame):
        """Handle Tag 1 - Pedestrian crossing with speed reduction"""
        current_time = time.time()
        
        if not self.in_pedestrian_zone:
            # Start of pedestrian zone - reduce speed
            self.motor.move(self.normal_speed / 2, 0, 0.05)
            self.in_pedestrian_zone = True
            self.pedestrian_zone_start_time = current_time
        
        remaining_time = 10 - (current_time - self.pedestrian_zone_start_time)
        
        if remaining_time > 0:
            # Still in reduced speed period
            self.draw_overlay(frame, 
                            f"Reduced Speed Zone: {remaining_time:.1f}s",
                            410, 
                            (255, 165, 0))
            return True
        else:
            # Time's up - return to normal speed
            self.motor.move(self.normal_speed, 0, 0.05)
            self.in_pedestrian_zone = False
            return False
    
    def handle_traffic_light(self, frame, corners):
        """Handle Tag 2 - Traffic light detection"""
        color, roi_x1, roi_y1, roi_x2, roi_y2 = self.color_detector.detect_color(frame, corners)
        
        if roi_x1 is not None:
            cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 0), 2)
        
        if color:
            if color == 'red':
                self.motor.stop()
                self.draw_overlay(frame, 
                                "Red Light - Stopping",
                                450, 
                                (0, 0, 255))
                return True
            elif color == 'green':
                self.draw_overlay(frame,
                                "Green Light - Moving",
                                450,
                                (0, 255, 0))
                return False
        return False
    
    def handle_obstacle_avoidance(self, frame):
        """Handle Tag 3 - Obstacle avoidance sequence"""
        if not self.movement_sequence.sequence_running:
            distance = self.ultrasonic.get_distance()
            if distance is not None:
                self.draw_overlay(frame, f"Distance: {distance}cm", 380)
                if distance <= 10:
                    self.movement_sequence.sequence_running = True
                    return True
        
        if self.movement_sequence.sequence_running:
            status, continue_sequence = self.movement_sequence.perform_avoidance_sequence()
            
            # Draw sequence steps
            steps = [
                "1. Turn right 90 degrees",
                "2. Move forward 1 second",
                "3. Turn left 90 degrees",
                "4. Move forward 2 seconds",
                "5. Turn left 90 degrees",
                "6. Move forward 1 second",
                "7. Turn right 90 degrees",
                "8. Move forward 2 seconds"
            ]
            
            # Draw semi-transparent background for sequence
            overlay = frame.copy()
            cv2.rectangle(overlay, 
                         (frame.shape[1]-400, 10),
                         (frame.shape[1]-10, 250),
                         (0, 0, 0),
                         -1)
            cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
            
            # Draw sequence steps
            for i, step in enumerate(steps):
                y_pos = 30 + (i * 25)
                color = (0, 255, 0) if i == self.movement_sequence.current_step else (255, 255, 255)
                thickness = 2 if i == self.movement_sequence.current_step else 1
                cv2.putText(frame, step,
                           (frame.shape[1]-390, y_pos),
                           cv2.FONT_HERSHEY_SIMPLEX,
                           0.6,
                           color,
                           thickness)
            
            self.draw_overlay(frame,
                            f"Current Action: {status}",
                            frame.shape[0]-20,
                            (0, 255, 255))
            
            return continue_sequence

#####################################################################











class ColorDetector:
    def __init__(self):
        # HSV color ranges for traffic lights
        self.red_lower1 = np.array([0, 50, 50])
        self.red_upper1 = np.array([10, 255, 255])
        self.red_lower2 = np.array([170, 50, 50])  # Red wraps around HSV
        self.red_upper2 = np.array([180, 255, 255])
        
        self.green_lower = np.array([40, 30, 30])
        self.green_upper = np.array([85, 255, 255])
        
    def detect_color(self, frame, tag_corners):
        # Calculate tag dimensions
        x_min = np.min(tag_corners[:, 0])
        x_max = np.max(tag_corners[:, 0])
        y_min = np.min(tag_corners[:, 1])
        y_max = np.max(tag_corners[:, 1])
        
        tag_width = x_max - x_min
        tag_height = y_max - y_min
        
        # Define region of interest in upper right of tag
        # Area is 4 times the tag area
        roi_width = int(4 * tag_width)  # 2 times wider
        roi_height = int(4 * tag_height)  # 2 times taller
        
        # Calculate ROI coordinates (upper right of tag)
        roi_x1 = int(x_max)  # Start from right edge of tag
        roi_x2 = min(roi_x1 + roi_width, frame.shape[1])  # Ensure within frame
        roi_y1 = max(0, int(y_min - roi_height/2))  # Center vertically on tag's top edge
        roi_y2 = min(roi_y1 + roi_height, frame.shape[0])  # Ensure within frame
        
        # Extract ROI
        roi = frame[roi_y1:roi_y2, roi_x1:roi_x2]
        
        if roi.size == 0:
            return None, None, None, None, None
        
        # Convert ROI to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Detect red and green in the ROI
        red_mask1 = cv2.inRange(hsv, self.red_lower1, self.red_upper1)
        red_mask2 = cv2.inRange(hsv, self.red_lower2, self.red_upper2)
        red_mask = red_mask1 + red_mask2
        green_mask = cv2.inRange(hsv, self.green_lower, self.green_upper)
        
        # Calculate percentage of red and green pixels
        red_percent = np.sum(red_mask > 0) / red_mask.size * 100
        green_percent = np.sum(green_mask > 0) / green_mask.size * 100
        
        # Threshold for detection (adjust as needed)
        threshold = 5
        color = None
        if red_percent > threshold and red_percent > green_percent:
            color = 'red'
        elif green_percent > threshold and green_percent > red_percent:
            color = 'green'
            
        return color, roi_x1, roi_y1, roi_x2, roi_y2
    
    

class AprilTagDetector:
    def __init__(self):
        self.at_detector = Detector(
            families='tag36h11',
            nthreads=4,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )
        self.tag_actions = {
            0: "Railway level crossing: Monitor for obstacles",
            1: "Watch out for pedestrians: Wait for pedestrians to pass",
            2: "Traffic light intersection: Follow traffic light signals",
            3: "Double lane obstacle: Perform avoidance sequence",
        }
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
        self.monitoring_tag_0 = False
        self.monitoring_tag_3 = False
        self.waiting_timer = None

    def detect_and_process_tags(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.at_detector.detect(gray)
        
        detected_tags = []
        for tag in tags:
            tag_id = tag.tag_id
            if tag_id in self.tag_actions:
                action = self.tag_actions[tag_id]
                detected_tags.append((tag_id, action, tag.center, tag.corners))
            else:
                self.logger.warning(f"Detected unknown tag ID: {tag_id}")
        
        return detected_tags
    
    
# calling the turn left and right 90 degrees functions frmo the motor module
class MovementSequence:
    def __init__(self, motor):
        self.motor = motor
        self.current_step = 0
        self.sequence_running = False
        
    def perform_avoidance_sequence(self):
        sequence_steps = [
            (self.turn_right_90, "Turning right 90°"),
            (lambda: self.forward_move(1), "Moving forward 1s"),
            (self.turn_left_90, "Turning left 90°"),
            (lambda: self.forward_move(2), "Moving forward 2s"),
            (self.turn_left_90, "Turning left 90°"),
            (lambda: self.forward_move(1), "Moving forward 1s"),
            (self.turn_right_90, "Turning right 90°"),
            (lambda: self.forward_move(2), "Moving forward 2s")
        ]
        
        if self.current_step < len(sequence_steps):
            action, description = sequence_steps[self.current_step]
            action()
            self.current_step += 1
            return description, self.current_step < len(sequence_steps)
        else:
            self.sequence_running = False
            self.current_step = 0
            return "Sequence complete", False
            
    def turn_right_90(self):
        """Turns the robot 90 degrees to the right using the motor class implementation."""
        self.motor.turn_right_90()

    def turn_left_90(self):
        """Turns the robot 90 degrees to the left using the motor class implementation."""
        self.motor.turn_left_90()
        
    def forward_move(self, duration):
        self.motor.move(0.3, 0, 0.05)
        time.sleep(duration)
        time.sleep(1)
    












def main():
    cap = cv2.VideoCapture(0)
    detector = AprilTagDetector()
    color_detector = ColorDetector()
    motor = Motor(4, 17, 27, 22)  # PWM_A, DIR_A, PWM_B, DIR_B
    ultrasonic = UltrasonicSensor(trig_pin=14, echo_pin=15)
    movement_sequence = MovementSequence(motor)
    
    # Initial state
    motor.move(0.3, 0, 0.05)  # speed, turn, delay
    waiting_obstacle = False
    wait_start_time = 0
    at_traffic_light = False
    last_detected_color = None
    sequence_status = ""
    in_pedestrian_zone = False
    pedestrian_zone_start_time = 0
    normal_speed = 0.3

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                detector.logger.error("Failed to capture frame")
                break

            detected_tags = detector.detect_and_process_tags(frame)
            current_time = time.time()
            
            # Process detected tags
            for tag_id, action, center, corners in detected_tags:
                # Draw tag outline
                cv2.polylines(frame, [np.int32(corners)], True, (0, 255, 0), 2)
                cv2.circle(frame, tuple(np.int32(center)), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"ID: {tag_id}", (int(center[0]) - 10, int(center[1]) - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, f"Action: {action}", (10, 30 * (tag_id + 1)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

                # Handle tag 0 - Railway crossing with ultrasonic sensor
                if tag_id == 0:
                    detector.monitoring_tag_0 = True
                    if not waiting_obstacle:
                        distance = ultrasonic.get_distance()
                        if distance is not None:
                            cv2.putText(frame, f"Distance: {distance}cm", (10, 380),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                            if distance <= 20:  # Object detected within 20cm
                                detector.logger.info("Obstacle detected: Stopping for 15 seconds")
                                motor.stop()
                                waiting_obstacle = True
                                wait_start_time = current_time
                
                # Handle tag 1 - Pedestrian crossing
                elif tag_id == 1:
                    if not in_pedestrian_zone:
                        detector.logger.info("Pedestrian zone detected: Reducing speed")
                        motor.move(normal_speed / 2, 0, 0.05)  # Reduce to half speed
                        in_pedestrian_zone = True
                        pedestrian_zone_start_time = current_time
                    
                    # Display slow speed status
                    remaining_time = 10 - (current_time - pedestrian_zone_start_time)
                    if remaining_time > 0:
                        cv2.putText(frame, f"Reduced Speed Zone: {remaining_time:.1f}s", (10, 410),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 165, 0), 2)
                
                # Handle tag 2 - Traffic light
                elif tag_id == 2:
                    at_traffic_light = True
                    color, roi_x1, roi_y1, roi_x2, roi_y2 = color_detector.detect_color(frame, corners)
                    
                    if roi_x1 is not None:
                        cv2.rectangle(frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 0), 2)
                    
                    if color:
                        last_detected_color = color
                        if color == 'red':
                            motor.stop()
                            cv2.putText(frame, "Red Light - Stopping", (10, 450),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                        elif color == 'green':
                            motor.move(0.3, 0, 0.05)
                            cv2.putText(frame, "Green Light - Moving", (10, 450),
                                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Handle tag 3 - Obstacle avoidance sequence
                elif tag_id == 3:
                    detector.monitoring_tag_3 = True
                    if not movement_sequence.sequence_running:
                        distance = ultrasonic.get_distance()
                        if distance is not None:
                            cv2.putText(frame, f"Distance: {distance}cm", (10, 380),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                            if distance <= 10:  # Object detected within 10cm
                                detector.logger.info("Starting obstacle avoidance sequence")
                                movement_sequence.sequence_running = True
                    
                    # Display sequence steps on screen
                    steps = [
                        "1. Turn right 90 degrees",
                        "2. Move forward 1 second",
                        "3. Turn left 90 degrees",
                        "4. Move forward 2 seconds",
                        "5. Turn left 90 degrees",
                        "6. Move forward 1 second",
                        "7. Turn right 90 degrees",
                        "8. Move forward 2 seconds",
                        "9. Resume forward movement"
                    ]
                    
                    # Draw semi-transparent background for better text visibility
                    overlay = frame.copy()
                    cv2.rectangle(overlay, (frame.shape[1]-400, 10), (frame.shape[1]-10, 250), (0, 0, 0), -1)
                    cv2.addWeighted(overlay, 0.5, frame, 0.5, 0, frame)
                    
                    # Draw sequence steps
                    for i, step in enumerate(steps):
                        y_pos = 30 + (i * 25)  # Spacing between lines
                        # Highlight current step if sequence is running
                        if movement_sequence.sequence_running and i == movement_sequence.current_step:
                            color = (0, 255, 0)  # Green for current step
                            thickness = 2
                        else:
                            color = (255, 255, 255)  # White for other steps
                            thickness = 1
                        cv2.putText(frame, step, (frame.shape[1]-390, y_pos),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, thickness)

            # Handle obstacle avoidance sequence
            if movement_sequence.sequence_running:
                status, continue_sequence = movement_sequence.perform_avoidance_sequence()
                sequence_status = status
                # Display current action in bottom left
                cv2.putText(frame, f"Current Action: {status}", (10, frame.shape[0]-20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                if not continue_sequence:
                    motor.move(0.3, 0, 0.05)  # Resume normal forward movement

            # Handle waiting state for obstacle detection
            if waiting_obstacle:
                remaining_time = 15 - (current_time - wait_start_time)
                if remaining_time > 0:
                    cv2.putText(frame, f"Waiting: {remaining_time:.1f}s", (10, 460),
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    detector.logger.info("Wait complete: Resuming movement")
                    motor.move(0.3, 0, 0.05)
                    waiting_obstacle = False

            # Display sequence status if active
            if sequence_status:
                cv2.putText(frame, f"Status: {sequence_status}", (10, 420),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)

            # Reset monitoring states if tags are no longer detected
            if detector.monitoring_tag_0 and not any(tag[0] == 0 for tag in detected_tags):
                detector.monitoring_tag_0 = False
            if detector.monitoring_tag_3 and not any(tag[0] == 3 for tag in detected_tags):
                detector.monitoring_tag_3 = False

            # If no tag 2 is detected anymore and we were at a traffic light
            if at_traffic_light and not any(tag[0] == 2 for tag in detected_tags):
                at_traffic_light = False
                last_detected_color = None
                if not movement_sequence.sequence_running:
                    motor.move(0.3, 0, 0.05)  # Resume normal movement

            cv2.imshow("AprilTag Detection", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        motor.stop()
        GPIO.cleanup()  # Clean up GPIO
        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()