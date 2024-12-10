# final code, nice steering and detection 4:36 pm, 11/5

# with april tag and ir

# commented out all gui, but functionality is the same as main robot lane 5

import cv2
from MotorModule_FINAL import Motor
from LaneModule import getLaneCurve
import utils_new as utils
from testqrfinal_14_lenador import AprilTagDetector_2, TagHandler
import Mock.GPIO as GPIO
from time import sleep
from irModule import IRSensorHandler

# Initialize motor with appropriate pins
motor = Motor(4, 17, 27, 22)

# Initialize cameras
lane_cam = cv2.VideoCapture(0)  # Lane detection camera
tag_cam = cv2.VideoCapture(1)   # AprilTag detection camera

# Initialize the trackbars window with initial values
initialTrackBarVals = [0, 0, 0, 240]  # Use the same initial values as in LaneModule
#utils.initializeTrackbars(initialTrackBarVals)  # Commenting out UI initialization

# Initialize tag detection
at_detector = AprilTagDetector_2()
tag_handler = TagHandler(motor)

def getImg(display=False, size=[480, 240]):
    ret, img_lane = lane_cam.read()
    if not ret:
        print("Error: Could not read from lane camera")
        return None

    if img_lane is None or img_lane.size == 0:
        print("Error: Empty image from lane camera")
        return None

    #print(f"Original image shape: {img_lane.shape}")

    try:
        img_lane = cv2.resize(img_lane, (size[0], size[1]))
    #    print(f"Resized image shape: {img_lane.shape}")
        return img_lane
    except cv2.error as e:
    #    print(f"Error resizing image: {e}")
        return None

def handleLaneFollowing(img):
    # Calculate the curve value for lane detection
    curveVal = getLaneCurve(img, display=0)  # Changed to display=1 to match LaneModule

    # Motor movement based on lane curve
    sen = 0.65  # SENSITIVITY
    maxVal = 0.2 # MAX SPEED
    deadzone = 0.10

    if curveVal > maxVal: curveVal = maxVal
    if curveVal < -maxVal: curveVal = -maxVal

    if curveVal > 0:
        sen = 1
        if curveVal < deadzone: curveVal = 0
    else:
        sen = 1
        if curveVal > -deadzone: curveVal = 0

    # Use negative speed to move forward with the inverted motor
    motor.move(-maxVal, curveVal * sen, 0.05)  # Changed to negative speed for forward motion

    # Print steering information
    steering_intensity = abs(curveVal)
    steering_bar = "=" * int(steering_intensity * 20)
    direction = "LEFT " if curveVal < 0 else "RIGHT" if curveVal > 0 else "STRAIGHT"
    print(f"Steering {direction} [{steering_bar:<20}] {curveVal:.2f}")

def main():
    # Initialize IR sensors
    ir_handler = IRSensorHandler(left_pin=5, center_pin=6, right_pin=13)

    # Initial movement - go forward for 2 seconds
    #print("Starting initial forward movement...")
    #motor.move(-0.3, 0, 2)  # Move forward for 2 seconds
    #motor.stop(0.1)  # Brief stop before starting main loop
    #print("Initial movement complete. Starting main loop...")

    try:
        while True:
            # Check IR sensors first
            #if ir_handler.all_white():
            #    print("All white detected - STOPPING")
            #    motor.stop()
            #    sleep(0.1)
            #    continue

            # Get lane detection image
            img_lane = getImg()

            # Get tag detection image
            ret_tag, img_tag = tag_cam.read()
            if not ret_tag:
                continue

            # Default to lane following
            should_follow_lanes = True

            # Detect and process AprilTags
            tags = at_detector.detect_tags(img_tag)

            # Process detected tags
            for tag in tags:
                tag_id = tag.tag_id

                if tag_id == 0:  # Railway crossing
                    if tag_handler.handle_railway_crossing(img_tag):
                        should_follow_lanes = False

                elif tag_id == 1:  # Pedestrian zone
                    if tag_handler.handle_pedestrian_zone(img_tag):
                        should_follow_lanes = True  # Continue following lanes but at reduced speed

                elif tag_id == 2:  # Traffic light
                    if tag_handler.handle_traffic_light(img_tag, tag.corners):
                        should_follow_lanes = False

                elif tag_id == 3:  # Obstacle avoidance
                    if tag_handler.handle_obstacle_avoidance(img_tag):
                        should_follow_lanes = False

            if should_follow_lanes and img_lane is not None:
                handleLaneFollowing(img_lane)

            # Display the images
            #cv2.imshow('Lane Camera', img_lane)
            #cv2.imshow('Tag Detection', img_tag)

            # Uncomment the exit functionality
            #if cv2.waitKey(1) & 0xFF == ord(' '):
            #    print("Exiting program...")
            #    break
    #except cv2.error as e:
    #    print(f"Error displaying images: {e}")
        # Handle the error, e.g., continue without displaying images

    finally:
        motor.stop()
        GPIO.cleanup()
        lane_cam.release()
        tag_cam.release()
        # cv2.destroyAllWindows()  # Commenting out window destruction

if __name__ == '__main__':
    main()