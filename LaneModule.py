# uses threshold to detect lines
# pair with main robot copy 17

# USES THE UTILS_NEW<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 11/6, 7:46

import cv2
import numpy as np
import utils_new as utils

curveList = []
avgVal = 10

def project_parallel_line(points, lane_width, is_left_line):
    """Project a parallel line at a fixed distance from detected line"""
    if not points:
        return []

    parallel_points = []
    offset = lane_width if is_left_line else -lane_width

    for x, y in points:
        new_x = int(x + offset)
        parallel_points.append((new_x, y))

    return parallel_points

def find_lane_pixels(binary_img, strip_y, strip_height, width):
    """Find lane pixels in a horizontal strip of the thresholded image"""
    strip = binary_img[strip_y:strip_y + strip_height, :]

    left_half = strip[:, :width//2]
    right_half = strip[:, width//2:]

    left_pixels = np.nonzero(left_half)
    right_pixels = np.nonzero(right_half)

    leftx = None
    rightx = None

    if len(left_pixels[1]) > 20:
        leftx = int(np.mean(left_pixels[1]))

    if len(right_pixels[1]) > 20:
        rightx = int(np.mean(right_pixels[1])) + width//2

    return leftx, rightx

def validate_lane_points(points, min_points=3, max_gap=50):
    """Validate and clean lane points"""
    if len(points) < min_points:
        return []

    valid_points = []
    last_x = None

    for x, y in sorted(points, key=lambda p: p[1], reverse=True):
        if last_x is None:
            valid_points.append((x, y))
            last_x = x
        else:
            if abs(x - last_x) < max_gap:
                valid_points.append((x, y))
                last_x = x

    return valid_points if len(valid_points) >= min_points else []

def getLaneCurve(img, display=2):
    imgCopy = img.copy()
    imgResult = img.copy()

    imgThres = utils.thresholding(img)

    hT, wT, c = img.shape
    points = utils.valTrackbars()
    imgWarp = utils.warpImg(imgThres, points, wT, hT)
    imgWarpPoints = utils.drawPoints(imgCopy, points)

    _, binary_warped = cv2.threshold(imgWarp, 127, 255, cv2.THRESH_BINARY)

    num_strips = 15
    strip_height = hT // num_strips
    lane_width = wT // 1.5

    left_points = []
    right_points = []
    center_points = []

    for i in range(num_strips-1, -1, -1):
        strip_y = i * strip_height
        strip_center_y = strip_y + strip_height // 2

        leftx, rightx = find_lane_pixels(binary_warped, strip_y, strip_height, wT)

        if leftx is not None:
            left_points.append((leftx, strip_center_y))
        if rightx is not None:
            right_points.append((rightx, strip_center_y))

    left_points = validate_lane_points(left_points)
    right_points = validate_lane_points(right_points)

    if len(left_points) >= 3 and len(right_points) < 3:
        right_points = project_parallel_line(left_points, lane_width, True)
    elif len(right_points) >= 3 and len(left_points) < 3:
        left_points = project_parallel_line(right_points, lane_width, False)

    if left_points and right_points:
        min_points = min(len(left_points), len(right_points))
        for i in range(min_points):
            left_x, y = left_points[i]
            right_x, _ = right_points[i]
            center_x = (left_x + right_x) // 2
            center_points.append((center_x, y))

    curve = 0
    if center_points:
        bottom_centers = center_points[-3:] if len(center_points) >= 3 else center_points
        avg_center_x = sum(x for x, _ in bottom_centers) / len(bottom_centers)

        curve = avg_center_x - (wT // 2)

        dead_zone = 10
        if abs(curve) < dead_zone:
            curve = 0

        curveList.append(curve)
        if len(curveList) > avgVal:
            curveList.pop(0)
        curve = int(sum(curveList) / len(curveList))

        if display != 0:
            # Left line points (red)
            for x, y in left_points:
                cv2.circle(imgResult, (x, y), 5, (0, 0, 255), -1)

            # Right line points (blue)
            for x, y in right_points:
                cv2.circle(imgResult, (x, y), 5, (255, 0, 0), -1)

            # Center points (green)
            for x, y in center_points:
                cv2.circle(imgResult, (x, y), 5, (0, 255, 0), -1)

            # Draw lines connecting points
            if len(left_points) > 1:
                pts = np.array(left_points, np.int32).reshape((-1, 1, 2))
                cv2.polylines(imgResult, [pts], False, (0, 0, 255), 2)

            if len(right_points) > 1:
                pts = np.array(right_points, np.int32).reshape((-1, 1, 2))
                cv2.polylines(imgResult, [pts], False, (255, 0, 0), 2)

            if len(center_points) > 1:
                pts = np.array(center_points, np.int32).reshape((-1, 1, 2))
                cv2.polylines(imgResult, [pts], False, (0, 255, 255), 2)

            # Steering visualization
            arrow_length = 80
            centerX = wT // 2
            midY = hT - 50

            arrow_start = (centerX, midY - 30)
            arrow_angle = np.radians(curve * 0.5)
            arrow_end = (
                int(arrow_start[0] + arrow_length * np.sin(arrow_angle)),
                int(arrow_start[1] - arrow_length * np.cos(arrow_angle))
            )

            steering_intensity = min(255, abs(int(curve * 2.55)))
            arrow_color = (0, 255 - steering_intensity, steering_intensity) if curve < 0 else (steering_intensity, 255 - steering_intensity, 0)
            cv2.arrowedLine(imgResult, arrow_start, arrow_end, arrow_color, 3, tipLength=0.3)

            # Telemetry
            cv2.putText(imgResult, f"Curve: {curve}", (30, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

            direction = "STRAIGHT" if abs(curve) < dead_zone else "LEFT" if curve < 0 else "RIGHT"
            direction_color = (0, 255, 0) if abs(curve) < dead_zone else (0, 0, 255) if abs(curve) > 50 else (0, 255, 255)
            cv2.putText(imgResult, f"STEERING {direction}", (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, direction_color, 2)

            # Line status indicator
            status = []
            if len(left_points) > 0 and len(right_points) > 0:
                status.append("BOTH LINES")
                status_color = (0, 255, 0)
            elif len(left_points) > 0:
                status.append("LEFT + PROJECTED")
                status_color = (0, 255, 255)
            elif len(right_points) > 0:
                status.append("RIGHT + PROJECTED")
                status_color = (0, 255, 255)
            else:
                status.append("NO LINES")
                status_color = (0, 0, 255)

            cv2.putText(imgResult, " ".join(status), (30, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, status_color, 2)

            # Confidence meter
            total_possible_points = num_strips
            detected_points = len(center_points)
            confidence = (detected_points / total_possible_points) * 100

            cv2.rectangle(imgResult, (wT-120, 20), (wT-20, 40), (0, 0, 0), -1)
            cv2.rectangle(imgResult, (wT-120, 20), (wT-120 + int(confidence), 40),
                         (0, 255, 0), -1)
            cv2.putText(imgResult, f"{int(confidence)}%", (wT-110, 35),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        # Normalize curve value
    curve = curve / 100
    curve = max(-1, min(1, curve))

    if display == 2:
        imgStacked = utils.stackImages(0.7, ([img, imgWarpPoints, imgWarp]))
        cv2.imshow('ImageStack', imgStacked)
        cv2.imshow('Result', imgResult)
    elif display == 1:
        cv2.imshow('Result', imgResult)

    elif display == 0:
        return curve  # Return normalized curve value without any display


    return curve

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    initialTrackBarVals = [0, 0, 0, 240]
    utils.initializeTrackbars(initialTrackBarVals)
    frameCounter = 0

    while True:
        frameCounter += 1
        if cap.get(cv2.CAP_PROP_FRAME_COUNT) == frameCounter:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            frameCounter = 0

        success, img = cap.read()
        if not success:
            break

        img = cv2.resize(img, (480, 240))
        curve = getLaneCurve(img, display=1)

        steering_intensity = abs(curve)
        steering_bar = "=" * int(steering_intensity * 20)
        direction = "LEFT " if curve < 0 else "RIGHT" if curve > 0 else "STRAIGHT"
        print(f"Steering {direction} [{steering_bar:<20}] {curve:.2f}")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()