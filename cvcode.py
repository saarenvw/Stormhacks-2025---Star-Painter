import cv2
import numpy as np
from scipy.fft import fft, fftfreq

# Global variables for color range
lower_colors = np.empty((0, 3), dtype=np.uint8)
upper_colors = np.empty((0, 3), dtype=np.uint8)
color_selected = False


# Mouse callback function to select color
def select_color(event, x, y, _flags, _param):
    global lower_colors, upper_colors, color_selected, frame
    if event == cv2.EVENT_LBUTTONDOWN:
        # Convert the frame to HSV color space

        float_frame = frame.astype(np.float32) / 255.0
        exp_frame = cv2.exp(float_frame)
        rgb_frame = cv2.normalize(exp_frame, None, 0, 255, cv2.NORM_MINMAX)
        rgb_frame = np.uint8(rgb_frame)
        hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)
        # Get the HSV color of the clicked pixel
        hsv_color = hsv_frame[y, x]
        h, s, v = map(int, hsv_color)

        # Define a tolerance range for the selected HSV color
        hue_tolerance = 30
        saturation_tolerance = 70
        value_tolerance = 70

        # Calculate lower and upper bounds for the HSV range
        lower_h = max(0, h - hue_tolerance)
        upper_h = min(179, h + hue_tolerance)

        lower_s = max(0, s - saturation_tolerance)
        upper_s = min(255, s + saturation_tolerance)

        # For vivid light, ensure the value (brightness) is high
        lower_v = max(150, v - value_tolerance)
        upper_v = min(255, v + value_tolerance)

        lower_colors = np.append(
            lower_colors,
            np.array([[lower_h, lower_s, lower_v]], dtype=np.uint8),
            axis=0,
        )
        upper_colors = np.append(
            upper_colors,
            np.array([[upper_h, upper_s, upper_v]], dtype=np.uint8),
            axis=0,
        )

        color_selected = True
        print(f"Color selected. New HSV range: {lower_colors} to {upper_colors}")


# Initialize Video Capture
cap = cv2.VideoCapture(1)  # Changed to 0 for default camera; adjust if needed

# Attempt to set a higher resolution for better quality
# Note: The camera must support this resolution.
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)


if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

# Set camera exposure
# Note: These settings are hardware-dependent and may not work on all cameras.
# First, try to disable auto exposure. For some backends (like V4L2 on Linux),
# a value of 1 disables it. For others (like DirectShow on Windows), it's 0.
# We'll try setting it to 0, which often means "manual mode".
# cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
# Set a manual exposure value. The range is camera-dependent.
# Common values are often in the range of -1 to -13. Start with a value like -6 and adjust.
# exposure_val = 10
# cap.set(cv2.CAP_PROP_EXPOSURE, exposure_val)
# print(f"Attempted to set exposure to: {exposure_val}")
# print(f"Current exposure: {cap.get(cv2.CAP_PROP_EXPOSURE)}")


# Create a window and set the mouse callback
cv2.namedWindow("Shape Tracker")
cv2.setMouseCallback("Shape Tracker", select_color)

# Reference frame size for scaling (e.g., 640x480)
ref_width, ref_height = 1000,1000
ref_area = ref_width * ref_height
scale_factor = 1

frame = None  # Initialize frame to be accessible in select_color
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Adjust min_shape_area for small light sources
    min_shape_area = 1  # Use a small, fixed area threshold

    # 1. Preprocessing: Convert the frame to HSV color space for better color detection.
    float_frame = frame.astype(np.float32) / 255.0
    exp_frame = cv2.exp(float_frame)
    rgb_frame = cv2.normalize(exp_frame, None, 0, 255, cv2.NORM_MINMAX)
    rgb_frame = np.uint8(rgb_frame)
    hsv_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_frame, (0, 0, 0), (0, 0, 0))  # Initialize empty mask
    # Create a mask for the specified color range in the HSV space
    for lower_color, upper_color in zip(lower_colors, upper_colors):
        if "mask" not in locals():
            mask = cv2.inRange(hsv_frame, lower_color, upper_color)
        else:
            mask = cv2.bitwise_or(
                mask, cv2.inRange(hsv_frame, lower_color, upper_color)
            )

    # Optional: Apply morphological operations to clean up the mask
    # Reduced iterations to avoid removing small light sources
    # mask = cv2.erode(mask, None, iterations=1)
    # mask = cv2.dilate(mask, None, iterations=1)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)

    # 2. Find Contours
    contours, _ = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    merged_contours = []
    used = [False] * len(contours)

    for i, c1 in enumerate(contours):
        if used[i]:
            continue
        (x1, y1, w1, h1) = cv2.boundingRect(c1)
        merged = c1.copy()
        for j, c2 in enumerate(contours):
            if i != j and not used[j]:
                (x2, y2, w2, h2) = cv2.boundingRect(c2)
                # If centers are within 10 pixels, merge them
                if abs((x1 + w1/2) - (x2 + w2/2)) < 10 and abs((y1 + h1/2) - (y2 + h2/2)) < 10:
                    merged = np.vstack((merged, c2))
                    used[j] = True
        merged_contours.append(merged)
        used[i] = True

    contours = merged_contours
    # 3. Filter/Analyze Contours
    if color_selected:
        centers = []
        for contour in contours:
            area = cv2.contourArea(contour)

            if area > min_shape_area:
                epsilon = 0.04 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)

                # --- CUSTOM SHAPE LOGIC: Identify a Triangle ---
                if len(approx) == 3:
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(hsv_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        "Custom Triangle",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )

                # --- CUSTOM SHAPE LOGIC: Identify a Rectangle/Square ---
                elif len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    cv2.rectangle(rgb_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
                    cv2.putText(
                        rgb_frame,
                        "Rectangle/Square",
                        (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 0, 0),
                        2,
                    )
                M = cv2.moments(contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    centers.append((cx, cy,area))
                    cv2.circle(frame, (cx, cy), 4, (0, 255, 255), -1)
                    if len(centers) >= 2:
                        # Sort centers by contour area (descending)
                        centers_sorted = sorted(centers, key=lambda c: c[2], reverse=True)

                        # Take the two largest
                        (x1, y1, _), (x2, y2, _) = centers_sorted[:2]

                        dx, dy = x2 - x1, y2 - y1
                        distance_px = (dx**2 + dy**2) ** 0.5

                        cv2.line(frame, (x1, y1), (x2, y2), (255, 255, 255), 2)
                        cv2.putText(rgb_frame, f"{distance_px:.1f} px",
                                    ((x1 + x2)//2, (y1 + y2)//2 - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        print(f"Distance between two largest objects: {distance_px:.2f} pixels")
                        cv2.circle(frame, (x1, y1), 6, (0, 255, 0), -1)
                        cv2.circle(frame, (x2, y2), 6, (0, 0, 255), -1)
                # if len(centers) >= 2:

                #     (x1,y1),(x2,y2) = centers[0], centers[1]
                #     dx, dy = x2-x1, y2 -y1
                #     distance_px = (dx**2 + dy**2) ** 0.5
                #     cv2.line(frame, (x1,y1), (x2,y2),(255,255,255), 2)
                #     cv2.putText(rgb_frame, f"{distance_px:.1f} px", ((x1+x2)//2, (y1+y2)//2 - 10),
                #                 cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255),2)
                #     print(f"Distance between objects: {distance_px:.2f} pixels")
    else:
        # Prompt user to select a color
        cv2.putText(
            frame,
            "Click on a color to track",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (255, 255, 255),
            2,
        )

        # Display the result
    cv2.imshow("Shape Tracker", rgb_frame)
    cv2.imshow("Mask", mask)  # Optional: show the mask for debugging

# Exit on 'q' press
    key = cv2.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    if cv2.waitKey(1) & 0xFF == ord("c"):
        lower_colors = np.empty((0, 3), dtype=np.uint8)
        upper_colors = np.empty((0, 3), dtype=np.uint8)
        color_selected = False
        print("Cleared selected colors.")


cap.release()
cv2.destroyAllWindows()
