import cv2
import numpy as np

def classify_color(hsv_mean):
    """
    Classify a mean HSV value into one of the target colors: green, yellow, or blue.

    Args:
        hsv_mean (tuple): Mean HSV value (H, S, V) within the detected circle region.

    Returns:
        str or None: One of 'green', 'yellow', 'blue', or None if no match.
    """
    h, s, v = hsv_mean
    # Updated HSV ranges for more accurate classification
    # Green: hue ~35-85, high saturation
    if 35 <= h <= 85 and s >= 100 and v >= 50:
        return "green"
    # Yellow: hue ~15-40, very high saturation and brightness
    if 15 <= h <= 40 and s >= 100 and v >= 100:
        return "yellow"
    # Blue: hue ~90-130, high saturation
    if 90 <= h <= 130 and s >= 100 and v >= 50:
        return "blue"
    return None


def detect_colored_balls(image, verbose=False):
    """
    Detect colored balls (green, yellow, blue) in the input image using Hough Circle Transform.
    Optionally visualize detections and mean-color fills when verbose=True.

    Args:
        image (numpy.ndarray): Input BGR image.
        verbose (bool): If True, show debug images/windows and HSV values.

    Returns:
        list of tuples: [(color_str, radius_px, (center_x, center_y)), ...]
    """
    if verbose:
        # Show original image for debugging
        cv2.imshow('Original Image', image)
        cv2.waitKey(0)
    # Resize image keeping aspect ratio, max 400px on the longest side
    h, w = image.shape[:2]
    max_dim = max(h, w)
    scale = 400 / max_dim if max_dim > 400 else 1.0
    new_w, new_h = int(w * scale), int(h * scale)
    resized_image = cv2.resize(image, (new_w, new_h))
    if verbose:
        cv2.imshow('Resized Image', resized_image)
        cv2.waitKey(0)
    image = resized_image

    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if verbose:
        # Show grayscale image for debugging
        cv2.imshow('Gray Image', gray)
        cv2.waitKey(0)
    # Apply Gaussian blur to smooth edges
    blurred = cv2.GaussianBlur(gray, (3, 3), 2)
    if verbose:
        # Show blurred image for debugging
        cv2.imshow('Blurred Image', blurred)
        cv2.waitKey(0)

    # Perform Hough Circle detection with more restrictive parameters
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=50,
        param1=50,
        param2=80,
        minRadius=10,
        maxRadius=0
    )

    results = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype(int)
        # Prepare visualization copy
        if verbose:
            detected_img = image.copy()
            for (x, y, r) in circles:
                cv2.circle(detected_img, (x, y), r, (0, 0, 255), 2)
                cv2.circle(detected_img, (x, y), 2, (0, 255, 0), 3)
            cv2.imshow('Detected Circles', detected_img)
            cv2.waitKey(0)

        # Convert original to HSV for color measurement
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for (x, y, r) in circles:
            # Create mask and compute mean HSV
            mask = np.zeros(gray.shape, dtype='uint8')
            cv2.circle(mask, (x, y), r, 255, -1)
            mean_hsv = cv2.mean(hsv, mask=mask)[:3]

            if verbose:
                print(f"Circle at ({x},{y}), r={r}: mean HSV = {mean_hsv}")
                # Show filled region with mean color
                hsv_pixel = np.uint8([[mean_hsv]])
                mean_bgr = cv2.cvtColor(hsv_pixel, cv2.COLOR_HSV2BGR)[0][0].tolist()
                filled = image.copy()
                cv2.circle(filled, (x, y), r, mean_bgr, -1)
                cv2.imshow(f'Filled ({x},{y})', filled)
                cv2.waitKey(0)

            # Classify and record
            color = classify_color(mean_hsv)
            if color:
                print(color)
                results.append((color, r, (x, y)))

    # Close all debug windows if opened
    if verbose:
        cv2.destroyAllWindows()

    return results


if __name__ == '__main__':
    img = cv2.imread('anais/visio/exemple1.jpg')
    if img is None:
        print('Error: could not read input image.')
    else:
        detected = detect_colored_balls(img, verbose=False)
        for col, radius, center in detected:
            print(f"Detected {col} ball at {center} with radius {radius} px")
