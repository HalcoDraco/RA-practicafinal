import cv2
import numpy as np
import matplotlib.pyplot as plt

def resize_image_by_height(img, target_height=300):
    h, w = img.shape[:2]
    scale_factor = target_height / h
    new_width = int(w * scale_factor)
    resized_img = cv2.resize(img, (new_width, target_height))
    return resized_img, scale_factor

def get_hsv_mask(img, color):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    if color == 'green':
        lower = np.array([30, 40, 40])
        upper = np.array([90, 255, 255])
    elif color == 'blue':
        lower = np.array([85, 40, 40])
        upper = np.array([140, 255, 255])
    elif color == 'yellow':
        lower = np.array([15, 40, 40])
        upper = np.array([30, 255, 255])
    else:
        raise ValueError("Color must be 'green', 'blue', or 'yellow'.")
    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def detect_circles(mask, dp=1.2, min_dist=40, param1=70, param2=20, min_radius=10, max_radius=120):
    blur = cv2.GaussianBlur(mask, (9, 9), 2)
    circles = cv2.HoughCircles(
        blur,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=min_dist,
        param1=param1,
        param2=param2,
        minRadius=min_radius,
        maxRadius=max_radius
    )
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        return circles  # List of (x, y, r)
    else:
        return []

def detect_colored_balls(image, min_radius=20, verbose=False):
    colors = ['green', 'blue', 'yellow']
    detected_colors = []
    resized_img, scale_factor = resize_image_by_height(image, target_height=450)
    masks = [get_hsv_mask(resized_img, color) for color in colors]
    detected_circles = []

    for i, (mask, color) in enumerate(zip(masks, colors)):
        circles = detect_circles(
            mask,
            dp=1.2,
            min_dist=40,
            param1=50,
            param2=30,
            min_radius=min_radius,
            max_radius=120
        )
        if len(circles) > 0:
            # Check if any circle meets the min_radius criterion
            for (x, y, r) in circles:
                if r >= min_radius:
                    print(color)
                    detected_colors.append(color)
                    detected_circles.append((color, x, y, r))
                    break  # Print only once per color

    if verbose:
        fig, axs = plt.subplots(1, 3, figsize=(18, 6))
        for i, (mask, color) in enumerate(zip(masks, colors)):
            mask_color = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            for (col, x, y, r) in detected_circles:
                if col == color:
                    cv2.circle(mask_color, (x, y), r, (0, 0, 255), 2)
                    cv2.circle(mask_color, (x, y), 2, (0, 255, 0), 3)
            axs[i].imshow(mask_color)
            axs[i].set_title(f"{color.capitalize()} - circles")
            axs[i].axis('off')
        plt.show()

    return detected_colors  # In case you want to use the result programmatically

# Example of usage:
if __name__ == '__main__':
    img = cv2.imread('anais/visio/yellow2.png')
    if img is None:
        print('Error: could not read input image.')
    else:
        detect_colored_balls(img, min_radius=10, verbose=True)
