#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool

def resize_image_by_height(img, target_height=300):
    """
    Resizes `img` so that its height equals `target_height`, keeping aspect ratio.
    Returns the resized image and the scale factor used.
    """
    h, w = img.shape[:2]
    scale_factor = target_height / float(h)
    new_width = int(w * scale_factor)
    resized_img = cv2.resize(img, (new_width, target_height))
    return resized_img, scale_factor

def get_hsv_mask(img, color):
    """
    Generates a binary mask for `img` in HSV space, filtering by the given `color`.
    Valid colors: 'green', 'blue', 'yellow'.
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    if color == 'green':
        lower = np.array([30,  40,  40])
        upper = np.array([90, 255, 255])
    elif color == 'blue':
        lower = np.array([85,  40,  40])
        upper = np.array([140, 255, 255])
    elif color == 'yellow':
        lower = np.array([15,  40,  40])
        upper = np.array([30, 255, 255])
    else:
        raise ValueError("Color must be 'green', 'blue', or 'yellow'.")

    mask = cv2.inRange(hsv, lower, upper)
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask

def detect_circles(mask, dp=1.2, min_dist=40, param1=70, param2=20, min_radius=10, max_radius=120):
    """
    Applies Hough Circle Transform on the blurred `mask` image.
    Returns a list of detected circles as (x, y, r).
    """
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
        return np.round(circles[0, :]).astype("int")
    return []

def detect_colored_balls(image, min_radius=20):
    """
    Detects green, blue, or yellow circular regions in `image`.
    Returns a list of color strings that were detected ('green', 'blue', 'yellow').
    """
    colors = ['green', 'blue', 'yellow']
    detected_colors = []

    # Resize image for faster processing
    resized_img, scale_factor = resize_image_by_height(image, target_height=300)

    # Create a mask for each color
    masks = [get_hsv_mask(resized_img, color) for color in colors]

    # For each color mask, detect circles
    for mask, color in zip(masks, colors):
        circles = detect_circles(
            mask,
            dp=1.2,
            min_dist=40,
            param1=70,
            param2=50,
            min_radius=min_radius,
            max_radius=120
        )
        # If any circle of sufficient radius is found, record the color once
        for (x, y, r) in circles:
            if r >= min_radius:
                detected_colors.append(color)
                break

    return detected_colors


class BallColorDetectorNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('ball_color_detector', anonymous=True)

        # CV Bridge for converting ROS Image ↔ OpenCV
        self.bridge = CvBridge()

        # Interval between image processing
        self.process_interval = rospy.get_param('~process_interval', 0.3)
        self.last_process_time = rospy.Time(0)

        # Estate variable
        self.vision_active = False

        # Subscriber: listen to the robot's camera topic
        self.image_sub = rospy.Subscriber(
            "/usb_cam_node/image_raw",
            Image,
            self.camera_callback,
            queue_size=1
        )

        # Estate subscriber: listen to the robot's vision state
        self.active_sub = rospy.Subscriber(
            "/vision_active",
            Bool,
            self.active_callback,
            queue_size=1
        )

        # Publisher: will publish detected color as String on '/detected_color'
        self.color_pub = rospy.Publisher(
            '/detected_color',
            String,
            queue_size=10
        )

        rospy.loginfo("[VIS] BallColorDetectorNode initialized")

    def active_callback(self, msg: Bool):
        self.vision_active = msg.data
        status = "active" if self.vision_active else "inactive"
        rospy.loginfo(f"[VIS] Vision system is now {status}.")
        self.last_process_time = rospy.Time.now()  # Reset last process time

    def camera_callback(self, msg):
        """
        Called whenever a new Image message arrives on /camera/image.
        Converts the ROS Image to OpenCV format, runs color detection,
        and publishes the first detected color (if any), at most once every 0.5s.
        """
        if not self.vision_active:
            #rospy.loginfo("Vision system is inactive, skipping image processing.")
            return
        
        # Check if enough time has passed since the last processing
        now = rospy.Time.now()
        if (now - self.last_process_time).to_sec() < self.process_interval:
            #rospy.loginfo("Skipping image processing, waiting for next interval.")
            return
        
        self.last_process_time = now  # Update last process time

        rospy.loginfo("[VIS] Processing new image...")
        
        try:
            # Convert ROS Image into BGR OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        # Detect colored balls in the current frame
        detected = detect_colored_balls(cv_image, min_radius=10)

        # If at least one color is detected, publish the first one and stop vision
        if detected:
            color_to_publish = detected[0]
            rospy.loginfo(f"[VIS] Detected color: {color_to_publish}")
            self.color_pub.publish(color_to_publish)
            self.vision_active = False
            rospy.loginfo("[VIS] Vision system deactivated after detecting a color.")


    def run(self):
        """
        Keeps the node alive until ROS is shut down.
        """
        rospy.spin()
        # On shutdown, just destroy any OpenCV windows if opened
        cv2.destroyAllWindows()


if __name__ == '__main__':
    node = BallColorDetectorNode()
    node.run()
