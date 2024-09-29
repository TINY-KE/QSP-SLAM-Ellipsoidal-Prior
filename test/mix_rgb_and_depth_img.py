import cv2
import numpy as np
import matplotlib.pyplot as plt

def overlay_depth_rgb(depth_img, rgb_img, alpha=0.5):
    # Normalize depth image for visualization
    normalized_depth = cv2.normalize(depth_img, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

    # Convert depth image to 3 channels for compatibility with RGB image
    depth_colored = cv2.cvtColor(normalized_depth, cv2.COLOR_GRAY2BGR)

    # Blend the depth image with the RGB image
    overlay = cv2.addWeighted(rgb_img, 1 - alpha, depth_colored, alpha, 0)

    return overlay

if __name__ == "__main__":
    # Replace these paths with the paths to your depth and RGB images
    depth_image_path = f'files/test_depth_img/redwood_01053_1_depth.png'
    rgb_image_path = f'files/test_depth_img/redwood_01053_1_rgb.jpg'

    # Load depth and RGB images
    depth_img = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    rgb_img = cv2.imread(rgb_image_path)

    # Check if the images are loaded successfully
    if depth_img is None or rgb_img is None:
        print("Error: Unable to load images.")
    else:
        # Overlay depth and RGB images
        overlay = overlay_depth_rgb(depth_img, rgb_img)

        # Display the overlay image
        plt.imshow(cv2.cvtColor(overlay, cv2.COLOR_BGR2RGB))
        plt.axis('off')
        plt.show()