import cv2
import numpy as np
import matplotlib.pyplot as plt

def display_depth_images_with_colorbar(depth_img1, depth_img2, title1='Depth Image 1', title2='Depth Image 2'):
    # Create a figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Display the first depth image
    im1 = ax1.imshow(depth_img1, cmap='viridis')
    ax1.set_title(title1)
    ax1.set_axis_off()

    # Display the second depth image
    im2 = ax2.imshow(depth_img2, cmap='viridis')
    ax2.set_title(title2)
    ax2.set_axis_off()

    # Add colorbars to the subplots
    cbar1 = fig.colorbar(im1, ax=ax1, orientation='vertical', fraction=0.046, pad=0.04)
    cbar2 = fig.colorbar(im2, ax=ax2, orientation='vertical', fraction=0.046, pad=0.04)

    # Display the colorbars
    plt.show()

if __name__ == "__main__":
    # Replace these paths with the paths to your depth images
    depth_image_path1 = f'files/test_depth_img/redwood_01053_1_depth.png'
    depth_image_path2 = f'files/test_depth_img/tum_fr1desk_1_depth.png'

    # Load depth images
    depth_img1 = cv2.imread(depth_image_path1, cv2.IMREAD_UNCHANGED)
    depth_img2 = cv2.imread(depth_image_path2, cv2.IMREAD_UNCHANGED)

    # Check if the images are loaded successfully
    if depth_img1 is None or depth_img2 is None:
        print("Error: Unable to load depth images.")
    else:
        # Display the depth images with colorbars without normalization
        display_depth_images_with_colorbar(depth_img1, depth_img2)