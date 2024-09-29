import cv2
import numpy as np

def read_depth_image(image_path):
    # 读取深度图像
    depth_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        raise FileNotFoundError(f"Unable to read the depth image at path: {image_path}")

    return depth_image

def calculate_min_max_depth(depth_image):
    # 计算深度图像中的最大和最小值
    min_depth = np.min(depth_image)
    max_depth = np.max(depth_image)

    return min_depth, max_depth

def evaluate_depth_img(depth_image_path):
    # 读取深度图像
    depth_image = read_depth_image(depth_image_path)

    print(f"depth_image = \n{depth_image}")

    # 计算最大和最小深度值
    min_depth, max_depth = calculate_min_max_depth(depth_image)

    # 打印结果
    # print(f"Type: {depth_image.dtype()}")
    print(f"Minimum Depth: {min_depth}")
    print(f"Maximum Depth: {max_depth}")

    cv2.imshow("Depth Image", depth_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 指定深度图像路径
    depth_image_path1 = "/media/lj/TOSHIBA/dataset/FromZHJD/circle/depth/000042.png"
    depth_image_path2 = "/home/lj/Documents/dataset/RedwoodOS/data/rgbd/01053/depth/0000001-000000000000.png"
    # depth_image_path = "path/to/your/depth/image.png"

    evaluate_depth_img(depth_image_path1)

    evaluate_depth_img(depth_image_path2)

    cv2.destroyAllWindows()

