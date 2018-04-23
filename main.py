import numpy as np
import math
import cv2
import pymap3d

image_front = image_back = image_left = image_right = None
# Lat, Lon, Alt, Qs, Qx, Qy, Qz
camera_config = []
point_cloud = []

def read_image_files():
    
    image_front = cv2.imread("data/image/front.jpg", cv2.IMREAD_COLOR)
    image_back = cv2.imread("data/image/back.jpg", cv2.IMREAD_COLOR)
    image_left = cv2.imread("data/image/left.jpg", cv2.IMREAD_COLOR)
    image_right = cv2.imread("data/image/right.jpg", cv2.IMREAD_COLOR)

    with open("data/image/camera.config", "r") as config:
        config.readline()
        secondline = config.readline()
        config = secondline.split(", ")
        for i in range(0, len(config)):
            camera_config[i] = float(config[i])

    with open("data/point_cloud.fuse") as csv:
        for line in csv:
            point_array = line.split(" ")
            for i in range(0, len(point_array)):
                point_array[i] = float(point_array[i])
            point_cloud.append(point_array)

    return

def enu_to_camera_coords(east, north, up):
    qs = camera_config[3]
    qx = camera_config[4]
    qy = camera_config[5]
    qz = camera_config[6]
    row_1_col_1 = (qs * qs) + (qx * qx) - (qy * qy) - (qz * qz)
    row_1_col_2 = (2 * qx * qy) - (2 * qs * qz)
    row_1_col_3 = (2 * qx * qz) + (2 * qs * qy)
    row_2_col_1 = (2 * qx * qy) + (2 * qs * qz)
    row_2_col_2 = (qs * qs) - (qx * qx) + (qy * qy) - (qz * qz)
    row_2_col_3 = (2 * qz * qy) - (2 * qs * qx)
    row_3_col_1 = (2 * qx * qz) - (2 * qs * qy)
    row_3_col_2 = (2 * qz * qy) + (2 * qs * qx)
    row_3_col_3 = (qs * qs) - (qx * qx) - (qy * qy) + (qz * qz)
    rq = [[row_1_col_1, row_1_col_2, row_1_col_3], [row_2_col_1, row_2_col_2, row_2_col_3], [row_3_col_1, row_3_col_2, row_3_col_3]]
    camera_coordinates = np.dot(rq, [north, east, -up])
    x = camera_coordinates[0]
    y = camera_coordinates[1]
    z = camera_coordinates[2]
    return x, y, z

def convert_point(point):
    # Convert point geodetic coordinates (lat, lon, alt) to ECEF coordinates (x, y, z)
    x_ecef, y_ecef, z_ecef = pymap3d.geodetic2ecef(point[0], point[1], point[2])
    
    # Convert point ECEF coordinates (x, y, z) to ENU coordinates (East, North, Up)
    # pymap3d.ecef2enuv(x, y, z, lat0, lon0, h0) where 0 is camera coordinates
    u_east, v_north, w_up = pymap3d.ecef2enu(x_ecef, y_ecef, z_ecef, camera_config[0], camera_config[1], camera_config[2])

    # Convert ENU coordinates to camera perspective
    camera_x, camera_y, camera_z = enu_to_camera_coords(u_east, v_north, w_up)

    return camera_x, camera_y, camera_z

def map_points():
    for point in point_cloud:
        camera_x, camera_y, camera_z = convert_point(point)

def main():
    read_image_files()
    map_points()
    # Convert geodetic2ecef (cartesian). returns x, y, z
    # Calculate sin and cos: cos_lat = (math.cos(lat * math.pi / 180.0))
    # Convert ecef2enu
    # Matrix transformation
    # Create images from point cloud
    # Detect and match keypoints

if __name__ == "__main__":
    main()
