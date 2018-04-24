import numpy as np
import math
import cv2
import pymap3d

def read_config_files():
    # Lat, Lon, Alt, Qs, Qx, Qy, Qz
    with open("data/image/camera.config", "r") as config:
        config.readline()
        secondline = config.readline()
        config = secondline.split(", ")
        for i in range(0, len(config)):
            config[i] = float(config[i])

    point_cloud = []
    with open("data/final_project_point_cloud.fuse") as csv:
        for line in csv:
            point_array = line.split(" ")
            for i in range(0, len(point_array)):
                point_array[i] = float(point_array[i])
            point_cloud.append(point_array)

    return config, point_cloud

def enu_to_camera_coords(east, north, up, camera_config):
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

def convert_point(point, config):
    # Convert point geodetic coordinates (lat, lon, alt) to ECEF coordinates (x, y, z)
    x_ecef, y_ecef, z_ecef = pymap3d.geodetic2ecef(point[0], point[1], point[2])
    
    # Convert point ECEF coordinates (x, y, z) to ENU coordinates (East, North, Up)
    # pymap3d.ecef2enuv(x, y, z, lat0, lon0, h0) where 0 is camera coordinates
    u_east, v_north, w_up = pymap3d.ecef2enu(x_ecef, y_ecef, z_ecef, config[0], config[1], config[2])

    # Convert ENU coordinates to camera perspective
    camera_x, camera_y, camera_z = enu_to_camera_coords(u_east, v_north, w_up,config)

    return camera_x, camera_y, camera_z

def write_to_image_file(write_file, a, b, c):
    xi = int(((a / c) * 1024) + 1024)
    yi = int(((b / c) * 1024) + 1024)
    write_file[xi][yi] = 255

def map_points(point_cloud, config):
    car_x, car_y, car_z = convert_point(config, config)
    front_projection = back_projection = left_projection = right_projection = np.zeros((2048,2048), dtype = float)

    for point in point_cloud:
        camera_x, camera_y, camera_z = convert_point(point, config)
        camera_x = (abs(camera_x)-abs(car_x))
        camera_y = (abs(camera_y)-abs(car_y))
        camera_z = (abs(camera_z)-abs(car_z))

        z_is_positive = ((camera_z) > 0)
        x_is_positive = ((camera_x) > 0)
        z_more_than_x_from_camera = (abs(camera_z) > abs(camera_x))
        x_more_than_z_from_camera = (camera_x > abs(camera_z))
        x_from_camera_more_than_z_from_camera = (abs(camera_x) > abs(camera_z))
        z_more_than_y_from_camera = (camera_z > abs(camera_y))
        x_more_than_y_from_camera = (camera_x > abs(camera_y))
        z_from_camera_more_than_y_from_camera = (abs(camera_z) > abs(camera_y))
        x_from_camera_more_than_y_from_camera = (abs(camera_x) > abs(camera_y))
        neg_x = -camera_x
        neg_z = -camera_z

        if z_is_positive and z_more_than_x_from_camera and z_more_than_y_from_camera:
            write_to_image_file(front_projection, camera_y, camera_x, camera_z)
        elif (not z_is_positive) and (not x_from_camera_more_than_z_from_camera) and z_from_camera_more_than_y_from_camera:
            write_to_image_file(back_projection, camera_y, camera_x, neg_z)

        if x_is_positive and x_more_than_z_from_camera and x_more_than_y_from_camera:
            write_to_image_file(right_projection, camera_y, camera_z, camera_x)
        elif not x_is_positive and x_from_camera_more_than_z_from_camera and x_from_camera_more_than_y_from_camera:
            write_to_image_file(left_projection, camera_y, camera_z, neg_x)

    cv2.imwrite('output/front_projection.png', front_projection)
    cv2.imwrite('output/back_projection.png', back_projection)
    cv2.imwrite('output/left_projection.png', left_projection)
    cv2.imwrite('output/right_projection.png', right_projection)

    return

def find_and_match():
    # Load images
    image_front = cv2.imread("data/image/front.jpg", cv2.IMREAD_COLOR)
    image_back = cv2.imread("data/image/back.jpg", cv2.IMREAD_COLOR)
    image_left = cv2.imread("data/image/left.jpg", cv2.IMREAD_COLOR)
    image_right = cv2.imread("data/image/right.jpg", cv2.IMREAD_COLOR)

    front_projection = cv2.imread("output/front_projection.png")
    back_projection = cv2.imread("output/back_projection.png")
    left_projection = cv2.imread("output/left_projection.png")
    right_projection = cv2.imread("output/right_projection.png")

    # Initiate ORB detector
    orb = cv2.ORB_create()

    front_image_keypoints, front_image_descriptors = orb.detectAndCompute(image_front, None)
    left_image_keypoints, left_image_descriptors = orb.detectAndCompute(image_left, None)
    right_image_keypoints, right_image_descriptors = orb.detectAndCompute(image_right, None)
    back_image_keypoints, back_image_descriptors = orb.detectAndCompute(image_back, None)

    front_projection_keypoints, front_projection_descriptors = orb.detectAndCompute(front_projection, None)
    left_projection_keypoints, left_projection_descriptors = orb.detectAndCompute(left_projection, None)
    right_projection_keypoints, right_projection_descriptors = orb.detectAndCompute(right_projection, None)
    back_projection_keypoints, back_projection_descriptors = orb.detectAndCompute(back_projection, None)

    # Initiate Brute Force matcher
    bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    front_matches = bf_matcher.match(front_image_descriptors, front_projection_descriptors)
    left_matches = bf_matcher.match(left_image_descriptors, left_projection_descriptors)
    right_matches = bf_matcher.match(right_image_descriptors, right_projection_descriptors)
    back_matches = bf_matcher.match(back_image_descriptors, back_projection_descriptors)

    front_matches = sorted(front_matches, key=lambda val: val.distance)
    left_matches = sorted(left_matches, key=lambda val: val.distance)
    right_matches = sorted(right_matches, key=lambda val: val.distance)
    back_matches = sorted(back_matches, key=lambda val: val.distance)

    keypoints = [front_image_keypoints, left_image_keypoints, right_image_keypoints, back_image_keypoints,
        front_projection_keypoints, left_projection_keypoints, right_projection_keypoints, back_projection_keypoints]
    descriptors = [front_image_descriptors, left_image_descriptors, right_image_descriptors, back_image_descriptors,
        front_projection_descriptors, left_projection_descriptors, right_projection_descriptors, back_projection_descriptors]
    matches = [front_matches, left_matches, right_matches, back_matches]

    return keypoints, descriptors, matches

def main():
    camera_config, point_cloud = read_config_files()
    map_points(point_cloud, camera_config)
    keypoints, descriptors, matches = find_and_match()
    # comparison()

if __name__ == "__main__":
    main()
