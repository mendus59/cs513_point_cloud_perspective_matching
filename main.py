"""
Module compares a point cloud to perspective images,
matches them, then calculates angle of error
"""

import math
import numpy as np
import cv2
import pymap3d

def read_config_files():
    """ Returns config parameters and point cloud in array form """
    # Lat, Lon, Alt, Qs, Qx, Qy, Qz
    with open("data/image/camera.config", "r") as config:
        config.readline()
        secondline = config.readline()
        config = secondline.split(", ")
        for i in range(0, len(config)):
            config[i] = float(config[i])

    point_cloud = []
    with open("data/point_cloud.fuse") as csv:
        for line in csv:
            point_array = line.split(" ")
            for i in range(0, len(point_array)):
                point_array[i] = float(point_array[i])
            point_cloud.append(point_array)

    return config, point_cloud

def enu_to_camera_coords(east, north, up, camera_config):
    """ Convert East North Up coordinates to Camera Coordinates """
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
    """
    Convert Point to geodetic coordinates to ECEF coordinates to ENU coordinates
    and return camera perspective coordinates
    """
    # Convert point geodetic coordinates (lat, lon, alt) to ECEF coordinates (x, y, z)
    x_ecef, y_ecef, z_ecef = pymap3d.geodetic2ecef(point[0], point[1], point[2])
    # Convert point ECEF coordinates (x, y, z) to ENU coordinates (East, North, Up)
    # pymap3d.ecef2enuv(x, y, z, lat0, lon0, h0) where 0 is camera coordinates
    u_east, v_north, w_up = pymap3d.ecef2enu(x_ecef, y_ecef, z_ecef, config[0], config[1], config[2])
    # Convert ENU coordinates to camera perspective
    camera_x, camera_y, camera_z = enu_to_camera_coords(u_east, v_north, w_up, config)

    return camera_x, camera_y, camera_z

def write_to_image_file(write_file, a, b, c):
    """ Calculate pixel that should be made white """
    xi = (int(((a / c) * 1024) + 1024)) % 2048
    yi = (int(((b / c) * 1024) + 1024)) % 2048
    write_file[xi][yi] = 255

def map_points(point_cloud, config):
    """ Map point cloud to perspective images """
    car_x, car_y, car_z = convert_point(config, config)
    front_projection = back_projection = left_projection = right_projection = np.zeros((2048, 2048), dtype = float)

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

def load_images():
    """
    Load perspective and projection images and
    return an array with images and projection with
    order Front, Left, Right, Back
    """
    images = []

    image_front = cv2.imread("data/image/front.jpg", cv2.IMREAD_COLOR)
    images.append(image_front)
    image_left = cv2.imread("data/image/left.jpg", cv2.IMREAD_COLOR)
    images.append(image_left)
    image_right = cv2.imread("data/image/right.jpg", cv2.IMREAD_COLOR)
    images.append(image_right)
    image_back = cv2.imread("data/image/back.jpg", cv2.IMREAD_COLOR)
    images.append(image_back)

    front_projection = cv2.imread("output/front_projection.png")
    images.append(front_projection)
    left_projection = cv2.imread("output/left_projection.png")
    images.append(left_projection)
    right_projection = cv2.imread("output/right_projection.png")
    images.append(right_projection)
    back_projection = cv2.imread("output/back_projection.png")
    images.append(back_projection)

    return images

def find_and_match():
    """
    Use an ORB detector to match images
    and a Brute Force matcher to match them
    """
    images = load_images()

    # Initiate ORB detector
    orb = cv2.ORB_create()

    front_img_keys, front_img_descripts = orb.detectAndCompute(images[0], None)
    left_img_keys, left_img_descripts = orb.detectAndCompute(images[1], None)
    right_img_keys, right_img_descripts = orb.detectAndCompute(images[2], None)
    back_img_keys, back_img_descripts = orb.detectAndCompute(images[3], None)

    front_prj_keys, front_prj_descripts = orb.detectAndCompute(images[4], None)
    left_prj_keys, left_prj_descripts = orb.detectAndCompute(images[5], None)
    right_prj_keys, right_prj_descripts = orb.detectAndCompute(images[6], None)
    back_prj_keys, back_prj_descripts = orb.detectAndCompute(images[7], None)

    # Initiate Brute Force matcher
    bf_matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    front_matches = bf_matcher.match(front_img_descripts, front_prj_descripts)
    left_matches = bf_matcher.match(left_img_descripts, left_prj_descripts)
    right_matches = bf_matcher.match(right_img_descripts, right_prj_descripts)
    back_matches = bf_matcher.match(back_img_descripts, back_prj_descripts)

    front_matches = sorted(front_matches, key=lambda val: val.distance)
    left_matches = sorted(left_matches, key=lambda val: val.distance)
    right_matches = sorted(right_matches, key=lambda val: val.distance)
    back_matches = sorted(back_matches, key=lambda val: val.distance)

    keypoints = [front_img_keys, left_img_keys, right_img_keys,
                 back_img_keys, front_prj_keys, left_prj_keys,
                 right_prj_keys, back_prj_keys]
    descriptors = [front_img_descripts, left_img_descripts, right_img_descripts, back_img_descripts,
        front_prj_descripts, left_prj_descripts, right_prj_descripts, back_prj_descripts]
    matches = [front_matches, left_matches, right_matches, back_matches]

    return keypoints, descriptors, matches

def calculate_error_angle(image, img_keys, prj_keys, matches):
    """
    Calculates the angle of error between a perspective image
    and the image created from a projected point cloud
    """
    angles = []
    for match in matches:
        endpoint_1 = tuple(np.round(img_keys[match.trainIdx].pt).astype(int))
        endpoint_2 = tuple(np.round(prj_keys[match.queryIdx].pt).astype(int) + np.array([image.shape[1], 0]))
        angles.append(math.atan((float)(endpoint_2[1] - endpoint_1[1]) / (endpoint_1[0] - endpoint_2[0])) * (180 / math.pi))
    
    return sum(angles)/len(angles)

def get_error_angles(keypoints, matches):
    """ Calls 'calculate_error_angle' for each perspective/projection pair """
    error_angles = []
    images = load_images()
    front_angle = calculate_error_angle(images[0], keypoints[0], keypoints[4], matches[0])
    error_angles.append(front_angle)
    left_angle = calculate_error_angle(images[1], keypoints[1], keypoints[5], matches[1])
    error_angles.append(left_angle)
    right_angle = calculate_error_angle(images[2], keypoints[2], keypoints[6], matches[2])
    error_angles.append(right_angle)
    back_angle = calculate_error_angle(images[3], keypoints[3], keypoints[7], matches[3])
    error_angles.append(back_angle)

    return error_angles

def output_errors(error_angles):
    """ Prints Angle Errors between perspective images and projection images """
    print("Front projection to perspective angle error: ", error_angles[0])
    print("Left projection to perspective angle error: ", error_angles[1])
    print("Right projection to perspective angle error: ", error_angles[2])
    print("Back projection to perspective angle error: ", error_angles[3])
    return

def main():
    """ Main Script Runner """
    camera_config, point_cloud = read_config_files()
    map_points(point_cloud, camera_config)
    keypoints, descriptors, matches = find_and_match()
    error_angles = get_error_angles(keypoints, matches)
    output_errors(error_angles)

if __name__ == "__main__":
    main()
