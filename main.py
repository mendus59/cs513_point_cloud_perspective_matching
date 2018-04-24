import numpy as np
import math
import cv2
import pymap3d

image_front = image_back = image_left = image_right = None
point_cloud = []

front_projection = back_projection = left_projection = right_projection = np.zeros((2048,2048), dtype = float)

# Lat, Lon, Alt, Qs, Qx, Qy, Qz
camera_config = []
with open("data/image/camera.config", "r") as config:
    config.readline()
    secondline = config.readline()
    config = secondline.split(", ")
    for i in range(0, len(config)):
        config[i]=float(config[i])
    camera_config.append(float(config[i]))
camera_config=config
print(camera_config)
def read_image_files():
    
    image_front = cv2.imread("data/image/front.jpg", cv2.IMREAD_COLOR)
    image_back = cv2.imread("data/image/back.jpg", cv2.IMREAD_COLOR)
    image_left = cv2.imread("data/image/left.jpg", cv2.IMREAD_COLOR)
    image_right = cv2.imread("data/image/right.jpg", cv2.IMREAD_COLOR)
   
    
    with open("data/final_project_point_cloud.fuse") as csv:
        for line in csv:
            point_array = line.split(" ")
            for i in range(0, len(point_array)):
                point_array[i] = float(point_array[i])
            point_cloud.append(point_array)

    return point_cloud

def enu_to_camera_coords(east, north, up,camera_config):
    
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
    camera_x, camera_y, camera_z = enu_to_camera_coords(u_east, v_north, w_up,camera_config)

    return camera_x, camera_y, camera_z

def write_to_image_file(write_file, a, b, c):
    xi = int( ((a / c) * 1024) + 1024)
    yi = int( ((b / c) * 1024) + 1024)
    write_file[xi][yi] = 255

def map_points(point_cloud):
    car_x,car_y,car_z=convert_point(camera_config)

    for point in point_cloud:
##        car_x=camera_config[0]
##        print(car_x)
##        car_y=camera_config[1]
##        print(car_y)
##        car_z=camera_config[2]
##        print(car_z)
##        car_x,car_y,car_z=convert_point(camera_config,camera_config)

        camera_x, camera_y, camera_z = convert_point(point)
        print(car_x,car_y,car_z)

        print(camera_x,camera_y,camera_z)
        camera_x=(abs(camera_x)-abs(car_x))
        print(camera_x)
        camera_y=(abs(camera_y)-abs(car_y))
##        print(camera_y)

        camera_z=(abs(camera_z)-abs(car_z))
        z_is_positive = ((camera_z) > 0)
        y_is_positive= ((camera_z) > 0)
        x_is_positive = ((camera_x) > 0)
        z_more_than_x_from_camera = (abs(camera_z) > abs(camera_x))
        
        x_more_than_z_from_camera = (camera_x > abs(camera_z))
        x_from_camera_more_than_z_from_camera = (abs(camera_x) > abs(camera_z))
        z_more_than_y_from_camera = (camera_z > abs(camera_y))
        x_more_than_y_from_camera = (camera_x > abs(camera_y))
        z_from_camera_more_than_y_from_camera = (abs(camera_z) > abs(camera_y))
        x_from_camera_more_than_y_from_camera = (abs(camera_x) > abs(camera_y))
        neg_x=-camera_x
        neg_z=-camera_z
        if z_is_positive and z_more_than_x_from_camera and z_more_than_y_from_camera:
            write_to_image_file(front_projection, camera_y, camera_x, camera_z)
        elif (not z_is_positive) and (not x_from_camera_more_than_z_from_camera) and z_from_camera_more_than_y_from_camera:
            write_to_image_file(back_projection, camera_y, camera_x, neg_z)

        if x_is_positive and x_more_than_z_from_camera and x_more_than_y_from_camera:
            write_to_image_file(right_projection, camera_y, camera_z, camera_x)
        elif not x_is_positive and x_from_camera_more_than_z_from_camera and x_from_camera_more_than_y_from_camera:
            write_to_image_file(left_projection, camera_y, camera_z, neg_x)

 

    return
    
def main():
    point_cloud1=read_image_files()
##    print(camera_config1)
##    print(point_cloud1[0])
    map_points(point_cloud1)
    cv2.imwrite('front_projection.png', front_projection)
    cv2.imwrite('back_projection.png', back_projection)
    cv2.imwrite('left_projection.png', left_projection)
    cv2.imwrite('right_projection.png', right_projection)
    # Matrix transformation
    # Create images from point cloud
    # Detect and match keypoints
    # comparison()

if __name__ == "__main__":
    main()
