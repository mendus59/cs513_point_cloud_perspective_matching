import numpy as np
import math
import cv2
import pymap3d

image_front = image_back = image_left = image_right = None
# Lat, Lon, Alt, Qs, Qx, Qy, Qz
camera_config = []
point_cloud = []

def cartesian(lat, lon, alt):
    EARTH_RADIUS = 6378137
    cosLat = (math.cos(lat * math.pi / 180.0))
    sinLat = (math.sin(lat * math.pi / 180.0))
    cosLon = (math.cos(lon * math.pi / 180.0))
    sinLon = (math.sin(lon * math.pi / 180.0))
    a = EARTH_RADIUS+ alt
    f = 1.0 / 298.257224
    e = math.sqrt(f*(2-f))
    N = a / math.sqrt(1 - e * e * sinLat  *sinLat)
    x = (N + alt) * cosLon * cosLat
    y = (N + alt) * cosLon * sinLat
    z = ((1 - e * e) * N + alt) * sinLon
    return x, y, z, cosLat, cosLon, sinLat, sinLon

def read_image_files():
    image_front = cv2.imread("data/image/front.jpg")
    image_back = cv2.imread("data/image/back.jpg")
    image_left = cv2.imread("data/image/left.jpg")
    image_right = cv2.imread("data/image/right.jpg")

    with open("data/image/camera.config", "r") as camera_config:
        firstline = camera_config.readline()
        secondline = camera_config.readline()
        camera_config = secondline.split(", ")
        for i in range(0, len(camera_config)):
            camera_config[i] = float(camera_config[i])

    with open("data/point_cloud.fuse") as csv:
        for line in csv:
            point_array = line.split(" ")
            for i in range(0, len(point_array)):
                point_array[i] = float(point_array[i])
            point_cloud.append(point_array)

    return

def main():
    read_image_files()

if __name__ == "__main__":
    main()