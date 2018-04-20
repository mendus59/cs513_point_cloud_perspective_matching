import numpy as np
import math
import cv2
import pymap3d

image_front = image_back = image_left = image_right = None
# Lat, Lon, Alt, Qs, Qx, Qy, Qz
camera_config = []

def read_image_file():
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
    return

def main():
    read_image_file()

if __name__ == "__main__":
    main()