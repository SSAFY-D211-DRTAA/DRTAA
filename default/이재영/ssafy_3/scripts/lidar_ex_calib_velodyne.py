#!/usr/bin/env python3
# -*- coding:utf-8 -*-

import rospy
import cv2
import numpy as np
import math
import time
from sensor_msgs.msg import PointCloud2, CompressedImage
import sensor_msgs.point_cloud2 as pc2
from numpy.linalg import inv

# This script aligns LiDAR PointCloud2 data with Camera Image data in MORAI SIM.
# It uses Transformation Matrix (Extrinsic) and Camera Matrix (Intrinsic) for alignment.

# Step-by-step process:
# 1. Input Camera and LiDAR installation coordinates and parameters.
# 2. Calculate Extrinsic: Transformation Matrix (LiDAR to Camera Frame).
# 2.1 Implement functions to calculate Sensor to Vehicle Transformation Matrix.
# 2.1.1 Implement function to calculate Rotation Matrix.
# 2.1.2 Implement function to calculate Transformation Matrix.
# 2.2 Calculate LiDAR to Camera Transformation Matrix.
# 3. Calculate Intrinsic: Camera Matrix (Camera to Image Plane).
# 4. Receive LiDAR's PointCloud2 and Camera's Image data.
# 5. Visualize the processed image with PointCloud projected onto the Image.

# TODO: (1) Input Camera and LiDAR installation coordinates and parameters
parameters_cam = {
    "WIDTH": 640,  # image width
    "HEIGHT": 480,  # image height
    "FOV": 90,  # Field of view
    "X": 1.0,  # meter
    "Y": 0.0,
    "Z": 1.5,
    "YAW": 0.0,  # radian
    "PITCH": 0.0,
    "ROLL": 0.0
}

parameters_lidar = {
    "X": 0.0,  # meter
    "Y": 0.0,
    "Z": 1.0,
    "YAW": 0.0,  # radian
    "PITCH": 0.0,
    "ROLL": 0.0
}

def getRotMat(RPY):
    # TODO: (2.1.1) Implement function to calculate Rotation Matrix
    cosR = math.cos(RPY[0])
    cosP = math.cos(RPY[1])
    cosY = math.cos(RPY[2])
    sinR = math.sin(RPY[0])
    sinP = math.sin(RPY[1])
    sinY = math.sin(RPY[2])

    rotRoll = np.array([[1, 0, 0], [0, cosR, -sinR], [0, sinR, cosR]])
    rotPitch = np.array([[cosP, 0, sinP], [0, 1, 0], [-sinP, 0, cosP]])
    rotYaw = np.array([[cosY, -sinY, 0], [sinY, cosY, 0], [0, 0, 1]])

    rotMat = rotYaw.dot(rotPitch.dot(rotRoll))
    return rotMat

def getSensorToVehicleMat(sensorRPY, sensorPosition):
    # TODO: (2.1.2) Implement function to calculate Transformation Matrix
    sensorRotationMat = getRotMat(sensorRPY)
    sensorTranslationMat = np.array(sensorPosition).reshape(3, 1)
    Tr_sensor_to_vehicle = np.vstack((np.hstack((sensorRotationMat, sensorTranslationMat)), [0, 0, 0, 1]))
    return Tr_sensor_to_vehicle

def getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition):
    # TODO: (2.2) Calculate LiDAR to Camera Transformation Matrix
    Tr_lidar_to_vehicle = getSensorToVehicleMat(lidarRPY, lidarPosition)
    Tr_cam_to_vehicle = getSensorToVehicleMat(camRPY, camPosition)
    Tr_vehicle_to_cam = inv(Tr_cam_to_vehicle)
    Tr_lidar_to_cam = Tr_vehicle_to_cam.dot(Tr_lidar_to_vehicle)
    print(Tr_lidar_to_cam)
    return Tr_lidar_to_cam

def getTransformMat(params_cam, params_lidar):
    # Calculate the transformation matrix from LiDAR to Camera
    lidarPositionOffset = np.array([0, 0, -0.25])  # VLP16 offset
    camPositionOffset = np.array([0.1085, 0, 0])  # Camera offset

    camPosition = np.array([params_cam.get(i) for i in (["X", "Y", "Z"])]) + camPositionOffset
    camRPY = np.array([params_cam.get(i) for i in (["ROLL", "PITCH", "YAW"])]) + np.array([-90*math.pi/180, 0, -90*math.pi/180])

    lidarPosition = np.array([params_lidar.get(i) for i in (["X", "Y", "Z"])]) + lidarPositionOffset
    lidarRPY = np.array([params_lidar.get(i) for i in (["ROLL", "PITCH", "YAW"])])

    Tr_lidar_to_cam = getLiDARTOCameraTransformMat(camRPY, camPosition, lidarRPY, lidarPosition)
    return Tr_lidar_to_cam

def getCameraMat(params_cam):
    # TODO: (3) Calculate Camera Matrix (Intrinsic)
    fov_rad = params_cam["FOV"] * math.pi / 180
    focalLength = (params_cam["WIDTH"] / 2) / math.tan(fov_rad / 2)
    principalX = params_cam["WIDTH"] / 2
    principalY = params_cam["HEIGHT"] / 2

    CameraMat = np.array([
        [focalLength, 0, principalX],
        [0, focalLength, principalY],
        [0, 0, 1]
    ])
    print(CameraMat)
    return CameraMat

class LiDARToCameraTransform:
    def __init__(self, params_cam, params_lidar):
        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.scan_callback)
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.img_callback)
        self.pc_np = None
        self.img = None
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]
        self.TransformMat = getTransformMat(params_cam, params_lidar)
        self.CameraMat = getCameraMat(params_cam)

    # TODO: (4) Receive LiDAR's PointCloud2 and Camera's Image data
    def img_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def scan_callback(self, msg):
        point_list = []
        for point in pc2.read_points(msg, skip_nans=True):
            point_list.append((point[0], point[1], point[2], 1))
        self.pc_np = np.array(point_list, np.float32)

    # TODO: (5.1) Transform LiDAR Pointcloud to Camera Frame
    def transformLiDARToCamera(self, pc_lidar):
        pc_wrt_cam = self.TransformMat.dot(pc_lidar)
        pc_wrt_cam = np.delete(pc_wrt_cam, 3, axis=0)
        return pc_wrt_cam

    # TODO: (5.2) Project Camera Frame PointCloud to Image Plane with Filtering
    def transformCameraToImage(self, pc_camera):
        pc_proj_to_img = self.CameraMat.dot(pc_camera)
        pc_proj_to_img = np.delete(pc_proj_to_img, np.where(pc_proj_to_img[2, :] < 0), axis=1)
        pc_proj_to_img /= pc_proj_to_img[2, :]
        pc_proj_to_img = np.delete(pc_proj_to_img, np.where(pc_proj_to_img[0, :] > self.width), axis=1)
        pc_proj_to_img = np.delete(pc_proj_to_img, np.where(pc_proj_to_img[1, :] > self.height), axis=1)
        return pc_proj_to_img

def draw_pts_img(img, xi, yi):
    point_np = img
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (0, 255, 0), -1)
    return point_np

if __name__ == '__main__':
    rospy.init_node('ex_calib', anonymous=True)
    Transformer = LiDARToCameraTransform(parameters_cam, parameters_lidar)
    time.sleep(1)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        xyz_p = Transformer.pc_np[:, 0:3]
        xyz_p = np.insert(xyz_p, 3, 1, axis=1).T
        xyz_p = np.delete(xyz_p, np.where(xyz_p[0, :] < 0), axis=1)

        xyz_c = Transformer.transformLiDARToCamera(xyz_p)
        xy_i = Transformer.transformCameraToImage(xyz_c)

        # TODO: (6) Visualize the processed image with PointCloud projected onto the Image
        xy_i = xy_i.astype(np.int32)
        projectionImage = draw_pts_img(Transformer.img, xy_i[0, :], xy_i[1, :])
        cv2.imshow("LidartoCameraProjection", projectionImage)
        cv2.waitKey(1)