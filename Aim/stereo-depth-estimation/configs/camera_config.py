# -*-coding: utf-8 -*-
import numpy as np


def camera_stereo():
    """Intel RealSense depth camera"""
    width = 640
    height = 480
    intrinsic = np.asarray([[7.6159209686584518e+02, 0., 3.2031427422505453e+02],
                            [0., 7.6167321445963728e+02, 2.2467546927337131e+02],
                            [0., 0., 1.]])
    D1 = [3.4834574885170888e-02, -5.5261651661983137e-02, 5.7491952731614823e-04,
          -4.2764224824172658e-05, 1.8477350140315381e-02]
    config = {}
    config["size"] = (width, height)  # 图像分辨率
    config["fx"] = intrinsic[0, 0]
    config["fy"] = intrinsic[1, 1]
    config["cx"] = intrinsic[0, 2]
    config["cy"] = intrinsic[1, 2]
    config["K1"] = intrinsic  # 相机内参
    config["D1"] = D1
    return config


def camera_realsense():
    """Intel RealSense depth camera"""
    width = 1280
    height = 720
    intrinsic = np.asarray([[920.003, 0., 640.124],
                            [0., 919.888, 358.495],
                            [0., 0., 1.]])
    config = {}
    config["size"] = (width, height)  # 图像分辨率
    config["fx"] = intrinsic[0, 0]
    config["fy"] = intrinsic[1, 1]
    config["cx"] = intrinsic[0, 2]
    config["cy"] = intrinsic[1, 2]
    config["K1"] = intrinsic  # 相机内参
    config["D1"] = None
    return config


def camera_kinectv2():
    """kinectv2 camera"""
    width = 512
    height = 424
    intrinsic = np.asarray([[364.032, 0., 258.891],
                            [0., 364.032, 209.32],
                            [0., 0., 1.]])
    # intrinsic = np.asarray([[366.3530, 0., 262.8683],
    #                         [0., 366.7980, 208.1488],
    #                         [0., 0., 1.]])
    config = {}
    config["size"] = (width, height)  # 图像分辨率
    config["fx"] = intrinsic[0, 0]
    config["fy"] = intrinsic[1, 1]
    config["cx"] = intrinsic[0, 2]
    config["cy"] = intrinsic[1, 2]
    config["K1"] = intrinsic  # 相机内参
    config["D1"] = None
    return config
