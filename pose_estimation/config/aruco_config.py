# Aruco Config
import cv2 as cv
import numpy as np
import pose_estimation.config.packages as pkg

ARUCO_DIC = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_5X5_1000)

DRONE_DIC = {
    "3":pkg.DroneMarkerConfig(
        id=3,
        drone_name="tiny1",
        size=50                   # [mm]
    )
}

SLAM_MARKER_DIC = {
    "105":pkg.SlamMarkerConfig(
        id="105",
        size=146,
        position=np.array([86.0,1474,0]),
        rvec=np.array([0,0,0])
    ),
    "200":pkg.SlamMarkerConfig(
        id="200",
        size=146,
        position=np.array([1064,1468,0]),
        rvec=np.array([0,0,0])
        ),
    "400":pkg.SlamMarkerConfig(
        id="400",
        size=146,
        position=np.array([1065,70.5,0]),
        rvec=np.array([0,0,0])
        ),
    "600":pkg.SlamMarkerConfig(
        id="600",
        size=146,
        position=np.array([86.0,86.0,0]),
        rvec=np.array([0,0,0])
        )
}