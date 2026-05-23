import ctypes
from dataclasses import dataclass,field
import numpy as np

@dataclass(frozen=True,slots=True)
class SlamMarkerConfig():
    id: int
    size:float
    position:np.ndarray
    rvec:np.ndarray

@dataclass(slots=True)
class CameraPos():
    position:np.ndarray = field(default_factory=lambda:np.zeros(3))
    tvec:np.ndarray = field(default_factory=lambda:np.zeros(3))
    rvec:np.ndarray = field(default_factory=lambda:np.zeros(3))
    rotation_euler:np.ndarray = field(default_factory=lambda:np.zeros(3))
    fps:float = 0

    def __str__(self):
        return(
            f"############### Camera #################\n"
            f"postition     : x: {self.position[0]:.2f} [mm] y: {self.position[1]:.2f} [mm] z: {self.position[2]:.2f} [mm] \n"
            f"rotation (deg): roll: {self.rotation_euler[0]:.2f} [°] pitch: {self.rotation_euler[1]:.2f} [°] yaw: {self.rotation_euler[2]:.2f}"
        )
class MSGDroneMarker(ctypes.Structure):
    _pack_ = 1
    _fields_ = [("drone_name",ctypes.c_char),
                ("id",ctypes.c_char),
                ("x",ctypes.c_float),
                ("y",ctypes.c_float),
                ("z",ctypes.c_float),
                ("omega_x",ctypes.c_float),
                ("omega_y",ctypes.c_float),
                ("omega_z",ctypes.c_float)
                ]

@dataclass(frozen=True,slots=True)
class DroneMarkerConfig():
    id: int
    drone_name:str
    size:float
    
@dataclass(slots=True)
class DroneMarkerState():
    id:int
    tvec:np.ndarray = field(default_factory=lambda:np.zeros(3))
    rvec:np.ndarray = field(default_factory=lambda:np.zeros(3))
    position:np.ndarray = field(default_factory=lambda:np.zeros(3))
    rotation_euler:np.ndarray = field(default_factory=lambda:np.zeros(3))

    def __str__(self):
        return(
            f"############### Marker {self.id} #################\n"
            f"postition     : x: {self.position[0]:.2f} [mm] y: {self.position[1]:.2f} [mm] z: {self.position[2]:.2f} [mm] \n"
            f"rotation (deg): roll: {self.rotation_euler[0]:.2f} [°] pitch: {self.rotation_euler[1]:.2f} [°] yaw: {self.rotation_euler[2]:.2f}"
        )

    def __post_init__(self):
        self.position = np.asarray(self.position, dtype=float).reshape(3)
        self.rvec = np.asarray(self.rvec, dtype=float).reshape(3)

def transformation_4x4_Matrix(R,t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t.flatten()
    return T