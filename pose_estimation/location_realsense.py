import cv2 as cv
import pyrealsense2 as rs
import numpy as np
import config.realsense_config as realsense
import config.aruco_config as aruco_config
import config.packages as pkg
from scipy.spatial.transform import Rotation as Rot
import logging, os, time

logging.basicConfig(format='%(asctime)s [%(levelname)s] %(message)s',level=logging.INFO)

class RealsenseLocator():
    def __init__(self, serial_id:str=None, logger=None):
        # RealSense-Pipeline starten
        if logger is None:
            self.logger = logging.getLogger()
        else:
            self.logger = logger

        self.logger.info("Init Realsense Locator")
        self.marker_pos = {}
        self.fps = 0
        self.camera_pos = pkg.CameraPos()
        self.marker_pos_world = {}
        self.list_marker_pos_world = []
        # Init Realsense Camera
        self.init_aruco_detector()
        if serial_id is not None:
            self.init_realsense_camera(serial_id)
        else:
            self.logger.error("No Serial Number given")

    def init_aruco_detector(self):
        # Define Aruco detector 
        self.logger.info("Init Aruco Detector")
        self.aruco_parameters = cv.aruco.DetectorParameters()
        self.aruco_detector = cv.aruco.ArucoDetector(aruco_config.ARUCO_DIC,self.aruco_parameters)

    def init_realsense_camera(self, serial_id:str):
        # Init Realsense camera
        self.logger.info("init Realsense Pipeline")
        self.pipeline = rs.pipeline()
        config = rs.config()
        try:
            config.enable_device(serial_id)
        except:
            self.logger.error(f"No camera with Serial ID: {serial_id} is available")
            return

        config.enable_stream(rs.stream.depth,realsense.PIPELINE_WIDTH,realsense.PIPELINE_HEIGHT, rs.format.z16, realsense.PIPELINE_FPS)
        config.enable_stream(rs.stream.color, realsense.PIPELINE_WIDTH, realsense.PIPELINE_HEIGHT , rs.format.bgr8, realsense.PIPELINE_FPS)
        try:
            self.cfg = self.pipeline.start(config)
        except Exception as e:
            self.logger.error(f"Could not start pipeline: {e}")
            return
        
        # Stream-Profil holen
        profile = self.cfg.get_stream(rs.stream.color) # Fetch stream profile for depth stream
        intr = profile.as_video_stream_profile().get_intrinsics() 

        # Camera Matrix bauen (OpenCV-Format)
        self.camera_matrix = np.array([
            [intr.fx, 0, intr.ppx],
            [0, intr.fy, intr.ppy],
            [0, 0, 1]
        ])

        self.dist_coeffs = np.array(intr.coeffs)
        self.logger.info(f"Realsense Camera {serial_id} inizilized and pipline started")
    
    def get_realsense_image(self):
        # Get realsense Images
        self.logger.debug("Realsense Camera image recieved")
        frames = self.pipeline.wait_for_frames()
        realsense_color_frame = frames.get_color_frame()
        #realsense_depth_frame = frames.get_depth_frame()

        self.realsense_color_image = np.asanyarray(realsense_color_frame.get_data())
        #self.realsense_depth_image = np.asanyarray(realsense_depth_frame.get_data())

    def get_marker_position(self):
        self.logger.debug("Calculate Marker Position")
        corners, ids, _ = self.aruco_detector.detectMarkers(self.realsense_color_image)
        
        # Draw Aruco marker
        cv.aruco.drawDetectedMarkers(self.realsense_color_image, corners, ids)
        if ids is not None:
            for i,corner in enumerate(corners):
                image_points = corner.reshape(4,2).astype(np.float32)
                id_marker = int(ids[i][0])

                if str(id_marker) not in aruco_config.MARKER_DIC:
                    continue
                
                # Define Object points for PnP
                marker_length = aruco_config.MARKER_DIC[str(id_marker)].size
                object_points = np.array([
                    [-marker_length/2,marker_length/2,0],
                    [marker_length/2,marker_length/2,0],
                    [marker_length/2,-marker_length/2,0],
                    [-marker_length/2,-marker_length/2,0]
                ], dtype=np.float32)

                success, rvec, tvec = cv.solvePnP(
                    object_points,
                    image_points,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv.SOLVEPNP_ITERATIVE  # Possible Algorithm cv.SOLVEPNP_SQPNP
                )

                if not success:
                    self.logger.error("Solve PNP Marker not sucessfull")
                    continue

                self.marker_pos[str(id_marker)] = pkg.DroneMarkerState(id=id_marker,tvec=tvec,rvec=rvec)
    
    def get_camera_pos_slam(self):
        # Calculate Camera Pos
        self.logger.debug("Calculate Camera Pos via SLAM")
        corners, ids, _ = self.aruco_detector.detectMarkers(self.realsense_color_image)
        image_points_all = []
        object_points_all = []
        if ids is not None:
            for i, corner in enumerate(corners):

                id_marker = int(ids[i][0])

                # Check if Marker is a Drone
                if str(id_marker) not in aruco_config.SLAM_MARKER_DIC:
                    continue

                slam_marker = aruco_config.SLAM_MARKER_DIC[str(id_marker)]

                # Add all Image and Object points of SLAM Marker for better result
                image_points_all.extend(corner.reshape(4,2))

                marker_length = slam_marker.size
                marker_center_pos = np.array(slam_marker.position, dtype=np.float32)

                object_points = np.array([
                    [-marker_length/2, marker_length/2, 0],
                    [ marker_length/2, marker_length/2, 0],
                    [ marker_length/2,-marker_length/2, 0],
                    [-marker_length/2,-marker_length/2, 0]
                ], dtype=np.float32)

                object_points += marker_center_pos

                object_points_all.extend(object_points)

            # Transform Point list into numpy Array for PnP
            image_points_all = np.array(image_points_all, dtype=np.float32)
            object_points_all = np.array(object_points_all, dtype=np.float32)

            self.logger.debug(f"len image {len(image_points_all)} len object {len(object_points_all)} \n image points {image_points_all} objectpoints {object_points_all}")
            
            if len(image_points_all) == 0:
                self.logger.debug(f"no Image points detected")
                return False

            # Check if condition for PnP is given
            if len(image_points_all) ==  len(object_points_all):
                success, rvec, tvec = cv.solvePnP(
                    object_points_all,
                    image_points_all,
                    self.camera_matrix,
                    self.dist_coeffs,
                    flags=cv.SOLVEPNP_ITERATIVE  # Possible Algorithm cv.SOLVEPNP_SQPNP
                )
                if not success:
                    self.logger.error("Solve PNP SLAM not sucessfull")
                    return False
                # Get Camera Position
                R, _ = cv.Rodrigues(rvec)

                camera_position = -R.T @ tvec
                rot = Rot.from_matrix(R)
                rotation_marker_euler = rot.as_euler('xyz',degrees=True)

                self.camera_pos = pkg.CameraPos(position=camera_position.flatten(),tvec=tvec,rvec=rvec, rotation_euler=rotation_marker_euler.flatten())
                return True
            else:
                self.logger.error("Image Points and Object points are not the same")
        
    def transform_world_coordinate(self):
        # Transform Marker coordinates into World Cordinates
        # --- World -> Camera aus SLAM ---
        R_wc, _ = cv.Rodrigues(self.camera_pos.rvec)
        t_wc = self.camera_pos.tvec.reshape(3,1)

        # --- invertieren: Camera -> World ---
        T_wc = pkg.transformation_4x4_Matrix(R_wc, t_wc)
        T_cw = np.linalg.inv(T_wc)

        for marker_id, marker_state in self.marker_pos.items():

            # Marker Rotation/Translation in Kamera-Coordinates
            R_mc, _ = cv.Rodrigues(marker_state.rvec)
            t_mc = marker_state.tvec.reshape(3,1)

            # Transformation Marker->Camera
            T_mc = pkg.transformation_4x4_Matrix(R_mc, t_mc)

            # Transformation Marker->World
            T_wm = T_cw @ T_mc

            # Extract Rotation and Position
            position_marker_world = T_wm[:3, 3].flatten()
            rotation_marker_world_matrix = T_wm[:3, :3]

            rot = Rot.from_matrix(rotation_marker_world_matrix)
            rotation_marker_world_euler = rot.as_euler('xyz', degrees=True)
            rvec_world_marker, _ = cv.Rodrigues(rotation_marker_world_matrix)

            # Save Marker World Coordinates
            self.marker_pos_world[marker_id] = pkg.DroneMarkerState(
                id=marker_id,
                position=position_marker_world,
                rotation_euler=rotation_marker_world_euler,
                rvec=rvec_world_marker.flatten()
            )
        
        self.list_marker_pos_world.append(self.marker_pos_world)
        self.logger.debug("Marker World Position updated")

    def print_information(self):
        # Print information of marker
        os.system("clear")
        print("############################################################################")
        print(f"FPS: {float(self.fps):.2f}")
        print(f"{self.camera_pos.__str__()}")

        # Print Marker infomation
        for marker in self.marker_pos_world:
            print(f"{self.marker_pos_world[marker].__str__()}")

        print("############################################################################")

    def aruco_detection_mainloop(self):
        while True:
            cycle_start = time.perf_counter()
            self.get_realsense_image()
            self.get_marker_position()
            slam_succ = self.get_camera_pos_slam()
            if slam_succ == True:
                self.transform_world_coordinate()
            
            self.print_information()
            self.fps = 1/(time.perf_counter() - cycle_start)

            cv.imshow("Aruco Detection", self.realsense_color_image)
            cv.waitKey(1)
            
if __name__ == "__main__":
    locate = RealsenseLocator(realsense.REALSENSE_ID)
    try:
        locate.aruco_detection_mainloop()
    
    except Exception as e:
        locate.logger.info(f"Mainloop error {e}")
    
    except KeyboardInterrupt:
        locate.logger.info("Keyboard Interupt")