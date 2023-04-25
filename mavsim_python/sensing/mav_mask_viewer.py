"""
Adam Welker     MEEN 674    Winter 23

mav_mask_viewer.py -- makes a viewer
for a digital camera simulator
"""
import cv2 as cv
import numpy as np

from sensing.digital_camera import Simulated_Camera

class MAVMaskViewer:

    _camera = None


    def __init__(self, camera: Simulated_Camera) -> None:
        
        self._camera = camera



    def update(self, cam_position: np.ndarray, cam_orientation: np.ndarray, target_position:np.ndarray, target_orientation: np.ndarray) -> None:
        """
        Given an object's position, returns an mask image with the object. Assumes object is a sphere

        :param cam_position: the 3d intertial position of the camera in the 
        the from [[n],[e],[d]] or similar

        :param cam_position: the 3 angle rotation of the camera in the 
        the form [[phi],[theta],[psi]] or similar

        :param mav_position: the 3d intertial position of the mav in the 
        the from [[n],[e],[d]] or similar
        :param radius: the radius of the object in meters

        :return: Nothing
        """

        self._camera.set_pose(cam_position, cam_orientation)
        image = self._camera.draw_mav(target_position,target_orientation)

        cv.imshow('Defender View', image)

             
    