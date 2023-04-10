# Adam Welker   MEEN 674    Winter 2023
#
# pinhole_camera.py -- a class that implements 
# a pinhole camera (relative to the pinhole camera's frame).
import numpy as np
from sys import path
path.append('..')
from tools.rotations import *
from pinhole_camera import *
import cv2 as cv

class Simulated_Camera:

    def __init__(self, focal_distance: float, resolution: np.ndarray, image_size: np.ndarray) -> None:
        """
        Initializes the Camera Simulator

        :param focal distance: Focal distance in meters of the camera
        :param resolution: The pixel resolution of the camera in the format [x_pixels, y_pixels]
        :param image_size: The size of the camera's image plane

        :return: None
        """
        
        self.focal_distance = focal_distance
        self.resolution = resolution
        self.image_size = image_size

        self.pin_hole = Pinhole_Camera(self.focal_distance)

        self.image = np.zeros((resolution.item(0), resolution.item(1)), dtype=int)
        self.position = np.zeros((3,1))
        self.orientation = np.zeros((3,1))

    
    def is_in_field_of_view(self, obj_position:np.ndarray) -> bool:
        """
        Given the position of an object in the intertial frame, returns 
        a bool of whether the object is the in the field of view of the camera

        :param obj_position: the 3d intertial position of the object in the 
        the from [[n],[e],[d]] or similar

        :return: true if the object is in the frame, false if not.
        """

        phi = self.orientation.item(0)
        theta = self.orientation.item(1)
        psi = self.orientation.item(2)
    
        # Get the location of the object in the inertial plane
        R__i_to_c = Euler2Rotation(phi, theta, psi).T
        
        position_cam_frame = R__i_to_c @ obj_position

        x_0 = position_cam_frame.item(0)
        y_0 = position_cam_frame.item(1)
        z_0 = position_cam_frame.item(2)

        # Check if object is behind focal

        if z_0 <= 0:
            
            return False
        

        # check if the object is out of the field of view of the image plane
        # (based on image size)
        x_i = self.pin_hole.single_dimension_projection(x_0, z_0)
        z_i = self.pin_hole.single_dimension_projection(y_0, z_0)

        if abs(x_i) > self.image_size.item(0) or abs(y_i) > self.image_size.item(1):

            return False

        return True 

        

    def get_image_size(self, obj_position:np.ndarray, radius: float) -> float:
        """
        Given an object's position, finds its size in the image plane. Assumes the object is a sphere

        :param obj_position: the 3d intertial position of the object in the 
        the from [[n],[e],[d]] or similar
        :param radius: the radius of the object in meters

        :return: The object's diameter as a float
        """

        phi = self.orientation.item(0)
        theta = self.orientation.item(1)
        psi = self.orientation.item(2)
    
        # Get the location of the object in the inertial plane
        R__i_to_c = Euler2Rotation(phi, theta, psi).T
        
        position_cam_frame = R__i_to_c @ obj_position

        z_0 = position_cam_frame.item(2)

        # Now get radius size

        radius_i = self.pin_hole.single_dimension_projection(radius, z_0)

        return radius_i

    
    def get_image_location(self, obj_position:np.ndarray) -> np.ndarray:
        """
        Given an object's position, finds its center locatioon in the image plane. Assumes the object is a sphere

        :param obj_position: the 3d intertial position of the object in the 
        the from [[n],[e],[d]] or similar

        :return: The object's location in the form [xi, yi]
        """

        phi = self.orientation.item(0)
        theta = self.orientation.item(1)
        psi = self.orientation.item(2)
    
        # Get the location of the object in the inertial plane
        R__i_to_c = Euler2Rotation(phi, theta, psi).T
        
        position_cam_frame = R__i_to_c @ obj_position

        x_0 = position_cam_frame.item(0)
        y_0 = position_cam_frame.item(1)
        z_0 = position_cam_frame.item(2)

        location_i = self.pin_hole.get_image_projection(np.array([[x_0, y_0]]).T, z_0)

        return location_i


    def get_image(self, obj_position:np.ndarray, radius: float) -> np.ndarray:
        """
        Given an object's position, returns an mask image with the object. Assumes object is a sphere

        :param obj_position: the 3d intertial position of the object in the 
        the from [[n],[e],[d]] or similar
        :param radius: the radius of the object in meters

        :return: The mask image of the object as a numpy array
        """

        self.image = np.zeros((self.resolution.item(0), self.resolution.item(1)))

        location = self.get_image_location(obj_position)

        rad_i = self.get_image_size(obj_position, radius)

        x_step = self.image_size.item(0) / self.resolution.item(0)
        y_step = self.image_size.item(1) / self.resolution.item(1)

        for i in range(0, len(self.image)):

            for j in (range(0, len(self.image[0]))):

                # Map the current pixel to a physical location on the image plane
                x_location = -self.image_size.item(0) / 2 + i*x_step
                y_location = self.image_size.item(1) / 2 - j*y_step

                # If the object is visible in that location, then light the pixel up
                if (x_location - location.item(0))**2 + (y_location - location.item(1))**2 <= rad_i**2:

                    self.image[i][j] = 1



        return self.image


    def set_pose(self, position: np.ndarray, orientation: np.ndarray) -> None:
        """
        Set's the camera's pose

        :param obj_position: the 3d intertial position of the object in the 
        the from [[n],[e],[d]] or similar

        :param obj_position: the 3 angle rotation of the object in the 
        the form [[phi],[theta],[psi]] or similar


        :return: None
        """

        self.position = position
        self.orientation = orientation

        return
    


# ================================
# Main Method for testing purposes
# ================================

if __name__ == '__main__':

    object_radius = 10.0
    object_location = np.array([4,-3,100])

    cam = Simulated_Camera(10,np.array([300,300]),np.array([10,10]))

    mask = cam.get_image(object_location, object_radius)

    cv.imshow('Mask', mask)
    cv.waitKey(0)


    print(mask)

