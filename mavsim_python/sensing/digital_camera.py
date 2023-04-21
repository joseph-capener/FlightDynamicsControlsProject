# Adam Welker   MEEN 674    Winter 2023
#
# pinhole_camera.py -- a class that implements 
# a pinhole camera (relative to the pinhole camera's frame).
import numpy as np
from sys import path
path.append('..')
from tools.rotations import *
from pinhole_camera import *
import mav_points
import cv2 as cv

from mav_points import *

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

        position_cam_frame = position_cam_frame - self.position

        x_0 = position_cam_frame.item(0)
        y_0 = position_cam_frame.item(1)
        z_0 = position_cam_frame.item(2)

        # Check if object is behind focal

        if z_0 <= 0:
            
            return False
        

        # check if the object is out of the field of view of the image plane
        # (based on image size)
        x_i = self.pin_hole.single_dimension_projection(x_0, z_0)
        y_i = self.pin_hole.single_dimension_projection(y_0, z_0)

        if abs(x_i) > self.image_size.item(0) or abs(y_i) > self.image_size.item(1):

            return False

        return True 

        

    def get_image_size(self, obj_position:np.ndarray, radius:float) -> float:
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
        
        position_cam_frame = position_cam_frame - self.position

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
    
    def draw_mav(self, obj_position:np.ndarray, obj_orientation:np.ndarray) -> np.ndarray:     
        """
        Given an object's position, returns an mask image with the object. Assumes object is a sphere

        :param obj_position: the 3d intertial position of the object in the 
        the from [n,e,d]

        :param radius: the radius of the object in meters

        :param obj_orientation: the 3d rotation of the mav in the inertial frame expressed
        as a quaternion in the form [e0, e1, e2, e3] or similar.

        :return: The mask image of the object as a numpy array
        """

        mask = np.zeros(self.resolution)

        rotation = Quaternion2Rotation(obj_orientation)

        drone_points = get_transformed_points(obj_position, rotation)

        drone_image_points = np.array([])

        for i in range(0, len(drone_image_points)):

            pointeroo = drone_image_points[i]

            image_point = 0 


        return mask



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
    

    def _make_render_image(surfaces: np.ndarray) -> np.ndarray:
        """
        Function that given the location of surfaces making up the uav 
        makes a full render of the uav in the image plane, making up of all surfaces that 
        touch the image plane

        :param surfaces: An ndarry containing smaller arrays which are each of the surfaces
        which make up the plane. For order see return of mav_points.points_to_polygon 

        :return: ndarray mask containing the rendered surfaces       
        """

        pass

    def _reduce_render_image(render_image: np.ndarray) -> np.ndarray:
        """
        Function that given the location of surfaces making up the uav 
        makes a full render of the uav in the image plane, making up of all surfaces that 
        touch the image plane

        :param surfaces: An ndarry containing smaller arrays which are each of the surfaces
        which make up the plane. For order see return of mav_points.points_to_polygon 

        :return: ndarray mask containing the rendered surfaces       
        """

        pass

    
    def _image_index(point: np.ndarray, image_size: np.ndarray, image_resolution: np.ndarray) -> np.ndarray:
        """
        Takes in the location of a point in the image frame and outputs the image 
        index 

        :param point: point location in format [[xi], [yi]] or similar
        :param image_size: image size in units of length in form [[size_x], [size_y]] or similar

        :return: mask location of point in form [i,j] or simlar
        """

        axis_origin = image_resolution / 2.


        for i in range(0, len(axis_origin)): 
            
            axis_origin[i] = int(axis_origin[i])

        # conversion factor of displacement in length to pixels
        loc_to_pix_x = image_resolution.item(0) / image_size.item(0)   
        loc_to_pix_y = image_resolution.item(0) / image_size.item(0)

        d_pix_x = int(point.item(0) *  loc_to_pix_x)
        d_pix_y = int(point.item(0) *  loc_to_pix_y)

        i = d_pix_x + axis_origin[0]
        j = -d_pix_y + axis_origin[1]

        return np.array([i,j])

    


# ================================
# Main Method for testing purposes
# ================================

if __name__ == '__main__':

    object_radius = 10.0
    object_location = np.array([4,-30,100])

    cam = Simulated_Camera(10,np.array([300,300]),np.array([10,10]))

    mask = cam.get_image(object_location, object_radius)

    cv.imshow('Mask', mask)
    cv.waitKey(0)


    print(mask)

