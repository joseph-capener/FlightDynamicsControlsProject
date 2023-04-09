# Adam Welker   MEEN 674    Winter 2023
#
# pinhole_camera.py -- a class that implements 
# a pinhole camera (relative to the pinhole camera's frame).
import numpy as np

class Pinhole_Camera:

    def __init__(self, focal_length) -> None:
        
        self.f = focal_length

    
    def single_dimension_projection(self, x_obj: float, z_obj: float) -> float:
        """
        Given the location of an object on a 2D plane (x,z),
        will return the location of the object on the image plane

        :param x_obj: The location of the point object along the x axis
        :param z_obj: The distance from the image plane in the z_axis

        :return: The x location of the point object on the image plane
        """

        x_i = -self.f * x_obj/z_obj
        
        return x_i



    def get_image_projection(self, location: np.ndarray , distance: float) -> np.ndarray:
        """
        Given the location of an object in 3d space, will
        return the location of the object on the image plane

        :param location: A numpy array formated as [x , y], which contains
        the location of the point object a plane parellel to the image plane
        
        :param distance: the distance of the x-y plane on which the point
        object lies to the image plane 

        :return: An ndarry object of the location of the object on the image
        plane. Formatted as [xi, yi].
        """

        x_obj = location.item(0)
        y_obj = location.item(1)

        x_i = self.single_dimension_projection(x_obj, distance)
        y_i = self.single_dimension_projection(y_obj, distance)

        return np.array([[x_i, y_i]]).T
    
