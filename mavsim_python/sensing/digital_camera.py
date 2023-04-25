# Adam Welker   MEEN 674    Winter 2023
#
# pinhole_camera.py -- a class that implements 
# a pinhole camera (relative to the pinhole camera's frame).
import numpy as np
from sys import path
path.append('..')
from tools.rotations import *
from sensing.pinhole_camera import *
import sensing.mav_points as mav_points
import cv2 as cv

class Simulated_Camera:

    def __init__(self, focal_distance: float, resolution: np.ndarray, image_size: np.ndarray, field_of_view=None) -> None:
        """
        Initializes the Camera Simulator

        :param focal distance: Focal distance in meters of the camera
        :param resolution: The pixel resolution of the camera in the format [x_pixels, y_pixels]
        :param image_size: The size of the camera's image plane

        :return: None
        """
        
        
        self.focal_distance = focal_distance
        self.resolution = resolution
        
        if type(field_of_view) == type(None):
            self.image_size = image_size
        else:
            xangle = np.deg2rad(field_of_view.item(0))
            yangle = np.deg2rad(field_of_view.item(1))
            self.image_size = np.array([self.focal_distance * np.tan(xangle / 2), self.focal_distance * np.tan(yangle / 2)])
            # print(self.image_size)
        self.pin_hole = Pinhole_Camera(self.focal_distance)

        self.image = np.zeros((resolution.item(0), resolution.item(1)), dtype=int)
        self.position = np.zeros((3,1))
        self.orientation = np.array([0,0,0])

    
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
        
        position_cam_frame = obj_position - self.position
        
        position_cam_frame =np.array([[0., 1., 0.],
                                      [0., 0., 1.],
                                      [1., 0., 0.]]) @ R__i_to_c @ position_cam_frame

        # position_cam_frame = R__i_to_c @ obj_position
        # position_cam_frame = position_cam_frame - self.position

        x_0 = position_cam_frame.item(0)
        y_0 = position_cam_frame.item(1)
        z_0 = position_cam_frame.item(2)

        x_i = self.pin_hole.single_dimension_projection(x_0, z_0)
        y_i = self.pin_hole.single_dimension_projection(y_0, z_0)

        # print(x_i, y_i, x_0, self.image_size)
        # Check if object is behind focal

        if z_0 <= 0:
            return False
        

        # check if the object is out of the field of view of the image plane
        # (based on image size)

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
        
        position_cam_frame = obj_position - self.position
        
        position_cam_frame = R__i_to_c @ position_cam_frame
        
        

        z_0 = position_cam_frame.item(0)

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
        
        position_cam_frame = R__i_to_c @ (obj_position - self.position)

        x_0 = position_cam_frame.item(0)
        y_0 = position_cam_frame.item(1)
        z_0 = position_cam_frame.item(2)

        location_i = self.pin_hole.get_image_projection(np.array([[y_0, -z_0]]).T, x_0)

        return -1*location_i
    
    def get_image_location_rel(self, obj_position:np.ndarray) -> np.ndarray:
        """
        Given an object's position, finds its center locatioon in the image plane. Assumes the object is a sphere

        :param obj_position: the 3d relative position of the object in the 
        the from [[n],[e],[d]] or similar

        :return: The object's location in the form [xi, yi]
        """
    
        # Get the location of the object in the inertial plane
        position_cam_frame = obj_position
        x_0 = position_cam_frame.item(0)

        y_0 = position_cam_frame.item(1)
        z_0 = position_cam_frame.item(2)

        location_i = self.pin_hole.get_image_projection(np.array([[y_0, -z_0]]).T, x_0)

        return -1*location_i


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

        rad_i = -1 * self.get_image_size(obj_position, radius)

        x_step = self.image_size.item(0) / self.resolution.item(0)
        y_step = self.image_size.item(1) / self.resolution.item(1)
        
        x_location = location.item(0) / x_step
        y_location = -location.item(1) / y_step
        
        loc = [int(x_location + self.resolution.item(0)/2), int(y_location + self.resolution.item(1)/2)]

        rad = int(rad_i / x_step)
        
     
        # print(loc)
        # cv.rectangle(img=self.image, pt1=(0,0), pt2=(self.resolution.item(0), self.resolution.item(1)), color=(0, 0, 0), thickness=-1)
        if loc[0] >= 0 and loc[1] >= 0 and loc[0] < self.resolution.item(0)-1 and loc[1] < self.resolution.item(1)-1:
            # print(loc, rad)
            cv.circle(img=self.image, center=(loc[0], loc[1]), radius=rad, color=1, thickness=-1)
        
            
        # for j in range(0, len(self.image)):

        #     for i in (range(0, len(self.image[0]))):

        #         # Map the current pixel to a physical location on the image plane
        #         x_location = -self.image_size.item(0) / 2 + i*x_step
        #         y_location = self.image_size.item(1) / 2 - j*y_step

        #         # If the object is visible in that location, then light the pixel up
        #         if (x_location - location.item(0))**2 + (y_location - location.item(1))**2 <= rad_i**2:

        #             self.image[j][i] = 1

        return self.image
    
    def pixel_to_vector_dir(self, pixel: np.ndarray) -> np.ndarray:
        
        px = pixel.item(0)
        py = pixel.item(1)
        x_step = self.image_size.item(0) / self.resolution.item(0)
        y_step = self.image_size.item(1) / self.resolution.item(1)
        locx = (px - self.resolution.item(0)/2) * x_step
        locy = (py - self.resolution.item(1)/2) * y_step
        locz = self.focal_distance
        
        return np.array([locx, locy, locz])
        
    
    def draw_mav(self, target_position:np.ndarray, target_orientation:np.ndarray) -> np.ndarray:     
        """
        Given an object's position, returns an mask image with the object. Assumes object is a sphere

        :param obj_position: the 3d intertial position of the object in the 
        the from [[n,e,d]]

        :param radius: the radius of the object in meters

        :param obj_orientation: the 3d rotation of the mav in the inertial frame expressed
        as a quaternion in the form [e0, e1, e2, e3] or similar.

        :return: The mask image of the object as a numpy array
        """

        mask = np.zeros(self.resolution)
        self.image = mask

        target_rotation = Euler2Rotation(target_orientation.item(0), target_orientation.item(1), target_orientation.item(2))
        mav_rotation = Euler2Rotation(self.orientation.item(0), self.orientation.item(1), self.orientation.item(2))

       # Load in points transformed to enemy mav's position in the inertial frame
        rotated_target = mav_points.get_transformed_points(target_position,target_rotation)

        relative_position = np.zeros_like(rotated_target)
        
       
        # Now move the rotated mav out and then rotate to body frame
        for i in range(0,len(relative_position)):

            temp_point = np.copy(np.array([rotated_target[i,:]]).T)
            
            transformed_point = mav_rotation.T @ (temp_point - self.position)

            relative_position[i] = transformed_point.T[0]

        
        if relative_position[0][0] <= 0:

            return mask

        # Setup for digital processing

        mask_points = []

        # Find the pixel location of the mav
        for i in range(0,len(relative_position)):
            
            tf_points = self.get_image_location_rel(relative_position[i])

            #print(relative_position[i])

            image_points = self._image_index_2(tf_points, self.image_size, self.resolution)

            #print(image_points)

            mask_points.append(image_points)

        render = mav_points.points_to_polygons(np.array(mask_points))

        # make the mask
        for i in range(0,len(render)):
            
            polygon = render[i]
            cv.fillConvexPoly(mask,polygon,1)

        self.image = mask

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
    

    
    def _image_index(self, point: np.ndarray, image_size: np.ndarray, image_resolution: np.ndarray) -> np.ndarray:
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
        loc_to_pix_y = image_resolution.item(1) / image_size.item(1)

        d_pix_x = int(point.item(0) *  loc_to_pix_x)
        d_pix_y = int(point.item(1) *  loc_to_pix_y)

        i = d_pix_x + axis_origin[0]
        j = -d_pix_y + axis_origin[1]

        return np.array([int(i),int(j)])
    
    
    def _image_index_2(self, location: np.ndarray, image_size: np.ndarray, image_resolution: np.ndarray) -> np.ndarray:
        """
        Takes in the location of a point in the image frame and outputs the image 
        index 

        :param location: point location in format [[xi], [yi]] or similar
        :param image_size: image size in units of length in form [[size_x], [size_y]] or similar

        :return: mask location of point in form [i,j] or simlar
        """

        x_step = self.image_size.item(0) / self.resolution.item(0)
        y_step = self.image_size.item(1) / self.resolution.item(1)
        
        x_location = location.item(0) / x_step
        y_location = -location.item(1) / y_step
        
        loc = [int(x_location + self.resolution.item(0)/2), int(y_location + self.resolution.item(1)/2)]

        return loc
    


# ================================
# Main Method for testing purposes
# ================================

if __name__ == '__main__':

    object_location = np.array([[574.93725],
                             [1.90394],
                             [-108.42]])
    
    obj_orientation = np.array([0.00094624, -0.0056153,0.070056])

    cam = Simulated_Camera(1.0,np.array([600,600]),np.array([3,3]))

    cam_rot = np.array([0.00167033, -0.446759, -0.0018345])

    cam.set_pose(np.array([[156.3434, -10.59820, -110.8446]]).T, cam_rot)

    print(cam.image_size)

    mask = cam.draw_mav(object_location, obj_orientation)

    cv.imshow('Mask', mask)
    cv.waitKey(0)

