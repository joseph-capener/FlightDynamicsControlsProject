import numpy as np
from sys import path
path.append('..')
from tools.rotations import *
import mav_points
from digital_camera import Simulated_Camera
import cv2 as cv


mav_position = np.array([[0,0,0]]).T
mav_phi = np.pi/8.
mav_theta = np.pi/8.
mav_psi = np.pi/8.

target_position = np.array([[5., 0, -5]]).T
target_phi = 0.
target_theta = np.pi/8.
target_psi = np.pi/8.

target_rotation = Euler2Rotation(target_phi,target_theta,target_psi)

mav_orientation = Euler2Rotation(mav_phi,mav_theta,mav_psi)

# Load in points transformed to enemy mav's position in the inertial frame
rotated_target = mav_points.get_transformed_points(target_position,target_rotation)

relative_position = np.zeros_like(rotated_target)

# Now move the rotated mav out and then rotate to body frame
for i in range(0,len(relative_position)):

    temp_point = np.array([rotated_target[i,:]]).T
    
    transformed_point = mav_orientation.T @ (temp_point - mav_position)

    relative_position[i] = transformed_point.T[0]


# Setup for digital processing
cam = Simulated_Camera(1.,np.array([500,500]),np.array([3,3]))

mask = np.zeros(cam.resolution)

mask_points = []

# Find the pixel location of the mav
for point in relative_position:
    
    tf_points = cam.get_image_location(point)

    image_points = cam._image_index(tf_points,cam.image_size, cam.resolution)

    mask_points.append(image_points)

render = mav_points.points_to_polygons(np.array(mask_points))


# make the mask
for i in range(0,len(render)):
    
    polygon = render[i]

    cv.fillConvexPoly(mask,polygon,1)

cv.imshow('Mask', mask)
cv.waitKey(0)




    
