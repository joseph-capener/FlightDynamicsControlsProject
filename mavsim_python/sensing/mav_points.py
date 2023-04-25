# Adam Welker       MEEN 674    Winter 23
#
# mav_points.py -- file that contains the points needed to plot the mav

import numpy as np
from sys import path
path.append('..')
from tools.rotations import *

# define MAV body parameters
unit_length = 10.
fuse_h = unit_length
fuse_w = unit_length
fuse_l1 = unit_length * 2
fuse_l2 = unit_length
fuse_l3 = unit_length * 4
wing_l = unit_length
wing_w = unit_length * 6
tail_h = unit_length
tail_l = unit_length
tail_w = unit_length * 2

# points are in NED coordinates
#   define the points on the aircraft following diagram Fig 2.14
uav_points = np.array([[fuse_l1, 0, 0],  # point 1 [0]
                    [fuse_l2, fuse_w / 2.0, -fuse_h / 2.0],  # point 2 [1]
                    [fuse_l2, -fuse_w / 2.0, -fuse_h / 2.0],  # point 3 [2]
                    [fuse_l2, -fuse_w / 2.0, fuse_h / 2.0],  # point 4 [3]
                    [fuse_l2, fuse_w / 2.0, fuse_h / 2.0],  # point 5 [4]
                    [-fuse_l3, 0, 0],  # point 6 [5]
                    [0, wing_w / 2.0, 0],  # point 7 [6]
                    [-wing_l, wing_w / 2.0, 0],  # point 8 [7]
                    [-wing_l, -wing_w / 2.0, 0],  # point 9 [8]
                    [0, -wing_w / 2.0, 0],  # point 10 [9]
                    [-fuse_l3 + tail_l, tail_w / 2.0, 0],  # point 11 [10]
                    [-fuse_l3, tail_w / 2.0, 0],  # point 12 [11]
                    [-fuse_l3, -tail_w / 2.0, 0],  # point 13 [12]
                    [-fuse_l3 + tail_l, -tail_w / 2.0, 0],  # point 14 [13]
                    [-fuse_l3 + tail_l, 0, 0],  # point 15 [14]
                    [-fuse_l3, 0, -tail_h],  # point 16 [15]
                    ])

def points_to_polygons(points:np.ndarray) -> list:
    """
    Given a list of uav points, returns the 
    surfaces of the uav
    """
    nose_top = np.array([points[0], points[1], points[2]]) # nose-top
    nose_right = np.array([points[0], points[1], points[4]])  # nose-right
    nose_bottom = np.array([points[0], points[3], points[4]])  # nose-bottom
    nose_left = np.array([points[0], points[3], points[2]])  # nose-left
    fuselage_left = np.array([points[5], points[2], points[3]])  # fuselage-left
    fuselage_top = np.array([points[5], points[1], points[2]])  # fuselage-top
    fuselage_right = np.array([points[5], points[1], points[4]])  # fuselage-right
    fuselage_bottom = np.array([points[5], points[3], points[4]])  # fuselage-bottom
    wing_1 = np.array([points[6], points[7], points[9]])  # wing 1
    wing_2 = np.array([points[7], points[8], points[9]])  # wing 2
    elevator_1 = np.array([points[10], points[11], points[12]])  # horizontal tail
    elevator_2 = np.array([points[10], points[12], points[13]])  # horizontal tail
    tail = np.array([points[5], points[14], points[15]])  # vertical tail


    return [nose_top, nose_right, nose_bottom, nose_left,\
           fuselage_left, fuselage_top, fuselage_right, fuselage_bottom,\
           wing_1, wing_2, elevator_1, elevator_2, tail]


def get_transformed_points(NED_translation: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    """
    Given a rotation matrix and translation, returns the points of the uav rotated into that frame

    :params NED_translation: the translation of the points in NED form [n, e, d].
    :params rotation_matrix: the rotation matix to rotate as np array

    :return: returns the plane points in the rotated frame
    """

    translation = np.array([NED_translation.item(0), NED_translation.item(1), NED_translation.item(2)])

    tf_points = np.copy(uav_points)

    #Translate points and rotate points
    for i in range(0, len(tf_points)):

        # rotate points

        tmp_n = tf_points[i][0]
        tmp_e = tf_points[i][1]
        tmp_d = tf_points[i][2]

        tmp_vec = np.array([[tmp_n, tmp_e, tmp_d]]).T
        tmp_vec = rotation_matrix @ tmp_vec

        tmp_point = [tmp_vec.item(0), tmp_vec.item(1), tmp_vec.item(2)]

        tf_points[i] = tmp_point

        tf_points[i] = tf_points[i] + translation # translate points


    return tf_points




if __name__ == '__main__':

        phi = np.pi / 2
        theta = np.pi / 2
        psi = 0

        rotation = Euler2Rotation(phi, theta, psi)

        tf = np.array([-1,-1,-1])

        print(get_transformed_points(tf,rotation))