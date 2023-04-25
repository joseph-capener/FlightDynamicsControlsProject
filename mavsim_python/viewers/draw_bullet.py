"""
mavsim_python: drawing tools
"""
import numpy as np
import pyqtgraph.opengl as gl
from tools.rotations import Euler2Rotation


class DrawBullet:
    def __init__(self, state, window, color=0):
        """
        Draw the bullet.

        The input to this function is a (message) class with properties that define the state.
        The following properties are assumed:
            state.north  # north position
            state.east  # east position
            state.altitude   # altitude
            state.phi  # roll angle
            state.theta  # pitch angle
            state.psi  # yaw angle
        """
        
        self.color = color # 0 for blue, 1 for red
        self.timer = 0.
        self.isDead = False
        # get points that define the non-rotated, non-translated bullet and the mesh colors
        self.bullet_points, self.bullet_meshColors = self.get_points()

        self.bullet_position = np.array([[state.north], [state.east], [-state.altitude]])  # NED coordinates
        # attitude of bullet as a rotation matrix R from body to inertial
        self.R = Euler2Rotation(state.phi, state.theta-state.alpha, state.psi)
        self.Vb = self.R @ np.array([[300.],[0.],[0.]])

        # rotate and translate points defining bullet
        self.rotated_points = self.rotate_points(self.bullet_points, self.R)
        self.translated_points = self.translate_points(self.rotated_points, self.bullet_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.translated_points = R @ self.translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(self.translated_points)
        self.bullet_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.bullet_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        #self.bullet_body.setGLOptions('translucent')
        # ============= options include
        # opaque        Enables depth testing and disables blending
        # translucent   Enables depth testing and blending
        #               Elements must be drawn sorted back-to-front for
        #               translucency to work correctly.
        # additive      Disables depth testing, enables blending.
        #               Colors are added together, so sorting is not required.
        # ============= ======================================================
        window.addItem(self.bullet_body)  # add body to plot
        # default_window_size = (500, 500)
        # window.resize(*default_window_size)


    def update(self, time_step):
        self.timer += time_step
        if self.timer > 3:
            self.isDead = True
        self.bullet_position = np.array([[self.bullet_position.item(0)], [self.bullet_position.item(1)], [self.bullet_position.item(2)]]) + self.Vb*time_step  # NED coordinates
        # rotate and translate points defining bullet
        translated_points = self.translate_points(self.rotated_points, self.bullet_position)
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        self.translated_points = (R @ translated_points)
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.points_to_mesh(self.translated_points)
        # draw bullet by resetting mesh using rotated and translated points
        self.bullet_body.setMeshData(vertexes=mesh, vertexColors=self.bullet_meshColors)

    def rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def get_points(self):
        """"
            Points that define the bullet, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define bullet body parameters
        unit_length = 1

        # points are in NED coordinates
        #   define the points on the aircraft following diagram Fig 2.14
        # TODO Fix geometry of bullets (see also meshS)
        points = np.array([[0, 0, 0],  # point 1 [0]
                           [unit_length, unit_length/2, unit_length/4],  # point 2 [1]
                           [unit_length, 0, 0],  # point 3 [2]
                           [unit_length, 0, unit_length/2],  # point 4 [3]
                           [unit_length/2, unit_length/2, 0],  # point 5 [4]
                           [0, 0, unit_length/4],  # point 6 [5]
                           ]).T

        # scale points for better rendering
        scale = 2
        points = scale * points

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        white = np.array([1., 1., 1., 0.])
        c = blue
        if self.color == 1:
            c = red
        
        meshColors = np.empty((7, 3, 4), dtype=np.float32)
        meshColors[0] = yellow
        meshColors[1] = yellow
        meshColors[2] = yellow
        meshColors[3] = yellow
        meshColors[4] = yellow
        meshColors[5] = yellow
        meshColors[6] = yellow 
        return points, meshColors

    def points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # nose-top
                         [points[2], points[0], points[3]],  # nose-right
                         [points[3], points[2], points[4]],  # nose-bottom
                         [points[4], points[3], points[5]],  # nose-left
                         [points[5], points[3], points[0]],  # fuselage-left
                         [points[0], points[1], points[5]],
                         [points[5], points[4], points[2]],
                         ])
        return mesh
