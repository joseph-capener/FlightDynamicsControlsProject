"""
mavsim_python: mav viewer (for chapter 2)
    - Beard & McLain, PUP, 2012
    - Update history:
        1/15/2019 - RWB
        4/15/2019 - BGM
        3/31/2020 - RWB
"""
import sys
sys.path.append("..")
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector
from viewers.draw_mav import DrawMav
from viewers.draw_path import DrawPath
from viewers.draw_bullet import DrawBullet
from viewers.draw_waypoints import DrawWaypoints

from qtpy import QtCore, QtGui, QtWidgets

class MavViewer():
    def __init__(self, app, aircraft = 1):
        self.scale = 100
        # initialize Qt gui application and window
        self.app = app  # initialize QT, external so that only one QT process is running
        self.window = gl.GLViewWidget()  # initialize the view object
        self.window.setWindowTitle('MAV Viewer')
        grid = gl.GLGridItem() # make a grid to represent the ground
        grid.scale(20, 20, 20) # set the size of the grid (distance between each line)
        self.window.addItem(grid) # add grid to viewer
        self.window.setCameraPosition(distance=200) # distance from center of plot to camera
        self.window.setBackgroundColor('k')  # set background color to black
        self.window.setGeometry(0, 0, 750, 750)  # args: upper_left_x, upper_right_y, width, height
        # center = self.window.cameraPosition()
        # center.setX(250)
        # center.setY(250)
        # center.setZ(0)
        # self.window.setCameraPosition(pos=center, distance=self.scale, elevation=50, azimuth=-90)
        self.window.noRepeatKeys.append(QtCore.Qt.Key.Key_Space)
        self.window.noRepeatKeys.append(QtCore.Qt.Key.Key_Backslash)
        self.window.follow_key = QtCore.Qt.Key.Key_Space
        self.window.fire_bullet = QtCore.Qt.Key.Key_Backslash
        self.window.show()  # display configured window
        # self.window.raise_() # bring window to the front

        self.plot_initialized = [False] * aircraft # has the mav been plotted yet?
        self.mav_plot = []
        self.numBullets = 20
        self.bullet_plot = [None] * 20
        self.bullet_initialized = [False] * self.numBullets
        self.curr_bullet_id = 0
        self.path_plot = []
        self.waypoint_plot = []
        self.follow_key_pressed = False
        self.bullet_key_pressed = False
        self.follow_id = 0
        self.timer = 0
        self.fire_flag = False

    def update(self, state, path, waypoints, id):
        blue = np.array([[30, 144, 255, 255]])/255.
        red = np.array([[1., 0., 0., 1]])
        # initialize the drawing the first time update() is called
        if not self.plot_initialized[id]:
            self.mav_plot.append(DrawMav(state, self.window, id))
            self.waypoint_plot.append(DrawWaypoints(waypoints, path.orbit_radius, blue, self.window))
            self.path_plot.append(DrawPath(path, red, self.window))
            self.plot_initialized[id] = True
        # else update drawing on all other calls to update()
        else:
            self.mav_plot[id].update(state)
            #self.bullet_plot[id].update(state) # Modify to update not based on aircraft state but set velocity
            if waypoints.flag_waypoints_changed:
                self.waypoint_plot[id].update(waypoints)
                waypoints.flag_waypoints_changed = False
            if not path.plot_updated:  # only plot path when it changes
                self.path_plot[id].update(path, red)
                path.plot_updated = True
        # update the center of the camera view to the mav location
        if id == self.follow_id:
            view_location = Vector(state.east, state.north, state.altitude)  # defined in ENU coordinates
            self.window.opts['center'] = view_location
        # print(self.window.keysPressed)
        if self.window.follow_key in self.window.keysPressed and self.follow_key_pressed == False:
            # print(self.follow_id)
            self.follow_id += 1
            self.follow_key_pressed = True
            if self.follow_id >= len(self.mav_plot):
                self.follow_id = 0
        if self.window.follow_key not in self.window.keysPressed:
            self.follow_key_pressed = False
            
    def update_bullet(self, state, time_step, bullet_id, isFiring):

        if self.timer < 2500:
            self.timer += 1
        else:
            self.timer = 0
            self.fire_flag = True

        # Auto Fire
        if isFiring and self.fire_flag and self.curr_bullet_id < self.numBullets:
            #print(self.curr_bullet_id)
            self.bullet_plot[self.curr_bullet_id] = DrawBullet(state,self.window)
            self.bullet_initialized[self.curr_bullet_id] = True
            self.curr_bullet_id += 1
            print('Bullets:',self.numBullets-self.curr_bullet_id)
            self.fire_flag = False


        # Manual Fire
        if self.window.fire_bullet in self.window.keysPressed and self.bullet_key_pressed == False and self.curr_bullet_id < self.numBullets:
            self.bullet_plot[self.curr_bullet_id] = DrawBullet(state,self.window)
            self.bullet_key_pressed = True
            self.bullet_initialized[self.curr_bullet_id] = True
            self.curr_bullet_id += 1
            print('Bullets:',self.numBullets-self.curr_bullet_id)
        if self.window.fire_bullet not in self.window.keysPressed:
            self.bullet_key_pressed = False

        #print(self.bullet_initialized)
        if self.bullet_initialized[bullet_id] == True:
            #print('works')
            if self.bullet_plot[bullet_id].isDead:
                self.bullet_initialized[bullet_id] = False
            else:
                self.bullet_plot[bullet_id].update(time_step)
            
                #self.bullet_initialized[bullet_id] = False
        
        # redraw
    
    def process_app(self):
        self.app.processEvents()

    def clear_viewer(self):
        self.window.clear()

