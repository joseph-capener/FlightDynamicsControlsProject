"""
mavsim_python: drawing tools
    - Beard & McLain, PUP, 2012
    - Update history:
        4/15/2019 - RWB
        3/30/2022 - RWB
"""

import numpy as np
import sys
sys.path.append('..')
from planning.dubins_parameters import DubinsParameters
from message_types.msg_path import MsgPath


class PathManager:
    def __init__(self):
        # message sent to path follower
        self.path = MsgPath()
        # pointers to previous, current, and next waypoints
        self.ptr_previous = 0
        self.ptr_current = 1
        self.ptr_next = 2
        self.num_waypoints = 0
        self.halfspace_n = np.inf * np.ones((3,1))
        self.halfspace_r = np.inf * np.ones((3,1))
        # state of the manager state machine
        self.manager_state = 1
        self.manager_requests_waypoints = True
        self.dubins_path = DubinsParameters()

    def update(self, waypoints, radius, state):
        if waypoints.num_waypoints == 0:
            self.manager_requests_waypoints = True
        if self.manager_requests_waypoints is True \
                and waypoints.flag_waypoints_changed is True:
            self.manager_requests_waypoints = False
        if waypoints.type == 'straight_line':
            self.line_manager(waypoints, state)
        elif waypoints.type == 'fillet':
            self.fillet_manager(waypoints, radius, state)
        elif waypoints.type == 'dubins':
            self.dubins_manager(waypoints, radius, state)
        else:
            print('Error in Path Manager: Undefined waypoint type.')
        return self.path

    def line_manager(self, waypoints, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        self.num_waypoints = waypoints.num_waypoints
        self.construct_line(waypoints)
        
        if waypoints.flag_waypoints_changed is True and self.num_waypoints >= 3:
            self.initialize_pointers()
        
        if self.inHalfSpace(mav_pos) is True:
            self.increment_pointers()
            waypoints.plot_updated = False
            
        self.path.plot_updated = False
        
        ##### TODO ######
        # Use functions - self.initialize_pointers(), self.construct_line()
        # self.inHalfSpace(mav_pos), self.increment_pointers(), self.construct_line()

        # Use variables - self.ptr_current, self.manager_requests_waypoints,
        # waypoints.__, radius
        

    def fillet_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer

        # print(waypoints.ned)

        ##### TODO ######
        # Use functions - self.initialize_pointers(), self.construct_fillet_line(),
        # self.inHalfSpace(), self.construct_fillet_circle(), self.increment_pointers()

        # Use variables self.num_waypoints, self.manager_state, self.ptr_current
        # self.manager_requests_waypoints, waypoints.__, radius
        self.num_waypoints = waypoints.num_waypoints
        if waypoints.flag_waypoints_changed is True and self.num_waypoints >= 3:
            waypoints.flag_waypoints_changed = False
            self.initialize_pointers()
        
        
        
        if self.inHalfSpace(mav_pos) is True:
            self.manager_state    += 1
            waypoints.plot_updated = False
            self.path.plot_updated = False
             
            if self.manager_state  > 2:
                
                self.increment_pointers()
                self.manager_state = 1  
        
        if self.manager_state == 1:
            self.construct_fillet_line(waypoints, radius)
            # print("1")
        if self.manager_state == 2:
            self.construct_fillet_circle(waypoints, radius)
            # print("2")
            

    def dubins_manager(self, waypoints, radius, state):
        mav_pos = np.array([[state.north, state.east, -state.altitude]]).T
        # if the waypoints have changed, update the waypoint pointer
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        # next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        previous_chi = waypoints.course[self.ptr_previous:self.ptr_previous+1][0]
        current_chi  = waypoints.course[self.ptr_current:self.ptr_current+1][0]
        # next_chi     = waypoints.course[self.ptr_next:self.ptr_next+1][0]
        self.num_waypoints = waypoints.num_waypoints
        if waypoints.flag_waypoints_changed is True and self.num_waypoints >= 3:
            waypoints.flag_waypoints_changed = False
            self.initialize_pointers()

        self.dubins_path.update(previous, previous_chi, current, current_chi, radius)
        
        
        if self.manager_state == 1:
            self.construct_dubins_circle_start(waypoints, self.dubins_path)
            if self.inHalfSpace(mav_pos) is False:
                self.manager_state += 1

        elif self.manager_state == 2:
            self.construct_dubins_circle_start(waypoints, self.dubins_path)
            if self.inHalfSpace(mav_pos) is True:
                self.manager_state += 1

        elif self.manager_state == 3:
            self.construct_dubins_line(waypoints, self.dubins_path)
            if self.inHalfSpace(mav_pos) is True:
                self.manager_state += 1

        elif self.manager_state == 4:
            self.construct_dubins_circle_end(waypoints, self.dubins_path)
            if self.inHalfSpace(mav_pos) is False:
                self.manager_state += 1

        elif self.manager_state == 5:
            self.construct_dubins_circle_end(waypoints, self.dubins_path)
            if self.inHalfSpace(mav_pos) is True:
                self.manager_state = 1

                self.increment_pointers()
                # print(mav_pos)
            
        
        ##### TODO #####
        # Use functions - self.initialize_pointers(), self.dubins_path.update(),
        # self.construct_dubins_circle_start(), self.construct_dubins_line(),
        # self.inHalfSpace(), self.construct_dubins_circle_end(), self.increment_pointers(),

        # Use variables - self.num_waypoints, self.dubins_path, self.ptr_current,
        # self.ptr_previous, self.manager_state, self.manager_requests_waypoints,
        # waypoints.__, radius

        
        
    def initialize_pointers(self):
        if self.num_waypoints >= 3:
            ##### TODO #####
            self.ptr_previous = 0
            self.ptr_current = 1
            self.ptr_next = 2
        else:
            print('Error Path Manager: need at least three waypoints')

    def increment_pointers(self):
        ##### TODO #####

        # print(self.ptr_previous, self.ptr_current, self.ptr_next)
        
        self.ptr_previous += 1
        self.ptr_current  += 1
        self.ptr_next     += 1
        if self.ptr_previous >= self.num_waypoints:
            self.ptr_previous = 0
        if self.ptr_current >= self.num_waypoints:
            self.ptr_current = 0
        if self.ptr_next >= self.num_waypoints:
            self.ptr_next = 0
        
    def construct_line(self, waypoints):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        ##### TODO #####
        current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
       
        # update halfspace variables
        qi_neg1 = (current - previous) / np.linalg.norm(current - previous)
        qi      = (next    - current)  / np.linalg.norm(next    - current)
        ni      = (qi + qi_neg1)       / np.linalg.norm(qi + qi_neg1)  
        
        self.halfspace_n = ni 
        self.halfspace_r = current
        
        # Update path variables
        # self.path.__ =
        self.path.line_direction = current - previous
        self.path.line_origin    = previous
        self.path.type           = 'line'
        

    def construct_fillet_line(self, waypoints, radius):
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        ##### TODO #####
        current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]

        qi_neg1 = (current - previous) / np.linalg.norm(current - previous)
        qi      = (next    - current)  / np.linalg.norm(next    - current)
        
        angle = np.arccos(-qi_neg1.T @ qi)
        z = current - (radius / np.tan(angle/2)) * qi_neg1
        p = previous
        
        # update halfspace variables
        self.halfspace_n = qi_neg1 
        self.halfspace_r = z
        
        # Update path variables
        # self.path.__ =
        self.path.line_direction = z - previous
        self.path.line_origin = previous
        self.path.type = 'line'
        

    def construct_fillet_circle(self, waypoints, radius):

        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        ##### TODO #####
        current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]

        qi_neg1 = (current - previous) / np.linalg.norm(current - previous)
        qi      = (next    - current)  / np.linalg.norm(next    - current)
        
        angle = np.arccos(-qi_neg1.T @ qi)
        z = current + (radius / np.tan(angle/2)) * qi
        p = current
        sign = np.sign(next[0,0] * current[1,0] - next[1,0] * current[0,0]) #previous[0] * current[1] - previous[1] * current[0]

        # update halfspace variables
        self.halfspace_n = qi 
        self.halfspace_r = z
        
        # Update path variables
        # self.path.__ =
        # print(sign)
        self.path.type = 'orbit'
        self.path.orbit_center = current - (radius / np.sin(angle/2)) * (qi_neg1 - qi) / (np.linalg.norm(qi_neg1 - qi))
        self.path.orbit_radius = radius
        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.path.orbit_direction = 'CW' if sign == -1 else 'CCW'
        self.path.plot_updated = False

    def construct_dubins_circle_start(self, waypoints, dubins_path):
        ##### TODO #####
        # previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        # current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        # next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        
        # update halfspace variables
        self.halfspace_n = dubins_path.n1
        self.halfspace_r = dubins_path.r1 
        
        # Update path variables
        # self.path.__ =
        self.path.type = 'orbit'
        self.path.orbit_center = dubins_path.center_s
        self.path.orbit_radius = dubins_path.radius
        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.path.orbit_direction = 'CW' if dubins_path.dir_s == 0 else 'CCW'
        self.path.plot_updated = False

    def construct_dubins_line(self, waypoints, dubins_path):
        ##### TODO #####
        # previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        # current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        # next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        
        # update halfspace variables
        self.halfspace_n = dubins_path.n1
        self.halfspace_r = dubins_path.r2 
        
        # Update path variables
        # self.path.__ =
        self.path.line_direction = dubins_path.r2 - dubins_path.r1
        self.path.line_origin = dubins_path.r1
        self.path.type = 'line'
        self.path.plot_updated = False

    def construct_dubins_circle_end(self, waypoints, dubins_path):
        ##### TODO #####
        previous = waypoints.ned[:, self.ptr_previous:self.ptr_previous+1]
        current  = waypoints.ned[:, self.ptr_current:self.ptr_current+1]
        next     = waypoints.ned[:, self.ptr_next:self.ptr_next+1]
        
        # update halfspace variables
        self.halfspace_n = dubins_path.n3
        self.halfspace_r = dubins_path.r3 
        
        # Update path variables
        # self.path.__ =
        self.path.type = 'orbit'
        self.path.orbit_center = dubins_path.center_e
        self.path.orbit_radius = dubins_path.radius
        # orbit direction: 'CW'==clockwise, 'CCW'==counter clockwise
        self.path.orbit_direction = 'CW' if dubins_path.dir_e == 0 else 'CCW'
        self.path.plot_updated = False

    def inHalfSpace(self, pos):
        if (pos-self.halfspace_r).T @ self.halfspace_n >= 0:
            return True
        else:
            return False

