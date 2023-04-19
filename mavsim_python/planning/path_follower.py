import numpy as np
from math import sin, cos
import sys

sys.path.append('..')
from message_types.msg_autopilot import MsgAutopilot
from tools.wrap import wrap
import parameters.aerosonde_parameters as PARAM

class PathFollower:
    def __init__(self):
        ##### TODO #####
        self.chi_inf = np.deg2rad(25.) # approach angle for large distance from straight-line path
        self.k_path = 0.25  # path gain for straight-line path following
        self.k_orbit = 4.  # path gain for orbit following
        self.gravity = PARAM.gravity
        self.autopilot_commands = MsgAutopilot()  # message sent to autopilot

    def update(self, path, state):
        if path.type == 'line':
            self._follow_straight_line(path, state)
        elif path.type == 'orbit':
            self._follow_orbit(path, state)
        return self.autopilot_commands

    def _follow_straight_line(self, path, state):
        Xq = np.arctan2(path.line_direction.item(1), path.line_direction.item(0))
        
        while Xq - state.chi < -np.pi:
            Xq = Xq + 2*np.pi
        while Xq - state.chi > np.pi:
            Xq = Xq - 2*np.pi
            
        rn = path.line_origin.item(0)
        re = path.line_origin.item(1)
        rd = path.line_origin.item(2)  
        epy = -np.sin(Xq)*(state.north - rn) + np.cos(Xq)*(state.east - re)
        
        pi = np.array([[state.north], [state.east], [-state.altitude]])
        
        
        epi = (pi - path.line_origin)
        # print(np.array([0,0,-1.]), path.line_direction.T[0])
        n = np.cross(np.array([0.,0., -1.]) ,path.line_direction.T).T
        n /= np.linalg.norm(n)
        si = epi - (epi.T @ n).item(0) * n
        # print(si)
        sn = si.item(0)
        se = si.item(1)
        sd = si.item(2)
        
        q = path.line_direction
        q /= np.linalg.norm(q)
        qn = q.item(0)
        qe = q.item(1)
        qd = q.item(2)
        
        
        
        ##### TODO #####
        #airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed
        
        # course command
        self.autopilot_commands.course_command =  Xq - self.chi_inf * 2/np.pi * np.arctan(self.k_path * epy)

        
        # altitude command
        
        self.autopilot_commands.altitude_command = -rd - (np.sqrt(sn**2 + se**2) * (qd / np.sqrt(qn**2 + qe**2))) 
        # feedforward roll angle for straight line is zero
        self.autopilot_commands.phi_feedforward = 0

    def _follow_orbit(self, path, state):
        lmbda = 0
        if path.orbit_direction == "CW":
            lmbda = 1
        elif path.orbit_direction == "CCW":
            lmbda = -1
        
        
        d = np.sqrt((state.north - path.orbit_center.item(0))**2 + (state.east - path.orbit_center.item(1))**2) 
        
        phi_alt = np.arctan2(state.east - path.orbit_center.item(1), state.north - path.orbit_center.item(0))
        
        
        while phi_alt - state.chi < -np.pi:
             phi_alt += 2*np.pi
        while phi_alt - state.chi > np.pi:
             phi_alt -= 2*np.pi
             
        Xo = phi_alt + lmbda*np.pi/2 
        
        ##### TODO #####
        # airspeed command
        self.autopilot_commands.airspeed_command = path.airspeed

        # course command
        self.autopilot_commands.course_command = Xo + lmbda*np.arctan(self.k_orbit * (d - path.orbit_radius)/path.orbit_radius)

        
        # print(self.autopilot_commands.course_command)

        # altitude command
        self.autopilot_commands.altitude_command = -path.orbit_center.item(2)
        
        # roll feedforward command
        self.autopilot_commands.phi_feedforward = lmbda * np.arctan((state.Vg**2) / (self.gravity * path.orbit_radius * np.cos(state.chi - state.psi)))




