


import numpy as np
from message_types.msg_intruder import MsgIntruder
from message_types.msg_autopilot import MsgAutopilot
from tools.rotations import *

class AutopilotSD:
    def __init__(self, id: int, mavs: list) -> None:
        # List of mav players in the world
        self.mav_list   = mavs
        # My mav's id
        self.id         = id
        # id of the enemy mav that we will hunt
        self.targetId   = None
        # Boolean Value to determine if the enemy is within view.
        self.canSeeTarget = False
        
    def update(self, radar_states_list: list[MsgIntruder]):
        # distances to each mav
        distances =  []
        myState   = self.mav_list[id].true_state
        
        # get the distance to each other target based on radar data
        for i in range(len(self.mav_list)):
            # temp variable for calculation
            temp_distance  = np.inf # our own mav is given a distance of infinity 
                                    # so that it doesn't pick itself as target
            
            # if the mav is not ourselves
            if not i == self.id:
                # get their position
                other_pos = np.array([[radar_states_list[i].north, 
                                       radar_states_list[i].east, 
                                       -radar_states_list[i].altitude]]).T
                # get our position
                my_pos    = np.array([[myState.north, 
                                       myState.east, 
                                       -myState.altitude]]).T
                # calculate our distance   
                temp_distance  = np.linalg.norm(other_pos - my_pos)

            # add distances to the list
            distances.append(temp_distance)
        
        # set the target to be the closest mav
        self.targetId = np.argmin(np.array(distances))
        
        cmd = self.get_autopilot_CMD(radar_states_list)

    def get_autopilot_CMD(self, radar_states_list: list[MsgIntruder]):
        # Get our state
        myState   = self.mav_list[id].true_state
        
        # Get our position
        pos0    = np.array([[myState.north, 
                             myState.east, 
                            -myState.altitude]]).T
        
        # get our velocity vector
        vel0 = myState.true_state.Va * Euler2Rotation(myState.true_state.phi, 
                                                      myState.true_state.theta, 
                                                      myState.true_state.psi) @ np.array([[1., 0., 0.]]).T
        
        pos1 = np.array([[radar_states_list[self.targetId].radar_n, 
                          radar_states_list[self.targetId].radar_e, 
                          -radar_states_list[self.targetId].radar_h]]).T
        
        vel1 = radar_states_list[self.targetId].radar_Vg * Euler2Rotation(0., 
                                                                          radar_states_list[self.targetId].radar_flight_path, 
                                                                          radar_states_list[self.targetId].radar_course) @ np.array([[1., 0., 0.]]).T
        
        R = (pos1 - pos0) # relative pos
        V = vel1 - vel0
        
        Relative_Pos = Euler2Rotation(myState.phi, 
                                      myState.theta, 
                                      myState.psi).T @ R
        
        
        
        
        # target = pos1 + vel1 / np.linalg.norm(vel1) * 5.0
        # R_t = target - pos0
        # target_angle = 40
        
        # ang0 = np.arctan2(R[1] , R[0]) + np.sign(np.arctan2(R[1] , R[0])) * np.deg2rad(-target_angle)
        # # ang0 = np.arctan2(R_t[1] , R_t[0]) - np.sign(np.arctan2(R_t[1] , R_t[0])) * np.deg2rad(target_angle) # PROJECTILE EXTRAPOLATION
        # # ang0 = np.arctan2(R_t[1] , R_t[0]) # This is for leading the enemy
        
        # commands.course_command = ang0[0]
        # commands.airspeed_command = mav[1].true_state.Va + (target_angle - np.abs(np.rad2deg(np.arctan2(R[1] , R[0]))[0])) / target_angle # CONSTANT FLYING ANGLE
        
        # # commands.airspeed_command = mav[1].true_state.Va + 10
        
        # commands.altitude_command = mav[1].true_state.altitude
        # autopilot_commands = commands
        # print("dist=", np.linalg.norm(R), "angle = ", np.rad2deg(np.arctan2(R[1] , R[0])))
        
        
        
        # print("dist=", np.linalg.norm(R), "angle = ", np.rad2deg(np.arctan2(R[1] , R[0])))
        # print("SPEED= ",commands.airspeed_command)
        # print((target_angle - np.abs(np.rad2deg(np.arctan2(R[1] , R[0])))[0]))
        # print("CC =" ,commands.course_command)