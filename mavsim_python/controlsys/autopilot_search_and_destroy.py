import numpy as np
from sys import path
path.append('..')
from message_types.msg_intruder import MsgIntruder
from message_types.msg_autopilot import MsgAutopilot
from tools.rotations import *
from scipy.optimize._minimize import minimize
from sensing.digital_camera import Simulated_Camera
from sensing.cam_sim_viewer import CamSimViewer
from sensing.mav_mask_viewer import MAVMaskViewer
from sensing.image_interpreter import ImageInterpreter
import parameters.simulation_parameters as SIM

import cv2 as cv
def projected_pos(pos: np.ndarray , vel: np.ndarray, t: float) -> np.ndarray:
        return pos + vel * t
def leading_objective(t, pb, pt, vt, spd_b):
    vb = (projected_pos(pt, vt, t) - pb) / np.linalg.norm(projected_pos(pt, vt, t) - pb) * spd_b
    return np.linalg.norm(projected_pos(pb, vb, t) - projected_pos(pt, vt, t))
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
        
        self.cam = Simulated_Camera(1 ,np.array([900,900]), np.array([10,10]), np.array([60.,60.]))
        self.mav_cam = Simulated_Camera(1 ,np.array([900,900]), np.array([10,10]), np.array([60.,60.]))
        self.camView = CamSimViewer(self.cam)
        self.mavView = MAVMaskViewer(self.mav_cam)

        self.interpreter = ImageInterpreter(self.mav_cam)
        
        self.updateTimers = 0. # camera update timer
        self.trackTimer = 0    # timer that regulates how long we need to track the other mav before we start trying to shoot.
        self.fireTimer = 0
        self.isFiring = False
        
        self.onScreenFlag = False
        self.tracking_SM = 0
        
        self.target_azimuth = 0.
        self.target_elevation = 0.
        

        
    def update(self, radar_states_list: list[MsgIntruder]):
        # distances to each mav
        distances =  []
        
        myState   = self.mav_list[self.id].true_state
        self.cam.set_pose(np.array([[myState.north, myState.east, -myState.altitude]]).T, 
                          np.array([[myState.phi, myState.theta, myState.psi,]]).T)
        
        self.mav_cam.set_pose(np.array([[myState.north, myState.east, -myState.altitude]]).T, 
                          np.array([[myState.phi, myState.theta, myState.psi,]]).T)
        
        # get the distance to each other target based on radar data
        for i in range(len(radar_states_list)):
            # temp variable for calculation
            temp_distance  = np.inf # our own mav is given a distance of infinity 
                                    # so that it doesn't pick itself as target
            
            # if the mav is not ourselves
            if not i == self.id:
                # get their position
                other_pos = np.array([[radar_states_list[i].radar_n, 
                                       radar_states_list[i].radar_e, 
                                       -radar_states_list[i].radar_h]]).T
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
        
        return cmd

    def get_autopilot_CMD(self, radar_states_list: list[MsgIntruder]):
        # initialize flight command
        commands = MsgAutopilot()
        # Get our state
        myState   = self.mav_list[self.id].true_state
        # Get our position
        pos0    = np.array([[myState.north, 
                             myState.east, 
                            -myState.altitude]]).T
        # get our velocity vector
        vel0 = myState.Va * Euler2Rotation(myState.phi, 
                                                      myState.theta, 
                                                      myState.psi) @ np.array([[1., 0., 0.]]).T
        # get other mav position
        pos1 = np.array([[radar_states_list[self.targetId].radar_n, 
                          radar_states_list[self.targetId].radar_e, 
                          -radar_states_list[self.targetId].radar_h]]).T
        # get other mav velocity
        vel1 = radar_states_list[self.targetId].radar_Vg * Euler2Rotation(0., 
                                                                          radar_states_list[self.targetId].radar_flight_path, 
                                                                          radar_states_list[self.targetId].radar_course) @ np.array([[1., 0., 0.]]).T
        
        R = pos1 - pos0 # relative pos between us and the other mav (inertial frame)
        V = vel1 - vel0 # relative velocity between us and the other mav (inertial frame)
        
        # get body frame relative position
        Relative_Pos = Euler2Rotation(myState.phi, 
                                      myState.theta, 
                                      myState.psi).T @ R
        
        # get the relative angle (body frame)
        Relative_Angle = np.arctan2(Relative_Pos[1], Relative_Pos[0])[0]
        while Relative_Angle > 2 * np.pi:
            Relative_Angle -= 2*np.pi
        while Relative_Angle < -2 * np.pi:
            Relative_Angle += 2*np.pi
        
        # get the relative angle between us and them (inertial frame)
        Inertial_Course = np.arctan2(R[1], R[0])[0]
        while Inertial_Course > 2 * np.pi:
            Inertial_Course -= 2*np.pi
        while Inertial_Course < -2 * np.pi:
            Inertial_Course += 2*np.pi
        
        
        

        # ----CAMERA UPDATE----
        self.updateTimers += SIM.ts_simulation
        if self.updateTimers > 0.1:
            # reset timer

            mav_state = np.array([[self.mav_list[0].true_state.north],
                                [self.mav_list[0].true_state.east],
                                [-self.mav_list[0].true_state.altitude]])
            
            target_state = np.array([[self.mav_list[1].true_state.north, 
                                      self.mav_list[1].true_state.east, 
                                      -self.mav_list[1].true_state.altitude]])

            t_phi = self.mav_list[1].true_state.phi
            t_theta = self.mav_list[1].true_state.theta
            t_psi = self.mav_list[1].true_state.psi

            m_phi = self.mav_list[0].true_state.phi
            m_theta = self.mav_list[0].true_state.theta
            m_psi = self.mav_list[0].true_state.psi

            #self.mavView.update(mav_state, np.array([[m_phi, m_theta, m_psi]]), 
                                # target_state, np.array([t_phi, t_theta, t_psi]))
            

            self.updateTimers = 0.
            # get the gray img
            gray_img = self.mav_cam.draw_mav(target_state, np.array([t_phi, t_theta, t_psi]))
            # convert this to color
            display_img = cv.merge([gray_img,gray_img,gray_img])
            # mask out what's onscreen
            ret, mask = cv.threshold(self.mav_cam.image, 127, 255, 0)
            # get the centroid of the mask
            centroid = self.interpreter.get_image_centroid(self.mav_cam.image)

            # print(centroid)
            # if the centroid is present onscreen
            if centroid[0] >= 0 and centroid[1] >= 0:
                # draw a red circle at it's location
                self.onScreenFlag = True
                cv.circle(img=display_img, center=centroid, radius=10, color=(0,0,255), thickness=2)
                # print debug data
                # print(self.cam.pixel_to_vector_dir(centroid), centroid)
                self.vector_centroid = self.cam.pixel_to_vector_dir(centroid)
                self.target_azimuth   = np.arctan(self.vector_centroid[0] / self.vector_centroid[2])
                self.target_elevation = np.arctan(self.vector_centroid[1] / self.vector_centroid[2])
                textstr = f'az={np.round(np.rad2deg(self.target_azimuth),2)}, el={np.round(np.rad2deg(self.target_elevation),2)}'
                cv.putText(display_img, textstr, (0, 80), cv.FONT_HERSHEY_PLAIN, 3, (0,0,255), 1)
                # print("az = ", np.rad2deg(self.target_azimuth) ,"el = ", np.rad2deg(self.target_elevation))
            else:
                self.onScreenFlag = False
            # update the camera
            self.camView.update_img(display_img)
        
        
        
        
        ## State Machine for Tracking
        if self.tracking_SM == 0:
                commands.course_command   = Inertial_Course
                commands.airspeed_command = 28.
                commands.altitude_command = -pos1.item(2)
                
                if self.onScreenFlag:
                    self.tracking_SM = 1
                
        elif self.tracking_SM == 1:
                
                Rotation_cam_inertial = Euler2Rotation(myState.phi, myState.theta, myState.psi)
                inertial_centroid = Rotation_cam_inertial @ np.array([[0., 1., 0.],
                                                                        [0., 0., 1.],
                                                                        [1., 0., 0.]]).T @ self.vector_centroid
                
                inertial_centroid /= np.linalg.norm(inertial_centroid)
                
                course_from_cam = np.arctan2(inertial_centroid[1], inertial_centroid[0])
                while course_from_cam  > 2 * np.pi:
                    course_from_cam   -= 2 * np.pi
                while course_from_cam < -2 * np.pi:
                    course_from_cam   += 2 * np.pi
                    
                altitude_angle_from_cam = -np.arctan(inertial_centroid[2]/np.linalg.norm(inertial_centroid[0:2]))
                
                
                predicted_height = myState.altitude + np.tan(altitude_angle_from_cam) * np.linalg.norm(R[0:2])
                
                # print("diff, ", np.rad2deg(myState.theta - self.target_elevation))
                
                # print("height=", )
                #print(myState.theta - self.target_elevation , np.abs(np.rad2deg(myState.chi - course_from_cam)))
                if np.rad2deg(myState.theta - self.target_elevation) < 1 and np.abs(np.rad2deg(myState.chi - course_from_cam)) < 1.:
                    self.trackTimer += SIM.ts_simulation
                else:
                    self.trackTimer = 0
                    
                if self.trackTimer > 5.0:
                    self.tracking_SM = 2
                    pass
                
                commands.course_command   = course_from_cam 
                commands.airspeed_command = 28.
                # commands.altitude_command = myState.altitude + (myState.alpha - self.target_elevation)*10
                commands.altitude_command = myState.altitude + (predicted_height - myState.altitude)*SIM.ts_simulation  # essentially an alpha filter    
                
        elif self.tracking_SM == 2:
            
            time = np.array([0.])
            res = minimize(fun=leading_objective, x0=time,args=(pos0,pos1,vel1, 300.))
            leading_t = res.x[0]
            new_target = projected_pos(pos1, vel1, leading_t)
            # print("pos= ", pos1, "\n vel=", vel1, "\n t=",leading_t)
            # print(new_target)
            # print(leading_t, res.fun)
            newCourse = np.arctan2(new_target[1,0] - myState.east, new_target[0,0]- myState.north)
            commands.course_command   = newCourse
            commands.airspeed_command = 28.
            commands.altitude_command = -new_target[2,0]
            
            self.fireTimer += SIM.ts_simulation
            
            if self.isFiring == True:
                self.isFiring = False
            
            if self.fireTimer > 0.5 and np.abs(np.rad2deg(myState.chi - newCourse)) < 1.:
                self.fireTimer = 0
                #print("SHOOT")
                self.isFiring = True
            
        return commands
    
    
    
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