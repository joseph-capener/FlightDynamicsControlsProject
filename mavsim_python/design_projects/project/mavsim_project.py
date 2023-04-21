"""
mavsim_python
    - Chapter 8 assignment for Beard & McLain, PUP, 2012
    - Last Update:
        2/21/2019 - RWB
        2/24/2020 - RWB
"""
import sys
sys.path.append('../..')
import numpy as np
import pyqtgraph as pg
import parameters.simulation_parameters as SIM
from tools.signals import Signals
from models.mav_dynamics_sensors import MavDynamics
from models.wind_simulation import WindSimulation
from controlsys.autopilot import Autopilot

from controlsys.autopilot_search_and_destroy import AutopilotSD

from estimation.observer import Observer
# from estimation.observer_full import Observer
from viewers.mav_viewer_multi import MavViewer
from viewers.mav_waypoint_viewer import MAVAndWaypointViewer
from viewers.data_viewer import DataViewer
from viewers.sensor_viewer import SensorViewer
from tools.quit_listener import QuitListener

import parameters.planner_parameters as PLAN
from planning.path_follower import PathFollower
from planning.path_manager import PathManager
from planning.path_planner import PathPlanner
from message_types.msg_waypoints import MsgWaypoints
from tools.rotations import *


waypoints = MsgWaypoints()


quitter = QuitListener()

VIDEO = False
DATA_PLOTS = False
SENSOR_PLOTS = False
ANIMATION = True
SAVE_PLOT_IMAGE = False
COMPUTE_MODEL = False
NUM_AIRCRAFT = 2 #PLANE 1 IS FOLLOWER AND PLANE 0 IS INTRUDER #TODO: Currently does not work for more than 1 follower and 1 intruder

# video initialization
if VIDEO is True:
    from viewers.video_writer import VideoWriter
    video = VideoWriter(video_name="chap8_video.avi",
                        bounding_box=(0, 0, 1000, 1000),
                        output_rate=SIM.ts_video)

#initialize the visualization
if ANIMATION or DATA_PLOTS or SENSOR_PLOTS:
    app = pg.QtWidgets.QApplication([]) # use the same main process for Qt applications
if ANIMATION:
    mav_view = MavViewer(app=app, aircraft=NUM_AIRCRAFT)  # initialize the mav viewer
if DATA_PLOTS:
    data_view = DataViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)
if SENSOR_PLOTS:
    sensor_view = SensorViewer(app=app,dt=SIM.ts_simulation, plot_period=SIM.ts_plot_refresh, 
                           data_recording_period=SIM.ts_plot_record_data, time_window_length=30)

# initialize elements of the architecture

#################
waypoints = MsgWaypoints()
#waypoints.type = 'straight_line'
waypoints.type = 'fillet'
# waypoints.type = 'dubins'
Va = PLAN.Va0
waypoints.add(np.array([[0, 0, -100]]).T, Va, np.radians(0), np.inf, 0, 0)
waypoints.add(np.array([[10000, 0, -300]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints.add(np.array([[0, -10000, -600]]).T, Va, np.radians(45), np.inf, 0, 0)
waypoints.add(np.array([[10000, -10000, -800]]).T, Va, np.radians(-135), np.inf, 0, 0)




###############


wind = [WindSimulation(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]
mav = [MavDynamics(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]

autopilotSD = AutopilotSD(0, mav)  
autopilot = [Autopilot(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]
observer = [Observer(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]   

measurements    = [None] * NUM_AIRCRAFT
estimated_state = [None] * NUM_AIRCRAFT  # estimate states from measurements
delta           = [None] * NUM_AIRCRAFT
commanded_state = [None] * NUM_AIRCRAFT
current_wind    = [None] * NUM_AIRCRAFT



 
mav[0]._state[0] = -300. 
mav[0]._state[1] = -200.
mav[0]._state[2] = -100. 
# mav[0]._state[6:10] = Euler2Quaternion(0.,0.,180)


# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
#TODO: Modify second Va/h commands
Va_command = [Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency = 0.01),
              Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency = 0.01)]
h_command = [Signals(dc_offset=100.0,
                    amplitude=0.0,
                    start_time=0.0,
                    frequency=0.02),
             Signals(dc_offset=100.0,
                    amplitude=20.0,
                    start_time=0.0,
                    frequency=0.02)]
chi_command = [Signals(dc_offset=np.radians(0.0),
                      amplitude=np.radians(45.0),
                      start_time=5.0,
                      frequency=0.015),
               Signals(dc_offset=np.radians(0.0),
                      amplitude=np.radians(45.0),
                      start_time=1.0,
                      frequency=0.015)]

autopilot = [Autopilot(SIM.ts_simulation), 
             Autopilot(SIM.ts_simulation)]

path_follower = [PathFollower(),
                 PathFollower()]

path_manager = [PathManager(),
                PathManager()]

# initialize the simulation time
sim_time = SIM.start_time
end_time = 800

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:

    # -------autopilot commands-------------
    #commands.airspeed_command = Va_command.polynomial(sim_time)
    # commands.course_command   = chi_command.polynomial(sim_time)
    #commands.altitude_command = h_command.polynomial(sim_time)

    # =============DO RADAR STUFF==========
    # if id == NUM_AIRCRAFT-1: # TODO: Currently does not work for multiple intruders/followers, must fix
    # for i in range(NUM_AIRCRAFT):
    #     mav[-1].getIntruderState(mav[i].true_state) # TODO:Replace with estimated states (radar or optic)
    radar_states = []
    for i in range(NUM_AIRCRAFT):
        radar_states.append(mav[i].getIntruderState(mav[i].true_state))
    # -------- autopilot -------------
    for id in range(NUM_AIRCRAFT):
        # commands.airspeed_command = Va_command[id].polynomial(sim_time)
        # commands.altitude_command = h_command[id].polynomial(sim_time)
        # commands.course_command = chi_command[id].polynomial(sim_time)
        
        measurements[id] = mav[id].sensors()  # get sensor measurements
        estimated_state[id] = observer[id].update(measurements[id])  # estimate states from measurements
        
        path = path_manager[id].update(waypoints, PLAN.R_min, mav[id].true_state)

        # -------path follower-------------
        autopilot_commands = path_follower[id].update(path, mav[id].true_state)
        
        
        
        
        if id == 0 and sim_time != 0:
            
            # pos0 = np.array([[mav[0].true_state.north , mav[0].true_state.east , -mav[0].true_state.altitude]]).T
            # vel0 = mav[0].true_state.Va * Euler2Rotation(0., 0., mav[0].true_state.chi) @ np.array([[1., 0., 0.]]).T
            
            # pos1 = np.array([[mav[1].true_state.north , mav[1].true_state.east , -mav[1].true_state.altitude]]).T
            # vel1 = mav[1].true_state.Va * Euler2Rotation(0., 0., mav[1].true_state.chi) @ np.array([[1., 0., 0.]]).T
            
            # R = pos1 - pos0
            # V = vel1 - vel0
            
            
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
            commands = autopilotSD.update(radar_states)
            autopilot_commands = commands
            # print("dist=", np.linalg.norm(R), "angle = ", np.rad2deg(np.arctan2(R[1] , R[0])))
            
            
            
            # print("dist=", np.linalg.norm(R), "angle = ", np.rad2deg(np.arctan2(R[1] , R[0])))
            # print("SPEED= ",commands.airspeed_command)
            # print((target_angle - np.abs(np.rad2deg(np.arctan2(R[1] , R[0])))[0]))
            # print("CC =" ,commands.course_command)
            
        
        delta[id], commanded_state[id] = autopilot[id].update(autopilot_commands, mav[id].true_state)
        
        # -------- physical system -------------
        current_wind[id] = wind[id].update()  # get the new wind vector
        
        mav[id].update(delta[id], current_wind[id])  # propagate the MAV dynamics

    # -------- update viewer -------------
        if ANIMATION:
            # for id in range(NUM_AIRCRAFT):    
            mav_view.update(mav[id].true_state, path, waypoints, id)  # plot body of MAV
        
    if DATA_PLOTS:
        plot_time = sim_time
        data_view.update(mav[0].true_state,  # true states
                         estimated_state[0], # estimated states
                         None,               # commanded states
                         delta[0])           # inputs to aircraft
    if SENSOR_PLOTS:
        sensor_view.update(measurements[0])
        
    
    if ANIMATION or DATA_PLOTS or SENSOR_PLOTS:
        app.processEvents()
    if VIDEO is True:
        video.update(sim_time)
        
    # -------Check to Quit the Loop-------
    if quitter.check_quit():
        break

    # -------increment time-------------
    sim_time += SIM.ts_simulation

# Save an Image of the Plot
if SAVE_PLOT_IMAGE:
    if DATA_PLOTS:
        data_view.save_plot_image("project_data_plot")
    if SENSOR_PLOTS:
        sensor_view.save_plot_image("project_sensor_plot")

if VIDEO is True:
    video.close()




