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
from estimation.observer import Observer
# from estimation.observer_full import Observer
from viewers.mav_viewer_multi import MavViewer
from viewers.data_viewer import DataViewer
from viewers.sensor_viewer import SensorViewer
from tools.quit_listener import QuitListener

quitter = QuitListener()

VIDEO = False
DATA_PLOTS = False
SENSOR_PLOTS = False
ANIMATION = True
SAVE_PLOT_IMAGE = False
COMPUTE_MODEL = False
NUM_AIRCRAFT = 2

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

wind = [WindSimulation(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]
mav = [MavDynamics(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]    
autopilot = [Autopilot(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]
observer = [Observer(SIM.ts_simulation) for i in range(NUM_AIRCRAFT)]   

measurements    = [None] * NUM_AIRCRAFT
estimated_state = [None] * NUM_AIRCRAFT  # estimate states from measurements
delta           = [None] * NUM_AIRCRAFT
commanded_state = [None] * NUM_AIRCRAFT
current_wind    = [None] * NUM_AIRCRAFT

mav[1]._state[1] = 35. 


# autopilot commands
from message_types.msg_autopilot import MsgAutopilot
commands = MsgAutopilot()
Va_command = Signals(dc_offset=25.0,
                     amplitude=3.0,
                     start_time=2.0,
                     frequency = 0.01)
h_command = Signals(dc_offset=100.0,
                    amplitude=20.0,
                    start_time=0.0,
                    frequency=0.02)
chi_command = [Signals(dc_offset=np.radians(0.0),
                      amplitude=np.radians(45.0),
                      start_time=5.0,
                      frequency=0.015),
               Signals(dc_offset=np.radians(0.0),
                      amplitude=np.radians(45.0),
                      start_time=1.0,
                      frequency=0.015)]


# initialize the simulation time
sim_time = SIM.start_time
end_time = 100

# main simulation loop
print("Press 'Esc' to exit...")
while sim_time < end_time:

    # -------autopilot commands-------------
    commands.airspeed_command = Va_command.polynomial(sim_time)
    # commands.course_command   = chi_command.polynomial(sim_time)
    commands.altitude_command = h_command.polynomial(sim_time)

    # -------- autopilot -------------
    for id in range(NUM_AIRCRAFT):
        commands.course_command = chi_command[id].polynomial(sim_time)
        
        measurements[id] = mav[id].sensors()  # get sensor measurements
        estimated_state[id] = observer[id].update(measurements[id])  # estimate states from measurements
        delta[id], commanded_state[id] = autopilot[id].update(commands, estimated_state[id])
        
        # -------- physical system -------------
        current_wind[id] = wind[id].update()  # get the new wind vector
        
        mav[id].update(delta[id], current_wind[id])  # propagate the MAV dynamics

    # -------- update viewer -------------
    if ANIMATION:
        for id in range(NUM_AIRCRAFT):    
            mav_view.update(mav[id].true_state, id)  # plot body of MAV
        
    if DATA_PLOTS:
        plot_time = sim_time
        data_view.update(mav[0].true_state,  # true states
                         estimated_state[0],  # estimated states
                         None,  # commanded states
                         delta[0])  # inputs to aircraft
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




