"""
observer
    - Beard & McLain, PUP, 2012
    - Last Update:
        3/2/2019 - RWB
"""
import sys
import numpy as np
from scipy import stats
sys.path.append('..')
import parameters.control_parameters as CTRL
import parameters.simulation_parameters as SIM
import parameters.sensor_parameters as SENSOR
import parameters.aerosonde_parameters as MAV 
from tools.wrap import wrap
from message_types.msg_state import MsgState
from message_types.msg_sensors import MsgSensors

class Observer:
    def __init__(self, ts_control, initial_measurements = MsgSensors()):
        # initialized estimated state message
        self.estimated_state = MsgState()
        # use alpha filters to low pass filter gyros and accels
        # alpha = Ts/(Ts + tau) where tau is the LPF time constant

        ##### TODO #####
        self.lpf_gyro_x  = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_x)
        self.lpf_gyro_y  = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_y)
        self.lpf_gyro_z  = AlphaFilter(alpha=0.7, y0=initial_measurements.gyro_z)
        self.lpf_accel_x = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_x)
        self.lpf_accel_y = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_y)
        self.lpf_accel_z = AlphaFilter(alpha=0.7, y0=initial_measurements.accel_z)
        # use alpha filters to low pass filter absolute and differential pressure
        self.lpf_abs  = AlphaFilter(alpha=0.9, y0=initial_measurements.abs_pressure)
        self.lpf_diff = AlphaFilter(alpha=0.7, y0=initial_measurements.diff_pressure)
        # ekf for phi and theta
        self.attitude_ekf = EkfAttitude()
        # ekf for pn, pe, Vg, chi, wn, we, psi
        self.position_ekf = EkfPosition()
        # ekf for pn, pe, pd, Vg, chi, gamma of intruder
        #self.intruder_ekf = EkfIntruder()

    def update(self, measurement):
        ##### TODO #####

        # estimates for p, q, r are low pass filter of gyro minus bias estimate
        self.estimated_state.p = self.lpf_gyro_x.update(measurement.gyro_x) - SENSOR.gyro_x_bias # self.estimated_state.bx  
        self.estimated_state.q = self.lpf_gyro_y.update(measurement.gyro_y) - SENSOR.gyro_y_bias # self.estimated_state.by
        self.estimated_state.r = self.lpf_gyro_z.update(measurement.gyro_z) - SENSOR.gyro_z_bias # self.estimated_state.bz

        # invert sensor model to get altitude and airspeed
        self.estimated_state.altitude = self.lpf_abs.update(measurement.abs_pressure) / (MAV.rho * MAV.gravity)
        self.estimated_state.Va       = np.sqrt(2 * self.lpf_diff.update(measurement.diff_pressure) / MAV.rho)

        # estimate phi and theta with simple ekf
        self.attitude_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, Vg, chi, wn, we, psi
        self.position_ekf.update(measurement, self.estimated_state)

        # estimate pn, pe, pd, Vg, chi, gamma of intruder
        #self.intruder_ekf.update(measurement, self.estimated_state)

        # not estimating these
        self.estimated_state.alpha = self.estimated_state.phi
        self.estimated_state.beta = 0.0
        self.estimated_state.bx = 0.0
        self.estimated_state.by = 0.0
        self.estimated_state.bz = 0.0
        return self.estimated_state


class AlphaFilter:
    # alpha filter implements a simple low pass filter
    # y[k] = alpha * y[k-1] + (1-alpha) * u[k]
    def __init__(self, alpha=0.5, y0=0.0):
        self.alpha = alpha  # filter parameter
        self.y = y0  # initial condition

    def update(self, u):
        ##### TODO #####
        self.y = self.alpha * self.y + (1 - self.alpha) * u 
        return self.y


# class EkfIntruder:
#     # estimate pn, pe, pd, Vg, chi, gamma of intruder
#     def __init__(self):
#         self.Q = np.diag([
#                     0.1,  # pn
#                     0.1,  # pe
#                     0.5,  # Vg
#                     0.0001, # chi
#                     0.001, # wn
#                     0.001, # we
#                     0.0001, #0.0001, # psi
#                     ])
#         self.R_gps = np.diag([
#                     SENSOR.gps_n_sigma**2,  # y_gps_n
#                     SENSOR.gps_e_sigma**2,  # y_gps_e
#                     SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
#                     SENSOR.gps_course_sigma**2,  # y_gps_course
#                     ])
#         self.R_pseudo = np.diag([
#                     0.,  # pseudo measurement #1
#                     0.,  # pseudo measurement #2
#                     ])
#         self.N = 10  # number of prediction step per sample
#         self.Ts = (SIM.ts_control / self.N)
#         self.xhat = np.array([[0.0], [0.0], [24.], [0.0], [0.0], [0.0], [0.0]])
#         self.P = np.diag([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0])
#         self.gps_n_old = 0
#         self.gps_e_old = 0
#         self.gps_Vg_old = 0
#         self.gps_course_old = 0
#         self.pseudo_threshold = 100000 #stats.chi2.isf(q=?, df=?)
#         self.gps_threshold = 100000 # don't gate GPS

#     def update(self, measurement, state):
#         self.propagate_model(measurement, state)
#         self.measurement_update(measurement, state)
#         state.north = self.xhat.item(0)
#         state.east = self.xhat.item(1)
#         state.Vg = self.xhat.item(2)
#         state.chi = self.xhat.item(3)
#         state.wn = self.xhat.item(4)
#         state.we = self.xhat.item(5)
#         state.psi = self.xhat.item(6)
#         # print('chi=',state.chi)

#     def f(self, x, measurement, state):

#         north = x.item(0)
#         east  = x.item(1)
#         Vg    = x.item(2)
#         chi   = x.item(3)
#         wn    = x.item(4)
#         we    = x.item(5)
#         psi   = x.item(6)

#         # p = measurement.gyro_x - state.bx
#         # q = measurement.gyro_y - state.by
#         # r = measurement.gyro_z - state.bz

#         Va = state.Va #np.sqrt(2 * measurement.diff_pressure  / MAV.rho)
#         p = state.p
#         q = state.q
#         r = state.r

#         psi_d = q * np.sin(state.phi) / np.cos(state.theta) + r * np.cos(state.phi) / np.cos(state.theta)

#         # print(Vg)
#         #print((state.Va * np.cos(psi) + wn)*(-state.Va * psi_d * np.sin(psi)) + (state.Va * np.sin(psi) + we)*(state.Va * psi_d * np.sin(psi)) / Vg)
        
#         # system dynamics for propagation model: xdot = f(x, u)
#         f_ = np.array([[Vg*np.cos(chi)],
#                        [Vg*np.sin(chi)],
#                        [((Va * np.cos(psi) + wn)*(-Va * psi_d * np.sin(psi)) + (Va * np.sin(psi) + we)*(Va * psi_d * np.cos(psi))) / Vg],
#                        [MAV.gravity / Vg * np.tan(state.phi) * np.cos(chi-psi)],
#                        [0.0],
#                        [0.0],
#                        [psi_d],
#                        ])
#         # print('f_=',f_[2])
#         # print('x=',x)
#         return f_

#     def h_gps(self, x, measurement, state):
#         # measurement model for gps measurements
#         h_ = np.array([
#             [x.item(0)], #pn
#             [x.item(1)], #pe
#             [x.item(2)], #Vg
#             [x.item(3)], #chi
#         ])
#         return h_

#     def h_pseudo(self, x, measurement, state):

#         north = x.item(0)
#         east  = x.item(1)
#         Vg    = x.item(2)
#         chi   = x.item(3)
#         wn    = x.item(4)
#         we    = x.item(5)
#         psi   = x.item(6)

#         Va = state.Va #np.sqrt(2 * measurement.diff_pressure  / MAV.rho)

#         # measurement model for wind triangale pseudo measurement
#         h_ = np.array([
#             [Va * np.cos(psi) + wn - Vg*np.cos(chi)],  # wind triangle x
#             [Va * np.sin(psi) + we - Vg*np.sin(chi)],  # wind triangle y
#         ])
#         return h_

#     def propagate_model(self, measurement, state):
#         # print('chi=',self.xhat[6,0])
#         # model propagation
#         Tp = self.Ts
#         for i in range(0, self.N):
#             # propagate model
#             # self.xhat = np.zeros((7,1))
#             self.xhat = self.xhat + Tp * self.f(self.xhat, measurement, state)
            
#             # compute Jacobian
#             A = jacobian(self.f, self.xhat, measurement, state)
#             # convert to discrete time models
#             Ad = np.eye(7,7) + A * Tp + A @ A * Tp**2
            
#             # update P with discrete time model
#             # self.P = np.zeros((7,7))
#             self.P = Ad @ self.P @ Ad.T + Tp**2 * self.Q


#     def measurement_update(self, measurement, state):
#         # always update based on wind triangle pseudo measurement
#         h = self.h_pseudo(self.xhat, measurement, state)
#         C = jacobian(self.h_pseudo, self.xhat, measurement, state)
#         y = np.array([[0.0, 0.0]]).T
#         # S_inv = np.zeros((2,2))
#         S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
#         if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
#             # self.P = np.zeros((7,7))
#             # self.xhat = np.zeros((7,1))
#             L = self.P @ C.T @ S_inv
#             tmp = np.eye(7) - L @ C
#             self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
#             self.xhat = self.xhat + L @ (y - h)

#         # only update GPS when one of the signals changes
#         if (measurement.gps_n != self.gps_n_old) \
#             or (measurement.gps_e != self.gps_e_old) \
#             or (measurement.gps_Vg != self.gps_Vg_old) \
#             or (measurement.gps_course != self.gps_course_old):

#             h = self.h_gps(self.xhat, measurement, state)
#             C = jacobian(self.h_gps, self.xhat, measurement, state)
#             y_chi = wrap(measurement.gps_course, h[3, 0])
#             y = np.array([[measurement.gps_n,
#                            measurement.gps_e,
#                            measurement.gps_Vg,
#                            y_chi]]).T
#             # S_inv = np.zeros((4,4))
#             S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
#             if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
#                 # self.P = np.zeros((7,7))
#                 # self.xhat = np.zeros((7,1))
#                 L = self.P @ C.T @ S_inv
#                 tmp = np.eye(7) - L @ C
#                 self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T
#                 self.xhat = self.xhat + L @ (y - h)

#             # update stored GPS signals
#             self.gps_n_old = measurement.gps_n
#             self.gps_e_old = measurement.gps_e
#             self.gps_Vg_old = measurement.gps_Vg
#             self.gps_course_old = measurement.gps_course

class EkfAttitude:
    # implement continous-discrete EKF to estimate roll and pitch angles
    def __init__(self):
        ##### TODO #####
        self.Q = np.diag([1e-6, 1e-6])
        self.Q_gyro = np.diag([SENSOR.gyro_sigma**2, SENSOR.gyro_sigma**2, SENSOR.gyro_sigma**2])
        self.R_accel = np.diag([SENSOR.accel_sigma**2, SENSOR.accel_sigma**2, SENSOR.accel_sigma**2])
        self.N = 10  # number of prediction step per sample
        self.xhat = np.array([[0.0], [0.0]]) # initial state: phi, theta
        self.P = np.diag([1.0, 1.0])
        self.Ts = SIM.ts_control/self.N
        self.gate_threshold = 9999 #stats.chi2.isf(q=?, df=?)

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.phi = self.xhat.item(0)
        state.theta = self.xhat.item(1)

    def f(self, x, measurement, state):
        # system dynamics for propagation model: xdot = f(x, u)
        ##### TODO #####

        phi = x.item(0)
        theta = x.item(1)
        # p = measurement.gyro_x - state.bx
        # q = measurement.gyro_y - state.by
        # r = measurement.gyro_z - state.bz

        p = state.p
        q = state.q
        r = state.r

        G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi)*np.tan(theta)],
                          [0, np.cos(phi), -np.sin(phi) ]])

        f_ = G @ np.array([[p],[q],[r]])
        return f_

    def h(self, x, measurement, state):

        phi = x.item(0)
        theta = x.item(1)
        # p = measurement.gyro_x - state.bx
        # q = measurement.gyro_y - state.by
        # r = measurement.gyro_z - state.bz

        p = state.p
        q = state.q
        r = state.r

        Va = np.sqrt(2 * measurement.diff_pressure  / MAV.rho)
 
        ##### TODO #####
        h_ = np.array([[(q * Va * np.sin(theta) + MAV.gravity * np.sin(theta))],    # x-accel
                       [(r * Va * np.cos(theta) - p * Va * np.sin(theta) - MAV.gravity * np.cos(theta) * np.sin(phi))],   # y-accel
                       [( -q * Va * np.cos(theta) - MAV.gravity * np.cos(theta) * np.cos(phi))]])  # z-accel
        

        return h_

    def propagate_model(self, measurement, state):
        # model propagation

        ##### TODO #####
        Tp = self.Ts
        for i in range(0, self.N):
            phi = self.xhat.item(0)
            theta = self.xhat.item(1)
            self.xhat = self.xhat + Tp * self.f(self.xhat, measurement, state)
            # A = np.zeros((2,2))
            A = jacobian(self.f, self.xhat, measurement, state)
            # Ad = np.zeros((2,2))
            Ad = np.eye(2,2) + A * Tp + A@A * Tp**2
            # self.P = np.zeros((2,2))
            G = np.array([[1, np.sin(phi) * np.tan(theta), np.cos(phi)*np.tan(theta)],
                          [0, np.cos(phi), -np.sin(phi) ]])
            self.P = Ad @ self.P @ Ad.T + Tp**2 * (G @ self.Q_gyro @ G.T + self.Q)

    def measurement_update(self, measurement, state):
        # measurement updates
        h = self.h(self.xhat, measurement, state)
        C = jacobian(self.h, self.xhat, measurement, state)
        y = np.array([[measurement.accel_x, measurement.accel_y, measurement.accel_z]]).T

        ##### TODO #####
        # S_inv = np.zeros((3,3))
        S_inv = np.linalg.inv(self.R_accel + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.gate_threshold:
            # self.P = np.zeros((2,2))
            # self.xhat = np.zeros((2,1))
            L = self.P @ C.T @ S_inv
            tmp = np.eye(2) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_accel @ L.T
            self.xhat = self.xhat + L @ (y - h)



class EkfPosition:
    # implement continous-discrete EKF to estimate pn, pe, Vg, chi, wn, we, psi
    def __init__(self):
        self.Q = np.diag([
                    0.1,  # pn
                    0.1,  # pe
                    0.5,  # Vg
                    0.0001, # chi
                    0.001, # wn
                    0.001, # we
                    0.0001, #0.0001, # psi
                    ])
        self.R_gps = np.diag([
                    SENSOR.gps_n_sigma**2,  # y_gps_n
                    SENSOR.gps_e_sigma**2,  # y_gps_e
                    SENSOR.gps_Vg_sigma**2,  # y_gps_Vg
                    SENSOR.gps_course_sigma**2,  # y_gps_course
                    ])
        self.R_pseudo = np.diag([
                    0.,  # pseudo measurement #1
                    0.,  # pseudo measurement #2
                    ])
        self.N = 10  # number of prediction step per sample
        self.Ts = (SIM.ts_control / self.N)
        self.xhat = np.array([[0.0], [0.0], [24.], [0.0], [0.0], [0.0], [0.0]])
        self.P = np.diag([1.0, 1.0, 1.0, 1.0, 0.0, 0.0, 1.0])
        self.gps_n_old = 0
        self.gps_e_old = 0
        self.gps_Vg_old = 0
        self.gps_course_old = 0
        self.pseudo_threshold = 100000 #stats.chi2.isf(q=?, df=?)
        self.gps_threshold = 100000 # don't gate GPS

    def update(self, measurement, state):
        self.propagate_model(measurement, state)
        self.measurement_update(measurement, state)
        state.north = self.xhat.item(0)
        state.east = self.xhat.item(1)
        state.Vg = self.xhat.item(2)
        state.chi = self.xhat.item(3)
        state.wn = self.xhat.item(4)
        state.we = self.xhat.item(5)
        state.psi = self.xhat.item(6)
        # print('chi=',state.chi)

    def f(self, x, measurement, state):

        north = x.item(0)
        east  = x.item(1)
        Vg    = x.item(2)
        chi   = x.item(3)
        wn    = x.item(4)
        we    = x.item(5)
        psi   = x.item(6)

        # p = measurement.gyro_x - state.bx
        # q = measurement.gyro_y - state.by
        # r = measurement.gyro_z - state.bz

        Va = state.Va #np.sqrt(2 * measurement.diff_pressure  / MAV.rho)
        p = state.p
        q = state.q
        r = state.r

        psi_d = q * np.sin(state.phi) / np.cos(state.theta) + r * np.cos(state.phi) / np.cos(state.theta)

        # print(Vg)
        #print((state.Va * np.cos(psi) + wn)*(-state.Va * psi_d * np.sin(psi)) + (state.Va * np.sin(psi) + we)*(state.Va * psi_d * np.sin(psi)) / Vg)
        
        # system dynamics for propagation model: xdot = f(x, u)
        f_ = np.array([[Vg*np.cos(chi)],
                       [Vg*np.sin(chi)],
                       [((Va * np.cos(psi) + wn)*(-Va * psi_d * np.sin(psi)) + (Va * np.sin(psi) + we)*(Va * psi_d * np.cos(psi))) / Vg],
                       [MAV.gravity / Vg * np.tan(state.phi) * np.cos(chi-psi)],
                       [0.0],
                       [0.0],
                       [psi_d],
                       ])
        # print('f_=',f_[2])
        # print('x=',x)
        return f_

    def h_gps(self, x, measurement, state):
        # measurement model for gps measurements
        h_ = np.array([
            [x.item(0)], #pn
            [x.item(1)], #pe
            [x.item(2)], #Vg
            [x.item(3)], #chi
        ])
        return h_

    def h_pseudo(self, x, measurement, state):

        north = x.item(0)
        east  = x.item(1)
        Vg    = x.item(2)
        chi   = x.item(3)
        wn    = x.item(4)
        we    = x.item(5)
        psi   = x.item(6)

        Va = state.Va #np.sqrt(2 * measurement.diff_pressure  / MAV.rho)

        # measurement model for wind triangale pseudo measurement
        h_ = np.array([
            [Va * np.cos(psi) + wn - Vg*np.cos(chi)],  # wind triangle x
            [Va * np.sin(psi) + we - Vg*np.sin(chi)],  # wind triangle y
        ])
        return h_

    def propagate_model(self, measurement, state):
        # print('chi=',self.xhat[6,0])
        # model propagation
        Tp = self.Ts
        for i in range(0, self.N):
            # propagate model
            # self.xhat = np.zeros((7,1))
            self.xhat = self.xhat + Tp * self.f(self.xhat, measurement, state)
            
            # compute Jacobian
            A = jacobian(self.f, self.xhat, measurement, state)
            # convert to discrete time models
            Ad = np.eye(7,7) + A * Tp + A @ A * Tp**2
            
            # update P with discrete time model
            # self.P = np.zeros((7,7))
            self.P = Ad @ self.P @ Ad.T + Tp**2 * self.Q


    def measurement_update(self, measurement, state):
        # always update based on wind triangle pseudo measurement
        h = self.h_pseudo(self.xhat, measurement, state)
        C = jacobian(self.h_pseudo, self.xhat, measurement, state)
        y = np.array([[0.0, 0.0]]).T
        # S_inv = np.zeros((2,2))
        S_inv = np.linalg.inv(self.R_pseudo + C @ self.P @ C.T)
        if (y-h).T @ S_inv @ (y-h) < self.pseudo_threshold:
            # self.P = np.zeros((7,7))
            # self.xhat = np.zeros((7,1))
            L = self.P @ C.T @ S_inv
            tmp = np.eye(7) - L @ C
            self.P = tmp @ self.P @ tmp.T + L @ self.R_pseudo @ L.T
            self.xhat = self.xhat + L @ (y - h)

        # only update GPS when one of the signals changes
        if (measurement.gps_n != self.gps_n_old) \
            or (measurement.gps_e != self.gps_e_old) \
            or (measurement.gps_Vg != self.gps_Vg_old) \
            or (measurement.gps_course != self.gps_course_old):

            h = self.h_gps(self.xhat, measurement, state)
            C = jacobian(self.h_gps, self.xhat, measurement, state)
            y_chi = wrap(measurement.gps_course, h[3, 0])
            y = np.array([[measurement.gps_n,
                           measurement.gps_e,
                           measurement.gps_Vg,
                           y_chi]]).T
            # S_inv = np.zeros((4,4))
            S_inv = np.linalg.inv(self.R_gps + C @ self.P @ C.T)
            if (y-h).T @ S_inv @ (y-h) < self.gps_threshold:
                # self.P = np.zeros((7,7))
                # self.xhat = np.zeros((7,1))
                L = self.P @ C.T @ S_inv
                tmp = np.eye(7) - L @ C
                self.P = tmp @ self.P @ tmp.T + L @ self.R_gps @ L.T
                self.xhat = self.xhat + L @ (y - h)

            # update stored GPS signals
            self.gps_n_old = measurement.gps_n
            self.gps_e_old = measurement.gps_e
            self.gps_Vg_old = measurement.gps_Vg
            self.gps_course_old = measurement.gps_course


def jacobian(fun, x, measurement, state):
    # compute jacobian of fun with respect to x
    f = fun(x, measurement, state)
    m = f.shape[0]
    n = x.shape[0]
    eps = 0.0001  # deviation
    J = np.zeros((m, n))
    for i in range(0, n):
        x_eps = np.copy(x)
        x_eps[i][0] += eps
        f_eps = fun(x_eps, measurement, state)
        df = (f_eps - f) / eps
        J[:, i] = df[:, 0]
    return J