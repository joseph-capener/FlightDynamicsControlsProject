"""
msg_intruder
    - messages type for output of intruder states
    
"""


class MsgIntruder:
    def __init__(self):
        self.radar_n = 0  # radar north
        self.radar_e = 0  # radar east
        self.radar_h = 0  # radar altitude
        self.radar_Vg = 0  # numerical derivative of position (dP/dt)
        self.radar_course = 0  # gps course angle
        self.radar_flight_path = 0 # radar flight path