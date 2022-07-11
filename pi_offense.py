#!/usr/bin/env python3

import serial
import time

# terminology:
#   - x_k: state
#   - m_k: mode
#   - u_k: input
#   - o_k: observation

#   - x (state vector, all in goal frame):
#       - x_e: ego position [mm]
#       - y_e: ego position [mm]
#       - theta_e: ego heading angle [rad]
#       - x_b: ball position [mm]
#       - y_b: ball position [mm]
#       - x_d: defender position [mm] (TO-DO: add this later)
#       - y_d: defender position [mm] (TO-DO: add this later)
#       - whistle: whistle has been blown [bool]
#       - t_w: unix time when whistle was blown [time object]

#   - m (operating mode):
#       - m = 0: pre-whistle
#       - m = 1: move to hold point (rough)
#       - m = 2: calculate shot
#       - m = 3: move to shot point (fine)
#       - m = 4: wait for spot to clear, take shot

#   - u (input vector):
#       - r_m1: motor 1 RPM [rev/min] (limit to +/- some feasible change from current RPM)
#       - r_m2: motor 2 RPM
#       - r_m3: motor 3 RPM
#       - r_m4: motor 4 RPM
#       - open_sol: open solenoid [bool]

#   - o (observation vector):
#       - d_r1: distance, range sensor 1 [mm]
#       - d_r2: distance, range sensor 2 [mm]
#       - v_b: ball horizontal position [px] (camera frame)
#       - w_b: ball vertical position [px] (camera frame)
#       - tone: whistle heard [bool]

# Q: should create data classes for x, u, o? fields have different data types, so difficult to hold in single array

# TO-DO: figure out serial transfer first, then worry about data strctures

def offense():
    # define constant parameters

    # initialize serial port to Mega
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
    ser.reset_input_buffer()

    # initialize vectors
    x_k = [0.0]

    terminate = False
    while terminate == False:

        # 1) send actuator command
        u_kb = u_to_bytes(u_k)
        u_kb = 0x1A3F
        ser.write(u_kb)

        # 2) receive sensor data
        o_kb = ser.read(size=8)     # waits until n bytes are read from buffer
        o_k = bytes_to_o(o_kb)

        # 3) estimate state
        x_k = estimate_state(o_k)

        # NOTE: might combine #4-6 or #5-6 to keep each mode's operation closer together
        # 4) update mode
        m_k = update_mode(x_k)

        # 5) update guidance

        # 6) calculate input

    return

# converts input vector to byte package for sending to Mega
def u_to_bytes(u):

    return

# converts byte package received from Mega into sensor observations
def bytes_to_o(o_b):

    return

if __name__ == "__main__":
    offense()