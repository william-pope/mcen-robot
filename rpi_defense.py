#!/usr/bin/env python3

import serial
import time
import copy
import numpy as np

from rpi_functions import *

def defense():
    # define constant parameters
    Dt = 0.25  # [s], period of main loop
    OBS_RPL_LENGTH = 25
    # TO-DO: would like to define a dataclass to hold constants in a struct (or just define as globals?)

    ser = serial.Serial('/dev/ttyACM1', 115200)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(1)

    # initialize vectors
    mode_k = 9

    rpm_k = np.zeros(4)
    act_k = np.append(rpm_k, 1)

    k_step = 0
    while k_step < 3:
        print(k_step)

        # 1) send actuator command
        act_kb = u_to_bytes(act_k)
        ser.write(act_kb)

        print("act")
        print(act_k)
        print(act_kb)

        ser.read(size=1)

        # 2) receive sensor data
        ser.write([0xbb])

        obs_kb = ser.read(size=OBS_RPL_LENGTH)
        obs_k = bytes_to_o(obs_kb)

        print("obs")
        print(obs_kb)
        print(obs_k)

        break
        
        # 3) enter mode operate
        if mode_k == 5:
            mode_k1, rpm_k1 = operate_m5(obs_k)
        elif mode_k == 6:
            mode_k1, rpm_k1 = operate_m6(obs_k)

        # test mode
        if mode_k == 9:
            mode_k1 = 9
            rpm_k1 = 0*np.ones(4)

        act_k1 = np.append(rpm_k1, 0)

        mode_k = copy.deepcopy(mode_k1)
        act_k = copy.deepcopy(act_k1)
        k_step += 1

        time.sleep(Dt)
    
    return

# NOTE: different motions used:
#   - m5: translate sideways based on camera data
#   - m6: translate sideways based on camera data


# TO-DO: make sure if...elif logic works properly, not meeting multiple criteria


# mode 5: pre-whistle
def operate_m5(obs_k):
    mode_k1 = 5

    # OBS: check if whistle detected
    if obs_k[-1] == 1:
        t_w = time.now()

    # process CV data to track attacker (TBD)
    # ...

    # MODE: if whistle delay has passed, move on to next mode   # TO-DO: need to store t_w outside function in order to reference in subsequest calls
    if time.now() - t_w >= 0:
        mode_k1 = 6
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # ACT: moved according to random plan, use ball as reference point (TO-DO)s
    rpm_k1 = np.zeros(4)

    return mode_k1, rpm_k1


# mode 6: follow ball
def operate_m6(obs_k):
    mode_k1 = 6
    cam_offset = 15

    # OBS: calculate current theta from CV observations
    v_ball = obs_k[2]
    dv_block = v_ball - cam_offset

    # ACT: generate action needed (either rotating or translating)
    rpm_k1 = control_m6_translate(dv_block)

    return mode_k1, rpm_k1


# main
if __name__ == "__main__":
    defense()