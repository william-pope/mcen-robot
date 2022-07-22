#!/usr/bin/env python3

import serial
import time
import copy
import numpy as np

from rpi_functions import *

def defense():
    # define constant parameters
    Dt = 0.1  # [s], period of main loop
    OBS_RPL_LENGTH = 25

    # initialize vectors
    mode_k = 5

    rpm_k = 0*np.ones(4)
    shoot_k = 0
    act_k = np.append(rpm_k, shoot_k)

    k_step = 0
    while k_step < 500:
        print("k: " + str(k_step) + ", mode: " + str(mode_k))

        # 1) send actuator command
        act_kb = u_to_bytes(act_k)
        ser.write(act_kb)

        ser.read(size=1)

        # print("act")
        # print(act_k)
        # print(act_kb)

        # 2) receive sensor data
        ser.write([0xdd])

        obs_kb = ser.read(size=OBS_RPL_LENGTH)
        obs_k = bytes_to_o(obs_kb)

        # print("obs")
        # print(obs_kb)
        # print(obs_k)
        
        # 3) enter mode operate
        if mode_k == 5:
            mode_k1, rpm_k1 = operate_m5(obs_k)
        elif mode_k == 6:
            mode_k1, rpm_k1 = operate_m6(obs_k)

        # test mode
        if mode_k == 9:
            mode_k1 = 9
            rpm_k1 = 0*np.ones(4)

        act_k1 = np.append(rpm_k1, 1)

        mode_k = copy.deepcopy(mode_k1)
        act_k = copy.deepcopy(act_k1)
        k_step += 1

        # time.sleep(Dt)

    stop_motors()
    
    return

# NOTE: different motions used:
#   - m5: translate sideways based on camera data
#   - m6: translate sideways based on camera data


# TO-DO: make sure if...elif logic works properly, not meeting multiple criteria


# mode 5: pre-whistle
def operate_m5(obs_k):
    mode_k1 = 5

    # OBS: check if whistle detected
    whistle = obs_k[-1]

    # MODE: if whistle heard, transition to shot mode
    if whistle == True:
        mode_k1 = 6
        rpm_k1 = np.zeros(4)

        print("whistle heard")
        return mode_k1, rpm_k1

    # ACT: moved according to random plan, use ball as reference point (TO-DO)s
    rpm_k1 = np.zeros(4)

    return mode_k1, rpm_k1


# mode 6: follow ball
def operate_m6(obs_k):
    mode_k1 = 6
    cam_offset = 15     # TO-DO: calibrate this

    # OBS: calculate current theta from CV observations
    v_ball = obs_k[2]
    dv_block = v_ball - cam_offset

    # ACT: generate action needed (either rotating or translating)
    rpm_k1 = control_m6_translate(dv_block)

    return mode_k1, rpm_k1


def stop_motors():
    rpm_k = np.zeros(4)
    act_k  = np.append(rpm_k, 0)
    act_kb = u_to_bytes(act_k)
    ser.write(act_kb)


# main
if __name__ == "__main__":
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(1)

    try:
        defense()
    except KeyboardInterrupt:
        stop_motors()
        print("stopped")