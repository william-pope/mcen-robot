#!/usr/bin/env python3

from smbus2 import SMBus
import time
import copy
import numpy as np

from rpi_functions import *

mega_addr = 0x1a
mega_reg = 0x00


# TO-DO: 
#   - finish operator functions
#   - test drive functions
#   - test mode switching
#   - write defense script


# terminology:
#   - x_k: state
#   - m_k: mode
#   - act_k: input
#   - obs_k: observation

#   - x (state vector, all in goal frame):
#       - x_e: ego position [mm]
#       - y_e: ego position [mm]
#       - theta_e: ego heading angle [rad]
#       - x_b: ball position [mm]
#       - y_b: ball position [mm]
#       - x_a: attacker position [mm] (TO-DO: add this later)
#       - y_a: attacker position [mm] (TO-DO: add this later)
#       - t_w: unix time when whistle was blown [time object]

#   - m (operating mode):
#       - m = 5: pre-whistle
#       - m = 6: tracking ball

#   - u (input vector):
#       - r_m1: motor 1 RPM [rev/min] (limit to +/- some feasible change from current RPM)
#       - r_m2: motor 2 RPM
#       - r_m3: motor 3 RPM
#       - r_m4: motor 4 RPM
#       - shoot: open solenoid [bool]

#   - o (observation vector):
#       - v_ball: ball horizontal position [px] (camera frame)
#       - w_ball: ball vertical position [px] (camera frame)
#       - v_attacker
#       - w_attacker
#       - tone: whistle heard [bool]

# (?): should create data classes for x, u, o? fields have different data types, so difficult to hold in single array

def defense():
    # define constant parameters
    Dt = 0.1   # [s], period of main loop
    # TO-DO: would like to define a dataclass to hold constants in a struct (or just define as globals?)

    # initialize I2C bus
    i2c = SMBus(1)
    time.sleep(1)

    # initialize vectors
    mode_k = 9

    rpm_k = np.zeros(4)
    act_k = np.append(rpm_k, 0)

    k_step = 0
    while k_step < 3:
        print(k_step)

        # 1) send actuator command
        act_kb = u_to_bytes(act_k)
        i2c.write_block_data(mega_addr, mega_reg, act_kb)

        # 2) receive sensor data
        while True:
            try:
                obs_kb = i2c.read_i2c_block_data(mega_addr, mega_reg, 2)
            except:
                print("i2c error, trying again")
                continue
            else:
                break

        obs_k = bytes_to_o(obs_kb)
        
        # 3) enter mode operate
        if mode_k == 5:
            mode_k1, rpm_k1 = operate_m5(obs_k)
        elif mode_k == 6:
            mode_k1, rpm_k1 = operate_m6(obs_k)

        # test mode
        if mode_k == 9:
            mode_k1 = 9
            rpm_k1 = 50*np.ones(4)

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

    # process CV data to track ball (TBD)
    v_ball = 100

    # MODE: if whistle delay has passed, move on to next mode   # TO-DO: need to store t_w outside function in order to reference in subsequest calls
    if time.now() - t_w >= 0:
        mode_k1 = 6
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # ACT: moved according to random plan, use ball as reference point
    rpm_k1 = mec_translate(50, "left")

    return mode_k1, rpm_k1


# mode 6: follow ball
def operate_m6(obs_k):
    mode_k1 = 6

    # TO-DO: should be some offset since camera isn't centered on robot cross section

    # OBS: calculate current theta from CV observations
    # v_ball = obs_k[6]
    v_ball = 100

    # ACT: generate action needed (either rotating or translating)
    rpm_k1 = control_m6_translate(v_ball)

    return mode_k1, rpm_k1


# main
if __name__ == "__main__":
    defense()