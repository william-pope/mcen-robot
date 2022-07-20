#!/usr/bin/env python3

from smbus2 import SMBus
import time
import copy
import numpy as np

from rpi_functions import *

mega_addr = 0x1a
mega_reg = 0x00


# TO-DO: 
#   - test drive functions
#   - test mode switching

# (?): should create data classes for x, u, o? fields have different data types, so difficult to hold in single array

def offense():
    # define constant parameters
    Dt = 0.25   # [s], period of main loop
    # TO-DO: would like to define a dataclass to hold constants in a struct (or just define as globals?)

    # initialize I2C bus
    i2c = SMBus(1)
    time.sleep(2)

    # initialize vectors
    mode_k = 9

    rpm_k = np.zeros(4)
    shoot_k = 0
    act_k = np.append(rpm_k, shoot_k)

    x_dr_k = [0, 0]

    k_step = 0
    while k_step < 3:
        print(k_step)

        # 1) send actuator command
        act_kb = u_to_bytes(act_k)
        i2c.write_block_data(mega_addr, mega_reg, act_kb)

        # 2) receive sensor data
        # obs_kb = i2c.read_i2c_block_data(mega_addr, mega_reg, 2)
        # while True:
        #     try:
        #         obs_kb = i2c.read_i2c_block_data(mega_addr, mega_reg, 2)
        #     except:
        #         print("i2c error, trying again")
        #         continue
        #     else:
        #         break

        # obs_k = bytes_to_o(obs_kb)
        
        # 3) enter mode operate
        shoot_k1 = 0
        if mode_k == 0:     # pre-whistle
            mode_k1, rpm_k1, target = operate_m0(obs_k)
        elif mode_k == 1:   # set angle
            mode_k1, rpm_k1 = operate_m1(obs_k, target)
        elif mode_k == 2:   # rough distance
            mode_k1, rpm_k1, x_dr_k1 = operate_m2(obs_k, x_dr_k)
            x_dr_k = copy.deepcopy(x_dr_k1)
        elif mode_k == 3:   # fine distance
            mode_k1, rpm_k1 = operate_m3(obs_k)
        elif mode_k == 4:   # hold for shot
            mode_k1, rpm_k1, shoot_k1 = operate_m4(obs_k)

        # test mode
        if mode_k == 9:
            mode_k1 = 9
            rpm_k1 = 0*np.ones(4)
            shoot_k1 = 0

        act_k1 = np.append(rpm_k1, shoot_k1)

        mode_k = copy.deepcopy(mode_k1)
        act_k = copy.deepcopy(act_k1)
        k_step += 1

        time.sleep(Dt)
    
    return

# TO-DO: make sure if...elif logic works properly, not meeting multiple criteria

# mode 0: pre-whistle
def operate_m0(obs_k):
    mode_k1 = 0

    # OBS: check if whistle detected
    if obs_k[-1] == 1:
        t_w = time.now()

    # process CV data to track goalie (TBD)
    # ...

    # calculate best shot
    target = "right"

    # MODE: if whistle delay has passed, move on to next mode   # TO-DO: need to store t_w outside function in order to reference in subsequent calls
    if time.now() - t_w >= 0:
        mode_k1 = 1
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1, target

    # ACT: no action in this mode
    rpm_k1 = np.zeros(4)

    return mode_k1, rpm_k1, target


# mode 1: set angle
def operate_m1(obs_k, target):
    mode_k1 = 1

    cent_tol = 20       # [px], TO-DO: set these to real values
    rot_tol = 5         # [px]
    targ_tol = 10       # [px]
    v_targ_offset = 30  # [px]

    # OBS: process Pixy data
    v_ball = obs_k[2]
    v_left_post = obs_k[4]
    v_right_post = obs_k[6]

    if target == "left":
        dv_targ = (v_left_post + v_targ_offset) - v_ball

        if v_left_post == 0:    # if post not recognized, stop and repeat loop
            rpm_k1 = np.zeros(4)
            return mode_k1, rpm_k1
    elif target == "right":
        dv_targ = (v_right_post - v_targ_offset) - v_ball

        if v_right_post == 0:   # if post not recognized, stop and repeat loop
            rpm_k1 = np.zeros(4)
            return mode_k1, rpm_k1

    # MODE: if aligned with target, move on to next mode
    if abs(v_ball) <= cent_tol and abs(dv_targ) <= targ_tol:
        mode_k1 = 9     # TEST: end run
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # ACT: generate action needed (either rotating or translating)
    if abs(v_ball) > cent_tol:
        rpm_k1 = control_m1_rotate(v_ball, rot_tol)
    else:
        rpm_k1 = control_m1_translate(dv_targ)

    return mode_k1, rpm_k1


# mode 2: set distance (rough)
def operate_m2(obs_k, x_dr_k):
    mode_k1 = 2

    x_ref = np.array([30, 20])

    # MODE: if at x_ref, move on to next mode
    if x_dr_k[1] >= x_ref[1] and x_dr_k[0] >= x_ref[0]:
        mode_k1 = 3
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1
        
    # ACT: select action based on dead-reckoning position
    if x_dr_k[1] < x_ref[1]:
        rpm_k1 = control_m2_left(x_dr_k, x_ref)
    elif x_dr_k[1] >= x_ref[1] and x_dr_k[0] < x_ref[0]:
        rpm_k1 = control_m2_fwd(x_dr_k, x_ref)

    # propagate state
    x_dr_k1 = [0, 0]
        
    return mode_k1, rpm_k1, x_dr_k1


# mode 3: set distance (fine)
def operate_m3(obs_k):
    mode_k1 = 3
    x_rel_targ = np.array([78.1, -65.6])  # [mm], distance from robot center to ball center # TO-DO
    x_rel_tol = 1.5             # [mm], allowable error

    # OBS: calculate relative x-y position from range sensors data
    x_rel_ball = estimate_m3_ball(obs_k)
    dx_rel = np.subtract(x_rel_targ, x_rel_ball)

    # MODE: if at target position, move on to next mode
    if max(np.abs(dx_rel)) <= x_rel_tol:
        mode_k1 = 4
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # ACT: select action based on relative position
    rpm_k1 = control_m3_translate(dx_rel)

    return mode_k1, rpm_k1


# mode 4: hold for shot
def operate_m4(obs_k):
    mode_k1 = 4

    # OBS: if goalie moves into target position, wait 5 seconds to see if they leave
    # ...

    # ACT
    rpm_k1 = np.zeros(4)
    shoot_k1 = True
    
    return mode_k1, rpm_k1, shoot_k1


# main
if __name__ == "__main__":
    offense()


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
#       - x_d: defender position [mm] (TO-DO: add this later)
#       - y_d: defender position [mm] (TO-DO: add this later)
#       - t_w: unix time when whistle was blown [time object]

#   - m (operating mode):
#       - m = 0: acquire env (w-30 sec) -> acquire objects w/ CV, choose target (@ point 1)
#       - m = 1: set angle (w+5 sec) -> move along arc to align shot angle (point 1->2)
#           - move horizontal until ball out of center by some threshold, rotate until realigned at center
#       - m = 2: set distance, rough (w+10 sec) -> move in to ultrasonic range (point 2->3)
#       - m = 3: set distance, fine (w+13 sec) -> move in to contact distance (point 3->4)
#       - m = 4: hold for shot (w+15 sec) -> wait for spot to clear, take shot (@ point 4)

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
#       - v_mL: green marker horizontal position [px] (camera frame)
#       - w_mL: green marker vertical position [px] (camera frame)
#       - v_mR: pink marker horizontal position [px] (camera frame)
#       - w_mR: pink marker vertical position [px] (camera frame)
#       - tone: whistle heard [bool]