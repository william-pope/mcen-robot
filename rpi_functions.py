#!/usr/bin/env python3

import time
import copy
import numpy as np

def control_m1_translate(dv_targ):
    # translate to move towards target angle
    if dv_targ > 0:
        rpm_k1 = mec_translate(100, "left")
    elif dv_targ < 0:
        rpm_k1 = mec_translate(100, "right")
    else:
        rpm_k1 = np.zeros(4)

    return rpm_k1

def control_m1_rotate(v_ball):
    # rotate to bring ball back to center (v_ball -> 0)
    if v_ball > 0:
        rpm_k1 = mec_rotate(100, "CW")
    elif v_ball < 0:
        rpm_k1 = mec_rotate(100, "CCW")
    else:
        rpm_k1 = np.zeros(4)

    return rpm_k1   

def control_m2_left(x_dr_k, x_ref):
    rpm_k1 = np.zeros(4)    # TO-DO: sideways to left, scale speed

    return rpm_k1 

def control_m2_fwd(x_dr_k, x_ref):
    rpm_k1 = np.zeros(4)    # TO-DO: forward, scale speed

    return rpm_k1 

# TO-DO:
# use range sensor measurements and vehicle geometry to calculate relative position of ball
def estimate_m3_ball(obs_k):
    r_ball_at_h = 80

    u1_pos_bf = [50, 50, -np.pi/4]
    u2_pos_bf = [45, -50, -np.pi/12]

    u1_meas = obs_k[0]
    u2_meas = obs_k[1]

    x_rel_ball = [10, 10]

    return x_rel_ball

# TO-DO:
def control_m3_translate(dx_rel):
    rpm_k1 = np.zeros(4)

    return rpm_k1 

def control_m6_translate(dv_block):
    if dv_block >= 0:
        direction = "right"
    elif dv_block < 0:
        direction = "left"

    # print(direction)

    max_dv = 25
    speed = np.clip(466 * abs(dv_block)/max_dv, -466, 466)

    # print(dv_block)
    # print(speed)
    # print("")

    # TO-DO: scale speed proportionally to ball distance
    rpm_k1 = mec_translate(speed, direction)

    return rpm_k1

# mecanum wheel translation (cardinal directions)
def mec_translate(speed, direction):
    d_wheel = 97  # [mm]

    # convert vehicle speed [mm/sec] to wheel RPM [rev/min]
    #   - max speed is 1077 mm/s (in theory)
    wheel_rpm = np.sqrt(2) * 60 * 1/(np.pi*d_wheel) * speed
    wheel_rpm = np.clip(wheel_rpm, -130, 130)

    if direction == "forward":
        wheel_rot = np.array([1, 1, 1, 1])
    elif direction == "backward":
        wheel_rot = np.array([-1, -1, -1, -1])
    elif direction == "left":
        wheel_rot = np.array([-1, 1, 1, -1])
    elif direction == "right":
        wheel_rot = np.array([1, -1, -1, 1])

    rpm_k1 = wheel_rpm * wheel_rot

    return rpm_k1

# mecanum wheel rotation (CW/CCW)
def mec_rotate(omega, direction):
    d_wheel = 97  # [mm]

    # TO-DO: actually calculate wheel RPM for given omega
    # convert vehicle speed [mm/sec] to wheel RPM [rev/min]
    # wheel_rpm = np.sqrt(2) * 60 * 1/(np.pi*d_wheel) * speed
    # wheel_rpm = np.clip(wheel_rpm, -300, 300)

    wheel_rpm = 50

    # speed input in rad/s, convert to wheel RPM
    if direction == "CW":
        wheel_rot = [1, -1, 1, -1]
    elif direction == "CCW":
        wheel_rot = [-1, 1, -1, 1]

    rpm_k1 = wheel_rpm * wheel_rot

    return rpm_k1

# converts input vector to byte package for sending to Mega
def u_to_bytes(act_k):
    max_rpm = 130

    p_m1_b = int(255/max_rpm * act_k[0]).to_bytes(2, "little", signed=True)     # TO-DO: would like to move RPM scaling onto Arduino
    p_m2_b = int(255/max_rpm * act_k[1]).to_bytes(2, "little", signed=True)
    p_m3_b = int(255/max_rpm * act_k[2]).to_bytes(2, "little", signed=True)
    p_m4_b = int(255/max_rpm * act_k[3]).to_bytes(2, "little", signed=True)

    o_sol_b = int(act_k[4])

    act_kb = [0xaa, p_m1_b[0], p_m1_b[1], p_m2_b[0], p_m2_b[1], p_m3_b[0], p_m3_b[1], p_m4_b[0], p_m4_b[1], o_sol_b]
    return act_kb

# converts byte package received from Mega into sensor observations
def bytes_to_o(obs_kb):
    obs_kba = bytearray(obs_kb)

    v_frame_offset = 315/2
    w_frame_offset = 207/2
    # NOTE: y measurement is from top, but not using

    # split byte array into values
    u1 = int.from_bytes(obs_kba[0:2], "little", signed=True)
    u2 = int.from_bytes(obs_kba[2:4], "little", signed=True)

    # NOTE: when object not detected, coordinates will show as [-157.5, -103.5]
    v_ball = int.from_bytes(obs_kba[4:6], "little", signed=True) - v_frame_offset
    w_ball = int.from_bytes(obs_kba[6:8], "little", signed=True) - w_frame_offset

    v_left_post = int.from_bytes(obs_kba[8:10], "little", signed=True) - v_frame_offset
    w_left_post = int.from_bytes(obs_kba[10:12], "little", signed=True) - w_frame_offset
    v_right_post = int.from_bytes(obs_kba[12:14], "little", signed=True) - v_frame_offset
    w_right_post = int.from_bytes(obs_kba[14:16], "little", signed=True) - w_frame_offset

    v_yellow = int.from_bytes(obs_kba[16:18], "little", signed=True) - v_frame_offset
    w_yellow = int.from_bytes(obs_kba[18:20], "little", signed=True) - w_frame_offset
    v_blue = int.from_bytes(obs_kba[20:22], "little", signed=True) - v_frame_offset
    w_blue = int.from_bytes(obs_kba[22:24], "little", signed=True) - w_frame_offset

    tone = obs_kba[24]

    obs_k = [u1, 
            u2, 
            v_ball, w_ball, 
            v_left_post, w_left_post, 
            v_right_post, w_right_post, 
            v_yellow, w_yellow, 
            v_blue, w_blue, 
            tone]

    for i in [2,4,6,8,10]:
        if obs_k[i] == -v_frame_offset:
            obs_k[i] = 0

    for i in [3,5,7,9,11]:
        if obs_k[i] == -w_frame_offset:
            obs_k[i] = 0

    return obs_k