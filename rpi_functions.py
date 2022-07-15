#!/usr/bin/env python3

from smbus2 import SMBus
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

# use range sensor measurements and vehicle geometry to calculate relative position of ball
def estimate_m3_rel(obs_k):

    x_rel_ball = [10, 10]

    return x_rel_ball

def control_m3_rel(x_rel_ball, x_rel_targ):
    rpm_k1 = np.zeros(4)

    return rpm_k1 

def control_m6_translate(v_ball):
    if v_ball >= 0:
        direction = "right"
    elif v_ball < 0:
        direction = "left"

    rpm_k1 = mec_translate(200, direction)

    return rpm_k1

# mecanum wheel translation (cardinal directions)
def mec_translate(speed, direction):
    d_wheel = 97  # [mm]

    # convert vehicle speed [mm/sec] to wheel RPM [rev/min]
    #   - max speed is 1077 mm/s (in theory)
    wheel_rpm = np.sqrt(2) * 60 * 1/(np.pi*d_wheel) * speed
    wheel_rpm = np.clip(wheel_rpm, -300, 300)

    if direction == "forward":
        wheel_rot = [1, 1, 1, 1]
    elif direction == "backward":
        wheel_rot = [-1, -1, -1, -1]
    elif direction == "left":
        wheel_rot = [-1, -1, 1, 1]
    elif direction == "right":
        wheel_rot = [1, 1, -1, -1]

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
    p_m1_b = int(255/300 * act_k[0]).to_bytes(2, "little", signed=True)     # TO-DO: would like to move RPM scaling onto Arduino
    p_m2_b = int(255/300 * act_k[1]).to_bytes(2, "little", signed=True)
    p_m3_b = int(255/300 * act_k[2]).to_bytes(2, "little", signed=True)
    p_m4_b = int(255/300 * act_k[3]).to_bytes(2, "little", signed=True)

    o_sol_b = int(act_k[4])

    act_kb = [p_m1_b[0], p_m1_b[1], p_m2_b[0], p_m2_b[1], p_m3_b[0], p_m3_b[1], p_m4_b[0], p_m4_b[1], o_sol_b]
    return act_kb

# converts byte package received from Mega into sensor observations
def bytes_to_o(obs_kb):
    obs_kba = bytearray(obs_kb)

    # split byte array into values
    test = int.from_bytes(obs_kba[0:2], "little", signed=True)

    obs_k = [test]

    return obs_k