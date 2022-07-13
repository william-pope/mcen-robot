#!/usr/bin/env python3

from smbus import SMBus
import time
import copy
import numpy as np

mega_addr = 0x1a
mega_reg = 0x00

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

# (?): should create data classes for x, u, o? fields have different data types, so difficult to hold in single array

def offense():
    # define constant parameters
    Dt = 0.5    # [s], period of main loop
    # TO-DO: would like to define a dataclass to hold constants in a struct (or just define as globals?)

    # initialize I2C bus
    i2c = SMBus(1)

    # initialize vectors
    mode_k = 0

    rpm_k = np.zeros(4)
    shoot_k = 0
    act_k = [rpm_k, shoot_k]

    k_step = 0
    while k_step < 1:
        # 1) send actuator command
        act_kb = u_to_bytes(act_k)
        i2c.write_block_data(mega_addr, mega_reg, act_kb)

        # 2) receive sensor data
        obs_kb = i2c.read_i2c_block_data(mega_addr, mega_reg, 3)
        obs_k = bytes_to_o(obs_kb)

        # 3) enter mode operate
        shoot_k1 = 0
        if mode_k == 0:
            mode_k1, rpm_k1, theta_targ = operate_m0(obs_k)
        elif mode_k == 1:
            mode_k1, rpm_k1 = operate_m1(obs_k, theta_targ)
        elif mode_k == 2:
            mode_k1, rpm_k1 = operate_m2(obs_k)
        elif mode_k == 2:
            mode_k1, rpm_k1 = operate_m3(obs_k)
        elif mode_k == 4:
            mode_k1, rpm_k1, shoot_k1 = operate_m4(obs_k)

        # TO-DO: see if this concatenates properly
        act_k1 = [rpm_k1, shoot_k1]

        mode_k = copy.deepcopy(mode_k1)
        act_k = copy.deepcopy(act_k1)
        k_step += 1

        time.sleep(Dt)
    
    return

# NOTE: different motions used:
#   - m1: translate sideways based on camera data
#   - m1: rotate in-place based on camera data
#   - m2: translate sideways based on dead-reckoning
#   - m2: translate forward based on dead-reckoning
#   - m3: translate forward/sideways based on range sensor data

# TO-DO: clean up operator functions to be more standardized
#   - act_k1 should be based on mode_k1
#   - mode_k1 is based on obs_k

# TO-DO: make sure if...elif logic works properly, not meeting multiple criteria

# mode 0: acquire environment
def operate_m0(obs_k):
    mode_k1 = 0

    # OBS: check if whistle detected
    if obs_k[-1] == 1:
        t_w = time.now()

    # MODE: if whistle delay has passed, move on to next mode
    if time.now() - t_w >= 0:
        mode_k1 = 1
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # process CV data (if needed)
    # ...

    # calculate best shot
    theta_targ = 0

    # ACT: no action in this mode
    rpm_k1 = np.zeros(4)

    return mode_k1, rpm_k1, theta_targ

# TO-DO: make sure "theta" is clearly defined as shot angle, not heading angle (although kinda the same thing)
# mode 1: set angle
def operate_m1(obs_k, theta_targ):
    mode_k1 = 1

    cent_tol = 10   # TO-DO: set this to some real value

    # OBS: calculate current theta from CV observations
    # v_ball = obs_k[6]
    # v_left_post = obs_k[8]
    # v_right_post = obs_k[10]
    v_ball = 0
    v_left_post = -100
    v_right_post = 100

    theta_k = 10

    # (?): should actions be based on a calculated theta_k or on v_posts directly?

    # MODE: if aligned with target, move on to next mode
    if abs(theta_k - theta_targ) <= 0.5 and abs(v_ball) <= cent_tol:
        mode_k1 = 2
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # ACT: generate action needed (either rotating or translating)
    if abs(v_ball) > cent_tol:
        # need to rotate to recenter the ball
        rpm_k1 = control_m1_rotate(v_ball)
    else:
        # ball near center, need to translate towards desired angle
        rpm_k1 = control_m1_translate(v_ball, v_left_post, v_right_post, theta_targ)

    return mode_k1, rpm_k1

# mode 2: set distance (rough)
def operate_m2(obs_k, x_dr_k):
    mode_k1 = 2

    x_ref = [30, 20]

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
    x_rel_targ = [8.1, 5.6]
    x_rel_tol = 1

    # OBS: calculate relative x-y position from range sensors data
    x_rel_ball = [12, 8]

    # MODE: if at target position, move on to next mode
    if abs(x_rel_ball[0] - x_rel_targ[0]) <= x_rel_tol and abs(x_rel_ball[1] - x_rel_targ[1]) <= x_rel_tol:
        mode_k1 = 4
        rpm_k1 = np.zeros(4)
        return mode_k1, rpm_k1

    # ACT: select action based on relative position
    rpm_k1 = control_m3_rel(x_rel_ball, x_rel_targ)

    return mode_k1, rpm_k1

# mode 4: hold for shot
def operate_m4(obs_k):
    mode_k1 = 4

    rpm_k1 = np.zeros(4)

    shoot_k1 = True
    
    return mode_k1, rpm_k1, shoot_k1

def control_m1_translate(v_ball, v_left_post, v_right_post, theta_targ):
    rpm_k1 = np.zeros(4)

    return rpm_k1

def control_m1_rotate(v_ball):
    rpm_k1 = np.zeros(4)

    return rpm_k1   

def control_m2_left(x_dr_k, x_ref):
    rpm_k1 = np.zeros(4)    # TO-DO: sideways to left, scale speed

    return rpm_k1 

def control_m2_fwd(x_dr_k, x_ref):
    rpm_k1 = np.zeros(4)    # TO-DO: forward, scale speed

    return rpm_k1 

def control_m3_rel(x_rel_ball, x_rel_targ):
    rpm_k1 = np.zeros(4)

    return rpm_k1 

# converts input vector to byte package for sending to Mega
def u_to_bytes(act_k):
    p_m1_b = int(255/300 * act_k[0]).to_bytes(2, "little", signed=True)     # TO-DO: would like to move RPM scaling onto Arduino
    p_m2_b = int(255/300 * act_k[1]).to_bytes(2, "little", signed=True)
    p_m3_b = int(255/300 * act_k[2]).to_bytes(2, "little", signed=True)
    p_m4_b = int(255/300 * act_k[3]).to_bytes(2, "little", signed=True)

    o_sol_b = act_k[4]

    act_kb = [p_m1_b[0], p_m1_b[1], p_m2_b[0], p_m2_b[1], p_m3_b[0], p_m3_b[1], p_m4_b[0], p_m4_b[1], o_sol_b]
    return act_kb

# TO-DO:
# converts byte package received from Mega into sensor observations
def bytes_to_o(obs_kb):
    obs_k = 0
    return obs_k

if __name__ == "__main__":
    offense()