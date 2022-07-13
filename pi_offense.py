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
#       - whistle: whistle has been blown [bool]
#       - t_w: unix time when whistle was blown [time object]

#   - m (operating mode):
#       - m = 0: acquire env (w-30 sec) -> acquire objects w/ CV, choose target (@ point 1)
#       - m = 1: set angle (w+5 sec) -> move along arc to align shot angle (point 1->2)
#           - move horizontal until ball out of center by some threshold, rotate until realigned at center
#       - m = 2: set distance (w+10 sec) -> move in to contact distance with ball (point 2->3)
#       - m = 3: hold for shot (w+15 sec) -> wait for spot to clear, take shot (@ point 3)

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
#       - v_mg: green marker horizontal position [px] (camera frame)
#       - w_mg: green marker vertical position [px] (camera frame)
#       - v_mp: pink marker horizontal position [px] (camera frame)
#       - w_mp: pink marker vertical position [px] (camera frame)
#       - tone: whistle heard [bool]

# Q: should create data classes for x, u, o? fields have different data types, so difficult to hold in single array

# TO-DO: figure out serial transfer first, then worry about data strctures

def offense():
    # define constant parameters
    Dt = 0.5    # [s], period of main loop

    # initialize I2C bus
    i2c = SMBus(1)

    # initialize vectors
    mode_k = 0
    shoot_k1 = 0

    k_step = 0
    while k_step < 1:
        # 1) send actuator command
        act_k = [100, 100, 100, 100, 0]
        act_kb = u_to_bytes(act_k)
        i2c.write_block_data(mega_addr, mega_reg, act_kb)

        # 2) receive sensor data
        obs_kb = i2c.read_i2c_block_data(mega_addr, mega_reg, 3)
        obs_k = bytes_to_o(obs_kb)

        # 3) enter mode operate
        if mode_k == 0:
            mode_k1, rpm_k1, theta_targ = operate_m0(obs_k)
        elif mode_k == 1:
            mode_k1, rpm_k1 = operate_m1(obs_k, theta_targ)
        elif mode_k == 2:
            mode_k1, rpm_k1 = operate_m2(obs_k)
        elif mode_k == 3:
            mode_k1, rpm_k1, shoot_k1 = operate_m3(obs_k)

        # TO-DO: see if this concatenates properly
        act_k1 = [rpm_k1, shoot_k1]

        mode_k = copy.deepcopy(mode_k1)
        act_k = copy.deepcopy(act_k1)
        k_step += 1

        time.sleep(Dt)
        
    return

# mode 0: acquire environment
def operate_m0(obs_k):
    mode_k1 = 0

    # check if whistle detected
    if obs_k[-1] == 1:
        t_w = time.now()

    # process CV data (if needed)
    # ...

    # calculate best shot
    theta_targ = 0

    # ACTION: no action in this mode
    rpm_k1 = [0, 0, 0, 0]

    # MODE: if whistle delay has passed, move on to next mode
    if time.now() - t_w >= 0:
        mode_k1 = 1

    return mode_k1, rpm_k1, theta_targ

# TO-DO: make sure "theta" is clearly defined as shot angle, not heading angle (although kinda the same thing)
# mode 1: set angle
def operate_m1(obs_k, theta_targ):
    mode_k1 = 1

    cent_tol = 10   # TO-DO: set this to some real value

    # TO-DO: need to figure which calculations go where
    #   - need a separate function for calculating theta from CV
    # calculate current theta from CV observations
    v_ball = 0
    v_left_post = -100
    v_right_post = 100

    theta_k = 10

    # ACTION: generate action needed (either rotating or translating)
    if abs(v_ball) > cent_tol:
        # need to rotate to recenter the ball
        rpm_k1 = control_m1_rotate(v_ball)
    else:
        # ball near center, need to translate towards desired angle
        rpm_k1 = control_m1_translate(v_ball, v_left_post, v_right_post, theta_targ)

    # MODE: if aligned with target, move on to next mode
    if abs(theta_k - theta_targ) <= 0.5 and abs(v_ball) <= cent_tol:
        mode_k1 = 2

    return mode_k1, rpm_k1

# (!): open-loop motion is difficult, since it relies on past actions
#   - able to calculate a dead-reckoning state and pass that through?

# mode 2: set distance
def operate_m2(obs_k):
    mode_k1 = 2

    rpm_k1 = 50*np.ones(4)
    
    return mode_k1, rpm_k1

# mode 3: hold for shot
def operate_m3(obs_k):
    mode_k1 = 3

    rpm_k1 = np.zeros(4)

    shoot_k1 = True
    
    return mode_k1, rpm_k1, shoot_k1


def control_m1_rotate(obs_k):
    rpm_k1 = np.zeros(4)

    return rpm_k1

def control_m1_translate(obs_k, theta_targ):
    rpm_k1 = np.zeros(4)

    return rpm_k1

# estimates state vector from sensor observations
def estimate_state(obs_k):
    x_k = 0
    return x_k

# updates operating mode based on state and observation
def update_mode(x_k, obs_k):
    m_k = 0
    return m_k

# converts input vector to byte package for sending to Mega
def u_to_bytes(act_k):
    p_m1_b = int(255/300 * act_k[0]).to_bytes(2, "little", signed=True)
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


# 3) estimate state
# x_k = estimate_state(obs_k)

# NOTE: might combine #3-6 since each mode will be so different

# 4) update mode
# m_k = update_mode(x_k, obs_k)

# 5) update guidance

# 6) calculate input