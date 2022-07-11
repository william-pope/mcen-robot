#!/usr/bin/env python3

from smbus import SMBus
import time

mega_addr = 0x1a
mega_reg = 0x00

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

    # initialize I2C bus
    i2c = SMBus(1)

    # initialize vectors
    x_k = [0.0]

    k_step = 0
    while k_step < 1:
        # 1) send actuator command
        u_k = [100, 100, 100, 100, 0]
        u_kb = u_to_bytes(u_k)
        i2c.write_block_data(mega_addr, mega_reg, u_kb)

        # 2) receive sensor data
        o_kb = i2c.read_i2c_block_data(mega_addr, mega_reg, 3)
        o_k = bytes_to_o(o_kb)

        # 3) estimate state
        # x_k = estimate_state(o_k)

        # NOTE: might combine #4-6 or #5-6 to keep each mode's operation closer together
        # 4) update mode
        # m_k = update_mode(x_k, o_k)

        # 5) update guidance

        # 6) calculate input

        time.sleep(1)
        k_step += 1

    return

# estimates state vector from sensor observations
def estimate_state(o_k):
    x_k = 0
    return x_k

# updates operating mode based on state and observation
def update_mode(x_k, o_k):
    m_k = 0
    return m_k

# converts input vector to byte package for sending to Mega
def u_to_bytes(u_k):
    p_m1_b = int(255/300 * u_k[0]).to_bytes(2, "little", signed=True)
    p_m2_b = int(255/300 * u_k[1]).to_bytes(2, "little", signed=True)
    p_m3_b = int(255/300 * u_k[2]).to_bytes(2, "little", signed=True)
    p_m4_b = int(255/300 * u_k[3]).to_bytes(2, "little", signed=True)

    o_sol_b = u_k[4]

    u_kb = [p_m1_b[0], p_m1_b[1], p_m2_b[0], p_m2_b[1], p_m3_b[0], p_m3_b[1], p_m4_b[0], p_m4_b[1], o_sol_b]
    return u_kb

# converts byte package received from Mega into sensor observations
def bytes_to_o(o_kb):
    o_k = 0
    return o_k

if __name__ == "__main__":
    offense()