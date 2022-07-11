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
    ser = serial.Serial('/dev/ttyACM0', 115200)
    ser.reset_input_buffer()

    # initialize vectors
    x_k = [0.0]

    k_step = 0
    while k_step < 1:
        # 1) send actuator command
        # u_kb = u_to_bytes(u_k)
        u_kb = 0x03
        ser.write(u_kb)

        # 2) receive sensor data
        #   - Pi script waits at ser.read() for new byte in buffer (time set by Mega delay())
        #       - when size=2, waits two cycles to collect both bytes before continuing on, output is 2 bytes
        
        # completed:
        # - able to send multiple bytes from Mega to Pi, come in correct order
        # - able to parse multiple bytes into an int

        # TO-DO:
        #   - send data to Mega, have it respond

        # ISSUE: gets hung up at ser.read()
        #   - does buffer on Pi and/or Mega get filled with wrong thing? (request vs reply)
        #       - looks like Pi possibly printed its own request as a reply
        #   - does message get missed because it gets sent so fast?
        #   - either:
        #       - 1) Mega never gets past Serial.readBytes() -> think this is more likely due to avrdude error
        #       - 2) Pi never gets past ser.read()
        #   - Mega giving "avrdude: stk500v2_ReceiveMessage(): timeout" error
        #       - is timeout occuring on Serial.available()==0 or Serial.readBytes()? feel like it has to be readBytes()
        #   - need to clean up serial port?
        #   - looks like Mega has trouble on Serial.readBytes()
        #   - able to respond to input in the serial monitor
        #   - Mega Serial.println() is properly printing hex values
        #   - make sure there's no assumed default line ending or something like that
        #   - doesn't seem like Serial.read() is taking value properly (but seems to receive something)

        # o_kb = ser.read(size=1)    # waits until n bytes are read from buffer
        # o_int = int.from_bytes(o_kb, "little", signed=False) 

        # print(o_kb)
        # print(o_int)
        # o_k = bytes_to_o(o_kb)

        # 3) estimate state
        # x_k = estimate_state(o_k)

        # NOTE: might combine #4-6 or #5-6 to keep each mode's operation closer together
        # 4) update mode
        # m_k = update_mode(x_k, o_k)

        # 5) update guidance

        # 6) calculate input

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
    u_kb = 0
    return u_kb

# converts byte package received from Mega into sensor observations
def bytes_to_o(o_kb):
    o_k = 0
    return o_k

if __name__ == "__main__":
    offense()