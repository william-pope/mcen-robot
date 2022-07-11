
# drive hierarchy:
#   RPi: calculate desired velocity/rotation vector towards objective from current state
#       RPi: convert velocity/rotation vector to individual wheel speeds, transmit to Arduino
#           Arduino: convert wheel speeds to I/O commands to motor driver

# assuming instant acceleration for now
# ?: what frames should be used?

# start with translation only
#   would like to be able to move in any arbitrary direction (pretty sure this is possible)
#   let user set velocity, clamp to max if over (since dependent on direction)
#   find max speed that is valid for all directions