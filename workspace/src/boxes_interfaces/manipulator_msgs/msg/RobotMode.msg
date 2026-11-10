# /boxes_prefix/mode. The real system sends the CiA-402 mode of the drives here:
# 1 POSITION, 3 VELOCITY, 6 HOMING, 7 INTERPOLATED_POSITION, 10 DISABLE.
# The constants of the original message are unknown, see future_works.md.

int8 mode
