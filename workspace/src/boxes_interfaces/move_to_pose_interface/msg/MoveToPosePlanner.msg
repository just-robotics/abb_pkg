# Planner and speed of a plan request; part of the goal of every plan action of /move_to_pose.

uint16 OMPL = 0
uint16 CARTESIAN = 1
uint16 CHOMP = 2
uint16 STOMP = 3
uint16 PILZ_PTP = 4        # point-to-point
uint16 PILZ_LIN = 5        # straight line of the TCP
uint16 PILZ_CIRC = 6       # arc

uint16 planner_id
float64 max_velocity_scaling_factor       # 0..1
float64 max_acceleration_scaling_factor   # 0..1
