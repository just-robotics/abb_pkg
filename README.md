EGM <-> robot:
```bash
ros2 launch abb_pkg egm_rviz.launch.py
```

send pose from abb_pkg [x, y, z, r, p, y]:
```bash
./send_pose.sh 1.0 -0.5 0.8 180 0 -90
```

lerobot node:
```bash
ros2 launch abb_lerobot lerobot_node.launch.py
```

record:
```bash
ros2 launch abb_lerobot record_dataset.launch.py
```