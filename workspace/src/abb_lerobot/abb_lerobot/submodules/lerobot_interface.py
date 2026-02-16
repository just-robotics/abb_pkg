# # from lerobot.cameras import make_cameras_from_configs
# # from lerobot.motors import Motor, MotorNormMode
# # from lerobot.motors.feetech import FeetechMotorsBus
# # from lerobot.robots import Robot

# from rclpy import Node

# class MyCoolRobot(Robot):
#     config_class = MyCoolRobotConfig
#     name = "my_cool_robot"

#     def __init__(self, config: MyCoolRobotConfig):
#         super().__init__(config)
#         self.bus = FeetechMotorsBus(
#             port=self.config.port,
#             motors={
#                 "joint_1": Motor(1, "sts3250", MotorNormMode.RANGE_M100_100),
#                 "joint_2": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
#                 "joint_3": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
#                 "joint_4": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
#                 "joint_5": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
#             },
#             calibration=self.calibration,
#         )
#         self.cameras = make_cameras_from_configs(config.cameras)