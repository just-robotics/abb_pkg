#!/usr/bin/env python3

import math
import struct
import random
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, Buffer, TransformListener


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles (rad) to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return (x, y, z, w)


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> List[List[float]]:
    """Quaternion (x,y,z,w) to 3x3 rotation matrix (row-major list of lists)."""
    return [
        [
            1 - 2 * (qy * qy + qz * qz),
            2 * (qx * qy - qz * qw),
            2 * (qx * qz + qy * qw),
        ],
        [
            2 * (qx * qy + qz * qw),
            1 - 2 * (qx * qx + qz * qz),
            2 * (qy * qz - qx * qw),
        ],
        [
            2 * (qx * qz - qy * qw),
            2 * (qy * qz + qx * qw),
            1 - 2 * (qx * qx + qy * qy),
        ],
    ]


def transform_point(
    x: float, y: float, z: float,
    R: List[List[float]],
    tx: float, ty: float, tz: float,
) -> Tuple[float, float, float]:
    """Transform point (x,y,z) by rotation R and translation (tx,ty,tz): p_out = R @ p + t."""
    return (
        R[0][0] * x + R[0][1] * y + R[0][2] * z + tx,
        R[1][0] * x + R[1][1] * y + R[1][2] * z + ty,
        R[2][0] * x + R[2][1] * y + R[2][2] * z + tz,
    )


class PointCloudEmulatorNode(Node):
    """Эмулирует облако точек 3D лидара мобильного робота и публикует TF для стыковки с миром манипулятора."""

    def __init__(self):
        super().__init__("point_cloud_emulator")

        self._declare_parameters()
        self._load_parameters()

        self._tf_broadcaster = StaticTransformBroadcaster(self)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publish_static_tf()

        self._pub = self.create_publisher(
            PointCloud2,
            self.point_cloud_topic,
            10,
        )
        self._timer = self.create_timer(1.0 / self.publish_rate_hz, self._timer_callback)

        self.get_logger().info(
            f"Point cloud emulator: topic={self.point_cloud_topic}, "
            f"frame={self.lidar_frame}, rate={self.publish_rate_hz} Hz"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("manipulator_base_frame", "base_link")
        self.declare_parameter("robot_base_frame", "robot_base_link")
        self.declare_parameter("lidar_frame", "lidar_link")

        self.declare_parameter("robot_base_in_world.x", 1.5)
        self.declare_parameter("robot_base_in_world.y", 0.0)
        self.declare_parameter("robot_base_in_world.z", 0.0)
        self.declare_parameter("robot_base_in_world.roll", 0.0)
        self.declare_parameter("robot_base_in_world.pitch", 0.0)
        self.declare_parameter("robot_base_in_world.yaw", 0.0)

        self.declare_parameter("lidar_in_robot_base.x", 0.0)
        self.declare_parameter("lidar_in_robot_base.y", 0.0)
        self.declare_parameter("lidar_in_robot_base.z", 0.3)
        self.declare_parameter("lidar_in_robot_base.roll", 0.0)
        self.declare_parameter("lidar_in_robot_base.pitch", 0.0)
        self.declare_parameter("lidar_in_robot_base.yaw", 0.0)

        self.declare_parameter("point_cloud_topic", "lidar/points")
        self.declare_parameter("publish_rate_hz", 10.0)

        self.declare_parameter("num_points", 20000)
        self.declare_parameter("range_min_m", 0.3)
        self.declare_parameter("range_max_m", 30.0)
        self.declare_parameter("horizontal_fov_deg", 360.0)
        self.declare_parameter("vertical_fov_min_deg", -15.0)
        self.declare_parameter("vertical_fov_max_deg", 15.0)
        self.declare_parameter("range_noise_std", 0.01)
        self.declare_parameter("angle_noise_std", 0.0005)

        self.declare_parameter("floor_z", 0.25)
        self.declare_parameter("floor_x_min", -2.0)
        self.declare_parameter("floor_x_max", 4.0)
        self.declare_parameter("floor_y_min", -2.0)
        self.declare_parameter("floor_y_max", 2.0)
        self.declare_parameter("floor_point_step", 0.02)

        self.declare_parameter("obstacle_spheres", [""])

    def _load_parameters(self) -> None:
        self.world_frame = self.get_parameter("world_frame").value
        self.manipulator_base_frame = self.get_parameter("manipulator_base_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.lidar_frame = self.get_parameter("lidar_frame").value

        def load_pose(prefix: str) -> List[float]:
            return [
                self.get_parameter(f"{prefix}.x").value,
                self.get_parameter(f"{prefix}.y").value,
                self.get_parameter(f"{prefix}.z").value,
                math.radians(self.get_parameter(f"{prefix}.roll").value),
                math.radians(self.get_parameter(f"{prefix}.pitch").value),
                math.radians(self.get_parameter(f"{prefix}.yaw").value),
            ]

        self.robot_base_in_world = load_pose("robot_base_in_world")
        self.lidar_in_robot_base = load_pose("lidar_in_robot_base")

        self.point_cloud_topic = self.get_parameter("point_cloud_topic").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        self.num_points = int(self.get_parameter("num_points").value)
        self.range_min_m = float(self.get_parameter("range_min_m").value)
        self.range_max_m = float(self.get_parameter("range_max_m").value)
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)
        self.vertical_fov_min_deg = float(self.get_parameter("vertical_fov_min_deg").value)
        self.vertical_fov_max_deg = float(self.get_parameter("vertical_fov_max_deg").value)
        self.range_noise_std = float(self.get_parameter("range_noise_std").value)
        self.angle_noise_std = float(self.get_parameter("angle_noise_std").value)

        self.floor_z = float(self.get_parameter("floor_z").value)
        self.floor_x_min = float(self.get_parameter("floor_x_min").value)
        self.floor_x_max = float(self.get_parameter("floor_x_max").value)
        self.floor_y_min = float(self.get_parameter("floor_y_min").value)
        self.floor_y_max = float(self.get_parameter("floor_y_max").value)
        self.floor_point_step = float(self.get_parameter("floor_point_step").value)

        raw = self.get_parameter("obstacle_spheres").value
        self.obstacle_spheres: List[Tuple[float, float, float, float]] = []
        if isinstance(raw, list):
            for item in raw:
                if not isinstance(item, str):
                    continue
                parts = item.replace(",", " ").split()
                if len(parts) != 4:
                    continue
                try:
                    x, y, z, r = (float(parts[0]), float(parts[1]), float(parts[2]), float(parts[3]))
                    self.obstacle_spheres.append((x, y, z, r))
                except ValueError:
                    continue

    def _make_transform(
        self,
        parent_frame: str,
        child_frame: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _publish_static_tf(self) -> None:
        rb = self.robot_base_in_world
        t_world_robot = self._make_transform(
            self.world_frame,
            self.robot_base_frame,
            rb[0], rb[1], rb[2],
            rb[3], rb[4], rb[5],
        )
        lr = self.lidar_in_robot_base
        t_robot_lidar = self._make_transform(
            self.robot_base_frame,
            self.lidar_frame,
            lr[0], lr[1], lr[2],
            lr[3], lr[4], lr[5],
        )
        self._tf_broadcaster.sendTransform([t_world_robot, t_robot_lidar])
        self.get_logger().info(
            f"TF: {self.world_frame} -> {self.robot_base_frame} -> {self.lidar_frame}"
        )

    def _get_world_to_lidar_transform(self) -> Optional[TransformStamped]:
        try:
            return self._tf_buffer.lookup_transform(
                self.lidar_frame,
                self.world_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return None

    def _transform_world_to_lidar(
        self,
        x: float, y: float, z: float,
        t: TransformStamped,
    ) -> Tuple[float, float, float]:
        q = t.transform.rotation
        R = quaternion_to_rotation_matrix(q.x, q.y, q.z, q.w)
        tx = t.transform.translation.x
        ty = t.transform.translation.y
        tz = t.transform.translation.z
        return transform_point(x, y, z, R, tx, ty, tz)

    def _generate_floor_points(self, t: TransformStamped) -> List[bytes]:
        """Точки пола в world на z=floor_z, преобразованные в lidar."""
        points: List[bytes] = []
        x = self.floor_x_min
        while x <= self.floor_x_max:
            y = self.floor_y_min
            while y <= self.floor_y_max:
                xl, yl, zl = self._transform_world_to_lidar(x, y, self.floor_z, t)
                points.append(struct.pack("fff", xl, yl, zl))
                y += self.floor_point_step
            x += self.floor_point_step
        return points

    def _generate_sphere_points(self, t: TransformStamped) -> List[bytes]:
        """Точки на поверхностях сфер-препятствий (world), преобразованные в lidar."""
        points: List[bytes] = []
        for cx, cy, cz, r in self.obstacle_spheres:
            # Число точек на сфере ~ пропорционально площади
            n = max(100, int(400 * r * r))
            for _ in range(n):
                # Равномерная выборка на сфере (файболовая сетка)
                phi = random.uniform(0, 2 * math.pi)
                u = random.uniform(-1, 1)
                theta = math.acos(u)
                xw = cx + r * math.sin(theta) * math.cos(phi)
                yw = cy + r * math.sin(theta) * math.sin(phi)
                zw = cz + r * math.cos(theta)
                xl, yl, zl = self._transform_world_to_lidar(xw, yw, zw, t)
                points.append(struct.pack("fff", xl, yl, zl))
        return points

    def _generate_point_cloud_data(self, world_to_lidar: Optional[TransformStamped]) -> Tuple[bytes, int]:
        """Генерирует облако только из пола и сфер (без фоновых точек)."""
        points: List[bytes] = []

        if world_to_lidar is not None:
            points.extend(self._generate_floor_points(world_to_lidar))
            points.extend(self._generate_sphere_points(world_to_lidar))

        return b"".join(points), len(points)

    def _timer_callback(self) -> None:
        world_to_lidar = self._get_world_to_lidar_transform()
        data, total_points = self._generate_point_cloud_data(world_to_lidar)
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.lidar_frame
        msg.height = 1
        msg.width = total_points
        msg.is_dense = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = data
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudEmulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
