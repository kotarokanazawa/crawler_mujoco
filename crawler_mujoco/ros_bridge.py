"""ROS 2 command/state bridge for the native MuJoCo crawler."""

from __future__ import annotations

import threading
import math

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


FLIPPER_JOINTS = (
    "joint_left_front", "joint_left_rear",
    "joint_right_front", "joint_right_rear",
)


class CrawlerMujocoRosNode(Node):
    def __init__(self, cfg: dict):
        super().__init__("crawler_mujoco_bridge")
        self.lock = threading.Lock()
        self.linear = 0.0
        self.angular = 0.0
        self.flippers = [0.0] * 4
        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel, 10)
        self.create_subscription(Twist, "/target/cmd_vel", self._cmd_vel, 10)
        self.create_subscription(Float64MultiArray, "/crawler/flipper_commands",
                                 self._flipper_commands, 10)
        self.create_subscription(JointState, "/target/joint_states",
                                 self._target_joint_states, 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.crawler_joint_pub = self.create_publisher(JointState, "/crawler/joint_states", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.crawler_imu_pub = self.create_publisher(Imu, "/crawler/imu", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.rgbd_cfg = cfg.get("sensors", {}).get("rgbd", {})
        self.camera_publishers = {}
        for side in ("front", "rear"):
            prefix = f"/camera/{side}"
            self.camera_publishers[side] = {
                "color": self.create_publisher(Image, f"{prefix}/color/image_raw", 2),
                "depth": self.create_publisher(Image, f"{prefix}/depth/image_raw", 2),
                "depth_gray": self.create_publisher(
                    Image, f"{prefix}/depth/image_grayscale", 2),
                "points": self.create_publisher(PointCloud2, f"{prefix}/points", 2),
                "color_info": self.create_publisher(
                    CameraInfo, f"{prefix}/color/camera_info", 2),
                "depth_info": self.create_publisher(
                    CameraInfo, f"{prefix}/depth/camera_info", 2),
            }
        self.combined_points_pub = self.create_publisher(
            PointCloud2, "/camera/points", 2)
        self.renderer = None
        self.next_camera_time = 0.0
        self.camera_error_reported = False
        self.tf_broadcaster = TransformBroadcaster(self)

    def _camera_info(self, stamp, frame_id: str) -> CameraInfo:
        width = int(self.rgbd_cfg.get("width", 320))
        height = int(self.rgbd_cfg.get("height", 240))
        fovy = math.radians(float(self.rgbd_cfg.get("fovy_deg", 70.0)))
        focal = 0.5 * height / math.tan(fovy / 2)
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = frame_id
        info.width, info.height = width, height
        info.distortion_model = "plumb_bob"
        info.d = [0.0] * 5
        info.k = [focal, 0.0, width / 2, 0.0, focal, height / 2, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [focal, 0.0, width / 2, 0.0,
                  0.0, focal, height / 2, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _depth_grayscale(self, depth: np.ndarray, stamp, frame_id: str) -> Image:
        """Make a display image while preserving metric depth on image_raw."""
        maximum = float(self.rgbd_cfg.get("grayscale_max_depth", 8.0))
        valid = np.isfinite(depth) & (depth > 0.0) & (depth <= maximum)
        gray = np.zeros(depth.shape, dtype=np.uint8)
        gray[valid] = np.clip(
            255.0 * (1.0 - depth[valid] / maximum), 1.0, 255.0
        ).astype(np.uint8)
        message = Image()
        message.header.stamp, message.header.frame_id = stamp, frame_id
        message.height, message.width = gray.shape
        message.encoding, message.is_bigendian = "mono8", False
        message.step, message.data = gray.shape[1], np.ascontiguousarray(gray).tobytes()
        return message

    def _points_in_base(self, depth: np.ndarray, rgb: np.ndarray, side: str) -> np.ndarray:
        """Back-project optical depth and express XYZ in the chassis base_link."""
        height, width = depth.shape
        fovy = math.radians(float(self.rgbd_cfg.get("fovy_deg", 70.0)))
        focal = 0.5 * height / math.tan(fovy / 2.0)
        stride = max(1, int(self.rgbd_cfg.get("pointcloud_stride", 1)))
        rows, columns = np.mgrid[0:height:stride, 0:width:stride]
        z_optical = depth[::stride, ::stride]
        x_optical = (columns - width / 2.0) * z_optical / focal
        y_optical = (rows - height / 2.0) * z_optical / focal

        # Optical (right, down, forward) -> camera link (forward, left, up),
        # followed by the configured downward camera mounting rotation.
        forward, left, up = z_optical, -x_optical, -y_optical
        pitch = math.radians(float(self.rgbd_cfg.get("downward_pitch_deg", 20.0)))
        c, s = math.cos(pitch), math.sin(pitch)
        position = np.asarray(self.rgbd_cfg.get(
            f"{side}_position", [0.275 if side == "front" else -0.275, 0.0, 0.065]
        ), dtype=np.float32)
        if side == "front":
            x_base = c * forward + s * up + position[0]
            y_base = left + position[1]
            z_base = -s * forward + c * up + position[2]
        else:
            x_base = -c * forward - s * up + position[0]
            y_base = -left + position[1]
            z_base = -s * forward + c * up + position[2]

        valid = np.isfinite(z_optical) & (z_optical > 0.0)
        colors = rgb[::stride, ::stride]
        packed_rgb = ((colors[..., 0].astype(np.uint32) << 16) |
                      (colors[..., 1].astype(np.uint32) << 8) |
                      colors[..., 2].astype(np.uint32))
        points = np.empty(int(np.count_nonzero(valid)), dtype=[
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<f4")])
        points["x"], points["y"], points["z"] = (
            x_base[valid], y_base[valid], z_base[valid])
        points["rgb"] = np.ascontiguousarray(packed_rgb[valid]).view(np.float32)
        return points

    @staticmethod
    def _pointcloud_message(points: np.ndarray, stamp) -> PointCloud2:
        message = PointCloud2()
        message.header.stamp, message.header.frame_id = stamp, "base_link"
        message.height, message.width = 1, len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian, message.point_step = False, 16
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = points.tobytes()
        return message

    def publish_cameras(self, model, data, stamp) -> None:
        if not bool(self.rgbd_cfg.get("enabled", True)):
            return
        rate = float(self.rgbd_cfg.get("publish_rate", 10.0))
        if data.time + 1e-12 < self.next_camera_time:
            return
        self.next_camera_time = data.time + 1.0 / max(rate, 1e-6)
        width = int(self.rgbd_cfg.get("width", 320))
        height = int(self.rgbd_cfg.get("height", 240))
        try:
            if self.renderer is None:
                self.renderer = mujoco.Renderer(model, height=height, width=width)
            combined_points = []
            for side in ("front", "rear"):
                camera_name = f"{side}_rgbd"
                frame_id = f"{side}_rgbd_optical_frame"
                self.renderer.disable_depth_rendering()
                self.renderer.update_scene(data, camera=camera_name)
                rgb = np.ascontiguousarray(self.renderer.render())
                self.renderer.enable_depth_rendering()
                self.renderer.update_scene(data, camera=camera_name)
                depth = np.ascontiguousarray(self.renderer.render(), dtype=np.float32)

                color = Image()
                color.header.stamp, color.header.frame_id = stamp, frame_id
                color.height, color.width = height, width
                color.encoding, color.is_bigendian, color.step = "rgb8", False, width * 3
                color.data = rgb.tobytes()
                depth_message = Image()
                depth_message.header.stamp, depth_message.header.frame_id = stamp, frame_id
                depth_message.height, depth_message.width = height, width
                depth_message.encoding, depth_message.is_bigendian = "32FC1", False
                depth_message.step, depth_message.data = width * 4, depth.tobytes()
                depth_gray = self._depth_grayscale(depth, stamp, frame_id)
                points = self._points_in_base(depth, rgb, side)
                points_message = self._pointcloud_message(points, stamp)
                info = self._camera_info(stamp, frame_id)
                publishers = self.camera_publishers[side]
                publishers["color"].publish(color)
                publishers["depth"].publish(depth_message)
                publishers["depth_gray"].publish(depth_gray)
                publishers["points"].publish(points_message)
                publishers["color_info"].publish(info)
                publishers["depth_info"].publish(info)
                combined_points.append(points)
            if combined_points:
                self.combined_points_pub.publish(self._pointcloud_message(
                    np.concatenate(combined_points), stamp))
        except Exception as error:
            if not self.camera_error_reported:
                self.get_logger().error(f"RGB-D rendering disabled: {error}")
                self.camera_error_reported = True

    def _cmd_vel(self, message: Twist) -> None:
        with self.lock:
            self.linear = float(message.linear.x)
            self.angular = float(message.angular.z)

    def _flipper_commands(self, message: Float64MultiArray) -> None:
        if len(message.data) != 4:
            self.get_logger().warning(
                "/crawler/flipper_commands requires 4 values: left_front, left_rear, right_front, right_rear")
            return
        with self.lock:
            self.flippers = [float(value) for value in message.data]

    def _target_joint_states(self, message: JointState) -> None:
        by_name = dict(zip(message.name, message.position))
        with self.lock:
            for index, name in enumerate(FLIPPER_JOINTS):
                if name in by_name:
                    self.flippers[index] = float(by_name[name])

    def commands(self):
        with self.lock:
            return self.linear, self.angular, tuple(self.flippers)

    def publish_state(self, model, data) -> None:
        seconds = int(data.time)
        nanoseconds = int((data.time - seconds) * 1_000_000_000)
        stamp = self.get_clock().now().to_msg()
        stamp.sec, stamp.nanosec = seconds, nanoseconds

        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)

        joints = JointState()
        joints.header.stamp = stamp
        for name in FLIPPER_JOINTS:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            joints.name.append(name)
            joints.position.append(float(data.qpos[model.jnt_qposadr[joint_id]]))
            joints.velocity.append(float(data.qvel[model.jnt_dofadr[joint_id]]))
        self.joint_pub.publish(joints)
        self.crawler_joint_pub.publish(joints)

        position = data.sensor("body_position").data
        quaternion = data.sensor("body_orientation").data  # MuJoCo w, x, y, z
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = map(float, position)
        odom.pose.pose.orientation.w = float(quaternion[0])
        odom.pose.pose.orientation.x = float(quaternion[1])
        odom.pose.pose.orientation.y = float(quaternion[2])
        odom.pose.pose.orientation.z = float(quaternion[3])
        odom.twist.twist.linear.x = float(data.qvel[0])
        odom.twist.twist.linear.y = float(data.qvel[1])
        odom.twist.twist.linear.z = float(data.qvel[2])
        odom.twist.twist.angular.x = float(data.qvel[3])
        odom.twist.twist.angular.y = float(data.qvel[4])
        odom.twist.twist.angular.z = float(data.qvel[5])
        self.odom_pub.publish(odom)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = "imu_link"
        imu.orientation.w = float(quaternion[0])
        imu.orientation.x = float(quaternion[1])
        imu.orientation.y = float(quaternion[2])
        imu.orientation.z = float(quaternion[3])
        gyro = data.sensor("body_gyro").data
        acceleration = data.sensor("body_acceleration").data
        imu.angular_velocity.x = float(gyro[0])
        imu.angular_velocity.y = float(gyro[1])
        imu.angular_velocity.z = float(gyro[2])
        imu.linear_acceleration.x = float(acceleration[0])
        imu.linear_acceleration.y = float(acceleration[1])
        imu.linear_acceleration.z = float(acceleration[2])
        # Zero covariance denotes ideal simulated ground truth measurements.
        imu.orientation_covariance = [0.0] * 9
        imu.angular_velocity_covariance = [0.0] * 9
        imu.linear_acceleration_covariance = [0.0] * 9
        self.imu_pub.publish(imu)
        self.crawler_imu_pub.publish(imu)
        self.publish_cameras(model, data, stamp)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(position[0])
        transform.transform.translation.y = float(position[1])
        transform.transform.translation.z = float(position[2])
        transform.transform.rotation.w = float(quaternion[0])
        transform.transform.rotation.x = float(quaternion[1])
        transform.transform.rotation.y = float(quaternion[2])
        transform.transform.rotation.z = float(quaternion[3])
        self.tf_broadcaster.sendTransform(transform)


class Ros2Bridge:
    def __init__(self, cfg: dict):
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        self.node = CrawlerMujocoRosNode(cfg)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def commands(self):
        return self.node.commands()

    def publish_state(self, model, data) -> None:
        self.node.publish_state(model, data)

    def close(self) -> None:
        if self.node.renderer is not None:
            self.node.renderer.close()
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
