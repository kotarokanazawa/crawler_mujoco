"""ROS 2 command/state bridge for the native MuJoCo crawler."""

from __future__ import annotations

import threading
import math

import mujoco
import numpy as np
import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, PointCloud2, PointField
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


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
        self.flipper_torque_pub = self.create_publisher(
            Float64MultiArray, "/crawler/flipper_torques", 10)
        self.contact_pub = self.create_publisher(
            Float64MultiArray, "/crawler/contacts", 10)
        self.contact_marker_pub = self.create_publisher(
            MarkerArray, "/crawler/contact_markers", 10)
        self.contact_force_scale = float(
            cfg.get("visualization", {}).get("contact_force_scale", 0.001))
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        imu_cfg = cfg.get("sensors", {}).get("imu", {})
        common_imu_noise = imu_cfg.get("noise", {})
        imu_mounts = imu_cfg.get("frames") or [{
            "site_name": "imu", "sensor_name": "body",
            "frame_id": "imu_link", "topic": "/imu/data",
            "aliases": ["/crawler/imu"],
        }]
        self.imus = []
        if bool(imu_cfg.get("enabled", True)):
            for index, mount in enumerate(imu_mounts):
                topic = str(mount.get("topic", f"/imu/{index}"))
                topics = [topic, *(str(value) for value in mount.get("aliases", []))]
                noise = {**common_imu_noise, **mount.get("noise", {})}
                rng = np.random.default_rng(
                    int(noise.get("seed", 42)) + index)
                gyro_bias_std = math.radians(float(
                    noise.get("gyro_initial_bias_std_dps", 0.05)))
                accel_bias_std = float(
                    noise.get("accel_initial_bias_std_mps2", 0.01))
                self.imus.append({
                    "site_name": str(mount.get("site_name", f"imu_{index}")),
                    "sensor_name": str(mount.get("sensor_name", f"imu_{index}")),
                    "frame_id": str(mount.get("frame_id", f"imu_{index}_link")),
                    "publishers": [self.create_publisher(Imu, value, 10)
                                   for value in topics],
                    "noise": noise,
                    "rng": rng,
                    "gyro_bias": rng.normal(0.0, gyro_bias_std, 3),
                    "accel_bias": rng.normal(0.0, accel_bias_std, 3),
                    "gyro_scale": rng.uniform(-1.0, 1.0, 3) * float(
                        noise.get("gyro_scale_error", 0.005)),
                    "accel_scale": rng.uniform(-1.0, 1.0, 3) * float(
                        noise.get("accel_scale_error", 0.005)),
                    "last_time": None,
                })
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
        self.lidar_cfg = cfg.get("sensors", {}).get("lidar", {})
        mounts = self.lidar_cfg.get("frames") or [{
            "site_name": "lidar", "frame_id": "lidar_link",
            "topic": "/livox/lidar",
        }]
        self.lidars = []
        for index, mount in enumerate(mounts):
            topic = str(mount.get("topic", f"/livox/lidar_{index}"))
            self.lidars.append({
                "site_name": str(mount.get("site_name", f"lidar_{index}")),
                "frame_id": str(mount.get("frame_id", f"lidar_{index}_link")),
                "publisher": self.create_publisher(PointCloud2, topic, 2),
                "sensor": None,
                "next_time": 0.0,
                "config": {**self.lidar_cfg, **mount},
            })
        self.renderer = None
        self.next_camera_time = 0.0
        self.camera_error_reported = False
        self.tf_broadcaster = TransformBroadcaster(self)

    @staticmethod
    def _noisy_imu(configured_imu: dict, gyro, acceleration,
                   simulation_time: float) -> tuple[np.ndarray, np.ndarray, float, float]:
        """Apply a configurable ICM-40609-like stochastic sensor model."""
        noise = configured_imu["noise"]
        gyro = np.asarray(gyro, dtype=np.float64).copy()
        acceleration = np.asarray(acceleration, dtype=np.float64).copy()
        if not bool(noise.get("enabled", True)):
            return gyro, acceleration, 0.0, 0.0
        previous = configured_imu["last_time"]
        default_rate = max(float(noise.get("sample_rate_hz", 30.0)), 1e-6)
        dt = (1.0 / default_rate if previous is None
              else max(simulation_time - previous, 1e-6))
        configured_imu["last_time"] = simulation_time
        rng = configured_imu["rng"]

        gyro_walk = math.radians(float(
            noise.get("gyro_bias_random_walk_dps_sqrt_s", 0.002)))
        accel_walk = float(
            noise.get("accel_bias_random_walk_mps2_sqrt_s", 0.001))
        configured_imu["gyro_bias"] += rng.normal(
            0.0, gyro_walk * math.sqrt(dt), 3)
        configured_imu["accel_bias"] += rng.normal(
            0.0, accel_walk * math.sqrt(dt), 3)

        # Convert noise density to per-sample sigma using Nyquist bandwidth.
        bandwidth = 0.5 / dt
        gyro_sigma = math.radians(float(
            noise.get("gyro_noise_density_dps_sqrt_hz", 0.0045))) * math.sqrt(bandwidth)
        accel_sigma = (float(noise.get(
            "accel_noise_density_g_sqrt_hz", 0.0001)) * 9.80665
                       * math.sqrt(bandwidth))
        gyro = (gyro * (1.0 + configured_imu["gyro_scale"])
                + configured_imu["gyro_bias"]
                + rng.normal(0.0, gyro_sigma, 3))
        acceleration = (acceleration * (1.0 + configured_imu["accel_scale"])
                        + configured_imu["accel_bias"]
                        + rng.normal(0.0, accel_sigma, 3))

        gyro_limit = math.radians(float(noise.get("gyro_full_scale_dps", 2000.0)))
        accel_limit = float(noise.get("accel_full_scale_g", 4.0)) * 9.80665
        gyro = np.clip(gyro, -gyro_limit, gyro_limit)
        acceleration = np.clip(acceleration, -accel_limit, accel_limit)
        if bool(noise.get("quantization_enabled", True)):
            levels = max(2, int(noise.get("adc_levels", 65536)))
            gyro_step = 2.0 * gyro_limit / levels
            accel_step = 2.0 * accel_limit / levels
            gyro = np.round(gyro / gyro_step) * gyro_step
            acceleration = np.round(acceleration / accel_step) * accel_step
        return gyro, acceleration, gyro_sigma, accel_sigma

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

    def publish_lidar(self, model, data, stamp) -> None:
        """Ray-cast and publish an XYZ cloud in the moving lidar frame."""
        if not bool(self.lidar_cfg.get("enabled", False)):
            return
        from .lidar import LivoxMid360Lidar
        for lidar in self.lidars:
            rate = max(float(lidar["config"].get("publish_rate", 10.0)), 1e-6)
            if data.time + 1e-12 < lidar["next_time"]:
                continue
            lidar["next_time"] = data.time + 1.0 / rate
            if lidar["sensor"] is None:
                lidar["sensor"] = LivoxMid360Lidar(
                    model, lidar["config"], lidar["site_name"])
            xyz = lidar["sensor"].scan(data)
            message = PointCloud2()
            message.header.stamp = stamp
            message.header.frame_id = lidar["frame_id"]
            message.height, message.width = 1, len(xyz)
            message.fields = [
                PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
                PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
                PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            ]
            message.is_bigendian, message.point_step = False, 12
            message.row_step = message.point_step * message.width
            message.is_dense = True
            message.data = np.ascontiguousarray(xyz, dtype=np.float32).tobytes()
            lidar["publisher"].publish(message)

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
            actuator_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{name}_position")
            joints.effort.append(
                float(data.actuator_force[actuator_id]) if actuator_id >= 0 else 0.0)
        self.joint_pub.publish(joints)
        self.crawler_joint_pub.publish(joints)
        torques = Float64MultiArray()
        torques.layout.dim.append(self._array_dimension("flippers", 4, 4))
        torques.data = list(joints.effort)
        self.flipper_torque_pub.publish(torques)

        position = data.sensor("body_position").data
        crawler_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_BODY, "crawler")
        world_from_base = np.asarray(
            data.xmat[crawler_id], dtype=np.float64).reshape(3, 3)

        # One row per active contact in map coordinates:
        # [position_x, position_y, position_z, force_x, force_y, force_z].
        # mj_contactForce returns the force in MuJoCo's contact frame, whose
        # axes are stored as rows in contact.frame.
        contacts = Float64MultiArray()
        contacts.layout.dim = []
        force_contact = np.zeros(6, dtype=np.float64)
        for index in range(data.ncon):
            contact = data.contact[index]
            mujoco.mj_contactForce(model, data, index, force_contact)
            frame = np.asarray(contact.frame, dtype=np.float64).reshape(3, 3)
            force_world = frame.T @ force_contact[:3]
            contacts.data.extend(float(value) for value in (
                *contact.pos, *force_world))
        contacts.layout.data_offset = 0
        contacts.layout.dim.append(self._array_dimension(
            "contacts", data.ncon, data.ncon * 6))
        contacts.layout.dim.append(self._array_dimension("fields", 6, 6))
        self.contact_pub.publish(contacts)
        self.contact_marker_pub.publish(self._contact_markers(
            model, data, stamp, position, world_from_base))

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

        for configured_imu in self.imus:
            sensor_name = configured_imu["sensor_name"]
            gyro = data.sensor(f"{sensor_name}_gyro").data
            acceleration = data.sensor(f"{sensor_name}_acceleration").data
            gyro, acceleration, gyro_sigma, accel_sigma = self._noisy_imu(
                configured_imu, gyro, acceleration, float(data.time))
            imu_message = Imu()
            imu_message.header.stamp = stamp
            imu_message.header.frame_id = configured_imu["frame_id"]
            # Six-axis IMUs do not directly measure absolute orientation.
            imu_message.orientation_covariance[0] = -1.0
            imu_message.angular_velocity.x = float(gyro[0])
            imu_message.angular_velocity.y = float(gyro[1])
            imu_message.angular_velocity.z = float(gyro[2])
            imu_message.linear_acceleration.x = float(acceleration[0])
            imu_message.linear_acceleration.y = float(acceleration[1])
            imu_message.linear_acceleration.z = float(acceleration[2])
            imu_message.angular_velocity_covariance = [
                gyro_sigma ** 2, 0.0, 0.0,
                0.0, gyro_sigma ** 2, 0.0,
                0.0, 0.0, gyro_sigma ** 2,
            ]
            imu_message.linear_acceleration_covariance = [
                accel_sigma ** 2, 0.0, 0.0,
                0.0, accel_sigma ** 2, 0.0,
                0.0, 0.0, accel_sigma ** 2,
            ]
            for publisher in configured_imu["publishers"]:
                publisher.publish(imu_message)
        self.publish_cameras(model, data, stamp)
        self.publish_lidar(model, data, stamp)

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

    def _contact_markers(self, model, data, stamp, position,
                         world_from_base: np.ndarray) -> MarkerArray:
        """Represent active contacts as base_link-relative points and arrows."""
        result = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        result.markers.append(clear)

        points = Marker()
        points.header.stamp = stamp
        points.header.frame_id = "base_link"
        points.ns = "contact_points"
        points.id = 0
        points.type = Marker.SPHERE_LIST
        points.action = Marker.ADD
        points.pose.orientation.w = 1.0
        points.scale.x = points.scale.y = points.scale.z = 0.025
        points.color.r, points.color.g, points.color.b, points.color.a = (
            1.0, 0.2, 0.05, 1.0)

        force_contact = np.zeros(6, dtype=np.float64)
        base_from_world = world_from_base.T
        for index in range(data.ncon):
            contact = data.contact[index]
            mujoco.mj_contactForce(model, data, index, force_contact)
            contact_frame = np.asarray(
                contact.frame, dtype=np.float64).reshape(3, 3)
            force_world = contact_frame.T @ force_contact[:3]
            point_base = base_from_world @ (
                np.asarray(contact.pos, dtype=np.float64) - position)
            force_base = base_from_world @ force_world

            start = Point()
            start.x, start.y, start.z = map(float, point_base)
            points.points.append(start)

            end = Point()
            end_vector = point_base + self.contact_force_scale * force_base
            end.x, end.y, end.z = map(float, end_vector)
            arrow = Marker()
            arrow.header.stamp = stamp
            arrow.header.frame_id = "base_link"
            arrow.ns = "contact_forces"
            arrow.id = index
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.points = [start, end]
            arrow.scale.x = 0.012
            arrow.scale.y = 0.025
            arrow.scale.z = 0.035
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (
                0.1, 0.75, 1.0, 0.9)
            result.markers.append(arrow)

        result.markers.append(points)
        return result

    @staticmethod
    def _array_dimension(label: str, size: int, stride: int):
        """Build a MultiArrayDimension without importing another message module."""
        from std_msgs.msg import MultiArrayDimension
        dimension = MultiArrayDimension()
        dimension.label = label
        dimension.size = size
        dimension.stride = stride
        return dimension

    def publish_clock(self, simulation_time: float) -> None:
        seconds = int(simulation_time)
        clock = Clock()
        clock.clock.sec = seconds
        clock.clock.nanosec = int((simulation_time - seconds) * 1_000_000_000)
        self.clock_pub.publish(clock)


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

    def publish_clock(self, simulation_time: float) -> None:
        self.node.publish_clock(simulation_time)

    def close(self) -> None:
        if self.node.renderer is not None:
            self.node.renderer.close()
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
