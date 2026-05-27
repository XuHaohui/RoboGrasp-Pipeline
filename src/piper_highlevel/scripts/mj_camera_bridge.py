#!/usr/bin/env python3

import os
os.environ['__GL_THREADED_OPTIMIZATIONS'] = '0'

import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from ament_index_python.packages import get_package_share_directory
import numpy as np
import glfw

# Local modules (installed to same directory)
import mj_physics
import mj_camera


class MuJoCoCameraBridge(Node):

    def __init__(self):
        super().__init__('mj_camera_bridge')

        self._joint_targets = {}
        self._step_count = 0

        # Camera config
        self._cam_lookat = np.array([0.4, 0.0, 0.14])
        self._cam_distance = 1.06
        self._cam_elevation = -90.0
        self._cam_azimuth = 0.0
        self._cam_fovy = 60.0
        self._depth_scale = 1.0

        # ── Load simulation ──
        pkg_desc = get_package_share_directory('piper_description')
        pkg_high = get_package_share_directory('piper_highlevel')
        robot_xml = os.path.join(pkg_desc, 'mujoco_model', 'piper_description.xml')
        world_xml = os.path.join(pkg_high, 'config', 'piper_world.xml')

        self._model, self._sim, self._viewer = mj_physics.load_simulation(
            robot_xml, world_xml, self.get_logger())

        glfw.swap_interval(1)
        self._viewer.cam.lookat[:] = self._cam_lookat
        self._viewer.cam.distance = self._cam_distance
        self._viewer.cam.elevation = self._cam_elevation
        self._viewer.cam.azimuth = self._cam_azimuth
        self._model.vis.global_.fovy = self._cam_fovy

        # ── Camera ──
        self._offscreen = mj_camera.create_offscreen_context(
            self._sim, self.get_logger())

        znear = self._model.vis.map.znear
        zfar = self._model.vis.map.zfar
        self.get_logger().info(
            f'Depth conversion: znear={znear:.4f} zfar={zfar:.4f} fovy={self._cam_fovy:.1f}')

        # ── ROS2 I/O ──
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 1)
        self._rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 1)
        self._depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 1)
        self._ci_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 1)
        self.create_timer(0.01, self._control_loop)

        self.get_logger().info('Camera bridge started: /joint_states → MuJoCo → /camera/*')

    # ── Joint callback ──────────────────────────────

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_targets[name] = pos
        if 'joint7' in self._joint_targets:
            self._joint_targets['joint8'] = -self._joint_targets['joint7']

    # ── Control loop (100 Hz) ───────────────────────

    def _control_loop(self):
        self._step_count += 1

        mj_physics.step_simulation(self._sim, self._joint_targets)
        glfw.make_context_current(self._viewer.window)
        self._viewer.render()

        if self._step_count % 10 == 0:
            self._publish_camera()

    # ── Camera publish (10 Hz) ──────────────────────

    def _publish_camera(self, width=640, height=480):
        stamp = self.get_clock().now().to_msg()

        try:
            rgb, depth_zbuf = mj_camera.render_camera(
                self._offscreen, self._sim,
                self._cam_lookat, self._cam_distance,
                self._cam_elevation, self._cam_azimuth, self._cam_fovy,
                width, height)
        except Exception as e:
            self.get_logger().warn(
                f'Camera render failed: {e}', throttle_duration_sec=5.0)
            return

        znear = self._model.vis.map.znear
        zfar = self._model.vis.map.zfar
        depth_metric = mj_camera.convert_depth(depth_zbuf, znear, zfar)

        if self._depth_scale == 1.0:
            self._depth_scale = mj_camera.calibrate_depth_scale(
                depth_metric, self._cam_distance, width, height)
            self.get_logger().info(f'Depth scale calibrated: {self._depth_scale:.4f}')
        depth_metric *= self._depth_scale

        # Diagnostic
        if self._step_count % 100 == 0:
            center_v, center_u = height // 2, width // 2
            self.get_logger().info(
                f'Camera frame #{self._step_count}: '
                f'RGB min={rgb.min():.0f} max={rgb.max():.0f} '
                f'nonzero_ratio={(rgb > 0).mean():.4f} '
                f'center_z_buf={depth_zbuf[center_v, center_u]:.6f} '
                f'center_depth={depth_metric[center_v, center_u]:.3f}',
                throttle_duration_sec=2.0)

        self._rgb_pub.publish(mj_camera.build_rgb_msg(rgb, stamp, width, height))
        self._depth_pub.publish(mj_camera.build_depth_msg(depth_metric, stamp, width, height))
        self._ci_pub.publish(mj_camera.build_camera_info(stamp, self._cam_fovy, width, height))


def main():
    rclpy.init(args=sys.argv)
    node = MuJoCoCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
