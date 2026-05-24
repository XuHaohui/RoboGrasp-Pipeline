#!/usr/bin/env python3

import os
os.environ['__GL_THREADED_OPTIMIZATIONS'] = '0'

import sys
import tempfile

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image, CameraInfo
from ament_index_python.packages import get_package_share_directory
import numpy as np

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV = True
except ImportError:
    HAS_CV = False

import glfw
import mujoco_py
from mujoco_py import MjSim, MjViewer, MjRenderContextOffscreen


class MuJoCoCameraBridge(Node):
    """MuJoCo simulation node with joint control and camera rendering."""

    def __init__(self):
        super().__init__('mj_camera_bridge')

        self._bridge = CvBridge() if HAS_CV else None
        self._joint_targets = {}
        self._step_count = 0
        self._tolerance = 0.05

        # Camera config — editable, uses viewer API (compatible with MuJoCo 2.1)
        self._cam_lookat = np.array([0.4, 0.0, 0.14])
        self._cam_distance = 1.06
        self._cam_elevation = -90.0
        self._cam_azimuth = 0.0
        self._cam_fovy = 60.0

        # 1. Load robot model from piper_description (untouched)
        pkg_desc = get_package_share_directory('piper_description')
        robot_xml_path = os.path.join(
            pkg_desc, 'mujoco_model', 'piper_description.xml')

        # 2. Load world additions from piper_highlevel
        pkg_high = get_package_share_directory('piper_highlevel')
        world_xml_path = os.path.join(pkg_high, 'config', 'piper_world.xml')

        # 3. Merge XMLs
        merged_xml = self._merge_xml(robot_xml_path, world_xml_path)

        # 4. Load MuJoCo model (chdir so mesh relative paths resolve)
        saved_cwd = os.getcwd()
        os.chdir(os.path.dirname(robot_xml_path))
        try:
            fd, tmp_path = tempfile.mkstemp(
                suffix='.xml', dir=os.path.dirname(robot_xml_path))
            with os.fdopen(fd, 'w') as f:
                f.write(merged_xml)
            self._model = mujoco_py.load_model_from_path(tmp_path)
            os.unlink(tmp_path)
        finally:
            os.chdir(saved_cwd)

        self._sim = MjSim(self._model)
        self._viewer = MjViewer(self._sim)
        self._viewer.render()
        glfw.swap_interval(1)

        # Set initial camera pose on viewer
        self._viewer.cam.lookat[:] = self._cam_lookat
        self._viewer.cam.distance = self._cam_distance
        self._viewer.cam.elevation = self._cam_elevation
        self._viewer.cam.azimuth = self._cam_azimuth
        self._sim.model.vis.global_.fovy = self._cam_fovy

        # ─── KEY FIX: Explicit offscreen context isolated from viewer ───
        # device_id=-1 → OSMesa (software rasterizer, no GPU context conflict with MjViewer)
        # device_id=0  → GLFW fallback only if OSMesa is unavailable
        self._offscreen = self._create_offscreen_context()

        self.get_logger().info(
            f'MuJoCo world loaded with {self._model.nbody} bodies')

        # 5. Subscribe to /joint_states
        self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 1)

        # 6. Camera publishers
        self._rgb_pub = self.create_publisher(
            Image, '/camera/color/image_raw', 1)
        self._depth_pub = self.create_publisher(
            Image, '/camera/depth/image_raw', 1)
        self._ci_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info', 1)

        # 7. Control loop (100 Hz)
        self._timer = self.create_timer(0.01, self._control_loop)

        self.get_logger().info(
            'Camera bridge started: /joint_states → MuJoCo → /camera/*')

    # ────────────────────────────────────────────
    #  Offscreen render context (isolated from MjViewer's GLFW context)
    # ────────────────────────────────────────────

    def _create_offscreen_context(self):
        """Create an offscreen render context.

        Priority: OSMesa (-1) → pure software, zero conflict with MjViewer.
        Fallback: GLFW device 0 if OSMesa is unavailable.
        """
        for device_id, label in [(-1, 'OSMesa'), (0, 'GLFW')]:
            try:
                ctx = MjRenderContextOffscreen(self._sim, device_id)
                self.get_logger().info(
                    f'Offscreen render context ({label}) created successfully')
                return ctx
            except Exception as e:
                self.get_logger().warn(
                    f'Offscreen context ({label}) failed: {e}')
        raise RuntimeError(
            'Failed to create any offscreen MuJoCo render context. '
            'Install OSMesa:  apt install libosmesa6-dev')

    # ────────────────────────────────────────────
    #  XML merge
    # ────────────────────────────────────────────

    @staticmethod
    def _merge_xml(robot_path, world_path):
        with open(robot_path, 'r') as f:
            xml = f.read()
        with open(world_path, 'r') as f:
            world_additions = f.read()

        # Insert world additions before </worldbody>
        xml = xml.replace('</worldbody>', world_additions + '\n</worldbody>')

        return xml

    # ────────────────────────────────────────────
    #  Joint state callback
    # ────────────────────────────────────────────

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_targets[name] = pos
        # Mirror joint8: always = -joint7 (gripper finger)
        if 'joint7' in self._joint_targets:
            self._joint_targets['joint8'] = \
                -self._joint_targets['joint7']

    # ────────────────────────────────────────────
    #  Control loop (100 Hz)
    # ────────────────────────────────────────────

    def _control_loop(self):
        self._step_count += 1

        # Write actuator targets
        for joint_name, target in self._joint_targets.items():
            if joint_name in self._sim.model.joint_names:
                try:
                    actuator_id = self._sim.model.actuator_name2id(joint_name)
                    self._sim.data.ctrl[actuator_id] = target
                except Exception:
                    pass

        self._sim.step()

        glfw.make_context_current(self._viewer.window)
        self._viewer.render()

        if self._step_count % 10 == 0:
            self._publish_camera()

    # ────────────────────────────────────────────
    #  Camera image publishing (offscreen FBO, isolated from MjViewer)
    # ────────────────────────────────────────────

    def _publish_camera(self):
        width, height = 640, 480
        stamp = self.get_clock().now().to_msg()

        # Configure offscreen camera from stored parameters.
        # sim.data does not expose an MjvCamera in this mujoco_py version —
        # configure ctx.cam directly.
        ctx = self._offscreen
        ctx.cam.lookat[:] = self._cam_lookat
        ctx.cam.distance = self._cam_distance
        ctx.cam.elevation = self._cam_elevation
        ctx.cam.azimuth = self._cam_azimuth

        # Configure fovy ───
        saved_fovy = self._sim.model.vis.global_.fovy
        self._sim.model.vis.global_.fovy = self._cam_fovy

        # Render + read pixels via the offscreen context directly.
        # Using ctx.render() + ctx.read_pixels() avoids the lazy-init path in
        # sim.render() which can create a second GLFW context on device 0.
        #
        # CRITICAL: The EGL offscreen context's FBOs are created in the EGL
        # context during __init__, but viewer.render() makes the GLFW context
        # current.  If we call mjr_render without switching back to the EGL
        # context, it operates on GLFW's default framebuffer → black frames.
        # make_context_current() restores the EGL context so that
        # mjr_render + mjr_readPixels target the correct offscreen FBO.
        try:
            ctx.opengl_context.make_context_current()
            ctx.render(width, height)
            rgb, depth_zbuf = ctx.read_pixels(width, height, depth=True)
        except Exception as e:
            self.get_logger().warn(
                f'Camera render failed: {e}', throttle_duration_sec=5.0)
            self._sim.model.vis.global_.fovy = saved_fovy
            return

        self._sim.model.vis.global_.fovy = saved_fovy

        # ─── FIX 4: Flip vertically (OpenGL bottom-up → ROS top-down) ───
        # mjr_readPixels uses OpenGL coordinate convention: first row = bottom.
        # ROS / RViz / OpenCV expect first row = top.  np.flipud corrects this.
        rgb = np.ascontiguousarray(np.flipud(rgb))
        depth_zbuf = np.flipud(depth_zbuf)

        # ─── FIX 5: Diagnostic — log pixel stats to confirm non-black frames ───
        if self._step_count % 100 == 0:
            rgb_min, rgb_max = float(rgb.min()), float(rgb.max())
            self.get_logger().info(
                f'Camera frame #{self._step_count}: '
                f'RGB min={rgb_min:.0f} max={rgb_max:.0f} '
                f'nonzero_ratio={(rgb > 0).mean():.4f}',
                throttle_duration_sec=2.0)

        # ─── Depth: z-buffer → metric (meters) ───
        znear = self._sim.model.vis.map.znear
        zfar = self._sim.model.vis.map.zfar
        depth_metric = np.zeros_like(depth_zbuf, dtype=np.float32)
        valid = (depth_zbuf > 0.0) & (depth_zbuf < 1.0)
        if valid.any():
            z_ndc = 2.0 * depth_zbuf[valid] - 1.0
            depth_metric[valid] = (2.0 * znear * zfar) / (
                zfar + znear - z_ndc * (zfar - znear))

        # ─── Publish RGB ───
        rgb_msg = Image()
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = 'top_down_camera'
        rgb_msg.height = height
        rgb_msg.width = width
        rgb_msg.encoding = 'rgb8'
        rgb_msg.is_bigendian = False
        rgb_msg.step = width * 3
        rgb_msg.data = rgb.tobytes()
        self._rgb_pub.publish(rgb_msg)

        # ─── Publish Depth ───
        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = 'top_down_camera'
        depth_msg.height = height
        depth_msg.width = width
        depth_msg.encoding = '32FC1'
        depth_msg.is_bigendian = False
        depth_msg.step = width * 4
        depth_msg.data = depth_metric.tobytes()
        self._depth_pub.publish(depth_msg)

        # ─── Publish CameraInfo ───
        ci = CameraInfo()
        ci.header.stamp = stamp
        ci.header.frame_id = 'top_down_camera'
        ci.width = width
        ci.height = height
        fovy_rad = np.deg2rad(self._cam_fovy)
        fy = (height / 2.0) / np.tan(fovy_rad / 2.0)
        fx = fy
        ci.k = [fx, 0.0, width / 2.0,
                0.0, fy, height / 2.0,
                0.0, 0.0, 1.0]
        ci.p = [fx, 0.0, width / 2.0, 0.0,
                0.0, fy, height / 2.0, 0.0,
                0.0, 0.0, 1.0, 0.0]
        ci.distortion_model = 'plumb_bob'
        self._ci_pub.publish(ci)


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
