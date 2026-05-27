"""Offscreen camera rendering: render, depth conversion, ROS message builders."""

import numpy as np
from mujoco_py import MjRenderContextOffscreen
from sensor_msgs.msg import Image, CameraInfo


def create_offscreen_context(sim, logger):
    """Create offscreen render context; tries OSMesa then GLFW."""
    for device_id, label in [(-1, 'OSMesa'), (0, 'GLFW')]:
        try:
            ctx = MjRenderContextOffscreen(sim, device_id)
            logger.info(f'Offscreen render context ({label}) created successfully')
            return ctx
        except Exception as e:
            logger.warn(f'Offscreen context ({label}) failed: {e}')
    raise RuntimeError(
        'Failed to create any offscreen MuJoCo render context. '
        'Install OSMesa:  apt install libosmesa6-dev')


def render_camera(ctx, sim, lookat, distance, elevation, azimuth, fovy,
                  width=640, height=480):
    """Render one offscreen frame.

    Returns:
        (rgb: np.ndarray[H,W,3], depth_zbuf: np.ndarray[H,W])
    """
    ctx.cam.lookat[:] = lookat
    ctx.cam.distance = distance
    ctx.cam.elevation = elevation
    ctx.cam.azimuth = azimuth

    saved_fovy = sim.model.vis.global_.fovy
    sim.model.vis.global_.fovy = fovy
    try:
        ctx.opengl_context.make_context_current()
        ctx.render(width, height)
        rgb, depth_zbuf = ctx.read_pixels(width, height, depth=True)
    finally:
        sim.model.vis.global_.fovy = saved_fovy

    rgb = np.ascontiguousarray(np.flipud(rgb))
    depth_zbuf = np.flipud(depth_zbuf)
    return rgb, depth_zbuf


def convert_depth(depth_zbuf, znear, zfar):
    """Convert z-buffer [0,1] to metric depth (meters)."""
    depth_metric = np.zeros_like(depth_zbuf, dtype=np.float32)
    valid = (depth_zbuf > 0.0) & (depth_zbuf < 1.0)
    if valid.any():
        z_ndc = 2.0 * depth_zbuf[valid] - 1.0
        depth_metric[valid] = (2.0 * znear * zfar) / (
            zfar + znear - z_ndc * (zfar - znear))
    return depth_metric


def calibrate_depth_scale(depth_metric, cam_distance, width=640, height=480):
    """Calibrate depth scale factor using center pixel vs known lookat distance."""
    center_v, center_u = height // 2, width // 2
    if depth_metric[center_v, center_u] > 0:
        return cam_distance / depth_metric[center_v, center_u]
    return 1.0


def build_rgb_msg(rgb, stamp, width=640, height=480):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = 'camera_link'
    msg.height = height
    msg.width = width
    msg.encoding = 'rgb8'
    msg.is_bigendian = False
    msg.step = width * 3
    msg.data = rgb.tobytes()
    return msg


def build_depth_msg(depth_metric, stamp, width=640, height=480):
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = 'camera_link'
    msg.height = height
    msg.width = width
    msg.encoding = '32FC1'
    msg.is_bigendian = False
    msg.step = width * 4
    msg.data = depth_metric.tobytes()
    return msg


def build_camera_info(stamp, fovy, width=640, height=480):
    ci = CameraInfo()
    ci.header.stamp = stamp
    ci.header.frame_id = 'camera_link'
    ci.width = width
    ci.height = height
    fovy_rad = np.deg2rad(fovy)
    fy = (height / 2.0) / np.tan(fovy_rad / 2.0)
    fx = fy
    ci.k = [fx, 0.0, width / 2.0,
            0.0, fy, height / 2.0,
            0.0, 0.0, 1.0]
    ci.p = [fx, 0.0, width / 2.0, 0.0,
            0.0, fy, height / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0]
    ci.distortion_model = 'plumb_bob'
    return ci
