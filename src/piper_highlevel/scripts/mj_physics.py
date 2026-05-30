"""MuJoCo simulation: model loading, XML merging, physics tuning, stepping."""

import os
import tempfile
from collections import namedtuple
import numpy as np
import mujoco_py
from mujoco_py.builder import functions


def merge_xml(robot_path, world_path):
    """Merge robot XML with world additions; increase nconmax."""
    with open(robot_path, 'r') as f:
        xml = f.read()
    with open(world_path, 'r') as f:
        world_additions = f.read()

    xml = xml.replace('nconmax="100"', 'nconmax="500"')
    xml = xml.replace('</worldbody>', world_additions + '\n</worldbody>')
    return xml


def tune_gripper_physics(model):
    """Apply friction, contact stiffness, and force range to gripper geoms/actuators."""
    for body_name in ('link7', 'link8'):
        body_id = model.body_name2id(body_name)
        geom_start = model.body_geomadr[body_id]
        geom_count = model.body_geomnum[body_id]
        for i in range(geom_count):
            gid = geom_start + i
            model.geom_friction[gid * 3 : gid * 3 + 3] = [5.0, 2.0, 0.5]
            model.geom_solimp[gid * 5 : gid * 5 + 5] = [0.9, 0.95, 0.001, 0.5, 2]
            model.geom_solref[gid * 2 : gid * 2 + 2] = [0.005, 1]
            model.geom_condim[gid] = 4

    for name in ('joint7', 'joint8'):
        aid = model.actuator_name2id(name)
        model.actuator_forcerange[aid * 2 : aid * 2 + 2] = [-500, 500]


def load_simulation(robot_xml_path, world_xml_path, logger):
    """Load MuJoCo model + sim + viewer; apply gripper physics tuning.

    Returns:
        (model, sim, viewer)
    """
    merged_xml = merge_xml(robot_xml_path, world_xml_path)

    saved_cwd = os.getcwd()
    os.chdir(os.path.dirname(robot_xml_path))
    try:
        fd, tmp_path = tempfile.mkstemp(
            suffix='.xml', dir=os.path.dirname(robot_xml_path))
        with os.fdopen(fd, 'w') as f:
            f.write(merged_xml)
        model = mujoco_py.load_model_from_path(tmp_path)
        os.unlink(tmp_path)
    finally:
        os.chdir(saved_cwd)

    sim = mujoco_py.MjSim(model)
    tune_gripper_physics(model)

    logger.info('Gripper physics tuned: friction/condim/solimp/force updated')
    logger.info(f'MuJoCo world loaded with {model.nbody} bodies')

    viewer = mujoco_py.MjViewer(sim)
    viewer.render()
    return model, sim, viewer


GripperForceResult = namedtuple('GripperForceResult', ['in_contact', 'force', 'joint7_pos'])

def get_gripper_contact_force(sim):
    """Scan MuJoCo contacts involving link7/link8 geometries; return total force magnitude."""
    model = sim.model
    data = sim.data

    gripper_geom_ids = set()
    for name in ('link7', 'link8'):
        try:
            body_id = model.body_name2id(name)
            geom_start = model.body_geomadr[body_id]
            geom_count = model.body_geomnum[body_id]
            for i in range(geom_count):
                gripper_geom_ids.add(geom_start + i)
        except Exception:
            continue

    if not gripper_geom_ids:
        return 0.0

    total_force = 0.0
    result = np.zeros(6)
    for i in range(data.ncon):
        contact = data.contact[i]
        if contact.geom1 in gripper_geom_ids or contact.geom2 in gripper_geom_ids:
            functions.mj_contactForce(model, data, i, result)
            total_force += abs(result[0]) + abs(result[1]) + abs(result[2])
    return total_force


def step_simulation_force_aware(sim, targets, force_threshold=50.0):
    """Two-stage damping gripper close.

    Stage 1 (fast):   ramp = 0.001   —  no contact, fast approach
    Stage 2 (slow):   ramp = 0.0001  —  contact detected, slow squeeze to target

    Returns:
        GripperForceResult(in_contact, force, joint7_actual_position)
    """
    FAST_RAMP = 0.001
    SLOW_RAMP = 0.0001

    pre_force = get_gripper_contact_force(sim)

    joints_to_ramp = {'joint7', 'joint8'}
    gripper_dir = 0.0
    current_j7 = 0.0
    target_j7 = 0.0

    try:
        j7_qpos = sim.model.get_joint_qpos_addr('joint7')
        current_j7 = sim.data.qpos[j7_qpos]
    except Exception:
        pass

    if 'joint7' in targets:
        target_j7 = targets['joint7']
        if target_j7 > current_j7 + FAST_RAMP:
            gripper_dir = 1.0
        elif target_j7 < current_j7 - FAST_RAMP:
            gripper_dir = -1.0

    if gripper_dir > 0:
        if pre_force >= force_threshold:
            ramp_step = SLOW_RAMP
        else:
            ramp_step = FAST_RAMP
    else:
        ramp_step = FAST_RAMP

    for joint_name, target in targets.items():
        if joint_name not in sim.model.joint_names:
            continue
        try:
            actuator_id = sim.model.actuator_name2id(joint_name)
            if joint_name in joints_to_ramp:
                try:
                    j_qpos = sim.model.get_joint_qpos_addr(joint_name)
                    current = sim.data.qpos[j_qpos]
                except Exception:
                    current = 0.0
                if ramp_step == 0.0:
                    ramp_target = current
                elif abs(target - current) <= ramp_step:
                    ramp_target = target
                else:
                    ramp_target = current + ramp_step if target > current else current - ramp_step
                sim.data.ctrl[actuator_id] = ramp_target
            else:
                sim.data.ctrl[actuator_id] = target
        except Exception:
            continue

    sim.step()

    force = get_gripper_contact_force(sim)
    actual_j7 = 0.0
    try:
        j7_qpos = sim.model.get_joint_qpos_addr('joint7')
        actual_j7 = sim.data.qpos[j7_qpos]
    except Exception:
        pass
    target_achieved = abs(target_j7 - actual_j7) <= 2 * FAST_RAMP
    in_contact = (gripper_dir > 0 and force >= force_threshold and target_achieved)

    return GripperForceResult(in_contact=in_contact, force=force, joint7_pos=actual_j7)


def step_simulation(sim, targets):
    """Write actuator targets and step physics (viewer render handled by caller)."""
    for joint_name, target in targets.items():
        if joint_name in sim.model.joint_names:
            try:
                actuator_id = sim.model.actuator_name2id(joint_name)
                sim.data.ctrl[actuator_id] = target
            except Exception:
                pass
    sim.step()
