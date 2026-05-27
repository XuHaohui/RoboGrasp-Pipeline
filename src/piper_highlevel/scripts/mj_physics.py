"""MuJoCo simulation: model loading, XML merging, physics tuning, stepping."""

import os
import tempfile
import mujoco_py


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
            model.geom_friction[gid * 3 : gid * 3 + 3] = [2.0, 1.0, 0.5]
            model.geom_solimp[gid * 5 : gid * 5 + 5] = [0.9, 0.95, 0.001, 0.5, 2]
            model.geom_solref[gid * 2 : gid * 2 + 2] = [0.02, 1]
            model.geom_condim[gid] = 4

    for name in ('joint7', 'joint8'):
        aid = model.actuator_name2id(name)
        model.actuator_forcerange[aid * 2 : aid * 2 + 2] = [-200, 200]


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
