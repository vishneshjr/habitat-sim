# ---
# jupyter:
#   accelerator: GPU
#   colab:
#     name: robotics_api_tutorial.ipynb
#     provenance: []
#   jupytext:
#     cell_metadata_filter: -all
#     formats: nb_python//py:percent,colabs//ipynb
#     notebook_metadata_filter: all
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.6.0
#   kernelspec:
#     display_name: Python 3
#     name: python3
#   language_info:
#     name: python
# ---

# %% [markdown]
# # Habitat-sim Articulated Object API
#
# This tutorial covers the articulated object API in Habitat-sim.

# %%
# @title Installation { display-mode: "form" }
# @markdown (double click to show code).

# !curl -L https://raw.githubusercontent.com/facebookresearch/habitat-sim/ao-api-tutorial/examples/colab_utils/colab_install.sh | PACKAGE=habitat-sim-ao bash -s
# !wget -c https://dl.fbaipublicfiles.com/habitat/URDF_demo_assets.zip && unzip -o URDF_demo_assets.zip -d /content/habitat-sim/data/URDF_demo_assets/


# %%
# @title Path Setup and Imports { display-mode: "form" }
# @markdown (double click to show code).

# %cd /content/habitat-sim
## [setup]
# import math
import os

# import random
import sys

import git
import magnum as mn
import numpy as np

import habitat_sim
from habitat_sim.utils import common as ut
from habitat_sim.utils import viz_utils as vut

# %matplotlib inline
# from matplotlib import pyplot as plt
# from PIL import Image


try:
    # import ipywidgets as widgets
    # from IPython.display import display as ipydisplay

    # For using jupyter/ipywidget IO components

    HAS_WIDGETS = True
except ImportError:
    HAS_WIDGETS = False


if "google.colab" in sys.modules:
    os.environ["IMAGEIO_FFMPEG_EXE"] = "/usr/bin/ffmpeg"

repo = git.Repo(".", search_parent_directories=True)
dir_path = repo.working_tree_dir
# %cd $dir_path
data_path = os.path.join(dir_path, "data")
output_directory = "examples/tutorials/robotics_api_output/"  # @param {type:"string"}
output_path = os.path.join(dir_path, output_directory)
if not os.path.exists(output_path):
    os.mkdir(output_path)

# define some globals the first time we run.
if "sim" not in globals():
    global sim
    sim = None
    global obj_attr_mgr
    obj_attr_mgr = None
    global prim_attr_mgr
    obj_attr_mgr = None
    global stage_attr_mgr
    stage_attr_mgr = None


# %%
# @title Define Simulation Utility Functions { display-mode: "form" }
# @markdown (double click to show code)

# @markdown These utility functions abstract away initial Simulator setup/configuration and repetative operations which would otherwise be duplicated:

# @markdown - remove_all_objects
def remove_all_objects(sim):
    for ob_id in sim.get_existing_object_ids():
        sim.remove_object(ob_id)
    for ob_id in sim.get_existing_articulated_object_ids():
        sim.remove_articulated_object(ob_id)


# @markdown - simulate
def simulate(sim, dt=1.0, get_frames=True):
    # simulate dt seconds at 60Hz to the nearest fixed timestep
    print("Simulating " + str(dt) + " world seconds.")
    observations = []
    start_time = sim.get_world_time()
    while sim.get_world_time() < start_time + dt:
        sim.step_physics(1.0 / 60.0)
        if get_frames:
            observations.append(sim.get_sensor_observations())

    return observations


# @markdown - place_robot_from_agent
def place_robot_from_agent(
    sim,
    robot_id,
    angle_correction=-1.56,
    local_base_pos=None,
):
    if local_base_pos is None:
        local_base_pos = np.array([0.0, -0.1, -2.0])
    # place the robot root state relative to the agent
    agent_transform = sim.agents[0].scene_node.transformation_matrix()
    base_transform = mn.Matrix4.rotation(
        mn.Rad(angle_correction), mn.Vector3(1.0, 0, 0)
    )
    base_transform.translation = agent_transform.transform_point(local_base_pos)
    sim.set_articulated_object_root_state(robot_id, base_transform)


# @markdown - place_camera
def place_camera(
    sim, pos=np.array([-0.15, -0.1, 1.0]), rot=np.quaternion(-0.83147, 0, 0.55557, 0)
):
    # place our "camera person" agent in the scene
    agent_state = habitat_sim.AgentState()
    agent_state.position = pos
    agent_state.rotation = rot
    agent = sim.initialize_agent(0, agent_state)
    return agent.scene_node.transformation_matrix()


# @markdown - make_default_settings
def make_default_settings():
    settings = {
        "width": 720,  # Spatial resolution of the observations
        "height": 544,
        "scene": "data/scene_datasets/habitat-test-scenes/apartment_1.glb",  # Scene path
        "seed": 1,
    }
    return settings


# @markdown - make_configuration
def make_configuration(sim_settings):
    # simulator configuration
    backend_cfg = habitat_sim.SimulatorConfiguration()
    backend_cfg.scene_id = sim_settings["scene"]
    backend_cfg.enable_physics = True

    # configure rgbd sensors coincident with "agent" position/orientation
    camera_resolution = [sim_settings["height"], sim_settings["width"]]
    sensors = {
        "rgba_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.COLOR,
            "resolution": camera_resolution,
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
        "depth_camera_1stperson": {
            "sensor_type": habitat_sim.SensorType.DEPTH,
            "resolution": camera_resolution,
            "position": [0.0, 0.0, 0.0],
            "orientation": [0.0, 0.0, 0.0],
        },
    }

    sensor_specs = []
    for sensor_uuid, sensor_params in sensors.items():
        sensor_spec = habitat_sim.CameraSensorSpec()
        sensor_spec.uuid = sensor_uuid
        sensor_spec.sensor_type = sensor_params["sensor_type"]
        sensor_spec.resolution = sensor_params["resolution"]
        sensor_spec.position = sensor_params["position"]
        sensor_spec.orientation = sensor_params["orientation"]
        sensor_specs.append(sensor_spec)

    # agent "camera person" configuration
    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = sensor_specs

    return habitat_sim.Configuration(backend_cfg, [agent_cfg])


# @markdown - remake_simulator
def remake_simulator(sim_settings):
    cfg = make_configuration(sim_settings)
    # clean-up the current simulator instance if it exists
    global sim
    global obj_attr_mgr
    global prim_attr_mgr
    global stage_attr_mgr
    if sim != None:
        # close the simulator to clear allocated asset memory
        # Note: force destruction of the background rendering thread
        sim.close()
    # initialize the simulator
    sim = habitat_sim.Simulator(cfg)
    # Managers of various Attributes templates
    obj_attr_mgr = sim.get_object_template_manager()
    obj_attr_mgr.load_configs(str(os.path.join(data_path, "objects")))
    prim_attr_mgr = sim.get_asset_template_manager()
    stage_attr_mgr = sim.get_stage_template_manager()


# %%
# @title Pre-define Paths to Example URDF Files Available for Import:
# @markdown  - Aliengo
# @markdown  - KUKA Iiwa
# @markdown  - Locobot (w/ and w/o gripper)

urdf_files = {
    "aliengo": os.path.join(data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"),
    "iiwa": os.path.join(data_path, "test_assets/urdf/kuka_iiwa/model_free_base.urdf"),
    "locobot": os.path.join(data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"),
    "locobot_light": os.path.join(
        data_path, "URDF_demo_assets/aliengo/urdf/aliengo.urdf"
    ),
}

# %%
# @title Setup Visualization Flags { display-mode: "form" }
# @markdown (double click to show code)

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--no-display", dest="display", action="store_false")
    parser.add_argument("--no-make-video", dest="make_video", action="store_false")
    parser.set_defaults(show_video=True, make_video=True)
    args, _ = parser.parse_known_args()
    show_video = args.display
    display = args.display
    make_video = args.make_video
else:
    show_video = False
    make_video = False
    display = False


# %%
# @title A Minimal Example

# @markdown Load a robot URDF into a scene and simulate.
def minimal_urdf_example(make_video=True, show_video=True):

    # create a fresh Simulator instance
    sim_settings = make_default_settings()
    remake_simulator(sim_settings)

    # position the camera
    camera_tilt = ut.quat_from_angle_axis(-0.4, np.array([1.0, 0, 0]))
    camera_pan = ut.quat_from_angle_axis(-1.12, np.array([0, 1.0, 0]))
    place_camera(sim, rot=camera_pan * camera_tilt)

    # load a URDF file
    robot_file = urdf_files["aliengo"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file)

    # place the robot root state relative to the agent
    place_robot_from_agent(sim, robot_id)

    # simulate
    observations = simulate(sim, dt=2.0, get_frames=make_video)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "minimal_example",
            open_vid=show_video,
        )


minimal_urdf_example(make_video, show_video)


# %%
# @title Get/set Robot State:


def robot_state_manipulation(make_video=True, show_video=True):
    # create a fresh Simulator instance
    sim_settings = make_default_settings()
    remake_simulator(sim_settings)

    # position the camera
    camera_tilt = ut.quat_from_angle_axis(-0.4, np.array([1.0, 0, 0]))
    camera_pan = ut.quat_from_angle_axis(-1.12, np.array([0, 1.0, 0]))
    place_camera(sim, rot=camera_pan * camera_tilt)

    # load a URDF file
    robot_file = urdf_files["aliengo"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file)

    # place the robot root state relative to the agent
    place_robot_from_agent(sim, robot_id)

    # @markdown Getting and setting torques, velocities, and positions.
    # @markdown - `get_articulated_object_forces(robot_id)`
    # @markdown - `set_articulated_object_forces(robot_id, tau)`
    # @markdown - `get_articulated_object_velocities(robot_id)`
    # @markdown - `set_articulated_object_velocities(robot_id, q_dot)`
    # @markdown - `get_articulated_object_positions(robot_id)`
    # @markdown - `set_articualted_object_positions(robot_id, q)`
    tau = sim.get_articulated_object_forces(robot_id)
    sim.set_articulated_object_forces(robot_id, tau)

    vel = sim.get_articulated_object_velocities(robot_id)
    sim.set_articulated_object_velocities(robot_id, vel)

    pos = sim.get_articulated_object_positions(robot_id)
    sim.set_articulated_object_positions(robot_id, pos)

    observations = []
    observations += simulate(sim, dt=1.0, get_frames=make_video)

    # @markdown Setting MotionType: a KINEMATIC articulated oject is collidable, but does not actively simulate dynamics.
    # @markdown - `set_articulated_object_motion_type(robot_id, MotionType)`
    # @markdown - `get_articulated_object_motion_type(robot_id)`
    sim.set_articulated_object_motion_type(
        robot_id, habitat_sim.physics.MotionType.KINEMATIC
    )
    assert (
        sim.get_articulated_object_motion_type(robot_id)
        == habitat_sim.physics.MotionType.KINEMATIC
    )

    # @markdown `reset_articulated_object(robot_id)`: resets the positions, velocities, and torques. Then computes forward kinematics and updates the collision state.
    sim.reset_articulated_object(robot_id)
    # @markdown - note: reset does not change the robot base state, do this manually with `set_articulated_object_root_state(robot_id, base_transform)`.
    place_robot_from_agent(sim, robot_id)

    # @markdown Querying link states and maximal coordinates.
    # @markdown - `get_articulated_link_rigid_state(robot_id, link_id)`
    # @markdown - `get_articulated_link_friction(robot_id, link_id)`
    # get rigid state of robot links and show proxy object at each link COM
    obj_mgr = sim.get_object_template_manager()
    cube_id = sim.add_object_by_handle(obj_mgr.get_template_handles("cube")[0])
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, cube_id)
    sim.set_object_is_collidable(False, cube_id)
    num_links = sim.get_num_articulated_links(robot_id)
    for link_id in range(num_links):
        link_rigid_state = sim.get_articulated_link_rigid_state(robot_id, link_id)
        sim.set_translation(link_rigid_state.translation, cube_id)
        sim.set_rotation(link_rigid_state.rotation, cube_id)
        # get the link friction
        print(
            "Link "
            + str(link_id)
            + " friction coefficient = "
            + str(sim.get_articulated_link_friction(robot_id, link_id))
        )
        # Note: set this with 'sim.get_articulated_link_friction(robot_id, link_id, friction)'
        observations += simulate(sim, dt=0.5, get_frames=make_video)
    sim.remove_object(cube_id)

    sim.set_articulated_object_motion_type(
        robot_id, habitat_sim.physics.MotionType.DYNAMIC
    )
    assert (
        sim.get_articulated_object_motion_type(robot_id)
        == habitat_sim.physics.MotionType.DYNAMIC
    )

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "URDF_basics",
            open_vid=show_video,
        )


robot_state_manipulation(make_video, show_video)


# %%
# @title Joint Motor Control

# @markdown Configuring position and velocity motors for robot joints.


def joint_motors(make_video=True, show_video=True):
    # create a fresh Simulator instance
    sim_settings = make_default_settings()
    remake_simulator(sim_settings)

    # position the camera
    place_camera(sim)
    observations = []

    # load a URDF file with a fixed base
    robot_file = urdf_files["iiwa"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file, True)

    # place the robot root state relative to the agent
    place_robot_from_agent(sim, robot_id, -3.14)

    # @markdown  - By default, joint damping values from the URDF are consumed as max_impulse for a set of velocity JointMotors (with gain=1).
    # @markdown  - `get_existing_joint_motors(robot_id)` - JointMotors can be queried for a robot returning `[motor_id, dof]` pairs.

    # query any damping motors created by default
    existing_joint_motors = sim.get_existing_joint_motors(robot_id)
    print("Default damping motors (motor_id -> dof): " + str(existing_joint_motors))

    # get the max_impulse of the damping motors
    for motor_id in existing_joint_motors:
        motor_settings = sim.get_joint_motor_settings(robot_id, motor_id)
        print(
            "   motor("
            + str(motor_id)
            + "): max_impulse = "
            + str(motor_settings.max_impulse)
        )

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # @markdown  - `create_joint_motor(robot_id, dof, settings)` - New JointMotors can be created from JointMotorSettings for Revolute and Prismatic joints.
    # create a new velocity motor
    joint_motor_settings = habitat_sim.physics.JointMotorSettings(
        0,  # position_target
        0,  # position_gain
        1.0,  # velocity_target
        1.0,  # velocity_gain
        10.0,  # max_impulse
    )
    new_motor_id = sim.create_joint_motor(
        robot_id, 1, joint_motor_settings  # robot object id  # dof  # settings
    )
    existing_joint_motors = sim.get_existing_joint_motors(robot_id)
    print("new_motor_id: " + str(new_motor_id))
    print(
        "Existing motors after create (motor_id -> dof): " + str(existing_joint_motors)
    )

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # @markdown  - `update_joint_motor(robot_id, motor_id, settings)` - JointMotors can be updated from JointMotorSettings.
    # reverse the motor velocity
    joint_motor_settings.velocity_target = -1.0
    sim.update_joint_motor(robot_id, new_motor_id, joint_motor_settings)

    # simulate
    observations += simulate(sim, dt=1.5, get_frames=make_video)

    # @markdown  - `remove_joint_motor(robot_id, motor_id)` - JointMotors can be removed by id.
    # remove the new joint motor
    sim.remove_joint_motor(robot_id, new_motor_id)

    # @markdown  - `create_motors_for_all_dofs(robot_id, (optional)settings)` - JointMotors can be created for all of a robot's joints (Revolute|Prismatic) from a single JointMotorSettings. Returns `[[dof, motor_id], ...]`.
    # create joint motors for all valid dofs to control a pose (1.1 for all dofs)
    joint_motor_settings = habitat_sim.physics.JointMotorSettings(0.5, 1.0, 0, 0, 1.0)
    dofs_to_motor_ids = sim.create_motors_for_all_dofs(
        robot_id,
        joint_motor_settings,  # (optional) motor settings, if not provided will be default (no gains)
    )
    print("New motors (motor_ids -> dofs): " + str(dofs_to_motor_ids))

    # simulate
    observations += simulate(sim, dt=2.0, get_frames=make_video)

    # remove all motors
    existing_joint_motors = sim.get_existing_joint_motors(robot_id)
    print("All motors (motor_id -> dof) before removal: " + str(existing_joint_motors))
    for motor_id in existing_joint_motors:
        sim.remove_joint_motor(robot_id, motor_id)
    print(
        "All motors (motor_id -> dof) before removal: "
        + str(sim.get_existing_joint_motors(robot_id))
    )

    # simulate
    observations += simulate(sim, dt=3.0, get_frames=make_video)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "URDF_joint_motors",
            open_vid=show_video,
        )


joint_motors(make_video, show_video)


# %%
# @title Scaling URDF and Caching

# @markdown Loaded URDF files are cached and future load calls import the cached version unless `force_reload=True` is specified:
# @markdown `add_articulated_object_from_urdf(URDF_file, use_fixed_base=False, urdf_global_scale=1, mass_scale=1, force_reload=False)`
# @markdown Note: A cached model can be re-scaled without re-parsing the file.
def urdf_scaling(make_video=True, show_video=True):
    # create a fresh Simulator instance
    sim_settings = make_default_settings()
    remake_simulator(sim_settings)

    # position the camera
    camera_tilt = ut.quat_from_angle_axis(-0.3, np.array([1.0, 0, 0]))
    camera_pan = ut.quat_from_angle_axis(-1.12, np.array([0, 1.0, 0]))
    place_camera(sim, rot=camera_pan * camera_tilt)

    observations = []
    for iteration in range(1, 4):
        # load a URDF file with variable scale
        robot_file = urdf_files["aliengo"]
        urdf_global_scale = iteration / 2.0
        robot_id = sim.add_articulated_object_from_urdf(
            robot_file, False, urdf_global_scale
        )
        print("Scaled URDF by " + str(urdf_global_scale))

        # place the robot root state relative to the agent
        place_robot_from_agent(sim, robot_id)

        # set a better initial joint state for the aliengo
        if robot_file == urdf_files["aliengo"]:
            pose = sim.get_articulated_object_positions(robot_id)
            calfDofs = [2, 5, 8, 11]
            for dof in calfDofs:
                pose[dof] = -1.0
                pose[dof - 1] = 0.45
                # also set a thigh
            sim.set_articulated_object_positions(robot_id, pose)

        # simulate
        observations += simulate(sim, dt=1.5, get_frames=make_video)

        # clear all robots
        # for robot_id in sim.get_existing_articulated_object_ids():
        #  sim.remove_articulated_object(robot_id)

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "URDF_scaling",
            open_vid=show_video,
        )


urdf_scaling(make_video, show_video)


# %%
# @title Creating and Managing Constraints
# @markdown Habitat-sim provides APIs for creating point-to-point (ball joint) constraints between any combination of articulated and rigid objects.
# @markdown *Note: all constraint creation functions allow an optional final variable* `max_impulse=2` *to configure the constraint strength.*


def test_constraints(make_video=True, show_video=True, test_case=0):
    # create the simulator
    sim_settings = make_default_settings()
    remake_simulator(sim_settings)
    place_camera(sim)
    observations = []

    # load a URDF file
    robot_file = urdf_files["aliengo"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file)
    ef_link_id = 16  # foot = 16, TODO: base = -1
    ef_link2_id = 12
    iiwa_ef_link = 6

    # add a constraint visualization object
    obj_mgr = sim.get_object_template_manager()
    sphere_id = sim.add_object_by_handle(obj_mgr.get_template_handles("sphere")[0])
    sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, sphere_id)
    sim.set_object_is_collidable(False, sphere_id)

    place_robot_from_agent(sim, robot_id)

    # Test constraint types:
    if test_case == 0:
        # @markdown **AO -> world**
        # should constrain to the center of the sphere
        link_rigid_state = sim.get_articulated_link_rigid_state(robot_id, ef_link_id)
        sim.set_translation(link_rigid_state.translation, sphere_id)
        # @markdown - `create_articulated_p2p_constraint(robot_id, link_id, global_point)`: constraint a link to a global point. The pivot is located at the constraint position.
        constraint_id = sim.create_articulated_p2p_constraint(
            object_id=robot_id,
            link_id=ef_link_id,
            global_constraint_point=link_rigid_state.translation,
        )
        observations += simulate(sim, dt=3.0, get_frames=make_video)
        sim.remove_constraint(constraint_id)
    elif test_case == 1:
        # - AO -> world w/ offset
        # should constrain to the boundary of the sphere
        link_rigid_state = sim.get_articulated_link_rigid_state(robot_id, ef_link_id)
        link_offset = mn.Vector3(0, 0, -0.1)
        global_constraint_position = link_rigid_state.translation
        sim.set_translation(global_constraint_position, sphere_id)
        # @markdown - `create_articulated_p2p_constraint(robot_id, link_id, link_offset, global_point)`: constraint a particular local point on the link to a global point.
        constraint_id = sim.create_articulated_p2p_constraint(
            object_id=robot_id,
            link_id=ef_link_id,
            link_offset=link_offset,
            global_constraint_point=global_constraint_position,
        )
        observations += simulate(sim, dt=3.0, get_frames=make_video)
        sim.remove_constraint(constraint_id)
    elif test_case == 2:
        # @markdown **AO -> AO** (w/ offsets)
        robot_id2 = sim.add_articulated_object_from_urdf(robot_file)
        place_robot_from_agent(
            sim=sim,
            robot_id=robot_id2,
            local_base_pos=np.array([0.35, -0.1, -2.0]),
        )
        # attach the agents' feet together
        link_b_rigid_state = sim.get_articulated_link_rigid_state(robot_id2, ef_link_id)
        # @markdown - `create_articulated_p2p_constraint(robot1_id, link1_id, offset1, robot2_id, link2_id, offset2)`: constraint a local point on one articulated object to a local point on another.
        constraint_id = sim.create_articulated_p2p_constraint(
            object_id_a=robot_id,
            link_id_a=ef_link_id,
            offset_a=mn.Vector3(),
            object_id_b=robot_id2,
            link_id_b=ef_link_id,
            offset_b=mn.Vector3(),
        )

        # constrain 1st robot in the air by other foot
        link_a2_rigid_state = sim.get_articulated_link_rigid_state(
            robot_id, ef_link2_id
        )
        global_constraint_position = link_a2_rigid_state.translation + mn.Vector3(
            0, 1.5, 0
        )
        sim.set_translation(global_constraint_position, sphere_id)
        # note: increase max impulse: the combined weight of the robots is greater than the default impulse correction (2)
        constraint_id2 = sim.create_articulated_p2p_constraint(
            object_id=robot_id,
            link_id=ef_link2_id,
            link_offset=mn.Vector3(),
            global_constraint_point=global_constraint_position,
            max_impulse=6,
        )

        observations += simulate(sim, dt=3.0, get_frames=make_video)
        sim.remove_constraint(constraint_id)
        sim.remove_constraint(constraint_id2)
        sim.remove_articulated_object(robot_id2)
    elif test_case == 3:
        # - AO -> AO (global)
        robot_id2 = sim.add_articulated_object_from_urdf(urdf_files["iiwa"], True)
        place_robot_from_agent(
            sim=sim,
            robot_id=robot_id2,
            local_base_pos=np.array([0.35, -0.4, -2.0]),
        )
        jm_settings = habitat_sim.physics.JointMotorSettings()
        jm_settings.position_gain = 2.0
        sim.create_motors_for_all_dofs(robot_id2, jm_settings)
        # TODO: not a great test, could be a better setup
        # attach two agent feet to the iiwa end effector
        link_b_rigid_state = sim.get_articulated_link_rigid_state(
            robot_id2, iiwa_ef_link
        )
        global_constraint_position = link_b_rigid_state.translation
        sim.set_translation(global_constraint_position, sphere_id)
        constraint_id = sim.create_articulated_p2p_constraint(
            object_id_a=robot_id,
            link_id_a=ef_link_id,
            object_id_b=robot_id2,
            link_id_b=iiwa_ef_link,
            global_constraint_point=global_constraint_position,
            max_impulse=4,
        )
        # @markdown - `create_articulated_p2p_constraint(robot1_id, link1_id, robot2_id, link2_id, global_point)`: constrain two articulated object links at a global point.
        constraint_id2 = sim.create_articulated_p2p_constraint(
            object_id_a=robot_id,
            link_id_a=ef_link2_id,
            object_id_b=robot_id2,
            link_id_b=iiwa_ef_link,
            global_constraint_point=global_constraint_position,
            max_impulse=4,
        )

        observations += simulate(sim, dt=3.0, get_frames=make_video)
        sim.remove_constraint(constraint_id)
        sim.remove_constraint(constraint_id2)
        sim.remove_articulated_object(robot_id2)
    elif test_case == 4:
        # @markdown **AO -> rigid**
        # @markdown  - *Note: rigid->world is also provided `create_rigid_p2p_constraint` but not demonstrated here.*
        # tilt the camera down
        prev_state = sim.get_agent(0).scene_node.rotation
        sim.get_agent(0).scene_node.rotation = (
            mn.Quaternion.rotation(
                mn.Rad(-0.4), prev_state.transform_vector(mn.Vector3(1.0, 0, 0))
            )
            * prev_state
        )

        # attach an active sphere to one robot foot w/ pivot at the object center
        active_sphere_id = sim.add_object_by_handle(
            obj_mgr.get_template_handles("sphere")[0]
        )
        link_rigid_state = sim.get_articulated_link_rigid_state(robot_id, ef_link_id)
        link2_rigid_state = sim.get_articulated_link_rigid_state(robot_id, ef_link2_id)
        sim.set_translation(
            link2_rigid_state.translation + mn.Vector3(0, -0.1, 0),
            active_sphere_id,
        )
        # @markdown - `create_articulated_p2p_constraint(robot_id, link_id, object_id)`: constrain an object to an articulated link with pivot at the object's COM.
        constraint_id = sim.create_articulated_p2p_constraint(
            object_id_a=robot_id,
            link_id=ef_link2_id,
            object_id_b=active_sphere_id,
            max_impulse=4,
        )
        # attach the visual sphere to another robot foot w/ pivots
        sim.set_object_motion_type(habitat_sim.physics.MotionType.DYNAMIC, sphere_id)
        # @markdown - `create_articulated_p2p_constraint(robot_id, link_id, object_id, offset_link, offset_object)`: constrain an object to an articulated link with provided local pivots for each.
        constraint_id2 = sim.create_articulated_p2p_constraint(
            object_id_a=robot_id,
            link_id=ef_link_id,
            object_id_b=sphere_id,
            pivot_a=mn.Vector3(0.1, 0, 0),
            pivot_b=mn.Vector3(-0.1, 0, 0),
            max_impulse=4,
        )

        observations += simulate(sim, dt=3.0, get_frames=make_video)
        sim.remove_constraint(constraint_id)
        sim.remove_constraint(constraint_id2)
        sim.set_object_motion_type(habitat_sim.physics.MotionType.KINEMATIC, sphere_id)
        sim.remove_object(active_sphere_id)

        sim.get_agent(0).scene_node.rotation = prev_state
    elif test_case == 5:
        # - AO -> rigid (fixed) TODO: not working as expected

        # tilt the camera down
        prev_state = sim.get_agent(0).scene_node.rotation
        sim.get_agent(0).scene_node.rotation = (
            mn.Quaternion.rotation(
                mn.Rad(-0.4), prev_state.transform_vector(mn.Vector3(1.0, 0, 0))
            )
            * prev_state
        )

        # attach an active sphere to one robot foot w/ pivot at the object center
        active_sphere_id = sim.add_object_by_handle(
            obj_mgr.get_template_handles("sphere")[0]
        )
        link2_rigid_state = sim.get_articulated_link_rigid_state(robot_id, ef_link2_id)
        sim.set_translation(
            link2_rigid_state.translation + mn.Vector3(0, -0.15, 0),
            active_sphere_id,
        )
        constraint_id = sim.create_articulated_fixed_constraint(
            object_id_a=robot_id,
            link_id=ef_link2_id,
            object_id_b=active_sphere_id,
            max_impulse=4,
        )

        observations += simulate(sim, dt=3.0, get_frames=make_video)
        sim.remove_constraint(constraint_id)
        sim.remove_object(active_sphere_id)

        sim.get_agent(0).scene_node.rotation = prev_state

    if make_video:
        vut.make_video(
            observations,
            "rgba_camera_1stperson",
            "color",
            output_path + "test_constraints_" + str(test_case),
            open_vid=show_video,
        )


# @markdown **Constraints can be removed by ID** (returned from creation):
# @markdown - `remove_constraint(constraint_id)`

# run all constraint variants
for test_case in range(5):
    test_constraints(make_video, show_video, test_case)


# %%
# @title Contact information queries:

# @markdown Habitat-sim provides several contact query features including:


def demo_contact_profile():
    sim_settings = make_default_settings()
    remake_simulator(sim_settings)
    place_camera(sim)
    observations = []

    # add a robot to the scene
    robot_file = urdf_files["aliengo"]
    robot_id = sim.add_articulated_object_from_urdf(robot_file)

    # @markdown - `get_physics_step_collision_summary()`: returns a summary string of the previous collision detection iteration.
    # gets nothing because physics has not stepped yet
    print(sim.get_physics_step_collision_summary())

    # gets nothing because no active collisions yet
    sim.step_physics(0.1)
    print(sim.get_physics_step_collision_summary())

    # give time for the robot to hit the ground then check contacts
    sim.step_physics(1.0)
    print("Step Collision Summary:")
    print(sim.get_physics_step_collision_summary())

    # now add two colliding robots and run discrete collision detection
    sim.remove_articulated_object(robot_id)
    robot_id1 = sim.add_articulated_object_from_urdf(robot_file)
    robot_id2 = sim.add_articulated_object_from_urdf(robot_file)
    place_robot_from_agent(sim, robot_id1)
    place_robot_from_agent(sim, robot_id2, local_base_pos=np.array([0.15, -0.1, -2.0]))
    # @markdown - `perform_discrete_collision_detection()`: performs a single discrete collision check for the full scene.
    sim.perform_discrete_collision_detection()
    print("Step Collision Summary:")
    print(sim.get_physics_step_collision_summary())
    # @markdown - `get_physics_num_active_overlapping_pairs()`: returns the number of "active" overlapping collision object pairs found during the previous collision check.
    print(
        "Num overlapping pairs: " + str(sim.get_physics_num_active_overlapping_pairs())
    )
    # @markdown - `get_physics_num_active_contact_points()`: returns the number of "active" contact points found during the previous collision check.
    print(
        "Num active contact points: " + str(sim.get_physics_num_active_contact_points())
    )
    # @markdown - `get_physics_contact_points()`: returns an information structure for all active contact points.
    contact_points = sim.get_physics_contact_points()
    print("Active contact points: ")
    for cp_ix, cp in enumerate(contact_points):
        print(" Contact Point " + str(cp_ix) + ":")
        print("     object_id_a = " + str(cp.object_id_a))
        print("     object_id_b = " + str(cp.object_id_b))
        print("     link_id_a = " + str(cp.link_id_a))
        print("     link_id_b = " + str(cp.link_id_b))
        print("     position_on_a_in_ws = " + str(cp.position_on_a_in_ws))
        print("     position_on_b_in_ws = " + str(cp.position_on_b_in_ws))
        print("     contact_normal_on_b_in_ws = " + str(cp.contact_normal_on_b_in_ws))
        print("     contact_distance = " + str(cp.contact_distance))
        print("     normal_force = " + str(cp.normal_force))
        print("     linear_friction_force1 = " + str(cp.linear_friction_force1))
        print("     linear_friction_force2 = " + str(cp.linear_friction_force2))
        print("     linear_friction_direction1 = " + str(cp.linear_friction_direction1))
        print("     linear_friction_direction2 = " + str(cp.linear_friction_direction2))
        print("     is_active = " + str(cp.is_active))

    observations.append(sim.get_sensor_observations())
    # TODO: visualize the contact points
    im = vut.observation_to_image(observations[-1]["rgba_camera_1stperson"], "color")
    im.show()


demo_contact_profile()
