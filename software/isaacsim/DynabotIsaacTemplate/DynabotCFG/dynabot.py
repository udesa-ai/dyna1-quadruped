#import omni.isaac.lab.sim as sim_utils
#from omni.isaac.lab.actuators import DCMotorCfg, ActuatorNetLSTMCfg 
#from omni.isaac.lab.assets.articulation import ArticulationCfg
#from omni.isaac.lab.utils.assets import ISAACLAB_NUCLEUS_DIR

# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the ANYbotics robots.

The following configuration parameters are available:

* :obj:`ANYMAL_B_CFG`: The ANYmal-B robot with ANYdrives 3.0
* :obj:`ANYMAL_C_CFG`: The ANYmal-C robot with ANYdrives 3.0
* :obj:`ANYMAL_D_CFG`: The ANYmal-D robot with ANYdrives 3.0

Reference:

* https://github.com/ANYbotics/anymal_b_simple_description
* https://github.com/ANYbotics/anymal_c_simple_description
* https://github.com/ANYbotics/anymal_d_simple_description

"""
import os
from isaaclab_assets.sensors.velodyne import VELODYNE_VLP_16_RAYCASTER_CFG

import isaaclab.sim as sim_utils
from isaaclab.actuators import ActuatorNetLSTMCfg, DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.sensors import RayCasterCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR


##
# Configuration
##

"""ANYDRIVE_3_LSTM_ACTUATOR_CFG = ActuatorNetLSTMCfg(
    joint_names_expr=[".*_shoulder", ".*shoulder_to_arm", ".*arm_to_hand"],
    #joint_names_expr=[".*HAA", ".*HFE", ".*KFE"],
    network_file=f"{ISAACLAB_NUCLEUS_DIR}/ActuatorNets/ANYbotics/anydrive_3_lstm_jit.pt",
    saturation_effort=120.0,
    effort_limit=80.0,
    velocity_limit=7.5,
)"""

USD_PATH = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "../assets/dyna1_gazebo3.usd")
)

DYNABOT_1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        #usd_path="/workspace/Dyna1ExtensionTemplate/urdf/dyna1_gazebo3/dyna1_gazebo3.usd",
        usd_path = USD_PATH,
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.3),
        joint_pos={
            ".*_shoulder": 0.0,
            ".*shoulder_to_arm": - 0.79,
            ".*arm_to_hand": 1.5,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": DCMotorCfg(
            joint_names_expr=[".*_shoulder", ".*shoulder_to_arm", ".*arm_to_hand"],
            effort_limit=14.5,
            saturation_effort=20.5,
            velocity_limit=21.0,
            stiffness=80.0,
            damping=0.5,
            friction=0.1,
        ),
    },
    #actuators={"legs": ANYDRIVE_3_LSTM_ACTUATOR_CFG},
    #soft_joint_pos_limit_factor=0.95,
)
