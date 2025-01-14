# Author: REZ3LIET

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Loading Gazebo
    world = os.path.join(get_package_share_directory("llm_arm_control"), "world/objects_world.sdf")
    ign_gazebo_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),
                "/gz_sim.launch.py"
            ]
        ),
        launch_arguments={'gz_args': [world, ' -r -v1']}.items()
    )

    # Loading Robot Model
    rob_dir = get_package_share_directory('kuka_gazebo')
    kuka_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(rob_dir, "launch"), "/kuka_bringup.launch.py"]
        ),
        launch_arguments={
            "ign_gz": 'False',
            "gripper_name": "robotiq_2f_140"
        }.items()
    )

    return LaunchDescription([
        ign_gazebo_node,
        kuka_node
    ])
