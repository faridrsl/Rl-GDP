import gymnasium as gym
from cranavgym.envs.drl_robot_navigation import DRLRobotNavigation

import os
import subprocess
import numpy as np

# load config
import yaml
from munch import Munch

import roslaunch
import rospy
import time


def launch_ROS(launchfile, ros_port):
    """
    Initialize and launch the ROS core and Gazebo simulation.

    Args:
        launchfile (str): The path of the launchfile to be used for the simulation.
        ros_port (int): The port number for the ROS core.
    """
    print(f"{ros_port=}")

    # Start roscore with subprocess
    subprocess.Popen(["roscore", "-p", str(ros_port)])

    rospy.loginfo("Roscore launched!")

    # Give roscore some time to initialize
    time.sleep(2)

    # Initialize ROS node
    rospy.init_node("gym", anonymous=True, log_level=rospy.ERROR)

    # Expand launch file path
    fullpath = os.path.abspath(os.path.expanduser(launchfile))

    # Create a launch parent to manage roslaunch processes
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Setup the roslaunch arguments with custom parameters (gui, rviz, robot_name)
    cli_args = [
        fullpath,
        "gui:=true",
        "rviz:=true",
        "robot_name:=uav1",
    ]

    roslaunch_file = [
        (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])
    ]

    # Create roslaunch parent process
    launch_parent = roslaunch.parent.ROSLaunchParent(
        uuid,
        roslaunch_file,
        is_core=False,
    )

    # Start roslaunch
    launch_parent.start()
    time.sleep(3)
    rospy.loginfo("Gazebo and RViz launched!")
    return

def load_ros_config():
    filepath = "../configs/ros_interface_config_drone.yaml"
    with open(filepath) as stream:
        try:
            config_dict = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    munch_config = Munch.fromDict(config_dict)
    return munch_config


def load_env_config():
    filepath = "../configs/env_config_drone.yaml"
    with open(filepath) as stream:
        try:
            config_dict = yaml.safe_load(stream)
        except yaml.YAMLError as exc:
            print(exc)
    munch_config = Munch.fromDict(config_dict)
    return munch_config


def main():
    ros_config = load_ros_config()
    env_config = load_env_config()
    env = gym.make(
        "DroneNavigationGDP-v0",
        ros_interface_config=ros_config,
        max_ep_steps=int(env_config.scenario_settings.max_episode_steps),
        obs_space_type=env_config.scenario_settings.observations.obs_space_type,
        reward_type=env_config.scenario_settings.reward_type,
        camera_noise=env_config.drl_robot_navigation.camera_noise,
        camera_noise_area_size=env_config.drl_robot_navigation.camera_noise_area_size,
        random_camera_noise_area=env_config.drl_robot_navigation.random_camera_noise_area,
        static_camera_noise_area_pos=env_config.drl_robot_navigation.static_camera_noise_area_pos,
        camera_noise_type=env_config.drl_robot_navigation.camera_noise_type,
        lidar_noise=env_config.drl_robot_navigation.lidar_noise,
        lidar_noise_area_size=env_config.drl_robot_navigation.lidar_noise_area_size,
        random_lidar_noise_area=env_config.drl_robot_navigation.random_lidar_noise_area,
        static_lidar_noise_area_pos=env_config.drl_robot_navigation.static_lidar_noise_area_pos,
        static_goal=env_config.scenario_settings.static_goal,
        static_goal_xy=env_config.scenario_settings.static_goal_xy,
        static_spawn=env_config.scenario_settings.static_spawn,
        static_spawn_xy=env_config.scenario_settings.static_spawn_xy
    )

    obs, _ = env.reset()

    for i in range(10000):
        action = env.action_space.sample()
        # print(f"step: {action=}")
        # action = np.array([1,1])

        # if i < 10:
        #     action = np.array([0,0,1])
        # else:
        #     action = np.array([1,0,0])
        obs, rew, done, _, _ = env.step(action)
        
        if done:
            obs, _ = env.reset()


if __name__ == "__main__":

    launchfile = f"/home/{os.getenv('USER')}/cranfield-navigation-gym/cranavgym/ros_interface/DroneNavSimple.launch"
    # launchfile = f"/home/{os.getenv('USER')}/ros-rl-env/catkin_ws/src/hector_quadrotor_noetic/hector_quadrotor/hector_quadrotor_demo/launch/nico_flight_arena.launch"
    launch_ROS(launchfile, 11311)
    main()
