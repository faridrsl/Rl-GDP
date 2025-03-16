#!/usr/bin/env python3.8

import rospy
import rospkg
import os
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

from stable_baselines3 import TD3, PPO
import numpy as np

def callback_data(lidar_received):
    rospy.loginfo("message received")

    cmd = Twist()
    # rospy.loginfo(lidar_received.ranges)
    lidar_state = np.array(lidar_received.ranges, dtype=np.float32)
    robot_state = np.array(
            [0, 0] + [0, 0, 0], #list(action)
            dtype=np.float32,
        )
    lidar_rl_inpput = np.append(robot_state, lidar_state)


    # action,_ = model.predict(observation=lidar_rl_inpput, deterministic=True)
    rospy.loginfo(lidar_rl_inpput)

    # cmd.linear.x = action[0]
    # cmd.linear.y = action[1]
    # cmd.linear.z = 0.0
    # cmd.angular.x = 0.0
    # cmd.angular.y = 0.0
    # cmd.angular.z = action[2]
    # pub.publish(cmd)


def env_maker(file_path):
 
    model = TD3.load(file_path)
    return model


if __name__ == "__main__":
    rospy.init_node("rl_agent_lidar_node")
    file_path = os.path.join(rospkg.RosPack().get_path('ros_uav_nico'), 'rl_model', 'best_model_lidar')

    model = env_maker(file_path)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
 
    sub = rospy.Subscriber("/scan", LaserScan, callback=callback_data)
    sub = rospy.Subscriber("/nav_target_data", Float32MultiArray, callback=callback_data)

    rospy.spin()
