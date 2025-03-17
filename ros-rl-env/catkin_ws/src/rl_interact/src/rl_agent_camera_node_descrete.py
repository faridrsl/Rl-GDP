#!/usr/bin/env python3

import rospy
import rospkg
import os
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


from stable_baselines3 import TD3, PPO
from customcnn_descrete import CustomCNN
import time


def callback_data(image_received):
	rospy.loginfo("message received")

	# Convert the ROS Image message to OpenCV format (without using OpenCV)
	if image_received.encoding != "bgr8":
		rospy.logerr(f"Unsupported encoding: {image_received.encoding}")
		return

	# Convert ROS Image to NumPy array
	height = image_received.height
	width = image_received.width
	image_bgr = np.frombuffer(image_received.data, dtype=np.uint8).reshape((height,width,3))

	# # Convert BGR8 to RGB8 by swapping channels
	# image_rgb = np.ascontiguousarray(image_bgr[..., ::-1]) # else use [:,:,[2,1,0]]
	# #rospy.loginfo(f"Received BFR image, converted to RGB with shape: {image_rgb.shape}")
	
	###### for simulation only #####
	image_rgb = np.frombuffer(image_received.data, dtype=np.uint8).reshape((height,width,3))

	image_rl_inpput = image_rgb	
	action,_ = model.predict(observation=image_rl_inpput, deterministic=True)
	
	cmd = Twist()
	if action == 1: # forward
		cmd.linear.x = 1
		cmd.angular.z = 0
	elif action == 2: # yaw to left
		cmd.linear.x = 0
		cmd.angular.z = 1
	elif action == 3: # yaw to right
		cmd.linear.x = 0
		cmd.angular.z = -1
	elif action == 0: # reset
		cmd.linear.x = 0
		cmd.angular.z = 0
		

	cmd.linear.y = 0.0
	cmd.linear.z = 0.0
	cmd.angular.x = 0.0
	cmd.angular.y = 0.0
	
	pub.publish(cmd)
	time.sleep(0.5)
	#r = rospy.Rate(2)
	#while not rospy.is_shutdown():
		#pub.publish(cmd)
		#r.sleep()
	# rospy.loginfo(cmd)

def env_maker(file_path):
	policy_kwargs = dict(
		features_extractor_class=CustomCNN,
		features_extractor_kwargs=dict(features_dim=512)
		)
	
	model = PPO.load(file_path, n_steps=2056, batch_size=1024, policy_kwargs=policy_kwargs, verbose=1, print_system_info=True)
	return model


if __name__ == "__main__":
	rospy.init_node("rl_agent_camera_node_descrete")
	
	file_path = os.path.join(rospkg.RosPack().get_path('rl_interact'), 'rl_model', 'best_model_camera')

	model = env_maker(file_path)

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	# sub = rospy.Subscriber("/v4l2_camera/image_processed", Image, callback=callback_data)
	sub = rospy.Subscriber("/front_cam/camera/image", Image, callback=callback_data)

	rospy.spin()
