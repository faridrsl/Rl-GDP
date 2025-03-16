#!/usr/bin/env python3

import rospy
import rospkg
import os
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


from stable_baselines3 import TD3, PPO
from customcnn import CustomCNN
from cv_bridge import CvBridge

bridge = CvBridge()

def callback_data(image_received):
	# rospy.loginfo("message received")

	#### on Jetson
	# # Convert the ROS Image message to OpenCV format (without using OpenCV)
	# if msg.encoding != "bgr8":
	# 	rospy.logerr(f"Unsupported encoding: {msg.encoding}")
	# 	return

	# # Convert ROS Image to NumPy array
	# height = msg.height
	# width = msg.width
	# image_bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape((height,width,3))

	# # Convert BGR8 to RGB8 by swapping channels
	# image_rgb = np.ascontiguousarray(image_bgr[..., ::-1]) # else use [:,:,[2,1,0]]
	# #rospy.loginfo(f"Received BFR image, converted to RGB with shape: {image_rgb.shape}")
	# # image_rl_inpput = image_rgb	
	
	cv_image = bridge.imgmsg_to_cv2(image_received, "rgb8")
	# cv2.imshow("Received Image", cv_image)
	# cv2.waitKey(1)
	image_rl_inpput = cv_image

	action,_ = model.predict(observation=image_rl_inpput, deterministic=True)

	cmd = Twist()
	cmd.linear.x = action[0]
	cmd.linear.y = action[1]
	cmd.linear.z = 0.0
	cmd.angular.x = 0.0
	cmd.angular.y = 0.0
	cmd.angular.z = action[2]
	pub.publish(cmd)

	# rospy.loginfo(cmd)

def env_maker(file_path):
    
	model = TD3.load(file_path)
	return model


if __name__ == "__main__":
	rospy.init_node("rl_agent_camera_node")
	
	file_path = os.path.join(rospkg.RosPack().get_path('rl_interact'), 'rl_model', 'best_model_camera')

	model = env_maker(file_path)

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	# sub = rospy.Subscriber("/v4l2_camera/image_processed", Image, callback=callback_data)
	sub = rospy.Subscriber("/front_cam/camera/image", Image, callback=callback_data)

	rospy.spin()
