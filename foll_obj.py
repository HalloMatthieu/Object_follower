#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Twist
from math import copysign, isnan
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import thread


class ObjectFollower:
    def __init__(self):
        rospy.init_node("object_follower")

        # set the shutdonw function to stop the robot
        rospy.on_shutdown(self.shutdown)

        # set the robot's motion update
        self.rate = rospy.get_param("rate", 10)

        r = rospy.Rate(self.rate)

        # use the Region of Interest (roi) of ROS, this offer the possibility
        # to focus on a part of a image
        self.scale_roi = rospy.get_param("scale", 10)

        # Define differents parameters
        # the max linear speed (m/s)
        self.max_linear_speed = rospy.get_param("max_linear_speed", 0.9)

        # the min linear speed (m/s)
        self.min_linear_speed = rospy.get_param("min_linear_speed", 0.02)

        # the maximum rotation speed (rad/s)
        self.max_rot_speed = rospy.get_param("max_rot_speed", 2.0)

        # the min rotation pseed (rad/s)
        self.min_rot_speed = rospy.get_pram("min_rot_speed", 0.5)

        # the following lines define the off-set for the image
        # this define the reaction's distnce
        self.x_treshold = rospy.get_param("x_treshold", 0.1)

        # distance in z axis
        self.z_treshold = rospy.get_param("z_treshold", 0.05)

        # max distance to resct
        self.max_z = rospy.get_param("max_z", 1.6)

        # min distance to react
        self.min_z = rospy.get_param("min_z", 0.5)

        # the goal distance between the robot and the object
        self.goal_z = rospy.get_param("goal_z", 0.75)

        # How much do we weight the goal distance (z) when making a movement
        self.z_scale = rospy.get_param("~z_scale", 0.5)

        # How much do we weight (left/right) of the person when making a movement
        self.x_scale = rospy.get_param("~x_scale", 2.0)

        # Slow down factor when stopping
        self.slow_down_factor = rospy.get_param("~slow_down_factor", 0.8)

        # init global roi
        self.roi = RegionOfInterest()

        # init publisher to command the robot
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=5)

        # init movement cmd
        self.move_cmd = Twist()

        # Get a lock for updating the self.move_cmd values
        self.lock = thread.allocate_lock()

        # We will get the image width and height from the camera_info topic
        self.image_width = 0
        self.image_height = 0

        # We need cv_bridge to convert the ROS depth image to an OpenCV array
        self.cv_bridge = CvBridge()
        self.depth_array = None

        # Set flag to indicate when the ROI stops updating
        self.target_visible = False

        # Wait for the camera_info topic to become available
        rospy.loginfo("Waiting for camera_info topic...")

        rospy.wait_for_message("camera_info", CameraInfo)

        # Subscribe to the camera_info topic to get the image width and height
        rospy.Subscriber("camera_info", CameraInfo, self.get_camera_info, queue_size=1)

        # Wait until we actually have the camera data
        while self.image_width == 0 or self.image_height == 0:
            rospy.sleep(1)

        # Wait for the depth image to become available
        rospy.loginfo("Waiting for depth_image topic...")

        rospy.wait_for_message("depth_image", Image)

        # Subscribe to the depth image
        self.depth_subscriber = rospy.Subscriber(
            "depth_image", Image, self.convert_depth_image, queue_size=1
        )

        # Subscribe to the ROI topic and set the callback to update the robot's motion
        rospy.Subscriber("roi", RegionOfInterest, self.set_cmd_vel, queue_size=1)

        # Wait until we have an ROI to follow
        rospy.loginfo("Waiting for an ROI to track...")

        rospy.wait_for_message("roi", RegionOfInterest)

        rospy.loginfo("ROI messages detected. Starting follower...")

        # Begin the tracking loop
        while not rospy.is_shutdown():
            # Acquire a lock while we're setting the robot speeds
            self.lock.acquire()

            try:
                if not self.target_visible:
                    # If the target is not visible, stop the robot smoothly
                    self.move_cmd.linear.x *= self.slow_down_factor
                    self.move_cmd.angular.z *= self.slow_down_factor
                else:
                    # Reset the flag to False by default
                    self.target_visible = False

                # Send the Twist command to the robot
                self.cmd_vel_pub.publish(self.move_cmd)

            finally:
                # Release the lock
                self.lock.release()

            # Sleep for 1/self.rate seconds
            r.sleep()

    def set_cmd_vel(self, msg):
        # Acquire a lock while we're setting the robot speeds
        self.lock.acquire()

        try:
            # If the ROI has a width or height of 0, we have lost the target
            if msg.width == 0 or msg.height == 0:
                self.target_visible = False
                return
            else:
                self.target_visible = True

            self.roi = msg

            # Compute the displacement of the ROI from the center of the image
            target_offset_x = msg.x_offset + msg.width / 2 - self.image_width / 2

            try:
                percent_offset_x = float(target_offset_x) / (
                    float(self.image_width) / 2.0
                )
            except:
                percent_offset_x = 0

            # Rotate the robot only if the displacement of the target exceeds the threshold
            if abs(percent_offset_x) > self.x_threshold:
                # Set the rotation speed proportional to the displacement of the target
                speed = percent_offset_x * self.x_scale
                self.move_cmd.angular.z = -copysign(
                    max(
                        self.min_rotation_speed,
                        min(self.max_rotation_speed, abs(speed)),
                    ),
                    speed,
                )
            else:
                self.move_cmd.angular.z = 0

            # Now compute the depth component

            # Initialize a few depth variables
            n_z = sum_z = mean_z = 0

            # Shrink the ROI slightly to avoid the target boundaries
            scaled_width = int(self.roi.width * self.scale_roi)
            scaled_height = int(self.roi.height * self.scale_roi)

            # Get the min/max x and y values from the scaled ROI
            min_x = int(
                self.roi.x_offset + self.roi.width * (1.0 - self.scale_roi) / 2.0
            )
            max_x = min_x + scaled_width
            min_y = int(
                self.roi.y_offset + self.roi.height * (1.0 - self.scale_roi) / 2.0
            )
            max_y = min_y + scaled_height

            # Get the average depth value over the ROI
            for x in range(min_x, max_x):
                for y in range(min_y, max_y):
                    try:
                        # Get a depth value in meters
                        z = self.depth_array[y, x]

                        # Check for NaN values returned by the camera driver
                        if isnan(z):
                            continue

                    except:
                        # It seems to work best if we convert exceptions to 0
                        z = 0

                    # A hack to convert millimeters to meters for the freenect driver
                    if z > 100:
                        z /= 1000.0

                    # Check for values outside max range
                    if z > self.max_z:
                        continue

                    # Increment the sum and count
                    sum_z = sum_z + z
                    n_z += 1

            # Stop the robot's forward/backward motion by default
            linear_x = 0

            # If we have depth values...
            if n_z:
                mean_z = float(sum_z) / n_z

                # Don't let the mean fall below the minimum reliable range
                mean_z = max(self.min_z, mean_z)

                # Check the mean against the minimum range
                if mean_z > self.min_z:
                    # Check the max range and goal threshold
                    if mean_z < self.max_z and (
                        abs(mean_z - self.goal_z) > self.z_threshold
                    ):
                        speed = (mean_z - self.goal_z) * self.z_scale
                        linear_x = copysign(
                            min(
                                self.max_linear_speed,
                                max(self.min_linear_speed, abs(speed)),
                            ),
                            speed,
                        )

            if linear_x == 0:
                # Stop the robot smoothly
                self.move_cmd.linear.x *= self.slow_down_factor
            else:
                self.move_cmd.linear.x = linear_x

        finally:
            # Release the lock
            self.lock.release()

    def convert_depth_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            # Convert the depth image using the default passthrough encoding
            depth_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError, e:
            print e

        # Convert the depth image to a Numpy array
        self.depth_array = np.array(depth_image, dtype=np.float32)

    def get_camera_info(self, msg):
        self.image_width = msg.width
        self.image_height = msg.height

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        # Unregister the subscriber to stop cmd_vel publishing
        self.depth_subscriber.unregister()
        rospy.sleep(1)
        # Send an emtpy Twist message to stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


if __name__ == "__main__":
    try:
        ObjectFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Object follower node terminated.")
