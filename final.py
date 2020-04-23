#!/usr/bin/env python

import rospy
import actionlib
import traceback
from simple_navigation_goals import simple_navigation_goals
from TurtleBotMap import *
from move_base_msgs.msg import *
from nav_msgs.srv import GetMap
from math import pi, cos, sin, isnan
from listener import *
from explorer import *
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

if __name__ == "__main__":
    try:
        rospy.init_node("final_prog")
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        rospy.loginfo("SimpleNavigationGoals Initialization")
        nav_goals = simple_navigation_goals.SimpleNavigationGoals()
        rospy.loginfo("Initializations done")
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

        listener = tf.TransformListener()

        cmd_vel = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        goal_number = 0
        target_frame = "map"
        base_link = "base_link"

        rospy.on_shutdown(nav_goals._shutdown)

        # test le temps
        sec = rospy.get_time()
        print("Sec : {}".format(sec))

        # fisrt, start to explore the house
        t0 = rospy.Time(0)
        listener.waitForTransform("map", base_link, t0, rospy.Duration(1))
        ((x, y, z), rot) = listener.lookupTransform("map", base_link, t0)
        euler = tf.transformations.euler_from_quaternion(rot)
        reach_goal(x, y, euler[2] + pi)
        print("G0 done")

        t0 = rospy.Time(0)
        listener.waitForTransform("map", base_link, t0, rospy.Duration(1))
        ((x, y, z), rot) = listener.lookupTransform("map", base_link, t0)
        euler = tf.transformations.euler_from_quaternion(rot)
        reach_goal(x, y, euler[2] + pi)
        print("G1 done")

        turtlebot_map = TurtleBotMap(target_frame, base_link, listener)
        (metadata, mapData) = turtlebot_map.get_map()
        pose_in_im = turtlebot_map.get_image_pose()
        rotate()

        remplissage_diff()
        (x_im, y_im) = find_ppv()

        while not (isnan(x_im) and isnan(y_im)) and not rospy.is_shutdown():
            (x, y, theta) = turtlebot_map.pix_to_pose((x_im, y_im, 0))
            print(x, y, theta)

            reach_goal(x, y, theta)
            rotate()
            (metadata, mapData) = turtlebot_map.get_map()
            pose_in_im = turtlebot_map.get_image_pose()

            remplissage_diff()
            (x_im, y_im) = find_ppv()
        print("cartographie finie")

        # cartographie finie
        # lancement de suiveur d'object (en permanence)
        while 1:
            listener()

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())
    rospy.loginfo("Final node done")
