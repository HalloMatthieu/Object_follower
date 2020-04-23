#!/usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from simple_navigation_goals import simple_navigation_goals
import sys
import select
import os
import time
import traceback
import actionlib
from move_base_msgs.msg import MoveBaseAction
from TurtleBotMap import *
from explorer import *


def callback(data):
    datax = []
    datay = []
    angle_min = data.angle_min
    angle_increment = data.angle_increment
    #    rospy.loginfo(data.ranges)
    rospy.loginfo(len(data.ranges))
    for i in range(len(data.ranges)):
        if np.isinf(data.ranges[i]):
            # deleted useles value
            None

        else:
            angle = angle_min + angle_increment * i
            x = data.ranges[i] * np.cos(angle)
            y = data.ranges[i] * np.sin(angle)
            if x > 0.2 and x < 1 and y > -0.25 and y < 0.25:
                datax.append(x)
                datay.append(y)
    x_moy = 0
    y_moy = 0
    for i in range(len(datax)):
        #        rospy.loginfo("x={}, y ={}".format(datax[i],datay[i]))
        x_moy = x_moy + datax[i]
        y_moy = y_moy + datay[i]
    if len(datax) != 0:
        x_moy = x_moy / len(datax)
        y_moy = y_moy / len(datay)
        rospy.loginfo("x={}, y ={}".format(x_moy, y_moy))
        # on veut ramener le baricentre vers la zone d'interet : x = 0.75 et y = 0
        erreur_x = (
            x_moy - 0.25
        )  # si erreur_x est positif on veut avancer et si erreur_x et negatif on veut reculer
        erreur_y = (
            y_moy - 0
        )  # si erreur_y est et positif il faut tourner vers la gauche et si erreur_y et negatif il faut tourner vers la droite
        k_l = 1
        k_r = 2.5
        input_x = erreur_x * k_l
        input_rot = erreur_y * k_r

        twist = Twist()

        twist.linear.x = input_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = input_rot

        pub.publish(twist)

        # Test if the robot is still moving
        # the purpose is to stop following it after 3s of none moving
        i = 0
        print("valeur de l'erreur en x : {} et valeur de i : {}".format(erreur_x, i))
        while erreur_x <= 0.001:
            t_deb = rospy.Time.from_sec(time.time())
            t0 = t_deb.to_sec()
            print("Sec : {}".format(t0))
            dt = rospy.Duration.from_sec(3)
            d = dt.to_sec()
            print("Duree d'arret : {}".format(d))
            if i >= 1:
                i += 1
                t_now = rospy.Time.from_sec(time.time())
                t1 = t_now.to_sec()
                print(
                    "Seconde depuis debut arret : {}, seconde maintennt : {}".format(
                        t0, t1
                    )
                )
                if t1 - t0 >= d:
                    rotate()
                    remplissage_diff()
                    nav_goals.go_to(-3.0, 1.0, 0.0)
                else:
                    # en attente
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    twist.linear.z = 0.0
                    twist.angular.x = 0.0
                    twist.angular.y = 0.0
                    twist.angular.z = 0.0
                    print("En attente .. ")
                    pub.publish(twist)
            else:
                i += 1

                # cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key
    else:
        rospy.loginfo("item lost !!!!!")
        # twist = Twist()
        # twist.linear.x = 0.0
        # twist.linear.y = 0.0
        # twist.linear.z = 0.0
        # twist.angular.x = 0.0
        # twist.angular.y = 0.0
        # twist.angular.z = 0.0
        # pub.publish(twist)

        # go home
        nav_goals.go_to(-3.0, 1.0, 0.0)

        # rospy.spin()


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("listener", anonymous=False)

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    try:
        rospy.init_node("listener")
        pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        rospy.loginfo("SimpleNavigationGoals Initialization")
        nav_goals = simple_navigation_goals.SimpleNavigationGoals()
        rospy.loginfo("Initializations done")

        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

        rospy.on_shutdown(nav_goals._shutdown)

        # test le temps
        sec = rospy.get_time()
        print("Sec : {}".format(sec))

        status = 0
        target_linear_vel = 0.0
        target_angular_vel = 0.0
        control_linear_vel = 0.0
        control_angular_vel = 0.0

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
    rospy.loginfo("Listener node done")
