#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import sys
import select
import os


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
            if x > 0.5 and x < 1 and y > -0.5 and y < 0.5:
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
            0.75 - x
        )  # si erreur_x est positif on veut avancer et si erreur_x et negatif on veut reculer
        erreur_y = (
            0 - y
        )  # si erreur_y est et positif il faut tourner vers la gauche et si erreur_y et negatif il faut tourner vers la droite
        k_l = 1
        k_r = 1
        input_x = erreur_x * k_l
        input_rot = erreur_y * k_r
        twist = Twist()

        twist.linear.x = input_x
        twist.linear.y = 0.0
        twist.linear.z = input_rot

        pub.publish(twist)

    # cmd_vel(erreur_x * k_l,erreur_y * k_r ) # to_do envoyer ces vitesses en s'inspirant du teleop_key
    else:
        rospy.loginfo("item lost !!!!!")


def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (
        target_linear_vel,
        target_angular_vel,
    )


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("/scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    rospy.init_node("listerner")
    pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

    status = 0
    target_linear_vel = 0.0
    target_angular_vel = 0.0
    control_linear_vel = 0.0
    control_angular_vel = 0.0

    try:
        while 1:
            listener()

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)