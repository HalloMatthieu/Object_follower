#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import std_msgs.msg
import tf


class InitException(Exception):
    pass


class get_map:
    def __init__(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

    def get_map(self):
        # The map is accessed by calling the service dynamic_map()
        get_map = rospy.ServiceProxy("dynamic_map", GetMap)
        # Request the map as well as the metadata
        m = get_map().map
        width = m.info.width
        height = m.info.height
        res = m.info.resolution
        origin = m.info.origin
        data = m.data
        self.metadata = (width, height, res)  # metadata pour la map
        x_origin = origin.position.x
        y_origin = origin.position.y
        theta_origin = (
            tf.transformations.euler_from_quaternion(
                (
                    origin.orientation.x,
                    origin.orientation.y,
                    origin.orientation.z,
                    origin.orientation.w,
                )
            )
        )[2]
        self.pose_origin = (x_origin, y_origin, theta_origin)  # origine de la map

        # we also need to know where we are
        t0 = rospy.Time(0)
        self.listener.waitForTransform("map", self.base_link, t0, rospy.Duration(1))
        ((x_robot, y_robot, z), rot) = self.listener.lookupTransform(
            "map", self.base_link, t0
        )
        euler = tf.transformations.euler_from_quaternion(rot)
        theta_robot = euler[2]
        self.pose_robot = (x_robot, y_robot, theta_robot)  # position du robot

        # mapData is the symmetric copy of the data in order to be able to modify the values. mapData is represented by  a row major order method.
        self.mapData = list(data)
        for i in range(height):
            for j in range(width):
                self.mapData[i * width + j] = data[(height - 1 - i) * width + j]

        return (self.metadata, self.mapData)

    def pose_to_pix(self, pose_robot):
        w = self.metadata[0]
        h = self.metadata[1]
        res = self.metadata[2]
        # We first convert pose_robot from the "map" frame to the image frame
        x_robot, y_robot, theta_robot = pose_robot
        x_origin, y_origin, theta_origin = self.pose_origin
        xr_in_im = (y_origin - y_robot) / res + h
        yr_in_im = (x_robot - x_origin) / res
        # And apply a rotation
        theta_in_im = theta_robot - theta_origin
        return (int(xr_in_im), int(yr_in_im), theta_in_im)

    def is_accessible(
        x, y, rayon_inflate=4
    ):  # Renvoie True si l'on n'est pas près d'un mur
        width = metadata[0]
        if mapData[x * width + y] == 0:
            # On regarde si l'on n'est pas près d'un mur (costmap)
            for m in range(-rayon_inflate, rayon_inflate + 1):
                for n in range(-rayon_inflate, rayon_inflate + 1):
                    if mapData[(x + m) * width + (y + n)] == 100:
                        return False
            return True
        return False

    def remplissage_diff():  # Diffuse la zone d'accessibilité en prenant en compte l'espacement des murs
        pile = []
        x_robot = pose_in_im[0]  # Position du robot
        y_robot = pose_in_im[1]
        for i in range(-3, 4):
            for j in range(-3, 4):
                # On définit tous pixels autours du robot comme accessible pour que l'algorithme puisse toujours se lancer
                pile.append([x_robot + i, y_robot + j])
        print("Analyse des zones accessibles")
        while pile != []:
            [x, y] = pile.pop()
            width = metadata[0]
            height = metadata[1]
            # La valeur arbitraire "-2" correspond à une zone accessible pour le robot
            mapData[x * width + y] = -2
            for k in range(-1, 2):
                for l in range(
                    -1, 2
                ):  # Pour chaque pixels adjacents au pixel actuel, on regarde si la zone est accessible
                    if k != 0 or l != 0:
                        if is_accessible(x + k, y + l):
                            pile.append([x + k, y + l])
            print("Analyse terminée")

    return

    def is_free(x, y):
        width = metadata[0]
        if mapData[x * width + y] == -2:
            for k in range(-1, 2):
                for l in range(-1, 2):  # On regarde les cellules adjacentes
                    if k != 0 or l != 0:
                        if (
                            mapData[(x + k) * width + (y + l)] == -1
                        ):  # Si une de ces cellules adjacentes est inconnue
                            print("Bordure trouvée")
                            return True
                    return False
        else:
            return False


def reach_goal(x, y, theta):
    target_quat = tf.transformations.quaternion_from_euler(0, 0, theta)
    t0 = rospy.Time.now()
    goal = MoveBaseGoal()

    goal.target_pose.header.seq = goal_number
    goal.target_pose.header.stamp = t0
    goal.target_pose.header.frame_id = target_frame
    goal.target_pose.pose.position = Point(x, y, 0)
    goal.target_pose.pose.orientation.x = target_quat[0]
    goal.target_pose.pose.orientation.y = target_quat[1]
    goal.target_pose.pose.orientation.z = target_quat[2]
    goal.target_pose.pose.orientation.w = target_quat[3]

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    client.wait_for_result()
    print(client.get_result())

    # Prints out the result of executing the action
    return client.get_result()


def rotate(vitesse=1):
    twist = Twist()
    twist.linear.x = 0
    twist.angular.z = vitesse  # vitesse en radians par secondes
    rate = rospy.Rate(10)
    timer = 0
    # Demarrage de la rotation
    while (
        timer < 2 * pi / vitesse
    ):  # On doit envoyer des messages de manière persistance pour que le robot continue de tourner
        cmd_vel.publish(twist)
        rate.sleep()
        timer = timer + 0.1  # Fréquence de 10 Hz donc 0.1 secondes


def whenshutdown():
    rospy.loginfo("Stop node")
    width = metadata[0]
    height = metadata[1]
    image_array = np.zeros((height, width, 3), dtype=int)

    # Plotting the map
    for i in range(height):
        for j in range(width):
            if mapData[i * width + j] == -1:  # Unknown
                image_array[i, j, 0] = 255
                image_array[i, j, 1] = 255
                image_array[i, j, 2] = 255
            elif mapData[i * width + j] == 0 or mapData[i * width + j] == -2:  # Free
                image_array[i, j, 0] = 125
                image_array[i, j, 1] = 125
                image_array[i, j, 2] = 125
            elif mapData[i * width + j] == 100:  # Walls
                image_array[i, j, 0] = 0
                image_array[i, j, 1] = 0
                image_array[i, j, 2] = 0

    # Plotting the location of the robot
    for i in range(-3, 4):
        for j in range(-3, 4):
            image_array[pose_in_im[0] + i, pose_in_im[1] + j] = (255, 0, 0)

    # Plotting its orientation
    for i in range(10):
        image_array[
            int(pose_in_im[0] + i * sin(-pose_in_im[2])),
            int(pose_in_im[1] + i * cos(-pose_in_im[2])),
        ] = (0, 0, 255)
        scipy.misc.imsave("map.png", image_array)
