#!/usr/bin/env python

import rospy
import actionlib
import tf
import sensor_msgs
from sensor_msgs.msg import LaserScan
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import (
    MoveBaseAction,
    MoveBaseGoal,
    MoveBaseActionFeedback,
    MoveBaseActionGoal,
)
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose


class InitException(Exception):
    pass


class FollowObject:
    """Interface with the move_base actionlb. Allows to set a position goal (x, y, theta) that will be followed by a robot, using a predefined GlobalPlanner and LocalPlanner

    Raises:
        InitException --
    """

    def __init__(self, hmi_wrapper=None):
        rospy.loginfo("Starting Follow_Object")

        # Using client, so creating client
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        # Telling what's going to happen
        rospy.loginfo("Waiting for server...")
        # Wait 5s, if no server responding, existing
        success = self.move_base.wait_for_server(rospy.Duration(5))
        if not (success):
            rospy.logerr("The server did not respond in time")
            raise InitException("Failed")
        rospy.Subscriber("move_base/feedback", MoveBaseActionFeedback, self._feedback)
        sub = rospy.Subscriber("/scan", LaserScan, self.update_scan)

        # Define scan's updates

        # Defines the goal position
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.stamp = rospy.Time.now()
        # Set the map
        self.goal.target_pose.header.frame_id = "map"

        # Save lthe last position (default : 0 0 0)
        self.feedback_pose = [0, 0, 0]

        # self.remaining_distance = -1
        # rospy.Subscriber(
        #     "/move_base/GlobalPlanner/plan", Path, self.compute_remaining_distance
        # )

        # if hmi_wrapper == None:
        #     rospy.logwarn("hmi_wrapper NOT SPECIFIED")
        # self.hmi = hmi_wrapper

    def update_scan(self, data):
        self.scan = data
        # self.scan.ranges = round(self.scan.ranges, 4)
        self.scan.angle_min = -0.21
        self.scan.angle_max = 0.21
        self.scan.range_min = 1.5
        self.scan.range.max = 2.5

    def _shutdown(self):
        rospy.loginfo("Cancel all")
        self.move_base.cancel_all_goals()

    def _feedback(self, feedback):
        data = feedback.feedback.base_position.pose

        self.feedback_pos[0] = data.position.x
        self.feedback_pos[1] = data.position.y
        quaternion = (
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.x,
        )
        self.feedback_pos[2] = tf.transformations.euler_from_quaternion(quaternion)[2]
        rospy.logdebug(
            "x:{}, y:{}, theta:{}".format(
                self.feedback_pose[0], self.feedback_pos[1], self.feedback_pos[2]
            )
        )

    # def _compute_remaining_distance(self, path):
    #     tmp = 0
    #     i = 1
    #     if len(path.poses) < 1:
    #         tmp = -1
    #     while i < len(path.poses):
    #         tmp += math.sqrt(
    #             math.pow(
    #                 path.poses[i - 1].pose.position.x - path.poses[i].pose.position.x, 2
    #             )
    #             + math.pow(
    #                 path.poses[i - 1].pose.position.y - path.poses[i].pose.position.y, 2
    #             )
    #         )
    #         i += 1
    #     self.remaining_distance = tmp

    def is_arrived(self):
        state = self.move_base.get_state()
        rospy.loginfo("State = {}".format(state))
        if state == GoalStatus.PENDING or state == GoalStatus.ACTIVE:
            return False
        return True

    def cancel_goal(self):
        if self.hmi is not None:
            self.hmi.blanck()
        self.move_base.cancel_goals()

    def cancel_all_goals(self):
        if self.hmi is not None:
            self.hmi.blank()
        self.move.base.cancel_all_goals()

    # The object size is : 0.3 m.
    # as we would like the visible selection is almost the same as
    # the object, we define that the selection size should be
    # 0.5x0.5
    # and the purpose it the robot should be at a distance of 2m
    # of the object
    # Moreover, the object should be in the middle of the selection
    # so the min and max readable selection should be :
    # from 1.75m to 2.25m
    # and the angle of view is 13 degrees or 0.23 rad
    # zone :
    # D H L P
    # C G K O
    # B F J N
    # A E I M
    #
    def go_to(self, frame="map", blocking=True):
        # Test si object in front
        i = -0.12
        # define the first line
        while i <= -0.06:
            if self.scan.ranges[i] > 1.75 & selg.scan.ranges[i] < 1.875:
                zoneA = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 1.875 & selg.scan.ranges[i] < 2:
                zoneB = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2 & selg.scan.ranges[i] < 2.125:
                zoneC = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2.125 & selg.scan.ranges[i] < 2.25:
                zoneD = 1
                i += 0.01
            else:
                i += 0.01

        while i > -0.06 & i <= 0:
            if self.scan.ranges[i] > 1.75 & selg.scan.ranges[i] < 1.875:
                zoneE = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 1.875 & selg.scan.ranges[i] < 2:
                zoneF = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2 & selg.scan.ranges[i] < 2.125:
                zoneG = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2.125 & selg.scan.ranges[i] < 2.25:
                zoneH = 1
                i += 0.01
            else:
                i += 0.01

        while i > 0 & i <= 0.06:
            if self.scan.ranges[i] > 1.75 & selg.scan.ranges[i] < 1.875:
                zoneI = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 1.875 & selg.scan.ranges[i] < 2:
                zoneJ = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2 & selg.scan.ranges[i] < 2.125:
                zoneK = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2.125 & selg.scan.ranges[i] < 2.25:
                zoneL = 1
                i += 0.01
            else:
                i += 0.01

        while i > 0.06 & i <= 0.12:
            if self.scan.ranges[i] > 1.75 & selg.scan.ranges[i] < 1.875:
                zoneM = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 1.875 & selg.scan.ranges[i] < 2:
                zoneN = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2 & selg.scan.ranges[i] < 2.125:
                zoneO = 1
                i += 0.01
            else:
                i += 0.01

            if self.scan.ranges[i] > 2.125 & selg.scan.ranges[i] < 2.25:
                zoneP = 1
                i += 0.01
            else:
                i += 0.01

        # TODO : Add cercle recognizer
        if zoneA == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.09

            theta = self.feedback_pos[2] - 2.75
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneB == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.09

            theta = self.feedback_pos[2] - 2.66
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneC == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.09

            theta = self.feedback_pos[2] - 2.5
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneD == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.09

            theta = self.feedback_pos[2] - 2.35
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneE == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.03

            theta = self.feedback_pos[2] - 0.95
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneF == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.03

            theta = self.feedback_pos[2] - 0.89
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneG == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.03

            theta = self.feedback_pos[2] - 0.83
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneH == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] + 0.03

            theta = self.feedback_pos[2] - 0.78
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneI == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.03

            theta = self.feedback_pos[2] + 0.95
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneJ == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.03

            theta = self.feedback_pos[2] + 0.89
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneK == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.03

            theta = self.feedback_pos[2] + 0.83
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneL == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.03

            theta = self.feedback_pos[2] + 0.78
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneM == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.09

            theta = self.feedback_pos[2] + 2.75
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneN == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] - 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.09

            theta = self.feedback_pos[2] + 2.66
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneO == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.125
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.09

            theta = self.feedback_pos[2] + 2.5
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        elif zoneP == 1:
            self.goal.target_pose.pose.position.x = self.feedback_pos[0] + 0.1875
            self.goal.target_pose.pose.position.x = self.feedback_pos[1] - 0.09

            theta = self.feedback_pos[2] + 2.35
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)
            self.goal.target_pose.pose.orientation.x = quat[0]
            self.goal.target_pose.pose.orientation.y = quat[1]
            self.goal.target_pose.pose.orientation.z = quat[2]
            self.goal.target_pose.pose.orientation.w = quat[3]

        self.move_base.send_goal(self.goal)


if __name__ == "__main__":
    try:
        rospy.init_node("follow_object", anonymous=False)

        nav_goals = FollowObject()
        rospy.on_shutdown(nav_goals._shutdown)

        while True:
            pass

        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr(traceback.format_exc())

    ropy.loginfo("Follow object node terminated")
