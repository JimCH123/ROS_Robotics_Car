#! /usr/bin/env python

import rospy
import actionlib
import math
from ros_project.msg import OdomRecordAction, OdomRecordResult, OdomRecordFeedback
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point32


class RecordOdometry(object):
    def __init__(self):
        
        self._feedback = OdomRecordFeedback()                                 #建立 Action 回傳用的 message 物件 就是個可以裝資料的容器
        self._result = OdomRecordResult()
        self.sub_odom = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.rate = rospy.Rate(1)

        self._as = actionlib.SimpleActionServer("/record_odom", OdomRecordAction, self.goal_callback, False)   #False位置是auto start
        self._as.start()                                                                                       #手動啟動

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.ori_x = msg.pose.pose.orientation.x
        self.ori_y = msg.pose.pose.orientation.y
        self.ori_z = msg.pose.pose.orientation.z
        self.ori_w = msg.pose.pose.orientation.w
        orientation_list = [self.ori_x, self.ori_y, self.ori_z, self.ori_w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.theta = yaw
        

    def goal_callback(self, goal):
        rospy.loginfo("Start Recording Odometry ...")
        odom_list = []
       
        first_reading = True
        if first_reading :
            first_x = self.x
            first_y = self.y
            first_reading = False

        prev_x = first_x
        prev_y = first_y
        
        total_distance = 0
        done_first_lap = False    

        while not done_first_lap:
            point = Point32()
            point.x = self.x
            point.y = self.y
            point.z = self.theta
            odom_list.append(point)

            rospy.loginfo("Start Calculating Distance ...")
            distance = math.sqrt((self.x - prev_x)**2 + (self.y - prev_y)**2)
            total_distance += distance
            prev_x = self.x
            prev_y = self.y

            rospy.loginfo("Current Distance : %f", total_distance)
            self._feedback.current_total = total_distance
            self._as.publish_feedback(self._feedback)

            distance_to_start = math.sqrt((self.x - first_x)**2 + (self.y - first_y)**2)

            if total_distance > 0.5 and distance_to_start < 0.1 :
                rospy.loginfo("Completed one lap")
                self._result.list_of_odoms = odom_list
                self._as.set_succeeded(self._result)
                done_first_lap = True

            self.rate.sleep()

        

if __name__ == '__main__':
    rospy.init_node("action_server_record_odom_node")
    RecordOdometry()
    rospy.spin()
