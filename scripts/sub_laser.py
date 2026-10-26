#! /usr/bin/env python

import rospy
import time
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from ros_project.srv import FindWall, FindWallRequest
from ros_project.msg import OdomRecordAction, OdomRecordGoal
from actionlib_msgs.msg import GoalStatus


class KobukiMovingSquare(object):
    def __init__(self):
        self.laser_subscriber = rospy.Subscriber("/scan", LaserScan, callback = self.laser_callback)
        self.vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
        self.cmd = Twist()
        self.rate = rospy.Rate(3)        #頻率設3 一秒執行三次 之後可以用於rate.sleep()
        self.laser_msg = None

        rospy.wait_for_service("/find_wall")                                     #程式卡著 等待server啟動/find_wall
        self.find_wall_service = rospy.ServiceProxy("/find_wall", FindWall)      #建立與service的連線代理

        self.action_client_record_odom = actionlib.SimpleActionClient("/record_odom", OdomRecordAction)
        self.action_client_record_odom.wait_for_server()

    def call_find_wall_service(self):
        try :
            find_wall = FindWallRequest()
            result = self.find_wall_service(find_wall)
            return result.wallfound
        except rospy.ServiceException as e :
            rospy.logerr("Service call failed: %s", e)
            return False

    def call_record_odom_action(self):
        action_goal = OdomRecordGoal()
        self.action_client_record_odom.send_goal(action_goal, feedback_cb=self.action_feedback_callback)
        

    def action_feedback_callback(self, feedback):
        rospy.loginfo(feedback)

    def publish_once_in_cmd_vel(self):
        while not rospy.is_shutdown():
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                rospy.loginfo("Cmd Published")
                self.rate.sleep()
                break
            else:
                self.rate.sleep()

    def moving_forward(self, linear_speed = 0.08, angular_speed = 0):
        rospy.loginfo("Robot Moving Forward ...")
        self.cmd.linear.x = linear_speed
        self.cmd.angular.z = angular_speed
        self.publish_once_in_cmd_vel()

    def detect_wall_right(self, linear_speed = 0.08, right_angular_speed = -0.1):
            rospy.loginfo("Robot Turning Right a Little Bit ...")
            self.cmd.linear.x = linear_speed
            self.cmd.angular.z = right_angular_speed
            self.publish_once_in_cmd_vel()
    
    def detect_wall_left(self, linear_speed = 0.08, left_angular_speed = 0.2):
            rospy.loginfo("Robot Turning Left a Little Bit ...")
            self.cmd.linear.x = linear_speed
            self.cmd.angular.z = left_angular_speed
            self.publish_once_in_cmd_vel()           

    def turning_left(self, linear_speed = 0.08, angular_speed = 0.5):
            rospy.loginfo("Robot Turning Left ...")
            self.cmd.linear.x = linear_speed
            self.cmd.angular.z = angular_speed
            self.publish_once_in_cmd_vel()

    def stop_robot(self, linear_speed = 0, angular_speed = 0):
        stop_cmd = Twist()  
        stop_cmd.linear.x = linear_speed
        stop_cmd.angular.z = angular_speed
        for _ in range(5):  
            rospy.loginfo("Robot Stopping ...")
            self.vel_publisher.publish(stop_cmd)
            time.sleep(0.1)

    def move_robot(self):
        rospy.loginfo("Robot Start Moving ...")
        while not rospy.is_shutdown():                               #只要還沒收到 /scan，就一直跳過 直到收到第一筆self.laser_msg
            if self.laser_msg is None:
                self.rate.sleep()
                continue
            
            goal_status = self.action_client_record_odom.get_state()
            if goal_status == GoalStatus.SUCCEEDED:
                rospy.loginfo("Complete One Lap !!!")
                result = self.action_client_record_odom.get_result()
                rospy.loginfo(result)
                self.stop_robot()
                break
            
            front = self.laser_msg.ranges[360]
            right = self.laser_msg.ranges[180]

            if front <= 0.5:
                self.turning_left()
            elif right >= 0.3:
                self.detect_wall_right()
            elif right <= 0.2:
                self.detect_wall_left()
            else:
                self.moving_forward()

    def laser_callback(self, msg):   #機器人前方是ranges[360] 右邊是[180] 左邊是[540] 後方是[0]  
        self.laser_msg = msg
        

if __name__ == '__main__':        
    rospy.init_node("sub_laser_node")
    robot = KobukiMovingSquare()
    if robot.call_find_wall_service():
        rospy.loginfo("Robot Start Recording Odometry ...")
        robot.call_record_odom_action()
        rospy.loginfo("Robot Start Following the Wall ...")
        robot.move_robot()

        