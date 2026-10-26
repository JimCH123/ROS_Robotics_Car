#! /usr/bin/env python

#service_server_find_wall.py

import rospy
from ros_project.srv import FindWall, FindWallResponse
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class FindNearestWall(object):
    def __init__(self):
        
        self.find_wall_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.find_wall_move = Twist()
        self.laser_subscriber = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.laser_msg = None
        self.rate = rospy.Rate(3)
        self.find_wall_service = rospy.Service('/find_wall', FindWall, self.find_wall_callback)

    def laser_callback(self, msg):
        self.laser_msg = msg

    def find_shortest_ray_index(self): 
        ranges = list(self.laser_msg.ranges)
        shortest_index = ranges.index(min(ranges))
        rospy.loginfo("Shortest ray index: %d", shortest_index)
        return shortest_index

    def rotating_to_face_wall(self, shortest_index, angle_min, angle_increment, rotate_speed, linear_speed = 0, left_angular_speed = 0.3, right_angular_speed = -0.3):    
        ray_angle = angle_min + (shortest_index * angle_increment)

        start_time = rospy.Time.now().to_sec()
        
        rotate_time = abs(ray_angle) / rotate_speed

        self.find_wall_move.linear.x = linear_speed
        self.find_wall_move.angular.z = left_angular_speed if ray_angle > 0 else right_angular_speed
        
        while rospy.Time.now().to_sec() - start_time < rotate_time:
            rospy.loginfo("Rotating to face wall...")
            self.find_wall_pub.publish(self.find_wall_move)
            self.rate.sleep()

    def moving_to_wall(self, distance_to_wall = 0.3, linear_speed = 0.08, angular_speed = 0.0):
        while not rospy.is_shutdown():
            if self.laser_msg.ranges[360] < distance_to_wall:
                break
            rospy.loginfo("Moving forward to wall...")
            self.find_wall_move.linear.x = linear_speed
            self.find_wall_move.angular.z = angular_speed
            self.find_wall_pub.publish(self.find_wall_move)
            self.rate.sleep()

    def to_right_side_face_wall(self, angle_min, angle_increment, rotate_speed, linear_speed = 0, left_angular_speed = 0.3, right_angular_speed = -0.3):    
        ray_angle = angle_min + (270 * angle_increment)
        rotate_time = abs(ray_angle) / rotate_speed

        start_time = rospy.Time.now().to_sec()
        
        self.find_wall_move.linear.x = linear_speed
        self.find_wall_move.angular.z = right_angular_speed if ray_angle > 0 else left_angular_speed

        while rospy.Time.now().to_sec() - start_time < rotate_time:
            rospy.loginfo("Rotating until robot's right side slightly faces wall...")
            self.find_wall_pub.publish(self.find_wall_move)
            self.rate.sleep()

    def stop_robot(self, linear_speed = 0, angular_speed = 0):
        rospy.loginfo("Wall Founded")
        self.find_wall_move.linear.x = linear_speed
        self.find_wall_move.angular.z = angular_speed
        self.find_wall_pub.publish(self.find_wall_move)

    def find_wall_callback(self, request):

        while self.laser_msg is None:
            self.rate.sleep()

        angle_min = self.laser_msg.angle_min
        angle_increment = self.laser_msg.angle_increment
        rotate_speed = 0.3

        shortest_index = self.find_shortest_ray_index()
        self.rotating_to_face_wall(shortest_index, angle_min, angle_increment, rotate_speed)
        self.moving_to_wall()
        self.to_right_side_face_wall(angle_min, angle_increment, rotate_speed)
        self.stop_robot()

        response = FindWallResponse()
        response.wallfound = True
        return response

if __name__ == '__main__':
    rospy.init_node("service_server_find_wall_node")
    rospy.loginfo("Laser Activitate for Looking Wall")
    robot_find_wall = FindNearestWall()
    rospy.spin()
