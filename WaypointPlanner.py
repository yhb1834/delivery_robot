#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from math import pi
import tf
from tf2_geometry_msgs import PoseStamped

class WaypointPlanner:
    def __init__(self):
        self.waypoints = []
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        #self.imu_sub = rospy.Subscriber('/handsfree/imu', Imu, self.get_imu)
        #self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.current_head = pi / 2 #-90.0 * 3.14 / 180.0
        

    def add_waypoint(self, x, y, theta):
        pose =  PoseStamped()
        pose.header.frame_id = 'map'

        ### PoseStamped
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)
       

    def execute_waypoints(self):
        print(self.waypoints)
        for waypoint in self.waypoints:
            rospy.sleep(5) # Wait for 5 seconds for robot to reach the goal
            rospy.loginfo('Moving to waypoint: {}'.format(waypoint))
            self.publisher.publish(waypoint)
            rospy.loginfo("Wait for 5s for robot to reach the goal")
            rospy.sleep(5) # Wait for 5 seconds for robot to reach the goal


    def odom_callback(self, msg):
        current_position = msg.pose.pose.position
        current_orientation = msg.pose.pose.orientation
        rospy.loginfo("Current position: (%f, %f, %f)", current_position.x, current_position.y, current_position.z)
        #rospy.loginfo("Current orientation: (%f, %f, %f, %f)", current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w)
        # Make messages saved and prompted in 5Hz rather than 100Hz


if __name__ == '__main__':
    rospy.init_node('waypoint_planner')
    planner = WaypointPlanner()

    # Add waypoints
    planner.add_waypoint(5.0, 0.0, 0.0)
    planner.add_waypoint(10.0, 0.0, 3.0)  
    planner.add_waypoint(-3.0, 0.0, 0.0)
    #planner.add_waypoint(0.0, 0.0, 0.0)

    # Execute waypoints
    planner.execute_waypoints()

    rospy.loginfo('Waypoints completed!')
