#!/usr/bin/env python2
# -*- coding: utf-8 -*-
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

import rospy

#import time
from pytz import timezone
from datetime import datetime

from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped

from math import *
import numpy as np

import pymysql

address_information = {"801":[24.0, -0.3], "802":[17.0, -0.3], "803":[8.0, 0.5], 
                        "804":[1.0, 0.0], "805":[-5.0, -0.5], "806":[2.4, -6.0], 
                        "807":[2.3, -23.0], "808":[3.0, -31.5], "809":[3.0, -40.0]}

max_delivery = 2

ocr_address = set()
address = []
first_address_point = []
address_point = []
final_address = []
final_address_point = []

current_pose_x = 0.0
current_pose_y = 0.0


class WaypointPlanner:
    def __init__(self):
        self.waypoints = []
        self.publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('/ocr_result', Int32, self.update_ocr)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        #self.current_head = pi / 2 #-90.0 * 3.14 / 180.0
    
    def update_ocr(self, msg):
        self.result = msg.data
        print(type(self.result))
        rospy.loginfo(rospy.get_caller_id() + 'I heard %s', self.result)

        global ocr_address
        if len(ocr_address) == max_delivery:
            self.subscriber.unregister()
        else:
            ocr_address.add(str(self.result))

    def add_waypoint(self, x, y, theta):
        pose = PoseStamped()
        pose.header.frame_id = 'map'

        ### PoseStamped
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = theta
        pose.pose.orientation.w = 1.0
        self.waypoints.append(pose)
       

    def execute_waypoints(self, num):
        '''
        # run all waypoints
        for waypoint in self.waypoints:
            rospy.sleep(5) # Wait for 5 seconds for robot to reach the goal
            rospy.loginfo('Moving to waypoint: {}'.format(waypoint))
            self.publisher.publish(waypoint)
            rospy.loginfo("Wait for 5s for robot to reach the goal")
            rospy.sleep(15) # Wait for 5 seconds for robot to reach the goal
        '''
        rospy.sleep(3) # Wait for 5 seconds for robot to reach the goal
        rospy.loginfo('Moving to waypoint: {}'.format(self.waypoints[num]))
        self.publisher.publish(self.waypoints[num])
        rospy.loginfo("Wait for 5s for robot to reach the goal")
        rospy.sleep(45) # Wait for x seconds for robot to reach the each goal


def shortest_way(current_room_x, current_room_y, address_list):
    shortest_dist = 10000000
    next_room_x = 0.0
    next_room_y = 0.0
    room = 0
    turn = 0
    for i in address_list:
        #distance = sqrt((i[0]-current_room[0])*(i[0]-current_room[0])+(i[1]-current_room[1])*(i[1]-current_room[1]))
        distance = np.sqrt((i[0]-current_room_x)*(i[0]-current_room_x)+(i[1]-current_room_y)*(i[1]-current_room_y))
        if distance < shortest_dist:
            room = turn
            next_room_x = i[0]
            next_room_y = i[1]
            shortest_dist = distance
        turn += 1
    
    address_list = address_list[:room] + address_list[room+1:]
    return next_room_x, next_room_y, address_list


if __name__ == '__main__':
    rospy.init_node('Delivery', anonymous=True)
    planner = WaypointPlanner()

    while True:
        if len(ocr_address) == max_delivery:
           break

    # change set into list
    address = list(ocr_address)

    ###################################### Path Planning ######################################
    if 1 <= len(address) <= 5:
        # Matching the room number(from ocr) and distance 
        for addr in address:
            try:
                address_point.append(address_information.get(addr))
            except:
                pass
        
    point_num = len(address_point)
        
    ################################### Shortest waypoint #####################################
    while True:
        next_room_x = 0.0
        next_room_y = 0.0
        next_room = ""
        next_room_x, next_room_y, address_point = shortest_way(next_room_x, next_room_y, address_point)
            
        next_room_point = [next_room_x,next_room_y]
        final_address_point.append(next_room_point)

        if point_num == len(final_address_point):
            break
        
    bb = {tuple(v):k for k,v in address_information.items()}
    for i in final_address_point:
        final_address.append(bb.get(tuple(i)))


    ###################################### Start delivery ###########################################
    for i in range(len(final_address_point)):
            
        ############################ Start 1st point #########################
        planner.add_waypoint(final_address_point[i][0], final_address_point[i][1], 0.0)
        planner.execute_waypoints(i)
            
        ########################### Insert Query #############################
        right_now = datetime.now(timezone('Asia/Seoul'))
        rospy.loginfo(right_now)
        con = pymysql.connect(host='ec2-54-180-8-155.ap-northeast-2.compute.amazonaws.com', port=13307, user='mobilio' ,password='mobilio', db='mobilio', charset='utf8')
        cursor = con.cursor()
        sql = "INSERT INTO delivery_list (departure_time) VALUES (%s) WHERE apartment = %s"
        
        #cursor.execute(sql, (right_now, final_address[i]))
            
        con.commit()

        ##################### User get package (Arrival Time) ################
        sql2 = "SELECT arrival_time FROM delivery_list WHERE apartment = %s"

        while rospy.sleep(15):
            cursor.execute(sql2, final_address[i])
            
            res = cursor.fetchall()
            print("select :", res)
                
            if res == "null":
                print("res : ", res)
                break
            
        con.commit()
        con.close() 
        
    ####################################### Return ###################################################
    planner.add_waypoint(0.0, 0.0, 0.0)
    planner.execute_waypoints(len(final_address_point))        
    rospy.loginfo('Completed!')
    
