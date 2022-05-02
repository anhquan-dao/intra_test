#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import OccupancyGrid

import intra_test.SHMTransportCPP as SHMTransportCPP
import moveit_ros_planning_interface._moveit_roscpp_initializer as moveit

import sys
import numpy as np
import cv2
import time
from io import BytesIO

count = 0

MAP_RESOLUTION = 0.1  #Unit: Meter
MAP_SIZE       = 2     #Unit: Meter, Shape: Square with center "base_link"

test_img = cv2.imread("test_img.png")
cv2.imshow("img", test_img)
cv2.waitKey(0)

map_img = cv2.cvtColor(test_img, cv2.COLOR_BGR2GRAY)
map_img /= 2
print(np.max(map_img))
print(map_img.shape)

cv2.imshow("grey img", map_img)
cv2.waitKey(0)

occupancy_grid = map_img.flatten()
occupancy_grid = occupancy_grid.tolist()
for i in range(len(occupancy_grid)):
    if i < 2500 or i > 20000:
        occupancy_grid[i] = 100
    else:
        occupancy_grid[i] = 0

map_msg = OccupancyGrid()

map_msg.header.frame_id = "sensor"

map_msg.info.height = map_img.shape[0]     #Unit: Pixel
map_msg.info.width  = map_img.shape[1]      #Unit: Pixel
map_msg.info.resolution = 1

map_msg.info.origin.position.x = 5     #Unit: Meter
map_msg.info.origin.position.y = 2     #Unit: Meter
map_msg.info.origin.position.z = 0

map_msg.info.origin.orientation.x = 0
map_msg.info.origin.orientation.y = 0
map_msg.info.origin.orientation.z = 0.2588190451
map_msg.info.origin.orientation.w = 0.96592582628

map_msg.data = map_img.flatten().tolist()

if __name__ == "__main__": 
    rospy.init_node("shm_py_test")  
    moveit.roscpp_init("shm_py_test", rospy.myargv(sys.argv))

    test_pub = SHMTransportCPP.PublisherOccupancyGridSHM("shm_map")

    rate = rospy.Rate(20);
    buff = BytesIO
    count = 0
    while(not rospy.is_shutdown()):
        start = time.time()
    
        end = time.time()
        test_pub.publish(map_msg)
        # count += 1;
        
        
        rate.sleep()
    
    moveit.roscpp_shutdown()
