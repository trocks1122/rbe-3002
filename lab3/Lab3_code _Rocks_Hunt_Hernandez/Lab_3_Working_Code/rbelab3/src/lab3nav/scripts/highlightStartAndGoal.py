#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import GridCells, OccupancyGrid

##
# Handles map event
# @param msg The map
def mapHandler(msg):
    global mapResolution

    mapResolution = msg.info.resolution


##
# Handles initial pose set event
# @param msg The initial pose
def initialPoseHandler(msg):
    gridMessage = GridCells()

    gridMessage.header.frame_id = 'map'

    gridMessage.cell_width = mapResolution
    gridMessage.cell_height = mapResolution

    gridMessage.cells.append(Point(mapResolution*int(round(msg.pose.pose.position.x/mapResolution)),mapResolution*int(round(msg.pose.pose.position.y/mapResolution)),0))

    initialPub.publish(gridMessage)


##
# Handles goal pose set event
# @param msg The goal pose
def goalPoseHandler(msg):
    gridMessage = GridCells()

    gridMessage.header.frame_id = 'map'

    gridMessage.cell_width = mapResolution
    gridMessage.cell_height = mapResolution

    gridMessage.cells.append(Point(mapResolution*int(round(msg.pose.position.x/mapResolution)),mapResolution*int(round(msg.pose.position.y/mapResolution)),0))

    goalPub.publish(gridMessage)


if __name__ == '__main__':
    global intialPub
    global goalPub
    global mapResolution
    
    mapResolution = 1

    try:
        rospy.init_node('highlight_start_and_goal_cells')
        mapSub = rospy.Subscriber("map", OccupancyGrid, mapHandler)

        initialSub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)    
        goalSub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalPoseHandler)
        initialPub = rospy.Publisher('/lab3_visualization/initialViz', GridCells, queue_size=10)
        goalPub = rospy.Publisher('/lab3_visualization/goalViz', GridCells, queue_size=10)

        while not rospy.is_shutdown():
            rospy.spin()
    

    except rospy.ROSInterruptException:
        print "STOPPED PUBLISHING INITIAL AND GOAL POSES TO GRID VISUALIZER"
