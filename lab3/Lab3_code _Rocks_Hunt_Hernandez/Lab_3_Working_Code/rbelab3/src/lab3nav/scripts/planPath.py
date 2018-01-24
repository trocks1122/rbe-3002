#!/usr/bin/env python

import rospy
import threading
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from time import sleep
from tf.transformations import euler_from_quaternion
from math import sin, cos, sqrt
from Queue import PriorityQueue

##
# Handles map event
# @param msg The map
def mapHandler(msg):
    global freshMap
    global occupancyGrid
    occupancyGrid = msg
    freshMap = True

##
# Handles initial pose set event
# @param msg The initial pose
def initialPoseHandler(msg):
    global freshInitialPose
    global initialPose
    initialPose = msg.pose.pose
    freshInitialPose = True
    pubGrid(initialPub, [poseToMapCoordinates(initialPose, occupancyGrid.info.resolution, occupancyGrid.info.origin)], occupancyGrid.info.resolution, occupancyGrid.info.origin)

##
# Handles goal pose set event
# @param msg The goal pose
def goalPoseHandler(msg):
    global freshGoalPose
    global goalPose

    goalPose = msg.pose
    freshGoalPose = True
    pubGrid(goalPub, [poseToMapCoordinates(goalPose, occupancyGrid.info.resolution, occupancyGrid.info.origin)], occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
##
# Publishes a grid visualization
# @param publisher The publisher to use
# @param cells The set of cells to highlight, referenced within the rows and columns
#              of an occupancy grid as a list: [[row,column],[row,column], etc...]
# @param resolution The map resolution
# @param origin The map origin pose
def pubGrid(publisher, cells, resolution, origin):
    
    gridMessage = GridCells()
    gridMessage.header.frame_id = 'map'
    gridMessage.cell_width=resolution
    gridMessage.cell_height=resolution
    for cell in cells:
        localX = cell[0]*resolution+resolution/2
        localY = cell[1]*resolution+resolution/2
        
        theta = euler_from_quaternion([origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w])[2]
        
        x = localX*cos(theta)-localY*sin(theta)
        y = localX*sin(theta)+localY*cos(theta)
        
        x += origin.position.x
        y += origin.position.y
        
        gridMessage.cells.append(Point(x,y,0))

    publisher.publish(gridMessage)

##
# Publishes a path visualization
# @param publisher The publisher to use
# @param cells The path to highlight, referenced within the rows and columns
#              of an occupancy grid as a list: [[row,column],[row,column], etc...]
# @param resolution The map resolution
# @param origin The map origin pose
def pubPath(publisher, cells, resolution, origin):
    
    pathMessage = Path()
    pathMessage.header.frame_id = 'map'
    for cell in cells:
        localX = cell[0]*resolution+resolution/2
        localY = cell[1]*resolution+resolution/2
        
        theta = euler_from_quaternion([origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w])[2]
        
        x = localX*cos(theta)-localY*sin(theta)
        y = localX*sin(theta)+localY*cos(theta)
        
        x += origin.position.x
        y += origin.position.y

        pose = PoseStamped()
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pathMessage.poses.append(pose)

    publisher.publish(pathMessage)

##
# Gets the linear distance between two int points
# @param pointA The first point as a tuple of two ints
# @param pointB The second point as a tuple of two ints
# @return The linear distance between the two input points
def getDistance(pointA, pointB):
    return sqrt((pointA[0]-pointB[0])*(pointA[0]-pointB[0])+(pointA[1]-pointB[1])*(pointA[1]-pointB[1]))

##
# Converts an occupancy grid into a two dimensional list
# @param occupancyGrid The grid to convert
# @return The occupancyGrid converted to a list of row lists
def mapToList(occupancyGrid):
    gridList = []
    
    # For each row
    for rowNumber in range(occupancyGrid.info.height):
        # Insert a row list
        gridList.append([])
        for columnNumber in range(occupancyGrid.info.width):
            # Index off the first element of the row, adding each element to the row list
            gridList[rowNumber].append(occupancyGrid.data[columnNumber*occupancyGrid.info.height + rowNumber])
            
    return gridList

##
# Converts a pose from the real-world into the occupancy grid (dimensionless) universe
# @param pose The pose to convert
# @param resolution The resolution of the map
# @param origin The map origin pose
# @return The pose converted into the nearest occupancy grid location in tuple format: (row, column)
def poseToMapCoordinates(pose, resolution, origin):
    x = pose.position.x - origin.position.x;
    y = pose.position.y - origin.position.y;
    
    theta = euler_from_quaternion([origin.orientation.x,origin.orientation.y,origin.orientation.z,origin.orientation.w])[2]
    
    localX = x*cos(-theta) - y*sin(-theta)
    localY = x*sin(-theta) + y*cos(-theta)
    
    localX = int(round((localX - resolution/2)/resolution))
    localY = int(round((localY - resolution/2)/resolution))
    
    return (localX, localY)

##
# Class for planning paths on a separate thread.
# 
# Termination method based on suggestion at:
# http://stackoverflow.com/questions/323972/is-there-any-way-to-kill-a-thread-in-python
#
class PlanningThread(threading.Thread):
    ##
    # Constructor for PlanningThread
    # @param initialPose The initial pose to plan from
    # @param goalPose The goal pose to plan to
    # @param occupancyGrid The grid in which to plan
    #
    def __init__(self, initialPose, goalPose, occupancyGrid):
        super(PlanningThread, self).__init__()
        self._stop = threading.Event()
        self.initialPose = initialPose
        self.goalPose = goalPose
        self.occupancyGrid = occupancyGrid
    
    ##
    # Stops running the thread
    #
    def stop(self):
        self._stop.set()

    ##
    # Thread runs this method automatically when started.
    def run(self):
        
        initial = poseToMapCoordinates(self.initialPose, self.occupancyGrid.info.resolution, self.occupancyGrid.info.origin)
        goal = poseToMapCoordinates(self.goalPose, self.occupancyGrid.info.resolution, self.occupancyGrid.info.origin)
        mapList = mapToList(self.occupancyGrid)
        reverseLookup = []

        frontier = []
        expanded = []
        
        # The frontier and expanded lists store pairs of the form (point, gCost)
        frontier.append((initial, 0))

        while len(frontier) > 0 and not self._stop.isSet():

            # Determine the lowest heuristic path cost node in the frontier
            minHCost = float("inf")
            for point in frontier:
                hCost = point[1] + getDistance(point[0],goal)
                if hCost <= minHCost:
                    minHCost = hCost
                    bestNode = point
            
            # Remove the lowest heuristic path cost node from the frontier
            frontier.remove(bestNode)
            
            if bestNode[0] == goal:
                break
            
            # The neighbors are north, south, east, west, and compound directions (for diagonal motion)
            straightCost = 1
            diagCost = 1.414

            neighbors = [((bestNode[0][0]+1,bestNode[0][1]),bestNode[1]+straightCost),
                         ((bestNode[0][0]-1,bestNode[0][1]),bestNode[1]+straightCost),
                         ((bestNode[0][0],bestNode[0][1]+1),bestNode[1]+straightCost),
                         ((bestNode[0][0],bestNode[0][1]-1),bestNode[1]+straightCost),
                         ((bestNode[0][0]+1,bestNode[0][1]+1),bestNode[1]+diagCost),
                         ((bestNode[0][0]+1,bestNode[0][1]-1),bestNode[1]+diagCost),
                         ((bestNode[0][0]-1,bestNode[0][1]+1),bestNode[1]+diagCost),
                         ((bestNode[0][0]-1,bestNode[0][1]-1),bestNode[1]+diagCost)]
            
            # For each neighbor:
            for neighbor in neighbors:
                # If the neighbor is an obstacle, skip over it
                if mapList[neighbor[0][0]][neighbor[0][1]] == 100:
                    continue

                # If the neighbor has been explored, or is on track to be explored, we should only add it to the
                # frontier if we have just discovered a shorter path to it:
                inFrontier = False
                bestFrontier = float('inf')
                inExplored = False
                bestExplored = float('inf')
                for point in frontier:
                    if point[0] == neighbor[0]:
                        inFrontier = True
                        if point[1] < bestFrontier:
                            bestFrontier = point[1]
                for point in expanded:
                    if point[0] == neighbor[0]:
                        inExplored = True
                        if point[1] < bestExplored:
                            bestExplored = point[1]

                if inFrontier and not neighbor[1] < bestFrontier:
                    continue;
                if inExplored and not neighbor[1] < bestExplored:
                    continue;
                
                frontier.append(neighbor)
                delIndices = []
                for i,node in enumerate(reverseLookup):
                    if node == neighbor[0]:
                        delIndices.append(i)

                for index in delIndices:
                    del reverseLookup[index]
                        
                reverseLookup.append((neighbor[0],bestNode[0]))
            
            # Add the node that was just expanded to the list of expanded nodes
            expanded.append(bestNode)

            expandedRaw = [node[0] for node in expanded]
            frontierRaw = [node[0] for node in frontier]

            pubGrid(expandedPub, expandedRaw, self.occupancyGrid.info.resolution, self.occupancyGrid.info.origin)
            pubGrid(frontierPub, frontierRaw, self.occupancyGrid.info.resolution, self.occupancyGrid.info.origin)

        if self._stop.isSet():
            return

        currentNode = goal
        finalPath = [currentNode]
        while currentNode != initial and not self._stop.isSet():
            for pair in reverseLookup:
                if pair[0] == currentNode:
                    currentNode = pair[1]
                    break
            finalPath.append(currentNode)

        if self._stop.isSet():
            return

        pubPath(pathPub, finalPath, self.occupancyGrid.info.resolution, self.occupancyGrid.info.origin)

        sleep(1.0)


##
# Main method
#
if __name__ == '__main__':
    global occupancyGrid
    global initialPose
    global goalPose

    global freshMap
    global freshInitialPose
    global freshGoalPose
    
    global initialPub
    global goalPub
    global expandedPub
    global frontierPub
    global pathPub

    freshInitialPose = False
    freshGoalPose = False
    freshMap = False

    killThread = False

    try:
        rospy.init_node('plan_path')

        mapSub = rospy.Subscriber("map", OccupancyGrid, mapHandler)
    
        initialPub = rospy.Publisher('/lab3_visualization/initialViz', GridCells, queue_size=10)
        goalPub = rospy.Publisher('/lab3_visualization/goalViz', GridCells, queue_size=10)
        expandedPub = rospy.Publisher("/lab3_visualization/expandedViz", GridCells, queue_size=100)
        frontierPub = rospy.Publisher("/lab3_visualization/frontierViz", GridCells, queue_size=100)
        pathPub = rospy.Publisher("/lab3_visualization/pathViz", Path, queue_size=100)
        
        # First, wait for a map
        while not rospy.is_shutdown():
            if freshMap:
                freshMap = False
                initialSub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)    
                goalSub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, goalPoseHandler)
                break

        # Next, wait for initial and goal pose assignments
        while not rospy.is_shutdown():
            if freshInitialPose and freshGoalPose:
                freshInitialPose = False
                freshGoalPose = False
                freshMap = False
                pathThread = PlanningThread(initialPose, goalPose, occupancyGrid)
                pathThread.start()
                break

        # Once planner is running, restart planner on any new pose assignment
        while not rospy.is_shutdown():
            if freshInitialPose or freshGoalPose or freshMap:
                freshInitialPose = False
                freshGoalPose = False
                freshMap = False
                pathThread.stop()
                sleep(0.5)
                pathThread = PlanningThread(initialPose, goalPose, occupancyGrid)
                pathThread.start()
        
        pathThread.stop()
        sleep(0.5)
        
        
    except rospy.ROSInterruptException:
        print "STOPPED PATH PLANNING"
