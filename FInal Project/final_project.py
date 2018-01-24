import rospy
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped
from nav_msgs.msg import GridCells, OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray
import tf
from threading import Lock
from math import sin, cos
from mapUtils import *
from collections import deque

##
# Handles debugging intial pose events
# @param msg The debugging pose estimate
def initialPoseHandler(msg):
    global freshPose
    global pose
    pose = msg.pose.pose
    freshPose = True
    

##
# Handles map event
# @param msg The map
def mapHandler(msg):
    global freshMap
    global grid
    grid = msg
    freshMap = True

##
# Handles navigation status events
# @param msg The goal status message
def statusHandler(msg):
    global freshStatus
    global status
    status = msg
    freshStatus = True

##
# Gets the frontiers from an occupancy grid
# @param occupancyGrid The occupancy grid in which to find frontiers
# @param pose The pose of the robot
# @param progressPub The GridCell publisher on which to publish wavefront progress (for debugging)
# @param resultPub The GridCell publisher on which to publish the result (for debugging)
# @return A list of lists of points in the occupancy grid which correspond to frontiers.
def getFrontiers(occupancyGrid, pose, progressPub, resultPub):
    gridList = mapToList(occupancyGrid)
    
    location = poseToMapCoordinates(pose, occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
    visited = [[False for cell in gridList[0]] for row in gridList]
    
    
    inFrontier = [[False for cell in gridList[0]] for row in gridList]
    
    wave = deque()
    wave.append(location)
    visited[location[0]][location[1]] = True
    
    i=0
    
    # First, find every reachable point that falls on a boundary between the map and unknown
    while not rospy.is_shutdown() and len(wave) > 0:
        
        
        # Pop the last point in the wave
        point = wave.popleft()
        
        # Compute the neighbors of this point
        neighbors = [(point[0]+1, point[1]),
                     (point[0], point[1]+1),
                     (point[0]-1, point[1]),
                     (point[0], point[1]-1),
                     (point[0]-1, point[1]-1),
                     (point[0]+1, point[1]+1),
                     (point[0]+1, point[1]-1),
                     (point[0]-1, point[1]+1)]
        
        addPoint = False
        
        for neighbor in neighbors:
            # If the neighbor is within the map bounds:
            if neighbor[0] >= 0 and neighbor[0] < len(gridList) and neighbor[1] >= 0 and neighbor[1] < len(gridList[0]):
                # If any of the neighbors is in the unknown, then we are at a frontier point.
                if gridList[neighbor[0]][neighbor[1]] == -1:
                    addPoint = True
                # If the neighbor is an obstacle, then it should not be explored again.
                # If it is open space, and has not been visited (and is not in the wave), it should be added to the wave
                elif gridList[neighbor[0]][neighbor[1]] != 100 and not visited[neighbor[0]][neighbor[1]]:
                    wave.append(neighbor)
                    visited[neighbor[0]][neighbor[1]] = True
        
        if addPoint and not inFrontier[point[0]][point[1]]:
            inFrontier[point[0]][point[1]] = True
        
        i+=1
        if i%100 == 0:
            if progressPub is not None:
                pubGrid(progressPub, wave, occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
    
    frontierLookup = [[None for cell in gridList[0]] for row in gridList]
    frontiers = []
    
    # Now, group points into subfrontiers
    for row in range(len(inFrontier)):
        for col in range(len(inFrontier[row])):
            if inFrontier[row][col] and (frontierLookup[row][col] is None):
                # If the current point is in the frontier, but not assigned to a subfrontier, find its neighbors
                neighbors = [(row+1, col),
                             (row, col+1),
                             (row-1, col),
                             (row, col-1),
                             (row-1, col-1),
                             (row+1, col+1),
                             (row+1, col-1),
                             (row-1, col+1)]
                             
                # If any neighbor is in a subfrontier already, add the current point to that subfrontier
                for neighbor in neighbors:
                    if neighbor[0] >= 0 and neighbor[0] < len(inFrontier) and neighbor[1] >= 0 and neighbor[1] < len(inFrontier[0]):
                        if frontierLookup[neighbor[0]][neighbor[1]] is not None:
                            frontierLookup[neighbor[0]][neighbor[1]].append((row,col))
                            frontierLookup[row][col] = frontierLookup[neighbor[0]][neighbor[1]]
                            break
                
                # If none of the neighbors is already in a frontier, create a new frontier for this point
                if frontierLookup[row][col] is None:
                    newFrontier = [(row,col)]
                    frontierLookup[row][col] = newFrontier
                    frontiers.append(newFrontier)
            
            
    totalFrontier = []
    for frontier in frontiers:
        for point in frontier:
            totalFrontier.append(point)
             
    if resultPub is not None:
        pubGrid(resultPub, totalFrontier, occupancyGrid.info.resolution, occupancyGrid.info.origin)
    
    return frontiers

##
# Called repeatedly by rospy at an interval
# @param event Timing information about the timer callback
def timerCallback(event):
    global pose
    global freshPose

    odom_list.waitForTransform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))

    (position, orientation) = odom_list.lookupTransform('map','base_footprint', rospy.Time(0))

    roll, pitch, yaw = euler_from_quaternion([orientation[0], orientation[1], orientation[2], orientation[3]])
    
    with threadLock:
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.orientation.z = yaw
        freshPose = True
        
##
# Main block
#
if __name__ == '__main__':
    global freshPose
    global freshMap
    global freshStatus
    global pose
    global grid
    global status
    global threadLock
    
    threadLock = Lock()
    
    freshPose = False
    freshMap = False
    freshStatus = False
    
    rospy.init_node('final_project_nav')
    
    
    odom_list = tf.TransformListener()

    # Repeating odometry callback
    timer = rospy.Timer(rospy.Duration(0.1), timerCallback)

    rospy.sleep(rospy.Duration(0.1))

    
    # Subscribe to the map and navigation status
    mapSub = rospy.Subscriber("/map", OccupancyGrid, mapHandler)
    statusSub = rospy.Subscriber("/move_base/status", GoalStatusArray, statusHandler)
    
    # Subscribe to the initial pose from rviz for debugging
    initialPoseSub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, initialPoseHandler)
    
    # Publish to the goal topic and a grid view for previewing frontiers
    goalPub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    frontierPub = rospy.Publisher('/frontiers', GridCells, queue_size=1)
    progressPub = rospy.Publisher('/wavefront_progress', GridCells, queue_size=1)
    
    # Pseudocode:
    # Wait for a map
    # On map recv, get frontiers.
    # If frontiers empty, print done.
    # If frontiers not empty, pick a frontier and find its centroid.
    # Publish centroid as goal.
    
    while not rospy.is_shutdown():
        while (not freshMap or not freshPose) and not rospy.is_shutdown():
            pass
        freshMap = False
        freshPose = False
        with threadLock:
            localPose = pose
            
        print "Computing frontiers."
        # Sort the frontiers according to size
        frontiers = sorted(getFrontiers(grid, localPose, progressPub, frontierPub), key=lambda elt: len(elt), reverse=True)
        # For each frontier, starting with the largest, attempt to drive to the centroid. If centroid cannot be reached, move
        # on to the next smallest frontier
        for frontier in frontiers:
            sumRow = 0
            sumCol = 0
            for point in frontier:
                sumRow += point[0]
                sumCol += point[1]
            centroid = (sumRow/len(frontier), sumCol/len(frontier))
            
            target = mapToPoseCoordinates(centroid, grid.info.resolution, grid.info.origin)
            target.pose.orientation = pose.orientation
            
            target.header.frame_id = '/map'
            print "Publishing goal..."
            goalPub.publish(target)
            atGoal = False
            goalId = None
            while not rospy.is_shutdown():
                if freshStatus:
                    freshStatus = False
                
                    # If the status is an "active" status, record the goal_id
                    if len(status.status_list) > 0:
                        if status.status_list[0].status == 1:
                            print "Goal accepted."
                            goalId = status.status_list[0].goal_id
                        elif goalId is not None and status.status_list[0].goal_id == goalId:
                            value = status.status_list[0].status
                            if value == 4 or value == 5 or value == 8:
                                print "Goal could not be reached."
                                break
                            elif value == 3:
                                print "Goal reached."
                                atGoal = True
                                break
            if atGoal:
                break
                        
                
