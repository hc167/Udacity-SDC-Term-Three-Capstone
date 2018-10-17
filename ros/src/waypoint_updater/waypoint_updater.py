#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.
Once you have created dbw_node, you will update this node to use the status of traffic lights too.
Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.
TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number
MAX_DECEL = .5 #

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)
	
	# TODO: Add other member variables you need below
        self.pose = None
        self.base_lane = None
        self.stopline_wp_idx = None
        self.waypoints_2d = None  # 2D points to create KDTree for the base point
        self.waypoint_tree = None  # KDTree to look up the closet waypoint       

	# use subscriber to get current pose and base waypoints 
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', PoseStamped, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)   

        #rospy.spin()
        self.loop()


    def loop(self):
        """
        control over the publishing frequency
        """
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        """
        return:
            the index of the waypoint which is closest 
            and in front of our car
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # query TB tree, give back position and index itself
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        # check is closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        # Equation for hyperplane through closest_coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)

        if val > 0:  # means the car pose is in front of the closest waypoint
            # choose the next waypoint as the cloeset one
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        '''
        take the wayopints and update their velocity base on 
        how we want the car to behave
        '''
        # create a new Lane msg
        lane = Lane()

        # splice the base waypoints
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx:farthest_idx]
        
        # if detect no TL concerned 
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            # no need to make any modifications, just publish the waypoints directly
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

	return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        '''
        create some new waypoints msg types.
        Args:
            waypoints: a slice of the base waypoints from the closest_idx to farthest_idx
            closest_idx: the index of the closest waypoint ahead of our vehicle
        '''
        # temporary list to store waypoints since we don't want to modify our base wayoints
        # this msg comes in only once v.s. keep the base waypoints preserved
        temporary_waypoints = []
        for i, base_wp in enumerate(waypoints):
            # create a new waypoint message
            p = Waypoint()
            # set the pose to the i-th base waypoint pose
            # the pose coatians the orientation
            p.pose = base_wp.pose

            # figure out the stopline index
            # because the vehicle pose is the exact center of the car
            # minus 2 or 3 to make the node of the car stop behind the line
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0) # Two waypoints back from line so front of car stops at line
            # waypoint distance of how far away from the TL stop line
            # distance will return 0 if i > stop_idx
            dist = self.distance(waypoints, i, stop_idx)
            vel = math.sqrt(2 * MAX_DECEL * dist)
            # if the velocity is small enough, just return 0
            if vel < 1.:
                vel = 0.

            # the square root vel is large when dist is large
            # treat the velocity of before as the speed limit
            p.twist.twist.linear.x = min(vel, base_wp.twist.twist.linear.x)
            temporary_waypoints.append(p)
        
        return temporary_waypoints

    def pose_cb(self, msg):
        # TODO: Implement
        """  
        param:
            msg: type PoseStamped, `rosmsg info geometry_msgs/PoseStamped` for details
        retrun:
            store current position of the car from topic /current_pose in self.pose
        """
        self.pose = msg


    def waypoints_cb(self, waypoints):
        # TODO: Implement
        """
        param: 
            waypoints: type Lane, `rosmsg info styx_msgs/Lane` for details
        return: 
            store the list of all waypoints of the track in self.waypoints
            apply KDTree to self.waypoint_tree
        """
        # latched subscriber, once called, doesn't send waypoints anymore 
        self.base_lane = waypoints
        # ensure the self.waypoints_2D is initialized before the subcriber is
        if not self.waypoints_2d:
            # get (x,y) for each waypoint
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            # construct data structure KDTree to look up the cloest point 
            self.waypoint_tree = KDTree(self.waypoints_2d)


    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        # stopline waypoint index(int32)
	self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        '''
        return:
            a linear piecewise distance from wp1 to wp2
            if wp1 > wp2, return 0
        '''
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            # Iterate through and add up the distances(only the line segments between waypoints)
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
