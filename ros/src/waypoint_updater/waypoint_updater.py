#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint

from std_msgs.msg import Int32
import math, sys
import tf
import copy


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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.cur_pos = None
	self.base_waypoints = None
	self.next_waypoints = None

	self.loop()
        rospy.spin()

    def pose_cb(self, msg):
	self.cur_pos = msg.pos

    def waypoints_cb(self, waypoints):
	if self.base_waypoints is None:
	    self.base_waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def loop(self):
	'''
	Publish to /final_waypoints with next waypoints 
	'''
	rate = rospy.Rate(40)
	while not rospy.is_shutdown():
	    # Build 'final_waypoints' message
	    if self.cur_pos is not None and self.base_waypoints is not None:
		rospy.logwarn("publishing from loop")

	        # Get nearest waypoint 
		nearest_waypoint_index = self.get_nearest_waypoint(self.cur_pos)
		
		# Deepcopy is used because I dont want to change original array
		self.next_waypoints = copy.deepcopy(self.base_waypoints[nearest_waypoint_index:nearest_waypoint_index + LOOKAHEAD_WPS])
	    
		# Create Lane object to publish
		lane = prepare_lane()

		# Publish
		self.final_waypoints_pub.publish(lane)
		
	    

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_nearest_waypoint(self, pose):
	nearest_len = sys.maxsize
	nearest_waypoint = 0
	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
	for index, waypoint in enumerate(self.base_waypoints):
    	    dist =dl(pose.position, waypoint.pose.pose.position)
	    if (nearest_len > dist):
		nearest_len = dist
		nearest_waypoint = index
	return nearest_waypoint

    def prepare_lane(self):
	# Prepare lane with endpoints
	lane = Lane()
	lane.header = self.base_waypoint.header
	lane.waypoints = np.asarray(self.next_waypoints)
	return lane


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
