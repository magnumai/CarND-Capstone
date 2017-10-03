#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
PUBLISHER_RATE = 1 # Publishin rate on channel /final_waypoints

dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below

        self.final_waypoints_pub = rospy.Publisher('/final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.all_waypoints = None
        self.len_all_waypoints = 0
        self.seq = 0
        self.current_waypoint_ahead = None

        rate = rospy.Rate(PUBLISHER_RATE)
        
        while not rospy.is_shutdown():
            self.publish_waypoints_ahead()
            
            rate.sleep()

    def pose_cb(self, msg):
        self.current_pose = msg.pose

        if self.all_waypoints == None:
            return
        
    def waypoints_cb(self, waypoints):
        self.all_waypoints = waypoints.waypoints
        self.len_all_waypoints = len(self.all_waypoints)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def _closest_waypoint_index(self):
        rospy.loginfo("computing closest_waypoint_index for pos %d, %d",
                    self.current_pose.position.x,
                    self.current_pose.position.y)
        
        if self.current_waypoint_ahead is None:
            possible_waypoint_indices = range(self.len_all_waypoints)
            closest_distance = float('inf')
        else:
            possible_waypoint_indices = range(self.current_waypoint_ahead, self.len_all_waypoints)
            closest_ditance = dl(self.all_waypoints[self.current_waypoint_ahead].pose.position,
                                self.current_pose.position)
            
        index = possible_waypoint_indices[0]
        closer_point_found = True

        while closer_point_found:
            index += 1
            distance = dl(self.all_waypoints[index].pose.pose.position,
                         self.current_pose.position)
        
            if distance > closest_distance:
                closer_point_found = False
            else:
                closest_distance = distance
                
        return index
        
    
    def publish_waypoints_ahead(self):
        if self.all_waypoints is None or self.current_pose is None:
            return
        
        start_index = self._closest_waypoint_index()
        
        # what should happen towards the end of the waypoints?
        end_index = start_index + LOOKAHEAD_WPS

        lane = Lane()
        lane.header.frame_id = '/world'
        lane.header.stamp = rospy.Time(0)
        lane.header.seq = self.seq
        lane.waypoints = self.all_waypoints[start_index:end_index]

        rospy.loginfo("sending range waypoints %d - %d",
                      start_index,
                      end_index)
        
        self.final_waypoints_pub.publish(lane)
        self.seq += 1

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
