#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32, Float32
from copy import deepcopy
import tf
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.wp_sub = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint',Int32, self.traffic_cb)
        rospy.Subscriber('/obstacle_waypoint',Int32, self.obstacle_cb)
        rospy.Subscriber('current_velocity', TwistStamped, self.current_velocity_cb )
        

       


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.current_pose = None
        self.base_waypoints = None
        self.current_velocity = None
        self.traffic_waypoint = None;
        self.final_waypoints = None
        

        rospy.spin()

    def pose_cb(self, msg):
        self.current_pose = msg
        rospy.loginfo('current pose recived')

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints.waypoints
        rospy.loginfo('base_waypoints recived')
        # We need to get base waypoints only once.
        self.wp_sub.unregister()
        self.wp_sub = None


    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist


    def traffic_cb(self, msg):
        self.traffic_waypoint = msg.data
        

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
