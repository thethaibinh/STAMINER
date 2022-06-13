# -*- coding: utf-8 -*-
"""
author: Thai Binh Nguyen
email: thethaibinh@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""
import time
import warnings
# import dodgeros_msgs.msg as dodgeros_msgs
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point
import rospy

# from agile_flight.envtest.ros.autopilot.rapid_trajectory_generator import RapidTrajectory

try:
    # Get monotonic time to ensure that time deltas are always positive
    _current_time = time.monotonic
except AttributeError:
    # time.monotonic() not available (using python < 3.3), fallback to time.time()
    _current_time = time.time
    warnings.warn('time.monotonic() not available in python < 3.3, using time.time() as fallback')


class TrajectoryGenerator:
    
    def __init__(self):
        # controllers
        self._init_time = None
        self._origin = [0.0, 0.0, 0.0]
        self._des_target1 = [-3.0, -23.0, 5.0]
        self._des_target2 = [5.0, 0.0, 5.0]
        self._des_target = None

        self._ref = visualization_msgs.Marker()
        self._ref.header.frame_id = "world"
        self._ref.ns = "ref"
        self._ref.lifetime = rospy.Duration(1)
        self._ref.type = visualization_msgs.Marker.SPHERE_LIST;
        self._ref.pose.position.x = 0
        self._ref.pose.position.y = 0
        self._ref.pose.position.z = 0
        self._ref.pose.orientation.w = 1
        self._ref.pose.orientation.x = 0
        self._ref.pose.orientation.y = 0
        self._ref.pose.orientation.z = 0
        self._ref.scale.x = 0.1
        self._ref.scale.y = 0.1
        self._ref.scale.z = 0.1
        self._ref.color.a = 1
        self._ref.color.r = 0
        self._ref.color.g = 1
        self._ref.color.b = 0

        # self._curr = self._ref
        # self._curr.ns = "actual"
        # self._curr.color.r = 1
        # self._curr.color.g = 0

        quad_namespace = 'kingfisher'
        self.trajectory_sub = rospy.Subscriber("/trajectory", Point, self.update_callback,
                                          queue_size=1, tcp_nodelay=True)
        self._reference_waypoint_pub = rospy.Publisher(
            quad_namespace + '/references/markers', visualization_msgs.Marker,
            queue_size=1)

        # self._current_position_pub = rospy.Publisher(
        #     quad_namespace + '/actual_position/markers', visualization_msgs.Marker,
        #     queue_size=1)

    # update_wpnav - run the wp controller - should be called at 100hz or higher
    def update_callback(self, traj):
        self._des_target = traj

    def update(self, states):
        now = _current_time()
        if self._init_time is None:
            self._init_time = _current_time()
            out =  self._des_target1
        elif ((now - self._init_time) > 30.0) and (self._des_target is not None):
            out = [self._des_target.x, self._des_target.y, self._des_target.z]
        else:
            out = self._des_target1
        
        # publish reference waypoint marker
        self._ref.header.stamp = rospy.Time.now()
        # ref_waypoint = self._des_target + states.pos
        # temp = Point(ref_waypoint[0],ref_waypoint[1],ref_waypoint[2])
        self._ref.points.append(self._des_target)
        self._reference_waypoint_pub.publish(self._ref)
        
        # self._curr.header.stamp = rospy.Time.now()
        # temp = Point(states.pos[0],states.pos[1],states.pos[2])
        # self._curr.points.append(temp)
        # self._current_position_pub.publish(self._curr)

        return out
    