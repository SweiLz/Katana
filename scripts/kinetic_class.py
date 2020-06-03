#!/usr/bin/env python
import math

import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import ColorRGBA, Header
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray


class KatanaKinetic(object):
    _br = tf.TransformBroadcaster()
    _render_freq = 20.0
    _update_freq = 30.0
    _time = 0.0

    def __init__(self):
        print("Hello KatanaKinetic ")

    def _update(self):
        print("Katana Update function.")


class FluxLight(KatanaKinetic):
    def __init__(self):
        print("FluxLight init")
        super(FluxLight, self).__init__()
        # self._update()

    def _update(self):
        print("Flux update")
        return super(FluxLight, self)._update()


class FluxLightNode:
    f = 0.0

    def __init__(self, nums=40, radius=2):
        rospy.init_node("flux_light_node")
        rospy.loginfo("Starting FluxLightNode as flux_light_node.")
        self.nums = nums
        self.radius = radius
        self.marker_pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / 20), self.__render)

    def __render(self, timer):
        marker = Marker(type=Marker.SPHERE_LIST, id=0, pose=Pose(Point(0, 0, 0), Quaternion(
            0, 0, 0, 1)), scale=Vector3(0.1, 0.1, 0.1), header=Header(frame_id="base_link"))

        for i in range(self.nums):
            pass
            # marker.points.append(Point(x, y, z))
            # marker.colors.append(ColorRGBA(r, g, b, 1.0))
        self.marker_pub.publish(marker)
        self.f += 1.0/20


class BallLightNode:
    f = 0.0

    def __init__(self, nums_ball=[40, 40]):
        rospy.init_node("ball_light_node")
        rospy.loginfo("Starting BallLightNode as ball_light_node.")
        self.nums_ball = nums_ball
        self.step_size = 0.2
        self.marker_pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / 20), self.__render)

    def __render(self, timer):
        marker = Marker(type=Marker.SPHERE_LIST, id=0, pose=Pose(Point(0, 0, 0), Quaternion(
            0, 0, 0, 1)), scale=Vector3(0.1, 0.1, 0.1), header=Header(frame_id="base_link"))

        for i in range(self.nums_ball[0]):
            for j in range(self.nums_ball[1]):
                z = 0.25*math.sin(self.f + i /
                                  float(self.nums_ball[0]) * 4 * math.pi)
                # z = 0
                # 0.25 * math.sin(self.f + i / float(self.nums_ball[0]) * 2 * math.pi) * math.sin(self.f + j / float(self.nums_ball[1]) * 2 * math.pi)
                x = i*self.step_size - \
                    ((self.nums_ball[0]+1) * self.step_size)/2.0
                y = j*self.step_size - \
                    ((self.nums_ball[1]+1) * self.step_size)/2.0

                r = 0.5+0.5*math.sin(self.f + i /
                                     float(self.nums_ball[0]) * 2 * math.pi)
                g = 0.5+0.5*math.cos(self.f + i /
                                     float(self.nums_ball[0]) * 2 * math.pi)
                b = 0.5

                marker.points.append(Point(x, y, z))
                marker.colors.append(ColorRGBA(r, g, b, 1.0))
        self.marker_pub.publish(marker)
        self.f += 1.0/20


class BallBounceNode:
    f = 0.0

    def __init__(self):
        rospy.init_node("ball_baunce_node")
        rospy.loginfo("Starting BallBounceNode as ball_baunce_node.")
        self.marker_pub = rospy.Publisher(
            "visualization_marker", Marker, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / 200), self.__render)
        self.ball_pose = Pose(Point(0, 0, 1.5), Quaternion(0, 0, 0, 1))
        self.ball_vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

    def __render(self, timer):

        if self.ball_pose.position.z < 0.075 and self.ball_vel.linear.z < 0:
            self.ball_vel.linear.z = -self.ball_vel.linear.z*0.85
        else:
            self.ball_vel.linear.z += -9.81 * 1.0/200

        self.ball_pose.position.z += self.ball_vel.linear.z * 1.0/200
        if self.ball_pose.position.z <= 0.07:
            self.ball_pose.position.z = 0.07
        marker = Marker(type=Marker.SPHERE, id=0, pose=self.ball_pose, scale=Vector3(
            0.15, 0.15, 0.15), header=Header(frame_id="base_link"), color=ColorRGBA(1, 1, 0, 1))

        # for i in range(self.nums):
        #     pass
        # marker.points.append(Point(x, y, z))
        # marker.colors.append(ColorRGBA(r, g, b, 1.0))
        self.marker_pub.publish(marker)
        self.f += 1.0/200


if __name__ == "__main__":
    # ball_light_node = BallLightNode()
    # ball_baunce_node = BallBounceNode()
    # rospy.spin()
    node = FluxLight()
    node._update()
