#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler
from partRing import CircleLight
import tf


class KineticEnvNode:
    __br = tf.TransformBroadcaster()
    __render_freq = 20.0
    __update_freq = 30.0
    __time = 0.0

    def __init__(self):
        rospy.init_node("kinetic_env_node")
        rospy.loginfo("Starting KineticEnvNode as kinetic_env_node.")
        self._circle_light = CircleLight()
        self.marker_pub = rospy.Publisher(
            "visualization_marker_array", MarkerArray, queue_size=10)

        rospy.Timer(rospy.Duration(1.0 / self.__render_freq), self.__render)
        rospy.Timer(rospy.Duration(1.0 / self.__update_freq), self.__update)
        self.ringPose = [Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                         Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1)),
                         Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))]

    def __update(self, timer):
        try:
            duration = (timer.current_real - timer.last_real).to_sec()

            # rospy.loginfo("Timer -> {}".format(duration))
            # for i, ring_pose in enumerate(self.ringPose):
            self.ringPose[0].position.z = 0.5 * \
                math.sin(0.1*self.__time * 2 * math.pi) - 0.5
            self.ringPose[1].position.z = 0.5 * \
                math.sin(0.2*self.__time * 2 * math.pi) - 0.5
            self.ringPose[2].position.z = 0.5 * \
                math.sin(0.1*self.__time * 2 * math.pi) - 0.5
            # self.ringPose[0].position.z = -0.5
            # self.ringPose[1].position.z = -0.5
            # self.ringPose[2].position.z = -0.5
            q = quaternion_from_euler(0.3 * math.sin(0.3*self.__time * 2 * math.pi),
                                      0.3 * math.cos(0.3*self.__time * 2 * math.pi), 0)
            self.ringPose[0].orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.ringPose[1].orientation = Quaternion(q[0], q[1], q[2], q[3])
            self.ringPose[2].orientation = Quaternion(q[0], q[1], q[2], q[3])

            self.__time += duration
        except Exception as e:
            rospy.logwarn(e)
            # pass

    def __render(self, timer):
        marker_array = MarkerArray()

        for i, ring_pose in enumerate(self.ringPose):
            translate = (ring_pose.position.x,
                         ring_pose.position.y, ring_pose.position.z)
            rotate = (ring_pose.orientation.x, ring_pose.orientation.y,
                      ring_pose.orientation.z, ring_pose.orientation.w)
            frame = "ring_link_{}".format(i)
            self.__br.sendTransform(
                translate, rotate, timer.current_real, frame, "base_link")

            marker = Marker(type=Marker.MESH_RESOURCE)
            marker.header = Header(frame_id=frame)
            marker.id = i
            radius = 0.0
            if i == 0:
                marker.mesh_resource = "package://Katana/meshes/ring_set1_s.stl"
                marker.pose = Pose(Point(-0.5, 0.5, 0), Quaternion(0, 0, 0, 1))
                radius = 0.5
            elif i == 1:
                marker.mesh_resource = "package://Katana/meshes/ring_set1_m.stl"
                marker.pose = Pose(Point(-0.25, 0.25, 0),
                                   Quaternion(0, 0, 0, 1))
                radius = 0.75
            elif i == 2:
                marker.mesh_resource = "package://Katana/meshes/ring_set1_l.stl"
                marker.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
                radius = 1.0
            marker.scale = Vector3(0.01, 0.01, 0.01)
            marker.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)

            marker_array.markers.append(marker)

            marker = Marker(type=Marker.SPHERE_LIST)
            marker.header = Header(frame_id=frame)
            marker.id = i+3
            marker.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
            marker.scale = Vector3(0.03, 0.03, 0.03)
            num = 300.0*radius
            for j in range(int(num)):
                x = radius * math.sin(j/num*2*math.pi)
                y = radius * math.cos(j/num*2*math.pi)
                marker.points.append(Point(x, y, 0.015))
                r = 0.5+0.5*math.sin(self.__time + j/num *
                                     2*math.pi + i*2*math.pi/3)
                g = 0.5+0.5*math.cos(self.__time + j/num *
                                     2*math.pi + i*2*math.pi/3)
                b = 0.5
                marker.colors.append(ColorRGBA(r, g, b, 1.0))
                x = (radius-0.1) * math.sin(j/num*2*math.pi)
                y = (radius-0.1) * math.cos(j/num*2*math.pi)
                marker.points.append(Point(x, y, 0.015))
                r = 0.5+0.5*math.cos(self.__time + j/num *
                                     2*math.pi + i*2*math.pi/3)
                g = 0.5+0.5*math.sin(self.__time + j/num *
                                     2*math.pi + i*2*math.pi/3)
                b = 0.5
                marker.colors.append(ColorRGBA(r, g, b, 1.0))

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


if __name__ == "__main__":
    kinetic_env_node = KineticEnvNode()
    rospy.spin()
