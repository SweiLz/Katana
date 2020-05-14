#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

class BallLight
{

public:
    BallLight(){};

private:
    visualization_msgs::Marker _marker;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "basic_shapes");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    ros::Rate loop_rate(30);
    float f = 0.0;
    while (ros::ok())
    {
        visualization_msgs::Marker marker, points, line_strip, line_list;

        marker.header.frame_id = points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "/my_frame";
        marker.header.stamp = points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        marker.ns = points.ns = line_strip.ns = line_list.ns = "basic_shapes";
        marker.action = points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;

        marker.id = 0;
        points.id = 1;
        line_strip.id = 2;
        line_list.id = 3;

        // visualization_msgs::Marker::ARROW
        // visualization_msgs::Marker::CUBE
        // visualization_msgs::Marker::SPHERE
        // visualization_msgs::Marker::CYLINDER
        marker.type = visualization_msgs::Marker::SPHERE;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_STRIP;

        // points.scale.x = 0.05;
        // points.scale.y = 0.05;

        // line_strip.scale.x = 0.1;
        geometry_msgs::Pose pose;
        pose.position.x = 0.5;
        pose.position.y = 0;
        pose.position.z = 0;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1.0;

        line_list.scale.x = 0.5;
        line_list.pose = pose;

        // points.color.g = 1.0f;
        // points.color.a = 1.0;

        // line_strip.color.b = 1.0;
        // line_strip.color.a = 1.0;

        // line_list.color.r = 1.0;
        // line_list.color.a = 1.0;

        geometry_msgs::Point p;
        p.x = 0;
        p.y = 0;
        p.z = 0;

        std_msgs::ColorRGBA c;
        c.r = 1.0;
        c.g = 0.0;
        c.b = 0.0;
        c.a = 1.0;

        line_list.points.push_back(p);
        line_list.colors.push_back(c);
        p.z += 2.0;
        c.r = 0.0;
        c.g = 1.0;
        line_list.points.push_back(p);
        line_list.colors.push_back(c);

        // for (uint32_t i = 0; i < 20; ++i)
        // {
        //     float y = 5 * sin(f + i / 20.0f * 2 * M_PI);
        //     float x = 5 * cos(f + i / 20.0f * 2 * M_PI);

        //     geometry_msgs::Point p;
        //     p.x = x; //(int32_t)i - 50;
        //     p.y = y;
        //     p.z = 0;

        //     points.points.push_back(p);
        //     // line_strip.points.push_back(p);

        //     // The line list needs two points for each line
        //     // line_list.points.push_back(p);
        //     // p.z += 1.0;
        //     // line_list.points.push_back(p);
        // }

        // marker.pose.position.x = 0;
        // marker.pose.position.y = 0;
        // marker.pose.position.z = 0;
        // marker.pose.orientation.x = 0.0;
        // marker.pose.orientation.y = 0.0;
        // marker.pose.orientation.z = 0.0;
        // marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        // marker.scale.x = 0.1;
        // marker.scale.y = 0.1;
        // marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        // marker.color.r = 0.0f;
        // marker.color.g = 1.0f;
        // marker.color.b = 1.0f;
        // marker.color.a = 1.0;

        // marker.lifetime = ros::Duration();
        // marker_pub.publish(marker);
        // marker_pub.publish(points);
        // marker_pub.publish(line_strip);
        marker_pub.publish(line_list);

        ros::spinOnce();
        loop_rate.sleep();
        // f += 0.04;
    }
}