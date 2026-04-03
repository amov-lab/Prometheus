#include <plan_visual.h>

namespace my_planner
{
    PlanVisual::PlanVisual(ros::NodeHandle &node)
    {
        nh = node;

        goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 10);
        jerk_pub = nh.advertise<visualization_msgs::Marker>("jerk_dir", 10);
        acc_pub = nh.advertise<visualization_msgs::Marker>("acc_dir", 10);
        vel_pub = nh.advertise<visualization_msgs::Marker>("vel_dir", 10);
        traj_pub = nh.advertise<visualization_msgs::Marker>("poly_traj", 10);
        ROS_INFO("[Visual]: Init");
    }

    void PlanVisual::visualInit(ros::NodeHandle &node)
    {
        nh = node;

        goal_point_pub = nh.advertise<visualization_msgs::Marker>("goal_point", 10);
        jerk_pub = nh.advertise<visualization_msgs::Marker>("jerk_dir", 10);
        acc_pub = nh.advertise<visualization_msgs::Marker>("acc_dir", 10);
        vel_pub = nh.advertise<visualization_msgs::Marker>("vel_dir", 10);
        traj_pub = nh.advertise<visualization_msgs::Marker>("poly_traj", 10);

        ROS_INFO("[Visual]: Init");
    }

    void PlanVisual::displayMakerList(ros::Publisher pub, const std::vector<Eigen::Vector3d> &list, Eigen::Vector4d color, const double scale, int id)
    {
        visualization_msgs::Marker sphere, line_strip;
        sphere.header.frame_id = line_strip.header.frame_id = "world";
        sphere.header.stamp = line_strip.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE_LIST;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        sphere.action = line_strip.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        line_strip.id = id + 1000;

        sphere.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
        sphere.color.r = line_strip.color.r = color(0);
        sphere.color.g = line_strip.color.g = color(1);
        sphere.color.b = line_strip.color.b = color(2);
        sphere.color.a = line_strip.color.a = color(3) > 1e-5 ? color(3) : 1.0;
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        line_strip.scale.x = scale / 2;

        geometry_msgs::Point pt;

        for (int i = 0; i < int(list.size()); i++)
        {
            pt.x = list[i](0);
            pt.y = list[i](1);
            pt.z = list[i](2);
            sphere.points.push_back(pt);
            line_strip.points.push_back(pt);
        }
        pub.publish(sphere);
        pub.publish(line_strip);
    }

    void PlanVisual::displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = scale;
        sphere.scale.y = scale;
        sphere.scale.z = scale;
        sphere.pose.position.x = goal_point(0);
        sphere.pose.position.y = goal_point(1);
        sphere.pose.position.z = goal_point(2);
        sphere.lifetime = ros::Duration();
        goal_point_pub.publish(sphere);
    }

    void PlanVisual::displayArrow(PlanVisual::pub_type type_id, Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector4d color, int id)
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::ARROW;
        sphere.action = visualization_msgs::Marker::ADD;
        sphere.id = id;
        sphere.lifetime = ros::Duration();
        sphere.pose.orientation.w = 1.0;
        sphere.color.r = color(0);
        sphere.color.g = color(1);
        sphere.color.b = color(2);
        sphere.color.a = color(3);
        sphere.scale.x = 0.1;
        sphere.scale.y = 0.2;
        sphere.scale.z = 0.1;

        geometry_msgs::Point point;
        point.x = start(0);
        point.y = start(1);
        point.z = start(2);
        sphere.points.push_back(point);
        point.x = end(0);
        point.y = end(1);
        point.z = end(2);
        sphere.points.push_back(point);
        switch (type_id)
        {
        case VEL:
            vel_pub.publish(sphere);
            break;
        case ACC:
            acc_pub.publish(sphere);
            break;
        case JERK:
            jerk_pub.publish(sphere);
        }
    }

    void PlanVisual::displayTraj(std::vector<Eigen::Vector3d> &list, int id)
    {
        if (traj_pub.getNumSubscribers() == 0)
        {
            return;
        }

        displayMakerList(traj_pub, list, Eigen::Vector4d(1, 0, 0, 1), 0.15, id);
    }

    void PlanVisual::deleteAllMarker()
    {
        visualization_msgs::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp = ros::Time::now();
        sphere.type = visualization_msgs::Marker::SPHERE;
        sphere.action = visualization_msgs::Marker::DELETEALL;
        sphere.id = 0;

        sphere.pose.orientation.w = 1.0;
        sphere.color.r = 0;
        sphere.color.g = 0;
        sphere.color.b = 0;
        sphere.color.a = 1;
        sphere.scale.x = 0;
        sphere.scale.y = 0;
        sphere.scale.z = 0;
        sphere.pose.position.x = 0;
        sphere.pose.position.y = 0;
        sphere.pose.position.z = 0;
        sphere.lifetime = ros::Duration();
        goal_point_pub.publish(sphere);
    }
}