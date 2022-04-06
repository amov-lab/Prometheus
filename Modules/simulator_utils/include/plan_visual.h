#ifndef _PLAN_VISUAL_H_
#define _PLAN_VISUAL_H_

// rviz\DisplayTypes\Marker: http://wiki.ros.org/rviz/DisplayTypes/Marker#Arrow_.28ARROW.3D0.29

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <istream>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace my_planner
{
    class PlanVisual
    {
    private:
        ros::NodeHandle nh;

        ros::Publisher goal_point_pub;
        ros::Publisher acc_pub;
        ros::Publisher vel_pub;
        ros::Publisher jerk_pub;
        ros::Publisher traj_pub;

    public:
        enum pub_type{VEL, ACC, JERK};
        PlanVisual(){};
        ~PlanVisual(){};
        PlanVisual(ros::NodeHandle &node);
        typedef std::shared_ptr<PlanVisual> Ptr;
        void visualInit(ros::NodeHandle &node);
        void displayMakerList(ros::Publisher pub, const std::vector<Eigen::Vector3d> &list, Eigen::Vector4d color, const double scale, int id);
        void displayGoalPoint(Eigen::Vector3d goal_point, Eigen::Vector4d color, const double scale, int id);
        void displayArrow(PlanVisual::pub_type type_id, Eigen::Vector3d start, Eigen::Vector3d end, Eigen::Vector4d color, int id);
        void displayTraj(std::vector<Eigen::Vector3d> &list, int id);
        void deleteAllMarker();
    };
}

#endif