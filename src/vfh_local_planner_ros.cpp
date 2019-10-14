#include "vfh_local_planner/vfh_local_planner_ros.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(vfh_local_planner, VFHPlannerRos, vfh_local_planner::VFHPlannerRos, nav_core::BaseLocalPlanner)

namespace vfh_local_planner
{
    VFHPlannerRos::VFHPlannerRos(): costmap_ros_(NULL), tf_(NULL), initialized_(false) {};

    VFHPlannerRos::VFHPlannerRos(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false)
    {
        // initialize planner
		initialize(name, tf, costmap_ros);
    };

    VFHPlannerRos::~VFHPlannerRos() {};


    void VFHPlannerRos::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
    {
        // check if plugin initialized
        if(!initialized_)
		{

        }
        else
        {
            ROS_WARN("This planner has already been initialized, doing nothing.");
        }
        
    };

    bool VFHPlannerRos::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
    };

    bool VFHPlannerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
    };

    bool VFHPlannerRos::isGoalReached()
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
    };
}