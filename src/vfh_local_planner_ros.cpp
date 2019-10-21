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
            ROS_INFO("Initializing VFH Planner");

            costmap_ros_ = costmap_ros;
            tf_ = tf;

            costmap_ = costmap_ros_->getCostmap();
            initialized_ = true;

            ROS_INFO("teste %d",(int)costmap_->getCost(30,30));

            tes.Initialize(costmap_);

                    
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

        //ROS_INFO("Got new plan!");
        
        
        while (true)
        {
            sleep(1);
            tes.UpdateHistogram(costmap_ros_->getCostmap());
        }
        
        goal_reached_ = false;

        return true;
    };

    bool VFHPlannerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

        /*
        int height = costmap_ros_->getCostmap()->getSizeInCellsY();
        int width = costmap_ros_->getCostmap()->getSizeInCellsX();

        for (int i=0; i < height; i++)
        {
            for (int j=0; j < width; j++)
            {
                std::cout << (int)costmap_ros_->getCostmap()->getCost(height-i,width-j) << " ";
            }
             std::cout << std::endl;
        }
        std::cout << std::endl << std::endl << std::endl;
        */

        //tes.UpdateHistogram(costmap_ros_->getCostmap());

        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.linear.z = 0;

        cmd_vel.angular.x = 0;
        cmd_vel.angular.y = 0;
        cmd_vel.angular.z = 0;

        return true;
    };

    bool VFHPlannerRos::isGoalReached()
    {
        // check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}
        return goal_reached_;
    };
}