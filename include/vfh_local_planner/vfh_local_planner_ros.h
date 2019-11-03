#ifndef VFH_LOCAL_PLANNER_ROS_H_
#define VFH_LOCAL_PLANNER_ROS_H_

#include <iostream>

#include <ros/ros.h>

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

// transforms
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/latched_stop_rotate_controller.h>

// msgs
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>

// config and parameters
#include <dynamic_reconfigure/server.h>
#include "vfh_local_planner/vfh_local_plannerConfig.h"

#include "vfh_local_planner/vfh_local_planner.h"


namespace vfh_local_planner 
{

    class VFHPlannerRos : public nav_core::BaseLocalPlanner
    {
    public:
        VFHPlannerRos();

        VFHPlannerRos(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);
        
        ~VFHPlannerRos();

        void initialize(std::string name, tf::TransformListener* tf,
          costmap_2d::Costmap2DROS* costmap_ros);

        bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);
        
        bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

        bool isGoalReached();

    private:

        int GetPlanPoint(std::vector<geometry_msgs::PoseStamped> transformed_plan, std::vector<geometry_msgs::PoseStamped> &global_plan, tf::Stamped<tf::Pose> current_pose);

        void reconfigureCB(vfh_local_plannerConfig &config, uint32_t level);

        // pointer to external objects (do NOT delete object)
        costmap_2d::Costmap2DROS* costmap_ros_; ///<@brief pointer to costmap
        costmap_2d::Costmap2D* costmap_;
        tf::TransformListener* tf_; ///<@brief pointer to Transform Listener
        std::vector<geometry_msgs::PoseStamped> global_plan_;
        base_local_planner::OdometryHelperRos odom_helper_;
        dynamic_reconfigure::Server<vfh_local_plannerConfig> *dsrv_;
        vfh_local_planner::vfh_local_plannerConfig default_config_;
        vfh_local_planner::vfh_local_plannerConfig config_;
        std::string global_frame_;

        VFHPlanner vfh_planner;

        int vfh_sections_number;
        int smooth_length;
        double vfh_threshold;
        double yaw_goal_tolerance_, xy_goal_tolerance_;
        double previews_direction;
        

        // flags
        bool rotating_to_goal_;
        bool xy_goal_latch_;
        bool finding_alternative_way_;
        bool initialized_;
        bool goal_reached_;
    };

}


#endif