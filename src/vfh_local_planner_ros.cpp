#include "vfh_local_planner/vfh_local_planner_ros.h"

#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(vfh_local_planner, VFHPlannerRos, vfh_local_planner::VFHPlannerRos, nav_core::BaseLocalPlanner)

namespace vfh_local_planner
{
    VFHPlannerRos::VFHPlannerRos(): costmap_ros_(NULL), tf_(NULL), initialized_(false), odom_helper_("odom") {};

    VFHPlannerRos::VFHPlannerRos(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
        : costmap_ros_(NULL), tf_(NULL), initialized_(false), odom_helper_("odom")
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

            xy_goal_tolerance_ = 0.10;
            yaw_goal_tolerance_ = 0.05;
            xy_goal_latch_ = false;
            rotating_to_goal_ = false;
            finding_alternative_way_ = false;

            costmap_ros_ = costmap_ros;
            tf_ = tf;

            global_frame_ = costmap_ros_->getGlobalFrameID();
            costmap_ = costmap_ros_->getCostmap();
            initialized_ = true;

            ROS_INFO("teste %d",(int)costmap_->getCost(30,30));

            vfh_planner.Initialize(costmap_);

                    
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

        ROS_INFO("Got new plan!");
        //reset the global plan
        global_plan_.clear();
        global_plan_ = orig_global_plan;
        xy_goal_latch_ = false;
        rotating_to_goal_ = true;
        finding_alternative_way_ = false;

        /*
        while (true)
        {
            sleep(1);
            vfh_planner.UpdateHistogram(costmap_ros_->getCostmap());
        }
        */
        goal_reached_ = false;

        return true;
    };

    bool VFHPlannerRos::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
    {
        //Check if plugin initialized
		if(!initialized_)
		{
		ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
		return false;
		}

        //Get the pose of the robot in the global frame of the costmap
        tf::Stamped<tf::Pose> current_pose;
        if (!costmap_ros_->getRobotPose(current_pose)) {
            std::cout << "cant get current pose" << std::endl;
            return false;
        }

        //Get current robot velocity
        tf::Stamped<tf::Pose> current_vel;
        odom_helper_.getRobotVel(current_vel);

        //Create transform used to transform coordinates from global planner frame to costmap frame
        tf::StampedTransform frame_transform;
        tf_->lookupTransform(global_frame_, ros::Time(), global_plan_.back().header.frame_id, global_plan_.back().header.stamp, 
          global_plan_.back().header.frame_id, frame_transform);

        //Transform plan goal from the global planner frame to the frame of the costmap
        const geometry_msgs::PoseStamped& plan_goal = global_plan_.back();
        tf::Stamped<tf::Pose> transformed_plan_goal;
        tf::poseStampedMsgToTF(plan_goal, transformed_plan_goal);
        transformed_plan_goal.setData(frame_transform * transformed_plan_goal);
        transformed_plan_goal.stamp_ = frame_transform.stamp_;
        transformed_plan_goal.frame_id_ = global_frame_;
        //geometry_msgs::PoseStamped newer_pose;
        //poseStampedTFToMsg(tf_pose, newer_pose);
        

        //Transforms the global plan of the robot from the global planner frame to the frame of the costmap
        std::vector<geometry_msgs::PoseStamped> transformed_plan;
        if (!base_local_planner::transformGlobalPlan(*tf_, global_plan_, current_pose, *costmap_, global_frame_, transformed_plan)) {
          ROS_WARN("Could not transform the global plan to the frame of the controller");
          return false;
        }

        //Trim off parts of the global plan that are far enough behind the robot
        //base_local_planner::prunePlan(current_pose, transformed_plan, global_plan_);

        if(transformed_plan.empty())
        {
            ROS_ERROR("Plan is empty");
            return false;
        }

        //Get intermediary goal point in the transformed plan
        tf::Stamped<tf::Pose> intermediary_goal_point;
        int point_index = GetPlanPoint(transformed_plan, global_plan_,current_pose);
        //std::cout << "point index" << point_index << std::endl;
        tf::poseStampedMsgToTF(transformed_plan.at(point_index), intermediary_goal_point);

        //std::cout << "Goal " << global_plan_.back().pose.position.x << " " << global_plan_.back().pose.position.y << " TGoal" << transformed_plan_goal.getOrigin().getX() << " " << transformed_plan_goal.getOrigin().getY() << " LGoal " << intermediary_goal_point.getOrigin().getX() << " " << intermediary_goal_point.getOrigin().getY() << std::endl;

        //Update VFH histogram with new costmap
        vfh_planner.UpdateHistogram(costmap_ros_->getCostmap());

        //#############################################################################################################################################
        //########################################### Check if the robot reached the goal position ####################################################

        //Check if the robot is at the goal coordinate x y
        double goal_x = transformed_plan_goal.getOrigin().getX();
        double goal_y = transformed_plan_goal.getOrigin().getY();
        if (base_local_planner::getGoalPositionDistance(current_pose, goal_x, goal_y) <= xy_goal_tolerance_ || xy_goal_latch_)
        {
            std::cout << "xy reached" << std::endl;
            xy_goal_latch_ = true;
            //Check if the robot is at the same orientation of the goal
            double goal_th = tf::getYaw(transformed_plan_goal.getRotation());
            if (fabs(base_local_planner::getGoalOrientationAngleDifference(current_pose, goal_th)) <= yaw_goal_tolerance_)
            {
                std::cout << "parou yupi" << std::endl;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                goal_reached_ = true;
                return true;
            }
            else{
                std::cout << "rotating to goal to end" << std::endl;
                vfh_planner.RotateToGoal(current_pose, current_vel, goal_th, cmd_vel);
                return true;
            }
        }
        //#############################################################################################################################################
        //######################################################## Drive to the goal ##################################################################
        double direction_to_follow;
        double goal_distance;
        if(finding_alternative_way_)
        {
            double global_plan_goal_angle = atan2((transformed_plan_goal.getOrigin().getY()-current_pose.getOrigin().getY()), (transformed_plan_goal.getOrigin().getX()-current_pose.getOrigin().getX()));
            goal_distance = 0.2;
            if (vfh_planner.DirectionIsClear(global_plan_goal_angle))
            {
                direction_to_follow = global_plan_goal_angle;
            }
            else
            {
                direction_to_follow = vfh_planner.GetNewDirection(global_plan_goal_angle);
            }
        }
        else
        {
            double intermediary_goal_orientation = atan2((intermediary_goal_point.getOrigin().getY()-current_pose.getOrigin().getY()), (intermediary_goal_point.getOrigin().getX()-current_pose.getOrigin().getX()));

            //Check if the path is free
            if (!vfh_planner.DirectionIsClear(intermediary_goal_orientation))
            {
                finding_alternative_way_ = true;
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.y = 0.0;
                cmd_vel.angular.z = 0.0;
                return true;
            }
            else
            {
                direction_to_follow = intermediary_goal_orientation;
                goal_distance = sqrt(pow((intermediary_goal_point.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((intermediary_goal_point.getOrigin().getY()-current_pose.getOrigin().getY()),2));
            }
        }

        std::cout << "Going to direction: " << radToDeg(direction_to_follow) << std::endl;
        if (rotating_to_goal_)
        {
            std::cout << "rotating to goal to start" << std::endl;
            vfh_planner.RotateToGoal(current_pose, current_vel, direction_to_follow, cmd_vel);
            if (fabs(base_local_planner::getGoalOrientationAngleDifference(current_pose, direction_to_follow)) < yaw_goal_tolerance_)
            {
                rotating_to_goal_ = false;
            }
            return true;
        }
        //Check if the robot is deviating too much from the plan
        else if (fabs(base_local_planner::getGoalOrientationAngleDifference(current_pose, direction_to_follow)) > M_PI/4)
        {
            std::cout << "rotating to goal to start" << std::endl;
            rotating_to_goal_ = true;
        }
        //Drive toward the plan
        else 
        {
            std::cout << "driving to goal" << std::endl;
            vfh_planner.DriveToward(angles::shortest_angular_distance(tf::getYaw(current_pose.getRotation()),direction_to_follow), goal_distance, cmd_vel);
        }

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

    int VFHPlannerRos::GetPlanPoint(std::vector<geometry_msgs::PoseStamped> transformed_plan, std::vector<geometry_msgs::PoseStamped> &global_plan, tf::Stamped<tf::Pose> current_pose)
    {
        int point = 0;
        for (int i = 0; i < transformed_plan.size(); i++)
        {
            double point_distance = sqrt(pow((transformed_plan.at(i).pose.position.x-current_pose.getOrigin().getX()),2)+pow((transformed_plan.at(i).pose.position.y-current_pose.getOrigin().getY()),2));
            if (point_distance < 0.6)
            {
                point = i;
                //global_plan.erase(global_plan.begin()+i);
                //break;
            }
            else
            {
                if(point)
                    break;
            }
            
        }
        return point;
    }
}