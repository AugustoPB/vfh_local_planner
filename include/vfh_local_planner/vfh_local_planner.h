#ifndef VFH_LOCAL_PLANNER_H_
#define VFH_LOCAL_PLANNER_H_

#include <stdio.h>
#include <vector>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>
#include <angles/angles.h>

#include <ros/ros.h>
#include "vfh_local_planner/utils.h"

namespace vfh_local_planner
{
    class VFHPlanner
    {
    public:
        VFHPlanner();
        ~VFHPlanner();

        bool Initialize(costmap_2d::Costmap2D* costmap);
        void Alocate();
        void UpdateHistogram(costmap_2d::Costmap2D* costmap);
        void SmoothHistogram();
        void GetCandidateValleys();

        bool RotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel);
        int DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::Twist& cmd_vel);
        bool DirectionIsClear(double goal_direction);
        double GetNewDirection(double global_plan_goal_direction, double current_robot_direction, double previews_direction);

    private:

    int window_width;
    int window_height;
    int vfh_sections_number;
    int a, b;
    int goal_weight, curr_direction_weight, prev_direction_weight;
    int smooth_length;
    double vfh_threshold;
    int wide_valley_threshold;
    int very_narrow_valley_threshold;

    double max_vel_x_, min_vel_x_, max_vel_th_, min_vel_th_, min_in_place_vel_th_;
    double acc_lim_x_, acc_lim_y_, acc_lim_theta_;
    double xy_goal_tolerance_, yaw_goal_tolerance_;

    double local_planner_frequence;
    double slow_down_factor;
    double cmd_vel_linear_x_;
    double cmd_vel_angular_z_rotate_;
    double cmd_vel_angular_z_;


    costmap_2d::Costmap2D* costmap_;
    std::vector<std::vector<double> > costmap_cells_angle;
    std::vector<std::vector<double> > costmap_cells_distance;
    std::vector<double> vfh_histogram;
    std::vector<double> smoothed_vfh_histogram;
    std::vector<std::vector<int> > candidate_valleys;
    std::vector<int> rejected_peaks;

    };
}

#endif