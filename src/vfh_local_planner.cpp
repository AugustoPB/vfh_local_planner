#include "vfh_local_planner/vfh_local_planner.h"

namespace vfh_local_planner
{
    VFHPlanner::VFHPlanner()
    {  

    }

    VFHPlanner::~VFHPlanner()
    {

    }

    //Reconfigure parameters
    void VFHPlanner::Reconfigure(vfh_local_plannerConfig &cfg)
    {
        config_ = cfg;
    }

    //Initializes the planner and set parameters
    bool VFHPlanner::Initialize(costmap_2d::Costmap2D* costmap)
    {
        window_width = costmap->getSizeInCellsX();
        window_height = costmap->getSizeInCellsY();

        cmd_vel_linear_x_ = 0;
        cmd_vel_angular_z_ = 0;
        
        Alocate();
        UpdateHistogram(costmap);

    }

    //Populates and allocates vectors and matrices used in the planner
    void VFHPlanner::Alocate()
    {
        vfh_histogram.resize(config_.vfh_sections_number, 0);
        costmap_cells_angle.resize(window_width, std::vector<double>(window_height));
        costmap_cells_distance.resize(window_width, std::vector<double>(window_height));

        for (int y=0; y < window_height; y++)
        {
            for (int x=0; x < window_width; x++)
            {
                costmap_cells_angle[x][y] = radToDeg(getCoordinateAngle(x-(window_width/2), (window_height/2)-y));
                costmap_cells_distance[x][y] = getCoordinateDistance(x-(window_width/2), (window_height/2)-y);
            }
        }
    }

    //Updates the Hitogram based on the costmap
    bool VFHPlanner::UpdateHistogram(costmap_2d::Costmap2D* costmap)
    {
        std::cout << "cleanning" << std::endl;
        fill(vfh_histogram.begin(), vfh_histogram.end(),0);

        for (int y=0; y < window_height; y++)
        {
            for (int x=0; x < window_width; x++)
            {
                double cell_sector = rint(costmap_cells_angle[x][y]/(360/(config_.vfh_sections_number-1)));
                double distance_cost = 100/(1+exp((config_.increase_rate*costmap_cells_distance[x][y])-(config_.increase_rate*config_.vhf_detection_range)));
                double magnitude = pow(((costmap->getCost(x, window_height-y-1))/254),2);
                vfh_histogram[cell_sector] += magnitude*distance_cost;
            }
        }
        std::cout << "updated hist" << std::endl;
        
        SmoothHistogram();

        std::cout << "smoothed" << std::endl;

        GetCandidateValleys();

        std::cout << "candidates" << std::endl;

        if (candidate_valleys.size() < 1)
            return false;

        for (int i=0; i < config_.vfh_sections_number; i++)
        {
            std::cout << vfh_histogram[i] << " ";
        }
        printf("\nCandidates\n");
        for (int i=0; i < candidate_valleys.size(); i++)
        {
            for (int j=0; j < candidate_valleys[i].size(); j++)
            {
                std::cout << candidate_valleys[i][j] << " ";
            }
            std::cout << std::endl;
        }
        printf("\nRejected\n");
        for (int j=0; j < rejected_peaks.size(); j++)
        {
            std::cout << rejected_peaks.at(j) << " ";
        }
        std::cout << std::endl;

        return true;

    }

    //Filters and smooths the histogram
    void VFHPlanner::SmoothHistogram()
    {
        std::vector<double> smoothed_histogram(config_.vfh_sections_number);
        double smoothed;
        for (int i=0; i < config_.vfh_sections_number; i++)
        {
            smoothed = config_.smooth_length*vfh_histogram[i];
            for (int k=1; k < config_.smooth_length-1; k++)
            {
                int lower_it = (i-k < 0)? config_.vfh_sections_number - i-k: i-k;
                int upper_it = (i+k > config_.vfh_sections_number-1)? i+k - config_.vfh_sections_number: i+k;
                
                smoothed += (config_.smooth_length-k)*vfh_histogram[lower_it] + (config_.smooth_length-k)*vfh_histogram[upper_it];
            }
            smoothed_histogram[i] = smoothed/(2*config_.smooth_length+1);
        }
        vfh_histogram = smoothed_histogram;
    }

    //Get all valleys that the robot can pass
    void VFHPlanner::GetCandidateValleys()
    {
        candidate_valleys.clear();
        rejected_peaks.clear();
        std::vector<int> valley;
        for (int i=0; i < config_.vfh_sections_number; i++)
        {
            if (vfh_histogram[i] < config_.vfh_threshold)
            {
                valley.push_back(i);
            }
            else
            {
                rejected_peaks.push_back(i);
                if (valley.size() > 0)
                    candidate_valleys.push_back(valley);
                valley.clear();
            }
        }
        if (valley.size() > 0)
            candidate_valleys.push_back(valley);

        if (candidate_valleys.size() > 1)
        {
            if (candidate_valleys.front().front() == 0 && candidate_valleys.back().back() == config_.vfh_sections_number-1)
            {
                candidate_valleys.back().insert(candidate_valleys.back().end(), candidate_valleys.front().begin(), candidate_valleys.front().end());
                candidate_valleys.erase(candidate_valleys.begin());
            }
        }
        for (int i=0; i<candidate_valleys.size(); i++)
        {
            if (candidate_valleys.at(i).size() <= config_.very_narrow_valley_threshold)
            {
                rejected_peaks.insert(rejected_peaks.end(),candidate_valleys.at(i).begin(), candidate_valleys.at(i).end());
                candidate_valleys.erase(candidate_valleys.begin()+i);
            }
        }
    }

    //Checks if the given direction is clean
    bool VFHPlanner::DirectionIsClear(double goal_direction)
    {
        int goal_sector = rint(radToDeg(goal_direction)/(360/(config_.vfh_sections_number-1)));
        std::cout << "sector of goal: " << goal_sector << std::endl;

        for (int k=0; k <= (config_.very_narrow_valley_threshold/2)-1; k++)
            {
                int upper_sector = (goal_sector+k > config_.vfh_sections_number-1)? goal_sector+k - config_.vfh_sections_number: goal_sector+k;
                int lower_sector = (goal_sector-k < 0)? config_.vfh_sections_number - goal_sector-k: goal_sector-k;
                std::cout << "vsec " << upper_sector << "  " << lower_sector << std::endl;
                if (vfh_histogram.at(upper_sector) > config_.vfh_threshold || vfh_histogram.at(lower_sector) > config_.vfh_threshold)
                    return false;
            }
        /*
        bool side_obstructed = false;
        for (int i=0; i < rejected_peaks.size(); i++)
        {
            for (int k=1; k <= (int)(config_.very_narrow_valley_threshold/2)-1; k++)
            {
                int upper_sector = (goal_sector+k < config_.vfh_sections_number-1)? goal_sector+k - config_.vfh_sections_number: goal_sector+k;
                int lower_sector = (goal_sector-k < 0)? config_.vfh_sections_number - goal_sector-k: goal_sector-k;
                if ((upper_sector == rejected_peaks.at(i)) || (lower_sector == rejected_peaks.at(i)))
                    return false;
            }

            if (goal_sector == rejected_peaks.at(i))
            {
                return false;
            }
        }
        */
        return true;
    }

    //Gets new direction to avoid obstacle based on the goal direction
    double VFHPlanner::GetNewDirection(double global_plan_goal_direction, double current_robot_direction, double previews_direction)
    {

        double goal_diff;
        double curr_direction_diff;
        double prev_direction_diff;
        double direction_cost;
        double smallest_cost = (M_PI*config_.goal_weight) + (M_PI*config_.curr_direction_weight) + (M_PI*config_.prev_direction_weight);
        int best_valley;
        bool valley_front;

        for (int i=0; i < candidate_valleys.size(); i++)
        {
            goal_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).front()*(360/(config_.vfh_sections_number-1))),global_plan_goal_direction));
            curr_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).front()*(360/(config_.vfh_sections_number-1))),current_robot_direction));
            prev_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).front()*(360/(config_.vfh_sections_number-1))),previews_direction));
            direction_cost = (goal_diff*config_.goal_weight) + (curr_direction_diff*config_.curr_direction_weight) + (prev_direction_diff*config_.prev_direction_weight);
            if (direction_cost < smallest_cost)
            {
                smallest_cost = direction_cost;
                best_valley = i;
                valley_front = true;
            }
            goal_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).back()*(360/(config_.vfh_sections_number-1))),global_plan_goal_direction));
            curr_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).back()*(360/(config_.vfh_sections_number-1))),current_robot_direction));
            prev_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).back()*(360/(config_.vfh_sections_number-1))),global_plan_goal_direction));
            direction_cost = (goal_diff*config_.goal_weight) + (curr_direction_diff*config_.curr_direction_weight) + (prev_direction_diff*config_.prev_direction_weight);
            if (direction_cost < smallest_cost)
            {
                smallest_cost = direction_cost;
                best_valley = i;
                valley_front = false;
            }
        }
        
        int valley_length = candidate_valleys.at(best_valley).size();
        double deviation_angle;
        if (valley_length < config_.wide_valley_threshold)
        {
            deviation_angle = candidate_valleys.at(best_valley).at(floor(valley_length/2))*(360/(config_.vfh_sections_number-1));
        }
        else
        {
            if (valley_front)
            {
                deviation_angle = candidate_valleys.at(best_valley).at(floor(config_.wide_valley_threshold/2))*(360/(config_.vfh_sections_number-1));
            }
            else
            {
                deviation_angle = candidate_valleys.at(best_valley).at(valley_length-1-floor(config_.wide_valley_threshold/2))*(360/(config_.vfh_sections_number-1));
            }
        }
        return degToRad(deviation_angle);
    }

    //Get speeds to rotate the robot to the angle
    bool VFHPlanner::RotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel)
    {
        cmd_vel.linear.x = 0.0;

        double yaw = tf::getYaw(global_pose.getRotation());
        double vel_yaw = tf::getYaw(robot_vel.getRotation());
        double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

        double v_theta_samp = ang_diff > 0.0 ? std::min(config_.max_vel_th_,
            std::max(config_.min_in_place_vel_th_, ang_diff)) : std::max(config_.min_vel_th_,
            std::min(-1.0 * config_.min_in_place_vel_th_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = fabs(vel_yaw) + config_.acc_lim_theta_/config_.local_planner_frequence;
        double min_acc_vel = fabs(vel_yaw) - config_.acc_lim_theta_/config_.local_planner_frequence;

        v_theta_samp = ((v_theta_samp < 0.0) ? -1.0 : 1.0) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = sqrt(2 * config_.acc_lim_theta_ * fabs(ang_diff)); 

        v_theta_samp = ((v_theta_samp < 0.0) ? -1.0 : 1.0) * std::min(max_speed_to_stop, fabs(v_theta_samp));

        // Re-enforce config_.min_in_place_vel_th_.  It is more important than the acceleration limits.
        v_theta_samp = v_theta_samp > 0.0
          ? std::min( config_.max_vel_th_, std::max( config_.min_in_place_vel_th_, v_theta_samp ))
          : std::max( config_.min_vel_th_, std::min( -1.0 * config_.min_in_place_vel_th_, v_theta_samp ));
        
        cmd_vel_angular_z_ = v_theta_samp;
        cmd_vel.angular.z = v_theta_samp;
        return true;
    }

    //Get speeds to drive the robot towards the goal
    void VFHPlanner::DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::Twist& cmd_vel)
    {
        std::cout << "begin drive" << std::endl;
        double x_speed = std::min(goal_distance, cmd_vel_linear_x_ + config_.acc_lim_x_/config_.local_planner_frequence);

        x_speed = std::min(x_speed,config_.max_vel_x_);

        double th_speed = (angle_to_goal < 0)? std::max(angle_to_goal, cmd_vel_angular_z_ - config_.acc_lim_theta_/config_.local_planner_frequence) : std::min(angle_to_goal, cmd_vel_angular_z_ + config_.acc_lim_theta_/config_.local_planner_frequence);

        th_speed = (th_speed > 0)? std::min(th_speed, config_.max_vel_th_): std::max(th_speed, config_.min_vel_th_);

        cmd_vel_linear_x_ = x_speed;
        cmd_vel_angular_z_ = th_speed;
        cmd_vel.linear.x = x_speed;
        cmd_vel.angular.z = th_speed;
        std::cout << "x vel: " << cmd_vel.linear.x << " z vel: " << cmd_vel.angular.z << std::endl;
    }

}