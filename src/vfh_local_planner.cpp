#include "vfh_local_planner/vfh_local_planner.h"

namespace vfh_local_planner
{
    VFHPlanner::VFHPlanner()
    {  

    }

    VFHPlanner::~VFHPlanner()
    {

    }

    //Initializes the planner and set parameters
    bool VFHPlanner::Initialize(costmap_2d::Costmap2D* costmap)
    {
        window_width = costmap->getSizeInCellsX();
        window_height = costmap->getSizeInCellsY();

        vfh_sections_number = 36;
        goal_weight = 1;
        curr_direction_weight = 1;
        prev_direction_weight = 2;
        smooth_length = 5;
        vfh_threshold = 150;
        wide_valley_threshold = 6;
        very_narrow_valley_threshold = 2;
        vhf_detection_range = 15;
        increase_rate = 1;

        max_vel_x_ = 0.4;
        acc_lim_x_ = 0.2;
        max_vel_th_ = M_PI/6;
        min_vel_th_ = -1.0 * max_vel_th_;
        min_in_place_vel_th_ = 0.4;
        acc_lim_theta_ = M_PI/10;
        xy_goal_tolerance_ = 0.10;
        yaw_goal_tolerance_ = 0.05;

        cmd_vel_angular_z_rotate_ = 0;
        cmd_vel_linear_x_ = 0;
        cmd_vel_angular_z_ = 0;

        local_planner_frequence = 5.0;
        
        Alocate();
        UpdateHistogram(costmap);

    }

    //Populates and allocates vectors and matrices used in the planner
    void VFHPlanner::Alocate()
    {
        vfh_histogram.resize(vfh_sections_number, 0);
        smoothed_vfh_histogram.resize(vfh_sections_number, 0);
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
    void VFHPlanner::UpdateHistogram(costmap_2d::Costmap2D* costmap)
    {
        fill(vfh_histogram.begin(), vfh_histogram.end(),0);

        for (int y=0; y < window_height; y++)
        {
            for (int x=0; x < window_width; x++)
            {
                double cell_sector = rint(costmap_cells_angle[x][y]/(360/vfh_sections_number));
                double distance_cost = 100/(1+exp((increase_rate*costmap_cells_distance[x][y])-(increase_rate*vhf_detection_range)));
                double magnitude = pow(((costmap->getCost(x, window_height-y-1))/254),2);
                vfh_histogram[cell_sector] += magnitude*distance_cost;
            }
        }
        
        SmoothHistogram();

        GetCandidateValleys();

        for (int i=0; i < vfh_sections_number; i++)
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

    }

    //Filters and smooths the histogram
    void VFHPlanner::SmoothHistogram()
    {
        double smoothed;
        for (int i=0; i < vfh_sections_number; i++)
        {
            smoothed = smooth_length*vfh_histogram[i];
            for (int k=1; k < smooth_length-1; k++)
            {
                int lower_it = (i-k < 0)? vfh_sections_number - i-k: i-k;
                int upper_it = (i+k > vfh_sections_number-1)? i+k - vfh_sections_number: i+k;
                
                smoothed += (smooth_length-k)*vfh_histogram[lower_it] + (smooth_length-k)*vfh_histogram[upper_it];
            }
            vfh_histogram[i] = smoothed/(2*smooth_length+1);
        }
    }

    //Get all valleys that the robot can pass
    void VFHPlanner::GetCandidateValleys()
    {
        candidate_valleys.clear();
        rejected_peaks.clear();
        std::vector<int> valley;
        for (int i=0; i < vfh_sections_number; i++)
        {
            if (vfh_histogram[i] < vfh_threshold)
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
            if (candidate_valleys.front().front() == 0 && candidate_valleys.back().back() == vfh_sections_number-1)
            {
                candidate_valleys.back().insert(candidate_valleys.back().end(), candidate_valleys.front().begin(), candidate_valleys.front().end());
                candidate_valleys.erase(candidate_valleys.begin());
            }
        }
        for (int i=0; i<candidate_valleys.size(); i++)
        {
            if (candidate_valleys.at(i).size() <= very_narrow_valley_threshold)
            {
                candidate_valleys.erase(candidate_valleys.begin()+i);
            }
        }
    }

    //Checks if the given direction is clean
    bool VFHPlanner::DirectionIsClear(double goal_direction)
    {
        int intermediary_goal_sector = rint(radToDeg(goal_direction)/(360/vfh_sections_number));
        std::cout << "sector of goal: " << intermediary_goal_sector << std::endl;

        for (int i=0; i < rejected_peaks.size(); i++)
        {
            if (intermediary_goal_sector == rejected_peaks.at(i))
            {
                return false;
            }
        }
        return true;
    }

    //Gets new direction to avoid obstacle based on the goal direction
    double VFHPlanner::GetNewDirection(double global_plan_goal_direction, double current_robot_direction, double previews_direction)
    {

        double goal_diff;
        double curr_direction_diff;
        double prev_direction_diff;
        double direction_cost;
        double smallest_cost = (M_PI*goal_weight) + (M_PI*curr_direction_weight) + (M_PI*prev_direction_weight);
        int best_valley;
        bool valley_front;

        for (int i=0; i < candidate_valleys.size(); i++)
        {
            std::cout << "valley : " << i << std::endl;
            goal_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).front()*10),global_plan_goal_direction));
            curr_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).front()*10),current_robot_direction));
            prev_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).front()*10),previews_direction));
            direction_cost = (goal_diff*goal_weight) + (curr_direction_diff*curr_direction_weight) + (prev_direction_diff*prev_direction_weight);
            std::cout << "sec dif: " << direction_cost << std::endl;
            if (direction_cost < smallest_cost)
            {
                smallest_cost = direction_cost;
                best_valley = i;
                valley_front = true;
            }
            goal_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).back()*10),global_plan_goal_direction));
            curr_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).back()*10),current_robot_direction));
            prev_direction_diff = fabs(angles::shortest_angular_distance(degToRad(candidate_valleys.at(i).back()*10),global_plan_goal_direction));
            direction_cost = (goal_diff*goal_weight) + (curr_direction_diff*curr_direction_weight) + (prev_direction_diff*prev_direction_weight);
            std::cout << "sec dif: " << direction_cost << std::endl;
            if (direction_cost < smallest_cost)
            {
                smallest_cost = direction_cost;
                best_valley = i;
                valley_front = false;
            }
        }
        
        int valley_length = candidate_valleys.at(best_valley).size();
        double deviation_angle;
        if (valley_length < wide_valley_threshold)
        {
            deviation_angle = candidate_valleys.at(best_valley).at(floor(valley_length/2))*10;
        }
        else
        {
            if (valley_front)
            {
                deviation_angle = candidate_valleys.at(best_valley).at(floor(wide_valley_threshold/2))*10;
            }
            else
            {
                deviation_angle = candidate_valleys.at(best_valley).at(valley_length-1-floor(wide_valley_threshold/2))*10;
            }
        }
        std::cout << "deviation deg: " << deviation_angle << std::endl;
        deviation_angle = degToRad(deviation_angle);
        std::cout << "deviation rad: " << deviation_angle << std::endl;
        return deviation_angle;
    }

    //Get speeds to rotate the robot to the angle
    bool VFHPlanner::RotateToGoal(const tf::Stamped<tf::Pose>& global_pose, const tf::Stamped<tf::Pose>& robot_vel, double goal_th, geometry_msgs::Twist& cmd_vel)
    {
        if (cmd_vel.linear.x - 0.1 >= 0)
            cmd_vel.linear.x -= 0.1;

        double yaw = tf::getYaw(global_pose.getRotation());
        double vel_yaw = tf::getYaw(robot_vel.getRotation());
        double ang_diff = angles::shortest_angular_distance(yaw, goal_th);

        double v_theta_samp = ang_diff > 0.0 ? std::min(max_vel_th_,
            std::max(min_in_place_vel_th_, ang_diff)) : std::max(min_vel_th_,
            std::min(-1.0 * min_in_place_vel_th_, ang_diff));

        //take the acceleration limits of the robot into account
        double max_acc_vel = fabs(vel_yaw) + acc_lim_theta_/local_planner_frequence;
        double min_acc_vel = fabs(vel_yaw) - acc_lim_theta_/local_planner_frequence;

        v_theta_samp = ((v_theta_samp < 0.0) ? -1.0 : 1.0) * std::min(std::max(fabs(v_theta_samp), min_acc_vel), max_acc_vel);

        //we also want to make sure to send a velocity that allows us to stop when we reach the goal given our acceleration limits
        double max_speed_to_stop = sqrt(2 * acc_lim_theta_ * fabs(ang_diff)); 

        v_theta_samp = ((v_theta_samp < 0.0) ? -1.0 : 1.0) * std::min(max_speed_to_stop, fabs(v_theta_samp));

        // Re-enforce min_in_place_vel_th_.  It is more important than the acceleration limits.
        v_theta_samp = v_theta_samp > 0.0
          ? std::min( max_vel_th_, std::max( min_in_place_vel_th_, v_theta_samp ))
          : std::max( min_vel_th_, std::min( -1.0 * min_in_place_vel_th_, v_theta_samp ));

        cmd_vel.angular.z = v_theta_samp;
        return true;
    }

    //Get speeds to drive the robot towards the goal
    int VFHPlanner::DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::Twist& cmd_vel)
    {

        double x_speed = std::min(goal_distance, cmd_vel_linear_x_ + acc_lim_x_/local_planner_frequence);

        x_speed = std::min(x_speed,max_vel_x_);

        double th_speed = (angle_to_goal < 0)? std::max(angle_to_goal, cmd_vel_angular_z_ - acc_lim_theta_/local_planner_frequence) : std::min(angle_to_goal, cmd_vel_angular_z_ + acc_lim_theta_/local_planner_frequence);

        th_speed = (th_speed > 0)? std::min(th_speed, max_vel_th_): std::max(th_speed, min_vel_th_);

        cmd_vel_linear_x_ = x_speed;
        cmd_vel_angular_z_ = th_speed;
        cmd_vel.linear.x = x_speed;
        cmd_vel.angular.z = th_speed;
        
    }

}