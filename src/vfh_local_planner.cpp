#include "vfh_local_planner/vfh_local_planner.h"

namespace vfh_local_planner
{
    VFHPlanner::VFHPlanner()
    {  

    }

    VFHPlanner::~VFHPlanner()
    {

    }

    bool VFHPlanner::Initialize(costmap_2d::Costmap2D* costmap)
    {
        window_width = costmap->getSizeInCellsX();
        window_height = costmap->getSizeInCellsY();

        vfh_sections_number = 36;
        b = 10;
        a = 56.568542*b;
        smooth_length = 5;
        vfh_threshold = 160;
        wide_valley_threshold = 4;
        very_narrow_valley_threshold = 2;

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

        /*
        for (int i=0; i < window_height; i++)
        {
            for (int j=0; j < window_width; j++)
            {
                printf(" %lf ", costmap_cells_distance[i][j]);
                //ROS_INFO("dist: %lf", costmap_cells_distance[i][j]);
            }
            printf(" ");
        }
        */
    }

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
                //std::cout << costmap_cells_distance[x][y] << " ";
            }
            //std::cout << std::endl;
        }
        //printf("\n\n");
    }

    void VFHPlanner::UpdateHistogram(costmap_2d::Costmap2D* costmap)
    {
        fill(vfh_histogram.begin(), vfh_histogram.end(),0);

        for (int y=0; y < window_height; y++)
        {
            for (int x=0; x < window_width; x++)
            {
                //std::cout << (int)costmap->getCost(x, window_height-y-1) << " ";
                double cell_sector = rint(costmap_cells_angle[x][y]/(360/vfh_sections_number));
                double distance_cost = pow(0.5, ((costmap_cells_distance[x][y]/3)-10)); //(a - (b*costmap_cells_distance[x][y]));
                double magnitude = pow(((costmap->getCost(x, window_height-y-1))/254),2);
                vfh_histogram[cell_sector] += magnitude*distance_cost;
            }
            //std::cout << std::endl;
        }
        //std::cout << std::endl << std::endl;
        
        SmoothHistogram();

        GetCandidateValleys();

        /*
        for (int i=0; i < vfh_sections_number; i++)
        {
            std::cout << vfh_histogram[i] << " ";
        }
        printf("\n");
        */
        for (int i=0; i < vfh_sections_number; i++)
        {
            std::cout << smoothed_vfh_histogram[i] << " ";
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
            smoothed_vfh_histogram[i] = smoothed/(2*smooth_length+1);
        }
    }

    void VFHPlanner::GetCandidateValleys()
    {
        candidate_valleys.clear();
        rejected_peaks.clear();
        std::vector<int> valley;
        for (int i=0; i < vfh_sections_number; i++)
        {
            if (smoothed_vfh_histogram[i] < vfh_threshold)
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
            if (candidate_valleys.at(i).size() < very_narrow_valley_threshold)
            {
                candidate_valleys.erase(candidate_valleys.begin()+i);
            }
        }
    }

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

    double VFHPlanner::GetNewDirection(double global_plan_goal_direction)
    {
        int global_goal_sector = rint(radToDeg(global_plan_goal_direction)/(360/vfh_sections_number));
        int closest_valley;
        bool valley_front;
        int closest_sector_value = vfh_sections_number;
        int sectors_diff;
        double deviation_angle;

        for (int i=0; i < candidate_valleys.size(); i++)
        {
            std::cout << "valley : " << i << std::endl;
            sectors_diff = candidate_valleys.at(i).front()-global_goal_sector;
            if (sectors_diff < 0)
                sectors_diff += vfh_sections_number;
            std::cout << "sec dif: " << sectors_diff << std::endl;
            if (sectors_diff < closest_sector_value)
            {
                closest_sector_value = sectors_diff;
                closest_valley = i;
                valley_front = true;
            }
            sectors_diff = global_goal_sector-candidate_valleys.at(i).back();
            if (sectors_diff < 0)
                sectors_diff += vfh_sections_number;
            std::cout << "sec dif: " << sectors_diff << std::endl;
            if (sectors_diff < closest_sector_value)
            {
                closest_sector_value = sectors_diff;
                closest_valley = i;
                valley_front = false;
            }
        }
        //std::cout << "closest valley: " << closest_valley << " closest sector value: " << closest_sector_value << " valley front: " << valley_front << std::endl;
        
        int valley_length = candidate_valleys.at(closest_valley).size();
        if (valley_length < wide_valley_threshold)
        {
            deviation_angle = candidate_valleys.at(closest_valley).at(floor(valley_length/2))*10;
        }
        else
        {
            if (valley_front)
            {
                deviation_angle = candidate_valleys.at(closest_valley).at(floor(wide_valley_threshold/2))*10;
            }
            else
            {
                deviation_angle = candidate_valleys.at(closest_valley).at(valley_length-1-floor(wide_valley_threshold/2))*10;
            }
        }
        std::cout << "deviation deg: " << deviation_angle << std::endl;
        deviation_angle = degToRad(deviation_angle);
        std::cout << "deviation rad: " << deviation_angle << std::endl;
        return deviation_angle;
    }

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

    int VFHPlanner::DriveToward(double angle_to_goal, double goal_distance, geometry_msgs::Twist& cmd_vel)
    {

        //double distance = sqrt(pow((goal_pose.getOrigin().getX()-current_pose.getOrigin().getX()),2)+pow((goal_pose.getOrigin().getY()-current_pose.getOrigin().getY()),2));

        //double ang_diff = angles::shortest_angular_distance(tf::getYaw(current_pose.getRotation()),angle);

        double x_speed = std::min(goal_distance, cmd_vel_linear_x_ + acc_lim_x_/local_planner_frequence);

        x_speed = std::min(x_speed,max_vel_x_);

        double th_speed = (angle_to_goal < 0)? std::max(angle_to_goal, cmd_vel_angular_z_ - acc_lim_theta_/local_planner_frequence) : std::min(angle_to_goal, cmd_vel_angular_z_ + acc_lim_theta_/local_planner_frequence);

        th_speed = (th_speed > 0)? std::min(th_speed, max_vel_th_): std::max(th_speed, min_vel_th_);

        //double time_to_reach_th = angle_to_goal/th_speed;

        //if (time_to_reach_xy < time_to_reach_th)
        //{
        //    x_speed = goal_distance/time_to_reach_th;
        //}
        //else
        //{
        //    th_speed = angle_to_goal/time_to_reach_xy;
        //}

        //x_speed = (std::min(x_speed, cmd_vel.linear.x + acc_lim_x_/local_planner_frequence));

        
        
        //std::cout << "x vel: " << x_speed << " th vel: " << th_speed << std::endl;

        cmd_vel_linear_x_ = x_speed;
        cmd_vel_angular_z_ = th_speed;
        cmd_vel.linear.x = x_speed;
        cmd_vel.angular.z = th_speed;
        
    }

}