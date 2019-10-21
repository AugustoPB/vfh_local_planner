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

        n_sections = 36;
        b = 4;
        a = 56.568542*b;
        smooth_length = 3;

        vfh_threshold = 10;
        
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
        vfh_histogram.resize(n_sections, 0);
        smoothed_vfh_histogram.resize(n_sections, 0);
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
                vfh_histogram[rint(costmap_cells_angle[x][y]/(360/n_sections))] += pow(((costmap->getCost(window_height-y-1,window_width-x-1))/254),2)*(a - (b*costmap_cells_distance[x][y]));
            }
        }

        SmoothHistogram();

        GetCandidateValleys();
        /*
        for (int i=0; i < n_sections; i++)
        {
            std::cout << vfh_histogram[i] << " ";
        }
        printf("\n");
        */
        for (int i=0; i < n_sections; i++)
        {
            std::cout << smoothed_vfh_histogram[i] << " ";
        }
        printf("\n\n");
        for (int i=0; i < candidate_valleys.size(); i++)
        {
            for (int j=0; j < candidate_valleys[i].size(); j++)
            {
                std::cout << candidate_valleys[i][j] << " ";
            }
            std::cout << std::endl;
        }
        printf("\n");
    }

    void VFHPlanner::SmoothHistogram()
    {
        double smoothed;
        for (int i=0; i < n_sections; i++)
        {
            smoothed = smooth_length*vfh_histogram[i];
            for (int k=1; k < smooth_length-1; k++)
            {
                int lower_it = (i-k < 0)? n_sections - i-k: i-k;
                int upper_it = (i+k > n_sections-1)? i+k - n_sections: i+k;
                
                smoothed += (smooth_length-k)*vfh_histogram[lower_it] + (smooth_length-k)*vfh_histogram[upper_it];
            }
            smoothed_vfh_histogram[i] = smoothed/(2*smooth_length+1);
        }
    }

    void VFHPlanner::GetCandidateValleys()
    {
        candidate_valleys.clear();
        std::vector<int> valley;
        for (int i=0; i < n_sections; i++)
        {
            if (smoothed_vfh_histogram[i] < vfh_threshold)
            {
                valley.push_back(i);
            }
            else
            {
                candidate_valleys.push_back(valley);
                valley.clear();
            }
        }
        candidate_valleys.push_back(valley);

        if (candidate_valleys.size() > 1)
        {
            if (candidate_valleys.front().front() == 0 && candidate_valleys.back().back() == n_sections-1)
            {
                candidate_valleys.back().insert(candidate_valleys.back().end(), candidate_valleys.front().begin(), candidate_valleys.front().end());
                candidate_valleys.erase(candidate_valleys.begin());
            }
        }

        //if (valley.back() == n_sections-1 && candidate_valleys[0].front() == 0)
        //{
        //    valley.insert(valley.end(), candidate_valleys[0].begin(), candidate_valleys[0].end());
        //    candidate_valleys.erase(candidate_valleys.begin());
        //    candidate_valleys.push_back(valley);
        //}
    }
}