#ifndef VFH_LOCAL_PLANNER_H_
#define VFH_LOCAL_PLANNER_H_

#include <stdio.h>
#include <vector>

// costmap & geometry
#include <costmap_2d/costmap_2d_ros.h>

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

    private:

    int window_width;
    int window_height;
    int n_sections;
    int a, b;
    int smooth_length;
    double vfh_threshold;

    costmap_2d::Costmap2D* costmap_;
    std::vector<std::vector<double> > costmap_cells_angle;
    std::vector<std::vector<double> > costmap_cells_distance;
    std::vector<double> vfh_histogram;
    std::vector<double> smoothed_vfh_histogram;
    std::vector<std::vector<int> > candidate_valleys;

    };
}

#endif