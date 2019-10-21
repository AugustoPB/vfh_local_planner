#include <iostream>
#include <vector>
#include "vfh_local_planner/utils.h"

using namespace std;

int main ()
{
    /*
    int fakecostmap[10*10] = { 0,0,0,50,90,150,200,250,190,80,
                               0,0,20,100,189,254,254,254,200,100,
                               0,0,0,30,90,178,250,254,190,60,
                               0,0,0,0,30,100,150,159,100,30,
                               0,0,0,0,0,40,70,80,50,10,
                               0,0,0,0,0,0,10,20,7,0,
                               0,0,0,0,0,0,0,10,0,0,
                               0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,
                               0,0,0,0,0,0,0,0,0,0,};
    */
    std::vector<std::vector<int>> costmap = {{ 0,0,0,0,50,90,150,200,250,190,80},
                                              {0,0,0,20,100,189,254,254,254,200,100},
                                              {0,0,0,0,30,90,178,250,254,190,60},
                                              {0,0,0,0,0,0,100,150,159,100,30},
                                              {0,0,0,0,0,0,0,0,80,50,10},
                                              {0,0,0,0,0,0,0,10,20,7,0},
                                              {0,0,0,0,0,0,0,0,10,0,0},
                                              {0,0,0,0,0,0,0,0,0,0,0},
                                              {0,0,0,0,0,0,0,0,0,0,0},
                                              {0,0,0,0,0,0,0,0,0,0,0},
                                              {0,0,0,0,0,0,0,0,0,0,0}};

    /*
    std::vector<int> tempvec(10);
    for (int i=0; i < 10; i++)
    {
        std::copy(&fakecostmap[i*10], &fakecostmap[i*10+9], tempvec.begin());
        costmap.push_back(tempvec);
    }
    */
    
    int window_width = 11;
    int window_height = 11;

    double dmax = 7.07107;
    double a = dmax;
    int b = 1;

    std::vector<std::vector<double> > costmap_cells_angle;
    std::vector<std::vector<double> > costmap_cells_distance;

    costmap_cells_angle.resize(window_width, std::vector<double>(window_width));

    //costmap_cells_angle.resize(window_height);
    costmap_cells_distance.resize(window_height);
    for(int i = 0 ; i < window_width ; ++i)
    {
        //Grow Columns by n
        //costmap_cells_angle[i].resize(window_width);
        costmap_cells_distance[i].resize(window_width);
    }

    for (int i=0; i < window_height; i++)
    {
        for (int j=0; j < window_width; j++)
        {
            costmap_cells_angle[i][j] = radToDeg(getCoordinateAngle(j-(window_width/2), (window_height/2)-i));
            costmap_cells_distance[i][j] = getCoordinateDistance(j-(window_width/2), (window_height/2)-i);
        }
    }

    for (int i=0; i < window_height; i++)
    {
        for (int j=0; j < window_width; j++)
        {
            cout << " " << costmap_cells_angle[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl << endl;

    for (int i=0; i < window_height; i++)
    {
        for (int j=0; j < window_width; j++)
        {
            cout << " " << costmap_cells_distance[i][j] << " ";
        }
        cout << endl;
    }
    cout << endl << endl;

    for (int i=0; i < window_height; i++)
    {
        for (int j=0; j < window_width; j++)
        {
            cout << " " << costmap[i][j] << " ";
        }
        cout << endl;
    }

    int sections_n = 18;

    std::vector<double> HIST(18);
    cout << endl << endl;
    for (int i=0; i < window_height; i++)
    {
        for (int j=0; j < window_width; j++)
        {
            cout << " " << rint(costmap_cells_angle[i][j]/(360/sections_n)) << " ";
            HIST[rint(costmap_cells_angle[i][j]/(360/sections_n))] += (pow(costmap[i][j], 2))*(a + (b*costmap_cells_distance[i][j]));
        }
        cout << endl;
    }
    cout << endl << endl;
    for (int i=0; i < sections_n; i++)
    {
        cout << " " << HIST[i] << " ";
    }
    cout << endl;


}