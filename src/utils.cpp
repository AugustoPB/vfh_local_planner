#include "vfh_local_planner/utils.h"

double degToRad(double deg)
{
    double rad = deg * M_PI / 180.0;
    if (rad > M_PI)
        rad -= 2*M_PI;
    return rad;
};

double radToDeg(double rad)
{
    double deg = rad * 180.0 / M_PI;
    if (deg < 0)
        deg += 360;
    return deg;
};

double getCoordinateAngle(int x, int y)
{
    return atan2(y,x);
};

double getCoordinateDistance(int x, int y)
{
    return sqrt(pow(x,2) + pow(y,2));
};