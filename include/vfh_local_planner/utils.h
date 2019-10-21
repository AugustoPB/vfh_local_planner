#ifndef UTILS_H_
#define UTILS_H_

#include <cmath>

//#define degToRad(Deg) ((Deg) * M_PI / 180.0)
//#define radToDeg(Rad) ((Rad) * 180.0 / M_PI)

double getCoordinateAngle(int x, int y);
double getCoordinateDistance(int x, int y);
double degToRad(double deg);
double radToDeg(double rad);

#endif