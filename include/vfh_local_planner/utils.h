#ifndef UTILS_H_
#define UTILS_H_

#include <cmath>

#define degToRad(Deg) ((Deg) * M_PI / 180.0)
#define radToDeg(Rad) ((Rad) * 180.0 / M_PI)

double getCoodinateAngle(int x, int y);

#endif