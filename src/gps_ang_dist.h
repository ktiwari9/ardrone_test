#ifndef GPS_ANG_DIST_H
#define GPS_ANG_DIST_H

#include <cmath>

double degreeToRadian (double degree);
double radianToDegree (double radian);

double CoordinatesToAngle (const double latitude1,
                           const double longitude1,
                           const double latitude2,
                           const double longitude2);

double CoordinatesToMeters (const double latitude1,
                            const double longitude1,
                            const double latitude2,
                            const double longitude2);

#endif