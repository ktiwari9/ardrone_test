#include "gps_ang_dist.h"

static const double PI = 3.14159265358979323846, R = 6371.0 * 1000;

double degreeToRadian (double degree) { return (degree * PI / 180); };
double radianToDegree (double radian) { return (radian * 180 / PI); };

double CoordinatesToAngle (const double latitude1,
                           const double longitude1,
                           const double latitude2,
                           const double longitude2)
{
  
  const double longitudeDifferenceRadians = degreeToRadian(longitude2 - longitude1);
  double latitude1Radian = degreeToRadian(latitude1);
  double latitude2Radian = degreeToRadian(latitude2);


  double y = std::sin(longitudeDifferenceRadians) * std::cos(latitude2Radian);
  double x =  std::cos(latitude1Radian) * std::sin(latitude2Radian)
              - std::sin(latitude1Radian) * std::cos(latitude2Radian) * std::cos(longitudeDifferenceRadians);

  double brng     = std::atan2(y,x);
  double brng_deg = radianToDegree(brng);
  return brng_deg > 180 ? brng_deg - 360   : brng_deg;
}

double CoordinatesToMeters (const double latitude1,
                            const double longitude1,
                            const double latitude2,
                            const double longitude2)
{
  double longitudeDifferenceRadians = degreeToRadian(longitude2 - longitude1);
  double latitudeDifferenceRadians = degreeToRadian(latitude2 - latitude1);
  double latitude1Radian = degreeToRadian(latitude1);
  double latitude2Radian = degreeToRadian(latitude2);
  double x =  std::sin(latitudeDifferenceRadians/2) * std::sin(latitudeDifferenceRadians/2)
              + std::cos(latitude1Radian) * std::cos(latitude2Radian) * std::sin(longitudeDifferenceRadians/2)
              * std::sin(longitudeDifferenceRadians/2);
  x = 2 * std::atan2(std::sqrt(x),std::sqrt(1-x));

  return R * x;
}