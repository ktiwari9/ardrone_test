#include "gps_ang_dist.h"

static const double PI = 3.14159265358979323846, diameterOfEarthMeters = 6371.0 * 2 * 1000;

double degreeToRadian (double degree) { return (degree * PI / 180); };
double radianToDegree (double radian) { return (radian * 180 / PI); };

double CoordinatesToAngle (const double latitude1,
                           const double longitude1,
                           const double latitude2,
                           const double longitude2)
{
  const double longitudeDifferenceRadians = degreeToRadian(longitude2 - longitude1);
  double latitude1Radian = degreeToRadian(latitude1),
       latitude2Radian = degreeToRadian(latitude2);

  const double x = std::cos(latitude1Radian) * std::sin(latitude2Radian) -
                 std::sin(latitude1Radian) * std::cos(latitude2Radian) *
                 std::cos(longitudeDifferenceRadians);
  const double y = std::sin(longitudeDifferenceRadians) * std::cos(latitude2Radian);

  return radianToDegree(std::atan2(y, x));
}

double CoordinatesToMeters (const double latitude1,
                            const double longitude1,
                            const double latitude2,
                            const double longitude2)
{
  double latitude1Radian = degreeToRadian(latitude1),
       longitude1Radian = degreeToRadian(longitude1),
       latitude2Radian = degreeToRadian(latitude2),
       longitude2Radian = degreeToRadian(longitude2);
  double x = std::sin((latitude2Radian - latitude1Radian) / 2),
       y = std::sin((longitude2Radian - longitude1Radian) / 2);

  return diameterOfEarthMeters *
         std::asin(std::sqrt((x * x) +
                             (std::cos(latitude1Radian) * std::cos(latitude2Radian) * y * y)));
}