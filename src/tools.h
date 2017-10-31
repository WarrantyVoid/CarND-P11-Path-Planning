#ifndef TOOLS_H
#define TOOLS_H

#include <math.h>
#include <limits>

class Tools
{
public:

  static constexpr double pi()
  {
    return M_PI;
  }

  static double deg2rad(double x)
  {
    return x * pi() / 180;
  }

  static double rad2deg(double x)
  {
    return x * 180 / pi();
  }

  static double mph2mps(double mph)
  {
    return mph * 0.44704;
  }

  static double calculateAngleDelta(double  a1, double a2)
  {
    return Tools::normalizeAngle(Tools::normalizeAngle(a2) - Tools::normalizeAngle(a1));
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI)
    {
      a -= 2.0f * M_PI;
    }
    while (a < -M_PI)
    {
      a += 2.0f * M_PI;
    }
    return a;
  }

  static double vabs(double x, double y)
  {
    return sqrt(x * x + y * y);
  }

  static double vangle(double x, double y)
  {
    return atan2(y, x);
  }

  static double distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  static bool isZero(double f)
  {
    return fabs(f) < std::numeric_limits<double>::epsilon();
  }
};

#endif /* TOOLS_H */
