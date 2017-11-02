#ifndef TOOLS_H
#define TOOLS_H

#include <math.h>
#include <limits>

class Tools
{
public:

  /**
   * @brief Retrieves the contant PI.
   * @return the constant PI
   */
  static constexpr double pi()
  {
    return M_PI;
  }

  /**
   * @brief Unit translation.
   * @return degree for given radians
   */
  static double deg2rad(double x)
  {
    return x * pi() / 180;
  }

  /**
   * @brief Unit translation.
   * @return radians for given degree
   */
  static double rad2deg(double x)
  {
    return x * 180 / pi();
  }

  /**
   * @brief Unit translation.
   * @return meter/s for given miles/h
   */
  static double mph2mps(double mph)
  {
    return mph * 0.44704;
  }

  /**
   * @brief Retrieves difference between two angles.
   * @param a1 angle 1 in radians
   * @param a2 angle 2 in radians
   * @return angle difference in radians
   */
  static double calculateAngleDelta(double  a1, double a2)
  {
    return Tools::normalizeAngle(Tools::normalizeAngle(a2) - Tools::normalizeAngle(a1));
  }

  /**
   * @brief Normalizes an angle between -PI an PI.
   * @param a angle in radians
   * @return normalized angle in radians
   */
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

  /**
   * @brief Retrieves vector length.
   * @param x vector x component
   * @param y vector y component
   * @return |v|
   */
  static double vabs(double x, double y)
  {
    return sqrt(x * x + y * y);
  }

  /**
   * @brief  Retrieves vector angle.
   * @param x vector x component
   * @param y vector y component
   * @return vector angle in radians
   */
  static double vangle(double x, double y)
  {
    return atan2(y, x);
  }

  /**
   * @brief Retrieves euclidian distance in between two points.
   * @param x1 point 1 x
   * @param y1 point 1 y
   * @param x2 point 2 x
   * @param y2 point 2 y
   * @return euclidian distance
   */
  static double distance(double x1, double y1, double x2, double y2)
  {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  /**
   * @brief Checks whether a floating point number is zero.
   * @param The floating point number
   * @return True if number is zero, false otherwise
   */
  static bool isZero(double f)
  {
    return fabs(f) < std::numeric_limits<double>::epsilon();
  }
};

#endif /* TOOLS_H */
