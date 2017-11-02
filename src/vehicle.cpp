
#include "vehicle.h"
#include "tools.h"
#include "map.h"


Vehicle::Vehicle(double xPos, double yPos, double sPos, double dPos, double yawAngle, double velocity,
                 const Trajectory &trajectoryPath, double trajectoryS, double trajectoryD)
  : x(xPos)
  , y(yPos)
  , s (sPos)
  , d(dPos)
  , yaw(yawAngle)
  , v(velocity)
  , trajectory(trajectoryPath)
  , bounds()
  , radiusX(1.5)
  , radiusY(3.0)
{
  int len = trajectory.size();
  if (len > 2)
  {
    // Move to end of trajectory
    double prevX = trajectory.x[len - 2];
    double prevY = trajectory.y[len - 2];
    x = trajectory.x[len - 1];
    y = trajectory.y[len - 1];
    s = trajectoryS;
    d = trajectoryD;
    yaw = Tools::vangle(x - prevX, y - prevY);
    v = Tools::vabs(x - prevX, y - prevY) * 50.0;
  }

  // Calc bounds in map coords
  double x1 = d - radiusX;
  double y1 = s - radiusY;
  double x2 = d + radiusX;
  double y2 = s - radiusY;
  double x3 = d - radiusX;
  double y3 = s + radiusY;
  double x4 = d + radiusX;
  double y4 = s + radiusY;
  //vehicleToMapCoordinates(x1, y1);
  //vehicleToMapCoordinates(x2, y2);
  //vehicleToMapCoordinates(x3, y3);
  //vehicleToMapCoordinates(x4, y4);
  bounds[0] << x1, y1;
  bounds[1] << x2, y2;
  bounds[2] << x3, y3;
  bounds[3] << x4, y4;
}


void Vehicle::vehicleToMapCoordinates(double &inOutX, double &inOutY) const
{
  double xRef = inOutX;
  double yRef = inOutY;
  inOutX = xRef * cos(yaw) - yRef * sin(yaw) + x;
  inOutY = xRef * sin(yaw) + yRef * cos(yaw) + y;
}


void Vehicle::mapToVehicleCoordinates(double &inOutX, double &inOutY) const
{
  double shiftX = inOutX - x;
  double shiftY = inOutY - y;
  inOutX = shiftX * cos(-yaw) - shiftY * sin(-yaw);
  inOutY = shiftX * sin(-yaw) + shiftY * cos(-yaw);
}


Vehicle Vehicle::predict(const Map &map, double dt) const
{
  // Rudimental model-based prediction
  double newS = s + dt * v;
  std::vector<double> xy = map.getXY(newS, d);
  Vehicle predictedVehicle(
    xy[0],
    xy[1],
    newS,
    d,
    yaw,
    v);
  predictedVehicle.radiusY += 0.5 * dt;
  return predictedVehicle;
}

double Vehicle::getSafetyDistance(double friction) const
{
  return (v * v) / (2.0 * friction * 9.81);
}


bool Vehicle::isCollidingWith(const Vehicle &other) const
{
  Eigen::Vector2d axis(bounds[1] - bounds[0]);
  double axisAbs = axis.squaredNorm();
  bool isSeparatable(true);
  for (int i = 0; i < 4 && isSeparatable; ++i)
  {
    double dp = (other.bounds[i] - bounds[0]).dot(axis) / axisAbs;
    isSeparatable = dp < 0.0 || dp > 1.0;
  }
  if (!isSeparatable)
  {
    axis = bounds[2] - bounds[0];
    axisAbs = axis.squaredNorm();
    isSeparatable = true;
    for (int i = 0; i < 4 && isSeparatable; ++i)
    {
      double dp = (other.bounds[i] - bounds[0]).dot(axis) / axisAbs;
      isSeparatable = dp < 0.0 || dp > 1.0;
    }
  }
  return !isSeparatable;
}


int Vehicle::getLane() const
{
  return std::max(0, int(d) / 4);
}

