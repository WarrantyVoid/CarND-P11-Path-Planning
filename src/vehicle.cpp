
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
  double x1 = -radiusX;
  double y1 = -radiusY;
  double x2 = radiusX;
  double y2 = -radiusY;
  double x3 = -radiusX;
  double y3 = radiusY;
  double x4 = radiusX;
  double y4 = radiusY;
  vehicleToMapCoordinates(x1, y1);
  vehicleToMapCoordinates(x2, y2);
  vehicleToMapCoordinates(x3, y3);
  vehicleToMapCoordinates(x4, y4);
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
  double newX = x + cos(yaw) * dt;
  double newY = y + sin(yaw) * dt;
  double newYaw = yaw;
  double newV = v;
  std::vector<double> frenet = map.getFrenet(newX, newY, newYaw);
  Vehicle predictedVehicle(
    newX,
    newY,
    frenet[0],
    frenet[1],
    newYaw,
    newV);
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

