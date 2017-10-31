
#include "vehicle.h"
#include "tools.h"

Vehicle::Vehicle(double xPos, double yPos, double sPos, double dPos, double yawAngle, double velocity,
                 const Trajectory &trajectoryPath, double trajectoryS, double trajectoryD)
  : x(xPos)
  , y(yPos)
  , s (sPos)
  , d(dPos)
  , yaw(yawAngle)
  , v(velocity)
  , trajectory(trajectoryPath)
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
}

double Vehicle::getDistanceTo(const Vehicle &other) const
{
  return Tools::distance(x, y, other.x, other.y);
}

int Vehicle::getLane() const
{
  return int(d) / 4;
}


