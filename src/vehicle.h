#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <assert.h>

struct Trajectory
{
  typedef std::vector<double> TPoints;
  Trajectory() : x(), y() { }
  Trajectory(const TPoints &pointsX, const TPoints &pointsY) : x(pointsX), y(pointsY) { }
  int size() const { assert(x.size() == y.size()); return x.size(); }
  TPoints x;
  TPoints y;
};


class Vehicle
{
public:
  Vehicle(double xPos, double yPos, double sPos, double dPos, double yawAngle, double velocity,
          const Trajectory &trajectoryPath = Trajectory(), double trajectoryS = 0.0, double trajectoryD = 0.0);

public:
  double getDistanceTo(const Vehicle &other) const;
  int getLane() const;

public:
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double v;
  Trajectory trajectory;
};

#endif // VEHICLE_H
