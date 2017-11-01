#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

class Map;

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
  void vehicleToMapCoordinates(double &inOutX, double &inOutY) const;
  void mapToVehicleCoordinates(double &inOutX, double &inOutY) const;
  Vehicle predict(const Map &map, double dt) const;
  double getSafetyDistance(double friction) const;
  bool isCollidingWith(const Vehicle &other) const;
  int getLane() const;

public:
  double x;
  double y;
  double s;
  double d;
  double yaw;
  double v;
  Trajectory trajectory;
  Eigen::Vector2d bounds[4];
  double radiusX;
  double radiusY;
};

#endif // VEHICLE_H
