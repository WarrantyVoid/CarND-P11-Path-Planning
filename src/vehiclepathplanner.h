#ifndef VEHICLEPATHPLANNER_H
#define VEHICLEPATHPLANNER_H

#include "map.h"
#include "vehicle.h"

class VehiclePathPlanner
{
public:
  VehiclePathPlanner(const Map &map, double planningHorizon, double speedLimit);

public:
  void updateState(const Vehicle &car, const std::vector<Vehicle> &obstacles);

  Trajectory getTrajectory() const;

protected:

private:
  const Map &mMap;

  Vehicle mCar;

  std::vector<Vehicle> mObstacles;

  double mPlanningHorizon;

  double mSpeedLimit;

  double mTargetSpeed;

  int mTargetLane;
};

#endif // VEHICLEPATHPLANNER_H
