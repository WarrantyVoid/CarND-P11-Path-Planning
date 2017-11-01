#ifndef VEHICLEPATHPLANNER_H
#define VEHICLEPATHPLANNER_H

#include "map.h"
#include "vehicle.h"

enum EVehicleBehaviour
{
  FollowingLane,
  FollowingCar,
  ChangingLaneLeft,
  ChangingLaneRight
};


struct VehicleBehavior
{
  VehicleBehavior(EVehicleBehaviour behaviorId, double speed, int lane) : id(behaviorId), targetSpeed(speed), targetLane(lane), targetTrajectory(), cost(0.0) { }
  EVehicleBehaviour id;
  double targetSpeed;
  int targetLane;
  Trajectory targetTrajectory;
  double cost;
};


class VehiclePathPlanner
{
public:
  VehiclePathPlanner(const Map &map, int horizon, double speedLimit, double accelerationLimit, double roadFriction);

public:
  void updateState(const Vehicle &car, const std::vector<Vehicle> &obstacles);

  Trajectory getTrajectory() const;

protected:
  Trajectory calculateTrajectory(const VehicleBehavior &behavior, int horizon) const;
  double calculateCost(const VehicleBehavior &behavior) const;
  double calculateApproachSpeed(double ownV, double otherV, double curDist, double minDist) const;

private:
  const Map &mMap;

  Vehicle mCar;

  std::vector<Vehicle> mObstacles;

  int mHorizon;

  double mMaxV;

  double mMaxA;

  double mRoadFriction;

  VehicleBehavior mBehavior;

};

#endif // VEHICLEPATHPLANNER_H
