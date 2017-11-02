#ifndef VEHICLEPATHPLANNER_H
#define VEHICLEPATHPLANNER_H

#include "map.h"
#include "vehicle.h"


/**
 * @brief Enumerates available vehicle behaviors.
 */
enum EVehicleBehaviour
{
  FollowingLane,
  FollowingCar,
  ChangingLaneLeft,
  ChangingLaneRight
};


/**
 * @brief Describes a vehicle behavior.
 */
struct VehicleBehavior
{
  /**
   * @brief Constructs a new behavior.
   * @param behaviorId the identification enumeration
   * @param speed the targeted speed in m/s of the behavior
   * @param lane the targeted lane number of the behavior
   */
  VehicleBehavior(EVehicleBehaviour behaviorId, double speed, int lane) : id(behaviorId), targetSpeed(speed), targetLane(lane), targetTrajectory(), cost(0.0) { }

  /** the identification enumeration */
  EVehicleBehaviour id;

  /** the targeted speed in m/s of the behavior */
  double targetSpeed;

  /** the targeted lane number of the behavior */
  int targetLane;

  /** the targeted trajectory the behavior */
  Trajectory targetTrajectory;

  /** the cost associated with the targeted trajectory */
  double cost;
};


/**
 * @brief Plans future vehicle trajectories.
 */
class VehiclePathPlanner
{
public:
  /**
   * @brief Constructs a new planner.
   * @param map the map used for coordinate tranformations
   * @param horizon the horizon of the planner (trajectory length)
   * @param speedLimit the maximum speed in m/s to be planned
   * @param accelerationLimit the maximum speed difference in m/s to be planned
   * @param roadFriction the road friction used to determine buffer distance
   */
  VehiclePathPlanner(const Map &map, int horizon, double speedLimit, double accelerationLimit, double roadFriction);

public:
  /**
   * @brief Updates the planner with a new word state.
   * Calculates new behevior.
   * @param car the newest car state
   * @param obstacles the newest state of obstacles
   */
  void updateState(const Vehicle &car, const std::vector<Vehicle> &obstacles);

  /**
   * @brief Retrieves the trajectory from the current behvior.
   * @return trajectory object
   */
  Trajectory getTrajectory() const;

protected:
  /**
   * Helper function to calculate a single trajectory alternative.
   */
  Trajectory calculateTrajectory(const VehicleBehavior &behavior, int horizon) const;

  /**
   * Helper function to calculate cost for a trajectory alternative.
   */
  double calculateCost(const VehicleBehavior &behavior) const;

  /**
   * Helper function to calculate speed to maintain distance buffer.
   */
  double calculateApproachSpeed(double ownV, double otherV, double curDist, double minDist) const;

private:
  /** map for coordinate transformation. */
  const Map &mMap;

  /** current car state */
  Vehicle mCar;

  /** current obstacle state */
  std::vector<Vehicle> mObstacles;

  /** trajectory item count for planning */
  int mHorizon;

  /** maximum allowed velocity m/s */
  double mMaxV;

  /** maximum allowed velocitydetla m/s */
  double mMaxA;

  /** road friction used to determine buffer distance */
  double mRoadFriction;

  /** currently active vehicle behavior */
  VehicleBehavior mBehavior;

};

#endif // VEHICLEPATHPLANNER_H
