
#include "vehiclepathplanner.h"
#include "spline.h"
#include "tools.h"
#include <map>
#include <iostream>

VehiclePathPlanner::VehiclePathPlanner(const Map &map, int horizon, double speedLimit, double accelerationLimit, double roadFriction)
  : mMap(map)
  , mCar(0.0, 0.0, 0.0, 6.0, 0.0, 0.0)
  , mObstacles()
  , mHorizon(horizon)
  , mMaxV(speedLimit)
  , mMaxA(accelerationLimit)
  , mRoadFriction(roadFriction)
  , mBehavior(FollowingLane, speedLimit, mCar.getLane())
{

}


void VehiclePathPlanner::updateState(const Vehicle &car, const std::vector<Vehicle> &obstacles)
{
  // Maps lane index to obstacle vehicles
  typedef std::map<int, Vehicle*> TNextVehicles;

  // Update car and obstacles
  mCar = car;
  mObstacles = obstacles;
  int carLane = mCar.getLane();


  // Determine new car behavior
  double safetyDistance = mCar.getSafetyDistance(mRoadFriction);
  double approachDistance = safetyDistance;
  double dt = mCar.trajectory.size() * 0.02;

  // Determine lane speed for all lanes
  double followLaneSpeed = std::min(mCar.v + mMaxA, mMaxV);
  TNextVehicles nextCars;
  for (int i = 0; i < obstacles.size(); ++i)
  {
    double obstacleS = mObstacles[i].s  + mObstacles[i].v * dt;
    int obstacleLane = mObstacles[i].getLane();
    TNextVehicles::const_iterator nc = nextCars.find(obstacleLane);
    if (obstacleS > mCar.s && obstacleS < mCar.s + safetyDistance + approachDistance
        && (nc == nextCars.end() || nc->second->s > mObstacles[i].s))
    {
      nextCars[obstacleLane] = &mObstacles[i];
    }
  }

  // List of possible behaviours
  std::vector<VehicleBehavior> behaviors;

  // Check for car ahead on same lane
  TNextVehicles::const_iterator nc = nextCars.find(carLane);
  if (nc != nextCars.end() && nc->second->v < mMaxV)
  {
    double actualDistance = nc->second->s + nc->second->v * dt - mCar.s;
    if (actualDistance < safetyDistance + approachDistance)
    {
      // Add follow car behavior
      double followCarSpeed = calculateApproachSpeed(mCar.v, nc->second->v, actualDistance - safetyDistance, approachDistance);
      behaviors.push_back(VehicleBehavior(FollowingCar, followCarSpeed, carLane));

      // Add left lane change behaviour
      if (carLane > 0)
      {
        double newLaneSpeed = followLaneSpeed;
        nc = nextCars.find(carLane - 1);
        if (nc != nextCars.end())
        {
          actualDistance = nc->second->s + nc->second->v * dt - mCar.s;
          newLaneSpeed = calculateApproachSpeed(mCar.v, nc->second->v, actualDistance - safetyDistance, approachDistance);
        }
        behaviors.push_back(VehicleBehavior(ChangingLaneLeft, newLaneSpeed, carLane - 1));
      }

      // Add right lane change behaviour
      if (carLane < 2)
      {
        double newLaneSpeed = followLaneSpeed;
        nc = nextCars.find(carLane + 1);
        if (nc != nextCars.end())
        {
          actualDistance = nc->second->s + nc->second->v * dt - mCar.s;
          newLaneSpeed = calculateApproachSpeed(mCar.v, nc->second->v, actualDistance - safetyDistance, approachDistance);
        }
        behaviors.push_back(VehicleBehavior(ChangingLaneRight, newLaneSpeed, carLane + 1));
      }
    }
  }

  if (behaviors.empty())
  {
    // Add default lane following behavior
    behaviors.push_back(VehicleBehavior(FollowingLane, followLaneSpeed, carLane));
  }

  // Select best behavior
  VehicleBehavior *bestBehavior = 0;
  for (int i = 0; i < behaviors.size(); ++i)
  {
    behaviors[i].targetTrajectory = calculateTrajectory(behaviors[i], mHorizon);
    behaviors[i].cost = calculateCost(behaviors[i]);
    if (bestBehavior == 0 || bestBehavior->cost > behaviors[i].cost)
    {
      bestBehavior = &behaviors[i];
    }
  }
  bool printed(false);
  for (int i = 0; i < behaviors.size(); ++i)
  {
    if (behaviors[i].id != FollowingLane)
    {
      //printed = true;
      //std::cout << "[id: " << behaviors[i].id << ", v: " <<  behaviors[i].targetSpeed << ", cost: " << behaviors[i].cost << "] ";
    }
  }
  if (printed)
  {
    std::cout << std::endl;
  }
  assert(bestBehavior);
  mBehavior = *bestBehavior;
}


Trajectory VehiclePathPlanner::getTrajectory() const
{
  Trajectory trajectory;

  // Recreate base trajectory section from history
  for(int i = 0; i < mCar.trajectory.size(); ++i)
  {
    trajectory.x.push_back(mCar.trajectory.x[i]);
    trajectory.y.push_back(mCar.trajectory.y[i]);
  }

  // Fill up remainder of trajectory from behavior
  for(int i = 0; i < std::min(mHorizon - mCar.trajectory.size(), mBehavior.targetTrajectory.size()); ++i)
  {
    trajectory.x.push_back(mBehavior.targetTrajectory.x[i]);
    trajectory.y.push_back(mBehavior.targetTrajectory.y[i]);
  }
  return trajectory;
}


Trajectory VehiclePathPlanner::calculateTrajectory(const VehicleBehavior &behavior, int horizon) const
{
  Trajectory trajectory;

  // Generate sparse trajectory (past two points)
  std::vector<double> ptsX;
  std::vector<double> ptsY;
  double sparseStep = 25.0;
  int histLen = mCar.trajectory.size();
  if (histLen < 2)
  {
     // No history, backtrace one step from current state
    double prevCarX = mCar.x - 1.0 * cos(mCar.yaw);
    double prevCarY = mCar.y - 1.0 * sin(mCar.yaw);
    ptsX.push_back(prevCarX);
    ptsY.push_back(prevCarY);
    ptsX.push_back(mCar.x);
    ptsY.push_back(mCar.y);
  }
  else
  {
    // Use end of history
    double refXPrev = mCar.trajectory.x[histLen - 2];
    double refYPrev = mCar.trajectory.y[histLen - 2];
    ptsX.push_back(refXPrev);
    ptsY.push_back(refYPrev);
    ptsX.push_back(mCar.x);
    ptsY.push_back(mCar.y);
  }

  // Generate sparse trajectory (future three points)
  for(int i = 0; i < 3; ++i)
  {
    double s = mCar.s + (i + 1) * sparseStep;
    double d = 2.0 + 4.0 * behavior.targetLane;
    std::vector<double> xy = mMap.getXY(s, d);
    ptsX.push_back(xy[0]);
    ptsY.push_back(xy[1]);
  }

  // Convert trajectory to vehicle coordinate system
  for(int i = 0; i < ptsX.size(); ++i)
  {
    mCar.mapToVehicleCoordinates(ptsX[i], ptsY[i]);
  }

  // Match splines
  tk::spline fSpline;
  fSpline.set_points(ptsX, ptsY);

  // Fill trajectory
  double targetX = sparseStep;
  double targetY = fSpline(targetX);
  double targetDist = Tools::distance(0, 0, targetX, targetY);
  double xAddon = 0;

  for (int i = 0; i < horizon; ++i)
  {
    // Approximate point distance based on v
    double N = targetDist / (0.02 * behavior.targetSpeed);
    double xPoint = xAddon + targetX / N;
    double yPoint = fSpline(xPoint);
    xAddon = xPoint;

    // Transfer back to map coordinate system
    mCar.vehicleToMapCoordinates(xPoint, yPoint);

    // Add to output
    trajectory.x.push_back(xPoint);
    trajectory.y.push_back(yPoint);
  }
  return trajectory;
}


double VehiclePathPlanner::calculateCost(const VehicleBehavior &behavior) const
{
  double cost = 0.0;

  // Collision cost
  const Trajectory &t = behavior.targetTrajectory;
  bool isColliding = false;
  for (int i = 0; i < t.size() && !isColliding; ++i)
  {
    // Setup future car
    double futureX = t.x[i];
    double futureY = t.y[i];
    double futureYaw = (i == 0) ? mCar.yaw : Tools::vangle(t.x[i] - t.x[i - 1], t.y[i] - t.y[i - 1]);
    std::vector<double> futureSd = mMap.getFrenet(futureX, futureY, futureYaw);
    Vehicle futureCar(futureX, futureY, futureSd[0], futureSd[1], futureYaw, behavior.targetSpeed);

    // Predict future obstacles
    double futureTime = (mCar.trajectory.size() + i) * 0.02;
    for (int j = 0; j < mObstacles.size() && !isColliding; ++j)
    {
      Vehicle futureObstacle = mObstacles[j].predict(mMap, futureTime);
      if (futureCar.isCollidingWith(futureObstacle))
      {
        cost += 100.0 / (futureTime * futureTime);
        //isColliding=  true;
        //std::cout << "Collision [" << futureTime << "] lane " <<  futureCar.getLane() << ", dist: " << futureObstacle.s - futureCar.s << ", " << futureObstacle.d - futureCar.d  << std::endl;
      }
    }
  }

  // Efficiency cost
  cost += std::abs(mMaxV - behavior.targetSpeed);
  return cost;
}


double VehiclePathPlanner::calculateApproachSpeed(double ownV, double otherV, double curDist, double minDist) const
{
  double desiredV = otherV;
  if (curDist > 0)
  {
    desiredV += curDist / minDist * (ownV - otherV);
  }
  else
  {
    desiredV += curDist / minDist * std::max(mMaxA, ownV - otherV);
  }
  return std::min(ownV + mMaxA, std::max(ownV - mMaxA, desiredV));
}

