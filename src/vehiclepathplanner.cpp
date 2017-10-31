
#include "vehiclepathplanner.h"
#include "spline.h"
#include "tools.h"

VehiclePathPlanner::VehiclePathPlanner(const Map &map, double planningHorizon, double speedLimit)
  : mMap(map)
  , mCar(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
  , mObstacles()
  , mPlanningHorizon(planningHorizon)
  , mSpeedLimit(speedLimit)
  , mTargetSpeed(speedLimit)
  , mTargetLane(mCar.getLane())
{

}

void VehiclePathPlanner::updateState(const Vehicle &car, const std::vector<Vehicle> &obstacles)
{
  mCar = car;
  mObstacles = obstacles;

  // Check for collision with car ahead
  // Plan motion using target lane and speed
  mTargetLane = mCar.getLane();
  bool too_close = false;
  for (int i = 0; i < mObstacles.size(); ++i)
  {
    if (mObstacles[i].getLane() == mTargetLane)
    {
      double checkSpeed = mObstacles[i].v;
      double checkCarS = mObstacles[i].s + 0.02 * mCar.trajectory.size() * checkSpeed;
      if (checkCarS > mCar.s && checkCarS - mCar.s < 30)
      {
        too_close = true;
        if (mTargetLane > 0)
        {
          --mTargetLane;
        }
        else
        {
          ++mTargetLane;
        }
        break;
      }
    }
  }

  if (too_close)
  {
    mTargetSpeed -= 0.224;
  }
  else if (mTargetSpeed < mSpeedLimit)
  {
    mTargetSpeed += 0.224;
  }
}

Trajectory VehiclePathPlanner::getTrajectory() const
{
  Trajectory trajectory;

  // Generate base trajectory section from history
  for(int i = 0; i < mCar.trajectory.size(); ++i)
  {
    trajectory.x.push_back(mCar.trajectory.x[i]);
    trajectory.y.push_back(mCar.trajectory.y[i]);
  }

  // Generate sparse trajectory (past two points)
  std::vector<double> ptsX;
  std::vector<double> ptsY;
  int histLen = trajectory.size();
  if (histLen < 2)
  {
     // No history, backtrace one step from current state
    double prevCarX = mCar.x - 30 * cos(mCar.yaw);
    double prevCarY = mCar.y - 30 * sin(mCar.yaw);
    ptsX.push_back(prevCarX);
    ptsY.push_back(prevCarY);
    ptsX.push_back(mCar.x);
    ptsY.push_back(mCar.y);
  }
  else
  {
    // Use end of history
    double refXPrev = trajectory.x[histLen - 2];
    double refYPrev = trajectory.y[histLen - 2];
    ptsX.push_back(refXPrev);
    ptsY.push_back(refYPrev);
    ptsX.push_back(mCar.x);
    ptsY.push_back(mCar.y);
  }

  // Generate sparse trajectory (future three points)
  double dist_inc = 30;
  for(int i = 0; i < 3; ++i)
  {
    double s = mCar.s + (i + 1) * dist_inc;
    double d = 2.0 + 4.0 * mTargetLane;
    std::vector<double> xy = mMap.getXY(s, d);
    ptsX.push_back(xy[0]);
    ptsY.push_back(xy[1]);
  }

  // Convert trajectory to vehicle coordinate system
  for(int i = 0; i < ptsX.size(); ++i)
  {
    double shiftX = ptsX[i] - mCar.x;
    double shiftY = ptsY[i] - mCar.y;
    ptsX[i] = shiftX * cos(-mCar.yaw) - shiftY * sin(-mCar.yaw);
    ptsY[i] = shiftX * sin(-mCar.yaw) + shiftY * cos(-mCar.yaw);
  }

  // Match spline poly
  tk::spline fSpline;
  fSpline.set_points(ptsX, ptsY);

  // Fill up remainder of trajectory
  double targetX = 30.0;
  double targetY = fSpline(targetX);
  double targetDist = Tools::distance(0, 0, targetX, targetY);
  double xAddon = 0;

  for (int i = 1; i <=  50 - histLen; ++i)
  {
    double N = targetDist / (0.02 * mTargetSpeed);
    double xPoint = xAddon + targetX / N;
    double yPoint = fSpline(xPoint);

    xAddon = xPoint;

    double xRef = xPoint;
    double yRef = yPoint;

    // Transfer back to map coordinate system
    xPoint = xRef * cos(mCar.yaw) - yRef * sin(mCar.yaw);
    yPoint = xRef * sin(mCar.yaw) + yRef * cos(mCar.yaw);
    xPoint += mCar.x;
    yPoint += mCar.y;

    trajectory.x.push_back(xPoint);
    trajectory.y.push_back(yPoint);
  }


  return trajectory;
}
