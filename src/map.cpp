
#include "map.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <limits>
#include "tools.h"


Map::Map(const std::string &mapFile)
: mWaypointsX()
, mWaypointsY()
, mWaypointsS()
, mWaypointsDx()
, mWaypointsDy()
{
  // The max s value before wrapping around the track back to 0
  static const double maxS = 6945.554;
  std::ifstream inStream(mapFile.c_str(), std::ifstream::in);

  std::string line;
  while (std::getline(inStream, line))
  {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float dX;
    float dY;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> dX;
    iss >> dY;
    mWaypointsX.push_back(x);
    mWaypointsY.push_back(y);
    mWaypointsS.push_back(s);
    mWaypointsDx.push_back(dX);
    mWaypointsDy.push_back(dY);
  }
}


int Map::closestWaypoint(double x, double y) const
{
  double closestLen(std::numeric_limits<double>::max());
  int closestWaypoint(0);
  for(int i = 0; i < mWaypointsX.size(); ++i)
  {
    double mapX = mWaypointsX[i];
    double mapY = mWaypointsY[i];
    double dist = Tools::distance(x, y, mapX, mapY);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }
  return closestWaypoint;
}


int Map::nextWaypoint(double x, double y, double theta) const
{
  int closestWpIdx = closestWaypoint(x,y);
  double mapX = mWaypointsX[closestWpIdx];
  double mapY = mWaypointsY[closestWpIdx];
  double heading = atan2( (mapY - y),(mapX - x) );
  double angle = abs(theta - heading);
  if(angle > Tools::pi() / 4)
  {
    ++closestWpIdx;
  }
  return closestWpIdx;
}


std::vector<double> Map::getFrenet(double x, double y, double theta) const
{
  int next_wp = nextWaypoint(x,y, theta);
  int prev_wp = next_wp - 1;
  if(next_wp == 0)
  {
    prev_wp  = mWaypointsX.size() - 1;
  }

  double n_x = mWaypointsX[next_wp]- mWaypointsX[prev_wp];
  double n_y = mWaypointsY[next_wp]- mWaypointsY[prev_wp];
  double x_x = x - mWaypointsX[prev_wp];
  double x_y = y - mWaypointsY[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = Tools::distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - mWaypointsX[prev_wp];
  double center_y = 2000 - mWaypointsY[prev_wp];
  double centerToPos = Tools::distance(center_x,center_y,x_x,x_y);
  double centerToRef = Tools::distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += Tools::distance(mWaypointsX[i],mWaypointsY[i],mWaypointsX[i+1],mWaypointsY[i+1]);
  }

  frenet_s += Tools::distance(0, 0, proj_x,proj_y);

  return {frenet_s, frenet_d};
}


std::vector<double> Map::getXY(double s, double d) const
{
  int prev_wp = -1;

  while(s > mWaypointsS[prev_wp + 1] && prev_wp < static_cast<int>(mWaypointsS.size() - 1))
  {
    ++prev_wp;
  }

  int wp2 = (prev_wp + 1) % mWaypointsX.size();

  double heading = atan2((mWaypointsY[wp2]-mWaypointsY[prev_wp]),(mWaypointsX[wp2]-mWaypointsX[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - mWaypointsS[prev_wp]);

  double seg_x = mWaypointsX[prev_wp] + seg_s * cos(heading);
  double seg_y = mWaypointsY[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - Tools::pi() / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x, y};
}

