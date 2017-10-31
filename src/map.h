#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>

class Map
{
public:
  /**
   * @brief Constructs waypoint map
   * @param mapFile Waypoint map csv file to read from
   */
  explicit Map(const std::string &mapFile);

public:
  /**
   * @brief ClosestWaypoint
   * @param x
   * @param y
   * @param maps_x
   * @param maps_y
   * @return
   */
  int closestWaypoint(double x, double y) const;

  /**
   * @brief NextWaypoint
   * @param x
   * @param y
   * @param theta
   * @param maps_x
   * @param maps_y
   * @return
   */
  int nextWaypoint(double x, double y, double theta) const;

  /**
   * @brief Transform from Cartesian x,y coordinates to Frenet s,d coordinates
   * @param x
   * @param y
   * @param theta
   * @return
   */
  std::vector<double> getFrenet(double x, double y, double theta) const;

  /**
   * @brief Transform from Frenet s,d coordinates to Cartesian x,y
   * @param s
   * @param d
   * @return
   */
  std::vector<double> getXY(double s, double d) const;

private:
  std::vector<double> mWaypointsX;
  std::vector<double> mWaypointsY;
  std::vector<double> mWaypointsS;
  std::vector<double> mWaypointsDx;
  std::vector<double> mWaypointsDy;
};

#endif // MAP_H
