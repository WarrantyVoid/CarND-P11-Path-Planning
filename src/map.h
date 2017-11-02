#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>

/**
 * @brief The udacity provided functionality wrapped inside a class.
 */
class Map
{
public:
  /**
   * @brief Constructs waypoint map from a given CSV file.
   * @param mapFile Waypoint map csv file to read from
   */
  explicit Map(const std::string &mapFile);

public:
  /**
   * @brief Retrieves closest waypoint
   * @param x map x coordinate of reference point
   * @param y map y coordinate of reference point
   * @return index of closest waypoint
   */
  int closestWaypoint(double x, double y) const;

  /**
   * @brief Retrives next waypoint in heading direction.
   * @param x map x coordinate of reference point
   * @param y map y coordinate of reference point
   * @param theta heading direction in radians
   * @return index of next waypoint
   */
  int nextWaypoint(double x, double y, double theta) const;

  /**
   * @brief Transform from Cartesian x,y coordinates to Frenet s,d coordinates
   * @param x map x coordinate of reference point
   * @param y map y coordinate of reference point
   * @param theta heading direction in radians
   * @return vector containing s and d
   */
  std::vector<double> getFrenet(double x, double y, double theta) const;

  /**
   * @brief Transform from Frenet s,d coordinates to Cartesian x,y
   * @param s Frenet s coordinate of reference point
   * @param d Frenet d coordinate of reference point
   * @return vector containing x and y
   */
  std::vector<double> getXY(double s, double d) const;

private:
  /** list of waypoint x coordinates */
  std::vector<double> mWaypointsX;

  /** list of waypoint y coordinates */
  std::vector<double> mWaypointsY;

  /** list of waypoint s coordinates */
  std::vector<double> mWaypointsS;

  /** list of waypoint deltas x */
  std::vector<double> mWaypointsDx;

  /** list of waypoint deltas y */
  std::vector<double> mWaypointsDy;
};

#endif // MAP_H
