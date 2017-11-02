#ifndef VEHICLE_H
#define VEHICLE_H

#include <vector>
#include <assert.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"


class Map;


/**
 * @brief Describes a trajectory path of a vehicle.
 */
struct Trajectory
{
  typedef std::vector<double> TPoints;

  /**
   * @brief Constructs an empty trajectory.
   */
  Trajectory() : x(), y() { }

  /**
   * @brief Constructs a trajectory with given positions
   * @param pointsX x coordinates of vehicle in map coordinate system
   * @param pointsY y coordinates of vehicle in map coordinate system
   */
  Trajectory(const TPoints &pointsX, const TPoints &pointsY) : x(pointsX), y(pointsY) { }

  /**
   * @brief Retrieves the length of the trajectory.
   * @return Number of coordinates in trajectory
   */
  int size() const { assert(x.size() == y.size()); return x.size(); }

  /** x coordinates */
  TPoints x;

  /** y coordinates */
  TPoints y;
};


/**
 * @brief Describes a moving vehicle on the map.
 */
class Vehicle
{
public:
  /**
   * @brief Constructs new vehicle object.
   * @param xPos x coordinate of vehicle in map coordinate system
   * @param yPos y coordinate of vehicle in map coordinate system
   * @param sPos s coordinate of vehicle in Frenet coordinate system
   * @param dPos d coordinate of vehicle in Frenet coordinate system
   * @param yawAngle yaw angle of vehicle in radians in map coordinate system
   * @param velocity velocity of vehicle in m/s
   * @param trajectoryPath optional already planned trajectory path
   * @param trajectoryS s coordinate of trajectory path end
   * @param trajectoryD d coordinate of trajectory path end
   */
  Vehicle(double xPos, double yPos, double sPos, double dPos, double yawAngle, double velocity,
          const Trajectory &trajectoryPath = Trajectory(), double trajectoryS = 0.0, double trajectoryD = 0.0);

public:
  /**
   * @brief Maps coordinates in vehicle system to map coordinate.
   * @param inOutX x coordinate
   * @param inOutY y coordinate
   */
  void vehicleToMapCoordinates(double &inOutX, double &inOutY) const;

  /**
   * @brief Maps coordinates in vehicle system to map coordinate.
   * @param inOutX x coordinate
   * @param inOutY y coordinate
   */
  void mapToVehicleCoordinates(double &inOutX, double &inOutY) const;

  /**
   * @brief Predicts a future vehicle clone based on time delta
   * @param map Map used for coordinate conversion
   * @param dt time delta in s
   * @return
   */
  Vehicle predict(const Map &map, double dt) const;

  /**
   * @brief Retrieves buffer distance for car at current speed
   * @param friction Road friction
   * @return buffer distance in m
   */
  double getSafetyDistance(double friction) const;

  /**
   * @brief Retrieves whether this vehicle is colliding with another one
   * @param other the other vehicle
   * @return true if colliding, false otherwise
   */
  bool isCollidingWith(const Vehicle &other) const;

  /**
   * @brief Retrives the current lane of the vehicle.
   * @return lane number 0 to n
   */
  int getLane() const;

public:
  /** x coordinate of vehicle in map coordinate system */
  double x;

  /** y coordinate of vehicle in map coordinate system */
  double y;

  /** s coordinate of vehicle in Frenet coordinate system */
  double s;

  /** d coordinate of vehicle in Frenet coordinate system */
  double d;

  /** yaw angle of vehicle in radians in map coordinate system */
  double yaw;

  /** velocity of vehicle in m/s */
  double v;

  /** already planned trajectory path */
  Trajectory trajectory;

  /** bounding coorindates in Frenet coordinate system */
  Eigen::Vector2d bounds[4];

  /** horizontal vehicle extent (in heading direction) */
  double radiusX;

  /** vertical vehicle extent (in heading direction) */
  double radiusY;
};

#endif // VEHICLE_H
