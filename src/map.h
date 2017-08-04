//
// Created by josh on 8/1/17.
//

#ifndef PATH_PLANNING_MAP_H
#define PATH_PLANNING_MAP_H

#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "planner.h"

class Map {
public:
  // Initialize map
  Map(std::string map_file);

  // Default destructor
  ~Map() = default;

  // Find closest waypoint from given x,y position.
  int ClosestWaypoint(double x, double y);

  // Find next waypoint
  int NextWaypoint(double x, double y, double theta);

  // Get the map frenet coordinates from given state
  std::vector<double> getFrenet(double x, double y, double theta);

  // Get the x, y coordinates from frenet coordinates
  std::vector<double> getXY(double s, double d);

//private:
  // Data
  std::vector<double> map_waypoints_x;
  std::vector<double> map_waypoints_y;
  std::vector<double> map_waypoints_s;
  std::vector<double> map_waypoints_dx;
  std::vector<double> map_waypoints_dy;

  // Length of one lap
  double lap_length;

  // Max map index
  int n_waypoints;
};


#endif //PATH_PLANNING_MAP_H
