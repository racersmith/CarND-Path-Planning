//
// Created by josh on 7/23/17.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include "vehicle.h"

class Planner {
private:
  // Number of future points to generate
  int NUM_POINTS_;

  // Time steps between points
  float TIME_STEP_;

public:


  std::vector<double> next_x_;
  std::vector<double> next_y_;
  double car_x_;
  double car_y_;
  double car_s_;
  double car_d_;
  double car_yaw_;
  double car_speed_;
  std::vector<double> previous_path_x_;
  std::vector<double> previous_path_y_;
  double end_s_;
  double end_d_;
  std::vector<Vehicle> tracked_vehicles_;

  // Constructor
  Planner();

  // Destructor
  ~Planner()= default;

  /*
   * Run planner on current state
   */
  void Update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
              std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_s, double end_d,
              std::vector<Vehicle> tracked_vehicles);

  /*
   * Stay in lane and avoid forward and rearward collisions
   */
  void KeepLane();

};



#endif //PATH_PLANNING_PLANNER_H
