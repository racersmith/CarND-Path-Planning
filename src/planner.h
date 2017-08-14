//
// Created by josh on 7/23/17.
//

#ifndef PATH_PLANNING_PLANNER_H
#define PATH_PLANNING_PLANNER_H

#include <vector>
#include "vehicle.h"
#include "map.h"
#include "jmt.h"

class Planner {
private:
  // Number of future points to generate
  int NUM_POINTS_;

  // Time steps between points
  double TIME_STEP_;

  // Speed Limit
  double SPEED_LIMIT_;
  double LANE_WIDTH_;

  double FORWARD_BUFFER_;
  double REARWARD_BUFFER_;

  // Course map
  Map map;

  // Traffic
  std::vector<double> closest_ahead_;
  std::vector<double> closest_behind_;
  std::vector <double> lane_speeds_;
  std::vector <bool> is_lane_open_;

public:
  std::vector<double> next_x_;
  std::vector<double> next_y_;
  std::vector<double> next_s_;
  std::vector<double> next_d_;
  double car_s_;
  double car_d_;
  double car_speed_;
  std::vector<double> previous_path_x_;
  std::vector<double> previous_path_y_;
  double end_s_;
  double end_d_;
  int current_lane;
  std::vector<Vehicle> tracked_vehicles_;
  int lane_change_counter;
  int target_lane;

  // Constructor
  Planner(Map map);

  // Destructor
  ~Planner()= default;

  /*
   * Run planner on current state
   */
  void Update(double car_s, double car_d, double car_speed,
              std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_s, double end_d,
              std::vector<Vehicle> tracked_vehicles);

  /*
   * Analyze traffic flow
   */
  void Traffic();


  /*
   * Stay in lane and avoid forward and rearward collisions
   */
  void UpdatePath(int target_lane);
};



#endif //PATH_PLANNING_PLANNER_H
