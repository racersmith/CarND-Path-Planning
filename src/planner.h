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

  // Maximum Acceleration
  double MAX_ACCELERATION;

  // Speed Limit
  double SPEED_LIMIT;
  double LANE_WIDTH;

  double FORWARD_BUFFER;
  double REARWARD_BUFFER;

  // Course map
  Map map;

  // Traffic
  std::vector<double> closest_ahead;
  std::vector<double> closest_behind;
  std::vector <double> lane_speeds;
  std::vector <bool> open;

public:
  std::vector<double> next_x_;
  std::vector<double> next_y_;
  std::vector<double> next_s_;
  std::vector<double> next_d_;
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
  Planner(Map map);

  // Destructor
  ~Planner()= default;

  /*
   * Run planner on current state
   */
  void Update(double car_x, double car_y, double car_s, double car_d, double car_yaw, double car_speed,
              std::vector<double> previous_path_x, std::vector<double> previous_path_y, double end_s, double end_d,
              std::vector<Vehicle> tracked_vehicles);

  /*
   * Analyze traffic flow
   */
  void Traffic();


  /*
   * Stay in lane and avoid forward and rearward collisions
   */
  void KeepLane();

  /*
   * Change lane
   */
  void ChangeLane(int target_lane, double target_speed);

  /*
   * Driving in circles example
   */
  void Circles();

};



#endif //PATH_PLANNING_PLANNER_H
