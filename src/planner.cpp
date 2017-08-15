//
// Created by josh smith on 7/23/17.
//

#include <cmath>
#include <vector>
#include "planner.h"

Planner::Planner(Map map) : map(map) {
  NUM_POINTS_ = 50;
  TIME_STEP_ = 0.02;    // s
  SPEED_LIMIT_ = 46.5*0.44704;   // m/s
  LANE_WIDTH_ = 3.9;    // Lane width, Adjusted to compensate for simulator outside of lane error in lane 2.

  // How much room to give to cars in lane
  FORWARD_BUFFER_ = 8.0;    // Improvement would be to make this variable to speed
  REARWARD_BUFFER_ = 12.0;  // Improvement would be to make this variable to speed
  lane_change_counter = 0;
  target_lane = 1;
  this->map = map;
}

void Planner::Update(double car_s,
                     double car_d,
                     double car_speed,
                     std::vector<double> previous_path_x,
                     std::vector<double> previous_path_y,
                     double end_s,
                     double end_d,
                     std::vector<Vehicle> tracked_vehicles) {
  // Update state
  this->car_s_= car_s;
  this->car_d_= car_d;
  this->car_speed_= car_speed*0.44704;    // convert to m/s
  this->previous_path_x_= previous_path_x;
  this->previous_path_y_= previous_path_y;
  this->end_s_= end_s;
  this->end_d_ = end_d;
  this->tracked_vehicles_ = tracked_vehicles;

  current_lane = car_d_/LANE_WIDTH_;
  if(previous_path_x_.size() > 0){
    current_lane = end_d_/LANE_WIDTH_;
  }

  // Analyze traffic
  Planner::Traffic();

  // Examine options
  // costs {lane change left, keep lane and lane change right}
  std::vector<double> costs = {lane_change_counter+0.6, 0.4, lane_change_counter};

  if(previous_path_x_.size() > 2) {
    // Run cost analysis of options
    for (int i = -1; i <= 1; i++) {
      int test_lane = current_lane + i;
      // prefer to stay in lane
      costs[i+1] += 0.5 * i * i;

      // Don't drive off the road
      if (test_lane < 0 || test_lane > 2) {
        costs[i+1] += 1000000;
      } else {
        // Possible progress
        costs[i+1] += 3.0 * (1.0 - closest_ahead_[test_lane]/30.0);
        // Faster lane
        costs[i+1] += 3.0 * (1.0 - lane_speeds_[test_lane]/SPEED_LIMIT_);
        // Can change into lane
        costs[i+1] += 100000 * !is_lane_open_[test_lane];
      }
      // Look at what a double lane change would do for us
      int test_lane_2 = current_lane + 2*i;
      if (test_lane_2 >=0 && test_lane_2 <= 2){
        // Does a lane change give us better progress opportunity?
        costs[i+1] += 1.5 * (1.0 - closest_ahead_[test_lane_2]/30.0);
        // Does a lane change put us closer to a faster lane?
        costs[i+1] += 1.5 * (1.0 - lane_speeds_[test_lane_2]/SPEED_LIMIT_);
      }
    }

    // decide what to do
    double best_cost = costs[0];
    int best_option = 0;
    for (int i = 1; i < 3; i++) {
      if (costs[i] < best_cost) {
        best_cost = costs[i];
        best_option = i;
      }
    }

    switch (best_option) {
      case 0:
        target_lane -= 1;
        target_lane = std::max(target_lane, 0);
        lane_change_counter = 110;
        break;
      case 1:
        lane_change_counter -= 1;
        lane_change_counter = std::max(lane_change_counter, 0);
        break;
      case 2:
        target_lane += 1;
        target_lane = std::min(target_lane, 2);
        lane_change_counter = 110;
        break;
    }
    UpdatePath(target_lane);
  }
  else{
    UpdatePath(current_lane);
  }
}


void Planner::Traffic(){
  // Search each lane for closest leading and trailing car
  double forward_search_distance = 30;
  double rearward_search_distance = 30;

  closest_ahead_ = {forward_search_distance, forward_search_distance, forward_search_distance};
  closest_behind_ = {rearward_search_distance, rearward_search_distance, rearward_search_distance};
  lane_speeds_ = {SPEED_LIMIT_, SPEED_LIMIT_, SPEED_LIMIT_};
  is_lane_open_ = {true, true, true};

  int pos_s = car_s_;
  if(previous_path_x_.size() > 0){
    current_lane = end_d_/LANE_WIDTH_;
    pos_s = end_s_;
  }

  double dt = TIME_STEP_*previous_path_x_.size();

  // Search each tracked vehicle
  for(int i=0; i<tracked_vehicles_.size(); i++) {
    int tracked_lane = tracked_vehicles_[i].d / LANE_WIDTH_;
    double tracked_pos = tracked_vehicles_[i].predict(dt);
    // closer ahead?
    // is car ahead of us?
    if (tracked_pos > pos_s) {
      // Find slowest car ahead in lane
      if (tracked_vehicles_[i].v < lane_speeds_[tracked_lane]) {
        lane_speeds_[tracked_lane] = tracked_vehicles_[i].v;
      }
      if (tracked_pos - pos_s <= closest_ahead_[tracked_lane]) {
        closest_ahead_[tracked_lane] = tracked_pos - pos_s;
      }
    }
    // car is behind us
    else {
      if (pos_s - tracked_pos < closest_behind_[tracked_lane]) {
        closest_behind_[tracked_lane] = pos_s - tracked_pos;
      }
    }
  }

  // is the lane open?
  for(int i=0; i<3; i++){
    if(closest_ahead_[i] < FORWARD_BUFFER_ || closest_behind_[i] < REARWARD_BUFFER_){
      is_lane_open_[i] = false;
    }
  }
}

void Planner::UpdatePath(int target_lane) {
  double pos_s = car_s_;
  double pos_d = car_d_;
  double speed = car_speed_;

  // clear previous
  next_x_ = {};
  next_y_ = {};

  int path_size = previous_path_x_.size();

  for(int i = 0; i < path_size; i++) {
    next_x_.push_back(previous_path_x_[i]);
    next_y_.push_back(previous_path_y_[i]);
  }

  // purge past points
  while(next_s_.size() > path_size){
    next_s_.erase(next_s_.begin());
    next_d_.erase(next_d_.begin());
  }

  double target_d = target_lane*LANE_WIDTH_ + LANE_WIDTH_/2.0;
  tk::spline spline_d;
  if(path_size >= 2) {
    pos_s = next_s_.back();
    pos_d = next_d_.back();
    speed = (next_s_.back() - next_s_[next_s_.size() -2])/TIME_STEP_;


    spline_d.set_points(
        {next_s_[next_s_.size() -2], next_s_.back(), next_s_.back()+30, next_s_.back()+60},
        {next_d_[next_d_.size() -2], next_d_.back(), target_d, target_d});
  }
  else{
    spline_d.set_points(
        {car_s_-1, car_s_, car_s_+30, car_s_+60},
        {car_d_, car_d_, target_d, target_d});
  }


  double delta_v = 2.0*(SPEED_LIMIT_ - lane_speeds_[current_lane]);
  double target_speed = SPEED_LIMIT_-delta_v + delta_v / (1.0+std::exp((-closest_ahead_[current_lane] + 15.0)/2.0));

  std::vector<double> xy;

  while(next_x_.size() < NUM_POINTS_){
    speed += (target_speed - speed)*0.01;
    pos_s += TIME_STEP_*speed;
    pos_d = spline_d(pos_s);

    // getXY implements a spline smoothed path
    xy = map.getXY(pos_s, pos_d);

    // append our future state vectors
    next_x_.push_back(xy[0]);
    next_y_.push_back(xy[1]);
    next_s_.push_back(pos_s);
    next_d_.push_back(pos_d);
  }
}
