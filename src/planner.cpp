//
// Created by josh on 7/23/17.
//

#include <cmath>
#include <vector>
#include "planner.h"
//#include "vehicle.h"
#include "map.h"
#include "spline.h"
#include "jmt.h"

Planner::Planner(Map map) : map(map) {
  NUM_POINTS_ = 50;
  TIME_STEP_ = 0.02;    // s
  MAX_ACCELERATION = 9.81;   //m/s^2
  SPEED_LIMIT = 45*0.44704;   //m/s
//  SPEED_LIMIT = 100*0.44704;   //m/s -> 100 MPH
  LANE_WIDTH = 4.0;    // Lane width

  // How much room to give to cars in lane
  FORWARD_BUFFER = 100.0;
  REARWARD_BUFFER = 50.0;
  lane_change_counter = 0;
  target_lane = 1;
  this->map = map;
//
//  s_trajectory = JMT();
//  s_trajectory.set({0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, 1.0);
//  d_trajectory = JMT();
//  d_trajectory.set({6.0, 0.0, 0.0}, {6.0, 0.0, 0.0}, 1.0);
}

void Planner::Update(double car_x,
                     double car_y,
                     double car_s,
                     double car_d,
                     double car_yaw,
                     double car_speed,
                     std::vector<double> previous_path_x,
                     std::vector<double> previous_path_y,
                     double end_s,
                     double end_d,
                     std::vector<Vehicle> tracked_vehicles) {
  // Update state
  this->car_x_= car_x;
  this->car_y_= car_y;
  this->car_s_= car_s;
  this->car_d_= car_d;
  this->car_yaw_= car_yaw*M_PI/180.0;
  this->car_speed_= car_speed*0.44704;    // convert to m/s
  this->previous_path_x_= previous_path_x;
  this->previous_path_y_= previous_path_y;
  this->end_s_= end_s;
  this->end_d_ = end_d;
  this->tracked_vehicles_ = tracked_vehicles;

  current_lane = car_d/LANE_WIDTH;
  if(previous_path_x_.size() > 0){
    current_lane = end_d_/LANE_WIDTH;
  }


  // Analyze traffic
  Planner::Traffic();

  // Examine options
  // costs {lane change left, keep lane and lane change right}
  std::vector<double> costs = {lane_change_counter, 0, lane_change_counter};

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
        costs[i+1] += 3.0 * (1.0 - closest_ahead[test_lane]/50.0);
        // Faster lane
        costs[i+1] += 3.0 * (1.0 - lane_speeds[test_lane]/SPEED_LIMIT);
        // Can change into lane
        costs[i+1] += 100000 * !open[test_lane];
      }
      std::cout << costs[i + 1] << " ";
    }
    std::cout << std::endl;

    // decide what to do
    double best_cost = costs[0];
    int best_option = 0;
    for (int i = 1; i < 3; i++) {
      if (costs[i] < best_cost) {
        best_cost = costs[i];
        best_option = i;
      }
    }

//    if(std::abs((car_d_-LANE_WIDTH/2)/LANE_WIDTH) > LANE_WIDTH/3.0){
//      lane_change_counter += 1;
//    }


    switch (best_option) {
      case 0:
//        KeepLane(current_lane-1);
        target_lane -= 1;
        target_lane = std::max(target_lane, 0);
        lane_change_counter += 100;
        break;
      case 1:
//        KeepLane(current_lane);
        lane_change_counter -= 1;
        lane_change_counter = std::max(lane_change_counter, 0);
        break;
      case 2:
//        KeepLane(current_lane+1);
        target_lane += 1;
        target_lane = std::min(target_lane, 2);
        lane_change_counter += 100;
        break;
    }
    KeepLane(target_lane);
  }
  else{
    KeepLane(current_lane);
  }


  // Run planner
//  KeepLane(1);
//  Planner::Circles();
}


void Planner::Traffic(){
  // Search each lane for closest leading and trailing car
  double forward_search_distance = 50;
  double rearward_search_distance = 50;

  closest_ahead = {forward_search_distance, forward_search_distance, forward_search_distance};
  closest_behind = {rearward_search_distance, rearward_search_distance, rearward_search_distance};
  lane_speeds = {SPEED_LIMIT, SPEED_LIMIT, SPEED_LIMIT};
  open = {true, true, true};

  int pos_s = car_s_;
  if(previous_path_x_.size() > 0){
    current_lane = end_d_/LANE_WIDTH;
    pos_s = end_s_;
  }

  double dt = TIME_STEP_*previous_path_x_.size();

  // Search each tracked vehicle
  for(int i=0; i<tracked_vehicles_.size(); i++) {
    int tracked_lane = tracked_vehicles_[i].d / LANE_WIDTH;
    double tracked_pos = tracked_vehicles_[i].predict(dt);
    // closer ahead?
    // is car ahead of us?
    if (tracked_pos > pos_s) {
      // Find slowest car ahead in lane
      if (tracked_vehicles_[i].v < lane_speeds[tracked_lane]) {
        lane_speeds[tracked_lane] = tracked_vehicles_[i].v;
      }
      // Find closest car ahead in lane
      if (tracked_pos - pos_s <= closest_ahead[tracked_lane]) {
        closest_ahead[tracked_lane] = tracked_pos - pos_s;
      }
    }
      // car is behind us
    else {
      if (pos_s - tracked_pos < closest_behind[tracked_lane]) {
        closest_behind[tracked_lane] = pos_s - tracked_pos;
      }
    }
  }

  // is the lane open?
  for(int i=0; i<3; i++){
    if(closest_ahead[i] < 10.0 || closest_behind[i] < 10.0){
      open[i] = false;
    }
  }
}

void Planner::KeepLane(int target_lane) {
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

  double target_d = target_lane*LANE_WIDTH + LANE_WIDTH/2.0;
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


  double delta_v = 2.0*(SPEED_LIMIT - lane_speeds[current_lane]);
  double target_speed = SPEED_LIMIT-delta_v + delta_v / (1.0+std::exp((-closest_ahead[current_lane] + 25.0)/3.0));


//  std::cout << "Car " << int(closest_ahead[current_lane]) << "m ahead moving at " << int(lane_speeds[current_lane]) << " m/s." << std::endl;

//  tk::spline velocity;
//  velocity.set_points({-0.01, 0.0, 1.0, 1.01}, {vel_s, vel_s, SPEED_LIMIT, SPEED_LIMIT});
//
//  std::cout << velocity(0.2) << std::endl;
//  std::cout << NUM_POINTS_-next_x_.size() << std::endl;


  std::vector<double> xy;

//  pos_d = current_lane*LANE_WIDTH + LANE_WIDTH/2.0;
  while(next_x_.size() < NUM_POINTS_){
    speed += (target_speed - speed)*0.01;
    pos_s += TIME_STEP_*speed;
//    pos_s += TIME_STEP_*target_speed;
//    pos_d += (target_d - pos_d)*0.001;
    pos_d = spline_d(pos_s);
//    double s = 10*(i+1);
//    double s = pos_s + t*velocity(t);
//    std::cout << s << " ";

    // getXY implements a spline smoothed path
    xy = map.getXY(pos_s, pos_d);
//    std::cout << xy[0] << " " << xy[1] << std::endl;
    next_x_.push_back(xy[0]);
    next_y_.push_back(xy[1]);
    next_s_.push_back(pos_s);
    next_d_.push_back(pos_d);
  }
//  xy = map.getXY(car_s_, car_d_);
//  std::cout << xy[0] << " " << xy[1]  << std::endl;
//  xy = map.getXY(pos_s, pos_d);
//  std::cout << xy[0] << " " << xy[1]  << std::endl;
//  std::cout << next_x_[0] << " " << next_y_[0]  << std::endl;
//  std::cout << std::endl;
}

//void Planner::ChangeLane(int target_lane){
//  double pos_s = car_s_;
//  double pos_d = car_d_;
//  double speed = car_speed_;
//
//  // clear previous
//  next_x_ = {};
//  next_y_ = {};
//
//  int path_size = previous_path_x_.size();
//
//  for(int i = 0; i < path_size; i++) {
//    next_x_.push_back(previous_path_x_[i]);
//    next_y_.push_back(previous_path_y_[i]);
//  }
//
//  // purge past points
//  while(next_s_.size() > path_size){
//    next_s_.erase(next_s_.begin());
//    next_d_.erase(next_d_.begin());
//  }
//
//  if(path_size > 1) {
//    pos_s = next_s_.back();
//    pos_d = next_d_.back();
//    speed = (next_s_.back() - next_s_[next_s_.size()-2])/TIME_STEP_;
//  }
//
//  double target_d = target_lane*LANE_WIDTH + LANE_WIDTH/2.0;
//  std::vector<double> ptss = {next_s_[next_s_.size()-2], next_s_.back(), next_s_.back()+30, next_s_.back()+35};
//  std::vector<double> ptsd = {next_d_[next_d_.size()-2], next_d_.back(), target_d, target_d};
//
//  tk::spline d;
//  d.set_points(ptss, ptsd);
//
//  std::vector<double> xy;
//
//
//
//
//  while(next_x_.size() < NUM_POINTS_){
//    speed += (lane_speeds[target_lane] - speed)*0.01;
//    pos_s += TIME_STEP_*speed;
//    pos_d = d(pos_s);
////    pos_s += TIME_STEP_*target_speed;
////    pos_d += (target_d - pos_d)*0.01;
////    double s = 10*(i+1);
////    double s = pos_s + t*velocity(t);
////    std::cout << s << " ";
//
//    // getXY implements a spline smoothed path
//    xy = map.getXY(pos_s, pos_d);
////    std::cout << xy[0] << " " << xy[1] << std::endl;
//    next_x_.push_back(xy[0]);
//    next_y_.push_back(xy[1]);
//    next_s_.push_back(pos_s);
//    next_d_.push_back(pos_d);
//  }
//}

void Planner::Circles(){
  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x_.size();

  // clear previous
  next_x_ = {};
  next_y_ = {};

  for(int i = 0; i < path_size; i++)
  {
    next_x_.push_back(previous_path_x_[i]);
    next_y_.push_back(previous_path_y_[i]);
  }

  if(path_size == 0)
  {
    pos_x = car_x_;
    pos_y = car_y_;
    angle = car_yaw_*M_PI/180;
  }
  else
  {
    pos_x = previous_path_x_[path_size-1];
    pos_y = previous_path_y_[path_size-1];

    double pos_x2 = previous_path_x_[path_size-2];
    double pos_y2 = previous_path_y_[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
  }

  double dist_inc = 0.5;
  for(int i = 0; i < NUM_POINTS_-path_size; i++)
  {
    next_x_.push_back(pos_x+(dist_inc)*cos(angle+(i+1)*(M_PI/100)));
    next_y_.push_back(pos_y+(dist_inc)*sin(angle+(i+1)*(M_PI/100)));
    pos_x += (dist_inc)*cos(angle+(i+1)*(M_PI/100));
    pos_y += (dist_inc)*sin(angle+(i+1)*(M_PI/100));
  }
}
