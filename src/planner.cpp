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
//  SPEED_LIMIT = 22.0;   //m/s -> 50 MPH
  SPEED_LIMIT = 44.0;   //m/s -> 100 MPH
  LANE_WIDTH = 4.0;    // Lane width

  // How much room to give to cars in lane
  FORWARD_BUFFER = 100.0;
  REARWARD_BUFFER = 50.0;

  this->map = map;
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
  this->car_yaw_= car_yaw;
  this->car_speed_= car_speed;
  this->previous_path_x_= previous_path_x;
  this->previous_path_y_= previous_path_y;
  this->end_s_= end_s;
  this->end_d_ = end_d;
  this->tracked_vehicles_ = tracked_vehicles;

  // Run planner
  Planner::KeepLane();
//  Planner::Circles();
}

void Planner::KeepLane() {
  double pos_s;
  double pos_x;
  double pos_y;
  double angle;
  int path_size = previous_path_x_.size();
  int current_lane = car_d_/LANE_WIDTH;
//  current_lane = 0;   // Test alt. target lane

  // clear previous
  next_x_ = {};
  next_y_ = {};

  // Add current position to spline fit points
  std::vector<double> next_waypoints_s = {car_s_};
  std::vector<double> next_waypoints_x = {car_x_};
  std::vector<double> next_waypoints_y = {car_y_};

  for(int i = 0; i < path_size; i++)
  {
    next_x_.push_back(previous_path_x_[i]);
    next_y_.push_back(previous_path_y_[i]);
  }

  if(path_size == 0)
  {
    pos_s = car_s_;
    pos_x = car_x_;
    pos_y = car_y_;
    angle = car_yaw_*M_PI/180;
  }
  else
  {
    pos_s = end_s_;
    if(pos_s < next_waypoints_s.back()){
      pos_s += map.lap_length;
    }
    pos_x = previous_path_x_[path_size-1];
    pos_y = previous_path_y_[path_size-1];

    double pos_x2 = previous_path_x_[path_size-2];
    double pos_y2 = previous_path_y_[path_size-2];
    angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

    // Add the final old trajectory point to spline fit points
    next_waypoints_s.push_back(pos_s);
    next_waypoints_x.push_back(pos_x);
    next_waypoints_y.push_back(pos_y);
  }

  // Find car ahead and car behind in same lane
  double s_ahead = 1000.0;
  double v_ahead = SPEED_LIMIT;
  double s_behind = -1000.0;
  double v_behind = SPEED_LIMIT;
  for(int i=0; i<tracked_vehicles_.size(); i++){
    int lane = tracked_vehicles_[i].d/LANE_WIDTH;
    // Check if the car is in the same lane
    if(lane == current_lane){
      double s_diff = tracked_vehicles_[i].s - car_s_;
      if(s_diff > 0 && s_diff < s_ahead){
        s_ahead = s_diff;
        v_ahead = tracked_vehicles_[i].v;
      }
      else if(s_diff < 0 && s_diff > s_behind){
        s_behind = s_diff;
        v_behind = tracked_vehicles_[i].v;
      }
    }
  }
  std::cout << "Car " << int(s_ahead) << " ahead moving at " << int(v_ahead) << " m/s." << std::endl;

  for(int i=0; i<NUM_POINTS_-path_size; i++)
  {
//    next_x_.push_back(spline_x(pos_s + (i+1)*TIME_STEP_*SPEED_LIMIT));
//    next_y_.push_back(spline_y(pos_s + (i+1)*TIME_STEP_*SPEED_LIMIT));
//    double s = pos_s + (i+1)*TIME_STEP_*v_ahead;
    double s = pos_s + (i+1)*TIME_STEP_*SPEED_LIMIT*5;
    std::vector<double> xy = map.getXY(s, current_lane*LANE_WIDTH + LANE_WIDTH/2);

//    next_x_.push_back(spline_x(pos_s + (i+1)*TIME_STEP_*v_ahead));
//    next_y_.push_back(spline_y(pos_s + (i+1)*TIME_STEP_*v_ahead));
    next_x_.push_back(xy[0]);
    next_y_.push_back(xy[1]);
  }
}

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
