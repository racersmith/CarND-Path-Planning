//
// Created by josh on 7/23/17.
//

#include <cmath>
#include <vector>
#include "planner.h"
//#include "vehicle.h"
#include "map.h"
#include "spline.h"

Planner::Planner(Map map) : map(map) {
  NUM_POINTS_ = 100;
  TIME_STEP_ = 0.02;    // s
  MAX_ACCELERATION = 9.81;   //m/s^2
//  SPEED_LIMIT = 22.0;   //m/s -> 50 MPH
  SPEED_LIMIT = 44.0;   //m/s -> 50 MPH
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

  int next_waypoint_index = map.NextWaypoint(pos_x, pos_y, angle);

  for(int i=0; i<5; i++){
    int waypoint_index = (next_waypoint_index + i)%map.n_waypoints;
    double map_s = map.map_waypoints_s[waypoint_index];
    if(map_s < next_waypoints_s.back()){
      map_s += map.lap_length;
    }
    next_waypoints_s.push_back(map_s);
    next_waypoints_x.push_back(map.map_waypoints_x[waypoint_index]);
    next_waypoints_y.push_back(map.map_waypoints_y[waypoint_index]);
    std::cout << next_waypoints_s.back() << "  ";
    std::cout << next_waypoints_x.back() << "  ";
    std::cout << next_waypoints_y.back() << std::endl;
  }
  std::cout << std::endl;
  tk::spline spline_x;
  tk::spline spline_y;

  spline_x.set_points(next_waypoints_s, next_waypoints_x);
  spline_y.set_points(next_waypoints_s, next_waypoints_y);


  for(int i=0; i<NUM_POINTS_-path_size; i++)
  {
//    std::vector<double> next_xy = map.getXY(car_s_+i*dist_inc, car_d_);
//    next_x_.push_back(next_xy[0]);
//    next_y_.push_back(next_xy[1]);
    next_x_.push_back(spline_x(pos_s + (i+1)*TIME_STEP_*SPEED_LIMIT));
    next_y_.push_back(spline_y(pos_s + (i+1)*TIME_STEP_*SPEED_LIMIT));
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
