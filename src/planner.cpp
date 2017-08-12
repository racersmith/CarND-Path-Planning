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
  SPEED_LIMIT = 49*0.44704;   //m/s -> 50 MPH
//  SPEED_LIMIT = 100*0.44704;   //m/s -> 100 MPH
  LANE_WIDTH = 4.0;    // Lane width

  // How much room to give to cars in lane
  FORWARD_BUFFER = 100.0;
  REARWARD_BUFFER = 50.0;

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

  // Run planner
  Planner::KeepLane();
//  Planner::Circles();
}


//void Planner::KeepLane() {
//  double pos_s = car_s_;
//  double pos_x = car_x_;
//  double pos_y = car_y_;
//  double pos_yaw = car_yaw_;
//  int target_lane = car_d_/LANE_WIDTH;
//  int prev_size = previous_path_x_.size();
//  // Clear old path
//  next_x_ = {};
//  next_y_ = {};
//
//  // waypoints for spline fit
//  std::vector<double> ptsx;
//  std::vector<double> ptsy;
//  std::vector<double> ptss;
//
//  if(prev_size < 2){
//    // Generate two points that are tangent to the car heading
//    double prev_x = car_x_ - std::cos(car_yaw_);
//    double prev_y = car_y_ - std::sin(car_yaw_);
//    std::vector<double> prev_frenet = map.getFrenet(prev_x, prev_y, car_yaw_);
////    ptss.push_back(prev_frenet[0]);
//    ptss.push_back(car_s_);
////    ptsx.push_back(prev_x);
//    ptsx.push_back(car_x_);
////    ptsy.push_back(prev_y);
//    ptsy.push_back(car_y_);
//  }
//  else{
//    pos_s = end_s_;
//    pos_x = previous_path_x_.back();
//    pos_y = previous_path_y_.back();
//
//    double last_x =previous_path_x_[prev_size - 2];
//    double last_y =previous_path_y_[prev_size - 2];
//
//    pos_yaw = std::atan2(pos_y - last_y, pos_x - last_x);
//
//    std::vector<double> prev_frenet = map.getFrenet(last_x, last_y, pos_yaw);
//
//    // Push back last two points to create tangent path
////    ptss.push_back(prev_frenet[0]);
//    ptss.push_back(end_s_);
////    ptsx.push_back(last_x);
//    ptsx.push_back(pos_x);
////    ptsy.push_back(last_y);
//    ptsy.push_back(pos_y);
//  }
//
//  double target_d =  target_lane*LANE_WIDTH + LANE_WIDTH/2.0;
//  for(int i=1; i<=3; i++){
//    double next_s = ptss.back() + 30;
//    std::vector<double> xy = map.getXY(next_s, target_d);
//    ptss.push_back(next_s);
//    ptsx.push_back(xy[0]);
//    ptsy.push_back(xy[1]);
//  }
//
//  tk::spline spline_x;
//  tk::spline spline_y;
//
//  for(int i=0; i<ptss.size(); i++){
//    std::cout << ptss[i] << " ";
//  }
//  std::cout << std::endl;
//
//  spline_x.set_points(ptss, ptsx);
//  spline_y.set_points(ptss, ptsy);
//
//  // Clear old path
//  next_x_ = {};
//  next_y_ = {};
//
//  // Add the remaining points in the old path
//  for(int i=0; i<prev_size; i++){
//    next_x_.push_back(previous_path_x_[i]);
//    next_y_.push_back(previous_path_y_[i]);
//  }
//
//  double speed = car_speed_;
//  while(next_x_.size() < NUM_POINTS_){
//    speed += (SPEED_LIMIT - speed)*0.02;
//    pos_s += speed*TIME_STEP_;
//    next_x_.push_back(spline_x(pos_s));
//    next_y_.push_back(spline_y(pos_s));
//  }
//}


void Planner::KeepLane() {
  double pos_s = car_s_;
  double pos_d = car_d_;
  double speed = car_speed_;

//  current_lane = 0;   // Test alt. target lane

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

  if(path_size > 1) {
    pos_s = next_s_.back();
    pos_d = next_d_.back();
    speed = (next_s_.back() - next_s_[next_s_.size() -2])/TIME_STEP_;
  }


  int current_lane = pos_d/LANE_WIDTH;


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
        s_ahead = tracked_vehicles_[i].s;
        v_ahead = tracked_vehicles_[i].v;
      }
//      else if(s_diff < 0 && s_diff > s_behind){
//        s_behind = s_diff;
//        v_behind = tracked_vehicles_[i].v;
//      }
    }
  }
  std::cout << "Car " << int(s_ahead) << "m ahead moving at " << int(v_ahead) << " m/s." << std::endl;

//  tk::spline velocity;
//  velocity.set_points({-0.01, 0.0, 1.0, 1.01}, {vel_s, vel_s, SPEED_LIMIT, SPEED_LIMIT});
//
//  std::cout << velocity(0.2) << std::endl;
//  std::cout << NUM_POINTS_-next_x_.size() << std::endl;

  std::cout << car_s_ << " " << pos_s + TIME_STEP_*SPEED_LIMIT << std::endl;
  std::vector<double> xy;


  double target_speed = SPEED_LIMIT;
  double target_d = current_lane*LANE_WIDTH + LANE_WIDTH/2.0;
  while(next_x_.size() < NUM_POINTS_){
    speed += (target_speed - speed)*0.01;
    pos_s += TIME_STEP_*speed;
//    pos_s += TIME_STEP_*target_speed;
//    pos_d += (target_d - pos_d)*0.001;
//    double s = 10*(i+1);
//    double s = pos_s + t*velocity(t);
//    std::cout << s << " ";

    // getXY implements a spline smoothed path
    xy = map.getXY(pos_s, target_d);
//    std::cout << xy[0] << " " << xy[1] << std::endl;
    next_x_.push_back(xy[0]);
    next_y_.push_back(xy[1]);
    next_s_.push_back(pos_s);
    next_d_.push_back(target_d);
  }
//  xy = map.getXY(car_s_, car_d_);
//  std::cout << xy[0] << " " << xy[1]  << std::endl;
//  xy = map.getXY(pos_s, pos_d);
//  std::cout << xy[0] << " " << xy[1]  << std::endl;
//  std::cout << next_x_[0] << " " << next_y_[0]  << std::endl;
//  std::cout << std::endl;
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
