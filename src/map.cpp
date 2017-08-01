//
// Created by josh on 8/1/17.
//


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

#include "map.h"


double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}


Map::Map(std::string map_file) {
  std::ifstream in_map_(map_file.c_str(), std::ifstream::in);

  std::string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }



  int size = map_waypoints_x.size();
  double last_step = distance(map_waypoints_x[0], map_waypoints_y[0],
                              map_waypoints_x[size], map_waypoints_y[size]);

  // Determine the roll over point
  max_s = map_waypoints_s[size] + last_step;

  // Overlap the map to the start
  map_waypoints_x.push_back(map_waypoints_x[0]);
  map_waypoints_y.push_back(map_waypoints_y[0]);
  map_waypoints_s.push_back(max_s);
  map_waypoints_dx.push_back(map_waypoints_dx[0]);
  map_waypoints_dy.push_back(map_waypoints_dy[0]);

  map_size = map_waypoints_x.size();
}


int Map::ClosestWaypoint(double x, double y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < map_size; i++)
  {
    double map_x = map_waypoints_x[i];
    double map_y = map_waypoints_y[i];
    double dist = distance(x, y, map_x, map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;
}



int Map::NextWaypoint(double x, double y, double theta)
{

  int closestWaypoint = Map::ClosestWaypoint(x,y);

  double map_x = map_waypoints_x[closestWaypoint];
  double map_y = map_waypoints_y[closestWaypoint];

  double heading = atan2( (map_y-y),(map_x-x) );

  double angle = abs(theta-heading);

  if(angle > M_PI/4)
  {
    closestWaypoint++;
  }

  return closestWaypoint;

}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
std::vector<double> Map::getFrenet(double x, double y, double theta)
{
  int next_wp = Map::NextWaypoint(x, y, theta);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = map_size-1;
  }

  double n_x = map_waypoints_x[next_wp]-map_waypoints_x[prev_wp];
  double n_y = map_waypoints_y[next_wp]-map_waypoints_y[prev_wp];
  double x_x = x - map_waypoints_x[prev_wp];
  double x_y = y - map_waypoints_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-map_waypoints_x[prev_wp];
  double center_y = 2000-map_waypoints_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(map_waypoints_x[i],map_waypoints_y[i],map_waypoints_x[i+1],map_waypoints_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}


// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> Map::getXY(double s, double d)
{
  int prev_wp = -1;

  while(s > map_waypoints_s[prev_wp+1] && (prev_wp < (int)(map_waypoints_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%map_size;

  double heading = atan2((map_waypoints_y[wp2]-map_waypoints_y[prev_wp]),(map_waypoints_x[wp2]-map_waypoints_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-map_waypoints_s[prev_wp]);

  double seg_x = map_waypoints_x[prev_wp]+seg_s*cos(heading);
  double seg_y = map_waypoints_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-M_PI/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}
