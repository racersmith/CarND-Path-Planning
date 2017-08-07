//
// Created by josh on 8/6/17.
//

#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H

#include <vector>


/*
 * Jerk Minimizing Trajectory
 * Determine the 6th order polynomial that
 * minimizes the jerk between a initial and
 * final state in 1-D.
 */

class JMT {
public:
  // Constructor
  JMT(){};

  // Default destructor
  ~JMT() = default;

  // Set starting and ending state and transistion time
  void set(std::vector<double> start, std::vector<double> end, double time);

  // Target position at time
  double operator() (double t) const;

private:
  double T;
  double T2;
  double T3;
  double T4;
  double T5;
  std::vector<double> trajectory_coeff = {};
};


#endif //PATH_PLANNING_JMT_H
