# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program
   
### Simulator. You can download the Term3 Simulator BETA which contains the Path Planning Project from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).

In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 50 m/s^3.

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, 
car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet 
coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. 
The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector 
going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and 
normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that 
the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 
50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, 
it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized
 code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points 
 that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth 
 transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points 
 given to the simulator controller with the processed points already removed. You would either return a path that 
 extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using 
http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single header file is really easy to use.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!


## Path Planner

The path planner is implented in a few stages; smooth, predict, plan, generate.

### Smooth
The first stage was to generate a smooth path from the list of sparse map wayoints.  This was done using the superb
cubic spline library from [ttk592](https://github.com/ttk592/spline/).  The spline takes a set of x, y vectors and 
generates a cubic spline that can then be called as a function.  The one caveat is that it requires the x vector to be
sorted.  This could be done by doing a coordinate transform such that the x direction is pointed in the car's heading, 
however this would not guarantee x would be increasing in all cases and requires additional transformations and 
processing.  As another approach the Fernet coordinates can be used to help in the spline generation and create a 
spline for all positions of the map.  Multiple splines were created using the Fernet's S coordinate for the spline's x 
vector.  The splines created were:
* spline_x(s, x)
* spline_y(s, y)
* spline_dx(s, dx)
* spline_dy(s, dy)

Now taking any given road position in Fernet coordinates a corresponding x, y coordinate can be generated using
```c++
x = spline_x(s) + d*spline_dx(s)
y = spline_y(s) + d*spline_dy(s)
``` 
All path generation can then be done in Fernet coordinates.

### Predict
The first step in the update cycle is to analyze the traffic and predict the future positions.  Each car in the 
sensor_fusion vector sent from the simulator is constructed into a vehicle class instance which allows for simple 
prediction of behavior within the `Planner::Traffic` method.  The `Traffic` method constructs a simple view of the 
traffic pattern determining the speed of each lane based on the slowest car ahead as well as determining if the adjacent 
lanes are open to a lane change.

### Plan
The second step in the update cycle is to use the traffic pattern and our car's state to determine the best action from:
* Left lane change
* Stay in lane
* Right lane change

To determine the best choice cost functions are used that determines the cost of each action.  The action with the lowest 
cost is then executed.  The cost functions look at the following:
* Prefer to not be in the fast lane
* Prefer the lane with the most open space ahead
* Prefer the lane with the highest lane speed
* Prefer to move towards the lane that is faster
* Prefer to move towards the lane that has more open space ahead
* Prefer to stay in lane
* Prefer to make lane changes at least 2 seconds apart.

Each of these cost functions are assigned different weights that were determined from empirical tuning.  