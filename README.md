# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

[![Path planner demonstration video](https://img.youtube.com/vi/mDG67tWyG4k/0.jpg)](https://www.youtube.com/watch?v=mDG67tWyG4k)

## Simulator
The latest simulator is available from Udacity [here](https://github.com/udacity/self-driving-car-sim/releases).

Here is the data provided from the Simulator to the C++ Program

### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, 
car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet 
coordinates, car's d position in frenet coordinates. 

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

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

## Path Planner

The path planner is implemented in a few stages; smooth, predict, plan, generate.

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

The spline generation is done in the `map.cpp` initializer, lines 27-78 and the conversion from Fernet to Cartesien is
done in `getXY()` on lines 205-213.

### Predict
The first step in the update cycle is to analyze the traffic and predict the future positions, planner.  Each car in the 
sensor_fusion vector sent from the simulator is constructed into a vehicle class instance which allows for simple 
prediction of behavior within the `Planner::Traffic` method, lines 121-169.  The `Traffic` method constructs a 
simple view of the traffic pattern determining the speed of each lane based on the slowest car ahead as well as 
determining if the adjacent lanes are open to a lane change.

### Plan
The second step in the update cycle is to use the traffic pattern and our car's state to determine the best action from:
* Left lane change
* Stay in lane
* Right lane change

Each action is evaluated using a set of cost functions, `Planner::Update` lines 51-110.  The action with the lowest 
cost is executed.  The cost functions look at the following:
* Prefer to not drive on the shoulder or into oncoming traffic.
* Prefer to not be in the fast lane
* Prefer the lane with the most open space ahead
* Prefer the lane with the highest lane speed
* Prefer to move towards the lane that is faster
* Prefer to move towards the lane that has more open space ahead
* Prefer to stay in lane
* Prefer to make lane changes at least 2 seconds apart.

Each of these cost functions are assigned different weights that were determined from empirical testing.

### Path Generation
The target lane determined from the planner is given to the path generation `Planner::UpdatePath()`.  The previously 
generated future path points are first pushed into the next x, y vectors.  Due to descrepencies in the coordinate 
transforms performed by the simulator and `map.cpp` the Fernet coordinates used to generate the x, y vectors are also 
tracked and must be purged of past points.  To enable smooth lane changes a spline, `spline_d` is generated using Frenet s, d.  
The spline generates a path starting at the car's current d position and transitions to the target d position over 30m 
in forward progress, lines 193-209.

To generate a series of s points the speed of the car needs to be determined.  Speed is calculated by a sigmoid function 
that has a maximum speed of either the speed limit if the lane is open ahead or the lane speed if the lane has cars 
ahead.  The sigmoid is calculated based on the distance to the closest car ahead in the same lane.  As the distance 
decreases the sigmoid approaches one half the leading car speed, lines 212-213.

The next points are now generated starting at the end of the previous path.  Speed is calculated based on a percentage 
difference between the target speed and the current speed, line 218.  The next s position is then determined based on 
the speed and the initial s position, line 219.  The next d position is then determined, line 220, from `spline_d`.
x, y coordinates are generated by `getXY()` in `map.cpp` and finally the new values are pushed into their respective 
vectors.