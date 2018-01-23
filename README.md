# path-planner
The first project of the Udacity Self-Driving Car Nanodegree term 3.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning` 

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.

---

#### 1.The car is able to drive at least 4.32 miles without incident..
My path planner changes lane only when it make sense and safe. So as long as there is no suicidal driver who changes lane without checking, my planner should be able to drive along the track until out of electricity or gas.
#### 2.The car drives according to the speed limit.
The trajectory planner incorporates the speed limit to generate trajectories so that they do not exceed the speed limit.

#### 3.Max Acceleration and Jerk are not Exceeded.
I used the spline fitting and generated the waypoints from the previous waypoints so that the waypoints won't be far off that could cause the car to make hard acceleration or deceleration.

#### 4.Car does not have collisions.
The path planner decelerates the car if the distance between the leading car is lower than 30 meters.

#### 5.The car stays in its lane, except for the time between changing lanes.
I designed two cost functions to have the car changed lane only when necessary and safe. The first cost function penalizes the car for staying in the slow lane and the second one penalizes the car for trying to change lane without enough gap. So if the current lane is the fatest one, the car will stays in it. 

#### 6.The car is able to change lanes 
See the above point.

## Reflection

#### The code model for generating paths

My path planner is divided into three components:
#### 1. Predictor
Predcits the future longitudinal directions of other cars.  

#### 2. Behavior Planner
Suggests the maneuver for the trajector planner to generate trajectories the ego vehicle should follow. It employs the Finite State Machines which consists of five states:
- Lane Keep
- Prepare Lane Change Left
- Prepare Lane Change Right
- Lane Change Left
- Lane Change Right

I designed two cost functions to guide the ego vehicle to only changes lane when there is enough gap and it can make forward progress.

The output of the behavior planner is the lane number that the ego vehicle should take.

#### 3. Trajectory Planner 
It takes the lane number from the behavior planner and generate a smooth trajectory that does not create excessive jerk that might cause motion sickness. It also keeps the distance between the leading car to avoid collision and prevents speeding.
