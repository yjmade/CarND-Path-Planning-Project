# CarND-Path-Planning-Project

## Intro

At the heart of a self-driving car is the ability to plan a path. The car has to make decisions on its own: It has
to decide whether to change lanes, speed up or slow down and many other things.

In this scenario the car drives on a highway. It should drive 50 MPH most of the time, unless prevented by traffic, and
should change lanes when a better lane is available.

## Let's get started

To get started, I applied the suggestions of the walkthrough video of the path planning project.
By adding the [spline library](http://kluge.in-chemnitz.de/opensource/spline/) I could already generate trajectories, and I grasped a concept of how the simulator
and the algorithm interact.

## Trajectory generation

To generate the trajectory I chose this approach:

1. Get the latest planned orientation, speed and position of the car in Frenet `(s, d)` coordinates
2. Generate two next waypoints with the corresponding `d` for the desired lane with 15 meter intervals
3. Convert all of these target waypoints to absolute coordinates `(x, y)`
4. Transform these `(x, y)` points to be relative to the cars location and orientation
5. Generate a spline using these waypoints and take enough `(x', y')` coordinates to fill up the waypoint list -
  using distances appropriate for keeping a desired speed to affect for the `20ms` time gap between waypoints
6. Transform these relative `(x', y')` coordinates back to the absolute `(x, y)` coordinates

To prevent my trajectories from having acceleration or jerk above the allowed limits, I chose a more naive approach: Namely
keeping the changes in `speed` and `d` below certain thresholds, and always using the latest planned position and speed
of the car to make sure the changes were small enough.

This algorithm can be found in [`next_path()`](src/path_planner.cpp#L73-L179).

## Collision avoidance

In a first attempt I decided to always speed up when no other vehicle would be closer than 30m in front, and slow down
continuously while staying in that corridor. This had two downsides:

1. The car would speed up and slow down again all the time and never reach a stable position behind the next car
2. When a car would drive very slowly the own car would nearly come to a stop.

To solve this problem I have following approach:

1. [`next_state()`](src/path_planner.cpp#L53-L67) finds a desired speed based on the distance to and the speed of the next vehicle
2. [`next_path()`](src/path_planner.cpp#L74-L80) gets a velocity change value based on the wanted speed and the actual speed.

## Sensor fusion

All sensor fusion data was stored in a `vector<vector<double>>` make it hard to access the data and have to calculate the future s of each car again and again, so I created the classes
[SensorFusion](src/sensor_fusion.cpp) and [Car](src/car.h). That way I could access the parameters more safely
and have each useful propery calculate and be cached for further use in behavior planning, like `speed`, `lane` and `end_s` which stand for prediceted s. 

## Behavior planning

For planning the behavior I used two things: A state machine and a cost function. The code can be found in
[path_planner.cpp](src/path_planner.cpp).

For this project, I decided to use three states:

1. Keep the current lane
2. Change to the left lane
3. Change to the right lane

The [cost function](src/costs.h#L15-L102) I wrote used 5 cost functions:

1. [cost_efficiency](src/costs.cpp#L15): penalizing trajectories with a low
minimal speed in front of the vehicle
1. [cost_comfort](src/costs.cpp#L20): Keeping the current lane in general is more comfortable than changing lanes.
2. [cost_safety](src/costs.cpp#L25): A lane where the next vehicle is further away should be preferred.
3. [cost_legality](src/costs.cpp#L35): make the car stay on the road
3. [cost_feasibility](src/costs.cpp#L42): State changes that lead to collisions should be avoided.

In a method called [next_state](src/path_planner.cpp#L8-L69) I then call the cost function for each possible trajectory
and choose the state that corresponds to the lowest cost. This is done every 1 seconds.


## Discussion

For further improve this project, what I can do:

- Instead only 3 state currently using, I can make a full FSM by add `prepare_lane_shift_left` and `prepare_lane_shift_right` state, which can make lane shift much quicker and safer
- The prediction of the cars is assume they are in fixed speed without lane shift, which is not true all the time in real world, should consider the acceleration or even jerk.
- now the speed is not controled by cost function, later can creating multiple jerk minimal trajectories(JMTs) for each state. This would allow me to speed up / slow down purely based on cost functions.
