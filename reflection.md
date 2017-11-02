
# Reflection

## Path Planning

The goals / steps of this project are the following:

* Your code must compile without errors with cmake and make.
* The car is able to drive at least 4.32 miles without incident.
* The car drives according to the speed limit.
* Max Acceleration and Jerk are not Exceeded.
* Car does not have collisions.
* The car stays in its lane, except for the time between changing lanes.
* The car is able to change lanes
* There is a reflection on how to generate paths

[//]: # (Image References)
[image1]: ./img/PathPlanning.png

## Rubric Points
Here I will consider the reflection point of the [rubric](https://review.udacity.com/#!/rubrics/1020/view) individually and describe how I addressed each point in my implementation.

---

### 1. There is a reflection on how to generate paths

In order to plan a valid vehicle path, there is a setup of several components needed. Unfortunately the project code starts out with just a basic `main()` stub, so the implemented components are just a simple basic approach.

#### Behavior Planning Component

The responsibility of the behavior planning component is to cycle in between discrete behavior patterns, each of which can generate multiple possible trajectories for the vehicle. The component plans over a larger time frame than the other components, so it relies heavily on precise prediction of outside conditions.

In the project, I've implemented the behavior planning component in the class `VehiclePathPlanner`. I have implemented the following behaviors in order to suffice the rubric:

| Behavior          | Description  |
|-------------------|--------------|
| FollowingLane     | Follows current lane, tries to accelerate up to speed limit. |
| FollowingCar      | Follows current lane, tries to keeps buffer distance to car ahead. Checks for side lanes for possible lane change. |
| ChangingLaneLeft  | Changes to left lane, accelerating based on possible speed in new lane. |
| ChangingLaneRight | Changes to right lane, accelerating based on possible speed in new lane.|

The implementation of the planner allows all transitions in between behaviors at any point of time.

#### Trajectory Generator

The behaviors of the planner may generate various trajectories. It has to be ensured that the finally selected trajectory is optimal regarding certain the criteria:
* Feasibility (avoid collisions, off-road)
* Safety (buffer distance, speed)
* Legality (speed limit)
* Comfort (stays on center lane, low jerk)
* Efficiency ( chose fastest lane)

In the project, I've implemented the trajectory generation in the class `VehiclePathPlanner` as well. Optimally, it would have been implemented using a *Quintic Polynomial* allowing for explicit specification of target position, velocity and acceleration, but I rather took the easy route using the generator from the walkthrough. I also only generate one trajectory for each possible behavior.

###### Feasibility

In order to achieve the Feasibility criteria, I've implemented a simpled down version of the *Separating Axis Theorem* in the `Vehicle` class. It checks whether bounding boxes of future vehicle trajectory states will collide with obstacles and accordingly increases the cost for that trajectory.

###### Safety

The Safety criteria is implemented by linearly decreasing the trajectory velocity as the buffer distance to any leading vehicle is approached.

###### Legality

The Legality criteria is simply implemented by never increasing target speed over 50 mph.

###### Comfort

 For the Comfort criteria, I entirely rely on the generation of smooth splines along the way point sections/lane centers based on the code of the [walkthrough video](https://youtu.be/7sI3VHFPP0w). As in the video, I also keep the planned velocity constant over the entire trajectory. This only works sufficiently, because the planner is invoked fairly frequent so that the new parts which are stitched onto the last trajectory are sufficiently small.

###### Efficiency

In order to achieve the Efficiency criteria, I'm adding the predicted difference to the maximum possible speed to the cost for each trajectory.


#### Prediction Component

The prediction component is needed in order to avoid bumping into other cars during maneuvers. It keeps track of all the surrounding obstacles and predicts their movements into the future.

This could have been implemented very sophisticated (e.g. using a hybrid approach), but instead I just chose a very simplistic unimodal model-based approach with constant velocity and yaw. My implementation also only features a one-dimensional uncertainty in car motion direction, where multi dimensional gaussian would have been needed. The prediction is implemented in the `Vehicle` class.


#### Bringing it all together

Regardless of the excessively large scope of the project, and despite the only very basic implementation, it has been fun playing around with the simulator.

Here's a video of some successful lane changing maneuvers:

[Vehicle driving one save lap (PathPlanning.ogv)](https://github.com/WarrantyVoid/CarND-P11-Path-Planning/blob/master/img/PathPlanning.ogv)

![Screenshot of video][image1]