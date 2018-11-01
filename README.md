A small repo to conduct some ground up SLAM experiments, and for me to hone my C++ skills.

# TODO

Status: writing dims check for state vector comparison, should revisit test writing as it may be wrong.

- [ ] write movement function tests

## later, nth, as needed, notes

- [ ] Set up Cmake file
- [ ] Sort out slow compiles: https://github.com/catchorg/Catch2/blob/master/docs/slow-compiles.md#top

# Design

## elements

- class: robot object, has
    + ground truth: x, y, theta
    + sigma_x, sigma_y, sigma_theta
    + controls: v, w
    + control noise
    + vars for random motion algo
        * time since new maneuver
        * expectations for maneuver
    * vars: belief for position:
    * vars: belief for controls:
    + function: random motion change
    + function: update position
    + function: update controls
+ write out data each tick
+ visualization
    * in C++ or Python?

## random motion design

- random interval
- at each random point either:
    - continue straight
    - change velocity
    - turn left
    - turn right
- log normal distributions for each
- NTH: program distributions to bring the robot closer to a given point... a kind of gravity ... or a correction that brings them closer

# Notes for a post

- chose to make everything double for simplicity sake