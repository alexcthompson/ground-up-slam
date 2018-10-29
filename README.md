A small repo to conduct some ground up SLAM experiments, and for me to hone my C++ skills.

# TODO

- [x] write up robot class including
    - [x] ground truth
    - [x] v, w
    - [x] print robot location
    - [x] update position
- [x] write up main loop with iteration ... make robot go in a circle or something
- [x] set up testing
- [ ] write output function tests
    - [ ] redirecting output
- [ ] write movement function tests
- [ ] do angle normalization via utils file
- [ ] add angle normalization to theta at end of update step
- [ ] figure out setw for robot status printing
- [ ] convert Robot to Eigen approach
- [ ] test that when you initialize, it doesn't fail

## later, nth, as needed, notes

- For approximate matches see: https://github.com/catchorg/Catch2/blob/master/docs/assertions.md#natural-expressions, under floating point comparisons
- [ ] Set up Cmake file
- [ ] Sort out slow compiles: https://github.com/catchorg/Catch2/blob/master/docs/slow-compiles.md#top
- [ ] See if Ben can walk through this: https://github.com/catchorg/Catch2/blob/master/examples/231-Cfg-OutputStreams.cpp

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

# Notes