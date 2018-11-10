
# TODO

Status: writing dims check for state vector comparison, should revisit test writing as it may be wrong.

- [x] replaced Robot update functions with separate class RobotEvolver
- [ ] Design random movement feature

## later, nth, as needed, notes

- [ ] Set up Cmake file
- [ ] Sort out slow compiles: https://github.com/catchorg/Catch2/blob/master/docs/slow-compiles.md#top
- [ ] Catch2 benchmarks

# high level elements

- [x] robot class
- robot random movement
- visualize robot location
- landmarks
- observations
- map
- updates

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