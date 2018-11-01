#ifndef ROBOT
#define ROBOT

#include <cmath>
#include <iomanip>
#include <iostream>
#include "Eigen/Dense"
#include "utils.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Robot {
    private:
        int robot_dims = 5; // gives the dimensions of the robot state, all other dims are for the map
        string name;
        double t;     // last update
        VectorXd state = VectorXd::Constant(5, -666); // full state, including mappoint coordinates
        Utils u;

    public:
        Robot(string name_, double t_, double x_, double y_, double theta_, double v_, double w_);
        int get_robot_dims();
        string get_name();
        VectorXd get_robot_state();
        double get_t();
        double get_x();
        double get_y();
        double get_theta();
        double get_v();
        double get_w();
        void print_robot_state();
        void update_state(double new_t);
};

void Robot::print_robot_state()
{
    cout << fixed << setprecision(6);
    cout << "\"" << get_name() << "\", "
         << get_t() << ", "
         << get_x() << ", "
         << get_y() << ", "
         << get_theta() << ", "
         << get_v() << ", "
         << get_w()
         << endl;
}


Robot::Robot(string name_, double t_, double x_, double y_, double theta_, double v_, double w_)
{
    name = name_;
    t = t_;
    state << x_,
             y_,
             u.wrap_angle(theta_),
             v_,
             u.wrap_angle(w_);
}

int Robot::get_robot_dims() {
    return robot_dims;
}

string Robot::get_name() {
    return name;
}

VectorXd Robot::get_robot_state() {
    return state;
}

double Robot::get_t() {
    return t;
}

double Robot::get_x() {
    return state(0);
}

double Robot::get_y() {
    return state(1);
}

double Robot::get_theta() {
    return state(2);
}

double Robot::get_v() {
    return state(3);
}

double Robot::get_w() {
    return state(4);
}

void Robot::update_state(double new_t)
{
    double dt = new_t - t;

    t      = new_t;
    state[0] += dt * state[3] * cos(state[2]);
    state[1] += dt * state[3] * sin(state[2]);
    state[2] += state[4] * dt;
}

#endif // ROBOT