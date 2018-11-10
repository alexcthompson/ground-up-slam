#ifndef ROBOT
#define ROBOT

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include "Eigen/Dense"
#include "utils.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Robot {
    private:
        const int robot_dims = 5; // gives the dimensions of the robot state, all other dims are for the map

        std::string name;
        double t;     // last update
        VectorXd state = VectorXd::Constant(5, -666); // full state, including mappoint coordinates

    public:
        Robot(std::string name_, double t_, double x_, double y_, double theta_, double v_, double w_);

        int get_robot_dims();
        std::string get_name();

        VectorXd get_robot_state();
        double get_t();
        double get_x();
        double get_y();
        double get_theta();
        double get_v();
        double get_w();

        void set_t(double new_t);
        void set_state(VectorXd new_state);
        void set_v(double new_v);
        void set_w(double new_w);
};

Robot::Robot(std::string name_, double t_, double x_, double y_, double theta_, double v_, double w_)
{
    name = name_;
    t = t_;
    state << x_,
             y_,
             wrap_angle(theta_),
             v_,
             wrap_angle(w_);
}

int Robot::get_robot_dims() { return robot_dims; }

std::string Robot::get_name() { return name; }

VectorXd Robot::get_robot_state() { return state; }

double Robot::get_t() { return t; }

double Robot::get_x() { return state(0); }

double Robot::get_y() { return state(1); }

double Robot::get_theta() { return state(2); }

double Robot::get_v() { return state(3); }

double Robot::get_w() { return state(4); }

void Robot::set_t(double new_t) { t = new_t; }

void Robot::set_state(VectorXd new_state) {
    if (state.size() != new_state.size()) {
        throw std::runtime_error("Attempting to set new state with wrong dimensions");
    }

    state = new_state;
}

void Robot::set_v(double new_v) { state(3) = new_v; }

void Robot::set_w(double new_w) { state(4) = new_w; }


// Overload << for Robot output
std::ostream& operator<< (std::ostream& stream, Robot& robot) {
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\""
              << robot.get_name() << "\", "
              << robot.get_t() << ", "
              << robot.get_x() << ", "
              << robot.get_y() << ", "
              << robot.get_theta() << ", "
              << robot.get_v() << ", "
              << robot.get_w()
              << std::endl;
    return stream;
}


// CLASS THAT UPDATES ROBOT
class RobotEvolver {
    private:
        static double step; // regular time increment for updates across all robots

    public:
        double get_step();
        void update_state(Robot &robot, double dt);
        void step_forward(Robot &robot);
};

double RobotEvolver::step = 0.05;

double RobotEvolver::get_step() { return step; }

void RobotEvolver::update_state(Robot &robot, double dt)
{
    robot.set_t(robot.get_t() + dt);

    VectorXd state = robot.get_robot_state();

    double theta = state(2);
    double v = state(3);
    double w = state(4);

    if (abs(w) >= 0.001) {
        state(0) += (v / w) * (-sin(theta) + sin(theta + w * dt));
        state(1) += (v / w) * ( cos(theta) - cos(theta + w * dt));
        state(2) += wrap_angle(w * dt);
    }
    else {
        state(0) += v * dt * cos(theta);
        state(1) += v * dt * sin(theta);
    }

    robot.set_state(state);
}

void RobotEvolver::step_forward(Robot &robot) {
    update_state(robot, step);
}

#endif // ROBOT