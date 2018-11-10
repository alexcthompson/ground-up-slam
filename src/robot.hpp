#ifndef ROBOT
#define ROBOT

#include <cmath>
#include <iomanip>
#include <iostream>
#include "Eigen/Dense"
#include "utils.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

class Robot {
    private:
        static double step; // regular time increment for updates across all robots
        const int robot_dims = 5; // gives the dimensions of the robot state, all other dims are for the map

        std::string name;
        double t;     // last update
        VectorXd state = VectorXd::Constant(5, -666); // full state, including mappoint coordinates

        // random factors for random motion generation
        bool random_motion;
        double m_start;
        double m_end;

        double mu_v_strt = log(13.41); // log(30 mph)
        double sig_v_strt = log(2) / 2; // 2 std deviations = doubling or halving speed
        double mu_dur_strt = log(5); // seconds
        double sig_dur_strt = log(2); // 1 std deviation = doubling or halving duration

    public:
        Robot(std::string name_, double t_, double x_, double y_, double theta_, double v_, double w_,
              bool random_motion_);

        int get_robot_dims();
        std::string get_name();

        VectorXd get_robot_state();
        double get_t();
        double get_step();
        double get_x();
        double get_y();
        double get_theta();
        double get_v();
        double get_w();
        double get_random_motion();
        double get_m_start();
        double get_m_end();
        double get_mu_v_strt();
        double get_sig_v_strt();
        double get_mu_dur_strt();
        double get_sig_dur_strt();

        void set_v(double new_v);
        void set_w(double new_w);
        void update_state(double new_t);
        void step_forward();

        void random_strt();
};

double Robot::step = 0.05;

Robot::Robot(std::string name_, double t_, double x_, double y_, double theta_, double v_, double w_,
             bool random_motion_ = false)
{
    name = name_;
    t = t_;
    state << x_,
             y_,
             wrap_angle(theta_),
             v_,
             wrap_angle(w_);

    random_motion = random_motion_;
    m_start = t;
    m_end = t;
}

int Robot::get_robot_dims() { return robot_dims; }

std::string Robot::get_name() { return name; }

VectorXd Robot::get_robot_state() { return state; }

double Robot::get_t() { return t; }

double Robot::get_step() { return step; }

double Robot::get_x() { return state(0); }

double Robot::get_y() { return state(1); }

double Robot::get_theta() { return state(2); }

double Robot::get_v() { return state(3); }

double Robot::get_w() { return state(4); }

double Robot::get_random_motion() { return random_motion; }

double Robot::get_m_start() { return m_start; }

double Robot::get_m_end() { return m_end; }

double Robot::get_mu_v_strt() { return mu_v_strt; }

double Robot::get_sig_v_strt() { return sig_v_strt; }

double Robot::get_mu_dur_strt() { return mu_dur_strt; }

double Robot::get_sig_dur_strt() { return sig_dur_strt; }

void Robot::set_v(double new_v) {
    state(3) = new_v;
}

void Robot::set_w(double new_w) {
    state(4) = new_w;
}

void Robot::update_state(double dt)
{
    t += dt;
    double theta = state(2);
    double v = state(3);
    double w = state(4);

    if (abs(get_w()) >= 0.001) {
        state(0) += (v / w) * (-sin(theta) + sin(theta + w * dt));
        state(1) += (v / w) * ( cos(theta) - cos(theta + w * dt));
        state(2) += wrap_angle(w * dt);
    }
    else {
        state(0) += v * dt * cos(theta);
        state(1) += v * dt * sin(theta);
    }
}

void Robot::step_forward() {
    update_state(step);
}

void Robot::random_strt() {
    // generate random start and end times

    // generate random velocity

}

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

#endif // ROBOT