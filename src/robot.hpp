#include <cmath>
#include <iostream>
#include "utils.hpp"

using namespace std;

class Robot {
    private:
        string name;
        float t;     // last update
        float x;     // meters
        float y;     // meters
        float theta; // robot's facing direction
        float v;     // velocity in meters / second
        float w;     // angular velocity in radians
        Utils u;
    public:
        Robot(string name_, float t_, float x_, float y_, float theta_, float v_, float w_);
        string get_name();
        float get_t();
        float get_x();
        float get_y();
        float get_theta();
        float get_v();
        float get_w();
        void print_status();
        void update_state(float new_t);
};

void Robot::print_status()
{
    cout << "Robot " << name << ":  "
         <<   "t = " << t
         << "; x = " << x
         << "; y = " << y
         << "; theta = " << theta
         << "; v = " << v
         << "; w = " << w
         << endl;
}


Robot::Robot(string name_, float t_, float x_, float y_, float theta_, float v_, float w_)
{
    name = name_;
    t = t_;
    x = x_;
    y = y_;
    theta = u.wrap_angle(theta_);
    v = v_;
    w = u.wrap_angle(w_);

    cout << "Robot " << name << " initialized!" << endl;
    print_status();
}

string Robot::get_name() {
    return name;
}

float Robot::get_t() {
    return t;
}

float Robot::get_x() {
    return x;
}

float Robot::get_y() {
    return y;
}

float Robot::get_theta() {
    return theta;
}

float Robot::get_v() {
    return v;
}

float Robot::get_w() {
    return w;
}

void Robot::update_state(float new_t)
{
    float dt = new_t - t;

    t      = new_t;
    x     += dt * v * cos(theta);
    y     += dt * v * sin(theta);
    theta += w * dt;
}