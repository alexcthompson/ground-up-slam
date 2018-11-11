#define CATCH_CONFIG_MAIN
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include <stdexcept>
#include "catch2/catch.hpp"
#include "Eigen/Dense"
#include "robot.hpp"
#include "utils.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

TEST_CASE ( "Utils") {

    SECTION ( "Wrap angle tests" ) {
        double epsilon = 0.000000001; // margin for error to do comparisons

        double x = 1.414;
        double y = 10.0;
        double z = M_PI + 1;
        CHECK ( wrap_angle(0.0) == 0.0);
        CHECK ( wrap_angle(x) == x );
        CHECK ( wrap_angle(y) == y - 4 * M_PI);
        CHECK ( wrap_angle(z) == -M_PI + 1);
        CHECK ( wrap_angle(M_PI) == M_PI );
        CHECK ( wrap_angle(M_PI + epsilon) == -M_PI + epsilon );
        CHECK ( wrap_angle(M_PI - epsilon) == M_PI - epsilon );
        CHECK ( wrap_angle(-M_PI) == M_PI );
        CHECK ( wrap_angle(-M_PI + epsilon) == -M_PI + epsilon );
        CHECK ( wrap_angle(-M_PI - epsilon) == M_PI - epsilon );

        // check that multiples by integers are correct
        CHECK ( wrap_angle(x + 6 * M_PI) == Approx(wrap_angle(x)).margin(epsilon) );
        CHECK ( wrap_angle(y - 8 * M_PI) == Approx(wrap_angle(y)).margin(epsilon) );
        CHECK ( wrap_angle(z + 1000 * M_PI) == Approx(wrap_angle(z)).margin(epsilon) );
    }
}

TEST_CASE ( "Basic robot functions:" ) {

    SECTION ( "Variable retrieval" ) {
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);
        RobotEvolver evolver;

        CHECK( r.get_name() == "kat bot" );

        VectorXd expected_state = VectorXd::Zero(5);
        expected_state << 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10;
        VectorXd robot_state = r.get_robot_state();

        // REQUIRE these since C++ throws SIGABRT if matrix dimensions don't match
        REQUIRE( (robot_state.rows() == expected_state.rows() && robot_state.cols() == expected_state.cols()) );
        CHECK( robot_state.isApprox(expected_state) );

        // single variable checks
        CHECK( r.get_t()     == 0.1 );
        CHECK( r.get_x()     == 1.0 );
        CHECK( r.get_y()     == 2.3 );
        CHECK( r.get_theta() == -M_PI / 2 );
        CHECK( r.get_v()     == 0.5 );
        CHECK( r.get_w()     == M_PI / 10 );
        CHECK( evolver.get_step() == 0.05 );
    }

    SECTION ( "Variable setting" ) {
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

        r.set_t(3.14);
        CHECK( r.get_t() == 3.14 );

        // set v
        r.set_v(22.1);
        CHECK( r.get_v() == 22.1 );

        r.set_v(0.5);
        CHECK( r.get_v() == 0.5 );

        // set w
        r.set_w(-0.23 * M_PI);
        CHECK( r.get_w() == -0.23 * M_PI );

        r.set_w( M_PI / 10);
        CHECK( r.get_w() == M_PI / 10 );

        VectorXd expected_state(5);
        expected_state << 1.1, 3.4, -2, 7.7, M_PI / 9;
        r.set_state(expected_state);
        CHECK( r.get_robot_state() == expected_state );

        expected_state.resize(33);
        CHECK_THROWS_WITH(r.set_state(expected_state), "Attempting to set new state with wrong dimensions");
    }

    SECTION ( "Check wrap on initialization" ) {
        double margin = 0.000000001; // margin for error to do comparisons
        Robot q("kat bot 2" , 0.1, 1.0, 2.3, -11.3 * M_PI, 0.5, 4.1 * M_PI);

        CHECK ( wrap_angle(-11.3 * M_PI) == Approx(q.get_theta()).margin(margin) );
        CHECK ( wrap_angle(4.1 * M_PI) == Approx(q.get_w()).margin(margin) );
    }

    SECTION ( "Output functions") {
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

        // setup buffer capture
        std::stringstream buffer;

        // location of old buffer
        std::streambuf* old = std::cout.rdbuf(buffer.rdbuf());

        std::cout << r;
        std::string buffer_output = buffer.str();

        std::string expected_output = "\"kat bot\", 0.100000, 1.000000, 2.300000, -1.570796, 0.500000, 0.314159\n";

        CHECK( buffer_output == expected_output );

        // check that precision and fixed are unset

        buffer.str("");
        std::cout << "1.0 printed as: " << 1.0 << std::endl;
        std::cout << "0.0001 printed as: " << 0.0001 << std::endl;
        buffer_output = buffer.str();

        expected_output = "1.0 printed as: 1\n0.0001 printed as: 0.0001\n";

        CHECK( buffer_output == expected_output );

        std::cout.rdbuf(old);
    }

    SECTION ( "Updating state" ) {
        // note: w is M_PI / 5 here, different from above
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 5);
        RobotEvolver evolver;

        // update at 0.5, 1, 5 and 10 seconds
        VectorXd expected_state = VectorXd::Zero(5);
        VectorXd robot_state = r.get_robot_state();

        evolver.update_state(r, 0.5);
        CHECK( r.get_t() == 0.6 );
        expected_state << 1.038947986819, 2.054092089229, -1.256637061436, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        REQUIRE( (robot_state.rows() == expected_state.rows() && robot_state.cols() == expected_state.cols()) );
        CHECK( robot_state.isApprox(expected_state) );

        evolver.update_state(r, 1);
        expected_state << 1.328030073565, 1.656204731499, -0.628318530718, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        evolver.update_state(r, 5);
        expected_state << 2.263519357354, 2.943795268501, 2.513274122872, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        evolver.update_state(r, 10);
        expected_state << 2.263519357354, 2.943795268501, 2.513274122872, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        // set w to 0, test again
        r.set_w(0.0);

        evolver.update_state(r, 0.5);
        expected_state << 2.061265108760, 3.090741581574, 2.513274122872, 0.500000000000, 0.000000000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        evolver.update_state(r, 1);
        expected_state << 1.656756611573, 3.384634207720, 2.513274122872, 0.500000000000, 0.000000000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        evolver.update_state(r, 5);
        CHECK( r.get_t() == 0.1 + 16.5 + 6.5 );
        expected_state << -0.365785874365, 4.854097338451, 2.513274122872, 0.500000000000, 0.000000000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        // test that if w is small, it uses the `w = 0` motion model
        r.set_w(0.00001);
        evolver.update_state(r, 10);
        expected_state << -4.410870846240, 7.793023599913, 2.513274122872, 0.500000000000, 0.000010000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        // STEP_FORWARD
        Robot r2("kat bot 2", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 5);
        evolver.step_forward(r2);
        robot_state = r2.get_robot_state();
        double robot_t = r2.get_t();

        Robot r3("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 5);
        evolver.update_state(r3, evolver.get_step());

        CHECK( robot_t == r3.get_t() );
        CHECK( robot_state == r3.get_robot_state() );
    }

    // SECTION ( "Random motions" ) {
    //     Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 5);

    //     SECTION ( "Random straight move" ) {

    //         // CHECK ( r.get_m_start() == 0.1 );
    //         // CHECK ( r.get_m_end() > 0.1 ); // statistical test, unlikely but not impossible to be false
    //         // CHECK ( r.get_v() > 1 ); // statistical test, unlikely but not impossible to be false
    //         // CHECK ( r.get_v() < 50 ); // statistical test, unlikely but not impossible to be false

    //     }
    // }
}