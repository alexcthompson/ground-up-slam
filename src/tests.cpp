#define CATCH_CONFIG_MAIN
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include "catch2/catch.hpp"
#include "Eigen/Dense"
#include "robot.hpp"
#include "utils.hpp"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

TEST_CASE ( "Utils") {
    Utils u;

    SECTION ( "Wrap angle tests" ) {
        double margin = 0.000000001; // margin for error to do comparisons

        double x = 1.414;
        double y = 10.0;
        double z = M_PI + 1;
        CHECK ( u.wrap_angle(0.0) == 0.0);
        CHECK ( u.wrap_angle(x) == x );
        CHECK ( u.wrap_angle(y) == y - 4 * M_PI);
        CHECK ( u.wrap_angle(z) == -M_PI + 1);
        CHECK ( u.wrap_angle(M_PI) == M_PI );
        CHECK ( u.wrap_angle(M_PI + 0.000000001) == -M_PI + 0.000000001 );
        CHECK ( u.wrap_angle(M_PI - 0.000000001) == M_PI - 0.000000001 );
        CHECK ( u.wrap_angle(-M_PI) == M_PI );
        CHECK ( u.wrap_angle(-M_PI + 0.000000001) == -M_PI + 0.000000001 );
        CHECK ( u.wrap_angle(-M_PI - 0.000000001) == M_PI - 0.000000001 );

        // check that multiples by integers are correct
        CHECK ( u.wrap_angle(x + 6 * M_PI) == Approx(u.wrap_angle(x)).margin(margin) );
        CHECK ( u.wrap_angle(y - 8 * M_PI) == Approx(u.wrap_angle(y)).margin(margin) );
        CHECK ( u.wrap_angle(z + 1000 * M_PI) == Approx(u.wrap_angle(z)).margin(margin) );
    }
}

TEST_CASE ( "Basic robot functions:" ) {

    SECTION ( "Variable retrieval" ) {
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

        CHECK( r.get_name() == "kat bot" );

        VectorXd expected_state = VectorXd::Zero(5);
        expected_state << 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10;
        VectorXd robot_state = r.get_robot_state();

        CHECK( r.get_robot_dims() == robot_state.rows() );

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
    }

    SECTION ( "Check wrap on initialization" ) {
        Utils u;
        double margin = 0.000000001; // margin for error to do comparisons
        Robot q("kat bot 2" , 0.1, 1.0, 2.3, -11.3 * M_PI, 0.5, 4.1 * M_PI);

        CHECK ( u.wrap_angle(-11.3 * M_PI) == Approx(q.get_theta()).margin(margin) );
        CHECK ( u.wrap_angle(4.1 * M_PI) == Approx(q.get_w()).margin(margin) );
    }

    SECTION ( "Output functions") {
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

        // setup buffer capture
        stringstream buffer;

        // location of old buffer
        streambuf* old = cout.rdbuf(buffer.rdbuf());

        // reinitialize and collect buffer output
        r.print_robot_state();

        string buffer_output = buffer.str();
        string expected_output = "\"kat bot\", 0.100000, 1.000000, 2.300000, -1.570796, 0.500000, 0.314159\n";

        CHECK( buffer_output == expected_output );

        buffer.str(""); // clear buffer

        r.print_robot_state(3);

        buffer_output = buffer.str();
        expected_output = "\"kat bot\", 0.100, 1.000, 2.300, -1.571, 0.500, 0.314\n";

        CHECK( buffer_output == expected_output );

        cout.rdbuf(old);
    }

    SECTION ( "Setting v and w" ) {
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

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
    }

    SECTION ( "Updating state" ) {
        // note: w is M_PI / 5 here, different from above
        Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 5);

        // update at 0.5, 1, 5 and 10 seconds
        VectorXd expected_state = VectorXd::Zero(5);
        VectorXd robot_state = r.get_robot_state();

        r.update_state(0.5);
        CHECK( r.get_t() == 0.6 );
        expected_state << 1.038947986819, 2.054092089229, -1.256637061436, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        REQUIRE( (robot_state.rows() == expected_state.rows() && robot_state.cols() == expected_state.cols()) );
        CHECK( robot_state.isApprox(expected_state) );

        r.update_state(1);
        expected_state << 1.328030073565, 1.656204731499, -0.628318530718, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        r.update_state(5);
        expected_state << 2.263519357354, 2.943795268501, 2.513274122872, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        r.update_state(10);
        expected_state << 2.263519357354, 2.943795268501, 2.513274122872, 0.500000000000, 0.628318530718;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        // set w to 0, test again
        r.set_w(0.0);

        r.update_state(0.5);
        expected_state << 2.061265108760, 3.090741581574, 2.513274122872, 0.500000000000, 0.000000000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        r.update_state(1);
        expected_state << 1.656756611573, 3.384634207720, 2.513274122872, 0.500000000000, 0.000000000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        r.update_state(5);
        CHECK( r.get_t() == 0.1 + 16.5 + 6.5 );
        expected_state << -0.365785874365, 4.854097338451, 2.513274122872, 0.500000000000, 0.000000000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );

        // test that if w is small, it uses the `w = 0` motion model
        r.set_w(0.00001);
        r.update_state(10);
        expected_state << -4.410870846240, 7.793023599913, 2.513274122872, 0.500000000000, 0.000010000000;
        robot_state = r.get_robot_state();
        CHECK( robot_state.isApprox(expected_state) );
    }

    SECTION ( "Movement functions") {

    }
}