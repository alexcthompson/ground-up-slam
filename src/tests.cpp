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

    Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

    SECTION ( "Variable retrieval" ) {
        CHECK( r.get_name() == "kat bot" );

        VectorXd expected_state = VectorXd::Zero(5);
        expected_state << 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10;
        VectorXd robot_state = r.get_robot_state();

        CHECK( r.get_robot_dims() == robot_state.rows() );

        // REQUIRE these since C++ throws SIGABRT if matrix dimensions don't match
        REQUIRE( robot_state.rows() == expected_state.rows() );
        REQUIRE( robot_state.cols() == expected_state.cols() );
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
        // setup buffer capture
        stringstream buffer;

        // location of old buffer
        streambuf* old = cout.rdbuf(buffer.rdbuf());

        // reinitialize and collect buffer output
        r.print_robot_state();

        string buffer_output = buffer.str();
        string expected_output = "\"kat bot\", 0.100000, 1.000000, 2.300000, -1.570796, 0.500000, 0.314159\n";

        CHECK( buffer_output == expected_output );
    }

    SECTION ( "Movement functions") {

    }
}