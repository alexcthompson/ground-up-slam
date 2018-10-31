#define CATCH_CONFIG_MAIN
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>
#include "catch2/catch.hpp"
#include "robot.hpp"
#include "utils.hpp"

using namespace std;

TEST_CASE ( "Utils") {
    Utils u;

    SECTION ( "Wrap angle tests" ) {
        double margin = 0.000000001; // margin for error to do double and float comparisons

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

        // dupe above for float
        CHECK ( u.wrap_angle(0.0f) == 0.0f);
        CHECK ( u.wrap_angle(float(x)) == float(x) );
        CHECK ( u.wrap_angle(float(y)) == Approx(float(y - 4 * M_PI)).margin(margin) );
        CHECK ( u.wrap_angle(float(z)) == float(-M_PI + 1));
        CHECK ( u.wrap_angle(float(M_PI - 0.0000001)) == Approx(float(M_PI)).margin(margin) );
        CHECK ( u.wrap_angle(float(M_PI + 0.0000001)) == float(-M_PI + 0.0000001) );
        CHECK ( u.wrap_angle(float(M_PI - 0.0000001)) == float(M_PI - 0.0000001) );
        CHECK ( u.wrap_angle(float(-M_PI)) == Approx(float(M_PI)).margin(margin) );
        CHECK ( u.wrap_angle(float(-M_PI + 0.0000001)) == Approx(float(-M_PI + 0.0000001)).margin(margin) );
        CHECK ( u.wrap_angle(float(-M_PI - 0.0000001)) == Approx(float(M_PI - 0.0000001)).margin(margin) );

        // check that multiples by integers are correct
        CHECK ( u.wrap_angle(float(x + 6 * M_PI)) == Approx(u.wrap_angle(float(x))).margin(margin) );
        CHECK ( u.wrap_angle(float(y - 8 * M_PI)) == Approx(u.wrap_angle(float(y))).margin(margin) );
        CHECK ( u.wrap_angle(float(z + 1000 * M_PI)) == Approx(u.wrap_angle(float(z))).margin(0.01) );
    }
}

TEST_CASE ( "Basic robot functions:" ) {

    Robot r("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

    SECTION ( "Variable retrieval" ) {
        CHECK( r.get_name() == "kat bot" );
        CHECK( r.get_t()     == 0.1f);
        CHECK( r.get_x()     == 1.0f);
        CHECK( r.get_y()     == 2.3f);
        CHECK( r.get_theta() == float(-M_PI / 2));
        CHECK( r.get_v()     == 0.5f);
        CHECK( r.get_w()     == float(M_PI / 10));
    }

    SECTION ( "Check wrap on initialization" ) {
        double margin = 0.000000001; // margin for error to do double and float comparisons
        Robot q("kat bot 2" , 0.1, 1.0, 2.3, 0.75 * M_PI, 0.5, 4.1 * M_PI);

        CHECK ( r.get_w() == Approx(q.get_w()).margin(margin) );
    }

    SECTION ( "Output functions") {
        // setup buffer capture
        stringstream buffer;

        // location of old buffer
        streambuf* old = cout.rdbuf(buffer.rdbuf());

        // reinitialize and collect buffer output
        r = Robot("kat bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

        string buffer_output = buffer.str();
        string expected_output = "Robot kat bot initialized!\nRobot kat bot:  t = 0.1; x = 1; y = 2.3; theta = -1.5708; v = 0.5; w = 0.314159\n";

        CHECK( buffer_output == expected_output );
    }

    SECTION ( "Movement functions") {

    }
}