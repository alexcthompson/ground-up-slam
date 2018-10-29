#define CATCH_CONFIG_MAIN
#include <cmath>
#include "catch2/catch.hpp"
#include "robot.hpp"

using namespace std;

TEST_CASE ( "Basic robot functions:" ) {

    Robot r("Renteechi Kat Bot", 0.1, 1.0, 2.3, -M_PI / 2, 0.5, M_PI / 10);

    SECTION ( "Variable retrieval:" ) {
        CHECK( r.get_name() == "Renteechi Kat Bot" );
        CHECK( r.get_t()     == 0.1f);
        CHECK( r.get_x()     == 1.0f);
        CHECK( r.get_y()     == 2.3f);
        CHECK( r.get_theta() ==  float(-M_PI / 2));
        CHECK( r.get_v()     == 0.5f);
        CHECK( r.get_w()     == float(M_PI / 10));
    }

    SECTION ( "Output functions ") {

    }

    SECTION ( "Movement functions") {

    }
}