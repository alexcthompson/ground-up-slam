#include <cmath>
#include "robot.hpp"

int main() {
    Robot r("kat bot", 0, 1, 2, -M_PI / 2, 0.5, M_PI / 10);

    for (int i = 1; i < 50; i++) {
        r.update_state(i);
        r.print_status();
    }

    return 0;
}