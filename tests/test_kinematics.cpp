#include <catch2/catch.hpp>
#include "kinematics.h"
#include "constants.h"

TEST_CASE("alpha_instantaneous computes correct angle", "[kinematics]") {
    double alpha0 = 0.0, alpha1 = 15.0 * DEG2RAD, phi_alpha = 90.0 * DEG2RAD;
    double t = 0.0, omega = 1.0;
    
    double alpha = alpha_instantaneous(alpha0, alpha1, phi_alpha, t, omega);
    
    REQUIRE(alpha == Approx(alpha0).epsilon(0.01)); // At t=0, expect alpha0
}
