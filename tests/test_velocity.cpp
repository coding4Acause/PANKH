#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include "velocity.h"
#include "constants.h"

TEST_CASE("velocity_at_surface_of_the_body_inertial_frame computes correct velocity", "[velocity]") {
    double Qinf = 1.0, x_pitch = 0.5, y_pitch = 0.0, h0 = 0.0, h1 = 0.25, phi_h = 0.0;
    double alpha0 = 0.0, alpha1 = 0.0, phi_alpha = 0.0, t = 0.0, omega = 1.0;
    double x = 0.5, y = 0.0;
    
    Eigen::Vector2d velocity = velocity_at_surface_of_the_body_inertial_frame(Qinf, x_pitch, y_pitch, h0, h1, phi_h, alpha0, alpha1, phi_alpha, t, omega, x, y);
    
    REQUIRE(velocity(0) == Approx(Qinf).epsilon(0.01)); // Freestream velocity
    REQUIRE(velocity(1) == Approx(0.0).epsilon(0.01));
}
