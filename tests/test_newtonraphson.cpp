#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include "NewtonRaphsonNonLinear.h"
#include "geometry.h"
#include "velocity.h"

TEST_CASE("newton_raphson returns residuals of correct size", "[newtonraphson]") {
    int n = 10;
    double dt = 0.1, t = 0.0, lwp = 0.1, theta_wp = 0.0;
    Eigen::VectorXd freestream(2), vtotal_wp_cp(2), x_pp(n), y_pp(n), x_cp(n-1), y_cp(n-1), l(n-1);
    Eigen::VectorXd B_unsteady(n+1), gamma_unsteady(n+1), gamma_bound(n);
    std::vector<double> gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location;
    Eigen::VectorXd wake_panel_cp(2), wake_panel_normal(2);
    Eigen::MatrixXd A_unsteady(n+1, n+1), unit_normal(n-1, 2), wake_panel_coordinates(2, 2);
    double gamma_old = 0.0;
    
    freestream << 1.0, 0.0;
    x_pp.setZero(); y_pp.setZero(); x_cp.setZero(); y_cp.setZero(); l.setOnes();
    
    Eigen::VectorXd residuals = newton_raphson(n, dt, t, lwp, theta_wp, freestream, vtotal_wp_cp, x_pp, y_pp, x_cp, y_cp, l, B_unsteady, gamma_unsteady, gamma_old, gamma_bound, gamma_wake_strength, gamma_wake_x_location, gamma_wake_y_location, wake_panel_cp, wake_panel_normal, A_unsteady, unit_normal, wake_panel_coordinates);
    
    REQUIRE(residuals.size() == 2);
}
