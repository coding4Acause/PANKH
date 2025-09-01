#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include "geometry.h"

TEST_CASE("nodal_coordinates_initial produces correct number of nodes", "[geometry]") {
    int n = 10;
    double c = 1.0, q = 0.2, p = 0.04, t_m = 0.12;
    int trailing_edge_type = 0;
    Eigen::VectorXd x0(n), y0(n);
    
    nodal_coordinates_initial(n, c, q, p, trailing_edge_type, t_m, x0, y0);
    
    REQUIRE(x0.size() == n);
    REQUIRE(y0.size() == n);
    REQUIRE(x0(0) == Approx(0.0).epsilon(0.01)); // Leading edge
    REQUIRE(y0(0) == Approx(0.0).epsilon(0.01));
}

TEST_CASE("nodal_coordinates_initial handles invalid inputs", "[geometry]") {
    int n = 0;
    double c = 1.0, q = 0.2, p = 0.04, t_m = 0.12;
    int trailing_edge_type = 0;
    Eigen::VectorXd x0(n), y0(n);
    
    nodal_coordinates_initial(n, c, q, p, trailing_edge_type, t_m, x0, y0);
    
    REQUIRE(x0.size() == 0);
    REQUIRE(y0.size() == 0);
}
