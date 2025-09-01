#include <catch2/catch.hpp>
#include <Eigen/Dense>
#include "InfluenceMatrix.h"

TEST_CASE("influence_matrix returns correct size", "[influencematrix]") {
    double x1 = 0.0, y1 = 0.0, x2 = 1.0, y2 = 0.0, x = 0.5, y = 0.1;
    
    Eigen::MatrixXd matrix = influence_matrix(x1, y1, x2, y2, x, y);
    
    REQUIRE(matrix.rows() == 2);
    REQUIRE(matrix.cols() == 2);
}
