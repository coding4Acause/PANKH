/*
MIT License

Copyright (c) 2025 coding4Acause

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "VectorOperations.h"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
int checks = 0;
int failures = 0;
const double tolerance = 1e-12;

void expect_close(const char* name, double actual, double expected)
{
    ++checks;
    if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
        std::cerr << "FAIL " << name << ": expected " << expected
                  << ", got " << actual << '\n';
        ++failures;
    }
}

void expect_vector(const char* name, const Eigen::VectorXd& actual,
                   const Eigen::VectorXd& expected)
{
    ++checks;
    if (actual.size() != expected.size() || !actual.allFinite() ||
        (actual - expected).cwiseAbs().maxCoeff() > tolerance) {
        std::cerr << "FAIL " << name << '\n';
        ++failures;
    }
}
}

int main()
{
    Eigen::VectorXd a(2), b(2), zero(2), expected(2);
    a << 3.0, 4.0;
    b << -4.0, 3.0;
    zero << 0.0, 0.0;
    expect_close("magnitude 3-4-5", magnitude(a), 5.0);
    expect_close("magnitude zero", magnitude(zero), 0.0);
    expect_close("magnitude signed", magnitude(b), 5.0);
    a << 0.3, -0.4;
    expect_close("magnitude fractional", magnitude(a), 0.5);

    a << 3.0, 4.0;
    expected << 0.6, 0.8;
    expect_vector("normalize 3-4-5", normalize_2d(a), expected);
    b << -3.0, -4.0;
    expected << -0.6, -0.8;
    expect_vector("normalize signed", normalize_2d(b), expected);
    b << 0.0, -2.0;
    expected << 0.0, -1.0;
    expect_vector("normalize axis", normalize_2d(b), expected);
    a << 0.6, -0.8;
    expect_vector("normalize unit vector", normalize_2d(a), a);

    a << 3.0, 4.0;
    b << -4.0, 3.0;
    expect_close("dot perpendicular", dot(a, b), 0.0);
    expect_close("dot self", dot(a, a), 25.0);
    b << -3.0, -4.0;
    expect_close("dot opposite", dot(a, b), -25.0);
    expect_close("dot zero", dot(a, zero), 0.0);
    a << 0.5, -0.25;
    b << -2.0, 4.0;
    expect_close("dot fractional", dot(a, b), -2.0);

    Eigen::VectorXd x(3), y(3), z(3), null3(3);
    x << 1.0, 0.0, 0.0;
    y << 0.0, 1.0, 0.0;
    z << 0.0, 0.0, 1.0;
    null3.setZero();
    expect_vector("cross basis", cross(x, y), z);
    expect_vector("cross reversed", cross(y, x), -z);
    expect_vector("cross parallel", cross(x, x), null3);
    expect_vector("cross zero", cross(x, null3), null3);
    x << 2.0, -3.0, 4.0;
    y << -1.0, 5.0, 2.0;
    z << -26.0, -8.0, 7.0;
    expect_vector("cross mixed components", cross(x, y), z);

    std::cout << checks << " checks, " << failures << " failures\n";
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
