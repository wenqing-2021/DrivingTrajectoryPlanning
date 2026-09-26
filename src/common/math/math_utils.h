/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 * @brief Math-related util functions.
 */

#pragma once

#include <cmath>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

#include "vec2d.h"

#include <Eigen/Core>

/**
 * @namespace apollo::common::math
 * @brief apollo::common::math
 */
namespace common {
namespace math {

double Sqr(const double x);

/**
 * @brief Cross product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1, const Vec2d& end_point_2);

/**
 * @brief Inner product between two 2-D vectors from the common start point,
 *        and end at two other points.
 * @param start_point The common start point of two vectors in 2-D.
 * @param end_point_1 The end point of the first vector.
 * @param end_point_2 The end point of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1, const Vec2d& end_point_2);

/**
 * @brief Cross product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The cross product result.
 */
double CrossProd(const double x0, const double y0, const double x1, const double y1);

/**
 * @brief Inner product between two vectors.
 *        One vector is formed by 1st and 2nd parameters of the function.
 *        The other vector is formed by 3rd and 4th parameters of the function.
 * @param x0 The x coordinate of the first vector.
 * @param y0 The y coordinate of the first vector.
 * @param x1 The x coordinate of the second vector.
 * @param y1 The y coordinate of the second vector.
 *
 * @return The inner product result.
 */
double InnerProd(const double x0, const double y0, const double x1, const double y1);

/**
 * @brief Wrap angle to [0, 2 * PI).
 * @param angle the original value of the angle.
 * @return The wrapped value of the angle.
 */
double WrapAngle(const double angle);

/**
 * @brief Normalize angle to [-PI, PI).
 * @param angle the original value of the angle.
 * @return The normalized value of the angle.
 */
double NormalizeAngle(const double angle);

/**
 * @brief Calculate the difference between angle from and to
 * @param from the start angle
 * @param from the end angle
 * @return The difference between from and to. The range is between [-PI, PI).
 */
double AngleDiff(const double from, const double to);

/**
 * @brief Compute squared value.
 * @param value The target value to get its squared value.
 * @return Squared value of the input value.
 */
template<typename T>
inline T Square(const T value) {
    return value * value;
}

/**
 * @brief Clamp a value between two bounds.
 *        If the value goes beyond the bounds, return one of the bounds,
 *        otherwise, return the original value.
 * @param value The original value to be clamped.
 * @param bound1 One bound to clamp the value.
 * @param bound2 The other bound to clamp the value.
 * @return The clamped value.
 */
template<typename T>
T Clamp(const T value, T bound1, T bound2) {
    if (bound1 > bound2) { std::swap(bound1, bound2); }

    if (value < bound1) {
        return bound1;
    } else if (value > bound2) {
        return bound2;
    }
    return value;
}

// Gaussian
double Gaussian(const double u, const double std, const double x);

inline double Sigmoid(const double x) {
    return 1.0 / (1.0 + std::exp(-x));
}

// Rotate a 2d vector counter-clockwise by theta
Vec2d RotateVector2d(const Vec2d& v_in, const double theta);

inline std::pair<double, double> RFUToFLU(const double x, const double y) {
    return std::make_pair(y, -x);
}

inline std::pair<double, double> FLUToRFU(const double x, const double y) {
    return std::make_pair(-y, x);
}

inline void L2Norm(int feat_dim, float* feat_data) {
    if (feat_dim == 0) { return; }
    // feature normalization
    float l2norm = 0.0f;
    for (int i = 0; i < feat_dim; ++i) { l2norm += feat_data[i] * feat_data[i]; }
    if (l2norm == 0) {
        float val = 1.f / std::sqrt(static_cast<float>(feat_dim));
        for (int i = 0; i < feat_dim; ++i) { feat_data[i] = val; }
    } else {
        l2norm = std::sqrt(l2norm);
        for (int i = 0; i < feat_dim; ++i) { feat_data[i] /= l2norm; }
    }
}

// Cartesian coordinates to Polar coordinates
std::pair<double, double> Cartesian2Polar(double x, double y);

template<class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type almost_equal(T x, T y, int ulp) {
    // the machine epsilon has to be scaled to the magnitude of the values used
    // and multiplied by the desired precision in ULPs (units in the last place)
    // unless the result is subnormal
    return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp ||
           std::fabs(x - y) < std::numeric_limits<T>::min();
}

std::vector<double> ToContinuousAngle(const std::vector<double>& angle);

std::vector<bool> GetPathGears(const std::vector<double>& x, const std::vector<double>& y,
                               const std::vector<double>& theta);

inline std::vector<double> Product3D(const std::vector<double>& v1, const std::vector<double>& v2) {
    return {v1[1] * v2[2] - v2[1] * v1[2], -(v1[0] * v2[2] - v2[0] * v1[2]), v1[0] * v2[1] - v2[0] * v1[1]};
}

inline double GetTriangleAera(const Vec2d& A, const Vec2d& B, const Vec2d& C) {
    double a = (B - C).Length();
    double b = (A - C).Length();
    double c = (A - B).Length();
    double s = (a + b + c) / 2;
    return std::sqrt(s * (s - a) * (s - b) * (s - c));
}


/**
 * @brief Tangent majorant of exp(d) around `at`, valid for d <= 0.
 *
 * On d <= 0 the exponential has curvature at most one, so this quadratic touches
 * exp(d) at `at` and stays above it. Minimizing the majorant and repeating the
 * step converges to the minimizer of the exponential itself, which is how a
 * solver without an exponential cone can still keep an exp(d) objective.
 * @param d The point at which the bound is evaluated.
 * @param at The point where the quadratic touches exp.
 * @return The value of the majorant at d.
 */
inline double ExponentialUpperBound(const double d, const double at) {
  const double difference = d - at;
  return std::exp(at) * (1.0 + difference) + 0.5 * difference * difference;
}

/**
 * @brief Minimize quadratic * h^2 + linear * h + reciprocal / h on [lo, hi].
 *
 * The derivative 2 * quadratic * h + linear - reciprocal / h^2 is strictly
 * increasing for h > 0, so bisection returns the exact minimizer. The objective
 * is separable, which is the shape the OCEAN time subproblem has once the other
 * blocks are fixed.
 * @return The optimal h, or lo/hi when the minimizer lies outside the interval.
 * @throw std::invalid_argument if the interval or a coefficient is not finite,
 *        or if quadratic/reciprocal is negative, or if lo <= 0.
 */
double OptimalTimeStep(const double quadratic, const double linear, const double reciprocal,
                       const double lo, const double hi);

/**
 * @brief Minimize weight * exp(d) + rho / 2 * (d + offset)^2 subject to d <= upper.
 *
 * The derivative is strictly increasing, so a safeguarded Newton iteration
 * converges to the exact minimizer.
 * @param upper The upper bound of d, which the optimum never exceeds.
 * @return The optimal d.
 */
double OptimalDistance(const double offset, const double rho, const double weight, const double upper);

/**
 * @brief Counter-clockwise rotation matrix of `theta`.
 */
inline Eigen::Matrix2d RotationMatrix2d(const double theta) {
  Eigen::Matrix2d rotation;
  rotation << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
  return rotation;
}

}   // namespace math
}   // namespace common
