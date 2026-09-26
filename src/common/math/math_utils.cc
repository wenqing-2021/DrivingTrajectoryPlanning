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

#include "math_utils.h"

#define  _USE_MATH_DEFINES
#include <algorithm>
#include <cmath>
#include <math.h>
#include <stdexcept>
#include <utility>

namespace common {
namespace math {

double Sqr(const double x) { return x * x; }

double CrossProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).CrossProd(end_point_2 - start_point);
}

double InnerProd(const Vec2d& start_point, const Vec2d& end_point_1,
                 const Vec2d& end_point_2) {
  return (end_point_1 - start_point).InnerProd(end_point_2 - start_point);
}

double CrossProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * y1 - x1 * y0;
}

double InnerProd(const double x0, const double y0, const double x1,
                 const double y1) {
  return x0 * x1 + y0 * y1;
}

double WrapAngle(const double angle) {
  const double new_angle = std::fmod(angle, M_PI * 2.0);
  return new_angle < 0 ? new_angle + M_PI * 2.0 : new_angle;
}

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double AngleDiff(const double from, const double to) {
  return NormalizeAngle(to - from);
}

// Gaussian
double Gaussian(const double u, const double std, const double x) {
  return (1.0 / std::sqrt(2 * M_PI * std * std)) *
         std::exp(-(x - u) * (x - u) / (2 * std * std));
}

Vec2d RotateVector2d(const Vec2d& v_in,
                               const double theta) {
  const double cos_theta = std::cos(theta);
  const double sin_theta = std::sin(theta);

  auto x = cos_theta * v_in.x() - sin_theta * v_in.y();
  auto y = sin_theta * v_in.x() + cos_theta * v_in.y();

  return {x, y};
}

std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

std::vector<double> ToContinuousAngle(const std::vector<double> &angle) {
  std::vector<double> ret;
  ret = angle;
  for (size_t i = 1; i < ret.size(); i++) {
    while(ret[i] - ret[i-1] > M_PI + 0.001)
      ret[i] = ret[i] - 2 * M_PI;

    while(ret[i] - ret[i-1] < -M_PI- 0.001)
      ret[i] = ret[i] + 2 * M_PI;
  }
  return ret;
}

std::vector<bool> GetPathGears(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &theta) {
  std::vector<bool> gears(x.size(), false);

  for(size_t i = 0; i < x.size()-1; i++) {
    auto diff = Vec2d(x[i+1] - x[i], y[i+1] - y[i]);
    gears[i] = std::abs(NormalizeAngle(diff.Angle() - theta[i])) < M_PI_2;
  }

  gears.back() = gears[gears.size()-2];
  return gears;
}



double OptimalTimeStep(const double quadratic, const double linear, const double reciprocal,
                       const double lo, const double hi) {
  if (!std::isfinite(quadratic) || !std::isfinite(linear) || !std::isfinite(reciprocal) ||
      !std::isfinite(lo) || !std::isfinite(hi) || quadratic < 0.0 || reciprocal < 0.0 || lo <= 0.0 ||
      hi < lo) {
    throw std::invalid_argument("Invalid separable time objective");
  }
  const auto derivative = [&](double h) { return 2.0 * quadratic * h + linear - reciprocal / (h * h); };
  if (derivative(lo) >= 0.0) {
    return lo;
  }
  if (derivative(hi) <= 0.0) {
    return hi;
  }
  double lower = lo;
  double upper = hi;
  for (int i = 0; i < 60; ++i) {
    const double mid = 0.5 * (lower + upper);
    if (derivative(mid) > 0.0) {
      upper = mid;
    } else {
      lower = mid;
    }
  }
  return 0.5 * (lower + upper);
}

double OptimalDistance(const double offset, const double rho, const double weight, const double upper) {
  const auto gradient = [&](double d) { return rho * (d + offset) + weight * std::exp(d); };
  if (gradient(upper) <= 0.0) {
    return upper;
  }
  double lo = std::min(upper, -offset - weight / rho - 1.0);
  double hi = upper;
  double d = Clamp(-offset, lo, hi);
  for (int i = 0; i < 40; ++i) {
    const double grad = gradient(d);
    if (std::abs(grad) <= 1e-12 * (1.0 + rho)) {
      break;
    }
    if (grad > 0.0) {
      hi = d;
    } else {
      lo = d;
    }
    const double next = d - grad / (rho + weight * std::exp(d));
    d = (next > lo && next < hi) ? next : 0.5 * (lo + hi);
  }
  return d;
}

}  // namespace math
}  // namespace common
