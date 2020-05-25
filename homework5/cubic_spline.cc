// Copyright @2020 Pony AI Inc. All rights reserved.

#include "homework5/cubic_spline.h"
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include <Eigen/LU>

void CubicSpline::SetPoints(const std::vector<double>& xs, const std::vector<double>& ys) {
  int n = xs.size();
  for (int i = 0; i < n; ++i)
    points.push_back(std::make_pair(xs[i], ys[i]));
  std::sort(points.begin(), points.end());
}

void CubicSpline::OutputPoints() {
  for (auto p : points)
    std::cout << p.first << " " << p.second << std::endl;
}

void CubicSpline::Interpolate() {
  int n = points.size() - 1;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * n, 3 * n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(3 * n);
  for (int i = 0; i < n; ++i) {
    double d = points[i + 1].first - points[i].first;
    A(i, 3 * i) = d * d * d;
    A(i, 3 * i + 1) = d * d;
    A(i, 3 * i + 2) = d;
    b(i) = points[i + 1].second - points[i].second;
  }
  for (int i = 0; i < n - 1; ++i) {
    double d = points[i + 1].first - points[i].first;
    A(i + n, 3 * i) = 3 * d * d;
    A(i + n, 3 * i + 1) = 2 * d;
    A(i + n, 3 * i + 2) = 1;
    A(i + n, 3 * i + 5) = -1;
    A(i + 2 * n - 1, 3 * i) = 3 * d;
    A(i + 2 * n - 1, 3 * i + 1) = 1;
    A(i + 2 * n - 1, 3 * i + 4) = -1;
  }
  A(3 * n - 2, 0) = 1;
  A(3 * n - 2, 3) = -1;
  A(3 * n - 1, 3 * n - 6) = 1;
  A(3 * n - 1, 3 * n - 3) = -1;
  Eigen::VectorXd x = A.inverse() * b;
  for (int i = 0; i < n; ++i)
    polys.push_back((CubicPoly){x(3 * i), x(3 * i + 1), x(3 * i + 2)});
}

double CubicSpline::GetInterpolatedY(double x) const {
  int i = lower_bound(points.begin(), points.end(), std::make_pair(x, 999999999.0)) - points.begin() - 1;
  if (i < 0) i = 0;
  if (i > points.size() - 2) i = points.size() - 2;
  double d = x - points[i].first;
  return polys[i].a * d * d * d + polys[i].b * d * d + polys[i].c * d + points[i].second;
}
