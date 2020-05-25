// Copyright @2019 Pony AI Inc. All rights reserved.

#include "homework2/icp/icp.h"

#include <Eigen/SVD>
#include <Eigen/Dense>

#include "common/utils/math/math_utils.h"
#include "common/utils/math/transform/transform.h"

Icp::Icp(const PointCloud& src_pc, const PointCloud& target_pc) {
  src_points_ = Eigen::MatrixXd::Zero(3, src_pc.points.size());
  for (int i = 0; i < src_pc.points.size(); ++i) {
    src_points_.col(i) = src_pc.points[i];
  }
  transformed_src_points_ = src_points_;

  target_points_ = Eigen::MatrixXd::Zero(3, target_pc.points.size());
  for (int i = 0; i < target_pc.points.size(); ++i) {
    target_points_.col(i) = target_pc.points[i];
  }
  target_pc_knn_ = std::make_unique<KdTree>(&target_points_);
}

bool Icp::RunIteration() {
  int iter = 0;
  while (iter < max_iteration_) {
    int num_correspondence = FindCorrespondence(max_correspondence_distance_);
    if (num_correspondence == 0) {
      LOG(WARNING) << "Fail to find any correspondences, exit.";
      return false;
    }
    const double transform_delta = EstimatePose();
    if (transform_delta < 0.0) {
      LOG(WARNING) << "ICP fail to converge, exit. transform_delta < 0.0";
      return false;
    }
    if (transform_delta < kDefaultMinTransformDelta) {
      const Eigen::Quaterniond quaternion(rotation_);
      LOG(INFO) << "ICP converges at iter: " << iter
                << ". T: " << translation_.transpose()
                << ", R/P/Y: " << math::RadianToDegree(math::transform::GetRoll(quaternion))
                << "," << math::RadianToDegree(math::transform::GetPitch(quaternion))
                << "," << math::RadianToDegree(math::transform::GetYaw(quaternion));
      return true;
    }
    ++iter;
  }
  const Eigen::Quaterniond quaternion(rotation_);
  LOG(WARNING) << "ICP fail to converge within the required iteration. "
               << ". T: " << translation_.transpose()
               << ", R/P/Y: " << math::RadianToDegree(math::transform::GetRoll(quaternion))
               << "," << math::RadianToDegree(math::transform::GetPitch(quaternion))
               << "," << math::RadianToDegree(math::transform::GetYaw(quaternion));
  return false;
}

/*
 *
 * Implement this function. Try to find the correspondences by using the Kd-Tree.
 *
 */
int Icp::FindCorrespondence(double max_correspondence_distance) {
    correspondences_.clear();
    int n = transformed_src_points_.cols();
    double sqr_mcd = max_correspondence_distance * max_correspondence_distance;
    for (int i = 0; i < n; ++i) {
        std::vector<int> nn;
        std::vector<double> rr;
        int x = target_pc_knn_ -> Search(transformed_src_points_.col(i), 1, &nn, &rr);
        if (x == 1 && rr[0] <= max_correspondence_distance)
            correspondences_.push_back(Correspondence(i, nn[0], rr[0]));
    }
    return correspondences_.size();
}

/*
 *
 * Implement this function. Estimate R|T given correspondences.
 *
 */
double Icp::EstimatePose() {
  const int active_pt_num = correspondences_.size();
  if (active_pt_num < min_num_correspondence_) {
    return -1.0;
  }

  // 1. Construct source/target point matrix from select correspondences;

    const int n = active_pt_num;
    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(3, n), Q = Eigen::MatrixXd::Zero(3, n);
    for (int i = 0; i < n; ++i) {
        P.col(i) = transformed_src_points_.col(correspondences_[i].query_index);
        Q.col(i) = target_points_.col(correspondences_[i].match_index);
    }

  // 2. Find the centroid and demean source/target point matrix;

    Eigen::Vector3d mu_p = Eigen::Vector3d::Zero(), mu_q = Eigen::Vector3d::Zero();
    for (int i = 0; i < n; ++i) {
        mu_p += P.col(i);
        mu_q += Q.col(i);
    }
    mu_p /= n; mu_q /= n;

  // 3. Follow the proof in handout and estimate R|T for this iteration;

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (int i = 0; i < n; ++i) H += (P.col(i) - mu_p) * (Q.col(i) - mu_q).transpose();
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::Matrix3d R = svd.matrixV() * (svd.matrixU().transpose());
    Eigen::Vector3d T = mu_q - R * mu_p;

  // 4. Transform source pointcloud by using estimated R|T

    transformed_src_points_ = R * transformed_src_points_;
    for (int i = 0; i < transformed_src_points_.cols(); ++i)
        transformed_src_points_.col(i) += T;

  // 5. Update accumulated rotation_ and translation_.

    rotation_ = R * rotation_;
    translation_ = R * translation_ + T;

  return std::max((R - Eigen::Matrix3d::Identity()).norm(), T.norm());
}
