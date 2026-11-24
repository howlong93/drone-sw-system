#pragma once

#include <Eigen/Dense>
#include <libqhullcpp/Qhull.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace mighty::risp {

using Vec3 = Eigen::Vector3d;
using MatX3 = Eigen::Matrix<double, Eigen::Dynamic, 3>;
using RowMajorMatX3 = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

struct Result {
  MatX3 A;
  Eigen::VectorXd b;
  size_t generated_planes{0};
  std::vector<size_t> remaining_obstacles;
};

inline bool isStrictlyInside(const MatX3& A, const Eigen::VectorXd& b, const Vec3& point,
                             double tolerance = 1e-10) {
  return A.rows() == b.size() && A.cols() == 3 && A.allFinite() && b.allFinite() &&
         ((A * point).array() < (b.array() - tolerance)).all();
}

inline bool reduceRedundantPlanes(MatX3& A, Eigen::VectorXd& b, const Vec3& interior,
                                  std::string& failure_reason) {
  if (A.rows() != b.size() || A.cols() != 3 || A.rows() < 4 || !A.allFinite() ||
      !b.allFinite() || !interior.allFinite()) {
    failure_reason = "invalid_constraint_matrix";
    return false;
  }

  const Eigen::VectorXd margins = b - A * interior;
  if (!margins.allFinite() || (margins.array() <= 1e-10).any()) {
    failure_reason = "reduction_seed_not_strictly_inside";
    return false;
  }

  RowMajorMatX3 dual = A.array().colwise() / margins.array();
  try {
    orgQhull::Qhull hull("", 3, static_cast<int>(dual.rows()), dual.data(), "Qt");
    std::vector<int> indices;
    indices.reserve(hull.vertexList().size());
    for (auto vertex = hull.vertexList().begin(); vertex != hull.vertexList().end(); ++vertex) {
      indices.push_back(vertex->point().id());
    }
    if (indices.size() < 4) {
      failure_reason = "redundancy_reduction_degenerate";
      return false;
    }

    MatX3 reduced_A(indices.size(), 3);
    Eigen::VectorXd reduced_b(indices.size());
    for (size_t i = 0; i < indices.size(); ++i) {
      if (indices[i] < 0 || indices[i] >= A.rows()) {
        failure_reason = "redundancy_reduction_bad_index";
        return false;
      }
      reduced_A.row(static_cast<Eigen::Index>(i)) = A.row(indices[i]);
      reduced_b(static_cast<Eigen::Index>(i)) = b(indices[i]);
    }
    A.swap(reduced_A);
    b.swap(reduced_b);
  } catch (const std::exception&) {
    failure_reason = "redundancy_reduction_exception";
    return false;
  }
  return true;
}

inline Vec3 closestPointOnSegment(const Vec3& point, const Vec3& start, const Vec3& end) {
  const Vec3 direction = end - start;
  const double length_squared = direction.squaredNorm();
  if (length_squared <= std::numeric_limits<double>::epsilon()) return start;
  const double projection = (point - start).dot(direction) / length_squared;
  return start + std::clamp(projection, 0.0, 1.0) * direction;
}

template <typename ProjectionFn>
inline bool generate(const std::vector<Vec3>& obstacles, const MatX3& A_bound,
                     const Eigen::VectorXd& b_bound, const Vec3& reduction_seed, double alpha,
                     std::mt19937& rng, ProjectionFn&& project, Result& result,
                     std::string& failure_reason) {
  result = Result{};
  failure_reason.clear();
  if (!(alpha > 0.0 && alpha < 1.0) || !std::isfinite(alpha)) {
    failure_reason = "alpha_out_of_range";
    return false;
  }
  if (A_bound.rows() != b_bound.size() || A_bound.cols() != 3 || A_bound.rows() < 6 ||
      !A_bound.allFinite() || !b_bound.allFinite() || !reduction_seed.allFinite()) {
    failure_reason = "invalid_bounds";
    return false;
  }

  std::vector<Vec3> remaining;
  remaining.reserve(obstacles.size());
  for (const auto& obstacle : obstacles) {
    if (!obstacle.allFinite()) {
      failure_reason = "nonfinite_obstacle";
      return false;
    }
    remaining.push_back(obstacle);
  }

  MatX3 generated(static_cast<Eigen::Index>(obstacles.size()), 3);
  Eigen::VectorXd offsets(static_cast<Eigen::Index>(obstacles.size()));
  size_t plane_count = 0;
  result.remaining_obstacles.push_back(remaining.size());
  const double one_minus_alpha = 1.0 - alpha;

  while (!remaining.empty()) {
    std::uniform_int_distribution<size_t> choose(0, remaining.size() - 1);
    const Vec3 pivot = remaining[choose(rng)];
    const Vec3 projection = project(pivot);
    const Vec3 normal = pivot - projection;
    const double distance_squared = normal.squaredNorm();
    const double coordinate_scale =
        std::max(1.0, std::max(pivot.squaredNorm(), projection.squaredNorm()));
    const double degenerate_threshold =
        64.0 * std::numeric_limits<double>::epsilon() * coordinate_scale;
    if (!(distance_squared > degenerate_threshold) || !std::isfinite(distance_squared)) {
      failure_reason = "seed_intersects_obstacle";
      return false;
    }

    const double offset = normal.dot(projection) + one_minus_alpha * distance_squared;
    generated.row(static_cast<Eigen::Index>(plane_count)) = normal.transpose();
    offsets(static_cast<Eigen::Index>(plane_count)) = offset;
    ++plane_count;

    const size_t before = remaining.size();
    remaining.erase(std::remove_if(remaining.begin(), remaining.end(),
                                   [&](const Vec3& obstacle) {
                                     return normal.dot(obstacle) > offset;
                                   }),
                    remaining.end());
    if (remaining.size() == before) {
      failure_reason = "partition_made_no_progress";
      return false;
    }
    result.remaining_obstacles.push_back(remaining.size());
  }

  result.A.resize(static_cast<Eigen::Index>(plane_count) + A_bound.rows(), 3);
  result.b.resize(static_cast<Eigen::Index>(plane_count) + b_bound.size());
  if (plane_count > 0) {
    result.A.topRows(static_cast<Eigen::Index>(plane_count)) =
        generated.topRows(static_cast<Eigen::Index>(plane_count));
    result.b.head(static_cast<Eigen::Index>(plane_count)) =
        offsets.head(static_cast<Eigen::Index>(plane_count));
  }
  result.A.bottomRows(A_bound.rows()) = A_bound;
  result.b.tail(b_bound.size()) = b_bound;
  result.generated_planes = plane_count;

  if (!reduceRedundantPlanes(result.A, result.b, reduction_seed, failure_reason)) return false;
  return true;
}

inline bool pointSeed(const std::vector<Vec3>& obstacles, const Vec3& seed,
                      const MatX3& A_bound, const Eigen::VectorXd& b_bound, double alpha,
                      std::mt19937& rng, Result& result, std::string& failure_reason) {
  return generate(obstacles, A_bound, b_bound, seed, alpha, rng,
                  [&](const Vec3&) { return seed; }, result, failure_reason);
}

inline bool lineSeed(const std::vector<Vec3>& obstacles, const Vec3& start, const Vec3& end,
                     const MatX3& A_bound, const Eigen::VectorXd& b_bound, double alpha,
                     std::mt19937& rng, Result& result, std::string& failure_reason) {
  if ((end - start).squaredNorm() <= std::numeric_limits<double>::epsilon()) {
    return pointSeed(obstacles, start, A_bound, b_bound, alpha, rng, result, failure_reason);
  }
  const Vec3 midpoint = 0.5 * (start + end);
  return generate(obstacles, A_bound, b_bound, midpoint, alpha, rng,
                  [&](const Vec3& obstacle) {
                    return closestPointOnSegment(obstacle, start, end);
                  },
                  result, failure_reason);
}

}  // namespace mighty::risp

