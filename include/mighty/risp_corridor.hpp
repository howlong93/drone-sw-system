#pragma once

#include <Eigen/Dense>
#include <libqhullcpp/Qhull.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <utility>
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
  bool used_unreduced_retry{false};
};

struct PathCoverResult {
  std::vector<Result> corridors;
  std::vector<Vec3> waypoints;
  // Arc-length positions of `waypoints` on the reference polyline. Keeping
  // these positions avoids recovering seam ordering from floating-point
  // geometry when the optimizer path is expanded later.
  std::vector<double> waypoint_distances;
  std::vector<std::pair<Vec3, Vec3>> seed_segments;
  size_t reference_waypoints{0};
  double reference_length{0.0};
  double covered_length{0.0};
  double min_overlap_margin{std::numeric_limits<double>::infinity()};
  double risp_generation_ms{0.0};
  size_t unreduced_retry_count{0};
  bool full_coverage{false};
  bool horizon_exceeded{false};
};

struct ExpandedPathCover {
  std::vector<Vec3> waypoints;
  // One geometric PathCover corridor index for every optimizer segment.
  std::vector<size_t> corridor_indices;
  size_t inserted_seams{0};
};

inline bool isStrictlyInside(const MatX3& A, const Eigen::VectorXd& b, const Vec3& point,
                             double tolerance = 1e-10) {
  return A.rows() == b.size() && A.cols() == 3 && A.allFinite() && b.allFinite() &&
         ((A * point).array() < (b.array() - tolerance)).all();
}

inline double normalizedMinimumSlack(const MatX3& A, const Eigen::VectorXd& b,
                                     const Vec3& point) {
  if (A.rows() != b.size() || A.cols() != 3 || A.rows() == 0 || !A.allFinite() ||
      !b.allFinite() || !point.allFinite()) {
    return -std::numeric_limits<double>::infinity();
  }
  double minimum = std::numeric_limits<double>::infinity();
  for (Eigen::Index row = 0; row < A.rows(); ++row) {
    const double norm = A.row(row).norm();
    if (!(norm > 1e-12) || !std::isfinite(norm))
      return -std::numeric_limits<double>::infinity();
    minimum = std::min(minimum, (b(row) - A.row(row).dot(point)) / norm);
  }
  return minimum;
}

inline bool contains(const MatX3& A, const Eigen::VectorXd& b, const Vec3& point,
                     double tolerance_m = 1e-8) {
  return normalizedMinimumSlack(A, b, point) >= -tolerance_m;
}

inline bool excludesAllObstacles(const MatX3& A, const Eigen::VectorXd& b,
                                 const std::vector<Vec3>& obstacles,
                                 double tolerance_m = 1e-8) {
  return std::none_of(obstacles.begin(), obstacles.end(), [&](const Vec3& obstacle) {
    return contains(A, b, obstacle, tolerance_m);
  });
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

  // Qhull redundancy removal is a numerical optimization, not part of RISP's
  // safety argument. Preserve the complete constraint set and fall back to it
  // whenever reduction fails or accidentally admits an obstacle.
  const MatX3 complete_A = result.A;
  const Eigen::VectorXd complete_b = result.b;
  MatX3 reduced_A = complete_A;
  Eigen::VectorXd reduced_b = complete_b;
  std::string reduction_reason;
  const bool reduced =
      reduceRedundantPlanes(reduced_A, reduced_b, reduction_seed, reduction_reason);
  if (reduced && excludesAllObstacles(reduced_A, reduced_b, obstacles)) {
    result.A.swap(reduced_A);
    result.b.swap(reduced_b);
  } else {
    if (!excludesAllObstacles(complete_A, complete_b, obstacles)) {
      failure_reason = reduced ? "unreduced_obstacle_inside"
                               : "reduction_failed_and_unreduced_obstacle_inside";
      return false;
    }
    result.A = complete_A;
    result.b = complete_b;
    result.used_unreduced_retry = true;
  }
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

inline double polylineLength(const std::vector<Vec3>& path) {
  double length = 0.0;
  for (size_t i = 1; i < path.size(); ++i) length += (path[i] - path[i - 1]).norm();
  return length;
}

inline bool segmentExitParameter(const Result& corridor, const Vec3& start, const Vec3& end,
                                 double& exit_parameter) {
  if (!contains(corridor.A, corridor.b, start) || contains(corridor.A, corridor.b, end))
    return false;
  const Vec3 direction = end - start;
  double upper = 1.0;
  bool found = false;
  for (Eigen::Index row = 0; row < corridor.A.rows(); ++row) {
    const double denominator = corridor.A.row(row).dot(direction);
    if (denominator <= 1e-12) continue;
    const double candidate =
        (corridor.b(row) - corridor.A.row(row).dot(start)) / denominator;
    if (candidate >= -1e-10 && candidate < upper) {
      upper = candidate;
      found = true;
    }
  }
  if (!found || !std::isfinite(upper)) return false;
  exit_parameter = std::clamp(upper, 0.0, 1.0);
  return true;
}

template <typename PathContainer>
inline bool pathCoverLineSeed(const std::vector<Vec3>& obstacles,
                              const PathContainer& reference_path,
                              const MatX3& A_bound, const Eigen::VectorXd& b_bound,
                              double alpha, uint32_t random_seed, size_t max_corridors,
                              double overlap_backtrack_m, double minimum_overlap_margin_m,
                              PathCoverResult& output, std::string& failure_reason) {
  output = PathCoverResult{};
  failure_reason.clear();
  if (max_corridors == 0) {
    failure_reason = "invalid_max_corridors";
    return false;
  }
  if (!(overlap_backtrack_m >= 0.0) || !(minimum_overlap_margin_m >= 0.0) ||
      !std::isfinite(overlap_backtrack_m) || !std::isfinite(minimum_overlap_margin_m)) {
    failure_reason = "invalid_overlap_parameters";
    return false;
  }

  std::vector<Vec3> path;
  path.reserve(reference_path.size());
  for (const auto& point : reference_path) {
    if (!point.allFinite()) {
      failure_reason = "nonfinite_reference_path";
      return false;
    }
    if (path.empty() || (point - path.back()).norm() > 1e-8) path.push_back(point);
  }
  if (path.size() < 2) {
    failure_reason = "path_too_short";
    return false;
  }

  output.reference_waypoints = path.size();
  output.reference_length = polylineLength(path);
  std::vector<double> reference_distances(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i)
    reference_distances[i] = reference_distances[i - 1] + (path[i] - path[i - 1]).norm();
  output.waypoints.push_back(path.front());
  output.waypoint_distances.push_back(0.0);

  auto generate_corridor = [&](size_t corridor_index, const Vec3& start, const Vec3& end,
                               Result& result, std::string& reason) {
    std::seed_seq sequence{random_seed, static_cast<uint32_t>(corridor_index), 0x70617468U};
    std::mt19937 rng(sequence);
    const auto begin = std::chrono::steady_clock::now();
    const bool ok = lineSeed(obstacles, start, end, A_bound, b_bound, alpha, rng, result, reason);
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    output.risp_generation_ms += elapsed_ms;
    return ok;
  };

  Result first;
  if (!generate_corridor(0, path[0], path[1], first, failure_reason)) {
    failure_reason = "corridor_0_" + failure_reason;
    return false;
  }
  output.unreduced_retry_count += first.used_unreduced_retry ? 1U : 0U;
  output.corridors.push_back(first);
  output.seed_segments.emplace_back(path[0], path[1]);

  Vec3 last_inside = path[1];
  size_t path_index = 2;
  while (path_index < path.size()) {
    const Vec3 target = path[path_index];
    const Result& current = output.corridors.back();
    if (contains(current.A, current.b, target)) {
      last_inside = target;
      ++path_index;
      continue;
    }

    if (output.corridors.size() >= max_corridors) {
      output.horizon_exceeded = true;
      output.covered_length = 0.0;
      for (size_t i = 1; i < path_index; ++i)
        output.covered_length += (path[i] - path[i - 1]).norm();
      failure_reason = "max_corridors_exceeded";
      return false;
    }

    double exit_parameter = 0.0;
    if (!segmentExitParameter(current, last_inside, target, exit_parameter)) {
      failure_reason = "corridor_" + std::to_string(output.corridors.size() - 1) +
                       "_exit_not_found";
      return false;
    }
    const Vec3 edge = target - last_inside;
    const double edge_length = edge.norm();
    if (!(edge_length > 1e-8)) {
      failure_reason = "degenerate_reference_edge";
      return false;
    }

    const double backtrack_fraction = std::min(exit_parameter, overlap_backtrack_m / edge_length);
    Vec3 transition = last_inside + (exit_parameter - backtrack_fraction) * edge;
    Result next;
    std::string next_reason;
    const size_t next_index = output.corridors.size();
    if (!generate_corridor(next_index, transition, target, next, next_reason)) {
      failure_reason = "corridor_" + std::to_string(next_index) + "_" + next_reason;
      return false;
    }

    double overlap_margin = std::min(normalizedMinimumSlack(current.A, current.b, transition),
                                     normalizedMinimumSlack(next.A, next.b, transition));
    // If a short edge did not provide enough numerical overlap, retry from the
    // last fully covered HGP waypoint. This is also the conservative candidate
    // proposed for the next design iteration.
    if (overlap_margin < minimum_overlap_margin_m &&
        (transition - last_inside).norm() > 1e-8) {
      Result waypoint_seeded;
      std::string retry_reason;
      if (generate_corridor(next_index, last_inside, target, waypoint_seeded, retry_reason)) {
        const double retry_margin =
            std::min(normalizedMinimumSlack(current.A, current.b, last_inside),
                     normalizedMinimumSlack(waypoint_seeded.A, waypoint_seeded.b, last_inside));
        if (retry_margin > overlap_margin) {
          transition = last_inside;
          next = std::move(waypoint_seeded);
          overlap_margin = retry_margin;
        }
      }
    }
    if (!(overlap_margin >= minimum_overlap_margin_m)) {
      failure_reason = "corridor_" + std::to_string(next_index) + "_degenerate_overlap";
      return false;
    }

    output.min_overlap_margin = std::min(output.min_overlap_margin, overlap_margin);
    output.unreduced_retry_count += next.used_unreduced_retry ? 1U : 0U;
    output.waypoints.push_back(transition);
    output.waypoint_distances.push_back(
        reference_distances[path_index - 1] + (transition - last_inside).norm());
    output.seed_segments.emplace_back(transition, target);
    output.corridors.push_back(std::move(next));
    last_inside = target;
    ++path_index;
  }

  // MIGHTY's local solver requires at least two polynomial segments. If one
  // corridor naturally covers the whole horizon, split it at the reference
  // path's arc-length midpoint and regenerate two line-seeded corridors.
  if (output.corridors.size() == 1) {
    if (max_corridors < 2) {
      output.horizon_exceeded = true;
      output.covered_length = output.reference_length;
      failure_reason = "max_corridors_exceeded";
      return false;
    }
    const double target_length = 0.5 * output.reference_length;
    double accumulated = 0.0;
    Vec3 seam = path.front();
    for (size_t i = 1; i < path.size(); ++i) {
      const double edge_length = (path[i] - path[i - 1]).norm();
      if (accumulated + edge_length >= target_length && edge_length > 1e-12) {
        seam = path[i - 1] +
               ((target_length - accumulated) / edge_length) * (path[i] - path[i - 1]);
        break;
      }
      accumulated += edge_length;
    }
    if ((seam - path.front()).norm() <= 1e-8 || (path.back() - seam).norm() <= 1e-8) {
      failure_reason = "minimum_segment_count_degenerate_split";
      return false;
    }
    Result first_half, second_half;
    std::string first_reason, second_reason;
    if (!generate_corridor(0, path.front(), seam, first_half, first_reason)) {
      failure_reason = "minimum_segment_count_first_" + first_reason;
      return false;
    }
    if (!generate_corridor(1, seam, path.back(), second_half, second_reason)) {
      failure_reason = "minimum_segment_count_second_" + second_reason;
      return false;
    }
    const double split_margin =
        std::min(normalizedMinimumSlack(first_half.A, first_half.b, seam),
                 normalizedMinimumSlack(second_half.A, second_half.b, seam));
    if (!(split_margin >= minimum_overlap_margin_m)) {
      failure_reason = "minimum_segment_count_degenerate_overlap";
      return false;
    }
    output.corridors.clear();
    output.corridors.push_back(std::move(first_half));
    output.corridors.push_back(std::move(second_half));
    output.seed_segments.clear();
    output.seed_segments.emplace_back(path.front(), seam);
    output.seed_segments.emplace_back(seam, path.back());
    output.waypoints.clear();
    output.waypoints.push_back(path.front());
    output.waypoints.push_back(seam);
    output.waypoint_distances.clear();
    output.waypoint_distances.push_back(0.0);
    output.waypoint_distances.push_back(target_length);
    output.min_overlap_margin = split_margin;
    output.unreduced_retry_count =
        (output.corridors[0].used_unreduced_retry ? 1U : 0U) +
        (output.corridors[1].used_unreduced_retry ? 1U : 0U);
  }

  output.waypoints.push_back(path.back());
  output.waypoint_distances.push_back(output.reference_length);
  output.covered_length = output.reference_length;
  output.full_coverage = output.waypoints.size() == output.corridors.size() + 1 &&
                         output.waypoint_distances.size() == output.waypoints.size();
  if (output.corridors.size() == 1)
    output.min_overlap_margin = std::numeric_limits<double>::infinity();
  if (!output.full_coverage) {
    failure_reason = "corridor_waypoint_count_mismatch";
    return false;
  }
  return true;
}

// Preserve every bend of the HGP reference path while inserting PathCover
// transition seams. Each resulting optimizer segment is mapped to the
// geometric corridor that covers its arc-length interval. A corridor may
// therefore constrain several quintic spline pieces.
template <typename PathContainer>
inline bool expandPathCoverForOptimizer(const PathContainer& reference_path,
                                        const PathCoverResult& cover,
                                        ExpandedPathCover& output,
                                        std::string& failure_reason) {
  output = ExpandedPathCover{};
  failure_reason.clear();
  if (cover.corridors.empty() || cover.waypoints.size() != cover.corridors.size() + 1 ||
      cover.waypoint_distances.size() != cover.waypoints.size()) {
    failure_reason = "invalid_pathcover_result";
    return false;
  }

  std::vector<Vec3> path;
  path.reserve(reference_path.size());
  for (const auto& point_raw : reference_path) {
    const Vec3 point = point_raw.template cast<double>();
    if (!point.allFinite()) {
      failure_reason = "nonfinite_reference_path";
      return false;
    }
    if (path.empty() || (point - path.back()).norm() > 1e-8) path.push_back(point);
  }
  if (path.size() < 2) {
    failure_reason = "path_too_short";
    return false;
  }

  std::vector<double> path_distance(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i)
    path_distance[i] = path_distance[i - 1] + (path[i] - path[i - 1]).norm();
  const double total_length = path_distance.back();
  const double distance_tolerance = std::max(1e-8, total_length * 1e-9);
  if (std::abs(cover.waypoint_distances.front()) > distance_tolerance ||
      std::abs(cover.waypoint_distances.back() - total_length) > distance_tolerance) {
    failure_reason = "pathcover_length_mismatch";
    return false;
  }
  for (size_t i = 1; i < cover.waypoint_distances.size(); ++i) {
    if (!(cover.waypoint_distances[i] >
          cover.waypoint_distances[i - 1] + distance_tolerance)) {
      failure_reason = "nonmonotonic_pathcover_seams";
      return false;
    }
  }

  std::vector<double> expanded_distances;
  output.waypoints.reserve(path.size() + cover.corridors.size());
  expanded_distances.reserve(path.size() + cover.corridors.size());
  auto append = [&](const Vec3& point, double distance, bool seam) {
    if (!output.waypoints.empty() &&
        (point - output.waypoints.back()).norm() <= 1e-8) {
      return;
    }
    output.waypoints.push_back(point);
    expanded_distances.push_back(distance);
    if (seam) ++output.inserted_seams;
  };

  append(path.front(), 0.0, false);
  size_t seam_index = 1;
  for (size_t path_index = 1; path_index < path.size(); ++path_index) {
    while (seam_index + 1 < cover.waypoint_distances.size() &&
           cover.waypoint_distances[seam_index] <
               path_distance[path_index] - distance_tolerance) {
      if (cover.waypoint_distances[seam_index] <
          path_distance[path_index - 1] - distance_tolerance) {
        failure_reason = "pathcover_seam_out_of_order";
        return false;
      }
      append(cover.waypoints[seam_index], cover.waypoint_distances[seam_index], true);
      ++seam_index;
    }
    if (seam_index + 1 < cover.waypoint_distances.size() &&
        std::abs(cover.waypoint_distances[seam_index] - path_distance[path_index]) <=
            distance_tolerance) {
      append(cover.waypoints[seam_index], cover.waypoint_distances[seam_index], true);
      ++seam_index;
    } else {
      append(path[path_index], path_distance[path_index], false);
    }
  }
  if (seam_index + 1 != cover.waypoint_distances.size() || output.waypoints.size() < 3) {
    failure_reason = "pathcover_seam_merge_failed";
    return false;
  }

  output.corridor_indices.reserve(output.waypoints.size() - 1);
  for (size_t segment = 0; segment + 1 < output.waypoints.size(); ++segment) {
    const double midpoint = 0.5 * (expanded_distances[segment] +
                                   expanded_distances[segment + 1]);
    const auto first_seam = cover.waypoint_distances.begin() + 1;
    const auto last_seam = cover.waypoint_distances.end() - 1;
    const size_t corridor = static_cast<size_t>(
        std::upper_bound(first_seam, last_seam, midpoint) - first_seam);
    if (corridor >= cover.corridors.size() ||
        !contains(cover.corridors[corridor].A, cover.corridors[corridor].b,
                  output.waypoints[segment]) ||
        !contains(cover.corridors[corridor].A, cover.corridors[corridor].b,
                  output.waypoints[segment + 1])) {
      failure_reason = "expanded_segment_outside_corridor";
      return false;
    }
    output.corridor_indices.push_back(corridor);
  }
  return output.corridor_indices.size() + 1 == output.waypoints.size();
}

}  // namespace mighty::risp
