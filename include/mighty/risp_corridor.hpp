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
  size_t point_retry_count{0};
  size_t point_no_progress_count{0};
  size_t point_waypoint_retry_count{0};
  std::vector<double> corridor_progress_lengths;
  bool full_coverage{false};
  bool horizon_exceeded{false};
};

struct ExpandedPathCover {
  std::vector<Vec3> waypoints;
  // One geometric PathCover corridor index for every optimizer segment.
  std::vector<size_t> corridor_indices;
  size_t inserted_seams{0};
};

struct PathCoverReuseResult {
  std::vector<Result> corridors;
  // One corridor index for every original HGP segment. No waypoint is added.
  std::vector<size_t> segment_corridor_indices;
  std::vector<std::pair<Vec3, Vec3>> seed_segments;
  std::vector<size_t> segments_per_corridor;
  size_t reference_waypoints{0};
  double reference_length{0.0};
  double risp_generation_ms{0.0};
  size_t unreduced_retry_count{0};
  size_t reused_segment_count{0};
  size_t low_overlap_transition_count{0};
  size_t overlap_rollback_count{0};
  size_t weak_overlap_transition_count{0};
  double min_transition_margin{std::numeric_limits<double>::infinity()};
  bool full_coverage{false};
  bool horizon_exceeded{false};
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
                                 double tolerance_m = 1e-8,
                                 double obstacle_support_radius_m = 0.0) {
  return std::none_of(obstacles.begin(), obstacles.end(), [&](const Vec3& obstacle) {
    if (!(obstacle_support_radius_m > 0.0)) return contains(A, b, obstacle, tolerance_m);
    // The corridor intersects the conservative spherical support of this
    // voxel unless every separating plane still leaves some part of the
    // support on the corridor side.  A voxel is excluded only when at least
    // one plane places the complete support outside.
    for (Eigen::Index row = 0; row < A.rows(); ++row) {
      const double normal_norm = A.row(row).norm();
      if (!(normal_norm > 1e-12) || !std::isfinite(normal_norm)) return true;
      if (A.row(row).dot(obstacle) - obstacle_support_radius_m * normal_norm >
          b(row) + tolerance_m) {
        return false;
      }
    }
    return true;
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
                     std::string& failure_reason, double obstacle_support_radius_m = 0.0) {
  result = Result{};
  failure_reason.clear();
  if (!(alpha > 0.0 && alpha < 1.0) || !std::isfinite(alpha)) {
    failure_reason = "alpha_out_of_range";
    return false;
  }
  if (!(obstacle_support_radius_m >= 0.0) ||
      !std::isfinite(obstacle_support_radius_m)) {
    failure_reason = "invalid_obstacle_support_radius";
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
    const Vec3 center_direction = pivot - projection;
    const double center_distance = center_direction.norm();
    const double coordinate_scale =
        std::max(1.0, std::max(pivot.squaredNorm(), projection.squaredNorm()));
    const double degenerate_threshold =
        64.0 * std::numeric_limits<double>::epsilon() * coordinate_scale;
    if (!(center_distance * center_distance > degenerate_threshold) ||
        !std::isfinite(center_distance) ||
        !(center_distance > obstacle_support_radius_m)) {
      failure_reason = "seed_intersects_obstacle";
      return false;
    }

    // Treat every occupied voxel as a conservative sphere whose radius is
    // half the voxel diagonal.  The effective pivot is the closest point of
    // that support to the seed; the plane orientation is unchanged.
    const Vec3 unit_direction = center_direction / center_distance;
    const Vec3 supported_pivot = pivot - obstacle_support_radius_m * unit_direction;
    const Vec3 normal = supported_pivot - projection;
    const double distance_squared = normal.squaredNorm();

    const double offset = normal.dot(projection) + one_minus_alpha * distance_squared;
    generated.row(static_cast<Eigen::Index>(plane_count)) = normal.transpose();
    offsets(static_cast<Eigen::Index>(plane_count)) = offset;
    ++plane_count;

    const size_t before = remaining.size();
    remaining.erase(std::remove_if(remaining.begin(), remaining.end(),
                                   [&](const Vec3& obstacle) {
                                     return normal.dot(obstacle) -
                                                obstacle_support_radius_m * normal.norm() >
                                            offset;
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
  // safety argument. Keep the complete constraint set as a fallback, but do
  // not rescan every obstacle after a successful reduction on the planning
  // hot path. Only a genuine reduction failure pays for fallback validation.
  const MatX3 complete_A = result.A;
  const Eigen::VectorXd complete_b = result.b;
  MatX3 reduced_A = complete_A;
  Eigen::VectorXd reduced_b = complete_b;
  std::string reduction_reason;
  const bool reduced =
      reduceRedundantPlanes(reduced_A, reduced_b, reduction_seed, reduction_reason);
  if (reduced) {
    result.A.swap(reduced_A);
    result.b.swap(reduced_b);
  } else {
    if (!excludesAllObstacles(complete_A, complete_b, obstacles, 1e-8,
                              obstacle_support_radius_m)) {
      failure_reason = "reduction_failed_and_unreduced_obstacle_inside";
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
                      std::mt19937& rng, Result& result, std::string& failure_reason,
                      double obstacle_support_radius_m = 0.0) {
  return generate(obstacles, A_bound, b_bound, seed, alpha, rng,
                  [&](const Vec3&) { return seed; }, result, failure_reason,
                  obstacle_support_radius_m);
}

inline bool lineSeed(const std::vector<Vec3>& obstacles, const Vec3& start, const Vec3& end,
                     const MatX3& A_bound, const Eigen::VectorXd& b_bound, double alpha,
                     std::mt19937& rng, Result& result, std::string& failure_reason,
                     double obstacle_support_radius_m = 0.0) {
  if ((end - start).squaredNorm() <= std::numeric_limits<double>::epsilon()) {
    return pointSeed(obstacles, start, A_bound, b_bound, alpha, rng, result, failure_reason,
                     obstacle_support_radius_m);
  }
  const Vec3 midpoint = 0.5 * (start + end);
  return generate(obstacles, A_bound, b_bound, midpoint, alpha, rng,
                  [&](const Vec3& obstacle) {
                    return closestPointOnSegment(obstacle, start, end);
                  },
                  result, failure_reason, obstacle_support_radius_m);
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
                              PathCoverResult& output, std::string& failure_reason,
                              double obstacle_support_radius_m = 0.0) {
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
    const bool ok = lineSeed(obstacles, start, end, A_bound, b_bound, alpha, rng, result,
                             reason, obstacle_support_radius_m);
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

// Build a PathCover chain from point-seeded RISP corridors.  Each accepted
// corridor is followed along the HGP polyline until its first exit.  The next
// seed is placed before that exit to create deterministic overlap; if that
// candidate cannot make forward progress, the last fully covered HGP waypoint
// is tried as a conservative point-seed candidate.  This function never
// silently changes to line seeding.
template <typename PathContainer>
inline bool pathCoverPointSeed(const std::vector<Vec3>& obstacles,
                               const PathContainer& reference_path,
                               const MatX3& A_bound, const Eigen::VectorXd& b_bound,
                               double alpha, uint32_t random_seed, size_t max_corridors,
                               double overlap_backtrack_m, double minimum_overlap_margin_m,
                               double minimum_progress_m, size_t max_retries,
                               PathCoverResult& output, std::string& failure_reason,
                               double obstacle_support_radius_m = 0.0) {
  output = PathCoverResult{};
  failure_reason.clear();
  if (max_corridors == 0) {
    failure_reason = "invalid_max_corridors";
    return false;
  }
  if (!(overlap_backtrack_m >= 0.0) || !(minimum_overlap_margin_m >= 0.0) ||
      !(minimum_progress_m > 0.0) || !std::isfinite(overlap_backtrack_m) ||
      !std::isfinite(minimum_overlap_margin_m) || !std::isfinite(minimum_progress_m)) {
    failure_reason = "invalid_point_pathcover_parameters";
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

  std::vector<double> distances(path.size(), 0.0);
  for (size_t i = 1; i < path.size(); ++i)
    distances[i] = distances[i - 1] + (path[i] - path[i - 1]).norm();
  const double total_length = distances.back();
  if (!(total_length > minimum_progress_m)) {
    failure_reason = "reference_path_too_short";
    return false;
  }
  output.reference_waypoints = path.size();
  output.reference_length = total_length;

  auto point_at_distance = [&](double distance) -> Vec3 {
    const double clamped = std::clamp(distance, 0.0, total_length);
    const auto upper = std::upper_bound(distances.begin(), distances.end(), clamped);
    if (upper == distances.begin()) return path.front();
    if (upper == distances.end()) return path.back();
    const size_t index = static_cast<size_t>(upper - distances.begin());
    const double edge_length = distances[index] - distances[index - 1];
    const double fraction = edge_length > 1e-12
                                ? (clamped - distances[index - 1]) / edge_length
                                : 0.0;
    return (path[index - 1] + fraction * (path[index] - path[index - 1])).eval();
  };

  struct Coverage {
    double exit_distance{0.0};
    double last_waypoint_distance{0.0};
    bool reaches_end{false};
  };
  auto measure_coverage = [&](const Result& corridor, double seed_distance,
                              Coverage& coverage) {
    const Vec3 seed = point_at_distance(seed_distance);
    if (!contains(corridor.A, corridor.b, seed)) return false;
    coverage.exit_distance = seed_distance;
    coverage.last_waypoint_distance = seed_distance;
    coverage.reaches_end = false;
    Vec3 segment_start = seed;
    double segment_start_distance = seed_distance;
    size_t target_index = static_cast<size_t>(
        std::upper_bound(distances.begin(), distances.end(), seed_distance + 1e-10) -
        distances.begin());
    for (; target_index < path.size(); ++target_index) {
      const Vec3& target = path[target_index];
      const double target_distance = distances[target_index];
      if (contains(corridor.A, corridor.b, target)) {
        segment_start = target;
        segment_start_distance = target_distance;
        coverage.last_waypoint_distance = target_distance;
        coverage.exit_distance = target_distance;
        continue;
      }
      double exit_parameter = 0.0;
      if (!segmentExitParameter(corridor, segment_start, target, exit_parameter)) return false;
      coverage.exit_distance =
          segment_start_distance + exit_parameter * (target_distance - segment_start_distance);
      return true;
    }
    coverage.exit_distance = total_length;
    coverage.reaches_end = true;
    return true;
  };

  auto generate_corridor = [&](size_t corridor_index, size_t candidate_index,
                               size_t attempt, double seed_distance, Result& result,
                               std::string& reason) {
    std::seed_seq sequence{random_seed, static_cast<uint32_t>(corridor_index),
                           static_cast<uint32_t>(candidate_index),
                           static_cast<uint32_t>(attempt), 0x706f696eU};
    std::mt19937 rng(sequence);
    const auto begin = std::chrono::steady_clock::now();
    const bool ok = pointSeed(obstacles, point_at_distance(seed_distance), A_bound, b_bound,
                              alpha, rng, result, reason, obstacle_support_radius_m);
    output.risp_generation_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
            .count();
    return ok;
  };

  double previous_exit_distance = 0.0;
  double previous_seed_distance = -minimum_progress_m;
  Result previous_corridor;
  bool have_previous_corridor = false;
  std::vector<double> candidates{0.0};

  while (!candidates.empty()) {
    if (output.corridors.size() >= max_corridors) {
      output.horizon_exceeded = true;
      output.covered_length = previous_exit_distance;
      failure_reason = "max_corridors_exceeded";
      return false;
    }

    bool accepted = false;
    Result accepted_corridor;
    Coverage accepted_coverage;
    double accepted_seed_distance = 0.0;
    double accepted_overlap = std::numeric_limits<double>::infinity();
    std::string last_reason = "point_generation_failed";
    const size_t corridor_index = output.corridors.size();
    for (size_t candidate_index = 0; candidate_index < candidates.size() && !accepted;
         ++candidate_index) {
      const double candidate_distance = candidates[candidate_index];
      if (candidate_index > 0) ++output.point_waypoint_retry_count;
      if (candidate_distance <= previous_seed_distance + 1e-8) {
        ++output.point_no_progress_count;
        last_reason = "nonmonotonic_seed";
        continue;
      }
      const Vec3 candidate = point_at_distance(candidate_distance);
      const double previous_margin =
          have_previous_corridor
              ? normalizedMinimumSlack(previous_corridor.A, previous_corridor.b, candidate)
              : std::numeric_limits<double>::infinity();
      if (have_previous_corridor && previous_margin < minimum_overlap_margin_m) {
        last_reason = "degenerate_overlap";
        continue;
      }

      for (size_t attempt = 0; attempt <= max_retries && !accepted; ++attempt) {
        if (attempt > 0) ++output.point_retry_count;
        Result generated;
        std::string reason;
        if (!generate_corridor(corridor_index, candidate_index, attempt,
                               candidate_distance, generated, reason)) {
          last_reason = reason;
          continue;
        }
        const double new_margin = normalizedMinimumSlack(generated.A, generated.b, candidate);
        const double overlap_margin = std::min(previous_margin, new_margin);
        if (have_previous_corridor && overlap_margin < minimum_overlap_margin_m) {
          last_reason = "degenerate_overlap";
          continue;
        }
        Coverage coverage;
        if (!measure_coverage(generated, candidate_distance, coverage)) {
          last_reason = "exit_not_found";
          continue;
        }
        const double new_progress = coverage.exit_distance - previous_exit_distance;
        if (!coverage.reaches_end && new_progress < minimum_progress_m) {
          ++output.point_no_progress_count;
          last_reason = "no_forward_progress";
          continue;
        }
        accepted = true;
        accepted_corridor = std::move(generated);
        accepted_coverage = coverage;
        accepted_seed_distance = candidate_distance;
        accepted_overlap = overlap_margin;
      }
    }

    if (!accepted) {
      output.covered_length = previous_exit_distance;
      failure_reason = "corridor_" + std::to_string(corridor_index) + "_" + last_reason;
      return false;
    }

    if (have_previous_corridor) {
      output.waypoints.push_back(point_at_distance(accepted_seed_distance));
      output.waypoint_distances.push_back(accepted_seed_distance);
      output.min_overlap_margin = std::min(output.min_overlap_margin, accepted_overlap);
    } else {
      output.waypoints.push_back(path.front());
      output.waypoint_distances.push_back(0.0);
    }
    output.seed_segments.emplace_back(point_at_distance(accepted_seed_distance),
                                      point_at_distance(accepted_seed_distance));
    output.unreduced_retry_count += accepted_corridor.used_unreduced_retry ? 1U : 0U;
    output.corridor_progress_lengths.push_back(
        std::max(0.0, accepted_coverage.exit_distance - previous_exit_distance));
    output.corridors.push_back(accepted_corridor);
    previous_corridor = std::move(accepted_corridor);
    have_previous_corridor = true;
    previous_seed_distance = accepted_seed_distance;
    previous_exit_distance = accepted_coverage.exit_distance;
    output.covered_length = previous_exit_distance;

    if (accepted_coverage.reaches_end ||
        previous_exit_distance >= total_length - 1e-8) {
      output.waypoints.push_back(path.back());
      output.waypoint_distances.push_back(total_length);
      output.covered_length = total_length;
      output.full_coverage = output.waypoints.size() == output.corridors.size() + 1;
      if (output.corridors.size() == 1)
        output.min_overlap_margin = std::numeric_limits<double>::infinity();
      if (!output.full_coverage) {
        failure_reason = "corridor_waypoint_count_mismatch";
        return false;
      }
      return true;
    }

    const double primary = std::max(previous_seed_distance + 1e-8,
                                    previous_exit_distance - overlap_backtrack_m);
    candidates.clear();
    if (primary < previous_exit_distance - 1e-8) candidates.push_back(primary);
    const double waypoint = accepted_coverage.last_waypoint_distance;
    if (waypoint > previous_seed_distance + 1e-8 &&
        waypoint < previous_exit_distance - 1e-8 &&
        (candidates.empty() || std::abs(waypoint - candidates.front()) > 1e-8)) {
      candidates.push_back(waypoint);
    }
    if (candidates.empty()) {
      ++output.point_no_progress_count;
      failure_reason = "corridor_" + std::to_string(corridor_index) +
                       "_no_overlap_seed_candidate";
      return false;
    }
  }

  failure_reason = "point_pathcover_internal_error";
  return false;
}

// Reuse one line-seeded RISP corridor for as many consecutive original HGP
// segments as it contains. The optimizer path remains exactly the HGP path.
// If the last absorbed segment ends too close to the current corridor boundary,
// reserve that segment as the next seed instead of inserting a seam waypoint.
template <typename PathContainer>
inline bool pathCoverLineReuse(const std::vector<Vec3>& obstacles,
                               const PathContainer& reference_path,
                               const MatX3& A_bound, const Eigen::VectorXd& b_bound,
                               double alpha, uint32_t random_seed, size_t max_corridors,
                               double minimum_overlap_margin_m,
                               PathCoverReuseResult& output, std::string& failure_reason,
                               double obstacle_support_radius_m = 0.0) {
  output = PathCoverReuseResult{};
  failure_reason.clear();
  if (max_corridors == 0) {
    failure_reason = "invalid_max_corridors";
    return false;
  }
  if (!(minimum_overlap_margin_m >= 0.0) ||
      !std::isfinite(minimum_overlap_margin_m)) {
    failure_reason = "invalid_overlap_margin";
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
    if (!path.empty() && (point - path.back()).norm() <= 1e-8) {
      failure_reason = "degenerate_reference_segment";
      return false;
    }
    path.push_back(point);
  }
  if (path.size() < 2) {
    failure_reason = "path_too_short";
    return false;
  }
  output.reference_waypoints = path.size();
  output.reference_length = polylineLength(path);
  const size_t segment_count = path.size() - 1;
  output.segment_corridor_indices.reserve(segment_count);

  auto generate_corridor = [&](size_t corridor_index, size_t seed_segment,
                               Result& result, std::string& reason) {
    std::seed_seq sequence{random_seed, static_cast<uint32_t>(corridor_index),
                           static_cast<uint32_t>(seed_segment), 0x72657573U};
    std::mt19937 rng(sequence);
    const auto begin = std::chrono::steady_clock::now();
    const bool ok = lineSeed(obstacles, path[seed_segment], path[seed_segment + 1],
                             A_bound, b_bound, alpha, rng, result, reason,
                             obstacle_support_radius_m);
    output.risp_generation_ms +=
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin)
            .count();
    return ok;
  };

  size_t first_unassigned = 0;
  while (first_unassigned < segment_count) {
    if (output.corridors.size() >= max_corridors) {
      output.horizon_exceeded = true;
      failure_reason = "max_corridors_exceeded";
      return false;
    }

    const size_t seed_segment = first_unassigned;
    const size_t corridor_index = output.corridors.size();
    Result corridor;
    std::string reason;
    if (!generate_corridor(corridor_index, seed_segment, corridor, reason)) {
      failure_reason = "corridor_" + std::to_string(corridor_index) + "_" + reason;
      return false;
    }
    if (!contains(corridor.A, corridor.b, path[seed_segment]) ||
        !contains(corridor.A, corridor.b, path[seed_segment + 1])) {
      failure_reason = "corridor_" + std::to_string(corridor_index) +
                       "_seed_segment_outside";
      return false;
    }

    if (!output.corridors.empty()) {
      const Vec3& transition = path[first_unassigned];
      const Result& previous = output.corridors.back();
      const double transition_margin =
          std::min(normalizedMinimumSlack(previous.A, previous.b, transition),
                   normalizedMinimumSlack(corridor.A, corridor.b, transition));
      output.min_transition_margin =
          std::min(output.min_transition_margin, transition_margin);
      if (transition_margin < minimum_overlap_margin_m)
        ++output.weak_overlap_transition_count;
    }

    size_t last_contained = seed_segment;
    while (last_contained + 1 < segment_count &&
           contains(corridor.A, corridor.b, path[last_contained + 1]) &&
           contains(corridor.A, corridor.b, path[last_contained + 2])) {
      ++last_contained;
    }

    size_t last_assigned = last_contained;
    if (last_contained + 1 < segment_count) {
      const double outgoing_margin =
          normalizedMinimumSlack(corridor.A, corridor.b, path[last_contained + 1]);
      if (outgoing_margin < minimum_overlap_margin_m) {
        ++output.low_overlap_transition_count;
        if (last_contained > seed_segment) {
          // Keep the final absorbed segment unassigned. It becomes the next
          // line seed, so the two corridors overlap along that entire segment.
          --last_assigned;
          ++output.overlap_rollback_count;
        }
      }
    }

    const size_t assigned_count = last_assigned - seed_segment + 1;
    output.corridors.push_back(std::move(corridor));
    output.seed_segments.emplace_back(path[seed_segment], path[seed_segment + 1]);
    output.segments_per_corridor.push_back(assigned_count);
    output.unreduced_retry_count +=
        output.corridors.back().used_unreduced_retry ? 1U : 0U;
    output.reused_segment_count += assigned_count > 1 ? assigned_count - 1 : 0;
    for (size_t segment = seed_segment; segment <= last_assigned; ++segment)
      output.segment_corridor_indices.push_back(corridor_index);
    first_unassigned = last_assigned + 1;
  }

  output.full_coverage =
      output.segment_corridor_indices.size() == segment_count &&
      output.corridors.size() == output.segments_per_corridor.size();
  if (!output.full_coverage) {
    failure_reason = "segment_corridor_mapping_mismatch";
    return false;
  }
  if (output.corridors.size() == 1)
    output.min_transition_margin = std::numeric_limits<double>::infinity();
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
