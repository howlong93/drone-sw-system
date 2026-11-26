#include <gtest/gtest.h>

#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "mighty/risp_corridor.hpp"

namespace {

using mighty::risp::MatX3;
using mighty::risp::Result;
using mighty::risp::Vec3;

void makeBounds(MatX3& A, Eigen::VectorXd& b) {
  A.resize(6, 3);
  A << -1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0, 1.0;
  b = Eigen::VectorXd::Constant(6, 5.0);
}

bool outside(const Result& result, const Vec3& point) {
  return ((result.A * point - result.b).array() > 1e-10).any();
}

bool supportOutside(const Result& result, const Vec3& center, double radius) {
  for (Eigen::Index row = 0; row < result.A.rows(); ++row) {
    const double norm = result.A.row(row).norm();
    if (result.A.row(row).dot(center) - radius * norm > result.b(row) + 1e-10)
      return true;
  }
  return false;
}

TEST(RispCorridor, LineSeedContainsSegmentAndExcludesObstacles) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const Vec3 start(-1.0, 0.0, 0.0);
  const Vec3 end(1.0, 0.0, 0.0);
  const std::vector<Vec3> obstacles = {
      Vec3(0.0, 2.0, 0.0), Vec3(0.0, -2.0, 0.0), Vec3(0.0, 0.0, 2.0),
      Vec3(0.0, 0.0, -2.0), Vec3(3.0, 0.0, 0.0), Vec3(-3.0, 0.0, 0.0)};

  std::mt19937 rng(17);
  Result result;
  std::string reason;
  ASSERT_TRUE(mighty::risp::lineSeed(obstacles, start, end, A_bound, b_bound, 0.001,
                                      rng, result, reason))
      << reason;
  for (int i = 0; i <= 20; ++i) {
    const double t = static_cast<double>(i) / 20.0;
    EXPECT_TRUE(mighty::risp::isStrictlyInside(result.A, result.b,
                                                (1.0 - t) * start + t * end));
  }
  for (const auto& obstacle : obstacles) EXPECT_TRUE(outside(result, obstacle));
}

TEST(RispCorridor, FixedSeedIsDeterministic) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const std::vector<Vec3> obstacles = {
      Vec3(2.0, 0.0, 0.0), Vec3(-2.0, 0.0, 0.0), Vec3(0.0, 2.0, 0.0),
      Vec3(0.0, -2.0, 0.0), Vec3(0.0, 0.0, 2.0), Vec3(0.0, 0.0, -2.0)};
  Result first, second;
  std::string reason;
  std::mt19937 first_rng(42), second_rng(42);
  ASSERT_TRUE(mighty::risp::pointSeed(obstacles, Vec3::Zero(), A_bound, b_bound, 0.001,
                                       first_rng, first, reason))
      << reason;
  ASSERT_TRUE(mighty::risp::pointSeed(obstacles, Vec3::Zero(), A_bound, b_bound, 0.001,
                                       second_rng, second, reason))
      << reason;
  ASSERT_EQ(first.A.rows(), second.A.rows());
  EXPECT_TRUE(first.A.isApprox(second.A, 1e-12));
  EXPECT_TRUE(first.b.isApprox(second.b, 1e-12));
}

TEST(RispCorridor, LineSeedExcludesCompleteVoxelSupport) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const Vec3 start(-1.0, 0.0, 0.0);
  const Vec3 end(1.0, 0.0, 0.0);
  const std::vector<Vec3> obstacles{Vec3(0.0, 1.0, 0.0), Vec3(0.0, -1.0, 0.0)};
  constexpr double support_radius = 0.2;

  std::mt19937 rng(19);
  Result result;
  std::string reason;
  ASSERT_TRUE(mighty::risp::lineSeed(obstacles, start, end, A_bound, b_bound, 0.05,
                                      rng, result, reason, support_radius))
      << reason;
  for (const auto& obstacle : obstacles)
    EXPECT_TRUE(supportOutside(result, obstacle, support_radius));
}

TEST(RispCorridor, RejectsZeroAlphaAndObstacleOnLine) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  Result result;
  std::string reason;
  std::mt19937 rng(3);
  EXPECT_FALSE(mighty::risp::pointSeed({Vec3(2.0, 0.0, 0.0)}, Vec3::Zero(), A_bound,
                                        b_bound, 0.0, rng, result, reason));
  EXPECT_EQ(reason, "alpha_out_of_range");

  std::mt19937 line_rng(3);
  EXPECT_FALSE(mighty::risp::lineSeed({Vec3(0.0, 0.0, 0.0)}, Vec3(-1.0, 0.0, 0.0),
                                       Vec3(1.0, 0.0, 0.0), A_bound, b_bound, 0.001,
                                       line_rng, result, reason));
  EXPECT_EQ(reason, "seed_intersects_obstacle");
}

TEST(RispCorridor, PathCoverLineBuildsOverlappingCorridorChain) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const std::vector<Vec3> path = {Vec3(-2.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0),
                                  Vec3(0.0, 2.0, 0.0), Vec3(2.0, 2.0, 0.0)};
  const std::vector<Vec3> obstacles = {
      Vec3(-1.0, 0.6, 0.0), Vec3(-1.0, -0.6, 0.0), Vec3(-1.0, 0.0, 0.6),
      Vec3(-1.0, 0.0, -0.6), Vec3(0.6, 1.0, 0.0), Vec3(-0.6, 1.0, 0.0),
      Vec3(0.0, 1.0, 0.6), Vec3(0.0, 1.0, -0.6), Vec3(1.0, 2.6, 0.0),
      Vec3(1.0, 1.4, 0.0), Vec3(1.0, 2.0, 0.6), Vec3(1.0, 2.0, -0.6)};

  mighty::risp::PathCoverResult result;
  std::string reason;
  ASSERT_TRUE(mighty::risp::pathCoverLineSeed(obstacles, path, A_bound, b_bound, 0.05, 7,
                                               8, 0.25, 1e-4, result, reason))
      << reason;
  ASSERT_GE(result.corridors.size(), 2U);
  ASSERT_EQ(result.waypoints.size(), result.corridors.size() + 1);
  EXPECT_TRUE(result.full_coverage);
  EXPECT_NEAR(result.reference_length, result.covered_length, 1e-12);
  for (size_t i = 0; i < result.corridors.size(); ++i) {
    EXPECT_TRUE(mighty::risp::contains(result.corridors[i].A, result.corridors[i].b,
                                       result.waypoints[i]));
    EXPECT_TRUE(mighty::risp::contains(result.corridors[i].A, result.corridors[i].b,
                                       result.waypoints[i + 1]));
    for (const auto& obstacle : obstacles)
      EXPECT_TRUE(outside(result.corridors[i], obstacle));
  }
  for (size_t i = 1; i + 1 < result.waypoints.size(); ++i) {
    EXPECT_GT(mighty::risp::normalizedMinimumSlack(
                  result.corridors[i - 1].A, result.corridors[i - 1].b, result.waypoints[i]),
              0.0);
    EXPECT_GT(mighty::risp::normalizedMinimumSlack(
                  result.corridors[i].A, result.corridors[i].b, result.waypoints[i]),
              0.0);
  }
}

TEST(RispCorridor, PathCoverLineRejectsIncompleteHorizon) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const std::vector<Vec3> path = {Vec3(-2.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0),
                                  Vec3(0.0, 2.0, 0.0)};
  const std::vector<Vec3> obstacles = {Vec3(-1.0, 0.5, 0.0), Vec3(-1.0, -0.5, 0.0),
                                       Vec3(0.5, 1.0, 0.0), Vec3(-0.5, 1.0, 0.0)};
  mighty::risp::PathCoverResult result;
  std::string reason;
  EXPECT_FALSE(mighty::risp::pathCoverLineSeed(obstacles, path, A_bound, b_bound, 0.05, 7,
                                                1, 0.25, 1e-4, result, reason));
  EXPECT_TRUE(result.horizon_exceeded);
  EXPECT_EQ(reason, "max_corridors_exceeded");
}

TEST(RispCorridor, PathCoverPointBuildsProgressiveOverlappingChain) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const std::vector<Vec3> path = {Vec3(-2.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0),
                                  Vec3(0.0, 2.0, 0.0), Vec3(2.0, 2.0, 0.0)};
  const std::vector<Vec3> obstacles = {
      Vec3(-1.0, 0.6, 0.0), Vec3(-1.0, -0.6, 0.0), Vec3(-1.0, 0.0, 0.6),
      Vec3(-1.0, 0.0, -0.6), Vec3(0.6, 1.0, 0.0), Vec3(-0.6, 1.0, 0.0),
      Vec3(0.0, 1.0, 0.6), Vec3(0.0, 1.0, -0.6), Vec3(1.0, 2.6, 0.0),
      Vec3(1.0, 1.4, 0.0), Vec3(1.0, 2.0, 0.6), Vec3(1.0, 2.0, -0.6)};

  mighty::risp::PathCoverResult result;
  std::string reason;
  ASSERT_TRUE(mighty::risp::pathCoverPointSeed(obstacles, path, A_bound, b_bound, 0.05, 7,
                                                12, 0.25, 1e-4, 0.05, 3, result,
                                                reason))
      << reason;
  ASSERT_FALSE(result.corridors.empty());
  ASSERT_EQ(result.waypoints.size(), result.corridors.size() + 1);
  ASSERT_EQ(result.corridor_progress_lengths.size(), result.corridors.size());
  EXPECT_TRUE(result.full_coverage);
  EXPECT_NEAR(result.reference_length, result.covered_length, 1e-12);
  for (size_t i = 0; i < result.corridors.size(); ++i) {
    EXPECT_TRUE(mighty::risp::contains(result.corridors[i].A, result.corridors[i].b,
                                       result.waypoints[i]));
    EXPECT_TRUE(mighty::risp::contains(result.corridors[i].A, result.corridors[i].b,
                                       result.waypoints[i + 1]));
    for (const auto& obstacle : obstacles)
      EXPECT_TRUE(outside(result.corridors[i], obstacle));
  }

  mighty::risp::ExpandedPathCover expanded;
  ASSERT_TRUE(mighty::risp::expandPathCoverForOptimizer(path, result, expanded, reason))
      << reason;
  for (const auto& hgp_waypoint : path) {
    EXPECT_TRUE(std::any_of(expanded.waypoints.begin(), expanded.waypoints.end(),
                            [&](const Vec3& point) {
                              return point.isApprox(hgp_waypoint, 1e-8);
                            }));
  }
}

TEST(RispCorridor, ExpandsOptimizerPathWithoutDiscardingHgpBends) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const std::vector<Vec3> path = {Vec3(0.0, 0.0, 0.0), Vec3(1.0, 0.0, 0.0),
                                  Vec3(2.0, 0.0, 0.0), Vec3(3.0, 1.0, 0.0)};
  const Vec3 seam(2.5, 0.5, 0.0);

  mighty::risp::PathCoverResult cover;
  Result corridor;
  corridor.A = A_bound;
  corridor.b = b_bound;
  cover.corridors = {corridor, corridor};
  cover.waypoints = {path.front(), seam, path.back()};
  cover.reference_length = 2.0 + std::sqrt(2.0);
  cover.waypoint_distances = {0.0, 2.0 + std::sqrt(0.5), cover.reference_length};
  cover.full_coverage = true;

  mighty::risp::ExpandedPathCover expanded;
  std::string reason;
  ASSERT_TRUE(mighty::risp::expandPathCoverForOptimizer(path, cover, expanded, reason))
      << reason;
  ASSERT_EQ(expanded.waypoints.size(), 5U);
  EXPECT_TRUE(expanded.waypoints[1].isApprox(path[1]));
  EXPECT_TRUE(expanded.waypoints[2].isApprox(path[2]));
  EXPECT_TRUE(expanded.waypoints[3].isApprox(seam));
  EXPECT_EQ(expanded.inserted_seams, 1U);
  EXPECT_EQ(expanded.corridor_indices, (std::vector<size_t>{0, 0, 0, 1}));
}

TEST(RispCorridor, PathCoverLineReuseMapsOriginalSegmentsWithoutNewWaypoints) {
  MatX3 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);
  const std::vector<Vec3> path = {Vec3(-2.0, 0.0, 0.0), Vec3(-1.0, 0.0, 0.0),
                                  Vec3(0.0, 0.5, 0.0), Vec3(1.0, 0.5, 0.0),
                                  Vec3(2.0, 1.0, 0.0)};

  mighty::risp::PathCoverReuseResult result;
  std::string reason;
  ASSERT_TRUE(mighty::risp::pathCoverLineReuse({}, path, A_bound, b_bound, 0.05, 7,
                                                8, 1e-4, result, reason))
      << reason;
  ASSERT_TRUE(result.full_coverage);
  ASSERT_EQ(result.segment_corridor_indices.size(), path.size() - 1);
  ASSERT_EQ(result.segments_per_corridor.size(), result.corridors.size());
  EXPECT_EQ(std::accumulate(result.segments_per_corridor.begin(),
                            result.segments_per_corridor.end(), size_t{0}),
            path.size() - 1);
  EXPECT_LT(result.corridors.size(), path.size() - 1);
  EXPECT_GT(result.reused_segment_count, 0U);
  for (size_t segment = 0; segment + 1 < path.size(); ++segment) {
    const size_t corridor_index = result.segment_corridor_indices[segment];
    ASSERT_LT(corridor_index, result.corridors.size());
    EXPECT_TRUE(mighty::risp::contains(result.corridors[corridor_index].A,
                                       result.corridors[corridor_index].b,
                                       path[segment]));
    EXPECT_TRUE(mighty::risp::contains(result.corridors[corridor_index].A,
                                       result.corridors[corridor_index].b,
                                       path[segment + 1]));
  }
}

}  // namespace
