#include <gtest/gtest.h>

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

}  // namespace
