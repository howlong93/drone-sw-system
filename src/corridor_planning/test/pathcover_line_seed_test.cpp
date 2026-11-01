#include <gtest/gtest.h>

#include <cstdlib>
#include <vector>

#include "planner/PathCover.hpp"

namespace {

using Vec2 = Eigen::Vector2d;
using MatX2 = Eigen::Matrix<double, Eigen::Dynamic, 2>;

void makeBounds(MatX2 &A, Eigen::VectorXd &b) {
  A.resize(4, 2);
  A << -1.0, 0.0,
        1.0, 0.0,
        0.0, -1.0,
        0.0, 1.0;
  b.resize(4);
  b << 5.0, 5.0, 5.0, 5.0;
}

bool strictlyInside(const MatX2 &A, const Eigen::VectorXd &b,
                    const Vec2 &point, double tolerance = 1e-10) {
  return ((A * point).array() < (b.array() - tolerance)).all();
}

bool excludedByAtLeastOnePlane(const MatX2 &A, const Eigen::VectorXd &b,
                               const Vec2 &point,
                               double tolerance = 1e-10) {
  return ((A * point).array() > (b.array() + tolerance)).any();
}

TEST(LineSeedRisp, ContainsCompleteSegmentAndExcludesObstacles) {
  MatX2 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);

  const Vec2 start(-1.0, 0.0);
  const Vec2 end(1.0, 0.0);
  const std::vector<Vec2> obstacles = {
      Vec2(0.0, 2.0), Vec2(0.0, -2.0),
      Vec2(3.0, 0.0), Vec2(-3.0, 0.0),
      Vec2(2.5, 2.5), Vec2(-2.5, -2.5)};

  MatX2 A;
  Eigen::VectorXd b;
  std::vector<int> remaining;
  std::srand(7);
  PathCover::RISPLineSeed<double, 2>(
      obstacles, start, end, A_bound, b_bound, A, b, remaining, 0.1);

  EXPECT_TRUE(strictlyInside(A, b, start));
  EXPECT_TRUE(strictlyInside(A, b, end));
  for (int i = 0; i <= 20; ++i) {
    const double t = static_cast<double>(i) / 20.0;
    EXPECT_TRUE(strictlyInside(A, b, (1.0 - t) * start + t * end));
  }
  for (const Vec2 &point : obstacles) {
    EXPECT_TRUE(excludedByAtLeastOnePlane(A, b, point));
  }
  ASSERT_FALSE(remaining.empty());
  EXPECT_EQ(remaining.front(), static_cast<int>(obstacles.size()));
  EXPECT_EQ(remaining.back(), 0);
}

TEST(LineSeedRisp, FallsBackToPointSeedForZeroLengthSegment) {
  MatX2 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);

  const Vec2 seed(0.0, 0.0);
  const std::vector<Vec2> obstacles = {
      Vec2(2.0, 0.0), Vec2(-2.0, 0.0),
      Vec2(0.0, 2.0), Vec2(0.0, -2.0)};

  MatX2 A;
  Eigen::VectorXd b;
  std::vector<int> remaining;
  std::srand(11);
  PathCover::RISPLineSeed<double, 2>(
      obstacles, seed, seed, A_bound, b_bound, A, b, remaining, 0.1);

  EXPECT_TRUE(strictlyInside(A, b, seed));
  for (const Vec2 &point : obstacles) {
    EXPECT_TRUE(excludedByAtLeastOnePlane(A, b, point));
  }
}

TEST(LineSeedRisp, RejectsObstaclePointOnSegment) {
  MatX2 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);

  const Vec2 start(-1.0, 0.0);
  const Vec2 end(1.0, 0.0);
  const std::vector<Vec2> obstacles = {Vec2(0.25, 0.0)};
  MatX2 A;
  Eigen::VectorXd b;
  std::vector<int> remaining;

  EXPECT_THROW(
      (PathCover::RISPLineSeed<double, 2>(
          obstacles, start, end, A_bound, b_bound,
          A, b, remaining, 0.1)),
      std::invalid_argument);
}

TEST(LineSeedRisp, LimitsLongSeedSegments) {
  const Vec2 start(0.0, 0.0);
  const Vec2 target(3.0, 4.0);
  const Vec2 limited =
      PathCover::limitSegmentLength<double, 2>(start, target, 2.0);

  EXPECT_NEAR((limited - start).norm(), 2.0, 1e-12);
  EXPECT_NEAR(limited.x(), 1.2, 1e-12);
  EXPECT_NEAR(limited.y(), 1.6, 1e-12);
}

TEST(LineSeedPathCover, RespectsHorizonAndProducesCorridor) {
  MatX2 A_bound;
  Eigen::VectorXd b_bound;
  makeBounds(A_bound, b_bound);

  const std::vector<Vec2> obstacles = {
      Vec2(0.0, 2.0), Vec2(0.0, -2.0),
      Vec2(2.0, 2.0), Vec2(2.0, -2.0),
      Vec2(4.0, 2.0), Vec2(4.0, -2.0)};
  const std::vector<Vec2> path = {
      Vec2(-4.0, 0.0), Vec2(0.0, 0.0), Vec2(4.0, 0.0)};

  std::vector<MatX2> corridor_A;
  std::vector<Eigen::VectorXd> corridor_b;
  std::vector<Vec2> seeds;
  std::vector<std::vector<int>> remaining;
  std::srand(17);
  PathCover::pathCoverLineSeed<double, 2>(
      obstacles, path, A_bound, b_bound, corridor_A, corridor_b,
      seeds, remaining, 3, 0.1, 1.0);

  ASSERT_FALSE(corridor_A.empty());
  EXPECT_LE(corridor_A.size(), 3u);
  EXPECT_EQ(corridor_A.size(), corridor_b.size());
  EXPECT_EQ(corridor_A.size(), remaining.size());
  EXPECT_EQ(seeds.size(), corridor_A.size() + 1);
}

}  // namespace
