#ifndef PATHCOVER_HPP
#define PATHCOVER_HPP

#include <iostream>
#include <Eigen/Dense>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>
#include <libqhullcpp/Qhull.h>


namespace PathCover {

template<typename T, const int dim> // Uses qhull
inline void qConvexHull(Eigen::Matrix<T, -1, -1, Eigen::RowMajor> &Points, 
                        std::vector<int> &idx) {
    
    std::vector<double> points_mapped(Points.data(), Points.data() + Points.size());
    orgQhull::Qhull qhull = orgQhull::Qhull("", dim, Points.rows(), points_mapped.data(), "");
    
    idx.reserve(qhull.vertexList().size());
    for (auto vi = qhull.vertexList().begin(); vi != qhull.vertexList().end(); ++vi) {
      idx.push_back(vi->point().id());
    }
}    


template<typename T, const int dim>
inline void noredund(Eigen::Matrix<T, -1, dim> &A, 
                     Eigen::Matrix<T, -1, 1> &b, 
                     const Eigen::Matrix<T, dim, 1> &seed) {

    // Ensure seed is strictly inside the polytope by checking A*seed < b
    Eigen::Matrix<T, -1, 1> check = b - A * seed;
    for (int i = 0; i < check.size(); ++i) {
        if (check(i) <= 1e-8) {  // Allow small tolerance
            std::cout << A << "\n" << b.transpose() << "\n" << seed.transpose() << "\n";
            throw std::invalid_argument("Seed point is not strictly inside the polytope. Redundancy removal failed!");
        }
    }                        
    
    // obtain dual polytope vertices
    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> D = (A.array().colwise() / (b - A * seed).array());
    

    std::vector<int> idx;

    qConvexHull<T, dim>(D, idx);

    
    // Temporary matrices to store the rows corresponding to the convex hull
    Eigen::Matrix<T, -1, dim> An(idx.size(), A.cols());
    Eigen::Matrix<T, -1, 1> bn(idx.size());

    for (int i = 0; i < idx.size(); ++i) {
        An.row(i) = A.row(idx[i]);
        bn(i) = b(idx[i]);
    }

    A = An;
    b = bn;
}


// RISP (Randomized Iterative Space Partitioning), Algorithm 2 of the paper.
// For each randomly sampled obstacle point p, the cutting hyperplane is
//
//     a = p - y_seed,
//     b = a^T y_seed + (1 - alpha) * ||a||^2,
//
// which is Eq. (4) of the paper. The plane passes through the point
// alpha*y_seed + (1-alpha)*p and the open ball of radius (1-alpha)*||a||
// around y_seed is strictly contained in the half-space a^T x <= b.
// alpha must lie in (0, 1); smaller alpha -> larger polytope but less margin,
// larger alpha -> smaller polytope with a wider safety annulus around the seed.
template<typename T, const int dim>
inline void RISP(const std::vector<Eigen::Matrix<T, dim, 1>> &obstacle_pts, 
                 const Eigen::Matrix<T, dim, 1> &seed, 
                 const Eigen::Matrix<T, -1, dim> &A_bound, 
                 const Eigen::Matrix<T, -1, 1> &b_bound, 
                 Eigen::Matrix<T, -1, dim> &A, 
                 Eigen::Matrix<T, -1, 1> &b,
                 std::vector<int> &pts_remaining,
                 T alpha = T(0.01)) {
    assert(alpha > T(0) && alpha < T(1) && "RISP: alpha must lie in (0, 1)");

    std::vector<Eigen::Matrix<T, dim, 1>> obs_to_consider = obstacle_pts; // Copy for modification

    // Preallocate A and b with a conservative size
    A.conservativeResize(obstacle_pts.size(), dim);
    b.conservativeResize(obstacle_pts.size());

    // Clear and reserve pts_remaining
    pts_remaining.clear();
    pts_remaining.reserve(obstacle_pts.size());
    pts_remaining.push_back(static_cast<int>(obs_to_consider.size()));    

    // Start the loop
    Eigen::Matrix<T, dim, 1> xi, normal_vector;
    T b0;
    int k = 0;
    const T one_minus_alpha = T(1) - alpha;
    while (!obs_to_consider.empty()) {
        // Select a random obstacle point
        xi = obs_to_consider[rand() % obs_to_consider.size()];

        // Compute the normal vector and offset following Eq. (4):
        //   a = xi - seed,  b = a . seed + (1 - alpha) * ||a||^2.
        normal_vector = (xi - seed);
        b0 = normal_vector.dot(seed) + one_minus_alpha * normal_vector.squaredNorm();

        // Add the constraint to A and b
        A.row(k) = normal_vector.transpose();
        b(k) = b0;

        // Discard points strictly on the far side of the hyperplane (Alg. 2,
        // lines 7-9: remove q iff a^T q > b). With alpha > 0 the sampled xi
        // itself satisfies a^T xi - b = alpha*||a||^2 > 0, so it is removed
        // and the loop is guaranteed to terminate.
        auto it = std::remove_if(obs_to_consider.begin(), obs_to_consider.end(),
            [&normal_vector, b0](const Eigen::Matrix<T, dim, 1>& obs) {
                return normal_vector.dot(obs) > b0;
            });
        obs_to_consider.erase(it, obs_to_consider.end());

        // Record the number of remaining obstacle points after this separation step
        pts_remaining.push_back(static_cast<int>(obs_to_consider.size()));
                
        k++;
    }

    // Resize A and b to the actual number of constraints
    A.conservativeResize(k + A_bound.rows(), Eigen::NoChange);
    b.conservativeResize(k + b_bound.size());

    // Append A_bound and b_bound to A and b
    A.bottomRows(A_bound.rows()) = A_bound;
    b.tail(b_bound.size()) = b_bound;

    // Remove redundant constraints
    noredund<T, dim>(A, b, seed);
}


// Return the closest point on the closed line segment [segment_start,
// segment_end]. A degenerate segment is treated as a point seed.
template<typename T, const int dim>
inline Eigen::Matrix<T, dim, 1> closestPointOnSegment(
        const Eigen::Matrix<T, dim, 1> &point,
        const Eigen::Matrix<T, dim, 1> &segment_start,
        const Eigen::Matrix<T, dim, 1> &segment_end) {
    const Eigen::Matrix<T, dim, 1> direction = segment_end - segment_start;
    const T length_squared = direction.squaredNorm();
    if (length_squared <= std::numeric_limits<T>::epsilon()) {
        return segment_start;
    }

    const T projection = (point - segment_start).dot(direction) / length_squared;
    const T clamped = std::max(T(0), std::min(T(1), projection));
    return segment_start + clamped * direction;
}


// Line-seeded RISP.
//
// For a sampled obstacle point p, let z be its Euclidean projection onto the
// closed seed segment S=[segment_start, segment_end]. The separating plane is
//
//     a = p - z,
//     b = a^T z + (1 - alpha) * ||a||^2.
//
// Projection optimality gives a^T(x-z) <= 0 for every x in S, so the complete
// segment is strictly contained in a^T x <= b. The pivot is strictly excluded
// because a^T p - b = alpha*||a||^2 > 0. Once the plane is stored, every other
// obstacle q satisfying a^T q > b is already excluded by that same half-space
// and can be removed exactly as in point-seeded RISP.
template<typename T, const int dim>
inline void RISPLineSeed(
        const std::vector<Eigen::Matrix<T, dim, 1>> &obstacle_pts,
        const Eigen::Matrix<T, dim, 1> &segment_start,
        const Eigen::Matrix<T, dim, 1> &segment_end,
        const Eigen::Matrix<T, -1, dim> &A_bound,
        const Eigen::Matrix<T, -1, 1> &b_bound,
        Eigen::Matrix<T, -1, dim> &A,
        Eigen::Matrix<T, -1, 1> &b,
        std::vector<int> &pts_remaining,
        T alpha = T(0.01)) {
    assert(alpha > T(0) && alpha < T(1) &&
           "RISPLineSeed: alpha must lie in (0, 1)");

    const Eigen::Matrix<T, dim, 1> segment_direction =
        segment_end - segment_start;
    if (segment_direction.squaredNorm() <= std::numeric_limits<T>::epsilon()) {
        RISP<T, dim>(obstacle_pts, segment_start, A_bound, b_bound,
                     A, b, pts_remaining, alpha);
        return;
    }

    std::vector<Eigen::Matrix<T, dim, 1>> obs_to_consider = obstacle_pts;
    A.conservativeResize(obstacle_pts.size(), dim);
    b.conservativeResize(obstacle_pts.size());

    pts_remaining.clear();
    pts_remaining.reserve(obstacle_pts.size() + 1);
    pts_remaining.push_back(static_cast<int>(obs_to_consider.size()));

    int k = 0;
    const T one_minus_alpha = T(1) - alpha;
    while (!obs_to_consider.empty()) {
        const Eigen::Matrix<T, dim, 1> pivot =
            obs_to_consider[rand() % obs_to_consider.size()];
        const Eigen::Matrix<T, dim, 1> projection =
            closestPointOnSegment<T, dim>(pivot, segment_start, segment_end);
        const Eigen::Matrix<T, dim, 1> normal_vector = pivot - projection;
        const T distance_squared = normal_vector.squaredNorm();

        // This is a geometric invalidity, rather than the ordinary case where
        // a line is merely close to one side of a wide corridor. When the
        // distance is numerically zero no separating normal exists and the
        // supplied segment intersects an input obstacle sample.
        const T coordinate_scale = std::max(
            T(1), std::max(pivot.squaredNorm(), projection.squaredNorm()));
        const T degenerate_threshold =
            T(64) * std::numeric_limits<T>::epsilon() * coordinate_scale;
        if (distance_squared <= degenerate_threshold) {
            throw std::invalid_argument(
                "RISPLineSeed: seed segment intersects an obstacle point");
        }

        const T b0 = normal_vector.dot(projection) +
                     one_minus_alpha * distance_squared;
        A.row(k) = normal_vector.transpose();
        b(k) = b0;

        auto it = std::remove_if(
            obs_to_consider.begin(), obs_to_consider.end(),
            [&normal_vector, b0](const Eigen::Matrix<T, dim, 1> &obs) {
                return normal_vector.dot(obs) > b0;
            });
        obs_to_consider.erase(it, obs_to_consider.end());
        pts_remaining.push_back(static_cast<int>(obs_to_consider.size()));
        ++k;
    }

    A.conservativeResize(k + A_bound.rows(), Eigen::NoChange);
    b.conservativeResize(k + b_bound.size());
    A.bottomRows(A_bound.rows()) = A_bound;
    b.tail(b_bound.size()) = b_bound;

    // Every point of the segment is strictly inside all generated separators;
    // use its midpoint as the interior point for the dual-space reduction.
    const Eigen::Matrix<T, dim, 1> reduction_seed =
        (segment_start + segment_end) / T(2);
    noredund<T, dim>(A, b, reduction_seed);
}


template<typename T, const int dim>
inline Eigen::Matrix<T, dim, 1> limitSegmentLength(
        const Eigen::Matrix<T, dim, 1> &segment_start,
        const Eigen::Matrix<T, dim, 1> &target,
        T maximum_length) {
    const Eigen::Matrix<T, dim, 1> displacement = target - segment_start;
    const T length = displacement.norm();
    if (maximum_length <= T(0) || length <= maximum_length) {
        return target;
    }
    return segment_start + (maximum_length / length) * displacement;
}


template <typename T, int Dim>
inline Eigen::Matrix<T, Dim, 1> computePolyhedraIntersection(const Eigen::Matrix<T, -1, Dim> &A, 
                                                             const Eigen::Matrix<T, -1, 1> &b,
                                                             const Eigen::Matrix<T, Dim, 1> &p1, 
                                                             const Eigen::Matrix<T, Dim, 1> &p2) {
    T t_min = std::numeric_limits<T>::max();
    Eigen::Matrix<T, Dim, 1> intersection_point;

    Eigen::Matrix<T, Dim, 1> direction = p2 - p1;

    for (int i = 0; i < A.rows(); ++i) {
        T denom = A.row(i).dot(direction);
        if (std::abs(denom) < 1e-10) continue; // Skip parallel planes

        T t = (b(i) - A.row(i).dot(p1)) / denom;
        
        if (t >= 0.0 && t <= 1.0) { // Intersection must be within segment
            Eigen::Matrix<T, Dim, 1> candidate = p1 + t * direction;

            if (t < t_min) { // Find the first intersection along the segment
                t_min = t;
                intersection_point = candidate;
            }
        }
    }

    if(!intersection_point.size()) {
        throw std::runtime_error("No valid intersection found!\n");
    }

    return intersection_point;
}


template <typename T, int Dim>
inline void pathCover(const std::vector<Eigen::Matrix<T, Dim, 1>> &obstacle_pts, 
                      const std::vector<Eigen::Matrix<T, Dim, 1>> &path,
                      const Eigen::Matrix<T, -1, Dim> &A_bound, 
                      const Eigen::Matrix<T, -1, 1> &b_bound,
                      std::vector<Eigen::Matrix<T, -1, Dim>> &A, 
                      std::vector<Eigen::Matrix<T, -1, 1>> &b,
                      std::vector<Eigen::Matrix<T, Dim, 1>> &seed, 
                      std::vector<std::vector<int>> &all_pts_remaining,
                      int max_horizon = 1,
                      T alpha = T(0.01)) {

    Eigen::Matrix<T, -1, Dim> A_;
    Eigen::Matrix<T, -1, 1> b_;
    Eigen::Matrix<T, Dim, 1> intersection = path[0]; // Initialize to prevent garbage data
    std::vector<int> pts_remaining;

    RISP<T, Dim>(obstacle_pts, path[0], A_bound, b_bound, A_, b_, pts_remaining, alpha);
    A.push_back(A_);
    b.push_back(b_);
    seed.push_back(path[0]);
    all_pts_remaining.push_back(pts_remaining);

    int i = 1;
    while((i < path.size() || ((A_ * path.back()).array() > b_.array()).all())) {
        if (((A_ * path[i]).array() <= b_.array()).all()) {
            intersection = path[i]; // The point is already inside the polyhedron defined by previous constraints
            i++;
            continue;
        } 
        else {
            intersection = computePolyhedraIntersection<T, Dim>(A_, b_, intersection, path[i]);
            RISP<T, Dim>(obstacle_pts, intersection, A_bound, b_bound, A_, b_, pts_remaining, alpha);
            A.push_back(A_);
            b.push_back(b_);
            seed.push_back(intersection);
            all_pts_remaining.push_back(pts_remaining);
        }

        // Stop if the maximum number of polyhedra has been reached
        if (A.size() > max_horizon) {
            break;
        }
    }   

    if (((A_ * path.back()).array() > b_.array()).all()) {
        seed.push_back(path.back());
    }
    else {
        seed.push_back(intersection);
    }
}


// PathCover variant using a geometric line seed for every RISP call. The
// first seed follows the first reference-path edge. Each subsequent seed starts
// exactly at the point where the reference path exits the current polytope and
// extends toward the next uncovered waypoint. Long edges are split by
// max_seed_segment_length; the next iteration continues from the guaranteed
// interior end of the truncated segment.
//
// This variant intentionally treats the reference segment as a geometric guide.
// It guarantees segment containment in the point-cloud polytope, but does not
// require a body-eroded corridor to contain the same reference segment.
template <typename T, int Dim>
inline void pathCoverLineSeed(
        const std::vector<Eigen::Matrix<T, Dim, 1>> &obstacle_pts,
        const std::vector<Eigen::Matrix<T, Dim, 1>> &path,
        const Eigen::Matrix<T, -1, Dim> &A_bound,
        const Eigen::Matrix<T, -1, 1> &b_bound,
        std::vector<Eigen::Matrix<T, -1, Dim>> &A,
        std::vector<Eigen::Matrix<T, -1, 1>> &b,
        std::vector<Eigen::Matrix<T, Dim, 1>> &seed,
        std::vector<std::vector<int>> &all_pts_remaining,
        int max_horizon = 1,
        T alpha = T(0.01),
        T max_seed_segment_length = T(1.0)) {
    if (path.empty()) {
        throw std::invalid_argument("pathCoverLineSeed: path must not be empty");
    }
    if (max_horizon < 1) {
        throw std::invalid_argument(
            "pathCoverLineSeed: max_horizon must be at least one");
    }

    Eigen::Matrix<T, -1, Dim> A_current;
    Eigen::Matrix<T, -1, 1> b_current;
    std::vector<int> pts_remaining;

    const Eigen::Matrix<T, Dim, 1> first_target =
        path.size() > 1 ? path[1] : path[0];
    Eigen::Matrix<T, Dim, 1> covered_point =
        limitSegmentLength<T, Dim>(path[0], first_target,
                                   max_seed_segment_length);

    RISPLineSeed<T, Dim>(obstacle_pts, path[0], covered_point,
                         A_bound, b_bound, A_current, b_current,
                         pts_remaining, alpha);
    A.push_back(A_current);
    b.push_back(b_current);
    seed.push_back(path[0]);
    all_pts_remaining.push_back(pts_remaining);

    size_t path_index = 1;
    while (path_index < path.size()) {
        if (((A_current * path[path_index]).array() <=
             b_current.array()).all()) {
            covered_point = path[path_index];
            ++path_index;
            continue;
        }

        if (static_cast<int>(A.size()) >= max_horizon) {
            break;
        }

        const Eigen::Matrix<T, Dim, 1> exit_point =
            computePolyhedraIntersection<T, Dim>(
                A_current, b_current, covered_point, path[path_index]);
        const Eigen::Matrix<T, Dim, 1> segment_end =
            limitSegmentLength<T, Dim>(exit_point, path[path_index],
                                       max_seed_segment_length);

        RISPLineSeed<T, Dim>(obstacle_pts, exit_point, segment_end,
                             A_bound, b_bound, A_current, b_current,
                             pts_remaining, alpha);
        A.push_back(A_current);
        b.push_back(b_current);
        seed.push_back(exit_point);
        all_pts_remaining.push_back(pts_remaining);

        // The complete line seed is guaranteed to be inside the new polytope,
        // so this is a valid and strictly forward starting point for the next
        // intersection query if the same long path edge remains uncovered.
        covered_point = segment_end;
    }

    if (((A_current * path.back()).array() <= b_current.array()).all()) {
        seed.push_back(path.back());
    } else {
        seed.push_back(covered_point);
    }
}

}

#endif // PATHCOVER_HPP
