#!/usr/bin/env python3

import csv
import itertools
import json
import math
import os
import queue
import signal
import statistics
import subprocess
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from polytope_msgs.msg import Polytopes
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64, Int64MultiArray


GEOMETRY_TOLERANCE = 1.0e-7


def canonicalize_halfspaces(a_matrix, b_vector):
    """Normalize rows and keep only the tightest copy of parallel duplicate constraints."""
    normalized = {}
    for normal, offset in zip(a_matrix, b_vector):
        normal_norm = np.linalg.norm(normal)
        if normal_norm <= 1.0e-12:
            continue
        unit_normal = normal / normal_norm
        unit_offset = float(offset / normal_norm)
        key = tuple(np.round(unit_normal, 9))
        if key not in normalized or unit_offset < normalized[key]:
            normalized[key] = unit_offset
    if not normalized:
        return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)
    return np.asarray(list(normalized.keys()), dtype=np.float64), np.asarray(list(normalized.values()), dtype=np.float64)


def halfspace_vertices(a_matrix, b_vector, tolerance=GEOMETRY_TOLERANCE):
    """Return the vertices of a bounded 3-D H-polytope A x <= b."""
    if a_matrix.shape[1] != 3 or a_matrix.shape[0] < 4:
        return np.empty((0, 3), dtype=np.float64)
    combinations = np.asarray(list(itertools.combinations(range(a_matrix.shape[0]), 3)), dtype=np.int64)
    matrices = a_matrix[combinations]
    rhs = b_vector[combinations]
    nonsingular = np.abs(np.linalg.det(matrices)) > 1.0e-10
    if not np.any(nonsingular):
        return np.empty((0, 3), dtype=np.float64)
    candidates = np.linalg.solve(matrices[nonsingular], rhs[nonsingular])
    scale = np.maximum(1.0, np.abs(b_vector))
    feasible = np.all(candidates.dot(a_matrix.T) <= b_vector + tolerance * scale, axis=1)
    candidates = candidates[feasible]
    if not len(candidates):
        return np.empty((0, 3), dtype=np.float64)
    return np.unique(np.round(candidates, decimals=8), axis=0)


def polytope_volume(a_matrix, b_vector, vertices=None):
    """Compute volume by triangulating each outward-oriented polygonal face."""
    vertices = halfspace_vertices(a_matrix, b_vector) if vertices is None else vertices
    if len(vertices) < 4:
        return 0.0
    signed_volume = 0.0
    for normal, offset in zip(a_matrix, b_vector):
        normal_norm = np.linalg.norm(normal)
        if normal_norm <= 1.0e-12:
            continue
        distances = np.abs(vertices.dot(normal) - offset) / normal_norm
        face = vertices[distances <= 1.0e-6]
        if len(face) < 3:
            continue
        center = np.mean(face, axis=0)
        unit_normal = normal / normal_norm
        reference = np.array([1.0, 0.0, 0.0]) if abs(unit_normal[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        axis_u = np.cross(unit_normal, reference)
        axis_u /= np.linalg.norm(axis_u)
        axis_v = np.cross(unit_normal, axis_u)
        relative = face - center
        angles = np.arctan2(relative.dot(axis_v), relative.dot(axis_u))
        ordered = face[np.argsort(angles)]
        for index in range(len(ordered)):
            point_a = ordered[index]
            point_b = ordered[(index + 1) % len(ordered)]
            if np.dot(np.cross(point_a - center, point_b - center), unit_normal) < 0.0:
                point_a, point_b = point_b, point_a
            signed_volume += np.dot(center, np.cross(point_a, point_b)) / 6.0
    return abs(float(signed_volume))


def inscribed_radius_lower_bound(a_matrix, b_vector, vertices, seed=None, random_seed=0, sample_count=4096):
    """Return the best feasible face clearance among deterministic interior samples."""
    if not len(vertices):
        return None
    rng = np.random.RandomState(random_seed)
    # Convex combinations of known vertices are guaranteed to stay inside, so
    # the reported radius is conservative even though it is sampled.
    weights = rng.exponential(size=(sample_count, len(vertices)))
    weights /= np.sum(weights, axis=1, keepdims=True)
    candidates = weights.dot(vertices)
    extras = [np.mean(vertices, axis=0).reshape((1, 3))]
    if seed is not None:
        extras.append(np.asarray(seed, dtype=np.float64).reshape((1, 3)))
    candidates = np.vstack([candidates] + extras)
    norms = np.linalg.norm(a_matrix, axis=1)
    valid = norms > 1.0e-12
    margins = (b_vector[valid] - candidates.dot(a_matrix[valid].T)) / norms[valid]
    feasible_radii = np.min(margins, axis=1)
    return max(0.0, float(np.max(feasible_radii)))


def describe_polytope(a_matrix, b_vector, seed=None, random_seed=0):
    original_facet_count = int(a_matrix.shape[0])
    a_matrix, b_vector = canonicalize_halfspaces(a_matrix, b_vector)
    vertices = halfspace_vertices(a_matrix, b_vector)
    # Only constraints that support an actual face can affect volume or the
    # Chebyshev ball.  Removing the many redundant PathCover halfspaces before
    # the 4-D active-set enumeration changes no geometry and cuts runtime by
    # orders of magnitude for 30--50-row messages.
    if len(vertices):
        norms = np.linalg.norm(a_matrix, axis=1)
        distances = np.abs(vertices.dot(a_matrix.T) - b_vector) / norms
        supporting = np.count_nonzero(distances <= 1.0e-6, axis=0) >= 3
        if np.any(supporting):
            a_matrix = a_matrix[supporting]
            b_vector = b_vector[supporting]
    return {
        "facet_count": original_facet_count,
        "unique_facet_count": int(a_matrix.shape[0]),
        "vertex_count": int(vertices.shape[0]),
        "volume_m3": polytope_volume(a_matrix, b_vector, vertices),
        "inscribed_radius_lower_bound_m": inscribed_radius_lower_bound(
            a_matrix, b_vector, vertices, seed=seed, random_seed=random_seed
        ),
    }


def estimate_overlap(first, second, random_seed, sample_count=8192):
    """Estimate overlap volume and a conservative inscribed-radius lower bound."""
    first_a, first_b, first_seed, first_geometry = first
    second_a, second_b, second_seed, second_geometry = second
    first_vertices = halfspace_vertices(*canonicalize_halfspaces(first_a, first_b))
    second_vertices = halfspace_vertices(*canonicalize_halfspaces(second_a, second_b))
    if not len(first_vertices) or not len(second_vertices):
        return 0.0, 0.0, 0.0
    lower = np.maximum(np.min(first_vertices, axis=0), np.min(second_vertices, axis=0))
    upper = np.minimum(np.max(first_vertices, axis=0), np.max(second_vertices, axis=0))
    extent = upper - lower
    if np.any(extent <= 0.0):
        return 0.0, 0.0, 0.0

    rng = np.random.RandomState(random_seed)
    random_points = lower + rng.random_sample((sample_count, 3)) * extent
    candidates = np.vstack((random_points, first_vertices, second_vertices,
                            first_seed.reshape((1, 3)), second_seed.reshape((1, 3)),
                            ((first_seed + second_seed) / 2.0).reshape((1, 3))))
    first_inside = np.all(candidates.dot(first_a.T) <= first_b + 1.0e-7, axis=1)
    second_inside = np.all(candidates.dot(second_a.T) <= second_b + 1.0e-7, axis=1)
    feasible = first_inside & second_inside
    random_feasible_fraction = float(np.mean(feasible[:sample_count]))
    overlap_volume = float(np.prod(extent) * random_feasible_fraction)
    denominator = min(first_geometry["volume_m3"], second_geometry["volume_m3"])
    overlap_ratio = min(1.0, overlap_volume / denominator) if denominator > 1.0e-12 else 0.0

    if not np.any(feasible):
        return overlap_volume, overlap_ratio, 0.0
    combined_a, combined_b = canonicalize_halfspaces(
        np.vstack((first_a, second_a)), np.concatenate((first_b, second_b))
    )
    norms = np.linalg.norm(combined_a, axis=1)
    margins = (combined_b - candidates[feasible].dot(combined_a.T)) / norms
    radius_lower_bound = max(0.0, float(np.max(np.min(margins, axis=1))))
    return overlap_volume, overlap_ratio, radius_lower_bound


def finite_stats(values):
    return [float(value) for value in values if value is not None and math.isfinite(float(value))]


class PathCoverBenchmarkRunner:
    BAG_TOPICS = [
        "/benchmark/computation_time_ms",
        "/benchmark/remaining_points",
        "/benchmark/voxel_map_sample",
        "/clock",
        "/move_base_simple/goal",
        "/quadrotor/ground_truth",
        "/quadrotor/corridor_adoption",
        "/quadrotor/polytopes",
        "/quadrotor/state",
        "/quadrotor/trajectory",
        "/rosout",
    ]

    def __init__(self):
        self.start_x = float(rospy.get_param("~start_x"))
        self.start_y = float(rospy.get_param("~start_y"))
        self.goal_x = float(rospy.get_param("~goal_x"))
        self.goal_y = float(rospy.get_param("~goal_y"))
        self.goal_z = float(rospy.get_param("~goal_z", 0.75))
        self.goal_yaw = float(rospy.get_param("~goal_yaw", 0.0))
        self.tolerance = float(rospy.get_param("~goal_tolerance", 0.60))
        self.dwell_s = float(rospy.get_param("~arrival_dwell_s", 2.0))
        self.startup_delay_s = float(rospy.get_param("~startup_delay_s", 8.0))
        self.timeout_s = float(rospy.get_param("~timeout_s", 240.0))
        self.corridor_quality_stride = max(1, int(rospy.get_param("~corridor_quality_stride", 40)))
        self.run_id = str(rospy.get_param("~run_id"))
        self.output_root = os.path.abspath(os.path.expanduser(rospy.get_param("~output_dir")))
        self.run_dir = os.path.join(self.output_root, self.run_id)
        os.makedirs(self.run_dir, exist_ok=False)

        self.lock = threading.Lock()
        self.latest_odom = None
        self.last_path_position = None
        self.path_length_m = 0.0
        self.map_ready = False
        self.goal_published = False
        self.goal_wall_time = None
        self.goal_sim_time = None
        self.arrival_since = None
        self.status = "starting"
        self.finalized = False
        self.timings = []
        self.remaining_rows = []
        self.corridor_rows = []
        self.polytope_rows = []
        self.overlap_rows = []
        self.trajectory_rows = []
        self.published_corridors = {}
        self.active_corridor = []
        self.active_corridor_source_stamp_s = None
        self.active_adoption_index = None
        self.adoption_rows = []
        self.pending_adoptions = {}
        self.latest_map_points = None
        self.latest_map_point_count = 0
        self.last_clearance_wall_time = 0.0
        self.corridor_message_count = 0
        self.quality_samples_dropped = 0
        self.quality_queue = queue.Queue(maxsize=4)
        self.quality_worker = threading.Thread(target=self.corridor_quality_worker, daemon=True)
        self.quality_worker.start()

        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1, latch=True)
        rospy.Subscriber("/quadrotor/ground_truth", Odometry, self.odom_cb, queue_size=20)
        rospy.Subscriber("/benchmark/voxel_map_sample", PointCloud2, self.map_cb, queue_size=1)
        rospy.Subscriber("/quadrotor/polytopes", Polytopes, self.polytopes_cb, queue_size=10)
        rospy.Subscriber("/quadrotor/corridor_adoption", PointStamped, self.corridor_adoption_cb, queue_size=20)
        rospy.Subscriber("/benchmark/computation_time_ms", Float64, self.timing_cb, queue_size=1000)
        rospy.Subscriber("/benchmark/remaining_points", Int64MultiArray, self.remaining_cb, queue_size=100)

        self.bag_path = os.path.join(self.run_dir, self.run_id + ".bag")
        self.bag_proc = subprocess.Popen(
            ["rosbag", "record", "-O", self.bag_path] + self.BAG_TOPICS,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
        )
        rospy.on_shutdown(self.on_shutdown)

    def map_cb(self, msg):
        flat = np.fromiter(
            (coordinate for point in point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True
            ) for coordinate in point),
            dtype=np.float64,
        )
        points = flat.reshape((-1, 3)) if flat.size else np.empty((0, 3), dtype=np.float64)
        with self.lock:
            self.latest_map_points = points
            self.latest_map_point_count = int(points.shape[0])
            self.map_ready = True

    def odom_cb(self, msg):
        trajectory_row = None
        with self.lock:
            self.latest_odom = msg
            if not self.goal_published:
                return
            p = msg.pose.pose.position
            current = (p.x, p.y, p.z)
            if self.last_path_position is not None:
                dx = current[0] - self.last_path_position[0]
                dy = current[1] - self.last_path_position[1]
                dz = current[2] - self.last_path_position[2]
                step = math.sqrt(dx * dx + dy * dy + dz * dz)
                if step < 1.0:  # Reject simulator reset/teleport discontinuities.
                    self.path_length_m += step
            self.last_path_position = current

            if self.active_corridor:
                position = np.asarray(current, dtype=np.float64)
                signed_violations = []
                for a_matrix, b_vector in self.active_corridor:
                    norms = np.linalg.norm(a_matrix, axis=1)
                    valid = norms > 1.0e-12
                    if np.any(valid):
                        signed_violations.append(float(np.max(
                            (a_matrix[valid].dot(position) - b_vector[valid]) / norms[valid]
                        )))
                union_violation = min(signed_violations) if signed_violations else None
                contained = union_violation is not None and union_violation <= 1.0e-6
            else:
                union_violation = None
                contained = None

            now = time.time()
            clearance = None
            map_points = self.latest_map_points
            if map_points is not None and len(map_points) and now - self.last_clearance_wall_time >= 0.1:
                self.last_clearance_wall_time = now
                clearance = float(np.sqrt(np.min(np.sum((map_points - np.asarray(current)) ** 2, axis=1))))
            trajectory_row = (
                now, rospy.Time.now().to_sec(), current, contained, union_violation, clearance,
                self.active_corridor_source_stamp_s, self.active_adoption_index,
            )
            self.trajectory_rows.append(trajectory_row)

    def corridor_adoption_cb(self, msg):
        source_key = (int(round(msg.point.x)), int(round(msg.point.y)))
        adoption_sim_time = msg.header.stamp.to_sec()
        start_index = max(0, int(round(msg.point.z)))
        with self.lock:
            if not self.goal_published:
                return
            published = self.published_corridors.get(source_key)
            source_stamp_s = source_key[0] + source_key[1] * 1.0e-9
            event_index = len(self.adoption_rows)
            matched = published is not None
            if matched:
                corridor, source_message_index = published
                self.active_corridor = corridor[start_index:]
                self.active_corridor_source_stamp_s = source_stamp_s
                self.active_adoption_index = event_index
                adopted_count = len(self.active_corridor)
            else:
                source_message_index = None
                adopted_count = None
            self.adoption_rows.append({
                "adoption_index": event_index,
                "source_message_index": source_message_index,
                "source_corridor_stamp_s": source_stamp_s,
                "adoption_sim_time_s": adoption_sim_time,
                "publish_to_adoption_delay_ms": 1000.0 * (adoption_sim_time - source_stamp_s),
                "start_polytope_index": start_index,
                "adopted_polytope_count": adopted_count,
                "matched_source_corridor": matched,
            })
            if not matched:
                # ROS topics from different publishers can be delivered a few
                # milliseconds out of order. Resolve the event when its source
                # corridor callback arrives instead of losing the association.
                self.pending_adoptions[source_key] = event_index

    def timing_cb(self, msg):
        with self.lock:
            if self.goal_published:
                self.timings.append((time.time(), float(msg.data)))

    def remaining_cb(self, msg):
        with self.lock:
            if self.goal_published:
                self.remaining_rows.append((time.time(), list(msg.data)))

    def polytopes_cb(self, msg):
        with self.lock:
            if not self.goal_published:
                return
            source_message_index = self.corridor_message_count
            self.corridor_message_count += 1

        polytopes = []
        invalid_count = 0
        for index, polytope in enumerate(msg.polytope):
            if len(polytope.b) < 4 or len(polytope.A) != 3 * len(polytope.b) or len(polytope.seed) < 3:
                invalid_count += 1
                continue
            a_matrix = np.asarray(polytope.A, dtype=np.float64).reshape((-1, 3))
            b_vector = np.asarray(polytope.b, dtype=np.float64)
            seed = np.asarray(polytope.seed[:3], dtype=np.float64)
            polytopes.append((a_matrix, b_vector, seed))

        corridor = [(item[0], item[1]) for item in polytopes]
        source_key = (int(msg.header.stamp.secs), int(msg.header.stamp.nsecs))
        with self.lock:
            self.published_corridors[source_key] = (corridor, source_message_index)
            pending_index = self.pending_adoptions.pop(source_key, None)
            if pending_index is not None:
                pending = self.adoption_rows[pending_index]
                start_index = pending["start_polytope_index"]
                adopted = corridor[start_index:]
                pending["source_message_index"] = source_message_index
                pending["adopted_polytope_count"] = len(adopted)
                pending["matched_source_corridor"] = True
                if self.active_adoption_index is None or pending_index > self.active_adoption_index:
                    self.active_corridor = adopted
                    self.active_corridor_source_stamp_s = pending["source_corridor_stamp_s"]
                    self.active_adoption_index = pending_index
            # At ~17 Hz this retains roughly the latest minute while bounding memory.
            if len(self.published_corridors) > 1024:
                oldest_key = next(iter(self.published_corridors))
                del self.published_corridors[oldest_key]
        if source_message_index % self.corridor_quality_stride:
            return

        goal = np.asarray(msg.goal[:3], dtype=np.float64) if len(msg.goal) >= 3 else None
        try:
            self.quality_queue.put_nowait((source_message_index, invalid_count, polytopes, goal))
        except queue.Full:
            with self.lock:
                self.quality_samples_dropped += 1

    def corridor_quality_worker(self):
        while True:
            work = self.quality_queue.get()
            try:
                self.compute_corridor_quality(*work)
            except Exception as exc:
                rospy.logerr("Corridor quality worker failed: %s", exc)
            finally:
                self.quality_queue.task_done()

    def compute_corridor_quality(self, source_message_index, invalid_count, polytopes, goal):

        sample = source_message_index // self.corridor_quality_stride
        detailed_polytopes = []
        details = []
        for index, (a_matrix, b_vector, seed) in enumerate(polytopes):
            geometry = describe_polytope(
                a_matrix, b_vector, seed=seed,
                random_seed=source_message_index * 1000 + index,
            )
            detailed_polytopes.append((a_matrix, b_vector, seed, geometry))
            details.append((index, seed, geometry))
        polytopes = detailed_polytopes

        overlaps = []
        for index in range(max(0, len(polytopes) - 1)):
            first = polytopes[index]
            second = polytopes[index + 1]
            volume, ratio, radius_lower_bound = estimate_overlap(
                first, second, source_message_index * 1000 + index
            )
            overlaps.append((index, index + 1, volume, ratio, radius_lower_bound))

        seeds = [item[2] for item in polytopes]
        if goal is not None:
            seeds.append(goal)
        corridor_length = sum(float(np.linalg.norm(second - first)) for first, second in zip(seeds, seeds[1:]))
        volumes = [item[3]["volume_m3"] for item in polytopes]
        radii = [item[3]["inscribed_radius_lower_bound_m"] for item in polytopes]
        facets = [item[3]["facet_count"] for item in polytopes]
        overlap_volumes = [item[2] for item in overlaps]
        overlap_radii = [item[4] for item in overlaps]
        overlap_ratios = [item[3] for item in overlaps]
        row = {
            "sample": sample,
            "source_message_index": source_message_index,
            "wall_time_unix_s": time.time(),
            "polytope_count": len(polytopes),
            "invalid_polytope_count": invalid_count,
            "facet_count_total": sum(facets),
            "facet_count_mean": statistics.mean(facets) if facets else None,
            "sum_polytope_volume_m3": sum(volumes),
            "polytope_volume_mean_m3": statistics.mean(volumes) if volumes else None,
            "polytope_volume_min_m3": min(volumes) if volumes else None,
            "inscribed_radius_lower_bound_mean_m": statistics.mean(finite_stats(radii)) if finite_stats(radii) else None,
            "inscribed_radius_lower_bound_min_m": min(finite_stats(radii)) if finite_stats(radii) else None,
            "overlap_volume_mean_m3": statistics.mean(overlap_volumes) if overlap_volumes else None,
            "overlap_ratio_mean": statistics.mean(finite_stats(overlap_ratios)) if finite_stats(overlap_ratios) else None,
            "overlap_ratio_min": min(finite_stats(overlap_ratios)) if finite_stats(overlap_ratios) else None,
            "overlap_inscribed_radius_lower_bound_mean_m": statistics.mean(finite_stats(overlap_radii)) if finite_stats(overlap_radii) else None,
            "overlap_inscribed_radius_lower_bound_min_m": min(finite_stats(overlap_radii)) if finite_stats(overlap_radii) else None,
            "corridor_centerline_length_m": corridor_length,
        }

        with self.lock:
            if self.finalized:
                return
            self.corridor_rows.append(row)
            for index, seed, geometry in details:
                self.polytope_rows.append((sample, index, seed, geometry))
            for first_index, second_index, volume, ratio, radius_lower_bound in overlaps:
                self.overlap_rows.append((sample, first_index, second_index, volume, ratio, radius_lower_bound))

    @staticmethod
    def percentile(values, q):
        if not values:
            return None
        ordered = sorted(values)
        index = max(0, min(len(ordered) - 1, int(math.ceil(q * len(ordered))) - 1))
        return ordered[index]

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x = self.goal_x
        msg.pose.position.y = self.goal_y
        msg.pose.position.z = self.goal_z
        msg.pose.orientation.z = math.sin(self.goal_yaw / 2.0)
        msg.pose.orientation.w = math.cos(self.goal_yaw / 2.0)

        with self.lock:
            p = self.latest_odom.pose.pose.position
            self.last_path_position = (p.x, p.y, p.z)
            self.path_length_m = 0.0
            self.timings.clear()
            self.remaining_rows.clear()
            self.corridor_rows.clear()
            self.polytope_rows.clear()
            self.overlap_rows.clear()
            self.trajectory_rows.clear()
            self.published_corridors.clear()
            self.active_corridor = []
            self.active_corridor_source_stamp_s = None
            self.active_adoption_index = None
            self.adoption_rows.clear()
            self.pending_adoptions.clear()
            self.last_clearance_wall_time = 0.0
            self.corridor_message_count = 0
            self.quality_samples_dropped = 0
            self.goal_wall_time = time.time()
            self.goal_sim_time = rospy.Time.now().to_sec()
            self.goal_published = True
            self.status = "running"

        for _ in range(3):
            msg.header.stamp = rospy.Time.now()
            self.goal_pub.publish(msg)
            rospy.sleep(0.2)
        rospy.loginfo("Benchmark goal published: (%.3f, %.3f, %.3f)", self.goal_x, self.goal_y, self.goal_z)

    def current_error(self):
        with self.lock:
            if self.latest_odom is None:
                return None
            p = self.latest_odom.pose.pose.position
            dx, dy, dz = p.x - self.goal_x, p.y - self.goal_y, p.z - self.goal_z
            return math.sqrt(dx * dx + dy * dy + dz * dz)

    def write_results(self):
        # Finish the small number of sampled geometry jobs before freezing the
        # report. Raw corridor capture and flight control never wait on them.
        self.quality_queue.join()
        with self.lock:
            if self.finalized:
                return
            self.finalized = True
            timings = list(self.timings)
            remaining_rows = list(self.remaining_rows)
            corridor_rows = list(self.corridor_rows)
            polytope_rows = list(self.polytope_rows)
            overlap_rows = list(self.overlap_rows)
            trajectory_rows = list(self.trajectory_rows)
            adoption_rows = list(self.adoption_rows)
            latest_map_points = self.latest_map_points
            latest_map_point_count = self.latest_map_point_count
            corridor_message_count = self.corridor_message_count
            quality_samples_dropped = self.quality_samples_dropped
            odom = self.latest_odom
            path_length = self.path_length_m
            status = self.status
            goal_wall = self.goal_wall_time
            goal_sim = self.goal_sim_time

        timing_csv = os.path.join(self.run_dir, "pathcover_timing.csv")
        with open(timing_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sample", "wall_time_unix_s", "pathcover_ms"])
            for i, (stamp, value) in enumerate(timings):
                writer.writerow([i, "%.6f" % stamp, "%.9f" % value])

        remaining_csv = os.path.join(self.run_dir, "remaining_points.csv")
        with open(remaining_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sample", "wall_time_unix_s", "iteration", "remaining_points"])
            for sample, (stamp, values) in enumerate(remaining_rows):
                for iteration, value in enumerate(values):
                    writer.writerow([sample, "%.6f" % stamp, iteration, value])

        corridor_csv = os.path.join(self.run_dir, "corridor_quality.csv")
        corridor_fields = [
            "sample", "source_message_index", "wall_time_unix_s", "pathcover_ms", "computation_ms_per_polytope",
            "polytope_count", "invalid_polytope_count", "facet_count_total", "facet_count_mean",
            "sum_polytope_volume_m3", "polytope_volume_mean_m3", "polytope_volume_min_m3",
            "inscribed_radius_lower_bound_mean_m", "inscribed_radius_lower_bound_min_m", "overlap_volume_mean_m3",
            "overlap_ratio_mean", "overlap_ratio_min", "overlap_inscribed_radius_lower_bound_mean_m",
            "overlap_inscribed_radius_lower_bound_min_m", "corridor_centerline_length_m",
        ]
        computation_per_polytope = []
        with open(corridor_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=corridor_fields)
            writer.writeheader()
            for row in corridor_rows:
                output = dict(row)
                timing_index = row["source_message_index"]
                timing = timings[timing_index][1] if timing_index < len(timings) else None
                output["pathcover_ms"] = timing
                if timing is not None and row["polytope_count"]:
                    output["computation_ms_per_polytope"] = timing / row["polytope_count"]
                    computation_per_polytope.append(output["computation_ms_per_polytope"])
                else:
                    output["computation_ms_per_polytope"] = None
                writer.writerow(output)

        polytope_csv = os.path.join(self.run_dir, "polytope_quality.csv")
        with open(polytope_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sample", "polytope_index", "seed_x", "seed_y", "seed_z",
                             "facet_count", "vertex_count", "volume_m3", "inscribed_radius_lower_bound_m"])
            for sample, index, seed, geometry in polytope_rows:
                writer.writerow([sample, index, seed[0], seed[1], seed[2], geometry["facet_count"],
                                 geometry["vertex_count"], geometry["volume_m3"], geometry["inscribed_radius_lower_bound_m"]])

        overlap_csv = os.path.join(self.run_dir, "overlap_quality.csv")
        with open(overlap_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sample", "first_polytope_index", "second_polytope_index",
                             "estimated_overlap_volume_m3", "estimated_overlap_ratio_to_smaller_polytope",
                             "sampled_overlap_inscribed_radius_lower_bound_m"])
            for sample, first_index, second_index, volume, ratio, radius_lower_bound in overlap_rows:
                writer.writerow([sample, first_index, second_index, volume, ratio, radius_lower_bound])

        adoption_csv = os.path.join(self.run_dir, "corridor_adoption.csv")
        adoption_fields = [
            "adoption_index", "source_message_index", "source_corridor_stamp_s",
            "adoption_sim_time_s", "publish_to_adoption_delay_ms",
            "start_polytope_index", "adopted_polytope_count", "matched_source_corridor",
        ]
        with open(adoption_csv, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=adoption_fields)
            writer.writeheader()
            writer.writerows(adoption_rows)

        trajectory_csv = os.path.join(self.run_dir, "trajectory_quality.csv")
        with open(trajectory_csv, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["sample", "wall_time_unix_s", "sim_time_s", "x", "y", "z",
                             "inside_adopted_corridor", "corridor_union_violation_m",
                             "center_to_nearest_voxel_m", "active_corridor_source_stamp_s",
                             "active_adoption_index"])
            for sample, (stamp, sim_stamp, position, contained, violation, clearance,
                         source_stamp, adoption_index) in enumerate(trajectory_rows):
                writer.writerow([sample, stamp, sim_stamp, position[0], position[1], position[2],
                                 contained, violation, clearance, source_stamp, adoption_index])

        voxel_map_file = os.path.join(self.run_dir, "voxel_map_final.npz")
        if latest_map_points is not None:
            np.savez_compressed(voxel_map_file, points_xyz_m=latest_map_points)

        values = [value for _, value in timings]
        polytope_counts = [row["polytope_count"] for row in corridor_rows]
        facet_totals = [row["facet_count_total"] for row in corridor_rows]
        volume_sums = [row["sum_polytope_volume_m3"] for row in corridor_rows]
        min_volumes = finite_stats(row["polytope_volume_min_m3"] for row in corridor_rows)
        min_radii = finite_stats(row["inscribed_radius_lower_bound_min_m"] for row in corridor_rows)
        overlap_ratios = finite_stats(row["overlap_ratio_min"] for row in corridor_rows)
        overlap_radii = finite_stats(row["overlap_inscribed_radius_lower_bound_min_m"] for row in corridor_rows)
        corridor_lengths = [row["corridor_centerline_length_m"] for row in corridor_rows]
        containment = [row[3] for row in trajectory_rows if row[3] is not None]
        violations = finite_stats(row[4] for row in trajectory_rows)
        clearances = finite_stats(row[5] for row in trajectory_rows)
        adoption_delays = finite_stats(row["publish_to_adoption_delay_ms"] for row in adoption_rows)
        final_position = None
        final_error = None
        end_sim = rospy.Time.now().to_sec() if rospy.Time.now() else None
        if odom is not None:
            p = odom.pose.pose.position
            final_position = {"x": p.x, "y": p.y, "z": p.z}
            final_error = math.sqrt(
                (p.x - self.goal_x) ** 2 +
                (p.y - self.goal_y) ** 2 +
                (p.z - self.goal_z) ** 2
            )

        summary = {
            "run_id": self.run_id,
            "status": status,
            "success": status == "success",
            "start": {"x": self.start_x, "y": self.start_y},
            "goal": {"x": self.goal_x, "y": self.goal_y, "z": self.goal_z, "yaw": self.goal_yaw},
            "goal_tolerance_m": self.tolerance,
            "timeout_s": self.timeout_s,
            "wall_duration_s": None if goal_wall is None else time.time() - goal_wall,
            "sim_duration_s": None if goal_sim is None or end_sim is None else end_sim - goal_sim,
            "path_length_m": path_length,
            "final_position": final_position,
            "final_error_m": final_error,
            "pathcover_timing_ms": {
                "count": len(values),
                "mean": statistics.mean(values) if values else None,
                "median": statistics.median(values) if values else None,
                "p95": self.percentile(values, 0.95),
                "max": max(values) if values else None,
                "min": min(values) if values else None,
            },
            "corridor_quality": {
                "snapshot_count": len(corridor_rows),
                "raw_corridor_message_count": corridor_message_count,
                "quality_sampling_stride": self.corridor_quality_stride,
                "quality_samples_dropped": quality_samples_dropped,
                "timing_pair_count": sum(1 for row in corridor_rows if row["source_message_index"] < len(timings)),
                "polytope_count_mean": statistics.mean(polytope_counts) if polytope_counts else None,
                "polytope_count_min": min(polytope_counts) if polytope_counts else None,
                "polytope_count_max": max(polytope_counts) if polytope_counts else None,
                "facet_count_total_mean": statistics.mean(facet_totals) if facet_totals else None,
                "sum_polytope_volume_mean_m3": statistics.mean(volume_sums) if volume_sums else None,
                "minimum_single_polytope_volume_m3": min(min_volumes) if min_volumes else None,
                "minimum_single_polytope_inscribed_radius_lower_bound_m": min(min_radii) if min_radii else None,
                "minimum_consecutive_overlap_ratio": min(overlap_ratios) if overlap_ratios else None,
                "minimum_sampled_overlap_inscribed_radius_lower_bound_m": min(overlap_radii) if overlap_radii else None,
                "corridor_centerline_length_mean_m": statistics.mean(corridor_lengths) if corridor_lengths else None,
                "computation_ms_per_polytope_mean": statistics.mean(computation_per_polytope) if computation_per_polytope else None,
                "invalid_polytope_count": sum(row["invalid_polytope_count"] for row in corridor_rows),
            },
            "corridor_adoption": {
                "event_count": len(adoption_rows),
                "matched_event_count": sum(bool(row["matched_source_corridor"]) for row in adoption_rows),
                "publish_to_adoption_delay_ms_mean": statistics.mean(adoption_delays) if adoption_delays else None,
                "publish_to_adoption_delay_ms_p95": self.percentile(adoption_delays, 0.95),
                "publish_to_adoption_delay_ms_max": max(adoption_delays) if adoption_delays else None,
            },
            "trajectory_quality": {
                "containment_sample_count": len(containment),
                "corridor_containment_rate": (sum(bool(value) for value in containment) / len(containment)) if containment else None,
                "maximum_corridor_union_violation_m": max(violations) if violations else None,
                "clearance_sample_count": len(clearances),
                "minimum_center_to_nearest_voxel_m": min(clearances) if clearances else None,
                "p05_center_to_nearest_voxel_m": self.percentile(clearances, 0.05),
                "voxel_map_final_point_count": latest_map_point_count,
            },
            "quality_method": {
                "polytope_geometry": "Exact 3-D H-polytope vertex enumeration and volume by oriented face triangulation; inscribed radius is a conservative lower bound from 4096 deterministic feasible convex-combination samples plus the seed and vertex centroid.",
                "overlap": "Deterministic 8192-point uniform sampling in the intersected axis-aligned bounds; reported volume/ratio are estimates and reported radius is a feasible lower bound.",
                "geometry_sampling": "Every %d-th corridor message; every raw corridor remains in the rosbag." % self.corridor_quality_stride,
                "trajectory_containment": "Vehicle center tested against the truncated corridor explicitly reported as adopted after a successful trajectory replan.",
                "corridor_adoption": "trajectory_server emits the source corridor stamp and retained start-polytope index only after a replan succeeds; delay is adoption stamp minus source publish stamp.",
                "obstacle_clearance": "Euclidean distance from vehicle center to the nearest point in the latest throttled raw voxel map; this is not body-surface clearance.",
            },
            "artifacts": {
                "rosbag": os.path.basename(self.bag_path),
                "timing_csv": os.path.basename(timing_csv),
                "remaining_points_csv": os.path.basename(remaining_csv),
                "corridor_quality_csv": os.path.basename(corridor_csv),
                "polytope_quality_csv": os.path.basename(polytope_csv),
                "overlap_quality_csv": os.path.basename(overlap_csv),
                "corridor_adoption_csv": os.path.basename(adoption_csv),
                "trajectory_quality_csv": os.path.basename(trajectory_csv),
                "voxel_map_final_npz": os.path.basename(voxel_map_file) if latest_map_points is not None else None,
            },
        }
        with open(os.path.join(self.run_dir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2, sort_keys=True)

    def stop_bag(self):
        if self.bag_proc.poll() is not None:
            return
        self.bag_proc.send_signal(signal.SIGINT)
        try:
            self.bag_proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            self.bag_proc.terminate()
            try:
                self.bag_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.bag_proc.kill()

    def on_shutdown(self):
        if self.status in ("starting", "running"):
            self.status = "interrupted"
        self.stop_bag()
        self.write_results()

    def run(self):
        ready_deadline = time.time() + 90.0
        while not rospy.is_shutdown():
            ready = self.latest_odom is not None and self.map_ready and self.goal_pub.get_num_connections() > 0
            if ready:
                break
            if time.time() >= ready_deadline:
                self.status = "startup_timeout"
                rospy.logerr("Benchmark startup timed out waiting for odometry, voxel map, and goal subscriber")
                rospy.signal_shutdown("benchmark startup timeout")
                return
            rospy.sleep(0.2)

        rospy.loginfo("Benchmark inputs ready; waiting %.1f s before goal", self.startup_delay_s)
        rospy.sleep(self.startup_delay_s)
        if rospy.is_shutdown():
            return
        self.publish_goal()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            elapsed = time.time() - self.goal_wall_time
            error = self.current_error()
            if error is not None and error <= self.tolerance:
                if self.arrival_since is None:
                    self.arrival_since = time.time()
                elif time.time() - self.arrival_since >= self.dwell_s:
                    self.status = "success"
                    rospy.loginfo("Benchmark succeeded in %.3f s (error %.3f m)", elapsed, error)
                    self.stop_bag()
                    self.write_results()
                    rospy.signal_shutdown("benchmark complete")
                    return
            else:
                self.arrival_since = None

            if elapsed >= self.timeout_s:
                self.status = "timeout"
                rospy.logerr("Benchmark timed out after %.3f s", elapsed)
                self.stop_bag()
                self.write_results()
                rospy.signal_shutdown("benchmark timeout")
                return
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("pathcover_benchmark_runner")
    PathCoverBenchmarkRunner().run()
