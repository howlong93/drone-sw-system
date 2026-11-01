#!/usr/bin/env python3

import itertools
import math

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from polytope_msgs.msg import Polytopes
from visualization_msgs.msg import Marker, MarkerArray


def halfspace_vertices(a_matrix, b_vector):
    """Enumerate vertices of a bounded 3-D polytope A x <= b."""
    if a_matrix.shape[0] < 4:
        return np.empty((0, 3), dtype=np.float64), np.empty((0, 0), dtype=bool)
    triples = np.asarray(list(itertools.combinations(range(a_matrix.shape[0]), 3)), dtype=np.int64)
    matrices = a_matrix[triples]
    rhs = b_vector[triples]
    nonsingular = np.abs(np.linalg.det(matrices)) > 1.0e-10
    if not np.any(nonsingular):
        return np.empty((0, 3), dtype=np.float64), np.empty((0, a_matrix.shape[0]), dtype=bool)
    candidates = np.linalg.solve(matrices[nonsingular], rhs[nonsingular])
    feasible = np.all(candidates.dot(a_matrix.T) <= b_vector + 1.0e-6, axis=1)
    vertices = np.unique(np.round(candidates[feasible], 7), axis=0)
    if not len(vertices):
        return vertices, np.empty((0, a_matrix.shape[0]), dtype=bool)
    norms = np.linalg.norm(a_matrix, axis=1)
    active = np.abs(vertices.dot(a_matrix.T) - b_vector) <= 2.0e-5 * np.maximum(1.0, norms)
    return vertices, active


class ReplayVisualizer:
    def __init__(self):
        self.frame = rospy.get_param("~frame_id", "world")
        self.path_min_distance = float(rospy.get_param("~path_min_distance", 0.03))
        self.path_max_interval = float(rospy.get_param("~path_max_interval", 0.10))
        self.vehicle_period = 1.0 / float(rospy.get_param("~vehicle_rate_hz", 50.0))
        self.corridor_stride = max(1, int(rospy.get_param("~corridor_stride", 3)))

        self.path_pub = rospy.Publisher("/replay/flight_path", Path, queue_size=1, latch=True)
        self.vehicle_pub = rospy.Publisher("/replay/vehicle", Marker, queue_size=1)
        self.start_pub = rospy.Publisher("/replay/start", Marker, queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher("/replay/goal", Marker, queue_size=1, latch=True)
        self.corridor_pub = rospy.Publisher("/replay/corridor", MarkerArray, queue_size=1)

        self.path = Path()
        self.path.header.frame_id = self.frame
        self.last_path_position = None
        self.last_path_stamp = None
        self.last_vehicle_stamp = None
        self.odom_count = 0
        self.path_point_count = 0
        self.corridor_count = 0
        self.goal_count = 0

        rospy.Subscriber("/quadrotor/ground_truth", Odometry, self.odom_cb, queue_size=100)
        rospy.Subscriber("/quadrotor/polytopes", Polytopes, self.corridor_cb, queue_size=3)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=3)
        rospy.on_shutdown(self.on_shutdown)

    @staticmethod
    def safe_publish(publisher, message):
        if rospy.is_shutdown():
            return
        try:
            publisher.publish(message)
        except rospy.ROSException:
            # rosbag is a required finite node, so roslaunch can close publishers
            # while a final high-rate odometry callback is still returning.
            pass

    def make_marker(self, namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self.frame
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def reset_for_time_jump(self):
        self.path = Path()
        self.path.header.frame_id = self.frame
        self.last_path_position = None
        self.last_path_stamp = None
        delete = MarkerArray()
        marker = self.make_marker("replay", 0, Marker.CUBE, rospy.Time.now())
        marker.action = Marker.DELETEALL
        delete.markers.append(marker)
        self.safe_publish(self.corridor_pub, delete)

    def odom_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        if self.last_path_stamp is not None and stamp < self.last_path_stamp:
            self.reset_for_time_jump()

        self.odom_count += 1
        position = msg.pose.pose.position
        xyz = np.asarray([position.x, position.y, position.z], dtype=np.float64)
        add_path_point = self.last_path_position is None
        if self.last_path_position is not None:
            distance = float(np.linalg.norm(xyz - self.last_path_position))
            elapsed = (stamp - self.last_path_stamp).to_sec()
            add_path_point = distance >= self.path_min_distance or elapsed >= self.path_max_interval

        if add_path_point:
            pose = PoseStamped()
            pose.header.frame_id = self.frame
            pose.header.stamp = stamp
            pose.pose = msg.pose.pose
            self.path.header.stamp = stamp
            self.path.poses.append(pose)
            self.safe_publish(self.path_pub, self.path)
            self.last_path_position = xyz
            self.last_path_stamp = stamp
            self.path_point_count += 1
            if self.path_point_count == 1:
                start = self.make_marker("replay_start", 0, Marker.SPHERE, stamp)
                start.pose.position = position
                start.scale.x = start.scale.y = start.scale.z = 0.45
                start.color.r, start.color.g, start.color.b, start.color.a = 0.1, 0.3, 1.0, 1.0
                self.safe_publish(self.start_pub, start)

        if self.last_vehicle_stamp is None or (stamp - self.last_vehicle_stamp).to_sec() >= self.vehicle_period:
            vehicle = self.make_marker("replay_vehicle", 0, Marker.MESH_RESOURCE, stamp)
            vehicle.pose = msg.pose.pose
            vehicle.mesh_resource = "package://quadrotor_description/meshes/f250.dae"
            vehicle.mesh_use_embedded_materials = True
            vehicle.scale.x = vehicle.scale.y = vehicle.scale.z = 0.8
            self.safe_publish(self.vehicle_pub, vehicle)
            self.last_vehicle_stamp = stamp

    def goal_cb(self, msg):
        self.goal_count += 1
        marker = self.make_marker("replay_goal", 0, Marker.SPHERE, msg.header.stamp)
        marker.pose = msg.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.55
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.1, 0.1, 1.0
        self.safe_publish(self.goal_pub, marker)

    def corridor_cb(self, msg):
        self.corridor_count += 1
        if (self.corridor_count - 1) % self.corridor_stride:
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        edges = self.make_marker("corridor_edges", 0, Marker.LINE_LIST, stamp)
        edges.scale.x = 0.035
        edges.color.r, edges.color.g, edges.color.b, edges.color.a = 0.0, 0.75, 1.0, 0.45
        seeds = self.make_marker("corridor_seeds", 1, Marker.SPHERE_LIST, stamp)
        seeds.scale.x = seeds.scale.y = seeds.scale.z = 0.16
        seeds.color.r, seeds.color.g, seeds.color.b, seeds.color.a = 1.0, 0.75, 0.0, 0.9
        centerline = self.make_marker("corridor_centerline", 2, Marker.LINE_STRIP, stamp)
        centerline.scale.x = 0.06
        centerline.color.r, centerline.color.g, centerline.color.b, centerline.color.a = 1.0, 0.55, 0.0, 0.85

        for poly in msg.polytope:
            if len(poly.seed) >= 3:
                point = Point(float(poly.seed[0]), float(poly.seed[1]), float(poly.seed[2]))
                seeds.points.append(point)
                centerline.points.append(point)
            if len(poly.b) < 4 or len(poly.A) != 3 * len(poly.b):
                continue
            a_matrix = np.asarray(poly.A, dtype=np.float64).reshape((-1, 3))
            b_vector = np.asarray(poly.b, dtype=np.float64)
            vertices, active = halfspace_vertices(a_matrix, b_vector)
            for first, second in itertools.combinations(range(len(vertices)), 2):
                if np.count_nonzero(active[first] & active[second]) >= 2:
                    edges.points.append(Point(*vertices[first]))
                    edges.points.append(Point(*vertices[second]))

        markers = MarkerArray()
        clear = self.make_marker("replay", 0, Marker.CUBE, stamp)
        clear.action = Marker.DELETEALL
        markers.markers = [clear, edges, seeds, centerline]
        self.safe_publish(self.corridor_pub, markers)

    def on_shutdown(self):
        rospy.loginfo(
            "Replay visualizer summary: odometry=%d path_points=%d corridors=%d goals=%d",
            self.odom_count, self.path_point_count, self.corridor_count, self.goal_count,
        )


if __name__ == "__main__":
    rospy.init_node("pathcover_replay_visualizer")
    ReplayVisualizer()
    rospy.spin()
