"""
Custom markers for rviz.
"""

import copy
import rclpy
from rclpy.node import Node
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class FootballMarkerPublisher(Node):
    """Marker publisher for displaying the target pose."""

    def __init__(self):
        super().__init__("football_marker_publisher")
        self.publisher_ = self.create_publisher(Marker, "/football_marker_topic", 10)

    def publish_marker(self, pose_goal):
        """Publish a marker using a football mesh.
        Args:
            pose_goal: PoseStamped() of the marker
        """
        marker = Marker()
        marker.id = 0
        marker.header.frame_id = "base_link"
        marker.ns = "football_marker"
        marker.pose = pose_goal.pose
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://demo/meshes/football.dae"
        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
        marker.color.a = 1.0
        marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()

        self.publisher_.publish(marker)


class FootballMarkerArrayPublisher(Node):
    """MarkerArray publisher for displaying multiple target poses with footballs.
    This implementation slows down rviz in case thousands of poses are displayed."""

    def __init__(self):
        super().__init__("football_marker_array_publisher")
        self.publisher_ = self.create_publisher(
            MarkerArray, "/football_marker_array_topic", 10
        )
        self.marker_array = MarkerArray()
        self.id_ = 0

    def add_marker(self, pose_goal, color=""):
        """Add marker to the MarkerArray and publish it.
        Args:
            pose_goal: PoseStamped() of the marker
            color: empty string (g=1, r=b=0) or "red" (r=1, b=g=0)
        """
        marker = Marker()
        marker.id = self.id_
        self.id_ = self.id_ + 1
        marker.header.frame_id = "base_link"
        marker.ns = "football_markers"
        # deep copy pose (otherwise only the last one will be published)
        marker.pose = copy.deepcopy(pose_goal.pose)
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        if color == "red":
            marker.color.r, marker.color.g = 1.0, 0.0
        marker.action = Marker.ADD
        marker.type = Marker.MESH_RESOURCE
        marker.mesh_use_embedded_materials = True
        marker.mesh_resource = "package://demo/meshes/football.dae"
        # marker.frame_locked = True
        marker.scale.x, marker.scale.y, marker.scale.z = 0.1, 0.1, 0.1
        marker.lifetime = rclpy.duration.Duration(seconds=100).to_msg()

        self.marker_array.markers.append(marker)
        self.publisher_.publish(self.marker_array)

    def publish_marker(self):
        """Publish MarkerArray explicitly."""
        self.publisher_.publish(self.marker_array)


class PointsPublisher(Node):
    """PointMarker publisher for displaying the multiple points.
    This visualization comes handy in case thousands of points
    are displayed (it will not slow down rviz).
    """

    def __init__(self):
        super().__init__("points_marker_publisher")
        self.publisher_ = self.create_publisher(Marker, "/points_marker_topic", 10)
        self.marker = Marker()
        self.marker.id = 0
        self.marker.header.frame_id = "base_link"
        self.marker.ns = "points_marker"
        self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = 0.01, 0.01, 0.01
        self.marker.lifetime = rclpy.duration.Duration(seconds=0).to_msg()
        self.marker.action = Marker.ADD
        self.marker.type = Marker.POINTS

    def add_point(self, pose_goal, color=""):
        """Publish a simple point.
        Args:
            pose_goal: PoseStamped() of the marker
            color: empty string (g=1, r=b=0) or "red" (r=1, b=g=0)
        """
        point = Point()
        # deep copy pose (otherwise only the last one will be published)
        point = copy.deepcopy(pose_goal.pose.position)
        self.marker.points.append(point)
        point_color = ColorRGBA()
        point_color.a = 1.0
        point_color.r, point_color.g, point_color.b = 0.0, 1.0, 0.0
        if color == "red":
            point_color.r, point_color.g = 1.0, 0.0
        self.marker.colors.append(point_color)
        self.publish_marker()

    def publish_marker(self):
        """Publish Marker explicitly."""
        self.publisher_.publish(self.marker)
