"""
Custom markers for rviz.
"""

import copy
import rclpy
from rclpy.node import Node
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
    """MarkerArray publisher for displaying multiple target poses."""

    def __init__(self):
        super().__init__("football_marker_array_publisher")
        self.publisher_ = self.create_publisher(
            MarkerArray, "/football_marker_array_topic", 10
        )
        self.marker_array = MarkerArray()
        self.id_ = 0

    def add_marker(self, pose, color=""):
        """Add marker to the MarkerArray and publish it.
        Args:
            pose: the pose of the marker
            color: empty string (r=g=b=0) or "red" (r=1)
        """
        marker = Marker()
        marker.id = self.id_
        self.id_ = self.id_ + 1
        marker.header.frame_id = "base_link"
        marker.ns = "football_markers"
        marker.pose = copy.deepcopy(pose.pose)  # this is required
        marker.color.a = 1.0
        marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 0.0
        if color == "red":
            marker.color.r = 1.0
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
