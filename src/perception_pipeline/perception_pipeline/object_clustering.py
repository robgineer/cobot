#!/usr/bin/env python3
"""
Simple object detection based on clustering using a point cloud.

Identifies objects (as clusters) on the workbench and
publishes solid boxes for each cluster (to be displayed in rviz).

Run this node with an active PointCloud2 provided by Gazebo RGDB or RealSense.
For RealSense: update the pointcloud_topic.
"""

import rclpy
import numpy as np

from rclpy.node import Node
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import PointCloud2  # message
from sensor_msgs_py import point_cloud2  # helpers
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, TransformStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from scipy.spatial.transform import Rotation as rotation_helper


class WorkbenchObjectDetector(Node):
    def __init__(self):
        super().__init__("workbench_object_clustering")

        self.pointcloud_topic = (
            "/world/default/model/depth_camera/link/camera_link/"
            "sensor/depth_camera_sensor/points"
        )
        self.subscriber = self.create_subscription(
            PointCloud2, self.pointcloud_topic, self.cloud_callback, 1
        )
        # use PlanningScene publisher from MoveIt
        # => published objects will represent obstacles in rviz
        self.scene_publisher = self.create_publisher(
            PlanningScene, "/planning_scene", 5
        )

        # initialize transform
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # workbench coordinates (the region of interest)
        self.x_min, self.x_max = 0.28, 0.55
        self.y_min, self.y_max = -0.5, 0.0
        self.z_min, self.z_max = 0.75, 0.85

        # the expected object size
        self.min_cluster_size = 100
        self.max_cluster_size = 1000

        self.R = None
        self.t = None

        self.get_logger().info("Workbench object detector started")

    def cloud_callback(self, msg):
        """
        Callback for the reception of a new message from the point cloud topic.

        Handles point preprocessing, cluster creation and publishing of boxes
        representing obstacles in the planning_scene.

        Args:
            msg: the PointCloud2 message
        """
        # get the transformation
        # since the camera has its fixed position, we only need it once
        if self.R is None or self.t is None:
            self.get_camera_to_world_transformation(msg)
        # get the point cloud
        processed_point_cloud = self.preprocess_point_cloud(msg)
        # handle potential issues with the preprocessing
        # => do nothing in case an error was present
        if processed_point_cloud is None:
            return

        # create the clusters and store the labels
        dbscan_cluster = DBSCAN(eps=0.02, min_samples=30, n_jobs=-1).fit(
            processed_point_cloud
        )
        cluster_ids = dbscan_cluster.labels_
        unique_cluster_ids = np.unique(cluster_ids)

        # publish the clusters into the planning scene
        self.publish_clusters(processed_point_cloud, cluster_ids, unique_cluster_ids)

    def publish_clusters(self, cloud, cluster_ids, unique_cluster_ids):
        """
        Publishes an object into the MoveIt2 planning scene.

        Args:
            cloud: the processed PointCloud2 as numpy array
            cluster_ids: cluster ids (representing indexes of the cloud array)
            unique_cluster_ids: unique cluster ids
        """

        scene = PlanningScene()

        # iterate trough all clusters
        for cluster_id in unique_cluster_ids:
            if cluster_id == -1:  # this is noise => skip
                continue
            # get all points of the current cluster
            cluster_points = cloud[cluster_ids == cluster_id]
            # filter clusters that are too small (noise) or too big (links of cobot)
            if (
                cluster_points.shape[0] < self.min_cluster_size
                or cluster_points.shape[0] > self.max_cluster_size
            ):
                continue

            # get the cluster dimensions
            min_point = cluster_points.min(axis=0)
            max_point = cluster_points.max(axis=0)
            center_point_coordinates = (min_point + max_point) / 2.0
            object_size = max_point - min_point

            # create MoveIt obstacle
            obstacle = CollisionObject()
            obstacle.header.frame_id = "world"
            obstacle.id = f"workbench_obj_{cluster_id}"
            # define obstacle form (type and size)
            obstacle_form = SolidPrimitive()
            obstacle_form.type = SolidPrimitive.BOX
            obstacle_form.dimensions = [
                float(object_size[0]),
                float(object_size[1]),
                float(object_size[2]),
            ]
            obstacle.primitives = [obstacle_form]
            # define obstacle pose
            # => place obstacle at center of point cloud with neutral orientation
            obstacle_pose = Pose()
            obstacle_pose.position.x = float(center_point_coordinates[0])
            obstacle_pose.position.y = float(center_point_coordinates[1])
            obstacle_pose.position.z = float(center_point_coordinates[2])
            obstacle_pose.orientation.w = 1.0
            obstacle.primitive_poses = [obstacle_pose]

            obstacle.operation = CollisionObject.ADD
            scene.world.collision_objects.append(obstacle)

        if len(scene.world.collision_objects) > 0:
            self.scene_publisher.publish(scene)
            self.get_logger().info(
                f"Published {len(scene.world.collision_objects)} objects to MoveIt"
            )
        else:
            self.get_logger().info("No objects detected.")

    def get_camera_to_world_transformation(self, msg):
        """
        Identifies the transformation (R, t) from camera to world frame and stores
        the values into the corresponding member variables.

        This is required due to the fact that the point cloud is published
        in camera frame.

        Args:
        msg: the PointCloud2 message
        """
        # get the transform from the camera to the world frame
        try:
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                "world",
                msg.header.frame_id,
                rclpy.time.Time(),
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return

        # get rotation matrix R and translation vector t
        rot = rotation_helper.from_quat(
            [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ]
        )
        self.R = rot.as_matrix()
        self.t = np.array(
            [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z,
            ]
        )

    def preprocess_point_cloud(self, msg) -> np.array:
        """
        1. Extracts the x,y,z points from the point cloud and stores them into a numpy array.
        2. Transforms the points from camera to world frame
        3. Crops the point cloud to contain only points within the region of interest
        4. Checks for inconsistencies (empty point cloud / issues with transformation)

        Args:
        msg: the PointCloud2 message

        Returns:
            preprocessed point cloud as numpy array or None in case of processing errors
        """
        # check if transformation is available
        if self.R is None or self.t is None:
            self.get_logger().warn(
                "Do not have a valid transformation. Retrying on next callback ..."
            )
            return None
        # get points from point cloud (drop rgb and use only x,y,z values)
        points = []
        for point in point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True
        ):
            x, y, z = point
            points.append([x, y, z])
        # interpret point list as numpy array for further processing
        points = np.array(points)

        # check if points are empty
        # this could happen within the first callback
        if len(points) == 0:
            self.get_logger().warn(
                "There are no points in the point cloud. Retrying on next callback ..."
            )
            return None

        # transform points to world frame
        points_in_world_frame = (
            self.R @ points.T
        ).T + self.t  # note: could result in several NaN values

        # check if point cloud consists of NaN values only
        # this could happen within the first callback
        mask = np.isfinite(points_in_world_frame).all(axis=1)
        points_in_world_frame = points_in_world_frame[mask]
        if points_in_world_frame.shape[0] == 0:
            self.get_logger().warn(
                "Points are invalid after filtering. Retrying on next callback ..."
            )
            return None

        # crop points to region of interest (workbench) only
        roi = (
            (points_in_world_frame[:, 0] >= self.x_min)
            & (points_in_world_frame[:, 0] <= self.x_max)
            & (points_in_world_frame[:, 1] >= self.y_min)
            & (points_in_world_frame[:, 1] <= self.y_max)
            & (points_in_world_frame[:, 2] >= self.z_min)
            & (points_in_world_frame[:, 2] <= self.z_max)
        )
        points_in_world_frame_cropped = points_in_world_frame[roi]

        # if cropping removed all points, do nothing
        if points_in_world_frame_cropped.shape[0] == 0:
            self.get_logger().warn(
                "Points are invalid after cropping. Either the point cloud is not published or the ROI is too narrow."
            )
            return None

        # preprocessing successful, return the numpy array representing the point cloud
        return points_in_world_frame_cropped


def main(args=None):
    rclpy.init(args=args)
    node = WorkbenchObjectDetector()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
