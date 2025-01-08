#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from math import atan2, cos, sin, pi
from geometry_msgs.msg import Point

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.filtered_publisher = self.create_publisher(
            LaserScan, '/filtered_scan', 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/visualization_marker_array', 10
        )
        self.get_logger().info("Lidar Processing Node Started")

        # Parameters
        self.max_range = 5 # max range of the lidar. Point with larger range are discarded
        self.dbscan_eps = 0.12
        self.dbscan_min_samples = 2
        self.Tmax = 0.05
        self.line_point_threshold = 10

    def scan_callback(self, msg):
        # Convert LaserScan to Cartesian Points
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        distances = np.array(msg.ranges)
        valid_indices = np.isfinite(distances) and distances <= self.max_range
        valid_angles = angles[valid_indices]
        valid_distances = distances[valid_indices]
        x = valid_distances * np.cos(valid_angles)
        y = valid_distances * np.sin(valid_angles)
        points = np.column_stack((x, y))

        # Apply DBSCAN
        clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples).fit(points)
        labels = clustering.labels_

        # Process each cluster
        unique_labels = set(labels)
        cluster = {} # Create a dictionary with key being the cluster label and value is the numpy array of points in cluster
        for label in unique_labels:
            if label == -1:
                continue
            cluster[label]=points[labels == label]

        filtered_lines = []  # A list of pairs of lines and their cluster labels
        for label in cluster.keys():
            cluster_points = cluster[label]
            lines, line_indices = self.line_tracking(cluster_points, self.Tmax)
            lines, line_indices = self.backtracking(cluster_points, lines, line_indices, self.Tmax)
            
            # Append each line with its cluster label
            for line in lines:
                filtered_lines.append((line, label))

        # Classify outliers
        outliers = points[labels == -1]
        if len(filtered_lines) >0:
            remaining_outliers = []  
            for point in outliers:
                is_inlier = False
                for line, label in filtered_lines:
                    if self.point_to_line_distance(point, line) < self.Tmax:
                        cluster[label] = np.vstack((cluster[label], point))
                        is_inlier = True
                        break  # No need to check further lines
                if not is_inlier:
                    remaining_outliers.append(point)
            outliers = remaining_outliers 

        
        # Visualize clusters, lines, and outliers
        self.visualize_clusters(cluster, filtered_lines, outliers, "base_scan")

    def line_tracking(self, points, Tmax):
        i, j, l = 0, 0, 0
        lines = []
        line_indices = []

        while j < len(points) - 2:
            line = self.fit_line(points[i:j + 2])
            T = self.point_to_line_distance(points[j + 2], line)

            if T > Tmax:
                lines.append(line)
                line_indices.append((i, j + 1))
                l += 1
                i = j + 2
                j = i
            else:
                if j + 2 == len(points) - 1: # Handle the last line if last point belong to current considered line
                    lines.append(line)
                    line_indices.append((i, j + 2))
                j += 1

        return lines, line_indices

    def backtracking(self, points, lines, line_indices, Tmax):
        for i in range(len(lines) - 1):
            s1, e1 = line_indices[i]
            s2, e2 = line_indices[i + 1]

            # Emulate do-while loop
            while True:
                # Compute the distance from the endpoint of the first line to the second line
                d2 = self.point_to_line_distance(points[e1], lines[i + 1])

                # Compute the line without including the last point if possible
                if e1 - s1 + 1 > 2:
                    line = self.fit_line(points[s1:e1])
                    d1 = self.point_to_line_distance(points[e1], line)
                else:
                    d1 = Tmax

                # Adjust indexes and recompute lines if d2 is smaller
                if d2 < d1:
                    e1 -= 1
                    s2 -= 1
                    line_indices[i] = (s1, e1)
                    line_indices[i + 1] = (s2, e2)
                    lines[i] = self.fit_line(points[s1:e1 + 1])
                    lines[i + 1] = self.fit_line(points[s2:e2 + 1])
                else:
                    break  # Exit the loop if the condition is not met

                # Terminate the loop if `s1` equals `e1`
                if s1 == e1:
                    break

        # Remove lines with fewer points than threshold
        lines = [line for idx, line in enumerate(lines) if line_indices[idx][1] - line_indices[idx][0] + 1 > self.line_point_threshold]
        line_indices = [indices for indices in line_indices if indices[1] - indices[0] + 1 > self.line_point_threshold]

        return lines, line_indices

    def fit_line(self, points):
        x_bar = np.mean(points[:, 0])
        y_bar = np.mean(points[:, 1])
        Sxx = np.sum((points[:, 0] - x_bar) ** 2)
        Syy = np.sum((points[:, 1] - y_bar) ** 2)
        Sxy = np.sum((points[:, 0] - x_bar) * (points[:, 1] - y_bar))

        theta = 0.5 * atan2(-2 * Sxy, Syy - Sxx + 1e-6)
        r = x_bar * cos(theta) + y_bar * sin(theta)

        # if r < 0:
        #     theta = theta + pi
        #     r = -r
        return r, theta

    def point_to_line_distance(self, point, line):
        r, theta = line
        x, y = point
        return abs(r - x * cos(theta) - y * sin(theta))

    def visualize_points(self, points, color, marker_array, marker_id, frame_id):
        for x, y in points:
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.0
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0
            marker_array.markers.append(marker)
            marker_id += 1
        return marker_id

    def visualize_line(self, line, color, marker_array, marker_id, frame_id):
        r, theta = line
        x1 = r * cos(theta) - 10 * sin(theta)
        y1 = r * sin(theta) + 10 * cos(theta)
        x2 = r * cos(theta) + 10 * sin(theta)
        y2 = r * sin(theta) - 10 * cos(theta)

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0

        marker.points.append(Point(x=x1, y=y1, z=0.))
        marker.points.append(Point(x=x2, y=y2, z=0.))
        marker_array.markers.append(marker)
        return marker_id + 1

    def visualize_clusters(self, cluster, filtered_lines, outliers, frame_id):
        marker_array = MarkerArray()
        colors = plt.cm.get_cmap('Set2', len(cluster))
        marker_id = 0

        for label in cluster.keys():
            color = colors(label)[:3]
            # Visualize cluster point
            marker_id = self.visualize_points(cluster[label], color, marker_array, marker_id, frame_id)
            # Visualize cluster line
            # for line, cluster_id in filtered_lines:
            #     if cluster_id == label:
            #         marker_id = self.visualize_line(line, color, marker_array, marker_id, frame_id)
        # Visualize outliers in red
        marker_id = self.visualize_points(outliers, (1.0, 0.0, 0.0), marker_array, marker_id, frame_id)
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()