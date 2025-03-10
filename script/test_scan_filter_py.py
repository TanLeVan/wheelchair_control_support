#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from scipy.optimize import minimize
import matplotlib.pyplot as plt
from math import atan2, cos, sin, pi
from scipy.ndimage import maximum_filter
import seaborn as sns

"""
Process flow:
- Subsampling of scan point (supsampling_rate)
- Hough transform + non-max local suppressor filter. Parameters:
    - Number of rho bin
    - Number of theta bin
    - size of filter
    - Stride of filter
- Group line:
    - rho threshold
    - theta threshold
"""

class HoughFilterNode(Node):
    def __init__(self):
        super().__init__('line_tracking_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)

        # Parameter declaration
        self.get_logger().info("Line Tracking Node Started")
        self.subsampling_rate = 4
        self.hough_theta_bin = 45
        self.hough_rho_bin = 45
        self.hough_threshold = 0.6

        self.filter_size = 10
        self.filter_stride = 10 
        self.rho_threshold = 0.3
        self.theta_threshold = np.deg2rad(20)

    def scan_callback(self, msg):
        self.msg_frame = msg.header.frame_id 
        # Convert to Cartesian coordinates
        self.hough_filter(msg)

    def hough_filter(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        distances = np.array(msg.ranges)
        valid_indices = np.isfinite(distances)
        x = distances[valid_indices] * np.cos(angles[valid_indices])
        y = distances[valid_indices] * np.sin(angles[valid_indices])
        points = np.column_stack((x, y))
        # Subsample the points
        subsampled_points = points[::self.subsampling_rate]
        # Step 1: Detect lines using sparse Hough accumulator
        detected_line = self.hough_transform(subsampled_points, self.hough_theta_bin, self.hough_rho_bin, threshold=self.hough_threshold)
        # Step 2: Refine lines using inlier points
        grouped_lines = self.group_lines(detected_line, self.rho_threshold, self.theta_threshold)
        fitted_lines = self.fit_line_to_group(grouped_lines)
        # print([fitted_lines[i][0] for i in range(len(fitted_lines))])
        print(len(detected_line))
        print("/n")
        # # Visualization
        self.visualize_grouped_lines(grouped_lines)
        # self.visualize_lines(fitted_lines)


    def custom_local_maximum_filter(self, array, size, stride):
        """
        Custom implementation of a local maximum filter with customizable stride.
        Keeps only the local maxima and sets others to 0.

        Args:
            array (numpy.ndarray): Input 2D array to process.
            size (int): Size of the local region (e.g., 3 for a 3x3 window).
            stride (int): Step size for moving the filter window.

        Returns:
            numpy.ndarray: Processed array where only local maxima are retained.
        """
        output = np.zeros_like(array)  # Initialize the output array with zeros

        # Iterate over each element in the array with the given stride
        for i in range(0, array.shape[0], stride):
            for j in range(0, array.shape[1], stride):
                # Ensure the window doesn't exceed array bounds
                end_i = min(i + size, array.shape[0])
                end_j = min(j + size, array.shape[1])

                # Extract the local region (window)
                window = array[i:end_i, j:end_j]

                # Skip empty windows
                if window.size == 0:
                    continue

                # Get the maximum value in the window
                local_max = np.max(window)

                # Get the position (indices) of the maximum value in the window
                max_position = np.unravel_index(np.argmax(window), window.shape)

                # Set the maximum value in the output array
                output[i + max_position[0], j + max_position[1]] = local_max

        return output

    def hough_transform(self, points, theta_bins, rho_bins, threshold):
        # Compute the diagonal of the points for rho range
        diagonal = np.sqrt(np.max(points[:, 0])**2 + np.max(points[:, 1])**2)
        thetas = np.linspace(-np.pi/2, np.pi/2, theta_bins)
        rhos = np.linspace(-diagonal, diagonal, rho_bins)

        # Initialize the accumulator as a 2D array of lists
        accumulator_counts = np.zeros((len(rhos), len(thetas)), dtype=int)
        accumulator = [[[] for _ in range(len(thetas))] for _ in range(len(rhos))]

        # Populate the accumulator with points
        for x, y in points:
            for theta_idx, theta in enumerate(thetas):
                rho = x * np.cos(theta) + y * np.sin(theta)
                rho_idx = np.argmin(np.abs(rhos - rho))
                accumulator[rho_idx][theta_idx].append((x, y))
                accumulator_counts[rho_idx, theta_idx] += 1

        filtered_counts = self.custom_local_maximum_filter(accumulator_counts, size=self.filter_size, stride=self.filter_stride)
        # Detect lines with votes above threshold and pack corresponding points
        if threshold > 0 and threshold <=1:
            threshold = np.max(filtered_counts)*threshold
        detected_lines = []
        for rho_idx in range(len(rhos)):
            for theta_idx in range(len(thetas)):
                if  filtered_counts[rho_idx, theta_idx] > threshold:  # Check if the number of points exceeds threshold
                    line = (rhos[rho_idx], thetas[theta_idx])  # Line parameters
                    line_points = accumulator[rho_idx][theta_idx]  # Points contributing to this line
                    detected_lines.append((line, line_points))

        self.visualize_hough_space(accumulator_counts, thetas, rhos)
        return detected_lines
    
    def visualize_hough_space(self, accumulator_counts, thetas, rhos):
        """
        Visualize the Hough transform parameter space. The plot updates dynamically.

        Args:
            accumulator_counts: 2D array of counts in the accumulator.
            thetas: Array of theta values used in the transform.
            rhos: Array of rho values used in the transform.
        """
        filtered_counts = self.custom_local_maximum_filter(accumulator_counts, size=10, stride=10)

        # Dynamic figure setup
        if not hasattr(self, 'fig'):
            self.fig, self.axes = plt.subplots(1, 2, figsize=(12, 6))
            self.im1 = self.axes[0].imshow(accumulator_counts, cmap='hot', extent=[thetas[0], thetas[-1], rhos[-1], rhos[0]])
            self.axes[0].set_title('Original Hough Transform')
            self.axes[0].set_xlabel('Theta (radians)')
            self.axes[0].set_ylabel('Rho')
            self.axes[0].grid(True)
            self.cbar1 = plt.colorbar(self.im1, ax=self.axes[0], label='Number of Points')

            self.im2 = self.axes[1].imshow(filtered_counts, cmap='hot', extent=[thetas[0], thetas[-1], rhos[-1], rhos[0]])
            self.axes[1].set_title('Filtered Hough Transform')
            self.axes[1].set_xlabel('Theta (radians)')
            self.axes[1].set_ylabel('Rho')
            self.axes[1].grid(True)
            self.cbar2 = plt.colorbar(self.im2, ax=self.axes[1], label='Number of Points')
        else:
            self.im1.set_data(accumulator_counts)
            self.im2.set_data(filtered_counts)

        plt.pause(0.001)


    def line_angle_difference(self, theta1, theta2):
        """
         Calculate the smaller angle between 2 line
        """
        return  min(abs(theta1 - theta2), np.pi- abs(theta1 -  theta2))

    def is_line_similar(self, rho1, theta1, rho2, theta2, rho_threshold, theta_threshold):
        """
        Function to compare the line found by Hough transform
        theta has domain [-pi/2, pi/2]
        rho has domain [-p, p]

        Find similarity between line 1 and line 2 and the symmetry of line 2
        A line represented by (rho, theta) can also be represented by (-rho, theta + pi)
        """
        # Compute differences for the original representation

       # Make sure theta1 > theta2
        if theta1 < theta2:
            theta1, theta2 = theta2, theta1
            rho1, rho2 = rho2, rho1
        rho_diff_original = abs(rho1 - rho2)
        theta_diff_original = theta1 - theta2

        rho_diff_symmetric = abs(rho1 + rho2)
        theta_diff_symmetric = np.pi - theta1 + theta2

        return (rho_diff_original < rho_threshold and theta_diff_original < theta_threshold) or \
               (rho_diff_symmetric < rho_threshold and theta_diff_symmetric < theta_threshold)


    def find_average_line(self, rho1, theta1, rho2, theta2):
        """
            Find the line in the middle of the smaller angle between 2 line
        """
        delta_theta = self.line_angle_difference(theta1, theta2)

        # Make sure theta 1 < theta 2
        if theta1 > theta2:
            theta1 = theta2
            rho1 = rho2

        rho_middle = (-rho2 + rho1)/2
        theta_middle = theta1 - delta_theta/2
        if theta_middle < -np.pi/2:
            theta_middle = theta_middle + np.pi
            rho_middle = - rho_middle

            
        return (rho_middle, theta_middle)

    def group_lines(self, lines_with_points,  rho_threshold=10, theta_threshold=np.radians(5),):
        """
        Group lines based on proximity in theta and rho by checking all lines in the group.

        Args:
            lines_with_points: List of tuples [(line, points)], where:
                - line = (rho, theta)
                - points = list of (x, y)
            theta_threshold: Maximum difference in theta (in radians) for lines to be grouped.
            rho_threshold: Maximum difference in rho for lines to be grouped.

        Returns:
            grouped_lines: List of grouped lines [(mean_line, merged_points)], where:
                        - mean_line = (mean_rho, mean_theta)
                        - merged_points = list of all points in the group.
        """
        if not lines_with_points:
            return []

        # Sort lines by theta
        lines_with_points = sorted(lines_with_points, key=lambda lp: lp[0][1])  # Sort by theta

        grouped_lines = []  # List of grouped lines
        for line, points in lines_with_points:
            rho, theta = line
            line_added = False

            # Compare the line with all existing groups
            for group in grouped_lines:
                group_rho, group_theta = group[0]  # Mean rho and theta of the group
                if self.is_line_similar(group_rho, group_theta, rho, theta, rho_threshold, theta_threshold):
                    # Add the line to this group
                    # Update group rho and theta
                    group[0] = (rho, theta)
                    group[1].append((rho, theta))  # Add line to group
                    group[2].extend(points)  # Merge points
                    line_added = True
                    break

            # If line does not belong to any group, create a new group
            if not line_added:
                grouped_lines.append([(rho, theta), [(rho, theta)], points.copy()])

        # Compute mean rho and theta for each group
        # refined_groups = []
        # for group in grouped_lines:
        #     lines_in_group = group[1]
        #     all_points = group[2]
        #     mean_rho = np.mean([l[0] for l in lines_in_group])
        #     mean_theta = np.mean([l[1] for l in lines_in_group])
        #     refined_groups.append(((mean_rho, mean_theta), all_points))
        refined_groups = [(group[0], group[2]) for group in grouped_lines]
        return grouped_lines

    def fit_line_to_group(self,grouped_lines):
        """
        Fit a line for each group based on the points in the group using ODR.

        Args:
            grouped_lines: List of tuples [(mean_line, points)], where:
                - mean_line = (rho, theta)
                - points = list of (x, y) for the group.

        Returns:
            fitted_lines: List of tuples [(rho, theta)], where:
                - rho, theta are the refined parameters of the line for each group.
        """


        fitted_lines = []
        for _,_, points in grouped_lines:
            if len(points) < 2:
                continue  # Skip groups with fewer than 2 points

            points_array = np.array(points)

            x = points_array[:, 0]
            y = points_array[:, 1]

            # Create matrix A for Ax = 0
            A = np.column_stack((x, y, np.ones(len(x))))

            # Perform Singular Value Decomposition (SVD)
            _, _, vh = np.linalg.svd(A)
            line_params = vh[-1]  # The last row of V (or Vh) gives the solution

            # Normalize the parameters to ensure a^2 + b^2 = 1
            a, b, c = line_params
            norm = np.sqrt(a**2 + b**2)
            a, b, c = a / norm, b / norm, c / norm
            
            rho = abs(c) / np.sqrt(a**2 + b**2)
            theta = np.arctan2(b, a)
            fitted_lines.append(((rho, theta), points))

        return fitted_lines

    def visualize_lines(self, grouped_lines, line_length=5.0):
        """
        Visualize grouped lines in RViz using virtual points derived from rho and theta.
        Each group gets a different color.

        Args:
            grouped_lines: List of tuples [(line, points)], where:
                - line = (rho, theta)
                - points = list of (x, y) (optional, not used here)
            line_length: Length of the virtual line for visualization.
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Generate distinct colors for groups
        colors = self.generate_colors(len(grouped_lines))

        for group_idx, (line, _) in enumerate(grouped_lines):  # Ignore actual points
            rho, theta = line
            group_color = colors[group_idx]

            # Compute two endpoints of the virtual line
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            # Define two points along the line
            x1 = rho * cos_theta - line_length * sin_theta
            y1 = rho * sin_theta + line_length * cos_theta
            x2 = rho * cos_theta + line_length * sin_theta
            y2 = rho * sin_theta - line_length * cos_theta

            # Create a line marker for this group
            marker = Marker()
            marker.header.frame_id = self.msg_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02  # Line thickness
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *group_color, 1.0

            # Add the two endpoints as virtual points
            p1 = Point()
            p1.x = x1
            p1.y = y1
            p1.z = 0.0

            p2 = Point()
            p2.x = x2
            p2.y = y2
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_publisher.publish(marker_array)


    def visualize_grouped_lines(self, grouped_lines, line_length=5.0):
        """
        Visualize grouped lines in RViz. Each group gets a different color.
        Visualize with points.

        Args:
            grouped_lines: List of tuples [(mean_line, lines, points)], where:
                - mean_line = (rho, theta) (mean line for the group, optional, not visualized).
                - lines = list of (rho, theta) for each line in the group.
                - points = list of (x, y) for all points in the group.
            line_length: Length of the virtual line for visualization.
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Step 1: Clear all previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)
        self.marker_publisher.publish(marker_array)

        # Step 2: Generate distinct colors for groups
        colors = self.generate_colors(len(grouped_lines))

        # Step 3: Add new markers
        marker_array = MarkerArray()
        for group_idx, group in enumerate(grouped_lines):
            mean_line, lines, points = group  # Unpack group lines and points
            group_color = colors[group_idx]

            # Visualize each line in the group
            for line in lines:
                rho, theta = line

                # Compute two endpoints of the virtual line
                cos_theta = np.cos(theta)
                sin_theta = np.sin(theta)

                # Define two points along the line
                x1 = rho * cos_theta - line_length * sin_theta
                y1 = rho * sin_theta + line_length * cos_theta
                x2 = rho * cos_theta + line_length * sin_theta
                y2 = rho * sin_theta - line_length * cos_theta

                # Create a line marker for this line
                line_marker = Marker()
                line_marker.header.frame_id = self.msg_frame
                line_marker.header.stamp = self.get_clock().now().to_msg()
                line_marker.id = marker_id
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.02  # Line thickness
                line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a = *group_color, 1.0

                # Add the two endpoints as virtual points
                p1 = Point()
                p1.x = x1
                p1.y = y1
                p1.z = 0.0

                p2 = Point()
                p2.x = x2
                p2.y = y2
                p2.z = 0.0

                line_marker.points.append(p1)
                line_marker.points.append(p2)

                marker_array.markers.append(line_marker)
                marker_id += 1

            # Visualize the points associated with the group
            for px, py in points:
                point_marker = Marker()
                point_marker.header.frame_id = self.msg_frame
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.id = marker_id
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = 0.05
                point_marker.scale.y = 0.05
                point_marker.scale.z = 0.05
                point_marker.color.r, point_marker.color.g, point_marker.color.b, point_marker.color.a = *group_color, 1.0

                point_marker.pose.position.x = px
                point_marker.pose.position.y = py
                point_marker.pose.position.z = 0.0

                marker_array.markers.append(point_marker)
                marker_id += 1

        # Publish the updated markers
        self.marker_publisher.publish(marker_array)



    


    def generate_colors(self, num_colors):
        """Generate distinct RGB colors for groups."""
        cmap = plt.get_cmap('hsv')  # Use HSV colormap for distinct colors
        return [cmap(i / num_colors)[:3] for i in range(num_colors)]
    
    # def find_inliers(self, points, rho, theta, distance_threshold):
    #     inliers = []
    #     for x, y in points:
    #         distance = abs(x * np.cos(theta) + y * np.sin(theta) - rho)
    #         if distance < distance_threshold:
    #             inliers.append((x, y))
    #     return np.array(inliers)

    # def refine_line_parameters(self, points):
    #     X, Y = points[:, 0], points[:, 1]
    #     A = np.column_stack((X, Y, np.ones_like(X)))
    #     b = np.zeros_like(X)

    #     # Solve for line parameters using Least Squares
    #     _, _, Vt = np.linalg.svd(A, full_matrices=False)
    #     line = Vt[-1]
    #     rho = np.sqrt(line[0]**2 + line[1]**2)
    #     theta = np.arctan2(line[1], line[0])
    #     return rho, theta

    # def track_lines(self, new_lines):
    #     tracked_lines = []

    #     for rho_new, theta_new in new_lines:
    #         matched = False
    #         for rho_prev, theta_prev in self.previous_lines:
    #             if abs(rho_prev - rho_new) < self.rho_tolerance and abs(theta_prev - theta_new) < self.theta_tolerance:
    #                 tracked_lines.append((rho_new, theta_new))  # Keep the updated line
    #                 matched = True
    #                 break
    #         if not matched:
    #             tracked_lines.append((rho_new, theta_new))

    #     return tracked_lines

    # def visualize(self, lines, points):
    #     marker_array = MarkerArray()
    #     marker_id = 0

    #     # Visualize lines
    #     for rho, theta in lines:
    #         marker = Marker()
    #         marker.header.frame_id = 'laser'
    #         marker.id = marker_id
    #         marker.type = Marker.LINE_LIST
    #         marker.action = Marker.ADD
    #         marker.scale.x = 0.02

    #         p1, p2 = self.get_line_endpoints(rho, theta)
    #         marker.points.append(p1)
    #         marker.points.append(p2)

    #         marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0  # Yellow
    #         marker_array.markers.append(marker)
    #         marker_id += 1

    #     self.marker_publisher.publish(marker_array)

    # def get_line_endpoints(self, rho, theta, length=10.0):
    #     cos_theta, sin_theta = np.cos(theta), np.sin(theta)
    #     x1 = rho * cos_theta - length * sin_theta
    #     y1 = rho * sin_theta + length * cos_theta
    #     x2 = rho * cos_theta + length * sin_theta
    #     y2 = rho * sin_theta - length * cos_theta

    #     p1, p2 = Point(), Point()
    #     p1.x, p1.y = x1, y1
    #     p2.x, p2.y = x2, y2
    #     return p1, p2

class DBSCANScanFilterNode(Node):
    def __init__(self):
        super().__init__('dbscan_scan_filter_node')

        # Subscription to LaserScan topic
        self.subscription = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, 10
        )

        # Publisher for visualization
        self.marker_publisher = self.create_publisher(
            MarkerArray, 'visualization_marker_array', 10
        )

        self.get_logger().info("DBSCAN Scan Filter Node Started")

        self.msg_frame = None

    def scan_callback(self, msg):
        self.msg_frame = msg.header.frame_id
        # Step 1: Convert LaserScan to Cartesian Coordinates
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        distances = np.array(msg.ranges)
        valid_indices = np.isfinite(distances)

        x = distances[valid_indices] * np.cos(angles[valid_indices])
        y = distances[valid_indices] * np.sin(angles[valid_indices])
        points = np.column_stack((x, y))

        if len(points) < 2:
            self.get_logger().warning("Not enough valid points to cluster.")
            return

        # Step 2: Apply DBSCAN Clustering
        dbscan = DBSCAN(eps=0.12, min_samples=2)  # Adjust eps and min_samples as needed
        labels = dbscan.fit_predict(points)

        # Select a few point in each cluster and apply hough tranform.
        # represent_point = self.select_representative_points(points, labels, 20)
        # detected_line = self.hough_transform(represent_point, 45, 45, 25)
        # self.visualize_lines(detected_line)


        # Step 3: Visualize Clusters
        self.visualize_clusters(points, labels, self.msg_frame)
        # plot_polar_scan(msg)

    def select_representative_points(self, points, labels, num_points_per_cluster=2):
        """
        Select a specified number of representative points from each cluster and include outliers.

        Args:
            points: (N, 2) array of x, y coordinates.
            labels: Cluster labels for each point (output of DBSCAN `fit_predict`).
            outliers: Array of x, y coordinates classified as outliers.
            num_points_per_cluster: Number of representative points to randomly select from each cluster.

        Returns:
            representative_points: Array of selected representative points including outliers.
        """
        unique_labels = set(labels)
        representative_points = []

        for label in unique_labels:
            if label != -1:  # IF not outliers
                # Get points in the current cluster
                cluster_points = points[labels == label]

                # # Randomly select the specified number of points from the cluster
                # if len(cluster_points) > num_points_per_cluster:
                #     selected_points = cluster_points[np.random.choice(len(cluster_points), num_points_per_cluster, replace=False)]
                # else:
                #     # If the cluster has fewer points than required, select all points
                #     selected_points = cluster_points

            # Ensure there are at least n points in the cluster
            
            if len(cluster_points) >= num_points_per_cluster:
                # Generate n equally spaced indices
                indices = np.linspace(0, len(cluster_points) - 1, num_points_per_cluster, dtype=int)
                selected_points = cluster_points[indices]
            else:
                # If fewer points than n, take all points
                selected_points = cluster_points

            representative_points.extend(selected_points)
        return np.array(representative_points)


    def visualize_clusters(self, points, labels, frame_id):
        marker_array = MarkerArray()
        unique_labels = set(labels)
        marker_id = 0

        # Generate distinct colors for clusters (excluding red for outliers)
        colors = self.generate_non_red_colors(len(unique_labels))

        for cluster_id in unique_labels:
            cluster_points = points[labels == cluster_id]
            if cluster_id == -1:
                cluster_color = (1.0, 0.0, 0.0)  # Red for outliers
            else:
                cluster_color = colors[cluster_id]  # Colors for clusters

            # Create Marker for each cluster
            for px, py in cluster_points:
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD

                marker.pose.position.x = px
                marker.pose.position.y = py
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0

                marker.scale.x = 0.05
                marker.scale.y = 0.05
                marker.scale.z = 0.05

                marker.color.r, marker.color.g, marker.color.b, marker.color.a = *cluster_color, 1.0

                marker_array.markers.append(marker)
                marker_id += 1

        self.marker_publisher.publish(marker_array)

    def generate_non_red_colors(self,num_clusters):
        """
        Generate colors for clusters using Seaborn while avoiding red.

        Args:
            num_clusters (int): Number of clusters to generate colors for.

        Returns:
            list: A list of RGB tuples for the clusters.
        """
        # Generate a color palette (e.g., 'deep' or 'tab10')
        palette = sns.color_palette("Set2", num_clusters * 2)  # Generate more colors than needed to filter red

        # Filter out colors that resemble red
        non_red_colors = [
            color for color in palette
            if not (color[0] > 0.8 and color[1] < 0.3 and color[2] < 0.3)  # Avoid shades close to red
        ]

        # Ensure we have enough non-red colors
        if len(non_red_colors) < num_clusters:
            raise ValueError("Not enough non-red colors available in the selected palette.")

        return non_red_colors[:num_clusters]

    def hough_transform(self, points, theta_bins, rho_bins, threshold):
        # Compute the diagonal of the points for rho range
        diagonal = np.sqrt(np.max(points[:, 0])**2 + np.max(points[:, 1])**2)
        thetas = np.linspace(-np.pi/2, np.pi/2, theta_bins)
        rhos = np.linspace(-diagonal, diagonal, rho_bins)

        # Initialize the accumulator as a 2D array of lists
        accumulator_counts = np.zeros((len(rhos), len(thetas)), dtype=int)
        accumulator = [[[] for _ in range(len(thetas))] for _ in range(len(rhos))]

        # Populate the accumulator with points
        for x, y in points:
            for theta_idx, theta in enumerate(thetas):
                rho = x * np.cos(theta) + y * np.sin(theta)
                rho_idx = np.argmin(np.abs(rhos - rho))
                accumulator[rho_idx][theta_idx].append((x, y))
                accumulator_counts[rho_idx, theta_idx] += 1

        filtered_counts = self.custom_local_maximum_filter(accumulator_counts, size=10, stride = 10)
        # Detect lines with votes above threshold and pack corresponding points
        if threshold > 0 and threshold <=1:
            threshold = np.max(filtered_counts)*threshold
        detected_lines = []
        for rho_idx in range(len(rhos)):
            for theta_idx in range(len(thetas)):
                if  filtered_counts[rho_idx, theta_idx] > threshold:  # Check if the number of points exceeds threshold
                    line = (rhos[rho_idx], thetas[theta_idx])  # Line parameters
                    line_points = accumulator[rho_idx][theta_idx]  # Points contributing to this line
                    detected_lines.append((line, line_points))

        self.visualize_hough_space(accumulator_counts, thetas, rhos)
        return detected_lines
    
    def custom_local_maximum_filter(self, array, size, stride):
        """
        Custom implementation of a local maximum filter with customizable stride.
        Keeps only the local maxima and sets others to 0.

        Args:
            array (numpy.ndarray): Input 2D array to process.
            size (int): Size of the local region (e.g., 3 for a 3x3 window).
            stride (int): Step size for moving the filter window.

        Returns:
            numpy.ndarray: Processed array where only local maxima are retained.
        """
        output = np.zeros_like(array)  # Initialize the output array with zeros

        # Iterate over each element in the array with the given stride
        for i in range(0, array.shape[0], stride):
            for j in range(0, array.shape[1], stride):
                # Ensure the window doesn't exceed array bounds
                end_i = min(i + size, array.shape[0])
                end_j = min(j + size, array.shape[1])

                # Extract the local region (window)
                window = array[i:end_i, j:end_j]

                # Skip empty windows
                if window.size == 0:
                    continue

                # Get the maximum value in the window
                local_max = np.max(window)

                # Get the position (indices) of the maximum value in the window
                max_position = np.unravel_index(np.argmax(window), window.shape)

                # Set the maximum value in the output array
                output[i + max_position[0], j + max_position[1]] = local_max

        return output
    
    def visualize_hough_space(self, accumulator_counts, thetas, rhos):
        """
        Visualize the Hough transform parameter space. The plot updates dynamically.

        Args:
            accumulator_counts: 2D array of counts in the accumulator.
            thetas: Array of theta values used in the transform.
            rhos: Array of rho values used in the transform.
        """
        filtered_counts = self.custom_local_maximum_filter(accumulator_counts, size=10, stride=10)

        # Dynamic figure setup
        if not hasattr(self, 'fig'):
            self.fig, self.axes = plt.subplots(1, 2, figsize=(12, 6))
            self.im1 = self.axes[0].imshow(accumulator_counts, cmap='hot', extent=[thetas[0], thetas[-1], rhos[-1], rhos[0]])
            self.axes[0].set_title('Original Hough Transform')
            self.axes[0].set_xlabel('Theta (radians)')
            self.axes[0].set_ylabel('Rho')
            self.axes[0].grid(True)
            self.cbar1 = plt.colorbar(self.im1, ax=self.axes[0], label='Number of Points')

            self.im2 = self.axes[1].imshow(filtered_counts, cmap='hot', extent=[thetas[0], thetas[-1], rhos[-1], rhos[0]])
            self.axes[1].set_title('Filtered Hough Transform')
            self.axes[1].set_xlabel('Theta (radians)')
            self.axes[1].set_ylabel('Rho')
            self.axes[1].grid(True)
            self.cbar2 = plt.colorbar(self.im2, ax=self.axes[1], label='Number of Points')
        else:
            self.im1.set_data(accumulator_counts)
            self.im2.set_data(filtered_counts)

        plt.pause(0.001)

    def visualize_lines(self, grouped_lines, line_length=5.0):
        """
        Visualize grouped lines in RViz using virtual points derived from rho and theta.
        Each group gets a different color.

        Args:
            grouped_lines: List of tuples [(line, points)], where:
                - line = (rho, theta)
                - points = list of (x, y) (optional, not used here)
            line_length: Length of the virtual line for visualization.
        """
        marker_array = MarkerArray()
        marker_id = 0

        # Generate distinct colors for groups
        colors = self.generate_colors(len(grouped_lines))

        for group_idx, (line, _) in enumerate(grouped_lines):  # Ignore actual points
            rho, theta = line
            group_color = colors[group_idx]

            # Compute two endpoints of the virtual line
            cos_theta = np.cos(theta)
            sin_theta = np.sin(theta)

            # Define two points along the line
            x1 = rho * cos_theta - line_length * sin_theta
            y1 = rho * sin_theta + line_length * cos_theta
            x2 = rho * cos_theta + line_length * sin_theta
            y2 = rho * sin_theta - line_length * cos_theta

            # Create a line marker for this group
            marker = Marker()
            marker.header.frame_id = self.msg_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.02  # Line thickness
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *group_color, 1.0

            # Add the two endpoints as virtual points
            p1 = Point()
            p1.x = x1
            p1.y = y1
            p1.z = 0.0

            p2 = Point()
            p2.x = x2
            p2.y = y2
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

            marker_array.markers.append(marker)
            marker_id += 1

        self.marker_publisher.publish(marker_array)

    def generate_colors(self, num_colors):
        """Generate distinct RGB colors for groups."""
        cmap = plt.get_cmap('hsv')  # Use HSV colormap for distinct colors
        return [cmap(i / num_colors)[:3] for i in range(num_colors)]

class LineTrackingNode(Node):
    def __init__(self):
        super().__init__('line_tracking_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.get_logger().info("Line Tracking Node Started")

        # Parameter declaration
        self.threshold = 10 # Line with less points than threshold are discarded
        self.Tmax = 0.05

    def scan_callback(self, msg):
        # Convert LaserScan to Cartesian Points
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        distances = np.array(msg.ranges)
        valid_indices = np.isfinite(distances)
        valid_angles = angles[valid_indices]
        valid_distances = distances[valid_indices]
        x = valid_distances * np.cos(valid_angles)
        y = valid_distances * np.sin(valid_angles)
        points = np.column_stack((x, y))


        if len(points) < 2:
            self.get_logger().warning("Not enough valid points to cluster.")
            return

        # Step 2: Apply DBSCAN Clustering
        dbscan = DBSCAN(eps=0.12, min_samples=2)  # Adjust eps and min_samples as needed
        labels = dbscan.fit_predict(points)

        inliers, outliers = points[labels!=-1], points[labels==-1]

        # Apply Line Tracking
        lines, line_indices = self.line_tracking(inliers, Tmax=self.Tmax)

        # Apply Backtracking
        lines, line_indices = self.backtracking(inliers, lines, line_indices, Tmax=self.Tmax)

        remaining_outliers = []  # Temporary list for remaining outliers
        for point in outliers:
            is_inlier = False
            for line in lines:
                if self.point_to_line_distance(point, line) < self.Tmax:
                    inliers = np.vstack((inliers, point))
                    is_inlier = True
                    break  # No need to check further lines
            if not is_inlier:
                remaining_outliers.append(point)
        outliers = remaining_outliers 


        # Visualize Results
        self.visualize_lines(lines, line_indices, inliers)
        self.visualize_points(outliers, "base_scan")


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
        lines = [line for idx, line in enumerate(lines) if line_indices[idx][1] - line_indices[idx][0] + 1 > self.threshold]
        line_indices = [indices for indices in line_indices if indices[1] - indices[0] + 1 > self.threshold]

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

    def visualize_lines(self, lines, line_indices, points):
        marker_array = MarkerArray()
        colors = plt.cm.get_cmap('Set2', len(lines))
        marker_id = 0

        for idx, (line, (s, e)) in enumerate(zip(lines, line_indices)):
            # Line visualization
            r, theta = line
            color = colors(idx)[:3]

            marker = Marker()
            marker.header.frame_id = "base_scan"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = *color, 1.0

            x1, y1 = points[s]
            x2, y2 = points[e]

            # x1 = r * cos(theta) - 10 * sin(theta)
            # y1 = r * sin(theta) + 10 * cos(theta)
            # x2 = r * cos(theta) + 10 * sin(theta)
            # y2 = r * sin(theta) - 10 * cos(theta)

            marker.points.append(Point(x=x1, y=y1, z=0.))
            marker.points.append(Point(x=x2, y=y2, z=0.))
            marker_array.markers.append(marker)
            marker_id += 1

            # Point visualization
            for px, py in points[s:e + 1]:
                point_marker = Marker()
                point_marker.header.frame_id = "base_scan"
                point_marker.header.stamp = self.get_clock().now().to_msg()
                point_marker.id = marker_id
                point_marker.type = Marker.SPHERE
                point_marker.action = Marker.ADD
                point_marker.scale.x = 0.05
                point_marker.scale.y = 0.05
                point_marker.scale.z = 0.05
                point_marker.color.r, point_marker.color.g, point_marker.color.b, point_marker.color.a = *color, 1.0
                point_marker.pose.position.x = px
                point_marker.pose.position.y = py
                point_marker.pose.position.z = 0.
                marker_array.markers.append(point_marker)
                marker_id += 1

        self.marker_publisher.publish(marker_array)

    def visualize_points(self, points, frame_id):
        """
        Visualize a set of points in red in Rviz2.

        Args:
            points (list or numpy.ndarray): List or array of (x, y) coordinates of points.
            frame_id (str): The frame of reference for the visualization.
        """
        # Create a MarkerArray message
        marker_array = MarkerArray()

        # Define a single marker for all points
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0  # Unique ID for the marker
        marker.type = Marker.SPHERE_LIST  # Single marker for a list of spheres
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # Sphere size
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Add points to the marker
        for x, y in points:
            point = Point()
            point.x = x
            point.y = y
            point.z = 0.0  # Assuming 2D points in the XY plane
            marker.points.append(point)

        # Append the marker to the MarkerArray
        marker_array.markers.append(marker)

        # Publish the MarkerArray
        self.marker_publisher.publish(marker_array)

fig, ax, scatter = None, None, None 
def plot_polar_scan(msg):
    """
    Dynamically update a polar plot for a given scan message.

    Args:
        msg: ROS 2 LaserScan message containing the scan data.
    """
    global fig, ax, scatter

    # Extract angles and distances
    angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
    distances = np.array(msg.ranges)

    # Filter invalid ranges (e.g., Inf or NaN)
    valid_indices = np.isfinite(distances)
    valid_angles = angles[valid_indices]
    valid_distances = distances[valid_indices]

    if fig is None or ax is None:
        # Initialize the figure and axis on the first call
        fig, ax = plt.subplots(figsize=(10, 6))
        ax.set_title("Lidar Scan in Polar Distance Space")
        ax.set_xlabel("Theta (radians)")
        ax.set_ylabel("Distance (meters)")
        ax.grid(True)
        plt.ion()  # Enable interactive mode
        plt.show()

    if scatter is None:
        # Create the scatter plot on the first call
        scatter = ax.scatter(valid_angles, valid_distances, s=1, c='b')  # Initial plot
    else:
        # Update the scatter plot data
        scatter.set_offsets(np.column_stack((valid_angles, valid_distances)))

    # Redraw the plot
    fig.canvas.draw_idle()
    fig.canvas.flush_events()

    
def main(args=None):
    rclpy.init(args=args)
    node = LineTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
