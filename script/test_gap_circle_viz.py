#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from wheelchair_control_support.msg import Gap  # Update this with the correct message import
import math

class GapVisualizer(Node):
    def __init__(self):
        super().__init__('gap_visualizer_node')
        
        # Create the subscriber to the /intended_gap topic
        self.subscription = self.create_subscription(
            Gap,  # Change this if your message type differs
            '/intended_gap',
            self.callback,
            10
        )
        
        # Create the publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, '/gap_circle', 10)
        
        # Define the radius for the circle
        self.radius = 1.0  # Adjust as needed
        self.num_points = 100  # Number of points to approximate the circle (higher for smoother)
    
    def callback(self, msg):
        # Create markers for the left and right points (hollow circle)
        self.create_hollow_circle_marker(msg.left_point_x, msg.left_point_y, 'left_circle')
        self.create_hollow_circle_marker(msg.right_point_x, msg.right_point_y, 'right_circle')
    
    def create_hollow_circle_marker(self, x, y, frame_id):
        marker = Marker()

        # Set the marker type to LINE_STRIP to create a loop (hollow circle)
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set the frame_id and timestamp for visualization in RViz
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'base_footprint'  # or the frame you are using (adjust if needed)

        # Set the scale for the circle (a line with a width)
        marker.scale.x = 0.01  # Line width for the circle

        # Set the color of the circle (red in this case)
        marker.color = ColorRGBA()
        marker.color.r = 0.0  # Red
        marker.color.g = 1.0  # Green
        marker.color.b = 0.0  # Blue
        marker.color.a = 1.0  # Full opacity

        # Generate the points for the circle
        for i in range(self.num_points + 1):  # Adding an extra point to close the loop
            angle = 2 * math.pi * i / self.num_points
            px = x + self.radius * math.cos(angle)
            py = y + self.radius * math.sin(angle)

            # Add the point to the marker (x, y, z)
            marker.points.append(Point(x=px, y=py, z=0.0))  # Z is set to 0 for ground level


        # Publish the marker
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    gap_visualizer = GapVisualizer()
    rclpy.spin(gap_visualizer)
    gap_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
