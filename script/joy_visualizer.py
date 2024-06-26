#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import matplotlib.pyplot as plt

class JoyVisualizer(Node):
    def __init__(self):
        super().__init__('joy_visualizer')
        self.joy_sub = self.create_subscription(Joy, '/whill/states/joy', self.joy_callback, 10)
        self.virtual_joy_sub = self.create_subscription(Joy, '/whill/controller/joy', self.virtual_joy_callback, 10)
        self.virtual_joy_sub  # prevent unused variable warning
        self.joy_sub  # prevent unused variable warning
        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')  # Ensure equal scaling
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        self.ax.grid(True)  # Show grid
        self.circle = plt.Circle((0, 0), 1, edgecolor='b', facecolor='none')  # Create a circle
        self.ax.add_patch(self.circle)
        self.point1, = self.ax.plot([], [], 'ro', markersize=10)  # Initialize point as red circle
        self.point2, = self.ax.plot([], [], 'go', markersize=10)
        self.x1_data, self.y1_data = [], []
        self.x2_data, self.y2_data = [], []
        plt.show(block = False)

    def joy_callback(self, msg):
        self.y1_data = [msg.axes[1]]
        self.x1_data = [msg.axes[0]]
        self.update_plot()

    def virtual_joy_callback(self, msg):
        self.y2_data = [msg.axes[1]]
        self.x2_data = [msg.axes[0]]
        self.update_plot()

    def update_plot(self):
        # Clear previous pointself.ax.set_xlim
        self.point1.set_data([], [])
        self.point2.set_data([], [])

        # Update current point
        self.point1.set_data(self.x1_data, self.y1_data)
        self.point2.set_data(self.x2_data, self.y2_data)

        # Redraw plot
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    joy_visualizer = JoyVisualizer()
    rclpy.spin(joy_visualizer)
    joy_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()