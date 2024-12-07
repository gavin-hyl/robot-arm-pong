"""
Simulate a non-physical hollow circular bin and publish as a visualization marker
array to RVIZ. The bin moves to a random position when a ball hits or touches it.

Node:      /bindemo
Publish:   /visualization_marker_array   visualization_msgs.msg.MarkerArray
"""

import rclpy
import numpy as np

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray
from geometry_msgs.msg          import PoseStamped


class BinEngineNode(Node):
    def __init__(self, name, rate):
        super().__init__(name)

        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        
        self.pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)
        
        self.sub = self.create_subscription(
            PoseStamped, '/ball_position', self.ball_position_callback, 10
        )
        
        self.pub_goal_pos = self.create_publisher(
            PoseStamped, '/goal_position', 10
        )

        self.get_logger().info("Publishing the bin visualization markers to RVIZ")
        
        # Bin parameters
        self.bin_radius = 0.25  
        self.bin_height = 0.5  
        self.bin_wall_thickness = 0.1  
        self.bin_position = np.array([0.0, 3.0, self.bin_height / 2])

        # Create markers for the bin
        self.marker_outer = self.create_cylinder_marker(
            id=1,
            scale=Vector3(x=2 * self.bin_radius, y=2 * self.bin_radius, z=self.bin_height),
            color=ColorRGBA(r=0.5, g=0.5, b=0.5, a=1.0),  # Grey color for the outer part
        )
        
        self.marker_inner = self.create_cylinder_marker(
            id=2,
            scale=Vector3(x=2 * (self.bin_radius - self.bin_wall_thickness),
                          y=2 * (self.bin_radius - self.bin_wall_thickness),
                          z=self.bin_height),
            color=ColorRGBA(r=0.0, g=0.0, b=0.0, a=1.0),  # Black color for the inner part
        )

        self.markerarray = MarkerArray(markers=[self.marker_outer, self.marker_inner])

        self.ball_position = np.array([0.0, 0.0, 0.0])

        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update()
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate} Hz)")


    def create_cylinder_marker(self, id, scale, color):
        """Helper function to create a cylinder marker."""
        marker = Marker()
        marker.header.frame_id  = "world"
        marker.header.stamp     = self.get_clock().now().to_msg()
        marker.action           = Marker.ADD
        marker.ns               = "bin"
        marker.id               = id
        marker.type             = Marker.CYLINDER
        marker.pose.position    = Point(x=self.bin_position[0],
                                        y=self.bin_position[1],
                                        z=self.bin_position[2])
        marker.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        marker.scale            = scale
        marker.color            = color
        return marker

    def ball_position_callback(self, msg):
        """Update ball position based on PoseStamped message."""
        self.ball_position = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])


    def check_collision(self):
        """Check if the ball has collided with the bin."""
        dist_horizontal = np.linalg.norm(self.ball_position[:2] - self.bin_position[:2])
        dist_vertical = np.abs(self.ball_position[2] - self.bin_position[2])
        if dist_horizontal < self.bin_radius + self.bin_wall_thickness and dist_vertical < self.bin_height / 2:
            self.get_logger().info(f"Ball collided with the bin at position: {self.ball_position}")
            return True
        return False


    def move_bin_randomly(self):
        """Move the bin to a random horizontal position."""
        self.bin_position[0] = np.random.uniform(-4, 4)
        self.bin_position[1] = np.random.uniform(2, 4)
        self.bin_position[2] = np.random.uniform(0, 2)
        self.get_logger().info(f"Bin moved to new position: {self.bin_position}")

    def shutdown(self):
        self.destroy_node()

    def update(self):
        self.t += self.dt

        if self.check_collision():
            self.move_bin_randomly()

        self.marker_outer.pose.position = Point(x=self.bin_position[0],
                                                y=self.bin_position[1],
                                                z=self.bin_position[2])
        self.marker_inner.pose.position = Point(x=self.bin_position[0],
                                                y=self.bin_position[1],
                                                z=self.bin_position[2])

        self.marker_outer.header.stamp = self.get_clock().now().to_msg()
        self.marker_inner.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.markerarray)

        # Publish the new bin position as a PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = self.bin_position[0]
        msg.pose.position.y = self.bin_position[1]
        msg.pose.position.z = self.bin_position[2]
        self.pub_goal_pos.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BinEngineNode('bin_engine', 10)

    rclpy.spin(node)

    node.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
