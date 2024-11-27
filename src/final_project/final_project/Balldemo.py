"""balldemo.py

   Simulate a non-physical ball and publish as a visualization marker
   array to RVIZ.

   Node:      /balldemo
   Publish:   /visualization_marker_array   visualization_msgs.msg.MarkerArray

"""

import rclpy
import numpy as np

from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration
from geometry_msgs.msg          import Point, Vector3, Quaternion
from geometry_msgs.msg          import PoseStamped, TwistStamped
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray
from .TransformHelpers          import *


class BallEngineNode(Node):
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        
        self.pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)
        
        self.pub_pos = self.create_publisher(
            PoseStamped, '/ball_position', 10)
        
        self.pub_vel = self.create_publisher(
            TwistStamped, '/ball_velocity', 10)
        
        self.sub_pose = self.create_subscription(
            PoseStamped, '/pose', self.pose_callback, 10)
        
        self.sub_twist = self.create_subscription(
            TwistStamped, '/twist', self.twist_callback, 10)
        
        self.get_logger().info("Subscribed to /pose and /twist topics")
        
        self.restitution = 1

        self.paddle_pos = np.zeros(3)
        self.paddle_z = np.array([0, 0, 1])
        self.paddle_vel = np.zeros(3)
        

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = 0.05

        self.p = np.array([0.0, 0.5, self.radius + 1])
        self.v = np.array([0.0, 0.0,  5.0       ])
        self.a = np.array([0.0, 0.0, -9.81      ])

        # Create the sphere marker.
        diam        = 2 * self.radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.p)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        # Create the marker array message.
        self.markerarray = MarkerArray(markers = [self.marker])

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate} Hz)")

    # Shutdown
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

    # Update - send a new joint command every time step.
    def update(self):
        self.t += self.dt
        self.v += self.dt * self.a
        self.p += self.dt * self.v

        # check for collision with the ground
        if self.p[2] < self.radius:
            self.p[2] = self.radius + (self.radius - self.p[2])
            self.v[2] *= -self.restitution
            self.v[0] *= self.restitution   # make the ball lose energy on other axes as well
            self.v[1] *= self.restitution

        # check for collision with the paddle
        if np.linalg.norm(self.paddle_pos - self.p) < self.radius * 2:
            n = self.paddle_z
            delta_v = self.v - self.paddle_vel
            delta_v_proj = np.dot(delta_v, n) * n
            self.v -= 2 * self.restitution * delta_v_proj

        # Update the ID number to create a new ball and leave the
        # previous balls where they are.
        #####################
        # self.marker.id += 1
        #####################

        # Update the message and publish.
        self.marker.header.stamp  = self.now().to_msg()
        self.marker.pose.position = Point_from_p(self.p)
        self.pub.publish(self.markerarray)

        pos_msg = PoseStamped()
        pos_msg.header.stamp = self.now().to_msg()
        pos_msg.pose.position.x = self.p[0]
        pos_msg.pose.position.y = self.p[1]
        pos_msg.pose.position.z = self.p[2]
        self.pub_pos.publish(pos_msg)

        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.now().to_msg()
        vel_msg.twist.linear.x = self.v[0]
        vel_msg.twist.linear.y = self.v[1]
        vel_msg.twist.linear.z = self.v[2]
        self.pub_vel.publish(vel_msg)
        

    def pose_callback(self, msg):
        """Callback for the pose topic. Records the current position and orientation of the paddle.

        Args:
            msg (PoseStamped): The message containing the current pose of the paddle.
        """
        self.paddle_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        orientation = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        R_from_quaternion = R_from_quat(orientation)
        self.paddle_z = R_from_quaternion @ np.array([0, 0, 1])

    def twist_callback(self, msg):
        """Callback for the twist topic. Records the current linear and angular velocity of the paddle.

        Args:
            msg (TwistStamped): The message containing the current twist of the paddle.
        """
        self.paddle_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])


def main(args=None):
    rclpy.init(args=args)
    node = BallEngineNode('ball_engine', 100)

    rclpy.spin(node)

    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()