import numpy as np
import rclpy
import tf2_ros

from math import nan

from asyncio            import Future
from rclpy.node         import Node
from std_msgs.msg       import Bool
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState

from .TransformHelpers   import quat_from_R


class RobotControllerNode(Node):
    # Initialization.
    def __init__(self, name, rate, Trajectory):
        # Initialize the node, naming it as specified
        super().__init__(name)

        # Set up a trajectory.
        self.trajectory = Trajectory(self)
        self.jointnames = self.trajectory.jointnames()

        # Add a publisher to send the joint commands.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose  = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist = self.create_publisher(TwistStamped, '/twist', 10)

        self.sub_ball_pos = self.create_subscription(PoseStamped, '/ball_position', self.ball_pos_callback, 10)
        self.sub_ball_vel = self.create_subscription(TwistStamped, '/ball_velocity', self.ball_vel_callback, 10)

        self.sub_goal_pos = self.create_subscription(PoseStamped, '/goal_position', self.goal_pos_callback, 10)
        self.sub_regenerated = self.create_subscription(Bool, '/ball_regeneration', self.regenerated_callback, 10)

        # Initialize a regular and static transform broadcaster
        self.tfbroadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Create a future object to signal when the trajectory ends,
        # i.e. no longer returns useful data.
        self.future = Future()

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now()+rclpy.time.Duration(seconds=self.dt)

        # Create a timer to keep calculating/sending commands.
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))
        
        self.ball_pos = np.zeros(3)
        self.ball_vel = np.zeros(3)
        self.goal_pos = np.zeros(3)
        self.regenerated = False

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()

    # Spin
    def spin(self):
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.done():
            self.get_logger().info("Stopping: " + self.future.result())
        else:
            self.get_logger().info("Stopping: Interrupted")


    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        # Compute the trajectory for this time.
        regenerated = self.regenerated
        self.regenerated = False    # avoid race condition
        des = self.trajectory.evaluate(self.t, self.dt, self.ball_pos, self.ball_vel, self.goal_pos, regenerated)
        if des is None:
            self.future.set_result("Trajectory has ended")
            return

        # Extract the appropriate information.
        if   len(des) == 2:
            (q,qdot,p,v,R,w) = (des[0],des[1],None,None,None,None)
        elif len(des) == 4:
            (q,qdot,p,v,R,w) = (des[0],des[1],des[2],des[3],None,None)
        elif len(des) == 6:
            (q,qdot,p,v,R,w) = des
        else:
            raise ValueError("The trajectory must return 2, 4, 6 elements")

        # Check the joint results.
        if q    is None:    q    = [nan] * len(self.jointnames)
        if qdot is None:    qdot = [nan] * len(self.jointnames)
        if p    is None:    p    = [0.0, 0.0, 0.0]
        if v    is None:    v    = [0.0, 0.0, 0.0]
        if w    is None:    w    = [0.0, 0.0, 0.0]

        if R    is None:    quat  = [0.0, 0.0, 0.0, 1.0]
        else:               quat  = quat_from_R(R)

        # Turn into lists.
        if type(q).__module__    == np.__name__: q    = q.flatten().tolist()
        if type(qdot).__module__ == np.__name__: qdot = qdot.flatten().tolist()
        if type(p).__module__    == np.__name__: p    = p.flatten().tolist()
        if type(v).__module__    == np.__name__: v    = v.flatten().tolist()
        if type(w).__module__    == np.__name__: w    = w.flatten().tolist()

        # Verify the sizes.
        if not (len(q) == len(self.jointnames) and
                len(qdot) == len(self.jointnames)):
            print(q)
            print(qdot)
            raise ValueError("(q) and (qdot) must be same len as jointnames!")
        if not (len(p) == 3 and len(v) == 3):
            raise ValueError("(p) and (v) must be length 3!")
        if not (len(w) == 3):
            raise ValueError("(omega) must be length 3!")

        # Determine the corresponding ROS time (seconds since 1970).
        now = self.start + rclpy.time.Duration(seconds=self.t)

        # Build up a joint message and publish.
        msg = JointState()
        msg.header.stamp = now.to_msg()         # Current time for ROS
        msg.name         = self.jointnames      # List of joint names
        msg.position     = q                    # List of joint positions
        msg.velocity     = qdot                 # List of joint velocities
        self.pubjoint.publish(msg)

        # Build up a pose message and publish.
        msg = PoseStamped()
        msg.header.stamp       = now.to_msg()   # Current time for ROS
        msg.pose.position.x    = p[0]
        msg.pose.position.y    = p[1]
        msg.pose.position.z    = p[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        self.pubpose.publish(msg)

        # Build up a twist message and publish.
        msg = TwistStamped()
        msg.header.stamp    = now.to_msg()      # Current time for ROS
        msg.twist.linear.x  = v[0]
        msg.twist.linear.y  = v[1]
        msg.twist.linear.z  = v[2]
        msg.twist.angular.x = w[0]
        msg.twist.angular.y = w[1]
        msg.twist.angular.z = w[2]
        self.pubtwist.publish(msg)

        # Prepare a transform message and broadcast.
        msg = TransformStamped()
        msg.header.stamp            = now.to_msg()
        msg.header.frame_id         = 'world'
        msg.child_frame_id          = 'desired'
        msg.transform.translation.x = p[0]
        msg.transform.translation.y = p[1]
        msg.transform.translation.z = p[2]
        msg.transform.rotation.x    = quat[0]
        msg.transform.rotation.y    = quat[1]
        msg.transform.rotation.z    = quat[2]
        msg.transform.rotation.w    = quat[3]
        self.tfbroadcaster.sendTransform(msg)

    def ball_pos_callback(self, msg):
        self.ball_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    
    def ball_vel_callback(self, msg):
        self.ball_vel = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    def goal_pos_callback(self, msg):
        self.goal_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

    def regenerated_callback(self, msg):
        self.regenerated = True