import rclpy
import numpy as np

from math import fmod

from .ControllerNode    import RobotControllerNode
from .TransformHelpers  import *
from .TrajectoryUtils   import *

from .KinematicChain    import KinematicChain
from .BallNode          import GRAVITY

ARM_WEIGHTS = [0.3, 0.4, 0.5, 0.7, 1, 1.5, 1.5]
ARM_WEIGHTS = np.array(ARM_WEIGHTS)

class Controller():
    """
    Controller class for the robot arm. This class is responsible for computing the desired joint positions and velocities for RVIZ, as well as planning the trajectories to hit the ball and return to the idle position.

    Attributes:
        chain (KinematicChain): the kinematic chain of the robot arm
        node (Node): the ROS node that the controller uses to interface with the ROS network
        q0 (array): the idle joint positions
        qd0 (array): the idle joint velocities
        p0 (array): the idle task space position
        pd0 (array): the idle task space velocity
        R0 (array): the idle task space orientation matrix
        w0 (array): the idle task space angular velocity
        q (array): the current joint positions
        qd (array): the current joint velocities
        p (array): the current task space position
        pd (array): the current task space velocity
        R (array): the current task space orientation matrix
        w (array): the current task space angular velocity
        t_start (float): the start time of the current trajectory
        q_start (array): the start joint positions of the current trajectory
        qd_start (array): the start joint velocities of the current trajectory
        t_end (float): the end time of the current trajectory
        q_end (array): the end joint positions of the current trajectory
        qd_end (array): the end joint velocities of the current trajectory
    """

    def __init__(self, node):
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())
        self.node = node
        
        # Idle position
        self.q0 = np.radians(np.array([-90, 45, 0, -90, -45, 0, 0]))
        self.qd0 = np.zeros(7)
        p, R, _, _ = self.chain.fkin(self.q0)
        self.p0 = p
        self.pd0 = np.zeros(3)
        self.R0 = R
        self.w0 = np.zeros(3)

        # Initial joint positions and velocities.
        self.q = self.q0
        self.qd = np.zeros(7)
        self.p = p
        self.pd = np.zeros(3)
        self.R = R
        self.w = np.zeros(3)

        # Start and end conditions for the trajectory.
        # Note that we are computing a joint trajectory.
        self.t_start = None
        self.q_start = None
        self.qd_start = None

        self.t_end = None
        self.q_end = None
        self.qd_end = None


    def jointnames(self):
        """ Returns the expected joint names for the robot. """
        return [f'theta{i}' for i in range(1, 8)]
    

    def wrap_q(self, q):
        """ Wrap the joint angles to [-pi, pi]. """
        q_wrapped = q.copy()
        for i, qi in enumerate(q):
            q_wrapped[i] = fmod(qi + np.pi, 2*np.pi) - np.pi
        return q_wrapped
    

    def evaluate(self, t, dt, ball_pos, ball_vel, goal_pos, regenerated):
        """Compute the desired joint/task positions and velocities, as well as the orientation and angular velocity.

        Args:
            t (float): the current time
            dt (float): the time step size
            ball_pos (array): the ball position
            ball_vel (array): the ball velocity
            goal_pos (array): the goal position
            regenerated (bool): whether the ball has been regenerated in the last cycle

        Returns:
            q (array): the joint positions at the current time
            qd (array): the joint velocities at the current time
            p (array): the task space position at the current time
            pd (array): the task space velocity at the current time
            R (array): the task space orientation matrix at the current time
            w (array): the task space angular velocity at the current time
        """

        if regenerated:
            # if the ball has been regenerated, compute the inverse kinematics to take us there
            self.t_start = t
            self.q_start = self.q.copy()
            self.qd_start = self.qd.copy()
            p_end, pd_end, R_end, w_end, t_to_impact = self.compute_impact_conditions(ball_pos, ball_vel, goal_pos)
            self.t_end = t + t_to_impact
            self.q_end, self.qd_end = self.ikin(p_end, pd_end, R_end, w_end)
            if self.q_end is None:
                self.set_idle(t)
                self.node.get_logger().info("Newton Raphson did not converge.")
            else:
                self.node.get_logger().info(f"Expected impact parameters: p= {p_end}, R = {R_end}")
        if self.t_end is None or t > self.t_end:
            # if ANY trajectory has ended, return to idle position
            self.set_idle(t)

        # Track the trajectory given by t_start, t_end, q_start, q_end, qd_start, qd_end
        q_diff = self.q_end - self.q_start
        q, qd = spline(t - self.t_start, self.t_end - self.t_start,
                       np.zeros(7), self.wrap_q(q_diff),
                       self.qd_start, self.qd_end)
        q += self.q_start

        p, R, Jv, Jw = self.chain.fkin(q)
        pd = Jv @ qd
        w = Jw @ qd
        
        self.q = self.wrap_q(q)
        self.qd = qd
        self.p = p
        self.pd = pd
        self.R = R
        self.w = w

        return (q, qd, p, pd, R, w)
    

    def set_idle(self, t, t_to_idle=1.5):
        """Set the appropriate variables to return to the idle position.

        Args:
            t (float): the current time
            t_to_idle (float, optional): Prescribed time to return to idle. Defaults to 1.5.
        """
        self.t_start = t
        self.q_start = self.q.copy()
        self.qd_start = self.qd.copy()
        self.t_end = t + t_to_idle
        self.q_end = self.q0
        self.qd_end = self.qd0
    

    def compute_impact_conditions(self, p_ball, v_ball, p_target):
        """Return the task space pose and twist of the end effector at the time of impact.

        Args:
            p_ball (array): the current position of the ball
            v_ball (array): the current velocity of the ball
            p_target (array): the target position

        Returns:
            p (array): the task space position at the time of impact
            pd (array): the task space velocity at the time of impact
            R (array): the task space orientation matrix at the time of impact
            w (array): the task space angular velocity at the time of impact
        """
        # Assuming the task space is a sphere
        TASK_SPACE_R = 0.3
        TASK_SPACE_P = np.array([0, 0, TASK_SPACE_R * 2])

        # Forward integrate the velocity of the ball
        dt = 0.01
        found_impact_position = False   
        for t in np.arange(0, 3, dt):
            # forward integrate 3 seconds. This comes from the p_v init back-integrates 1 second, 
            # and we choose a time value that's larger than that to capture the full trajectory.
            p_ball += v_ball * dt
            v_ball += GRAVITY * dt
            r = np.linalg.norm(p_ball - TASK_SPACE_P)
            # use compute_task_space_goal to find impact position
            # then run ikin to find joint angles and Jac
            # to find the condition number
            if r < TASK_SPACE_R * 0.9 and p_ball[2] > 0:
                p_impact = p_ball
                t_impact_from_now = t
                pd_ball_impact = v_ball
                found_impact_position = True
                break

        if not found_impact_position:
            # if no suitable impact position if found, return to idle position
            return self.p0, self.pd0, self.R0, self.w0, 1
    
        return *self.compute_task_space_goal(p_impact, pd_ball_impact, p_target), t_impact_from_now
    

    def compute_task_space_goal(self, p_impact, pd_impact, p_target):
        """Compute the pose and twist of the end effector at the time of impact.

        Args:
            p_impact (array): the impact position
            pd_impact (array): the impact velocity
            p_target (array): the target position

        Returns:
            p (array): the task space position at the time of impact
            pd (array): the task space velocity at the time of impact
            R (array): the task space orientation matrix at the time of impact
            w (array): the task space angular velocity at the time of impact
        """
        p_impact_to_target = p_target - p_impact
        t_hit_to_target = 0.5
        pd_ball_after_impact = (p_impact_to_target - 0.5 * GRAVITY * t_hit_to_target**2) / t_hit_to_target # delta p = vt + 1/2 at^2

        # z-axis should be aligned with v_paddle
        z = pd_ball_after_impact - pd_impact
        z = z / np.linalg.norm(z)
        y_guess = np.array([0, 1, 0])   # y doesn't really matter
        x = np.cross(y_guess, z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        R_impact = np.vstack((x, y, z)).T

        # Optimize the joint velocities at the time of impact.
        pd_z = 1/2 * (np.dot(z, pd_impact+ pd_ball_after_impact))
        pd_paddle_at_impact = pd_z * z

        return p_impact, pd_paddle_at_impact, R_impact, np.zeros(3)
    

    def ikin(self, p_goal, pd_goal, R_goal, w_goal):
        """Compute the inverse kinematics of (p = fkin(q), pd = Jac @ qd) for the given position and orientation using the Newton-Raphson method.

        Args:
            p_goal (array): the goal position
            pd_goal (array): the goal velocity (x and y ignored)
            R_goal (array): the goal orientation (x and y axes ignored)
            w_goal (array): the goal angular velocity (ignored, added for completeness)

        Returns:
            q (array): q if converged, None otherwise
            qd (array): qd if converged, None otherwise
        """
        MAX_ITER = 2000
        converged = False
        q = self.q.copy()

        gamma = 0.1

        W2 = np.diag(ARM_WEIGHTS)
        W1 = np.diag([1, 1, 1, 10, 10, 10])

        err_magnitudes = []

        for _ in range(MAX_ITER):
            p, R, Jv, Jw = self.chain.fkin(q)
            p_error = ep(p_goal, p)
            R_error = 1/2 * cross(R[:, 2], R_goal[:, 2])
            error = np.concatenate((p_error, R_error))
          
            if np.linalg.norm(error) < 1e-2:
                converged = True
                break

            err_magnitudes.append(np.linalg.norm(error))

            Jac = np.vstack((Jv, Jw))
            gamma = 0.1
            J_pinv = np.linalg.pinv(Jac.T @ W1**2 @ Jac + gamma**2 * W2**2) @ Jac.T @ W1**2

            LAM1 = 0.5
            LAM2 = 0.02
            qd_primary = J_pinv @ error * LAM1
            qd_secondary = (np.eye(self.chain.dofs) - J_pinv @ Jac) @ W2 @ self.wrap_q(self.q0 - q) * LAM2
            q += (qd_primary + qd_secondary)
            
        
        p, R, Jv, Jw = self.chain.fkin(q)
        Jv_tip = R.T @ Jv
        Jv_tip = Jv_tip[2]
        Jw_tip = R.T @ Jw
        Jw_tip = Jw_tip[:2]
        Jac = np.vstack((Jv_tip, Jw_tip))
        J_pinv = np.linalg.pinv(Jac.T @ Jac + gamma**2 * W2**2) @ Jac.T

        qd = J_pinv @ np.hstack(((R.T @ pd_goal)[2], (R.T @ w_goal)[:2])) \
            + (np.eye(self.chain.dofs) - J_pinv @ Jac) @ W2 @ self.wrap_q(self.q0 - q) * LAM2

        if converged:
            return self.wrap_q(q), qd
        else:
            for i, e in enumerate(err_magnitudes):
                if i % 10 == 0:
                    self.node.get_logger().info(f"Iteration {i}: Error magnitude: {e}")
            return None, None



def main(args=None):
    """ Main function for the controller. """
    rclpy.init(args=args)
    generator = RobotControllerNode('generator', 100, Controller)
    generator.spin()
    generator.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
