import rclpy
import numpy as np

from .ControllerNode      import RobotControllerNode
from .TransformHelpers   import *
from .TrajectoryUtils    import *

from .KinematicChain     import KinematicChain
from .MatrixUtils        import weighted_pinv


class Trajectory():

    def __init__(self, node):
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # xd refers to the derivative of x.
        
        # Idle position
        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]))
        self.qd0 = np.zeros(7)
        p, R, _, _ = self.chain.fkin(self.q0)
        self.p0 = p
        self.pd0 = np.zeros(3)
        self.R0 = R
        self.w0 = np.zeros(3)

        # Initial joint positions and velocities.
        self.q = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]))
        self.qd = np.zeros(7)
        self.p = np.array([0.0, 0.55, 1.0])
        self.pd = np.zeros(3)
        self.R = np.eye(3)
        self.w = np.zeros(3)

        # Start and end conditions for the trajectory. We do a joint trajectory, so we only store the joint values.
        self.t_start = None
        self.q_start = None
        self.qd_start = None

        self.t_end = None
        self.q_end = None
        self.qd_end = None
    
    def jointnames(self):
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']


    def evaluate(self, t, dt, ball_pos, ball_vel, goal_pos, regenerated):
        """Compute the desired joint/task positions and velocities, as well as the orientation and angular velocity.

        Args:
            t (float): the current time
            dt (float): the time step
            ball_pos (array): the ball position
            ball_vel (array): the ball velocity
            goal_pos (array): the goal position
            regenerated (bool): whether the ball has been regenerated in the last cycle

        Returns:
            (array, array, array, array, array, array, str): q, qd, p, pd, R, w, msg_str
        """
        msg_str = ""

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
                msg_str += "Failed to find a valid trajectory to the ball"
        if self.t_end is None or t > self.t_end:
            # if ANY trajectory has ended, return to idle position
            self.set_idle(t)

        # Track the trajectory given by t_start, t_end, q_start, q_end, qd_start, qd_end
        q, qd = spline(t - self.t_start, self.t_end - self.t_start, self.q_start, self.q_end, self.qd_start, self.qd_end)
        
        p, R, Jv, Jw = self.chain.fkin(q)
        pd = Jv @ qd
        w = Jw @ qd

        self.q = q
        self.qd = qd
        self.p = p
        self.pd = pd
        self.R = R
        self.w = w

        return (q, qd, p, pd, R, w), msg_str
    

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
            (array, array, array, array, float): p, pd, R, w, t_impact_from_now
        """
        # Assuming the task space is a sphere
        TASK_SPACE_R = 0.3
        TASK_SPACE_P = np.array([0, 0, TASK_SPACE_R * 2])

        # Forward integrate the velocity of the ball
        dt = 0.01
        found_impact_position = False
        gravity = np.array([0, 0, -9.8])
        for t in np.arange(0, 3, dt):
            # forward integrate 3 seconds. This comes from the p_v init back-integrates 1 second, 
            # and we choose a time value that's larger than that to capture the full trajectory.
            p_ball += v_ball * dt
            v_ball += gravity * dt
            r = np.linalg.norm(p_ball - TASK_SPACE_P)
            if r < TASK_SPACE_R * 0.9 and p_ball[2] > 0:
                p_impact = p_ball
                t_impact_from_now = t
                pd_ball_impact = v_ball
                found_impact_position = True
                break

        if not found_impact_position:
            # if no suitable impact position if found, return to idle position
            return self.p0, self.pd0, self.R0, self.w0, 1
        
        p_impact_to_target = p_target - p_impact
        t_hit_to_target = 0.5
        pd_ball_after_impact = (p_impact_to_target - 0.5 * gravity * t_hit_to_target**2) / t_hit_to_target # delta p = vt + 1/2 at^2

        # z-axis should be aligned with v_paddle
        # z = pd_paddle_at_impact / np.linalg.norm(pd_paddle_at_impact)
        z = pd_ball_after_impact - pd_ball_impact
        z = z / np.linalg.norm(z)
        y_guess = np.array([0, 1, 0])
        x = np.cross(y_guess, z)
        x = x / np.linalg.norm(x)
        y = np.cross(z, x)
        R_impact = np.vstack((x, y, z)).T

        # express it in the tip frame
        pd_paddle_at_impact = z * 1/2 * (np.dot(z, pd_ball_impact+ pd_ball_after_impact))\
                            + y * 0 \
                            + x * 0

        return p_impact, pd_paddle_at_impact, R_impact, np.zeros(3), t_impact_from_now


    # CHANGE THIS FUNCTION TO CHANGE THE CALCULATED END q, WHICH CHANGES THE TRAJECTORY
    def ikin(self, p_goal, pd_goal, R_goal, w_goal):
        """Compute the inverse kinematics for the given position and orientation.

        Args:
            p_goal (array): the goal position
            R_goal (array): the goal orientation

        Returns:
            q, qd (array, array): the joint positions and velocities that achieve the desired position and orientation
        """
        MAX_ITER = 500
        converged = False
        q = self.q.copy()


        lower_arm_weights = np.array([0.1, 0.2, 0.3, 0.6, 1.0, 2.0, 3.0])
        W_inv = np.linalg.inv(np.diag(lower_arm_weights))

        for _ in range(MAX_ITER):
            p, R, Jv, Jw = self.chain.fkin(q)
            p_error = p_goal - p
            R_error = 0.5 * (np.cross(R[:,0], R_goal[:,0]) \
                            + np.cross(R[:,1], R_goal[:,1])\
                            + np.cross(R[:,2], R_goal[:,2]))
            error = np.concatenate((p_error, R_error))

          
            if np.linalg.norm(error) < 1e-5:
                converged = True
                break
          
            Jac = np.vstack((Jv, Jw))
            J_weighted_pinv = W_inv @ Jac.T @ np.linalg.inv(Jac @ W_inv @ Jac.T)

        # Update joint positions
            delta_q = J_weighted_pinv @ error
            q += 0.5 * delta_q
        
        p, R, Jv, Jw = self.chain.fkin(q)
        Jac = np.vstack((Jv, Jw))
        J_weighted_pinv = W_inv @ Jac.T @ np.linalg.inv(Jac @ W_inv @ Jac.T)
        qd = J_weighted_pinv @ np.concatenate((pd_goal, w_goal))

        if converged:
            return q, qd
        else:
            return None, None

#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = RobotControllerNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
