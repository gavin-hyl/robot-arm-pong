import rclpy
import numpy as np

from std_msgs.msg import Float64

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from .GeneratorNode      import RobotControllerNode
from .TransformHelpers   import *
from .TrajectoryUtils    import *

from .KinematicChain     import KinematicChain
from .MatrixUtils        import weighted_pinv


class Trajectory():
    # Initialization.
    def __init__(self, node):
        self.chain5 = KinematicChain(
            node, 'world', 'link5', self.jointnames()[0:5])
        self.chain4 = KinematicChain(
            node, 'world', 'link4', self.jointnames()[0:4])
        
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]))
        self.qd = self.q0
        self.p0 = np.array([0.0, 0.55, 1.0])
        self.p_goal = self.p0.copy()
        self.v_init = np.zeros(3)
        self.v_goal = np.zeros(3)
        self.R_goal = np.eye(3)

        self.t_hit = 0
        self.t_start = 0
        self.p_start = np.zeros(3)
        self.pd = np.zeros(3)
        self.v_start = np.zeros(3)
        self.vd = np.zeros(3)

        self.return_to_p0_time = 1

    def jointnames(self):
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluation
    def evaluate(self, t, dt, ball_pos, ball_vel, goal_pos, regenerated):
        """Compute the desired joint/task positions and velocities, as well as the orientation and angular velocity.

        Args:
            t (float): the current time
            dt (float): the time step
            ball_pos (array): the ball position
            ball_vel (array): the ball velocity

        Returns:
            (array, array, array, array, array, array): qd, qddot, pd, vd, Rd, wd
        """

        # Get information about the kinematic chain
        ptip, Rtip, Jv, Jw = self.chain.fkin(self.qd)
        msg_str = None
        
        # check if the ball has been regenerated
        if regenerated:
            msg_str = "Ball has been regenerated"
            self.update_goal(ball_pos, ball_vel, goal_pos)
            self.t_start = t
            self.p_start = ptip
            self.v_start = self.vd
            
        # Compute the desired position and velocity
        t_action = t-self.t_start
        if self.t_hit > 0 and t_action < self.t_hit:
            pd, vd = spline(t_action, self.t_hit, \
                            self.p_start, self.p_goal, \
                            self.v_start, self.v_goal)
        elif t_action >= self.t_hit:
            pd, vd = goto(t_action-self.t_hit, self.return_to_p0_time, self.p_goal, self.p0)
        else:
            pd, vd = self.p0, np.zeros(3)
        
        Rd = self.R_goal
        wd = np.zeros(3)

        # Compte xdot
        Jac = np.vstack((Jv, Jw))
        xd_dot = np.concatenate((vd, wd))

        # Compute error
        p_error = pd - ptip
        R_error = 0.5 * (np.cross(Rtip[:,0], Rd[:,0]) + np.cross(Rtip[:,1], Rd[:,1]) + np.cross(Rtip[:,2], Rd[:,2]))
        error = np.concatenate((p_error, R_error))

        # Compute qdot
        LAM = 20
        GAMMA = 0.1
        qddot = weighted_pinv(Jac, GAMMA) @ (xd_dot + LAM * error)

        # Integrate qdot
        self.qd += qddot * dt
        self.vd = vd
        self.pd = pd

        return (self.qd, qddot, pd, vd, Rd, wd), msg_str
    

    def update_goal(self, p_ball, v_ball, p_target):
        """Calculate the desired position and orientation of the end effector to hit the target.

        Args:
            p_ball (array): the position of the ball
            v_ball (array): the velocity of the ball
            p_target (array): the position of the target

        Returns:
            None (attributes are updated)
        """
        # Assuming the task space is a sphere
        TASK_SPACE_R = 0.5
        TASK_SPACE_P = np.array([0, 0, TASK_SPACE_R * 2])

        # Forward integrate the velocity of the ball
        # find the point where the ball is closest to sphere defined by the center of the task space
        # and 1/2 radius away from the center (arbitrary criteria for now, can engineer a cost later)
        dt = 0.01
        found_hit = False
        gravity = np.array([0, 0, -9.8])
        for t in np.arange(0, 3, dt): # forward integrate 3 seconds. This comes from the p_v init back-integrates 1 second, and we choose a time value that's larger than that to capture the full trajectory.
            p_ball += v_ball * dt
            v_ball += gravity * dt
            r = np.linalg.norm(p_ball - TASK_SPACE_P)
            if r < TASK_SPACE_R * 0.9 and p_ball[2] > 0:
                best_p = p_ball
                best_t = t
                best_v = v_ball
                found_hit = True
                break

        if not found_hit:
            return

        t_hit_to_target = 0.5 # time to hit the target from the point of hitting the ball (arbitrary value)
        # now we find the desired velocity OF THE BALL after being hit
        p_delta = p_target - best_p
        v_desired = (p_delta - 0.5 * gravity * t_hit_to_target**2) / t_hit_to_target

        # desired velocity of the end effector as it hits the ball
        v_paddle = v_desired - best_v
        # v_paddle = v_desired

        # z-axis should be aligned with v_paddle
        z = v_paddle / np.linalg.norm(v_paddle)

        # we should do something about the x and y axis, but right now we test the other functionality first
        pass

        self.p_goal = best_p
        self.v_goal = v_paddle
        self.R_goal = np.eye(3)
        self.t_hit = best_t


    def ikin(self, p_goal, R_goal):
        """Compute the inverse kinematics for the given position and orientation.

        Args:
            p_goal (array): the goal position
            R_goal (array): the goal orientation

        Returns:
            array: the joint positions
        """
        MAX_ITER = 20
        # Newton-Raphson iteration to convergence
        q = self.qd.copy()
        prev_q = q.copy()
        for _ in range(MAX_ITER):
            p, R, Jv, Jw = self.chain.fkin(q)
            p_error = p_goal - p
            R_error = 0.5 * (np.cross(R[:,0], R_goal[:,0]) + np.cross(R[:,1], R_goal[:,1]) + np.cross(R[:,2], R_goal[:,2]))
            error = np.concatenate((p_error, R_error))
            Jac = np.vstack((Jv, Jw))
            q += weighted_pinv(Jac) @ Jac.T @ error
            if np.linalg.norm(q - prev_q) < 1e-6:
                break
            prev_q = q.copy()
        return q


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
