import rclpy
import numpy as np

from std_msgs.msg import Float64

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

# Grab the utilities
from .GeneratorNode      import RobotControllerNode
from .TransformHelpers   import *
from .TrajectoryUtils    import *

# Grab the general fkin from HW6 P1.
from .KinematicChain     import KinematicChain


#
#   Repulsion Joint Torques
#
#   This computes the equivalent joint torques that mimic a repulsion
#   force between the forearm and the top edge of the wall.  It uses
#   the kinematic chains to the elbow (4 joints) and wrist (5 joints)
#   to get the forearm segment.
#
def repulsion(q, wristchain, elbowchain):
    # Compute the wrist and elbow points.
    (pwrist, _, Jv, Jw) = wristchain.fkin(q[0:5])  # 5 joints
    (pelbow, _, _, _)   = elbowchain.fkin(q[0:4])  # 4 joints

    # Determine the wall (obstacle) "line"
    pw = np.array([0, 0, 0.3])
    dw = np.array([0, 1, 0])

    # Determine the forearm "line"
    pa = pwrist
    da = pelbow - pwrist

    # Solve for the closest point on the forearm.
    a = (pw - pa) @ np.linalg.pinv(np.vstack((-dw, np.cross(dw, da), da)))
    parm = pa + max(0, min(1, a[2])) * da

    # Solve for the matching wall point.
    pwall = pw + dw * np.inner(dw, parm-pw) / np.inner(dw, dw)

    # Compute the distance and repulsion force
    d = np.linalg.norm(parm-pwall)
    F = (parm-pwall) / d**2

    # Map the repulsion force acting at parm to the equivalent force
    # and torque actiing at the wrist point.
    Fwrist = F
    Twrist = np.cross(parm-pwrist, F)

    # Convert the force/torque to joint torques (J^T).
    tau = np.vstack((Jv, Jw)).T @ np.concatenate((Fwrist, Twrist))

    # Return the 5 joint torques as part of the 7 full joints.
    return np.concatenate((tau, np.zeros(2)))


#
#   To the Trajectory Class, add:
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the intermediate kinematic chain objects.
        self.chain5 = KinematicChain(
            node, 'world', 'link5', self.jointnames()[0:5])
        self.chain4 = KinematicChain(
            node, 'world', 'link4', self.jointnames()[0:4])
        
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        self.q0 = np.radians(np.array([0, 90, 0, -90, 0, 0, 0]))
        self.qd = self.q0
        self.p0 = np.array([0.0, 0.55, 1.0])
        self.R0 = np.eye(3)

        self.P_LEFT = np.array([0.3, 0.5, 0.15])
        self.R_LEFT = np.array(
            [[0, 0, -1],
             [1, 0, 0],
             [0, -1, 0]]
        )
        self.P_RIGHT = np.array([-0.3, 0.5, 0.15])
        self.R_RIGHT = np.eye(3)
        
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluation
    def evaluate(self, t, dt, ball_pos, ball_vel):
        """Compute the desired joint/task positions and velocities, as well as the orientation and angular velocity.

        Args:
            t (float): the current time
            dt (float): the time step
            ball_pos (array): the ball position
            ball_vel (array): the ball velocity

        Returns:
            (array, array, array, array, array, array): qd, qddot, pd, vd, Rd, wd
        """

        # print(ball_pos, ball_vel)

        # TEST CASE 1
        # pd = self.p0    # blah whatever the position doesn't really matter for this test
        # Rd = self.R0
        # vd = np.array([0, 0, 0.5 * sin(2 * pi * t)])
        # wd = np.zeros(3)

        # TEST CASE 2
        pd = self.p0
        Rd = Rotx(0.3 * np.pi)
        vd = np.zeros(3)
        wd = np.zeros(3)

        ptip, Rtip, Jv, Jw = self.chain.fkin(self.qd)

        Jac = np.vstack((Jv, Jw))
        xd_dot = np.concatenate((vd, wd))

        p_error = pd - ptip
        R_error = 0.5 * (np.cross(Rtip[:,0], Rd[:,0]) + np.cross(Rtip[:,1], Rd[:,1]) + np.cross(Rtip[:,2], Rd[:,2]))
        error = np.concatenate((p_error, R_error))

        LAM = 20
        qddot = np.linalg.pinv(Jac) @ (xd_dot + LAM * error)

        self.qd += qddot * dt
        qd = self.qd

        return (qd, qddot, pd, vd, Rd, wd)


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
