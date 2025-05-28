from roboticstoolbox import DHRobot,RevoluteDH
import roboticstoolbox as rtb
from numpy import pi


robot = DHRobot([
        RevoluteDH(a=4.0),
        RevoluteDH(a=2.0),
        RevoluteDH(a=0.5)
    ], name="RRR")

print(robot)

q1 = [0,0,0]
q2 = [pi/2,-pi/4,pi]
traj = rtb.jtraj(q1, q2, 100)
print(traj)
robot.plot(traj.q, block=True)