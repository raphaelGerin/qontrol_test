import pinocchio as pin
import numpy as np
import time

from example_robot_data import load

robot = load('panda')

# Display a robot configuration.
# q0 = pin.neutral(robot.model)
q0 = robot.q0

q_max = robot.model.upperPositionLimit.T
q_min = robot.model.lowerPositionLimit.T

# set a random joint configuration
q = np.random.uniform(q_min,q_max)

# calculate the jacobian
data = robot.model.createData()
#pin.framesForwardKinematics(robot.model,data,q0)
pin.computeJointJacobians(robot.model,data, q)
J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
# use only position jacobian
J = J[:3,:]


# polytope python module
import pycapacity.robot as pycap
# get max torque
t_max = robot.model.effortLimit
t_min = -t_max
#options
opt = {
  'calculate_faces': True,
}
# calculate force polytope
f_poly =  pycap.force_polytope(J, t_max, t_min,options=opt)


# plotting the polytope
import matplotlib.pyplot as plt
from pycapacity.visual import * # pycapacity visualisation tools

## visualise the robot
from pinocchio.visualize import MeshcatVisualizer

viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
# Start a new MeshCat server and client.
viz.initViewer(open=True)
# Load the robot in the viewer.
viz.loadViewerModel()
viz.display(q)
time.sleep(0.2) 

fig = plt.figure()
# draw faces and vertices
plot_polytope(plot=plt, polytope=f_poly, label='force polytope', vertex_color='blue', face_color='blue', edge_color='blue', alpha=0.2)

plt.legend()
plt.show()
