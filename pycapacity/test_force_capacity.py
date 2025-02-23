

import pinocchio as pin
import numpy as np
import time

from example_robot_data import load

# import pycapacity 
import pycapacity as pycap

# get panda robot usinf example_robot_data
robot = load('panda')

# get joint position ranges
q_max = robot.model.upperPositionLimit.T
q_min = robot.model.lowerPositionLimit.T
# get max velocity
dq_max = robot.model.velocityLimit
dq_min = -dq_max

# Use robot configuration.
# q0 = np.random.uniform(q_min,q_max)
# set a joint configuration, comment on of them
q0 = np.array([-1.60559707,1.16204607,1.54152204, -1.73219968, -1.78493679,3.52859806,-2.45956125, 0.0064951, 0.01330165])
#q0 = np.array([-1.75000774, -1.49489362,  2.11747985, -1.13312843, -2.32413887, 2.53417134, 1.86947326, 0.0263245626, 0.00203188366])

# calculate the jacobian
data = robot.model.createData()
pin.framesForwardKinematics(robot.model,data,q0)
pin.computeJointJacobians(robot.model,data, q0)
J = pin.getFrameJacobian(robot.model, data, robot.model.getFrameId(robot.model.frames[-1].name), pin.LOCAL_WORLD_ALIGNED)
# use only position jacobian
J = J[:3,:]

# end-effector pose
Xee = data.oMf[robot.model.getFrameId(robot.model.frames[-1].name)]

## visualise the robot
from pinocchio.visualize import MeshcatVisualizer

viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
# Start a new MeshCat server and client.
viz.initViewer(open=True)
# Load the robot in the viewer.
viz.loadViewerModel()
viz.display(q0)
# small time window for loading the model 
# if meshcat does not visualise the robot properly, augment the time
# it can be removed in most cases
time.sleep(0.2) 

## visualise the polytope and the ellipsoid
import meshcat.geometry as g 

# calculate the polytope
opt = {'calculate_faces':True}
# calculate the polytope
vel_poly = pycap.robot.velocity_polytope(J, dq_min, dq_max,options=opt)
# meshcat triangulated mesh
poly = g.TriangularMeshGeometry(vertices=vel_poly.vertices.T/10 + Xee.translation, faces=vel_poly.face_indices)
viz.viewer['poly'].set_object(poly, g.MeshBasicMaterial(color=0x0022ff, wireframe=True, linewidth=3, opacity=0.2))

# calculate the ellipsoid
vel_ellipsoid = pycap.robot.velocity_ellipsoid(J, dq_max)
# meshcat ellipsoid
ellipsoid = g.Ellipsoid(radii=vel_ellipsoid.radii/10)
viz.viewer['ellipse'].set_object(ellipsoid, g.MeshBasicMaterial(color=0xff5500, transparent=True, opacity=0.2))
viz.viewer['ellipse'].set_transform(pin.SE3(vel_ellipsoid.rotation, Xee.translation).homogeneous)