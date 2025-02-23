import pinocchio as pin
import numpy as np
import time
from example_robot_data import load

robot = load('ur10')

# Display a robot configuration.
# q0 = pin.neutral(robot.model)
q0 = robot.q0

# calculate the jacobian
data = robot.model.createData()
pin.framesForwardKinematics(robot.model,data,q0)

# set the time-horizon
horizon_time = 0.2 # secs

# create a simple forward kinematics function
# taking the current joint configuration q
# and outputting the end-effector position
def fk(q):
    pin.framesForwardKinematics(robot.model,data,q)
    return data.oMf[robot.model.getFrameId(robot.model.frames[-1].name)].translation

# polytope python module
import pycapacity.robot as pycap
# get max torque
q_max = robot.model.upperPositionLimit.T
q_min = robot.model.lowerPositionLimit.T
# get max velocity
dq_max = robot.model.velocityLimit
dq_min = -dq_max

# set a random joint configuration
q = np.random.uniform(q_min,q_max)
q = q0
print(q)

opt = {
  'calculate_faces': True,
  'convex_hull': False, # if True, the reachable space will be approximated with a convex hull of the vertices (does not require CGAL)
  'n_samples':3,     # number of samples per dimension of the facet (the higher the better the approximation - n_samples^facet_dim samples)
  'facet_dim':2        # dimension of the joint-space facet to be sampled (0 for vertices, 1 for edges, 2 for faces, up to n_dof -1, where n_dof is the number of degrees of freedom)
}

# calculate force polytope
rw_poly = pycap.reachable_space_nonlinear(
            forward_func=fk,
            q0=q,
            q_max= q_max, 
            q_min= q_min,
            dq_max= dq_max,
            dq_min= dq_min,
            time_horizon=horizon_time,
            options=opt
    )


## visualise the robot
from pinocchio.visualize import MeshcatVisualizer

viz = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
# Start a new MeshCat server and client.
viz.initViewer(open=True)
# Load the robot in the viewer.
viz.loadViewerModel()
viz.display(q)
# small time window for loading the model 
# if meshcat does not visualise the robot properly, augment the time
# it can be removed in most cases
time.sleep(0.2) 

## visualise the polytope and the ellipsoid
import meshcat.geometry as g 

# meshcat triangulated mesh
poly = g.TriangularMeshGeometry(vertices=rw_poly.vertices.T, faces=rw_poly.face_indices)
viz.viewer['rwspace'].set_object(poly, g.MeshBasicMaterial(color=0x0022ff, wireframe=True, linewidth=3, opacity=0.2))

##second configuration
q = np.array([4.01182072, -1.81114319, 2.32129792, -0.959301, 1.17508946, 2.67894255])
# calculate force polytope
rw_poly2 = pycap.reachable_space_nonlinear(
            forward_func=fk,
            q0=q,
            q_max= q_max, 
            q_min= q_min,
            dq_max= dq_max,
            dq_min= dq_min,
            time_horizon=horizon_time,
            options=opt
    )

viz2 = MeshcatVisualizer(robot.model, robot.collision_model, robot.visual_model)
# Start a new MeshCat server and client.
viz2.initViewer(open=True)
# Load the robot in the viewer.
viz2.loadViewerModel()
viz2.display(q)
# small time window for loading the model 
# if meshcat does not visualise the robot properly, augment the time
# it can be removed in most cases
time.sleep(0.2) 
# meshcat triangulated mesh
poly2 = g.TriangularMeshGeometry(vertices=rw_poly2.vertices.T, faces=rw_poly2.face_indices)
viz2.viewer['rwspace'].set_object(poly2, g.MeshBasicMaterial(color=0x0022ff, wireframe=True, linewidth=3, opacity=0.2))
