// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mujoco/mujoco_sim.h"
#include "Qontrol/Qontrol.hpp"
#include "trajectory_generation/trajectory_generation.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"


using namespace Qontrol;

class MujocoQontrol : public MujocoSim 
{
public:
//------------------------------------------- simulation -------------------------------------------
std::shared_ptr<Qontrol::Model::RobotModel<Qontrol::Model::RobotModelImplType::PINOCCHIO>> model;
std::shared_ptr<Qontrol::JointVelocityProblem> velocity_problem;
std::shared_ptr<Qontrol::Task::CartesianVelocity<Qontrol::ControlOutput::JointVelocity>> main_task;
pinocchio::SE3 init_pose;

Qontrol::RobotState robot_state;

TrajectoryGeneration* traj;
std::string resource_path;
void initController() override
{
  model =
      Model::RobotModel<Model::RobotModelImplType::PINOCCHIO>::loadModelFromFile(resource_path+"robot.urdf");
  auto solver =
      std::make_shared<Solver::Solver<Solver::SolverImplType::qpmad>>();
  velocity_problem = std::make_shared<Qontrol::JointVelocityProblem>(model,solver);  
  main_task = velocity_problem->task_set->add<Task::CartesianVelocity>("MainTask"); // <-- Based on the template given will implement correct task representation  
  auto regularisation_task = velocity_problem->task_set->add<Task::JointVelocity>("RegularisationTask",1e-5); // <-- Based on the template given will implement correct task representationn

  auto joint_configuration_constraint = velocity_problem->constraint_set->add<Constraint::JointConfiguration>("JointConfigurationConstraint");
  auto joint_velocity_constraint = velocity_problem->constraint_set->add<Constraint::JointVelocity>("JointVelocityConstraint");

  mju_copy(d->qpos, m->key_qpos, m->nu);
  robot_state.joint_position.resize(model->getNrOfDegreesOfFreedom());
  robot_state.joint_velocity.resize(model->getNrOfDegreesOfFreedom());
  for (int i=0; i<model->getNrOfDegreesOfFreedom() ; ++i)
  {
    robot_state.joint_position[i] = d->qpos[i];
    robot_state.joint_velocity[i] = d->qvel[i];
  }
  model->setRobotState(robot_state);

  traj = new TrajectoryGeneration(resource_path+"trajectory.csv", m->opt.timestep);
}

void updateController() override
{

  for (int i=0; i<model->getNrOfDegreesOfFreedom() ; ++i)
  {
    robot_state.joint_position[i] = d->qpos[i];
    robot_state.joint_velocity[i] = d->qvel[i];
  }
  model->setRobotState(robot_state);

  pinocchio::SE3 current_pose(model->getFramePose(model->getTipFrameName()).matrix());
  traj->update();
  pinocchio::SE3 traj_pose(traj->pose.matrix());
  const pinocchio::SE3 tipMdes = current_pose.actInv(traj_pose);
  auto err = pinocchio::log6(tipMdes).toVector();
  Eigen::Matrix<double,6,1> p_gains;
  p_gains << 10,10,10,10,10,10;
  Eigen::Matrix<double,6,1> xd_star = p_gains.cwiseProduct(err);
  main_task->setTargetVelocity(xd_star);
  velocity_problem->update(m->opt.timestep);

  if (velocity_problem->solutionFound())
  {
    sendJointVelocity(velocity_problem->getJointVelocityCommand());    
  }}
 
};

int main(int argc, const char** argv) {
  MujocoQontrol mujoco_qontrol;
  Qontrol::Log::Logger::parseArgv(argc, argv);

  mjvScene scn;
  mjv_defaultScene(&scn);

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &scn, &cam, &opt, &pert, /* fully_managed = */ true
  );


  std::string robot = argv[1];
  std::string mujoco_model = "./resources/"+robot+"/scene.xml";
  mujoco_qontrol.resource_path = "./resources/"+robot+"/";

  // start physics thread
  std::thread physicsthreadhandle( &MujocoQontrol::PhysicsThread, mujoco_qontrol, sim.get(), mujoco_model);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}