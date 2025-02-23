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

#include <iostream>
#include <fstream>

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

  TrajectoryGeneration *traj;
  std::string resource_path;

  /*
  Method that creates the desired trajectory from initial robot pose and stores it in a file trajectory_test.csv
  Inputs: initial pose of the robot
  Returns : nothing (void)
  */

  void createTestTrajectory(pinocchio::SE3 init_pose)
  {
    std::ofstream myTrajFile;
    myTrajFile.open(resource_path + "trajectory_test.csv");
    // we set the first trajectory point from the initial pose of the robot
    myTrajFile << "x,y,z,qx,qy,qz,qw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az,aroll,apitch,ayaw\n";
    for (int i = 0; i < init_pose.translation().size(); ++i)
    {
      myTrajFile << init_pose.translation()[i] << ",";
    }
    double x_init = init_pose.translation()[0];
    double y_init = init_pose.translation()[1];
    double z_init = init_pose.translation()[2];
    double deltaX = 0.1;
    double deltaZ = -0.2;
    double deltaT = 0.001;
    double x = x_init;
    double vx = 0.0;
    double vx_max = 0.05;
    double ax = 0.1;
    double z = z_init;
    double vz = 0.0;
    double vz_max = -0.05;
    double az = -0.1;
    Eigen::Quaterniond q_init;
    q_init = init_pose.rotation();
    myTrajFile << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ",";
    myTrajFile << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << ",";
    myTrajFile << ax << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    // acceleration phase
    for (int t = 0; t < int(vx_max / (ax * deltaT)); t++)
    {
      x = x + deltaT * vx;
      vx = vx + deltaT * ax;
      myTrajFile << x << "," << y_init << "," << z_init << "," << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ","
                 << vx << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << ax << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    }
    // steady speed phase
    int t_steady = (deltaX / vx_max - vx_max / ax) / deltaT;
    for (int t = 0; t < t_steady; t++)
    {
      x = x + deltaT * vx;
      myTrajFile << x << "," << y_init << "," << z_init << "," << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ","
                 << vx << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    }
    // deceleration phase
    for (int t = 0; t < int(vx_max / (ax * deltaT)); t++)
    {
      x = x + deltaT * vx;
      vx = vx + deltaT * -ax;
      myTrajFile << x << "," << y_init << "," << z_init << "," << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ","
                 << vx << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << -ax << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    }
    // acceleration phase
    for (int t = 0; t < int(vz_max / (az * deltaT)); t++)
    {
      z = z + deltaT * vz;
      vz = vz + deltaT * az;
      myTrajFile << x << "," << y_init << "," << z << "," << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ","
                 << 0.0 << "," << 0.0 << "," << vz << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << az << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    }
    // steady speed phase
    t_steady = (deltaZ / vz_max - vz_max / az) / deltaT;
    for (int t = 0; t < t_steady; t++)
    {
      z = z + deltaT * vz;
      myTrajFile << x << "," << y_init << "," << z << "," << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ","
                 << 0.0 << "," << 0.0 << "," << vz << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    }
    // deceleration phase
    for (int t = 0; t < int(vz_max / (az * deltaT)); t++)
    {
      z = z + deltaT * vz;
      vz = vz + deltaT * -az;
      myTrajFile << x << "," << y_init << "," << z << "," << q_init.x() << "," << q_init.y() << "," << q_init.z() << "," << q_init.w() << ","
                 << 0.0 << "," << 0.0 << "," << vz << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << 0.0 << "," << -az << "," << 0.0 << "," << 0.0 << "," << 0.0 << "\n";
    }
    myTrajFile.close();
  }

  void initController() override
  {
    model =
        Model::RobotModel<Model::RobotModelImplType::PINOCCHIO>::loadModelFromFile(resource_path + "robot.urdf");

    velocity_problem = std::make_shared<Qontrol::JointVelocityProblem>(model);
    main_task = velocity_problem->task_set->add<Task::CartesianVelocity>("MainTask");
    auto regularisation_task = velocity_problem->task_set->add<Task::JointVelocity>("RegularisationTask", 1e-5);

    auto joint_configuration_constraint = velocity_problem->constraint_set->add<Constraint::JointConfiguration>("JointConfigurationConstraint");
    auto joint_velocity_constraint = velocity_problem->constraint_set->add<Constraint::JointVelocity>("JointVelocityConstraint");
    // Is this necessary?

    robot_state.joint_position.resize(model->getNrOfDegreesOfFreedom());
    robot_state.joint_velocity.resize(model->getNrOfDegreesOfFreedom());
    for (int i = 0; i < model->getNrOfDegreesOfFreedom(); ++i)
    {
      robot_state.joint_position[i] = d->qpos[i];
      robot_state.joint_velocity[i] = d->qvel[i];
    }
    model->setRobotState(robot_state);

    pinocchio::SE3 init_pose(model->getFramePose(model->getTipFrameName()).matrix());
    this->createTestTrajectory(init_pose);

    traj = new TrajectoryGeneration(resource_path + "trajectory_test.csv", m->opt.timestep);
  }

  void updateController() override
  {

    for (int i = 0; i < model->getNrOfDegreesOfFreedom(); ++i)
    {
      robot_state.joint_position[i] = d->qpos[i];
      robot_state.joint_velocity[i] = d->qvel[i];
    }
    model->setRobotState(robot_state);

    traj->update();
    pinocchio::SE3 traj_pose(traj->pose.matrix());

    pinocchio::SE3 current_pose(model->getFramePose(model->getTipFrameName()).matrix());

    /*Linear_t &translation = current_pose.translation();
    std::cout << "current pose x: " << std << endl;*/

    const pinocchio::SE3 tipMdes = current_pose.actInv(traj_pose);
    auto err = pinocchio::log6(tipMdes).toVector();
    Eigen::Matrix<double, 6, 1> p_gains;
    p_gains << 10, 10, 10, 10, 10, 10;
    Eigen::Matrix<double, 6, 1> xd_star = p_gains.cwiseProduct(err);

    main_task->setTargetVelocity(xd_star);

    velocity_problem->update(m->opt.timestep);

    if (velocity_problem->solutionFound())
    {
      sendJointVelocity(velocity_problem->getJointVelocityCommand());
    }
  }
};

int main(int argc, const char **argv)
{
  MujocoQontrol mujoco_qontrol;
  Qontrol::Log::Logger::parseArgv(argc, argv);

  // I want to simplify the launch of the simulation
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
      &scn, &cam, &opt, &pert, /* fully_managed = */ true);

  std::string robot = argv[1];
  std::string mujoco_model = "./resources/" + robot + "/scene.xml";
  mujoco_qontrol.resource_path = "./resources/" + robot + "/";

  // start physics thread
  std::thread physicsthreadhandle(&MujocoQontrol::PhysicsThread, mujoco_qontrol, sim.get(), mujoco_model);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
