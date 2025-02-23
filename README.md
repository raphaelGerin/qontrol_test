# qontrol_test

## 1-Qontrol

The first part of the test is about basic handling and understanding of the qontrol tool.

### Objective

The objective is to install qontrol and modify one example to move a robot (panda or ur5) by 0.1m along x axis.
Then by -0.2m along z axis.

### Installation

The installation went smoothly, however we could only launch the examples with the ur5 robot (simulation instable for the panda).

### Choice of the example

A set of four examples are explained in [qontrol documentation](https://auctus-team.gitlabpages.inria.fr/components/control/qontrol/md_doc_b-examples_intro.html): torque control, velocity control, custom task and custom constraint.
For the torque control and the velocity exmaples,the robot main tasks consists in following a simple trajectory defined in Cartesian space.
Thus, we arbitrarily choose to use the velocity control (velocityQontrol.cpp). 

### Working principle

The example runs using a predefined trajectory in a csv file (trajectory.csv). Each ms, the controller computes the error between the current robot pose and the trajectory pose. Then a proportionnal controller is used to defined the desired cartesian velocity to stay on the trajectory (reduce the error). Once we get the desired cartesian velocity, we solve the quadratic problem stated [here](https://auctus-team.gitlabpages.inria.fr/components/control/qontrol/md_doc_examples_velocity_qontrol.html) to compute the corresponding joints velocities. Note that a regularisation task is used to find the lowest joints velocities that achieve the goal.

### Trajectory generation

To achieve the objective, we need to modify the trajectory. Thus, we created a method "createTestTrajectory" that takes a argument the initial robot pose and from it, generates the desired trajectory in a new file "trajectory_test.csv".
For the trajectory generation, we used a classical trapezoidal velocity profile to ensure smooth movement.

We first starts with the move along x axis : 
We used as fixed parameters the maximum cartesian velocity $v_{x,max}\neq 0$ and the acceleration $a_{x}\neq 0$. 
$a_{x}$ value can vary according to the phase : its value is $a_{x}$ during acceleration phase, 0 during speed cruise phase and -$a_{x}$ during deceleration phase.
The time required to reach the cruise velocity is $t1 = \frac{v_{x,max}}{a_{x}}$. It is same for the decelration duration.
We must chose $v_{x,max}$ and $a_{x}$ st $\frac{v_{x,max}^2}{a_{x}} \leq \Delta_{x}$, with $\Delta_{x}$ the desired displacement along x axis.
Then, we must compute the duration of the cruise velocity phase (t_steady). We want the integral of the cartesian velocity to be equal to $\Delta_{x}$.
Integrating our trapezoidal velocity profile we get: $\frac{v_{x,max}^2}{a_{x}} + t_{steady}v_{x,max} = \Delta_{x}$.
So we have $t_{steady} = \frac{\Delta_{x}}{v_{x,max}} - \frac{v_{x,max}}{a_{x}}$.

Finally, we use the following equations to compute the velocity and the position of the trajectory :

$v_{x}(t_{k+1}) = v_{x}(t_{k}) + a_{x}*Delta_{T}$ where $Delta_{T}$=1ms here.

$x(t_{k+1}) = x(t_{k}) + v_{x}*Delta_{T}$ where $Delta_{T}$=1ms here.

We replicate this method for the movement along z axis, taking into account the new x pose of the robot.

### Results

With $v_{x,max}$ = 0.05m/s, $a_{x} = 0.1m/s²$, $v_{z,max}$ = -0.05m/s and $a_{z} = -0.1m/s²$ , the robot moves smoothly and achieve follows the desired trajectory.


## 2-Pycapacity