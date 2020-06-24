# Underactuated Robotics

Python implementations of Control and Optimization algorithms 
for simulated underactuated systems and walking robots

Inspired by the MIT's course [6.832 - Underactuated Robotics](http://underactuated.mit.edu/underactuated.html) 
of [Russ Tedrake](http://groups.csail.mit.edu/locomotion/russt.html).
The corresponding [edX course](https://courses.edx.org/courses/course-v1:MITx+6.832x_2+3T2015/course/) as well as the course notes largely assisted in studying the mathematical background of the showcased
algorithms.

## Implementations

## Rigid-Body Dynamical Systems

For benchmarking and simulation purposes, some well-known 
low-dimensional systems (frequently used in the literature) are
implemented. Both fully-actuated and underactuated versions
are tested.

<table>
  <th>Acrobot</th>
  <th></th>
  <th>Simple Pendulum</th>
  <th></th>
  <th>Cart-Pole</th>
  <tr>
    <td><img src="./assets/acrobot_passive.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/pendulum_passive.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/cartpole_passive.gif" width="200px" /></td>
  </tr>
  
  <th>Rimless Wheel</th>
  <tr>
    <td><img src="./assets/rimlesswheel_passive.gif" width="200px" /></td>
  </tr>
</table>

## Fully-actuated systems
#### Feedback Linearization

Fully-actuated control becomes trivial when using feedback linearization
by cancelling-out the complex dynamics of the system and converting it
to a linear system (feedback equivalent).

For a second-order control-affine system

<img src="https://render.githubusercontent.com/render/math?math=\ddot{q} = f_1(q, \dot{q}) %2B f_2(q, \dot{q})u">  

the feedback control 

<img src="https://render.githubusercontent.com/render/math?math=u = f_2^{-1}(q, \dot{q})(v - f_1(q, \dot{q})">

makes the system equivalent to <img src="https://render.githubusercontent.com/render/math?math=\ddot{q} = v"> allowing 
us to plug-in a simple PD controller with pole placement and transition the system
to the desired state. 

Swing-up control for pendulum and acrobot:

<table>
  <tr>
    <td><img src="./assets/acrobot_feedback_linearization.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/pendulum_feedback_linearization.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/cartpole_feedback_linearization.gif" width="200px" /></td>
  </tr>
</table>

## Underactuated systems
#### LQR Stabilization with Linearized Dynamics

Stabilize a non-linear system around a fixed point by applying a Linear Quadratic Regulator (LQR)
on a linearized version of the system. The system's initial state needs to be in the region of 
attraction of the fixed point.

LQR is a feedback controller derived from Dynamic Programming for linear systems and quadratic cost
on state and control signal.  

<img src="https://render.githubusercontent.com/render/math?math=u = -Kx = -R^{-1}B^{T}Sx">

<table>
  <tr>
    <td><img src="./assets/acrobot_lqr_stabilization.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/pendulum_lqr_stabilization.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/cartpole_lqr_stabilization.gif" width="200px" /></td>
  </tr>
</table>

#### Energy Shaping with Partial Feedback Linearization (PFL)
To apply LQR, we first have to bring the system near the fixed point.
To achieve this, energy shaping controller pumps or draws energy from/to 
the system until it reaches the energy of the desired state. This is
usually done by calculating the error between the
system's energy <img src="https://render.githubusercontent.com/render/math?math=E">
and the desired energy <img src="https://render.githubusercontent.com/render/math?math=E^{d}">
(that is <img src="https://render.githubusercontent.com/render/math?math=\tilde{E}=E-E^{d}">)
and expressing its time derivative in the form

<img src="https://render.githubusercontent.com/render/math?math=\dot{\tilde{E}} = \dot{E} = u \cdot h(x, \dot{x})">

Then, the control signal
<img src="https://render.githubusercontent.com/render/math?math=u = -k \cdot h(x, \dot{x}) \cdot \tilde{E}">
guarantees that the energy error is non-increasing (since <img src="https://render.githubusercontent.com/render/math?math=\dot{\tilde{E}} = -k \cdot h^{2}(x, \dot{x}) \cdot \tilde{E}">).

Theoretically, this should suffice but small perturbations (either physical
or due to numerical errors) can move the system far from the goal state.

In this situation, once the system is near the fixed point (inside its
basin of attraction) the controller can switch from energy shaping to LQR
and stabilize the system.

<table>
  <tr>
    <td><img src="./assets/cartpole_energy_shaping.gif" width="200px" /></td>
    <td></td>
  </tr>
</table>

## Walking systems
#### Rimless Wheel

<img src="./assets/rimlesswheel_passive.gif" width="300px" />

The simplest model of legged robot. Assumes there will always 
be a swing leg in position at the time of collision. It uses pendula
for legs and moves only thanks to the gravity. With appropriate 
initial leg angle and angular velocity the wheel reaches periodic
stability (limit cycle) and walks down the slope.

The foot standing on the ground is modeled with a simple pendulum (`systems/pendulum.py`)
with initial angle near the upward position. Using the simple
collision detection logic that the pendulum's angle (from the upward position)
is <img src="https://render.githubusercontent.com/render/math?math=\theta = \alpha %2B \gamma">
the pendulum's position is changed during the simulation and is
placed in the position of next leg to hit the ground.
