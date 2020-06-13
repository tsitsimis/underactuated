# Underactuated Robotics

Python implementations of Optimal Control, Optimization and Reinforcement Learning algorithms
applied in underactuated robots and dynamical systems.

Inspired by the MIT's course [6.832 - Underactuated Robotics](http://underactuated.mit.edu/underactuated.html) 
of [Russ Tedrake](http://groups.csail.mit.edu/locomotion/russt.html).
The corresponding [edX course](https://courses.edx.org/courses/course-v1:MITx+6.832x_2+3T2015/course/) as well as the course notes largely assisted in studying the mathematical background of the showcased
algorithms.

## Implementations

### Rigid-Body Dynamical Systems

For benchmarking and simulation purposes, some well-known 
low-dimensional systems (frequently used in the literature) are
implemented. Both fully-actuated and underactuated versions
are tested.

<table>
  <tr>
    <td><img src="./assets/acrobot_passive.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/pendulum_passive.gif" width="200px" /></td>
    <td></td>
    <td><img src="./assets/cartpole_passive.gif" width="200px" /></td>
  </tr>
</table>

### Feedback Linearization

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

### LQR Stabilization with Linearized Dynamics

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
