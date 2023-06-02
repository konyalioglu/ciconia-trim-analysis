## Trim Analysis

In general aviation terms, trimming is performed to bring the act of
altering trim tabs or artificial feel systems in order to remove the
pressure on the control stick (or pedals) that the pilot is supposed to
maintain during different flight conditions. When the aircraft is
properly trimmed, the flight controls are in a position where the
straight and level flight can be maintained without the need for force.
The goal of trimming is to achieve equilibrium between the forces and
moments acting on the aircraft. That occurs when the roll pitch and yaw
moments, as well as the axial normal and side forces, are all equal to
zero. The dynamic stability characteristics of the motion that occurs
after such a disturbance are specified, and the stable aircraft will
finally return to its equilibrium condition which occurs in steady-state
flight. In a variety of engineering research, determining aircraft
steady-state flight conditions or also trimmed states is of great
importance. Usually, trim points are used to derive linear models.
Linearization is performed around those trim states.

In general, trim analysis is defined as an optimization problem. The
state vector for the trim states is defined as two different parts which
are dynamic and kinematic as it is seen in equations (4.79) and (4.80).

$$\mathbf{x}_{\mathbf{d}}\mathbf{=}{\mathbf{(}u,v,w,p,q,r)}^{\mathbf{T}}$$

$$\mathbf{x}_{\mathbf{k}}\mathbf{=}{\mathbf{(}x,y,z,\phi,\ \theta,\ \psi)}^{\mathbf{T}}$$

$\mathbf{x}_{\mathbf{d}}$ is the dynamic state vector and
$\mathbf{x}_{\mathbf{k}}$ is a kinematic state vector. Also, the control
input is defined in equation (4.81), for the forward flight.

$$\mathbf{u}_{\mathbf{\text{ff}}} = {(\delta_{T},\ \text{δe},\ \text{δa},\ \text{δr})}^{T}$$

Where $\delta_{T}$ is throttle setting, $\text{δe}$ is elevator
deflection, $\text{δa}$ is aileron deflection and $\text{δr}$ is rudder
deflection. Because, in the transition state, the propulsion system for
the hover state will also be used, the control input vector becomes as
in equation (4.82).

$$\mathbf{u}_{\mathbf{\text{tr}}} = {(\delta_{T},\ \text{δe},\ \text{δa},\ \text{δr},\ u1,u2,u3,u4)}^{T}$$

$u1,u2,u3,$ and $u4$ are defined as *the* force in the x-axis, the force
that creates the pitching moment, and the force that creates the rolling
moment and yawing moment, respectively. Also, one can define the
implicit form of the non-linear aircraft state equation for the trim
state in equation (4.83).

$$g(\dot{\mathbf{x}},\mathbf{x},\mathbf{u}) = 0$$

Before establishing, the cost function for the optimization problem,
necessary parameters should be constructed. Translational dynamic,
rotational dynamic, translational kinematic, and rotational kinematic
cost function parameters are given in equations (4.84), (4.85), (4.86),
and (4.87), respectively.

$$f_{TD1} = \dot{u},\ \ f_{TD2} = \dot{v},\ \ f_{TD3} = \dot{w}$$

$$f_{RD1} = \dot{p},\ \ f_{RD2} = \dot{q},\ \ f_{RD3} = \dot{r}$$

$$f_{TK1} = \dot{x},\ \ f_{TK2} = \dot{y},\ \ f_{TK3} = \dot{z}$$

$$f_{RK1} = \dot{\phi},\ \ f_{RK2} = \dot{\theta},\ \ f_{RK3} = \dot{\psi}$$
