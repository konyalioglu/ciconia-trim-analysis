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

**x**<sub>**d**</sub>**=****(***u*, *v*, *w*, *p*, *q*, *r*)<sup>**T**</sup>

**x**<sub>**k**</sub>**=****(***x*, *y*, *z*, *ϕ*, *θ*, *ψ*)<sup>**T**</sup>

**x**<sub>**d**</sub> is the dynamic state vector and
**x**<sub>**k**</sub> is a kinematic state vector. Also, the control
input is defined in equation (4.81), for the forward flight.

**u**<sub>ff</sub> = (*δ*<sub>*T*</sub>, δe, δa, δr)<sup>*T*</sup>

Where *δ*<sub>*T*</sub> is throttle setting, δe is elevator deflection,
δa is aileron deflection and δr is rudder deflection. Because, in the
transition state, the propulsion system for the hover state will also be
used, the control input vector becomes as in equation (4.82).

**u**<sub>tr</sub> = (*δ*<sub>*T*</sub>, δe, δa, δr, *u*1, *u*2, *u*3, *u*4)<sup>*T*</sup>

*u*1, *u*2, *u*3, and *u*4 are defined as *the* force in the x-axis, the
force that creates the pitching moment, and the force that creates the
rolling moment and yawing moment, respectively. Also, one can define the
implicit form of the non-linear aircraft state equation for the trim
state in equation (4.83).

$$g(\\dot{\\mathbf{x}},\\mathbf{x},\\mathbf{u}) = 0$$

Before establishing, the cost function for the optimization problem,
necessary parameters should be constructed. Translational dynamic,
rotational dynamic, translational kinematic, and rotational kinematic
cost function parameters are given in equations (4.84), (4.85), (4.86),
and (4.87), respectively.

*f*<sub>*T**D*1</sub> = *u̇*,  *f*<sub>*T**D*2</sub> = *v̇*,  *f*<sub>*T**D*3</sub> = *ẇ*

*f*<sub>*R**D*1</sub> = *ṗ*,  *f*<sub>*R**D*2</sub> = *q̇*,  *f*<sub>*R**D*3</sub> = *ṙ*

*f*<sub>*T**K*1</sub> = *ẋ*,  *f*<sub>*T**K*2</sub> = *ẏ*,  *f*<sub>*T**K*3</sub> = *ż*

*f*<sub>*R**K*1</sub> = *ϕ̇*,  *f*<sub>*R**K*2</sub> = *θ̇*,  *f*<sub>*R**K*3</sub> = *ψ̇*
