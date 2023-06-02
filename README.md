# CICONIA AIRCRAFT TRIM ANALYSIS



## Usage


## Dependencies

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

$$\mathbf{x}_{\mathbf{d}}\mathbf{=}\left( u,v,w,p,q,r \right)^{\mathbf{T}}$$

$$\mathbf{x}_{\mathbf{k}}\mathbf{=}\left( x,y,z,\phi,\theta,\psi \right)^{\mathbf{T}}$$

$\mathbf{x}_{\mathbf{d}}$ is the dynamic state vector and
$\mathbf{x}_{\mathbf{k}}$ is a kinematic state vector. Also, the control
input is defined in equation (4.81), for the forward flight.

$$\mathbf{u}_{\mathbf{f}} = \left( \delta_{T},\text{δe},\text{δa},\text{δr} \right)^{T}$$

Where $\delta_{T}$ is throttle setting, $\text{δe}$ is elevator
deflection, $\text{δa}$ is aileron deflection and $\text{δr}$ is rudder
deflection. Because, in the transition state, the propulsion system for
the hover state will also be used, the control input vector becomes as
in equation (4.82).

$$\mathbf{u}_{\mathbf{t}} = \left( \delta_{T},\text{δe},\text{δa},\text{δr},u1,u2,u3,u4 \right)^{T}$$

$u1,u2,u3,$ and $u4$ are defined as *the* force in the x-axis, the force
that creates the pitching moment, and the force that creates the rolling
moment and yawing moment, respectively. Also, one can define the
implicit form of the non-linear aircraft state equation for the trim
state in equation (4.83).

$$g\left( \dot{\mathbf{x}},\mathbf{x},\mathbf{u} \right) = 0$$

Before establishing, the cost function for the optimization problem,
necessary parameters should be constructed. Translational dynamic,
rotational dynamic, translational kinematic, and rotational kinematic
cost function parameters are given in equations (4.84), (4.85), (4.86),
and (4.87), respectively.

$$f_{TD1} = \dot{u},\ \ f_{TD2} = \dot{v},\ \ f_{TD3} = \dot{w}$$

$$f_{RD1} = \dot{p},\ \ f_{RD2} = \dot{q},\ \ f_{RD3} = \dot{r}$$

$$f_{TK1} = \dot{x},\ \ f_{TK2} = \dot{y},\ \ f_{TK3} = \dot{z}$$

$$f_{RK1} = \dot{\phi},\ \ f_{RK2} = \dot{\theta},\ \ f_{RK3} = \dot{\psi}$$

However, considering the construction of the trim problem in order to
add flight path angle constraint to the cost function, $f_{TK3}$ can be
written as

$$f_{TK3} = sin\gamma - \lbrack(cos\alpha\ cos\beta\ sin\theta) - (sin\beta\ cos\theta\ sin\phi) - (sin\alpha\ cos\beta\ cos\theta\ cos\phi)\rbrack$$

These cost function parameters are obtained by using the equation of
motion of the hybrid VTOL aircraft as it was discussed in section 4.2.1.
For general straight flight trim analysis, the trim states are chosen as
shown in equation (4.89).

$$\mathbf{\zeta} = {(V_{T},\ \beta,\ \alpha,\ (u,\ v,\ w),\ p,\ q,\ r,\ \phi,\ \theta,\ \psi,\ h,\gamma,\ \delta_{T},\ \text{δe},\ \text{δa},\ \text{δr})}^{T}$$

Where specified and unknown trim state vectors are given in equations
(4.90) and (4.91),

$$\mathbf{\zeta}_{\text{specified}} = {(V_{T},\ h,\gamma,\ \phi)}^{T}$$

$$\mathbf{\zeta}_{\text{unknown}} = {(\ \beta,\ \alpha,\ p,\ q,\ r,\ \theta,\ \psi,\delta e,\ \text{δa},\ \delta r,\delta_{T})}^{T}$$

$\zeta_{\text{specified}}$ is actually defined as the trim condition of
the aircraft. $h$ determines the atmospheric conditions, $\gamma$
determines whether the aircraft descending or ascending. If a turn state
is analyzed, $\phi$ term can be given as input other than zero. The
generalized trim cost function is defined in equation (4.92).

$$J = f_{TD1}^{2} + f_{TD2}^{2} + f_{TD3}^{2} + f_{RD1}^{2} + f_{RD2}^{2} + f_{RD3}^{2} + f_{RK1}^{2} + f_{RK2}^{2} + f_{RK3}^{2} + f_{TK2}^{2} + f_{TK3}^{2}$$

Then, the optimization problem can be solved with 11 unknown and 11
known variables. The trim analysis is also run for transition dynamics
for the aircraft. Additional variables are the propulsion system for the
hover state of the aircraft. However, in order to make a number of
unknown and known equal some changes should be done. Thus, the
deflection of control surfaces is specified as 0 deflections. Also, one
additional parameter which is $\theta$, is set to zero. Additionally,
moment and forces are considered to be compensated by the hover
propulsion system.

$$\mathbf{\zeta} = {(V_{T},\ \beta,\ \alpha,\ (u,\ v,\ w),\ p,\ q,\ r,\ \phi,\ \theta,\ \psi,\ h,\gamma,\ \delta_{T},u1,u2,u3,u4)}^{T}$$

Where specified and unknown trim state vectors are given in equations
(4.94) and (4.95).

$$\mathbf{\zeta}_{\text{specified}} = {(V_{T},\ h,\gamma,\ \phi,\ \theta = 0,\ \delta e = 0,\ \delta a = 0,\ \delta r = 0)}^{T}$$

$$\mathbf{\zeta}_{\text{unknown}} = \left( \beta,\alpha,p,q,r,\theta,\psi,\delta e,\text{δa},\delta r,\delta_{T},u1,u2,u3,u4 \right)^{T}$$

The cost function for transition trim analysis is defined the same as
the general straight flight trim analysis which is established in
equation (4.92).

On the other hand, another trim analysis type which is steady-state turn
flight is performed in this thesis. Additional to the general straight
flight trim problem, the side-slip angle, $\beta$, is specified, body
rates are removed, and yaw rate is left optional to user definition,
regarding the trim state vector as it is seen in equation (4.96) and
(4.97).

$$\mathbf{\zeta}_{\text{specified}} = {(V_{T},\ h,\gamma,\beta,\ \phi\ (or\ \ \dot{\psi}))}^{T}$$

$$\mathbf{\zeta}_{\text{unknown}} = {(\alpha,\ \theta,\ \dot{\psi}\ (or\ \phi)\ ,\delta e,\ \text{δa},\ \delta r,\delta_{T})}^{T}$$

Therefore, in the trim state vector, there are seven unknown variables.
Because turning concerns the variables in the planar position, related
constraints are removed from the cost function as it is seen in equation
(4.98).

$$J = f_{TD1}^{2} + f_{TD2}^{2} + f_{TD3}^{2} + f_{RD1}^{2} + f_{RD2}^{2} + f_{RD3}^{2} + f_{TK3}^{2}$$

Therefore, for the optimization problem, there are seven unknown
parameters to be solved by using another seven known parameters.

<img src="media/image1.png" style="width:5.66783in;height:1.06796in" alt="Graphical user interface, diagram Description automatically generated" />

The trim analysis algorithm is illustrated in Figure 4.10. The
optimization algorithm is selected as Thrust Region Method (Conn, Gould,
& Toint, 2000). Firstly, the trim state vector is initialized with
regard to the problem definition. Then, external forces and moments are
computed in order to put them into the equation of motion. With obtained
parameters, the cost function is calculated, and the optimization
algorithm calculates the new trim state vector. If the trim state vector
converges considering the related converging condition for the
optimization algorithm, the problem is finalized. Otherwise, the trim
state vector is again used to calculate external forces and moments.

As it is mentioned before, the aircraft trim analysis is performed for
generalized trim analysis problems at different kinds of conditions. One
of them is that wind velocity, $V_{T},$ is 20.5 m/s, altitude, $h$, is 0
meters, flight path angle, $\gamma$, is 0 degrees as it is seen in
(4.99).

$$\mathbf{\zeta}_{\text{specified}} = {(20.5,\ 0,\ 0,\ 0)}^{T}$$

The initialization of the trim vector can be seen as it is in equation
(4.100).

$$\mathbf{\zeta}_{\text{unknown}} = {(\ 0.0,\ 0.02,\ 0.0,\ 0.0,\ 0.0,\ 0.02,\ 0.0,0.0,\ 0.0,\ 0.0,1.0)}^{T}$$

After the algorithm converges, the equation (4.101) is obtained as the
solution to the problem.

$$\mathbf{\zeta}_{\text{unknown}} = {(\ 0.0,\ 0.106,\ 0.0,\ 0.0,\ 0.0,\ 0.106,\ 0.0, - 0.20,\ 0.0,\ 0.0,0.579)}^{T}$$

For the case of steady-turn trim problem definition, the trim state
vector is defined in (4.102). wind velocity, $V_{T},$ is 20.5 m/s,
altitude, $h$, is 0 meters, flight path angle, $\gamma$, is 0 and bank
angle, $\phi$, is $45{^\circ}$.

$$\mathbf{\zeta}_{\text{specified}} = {(20.5,\ 0,0,0,\ \phi = 45{^\circ})}^{T}$$

And the trim state vector is initialized as in equation (4.103).

$$\mathbf{\zeta}_{\text{unknown}} = {(0.2,\ 0.2,\ 0.0,\ 0.0,\ 0.0,\ 0.0,\ 1)}^{T}$$

The solution of the steady-turn trim analysis is given in equation
(4.104).

$$\mathbf{\zeta}_{\text{unknown}} = {(0.020,\ 0.024,\ 0,0.0280,\ 0.002,\  - 0.003,\ 0.505)}^{T}$$


## Numerical linearization of flight and transition dynamics

There are six aircraft non-linear equations of motion that describes the
motion of an aircraft. The non-linear system could be linearized around
trim points in order to simplify the stability analysis and develop a
control system for the vehicle in that operation region. Numerous
engineering applications make use of linearization, and the linearized
system is reliable enough to stand in for the original system close to
the trim points. Different methods, such as obtaining partial
derivatives of the equation of motion or small perturbation approaches,
are used to linearize the aircraft's equations of motion. Beginning of
the aircraft equations of motion linearization starts with the study of
perturbed flight. Using a combination of steady state and perturbed
variables for aircraft motion parameters as well as for forces and
moments, the perturbed flight is described in relation to a steady state
flight condition or trim conditions. The sum of a steady state value and
a perturbed value is used to redefine each motion variable, Euler angle,
force, and moment in the equation of motion. It can be started by
defining the state equations. The explicit form of the nonlinear system
is as in equation (4.121).

$$\dot{\overline{x}}\  = f(\overline{x},\overline{u},\overline{d})$$

where $\dot{\overline{x}}\mathbf{\ }\epsilon\ \mathbb{R}^{n}$,
$\overline{x}\mathbf{\ }\epsilon\ \mathbb{R}^{n}$,
$\overline{u}\mathbf{\ }\epsilon\ \mathbb{R}^{m}$ and
$\overline{d}\mathbf{\ }\epsilon\ \mathbb{R}^{k}$. n is the number of
states, m is the number of inputs and k is the number of disturbance
variables. The explicit form of a nonlinear system can be written as the
implicit form in the equation (4.122).

$$0 = f(\overline{x},\overline{u},\overline{d}) - \dot{\overline{x}}\  = F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})$$

If all variables are stacked into one called $\eta$, the equation
(4.123) can be written.

$$0 = F(\overline{\eta})$$

$$\overline{\eta}\  = \ \begin{pmatrix}
\dot{\overline{x}} & \ \overline{x} & \overline{u} & \overline{d} \\
\end{pmatrix}^{T}$$

where $\overline{\eta}\epsilon\ \mathbb{R}^{2n + m + k}$. If the Taylor
series expansion is written for $F(\overline{\eta})$, we can obtain
equation (4.124).

$$F(\overline{\eta}) = F({\overline{\eta}}_{1}) + \left. \ \frac{\partial F(\overline{\eta})}{\partial\overline{\eta}} \right|_{{\overline{\eta}}_{1}}\left( \overline{\eta} - {\overline{\eta}}_{1} \right) + \left. \ \frac{\partial^{2}F(\overline{\eta})}{\partial{\overline{\eta}}^{2}} \right|_{{\overline{\eta}}_{1}}\left( \overline{\eta} - {\overline{\eta}}_{1} \right)^{2} + \cdots$$

And if the $\overline{\eta}$ the term is expanded, the equation (4.125)
can be written for the first gradient.

$$F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d}) = F({\dot{\overline{x}}}_{1},{\overline{x}}_{1},{\overline{u}}_{1},{\overline{d}}_{1}) + \frac{\partial F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})}{\partial\dot{\overline{x}}}\left( \dot{\overline{x}} - {\dot{\overline{x}}}_{1} \right) + \frac{\partial F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})}{\partial\ \overline{x}}\left( \ \overline{x} - {\overline{x}}_{1} \right) + \frac{\partial F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})}{\partial\ \overline{u}}\left( \ \overline{u} - {\overline{u}}_{1} \right) + \frac{\partial F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})}{\partial\ \overline{d}}\left( \ \overline{d} - {\overline{d}}_{1} \right) + \cdots$$

In equation (4.125), the subscript “1” refers to the trim state, related
to the steady-state flight. For the sake of simplicity, the difference
term can be written as,

$$\mathrm{\Delta}\dot{\overline{x}} = \left( \dot{\overline{x}} - {\dot{\overline{x}}}_{1} \right)$$

$$\mathrm{\Delta}\overline{x} = \left( \ \overline{x} - {\overline{x}}_{1} \right)$$

$$\mathrm{\Delta}\overline{u} = \left( \ \overline{u} - {\overline{u}}_{1} \right)$$

$$\mathrm{\Delta}\overline{d} = \left( \ \overline{d} - {\overline{d}}_{1} \right)$$

Then each Jacobian matrix can be written as follow,

$$\frac{\partial F({\overline{\eta}}_{1})}{\partial\dot{\overline{x}}} = \begin{pmatrix}
\frac{\partial F_{1}}{\partial\ {\dot{\overline{x}}}_{1}} & \ldots & \frac{\partial F_{1}}{\partial\ {\dot{\overline{x}}}_{n}} \\
 \vdots & \ddots & \vdots \\
\frac{\partial F_{n}}{\partial\ {\dot{\overline{x}}}_{1}} & \ldots & \frac{\partial F_{n}}{\partial\ {\dot{\overline{x}}}_{n}} \\
\end{pmatrix} = \ E$$

$$\frac{\partial F({\overline{\eta}}_{1})}{\partial\overline{x}} = \begin{pmatrix}
\frac{\partial F_{1}}{\partial\ {\overline{x}}_{1}} & \ldots & \frac{\partial F_{1}}{\partial\ {\overline{x}}_{n}} \\
 \vdots & \ddots & \vdots \\
\frac{\partial F_{n}}{\partial\ {\overline{x}}_{1}} & \ldots & \frac{\partial F_{n}}{\partial\ {\overline{x}}_{n}} \\
\end{pmatrix} = \ A'$$

$$\frac{\partial F({\overline{\eta}}_{1})}{\partial\overline{u}} = \begin{pmatrix}
\frac{\partial F_{1}}{\partial\ {\overline{u}}_{1}} & \ldots & \frac{\partial F_{1}}{\partial\ {\overline{u}}_{n}} \\
 \vdots & \ddots & \vdots \\
\frac{\partial F_{n}}{\partial\ {\overline{u}}_{1}} & \ldots & \frac{\partial F_{n}}{\partial\ {\overline{u}}_{n}} \\
\end{pmatrix} = \ B'$$

$$\frac{\partial F({\overline{\eta}}_{1})}{\partial\overline{d}} = \begin{pmatrix}
\frac{\partial F_{1}}{\partial\ {\overline{d}}_{1}} & \ldots & \frac{\partial F_{1}}{\partial\ {\overline{d}}_{n}} \\
 \vdots & \ddots & \vdots \\
\frac{\partial F_{n}}{\partial\ {\overline{d}}_{1}} & \ldots & \frac{\partial F_{n}}{\partial\ {\overline{d}}_{n}} \\
\end{pmatrix} = \ D'$$

Jacobians are given in equations (4.130), (4.131), (4.132), and (4.133).
Regarding the equation (4.125), the terms
$F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})$ and
$F({\dot{\overline{x}}}_{1},{\overline{x}}_{1},{\overline{u}}_{1},{\overline{d}}_{1})$
are expected to be zero because the flight conditions are steady-state
flight conditions. Therefore, the equation can be established.

$$0 = E\mathrm{\Delta}\dot{\overline{x}} + A'\mathrm{\Delta}\overline{x} + B'\mathrm{\Delta}\overline{u} + D'\mathrm{\Delta}\overline{d}$$

Using the equation (4.134), we can obtain state states form,

$$\mathrm{\Delta}\dot{\overline{x}} = - E^{- 1}A'\mathrm{\Delta}\overline{x} - E^{- 1}B'\mathrm{\Delta}\overline{u} - E^{- 1}D'\mathrm{\Delta}\overline{d}$$

Consequently, the state matrix, $A$, input matrix, $B$, and the
disturbance matrix, $D$, can be obtained using equations (4.136),
(4.137), and (4.138).

$$A = - E^{- 1}A'$$

$$B = - E^{- 1}B'$$

$$D = {- E}^{- 1}D'$$

The extraction of the Jacobians column by column creates the basis for
the stated algorithm to linearize the system numerically. The
discrepancy between the current column and the previously calculated
column is controlled by the convergence tolerance value 1e-6. It moves
on to the calculations for the following column if the difference is
less than the tolerance value.

<img src="media/image1.png" style="width:4.16667in;height:5.10442in" alt="Diagram Description automatically generated" />

**Figure** **4.12:** Numerical linearization algorithm.

In reference of (Stevens, Lewis, & Johnson, 2015) algorithm of the
numerical method which is also seen in the Figure 4.12 is described in
detail. The user determines the precision value, state vector, control
input vector and step size of the numerical derivation. Then, the
algorithm starts with finding trim data and initial perturbation of the
model regarding non-linear continuous state equation. Then, step size is
reduced, and partial derivative calculation is performed. The algorithm
adaptively reduces the step size during the numerical analysis and each
step the precision is controlled by checking the relative error for each
reduced step size adaptively. If the relative error is within the
precision value given by the user, then the calculation is counted as
converged and algorithm starts to calculate next derivative. This
operation is done for matrix *E*, $A'$, $C'$ and $D'$.

The linear state equations of the aircraft for are found by performing
numerical linearization, while the aircraft mode and state matrices were
found throughout the transition. Accordingly, the longitudinal and
lateral state matrices are given in equations (4.139) and (4.140).

$$A_{\text{lon}} = \begin{pmatrix}
X_{u} & X_{w} & X_{q} & - g\ cos\theta_{1} & 0 \\
Z_{u} & Z_{w} & Z_{q} & - g\ sin\theta_{1} & 0 \\
M_{u} & M_{w} & M_{q} & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & - 1 & 0 & V_{1} & 0 \\
\end{pmatrix}$$

$$A_{\text{lat}} = \begin{pmatrix}
Y_{v} & Y_{p} & Y_{r} & \text{g\ cos}\theta_{1} & 0 \\
L_{v} & L_{p} & L_{r} & 0 & 0 \\
N_{v} & N_{p} & N_{r} & 0 & 0 \\
0 & 1 & \tan\theta_{1} & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
\end{pmatrix}$$

The uppercase symbols in the matrices indicate the force or moment
derivatives, while the subscripts indicate the relevant state element.
$g$ is the gravitational acceleration, $\theta_{1}$ is the trim pitch
and $V_{1}$ is the wind speed in the trim state. In addition, the states
are as shown in equation (4.141).

$$\mathbf{x}_{\text{lon}} = \begin{pmatrix}
u & w & \begin{matrix}
q & \theta & h \\
\end{matrix} \\
\end{pmatrix}^{T}$$

$$\mathbf{x}_{\text{lat}} = \begin{pmatrix}
v & p & \begin{matrix}
r & \phi & \psi \\
\end{matrix} \\
\end{pmatrix}^{T}$$

Input matrices are also obtained in numerical linearization. Below are
the input matrices for longitudinal and lateral dynamics.

$$B_{lon,ff} = \begin{pmatrix}
X_{\text{δe}} & X_{T} \\
Z_{\text{δe}} & Z_{T} \\
M_{\text{δe}} & M_{T} \\
0 & 0 \\
0 & 0 \\
\end{pmatrix}$$

$$B_{lat,ff} = \begin{pmatrix}
Y_{\text{δa}} & Y_{\text{δr}} \\
L_{\text{δa}} & L_{\text{δr}} \\
N_{\text{δa}} & N_{\text{δr}} \\
0 & 0 \\
0 & 0 \\
\end{pmatrix}$$

Regarding the transition dynamics of the aircraft, there will be
additional input variables obtained from quadrotor dynamics. Therefore,
the matrices are 5x4 in size and there are eight control inputs in
total. The linearized longitudinal and lateral control matrices are
shown in (4.144) and (4.145).

$$B_{lon,tr} = \begin{pmatrix}
X_{\text{δe}} & X_{T} & 0 & 0 \\
Z_{\text{δe}} & Z_{T} & - \frac{1}{m} & 0 \\
M_{\text{δe}} & M_{T} & 0 & \frac{a}{I_{\text{yy}}} \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
\end{pmatrix}$$

$$B_{lat,tr} = \begin{pmatrix}
Y_{\text{δa}} & Y_{\text{δr}} & 0 & 0 \\
L_{\text{δa}} & L_{\text{δr}} & \frac{a}{I_{\text{xx}}} & 0 \\
N_{\text{δa}} & N_{\text{δr}} & 0 & \frac{1}{I_{\text{zz}}} \\
0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 \\
\end{pmatrix}$$

For the trim analysis whose conditions are given in equation (4.99), the
non-linear state equations are linearized with regard to given numerical
linearization techniques. The state matrix results for longitudinal and
lateral dynamics are given in equations (4.146) and (4.147).

$$A_{\text{lon}} = \begin{pmatrix}
 - 0.134 & 0.517 & - 2.030 & - 9.751 & 0 \\
 - 0.677 & - 2.623 & 19.027 & - 1.044 & 0 \\
0.263 & - 2.473 & - 5.197 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & - 0.994 & 0 & 20.5 & 0 \\
\end{pmatrix}$$

$$A_{\text{lat}} = \begin{pmatrix}
 - 0.298 & 2.207 & - 19.992 & 9.751 & 0 \\
0.624 & - 6.848 & 2.547 & 0 & 0 \\
0.997 & - 1.375 & - 0.859 & 0 & 0 \\
0 & 1 & 0.107 & 0 & 0 \\
0 & 0 & 1.006 & 0 & 0 \\
\end{pmatrix}$$

Also, the longitudinal and lateral input matrices are given below,

$$B_{lon,ff} = \begin{pmatrix}
0 & 5.233 \\
 - 6.996 & 0 \\
 - 13.952 & 0 \\
0 & 0 \\
0 & 0 \\
\end{pmatrix}$$

$$B_{lat,ff} = \begin{pmatrix}
0 & 3.079 \\
27.887 & - 0.420 \\
2.657 & - 8.965 \\
0 & 0 \\
0 & 0 \\
\end{pmatrix}$$

