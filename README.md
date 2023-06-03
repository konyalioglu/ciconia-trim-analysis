# CICONIA AIRCRAFT TRIM ANALYSIS



## Usage


## Dependencies


## Aircraft Trim Analysis

In general aviation terms, trimming is performed to bring the act of altering trim tabs or artificial feel systems in order to remove the pressure on the control stick (or pedals) that the pilot is supposed to maintain during different flight conditions. When the aircraft is properly trimmed, the flight controls are in a position where straight and level flight can be maintained without the need for force. The goal of trimming is to achieve equilibrium between the forces and moments acting on the aircraft. This occurs when the roll, pitch, and yaw moments, as well as the axial normal and side forces, are all equal to zero. The dynamic stability characteristics of the motion that occurs after such a disturbance are specified, and the stable aircraft will finally return to its equilibrium condition which occurs in steady-state flight.

In a variety of engineering research, determining aircraft steady-state flight conditions or trimmed states is of great importance. Usually, trim points are used to derive linear models. Linearization is performed around those trim states.

In general, trim analysis is defined as an optimization problem. The state vector for the trim states is defined as two different parts, which are dynamic and kinematic as shown in 

$$
\mathbf{x_d} = \left( u, v, w, p, q, r \right)^T
$$

$$
\mathbf{x_k} = \left( x, y, z, \phi, \theta, \psi \right)^T
$$

where $\mathbf{x_d}$ is the dynamic state vector and $\mathbf{x_k}$ is the kinematic state vector. The control input for forward flight is defined in equation below,

$$
\mathbf{u_f} = \left( \delta_T, \delta_e, \delta_a, \delta_r \right)^T
$$

where $\delta_T$ is throttle setting, $\delta_e$ is elevator deflection, $\delta_a$ is aileron deflection, and $\delta_r$ is rudder deflection. In the transition state, when the propulsion system for hover state is also used, the control input vector becomes as shown in equation below,

$$
\mathbf{u_t} = \left( \delta_T, \delta_e, \delta_a, \delta_r, u_1, u_2, u_3, u_4 \right)^T
$$

where $u_1, u_2, u_3,$ and $u_4$ represent the force in the x-axis, the force that creates the pitching moment, the force that creates the rolling moment, and the force that creates the yawing moment, respectively. Additionally, the implicit form of the non-linear aircraft state equation for the trim state is given in equation  below,

$$
g(\dot{\mathbf{x}}, \mathbf{x}, \mathbf{u}) = 0
$$

Before establishing the cost function for the optimization problem, necessary parameters should be constructed. Translational dynamic, rotational dynamic, translational kinematic, and rotational kinematic cost function parameters are given as,

$$f_{TD1} = \dot{u},\ \ f_{TD2} = \dot{v},\ \ f_{TD3} = \dot{w}$$

$$f_{RD1} = \dot{p},\ \ f_{RD2} = \dot{q},\ \ f_{RD3} = \dot{r}$$

$$f_{TK1} = \dot{x},\ \ f_{TK2} = \dot{y},\ \ f_{TK3} = \dot{z}$$

$$f_{RK1} = \dot{\phi},\ \ f_{RK2} = \dot{\theta},\ \ f_{RK3} = \dot{\psi}$$

However, considering the construction of the trim problem in order to add flight path angle constraint to the cost function, $f_{TK3}$ can be written as:

$$f_{TK3} = \sin\gamma - \left[\cos\alpha\cos\beta\sin\theta - \sin\beta\cos\theta\sin\phi - \sin\alpha\cos\beta\cos\theta\cos\phi\right]$$

These cost function parameters are obtained by using the equation of motion of the hybrid VTOL aircraft as discussed in section 4.2.1. For general straight flight trim analysis, the trim states are chosen as shown in equation:

$$\mathbf{\zeta} = \left(V_{T},\ \beta,\ \alpha,\ (u,\ v,\ w),\ p,\ q,\ r,\ \phi,\ \theta,\ \psi,\ h,\gamma,\ \delta_{T},\ \delta_e,\ \delta_a,\ \delta_r\right)^T$$

Where specified and unknown trim state vectors are given in equations:

$$
\mathbf{\zeta}_{\text{specified}} = \begin{pmatrix} V_T \\ h \\ \gamma \\ \phi \end{pmatrix}^T
$$

$$\mathbf{\zeta}_{\text{unknown}} = \left(\beta,\ \alpha,\ p,\ q,\ r,\ \theta,\ \psi,\delta_e,\ \delta_a,\ \delta_r,\delta_T\right)^T$$

Here, $\zeta_{\text{specified}}$ is defined as the trim condition of the aircraft. $h$ determines the atmospheric conditions, $\gamma$ determines whether the aircraft is descending or ascending. If a turn state is analyzed, the $\phi$ term can be given as an input other than zero. The generalized trim cost function is defined in equation:

$$J = f_{TD1}^2 + f_{TD2}^2 + f_{TD3}^2 + f_{RD1}^2 + f_{RD2}^2 + f_{RD3}^2 + f_{RK1}^2 + f_{RK2}^2 + f_{RK3}^2 + f_{TK2}^2 + f_{TK3}^2$$

Then, the optimization problem can be solved with 11 unknown and 11 known variables. The trim analysis is also run for transition dynamics of the aircraft. Additional variables are the propulsion system for the hover state of the aircraft. However, in order to make the number of unknown and known variables equal, some changes should be made. Thus, the deflection of control surfaces is specified as 0 deflections. Also, one additional parameter, $\theta$, is set to zero. Additionally, moments and forces are considered to be compensated by the hover propulsion system.

$$\mathbf{\zeta} = \left(V_{T},\ \beta,\ \alpha,\ (u,\ v,\ w),\ p,\ q,\ r,\ \phi,\ \theta,\ \psi,\ h,\gamma,\ \delta_{T},u1,u2,u3,u4\right)^T$$

Where specified and unknown trim state vectors are given in equations:

$$
\mathbf{\zeta}_{\text{specified}} = \begin{pmatrix} V_T \\ h \\ \gamma \\ \phi \\ \theta = 0 \\ \delta e = 0 \\ \delta a = 0 \\ \delta r = 0 \end{pmatrix}^T
$$

$$
\mathbf{\zeta}_{\text{unknown}} = \begin{pmatrix} \beta \\ \alpha \\ p \\ q \\ r \\ \theta \\ \psi \\ \delta e \\ \delta a \\ \delta r \\ \delta_T \\ u_1 \\ u_2 \\ u_3 \\ u_4 \end{pmatrix}^T
$$

The cost function for transition trim analysis is defined the same as the general straight flight trim analysis, as established in equation.

On the other hand, another trim analysis type, which is steady-state turn flight, is performed in this thesis. In addition to the general straight flight trim problem, the side-slip angle, $\beta$, is specified, body rates are removed, and yaw rate is left optional to user definition, regarding the trim state vector as it is seen,

$$
\mathbf{\zeta}_{\text{specified}} = \begin{pmatrix} V_T \\ h \\ \gamma \\ \beta \\ \phi \ \text{(or} \ \dot{\psi}) \end{pmatrix}^T
$$


$$
\mathbf{\zeta}_{\text{unknown}} = \begin{pmatrix} \alpha \\ \theta \\ \dot{\psi} \ \text{(or} \ \phi) \\ \delta e \\ \delta a \\ \delta r \\ \delta_T \end{pmatrix}^T
$$

Therefore, in the trim state vector, there are seven unknown variables.
Because turning concerns the variables in the planar position, related
constraints are removed from the cost function as it is seen in equation below,

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
meters, flight path angle, $\gamma$, is 0 degrees as it is seen in below,

$$\mathbf{\zeta}_{\text{specified}} = {(20.5,\ 0,\ 0,\ 0)}^{T}$$

The initialization of the trim vector can be seen as it is in equation:

$$\mathbf{\zeta}_{\text{unknown}} = {(\ 0.0,\ 0.02,\ 0.0,\ 0.0,\ 0.0,\ 0.02,\ 0.0,0.0,\ 0.0,\ 0.0,1.0)}^{T}$$

After the algorithm converges, the equation below is obtained as the
solution to the problem.

$$\mathbf{\zeta}_{\text{unknown}} = {(\ 0.0,\ 0.106,\ 0.0,\ 0.0,\ 0.0,\ 0.106,\ 0.0, - 0.20,\ 0.0,\ 0.0,0.579)}^{T}$$

For the case of steady-turn trim problem definition, wind velocity, $V_{T},$ is 20.5 m/s,
altitude, $h$, is 0 meters, flight path angle, $\gamma$, is 0 and bank
angle, $\phi$, is $45{^\circ}$.

$$\mathbf{\zeta}_{\text{specified}} = {(20.5,\ 0,0,0,\ \phi = 45{^\circ})}^{T}$$

And the trim state vector is initialized as in equation below,

$$\mathbf{\zeta}_{\text{unknown}} = {(0.2,\ 0.2,\ 0.0,\ 0.0,\ 0.0,\ 0.0,\ 1)}^{T}$$

The solution of the steady-turn trim analysis is given in equation,

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
is defined as,

$$\dot{\overline{x}}\  = f(\overline{x},\overline{u},\overline{d})$$

where $\dot{\overline{x}}\mathbf{\ }\epsilon\ \mathbb{R}^{n}$,
$\overline{x}\mathbf{\ }\epsilon\ \mathbb{R}^{n}$,
$\overline{u}\mathbf{\ }\epsilon\ \mathbb{R}^{m}$ and
$\overline{d}\mathbf{\ }\epsilon\ \mathbb{R}^{k}$. n is the number of
states, m is the number of inputs and k is the number of disturbance
variables. The explicit form of a nonlinear system can be written as the
implicit form,

$$0 = f(\overline{x},\overline{u},\overline{d}) - \dot{\overline{x}}\  = F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})$$

If all variables are stacked into one called $\eta$, the equation
below can be written.

$$0 = F(\overline{\eta})$$

$$\overline{\eta}\  = \ \begin{pmatrix}
\dot{\overline{x}} & \ \overline{x} & \overline{u} & \overline{d} \\
\end{pmatrix}^{T}$$

where $\overline{\eta}\epsilon\ \mathbb{R}^{2n + m + k}$. If the Taylor
series expansion is written for $F(\overline{\eta})$, we can obtain
equation

$$
F(\bar{\eta}) = F({\bar{\eta}}_1) + \left. \frac{\partial F(\bar{\eta})}{\partial\bar{\eta}} \right|_{{\bar{\eta}}_1} (\bar{\eta}-{\bar{\eta}}_1) + \left. \frac{\partial^2F(\bar{\eta})}{\partial{\bar{\eta}}^2} \right|_{{\bar{\eta}}_1} (\bar{\eta}-{\bar{\eta}}_1)^2 + \cdots
$$

And if the $\overline{\eta}$ term is expanded, the equation below can be written for the first gradient.

$$
F(\dot{\overline{x}}, \bar{x}, \bar{u}, \bar{d}) = F(\dot{\overline{x}}_1, \bar{x}_1, \bar{u}_1, \bar{d}_1) + \frac{\partial F(\dot{\overline{x}}, \bar{x}, \bar{u}, \bar{d})}{\partial \dot{\overline{x}}} (\dot{\overline{x}} - \dot{\overline{x}}_1) + \frac{\partial F(\dot{\overline{x}}, \bar{x}, \bar{u}, \bar{d})}{\partial \bar{x}} (\bar{x} - \bar{x}_1) + \frac{\partial F(\dot{\overline{x}}, \bar{x}, \bar{u}, \bar{d})}{\partial \bar{u}} (\bar{u} - \bar{u}_1) + \frac{\partial F(\dot{\overline{x}}, \bar{x}, \bar{u}, \bar{d})}{\partial \bar{d}} (\bar{d} - \bar{d}_1) + \cdots
$$

In equation above, the subscript “1” refers to the trim state, related
to the steady-state flight. For the sake of simplicity, the difference
term can be written as,

$$\mathrm{\Delta}\dot{\overline{x}} = \left( \dot{\overline{x}} - {\dot{\overline{x}}}_{1} \right)$$

$$\mathrm{\Delta}\overline{x} = \left( \ \overline{x} - {\overline{x}}_{1} \right)$$

$$\mathrm{\Delta}\overline{u} = \left( \ \overline{u} - {\overline{u}}_{1} \right)$$

$$\mathrm{\Delta}\overline{d} = \left( \ \overline{d} - {\overline{d}}_{1} \right)$$

Then each Jacobian matrix can be written as follow,

$$
\frac{\partial F(\bar{\eta}_1)}{\partial \dot{\overline{x}}} = \begin{pmatrix}
\frac{\partial F_1}{\partial \dot{\overline{x}}_1} & \ldots & \frac{\partial F_1}{\partial \dot{\overline{x}}_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial F_n}{\partial \dot{\overline{x}}_1} & \ldots & \frac{\partial F_n}{\partial \dot{\overline{x}}_n} \\
\end{pmatrix} = E
$$

$$
\frac{\partial F(\bar{\eta}_1)}{\partial \bar{x}} = \begin{pmatrix}
\frac{\partial F_1}{\partial \bar{x}_1} & \ldots & \frac{\partial F_1}{\partial \bar{x}_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial F_n}{\partial \bar{x}_1} & \ldots & \frac{\partial F_n}{\partial \bar{x}_n} \\
\end{pmatrix} = A'
$$

$$
\frac{\partial F(\bar{\eta}_1)}{\partial \bar{u}} = \begin{pmatrix}
\frac{\partial F_1}{\partial \bar{u}_1} & \ldots & \frac{\partial F_1}{\partial \bar{u}_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial F_n}{\partial \bar{u}_1} & \ldots & \frac{\partial F_n}{\partial \bar{u}_n} \\
\end{pmatrix} = B'
$$

$$
\frac{\partial F(\bar{\eta}_1)}{\partial \bar{d}} = \begin{pmatrix}
\frac{\partial F_1}{\partial \bar{d}_1} & \ldots & \frac{\partial F_1}{\partial \bar{d}_n} \\
\vdots & \ddots & \vdots \\
\frac{\partial F_n}{\partial \bar{d}_1} & \ldots & \frac{\partial F_n}{\partial \bar{d}_n} \\
\end{pmatrix} = D'
$$

Jacobians are given in equations above.
The terms
$F(\dot{\overline{x}},\ \overline{x},\overline{u},\overline{d})$ and
$F(\dot{\overline{x}}_1,\bar{x}_1,\bar{u}_1,\bar{d}_1)$
are expected to be zero because the flight conditions are steady-state
flight conditions. Therefore, the equation can be established.

$$
0 = E\Delta x + A'\Delta x + B'\Delta u + D'\Delta d
$$

Then, we can obtain state states form,

$$\mathrm{\Delta}\dot{\overline{x}} =-E^{-1}A'\mathrm{\Delta}\overline{x}-E^{-1}B'\mathrm{\Delta}\overline{u} -E^{-1}D'\mathrm{\Delta}\overline{d}$$

Consequently, the state matrix, $A$, input matrix, $B$, and the
disturbance matrix, $D$, can be obtained as,

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
numerical method which is also seen in the Figure above is described in
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
lateral state matrices are given in equations:

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
are as shown in equation below,

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
shown as, 

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
lateral dynamics are given in equations:

$$A_{\text{lon}} = \begin{pmatrix}
 -0.134 & 0.517 & -2.030 & -9.751 & 0 \\
 -0.677 & -2.623 & 19.027 & -1.044 & 0 \\
0.263 & -2.473 & -5.197 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 \\
0 & -0.994 & 0 & 20.5 & 0 \\
\end{pmatrix}$$

$$A_{\text{lat}} = \begin{pmatrix}
 -0.298 & 2.207 & -19.992 & 9.751 & 0 \\
0.624 & -6.848 & 2.547 & 0 & 0 \\
0.997 & -1.375 & -0.859 & 0 & 0 \\
0 & 1 & 0.107 & 0 & 0 \\
0 & 0 & 1.006 & 0 & 0 \\
\end{pmatrix}$$

Also, the longitudinal and lateral input matrices are given below,

$$B_{lon,ff} = \begin{pmatrix}
0 & 5.233 \\
 -6.996 & 0 \\
 -13.952 & 0 \\
0 & 0 \\
0 & 0 \\
\end{pmatrix}$$

$$B_{lat,ff} = \begin{pmatrix}
0 & 3.079 \\
27.887 & -0.420 \\
2.657 & -8.965 \\
0 & 0 \\
0 & 0 \\
\end{pmatrix}$$



## References
[1] Konyalioglu, T. (2023). DESIGN, DEVELOPMENT AND CONTROL OF UNMANNED AERIAL VEHICLES FOR A PARCEL DELIVERY SYSTEM. Master's Thesis. Istanbul Technical University.
[2] De Marco, A., Duke, E., & Berndt, J. (2007, August). A general solution to the aircraft trim problem. In AIAA Modeling and Simulation Technologies Conference and Exhibit (p. 6703).
[3] Stevens, B. L., Lewis, F. L., & Johnson, E. N. (2015). Aircraft control and simulation : dynamics, controls design, and autonomous systems.
[4] Konyalioglu, T., Alnipak, S., & Altug, E. (2021). Model Predictive Control of a Hybrid UAV for Parcel Delivery Applications. AIRPHARO 2021 - 1st AIRPHARO Workshop on Aerial Robotic Systems Physically Interacting with the Environment. https://doi.org/10.1109/AIRPHARO52252.2021.9571035
