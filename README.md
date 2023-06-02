# ciconia-trim-analysis

Trim Analysis
In general aviation terms, trimming is performed to bring the act of altering trim tabs or artificial feel systems in order to remove the pressure on the control stick (or pedals) that the pilot is supposed to maintain during different flight conditions. When the aircraft is properly trimmed, the flight controls are in a position where the straight and level flight can be maintained without the need for force. The goal of trimming is to achieve equilibrium between the forces and moments acting on the aircraft. That occurs when the roll pitch and yaw moments, as well as the axial normal and side forces, are all equal to zero. The dynamic stability characteristics of the motion that occurs after such a disturbance are specified, and the stable aircraft will finally return to its equilibrium condition which occurs in steady-state flight. In a variety of engineering research, determining aircraft steady-state flight conditions or also trimmed states is of great importance. Usually, trim points are used to derive linear models. Linearization is performed around those trim states.
In general, trim analysis is defined as an optimization problem. The state vector for the trim states is defined as two different parts which are dynamic and kinematic as it is seen in equations (4.79) and (4.80).

	$`\mathbit{x}_\mathbit{d}={(u,v,w,p,q,r)}^\mathbit{T}\`$ 	(4.79)

	$`\mathbit{x}_\mathbf{k}={(x,y,z,\phi,\ \theta,\ \psi)}^\mathbit{T}\`$ 	(4.80)

$\mathbit{x}_\mathbit{d}$ is the dynamic state vector and \mathbit{x}_\mathbit{k} is a kinematic state vector. Also, the control input is defined in equation (4.81), for the forward flight. 
	$\mathbit{u}_{\mathbf{ff}}={(\delta_T,\ \delta e,\ \delta a,\ \delta r)}^T$	(4.81)

Where \delta_T is throttle setting, \delta e is elevator deflection, \delta a is aileron deflection and \delta r is rudder deflection. Because, in the transition state, the propulsion system for the hover state will also be used, the control input vector becomes as in equation (4.82).
	\mathbit{u}_{\mathbf{tr}}={(\delta_T,\ \delta e,\ \delta a,\ \delta r,\ u1,u2,u3,u4)}^T	(4.82)

u1,u2,u3, and u4 are defined as the force in the x-axis, the force that creates the pitching moment, and the force that creates the rolling moment and yawing moment, respectively. Also, one can define the implicit form of the non-linear aircraft state equation for the trim state in equation (4.83). 
	g(\dot{\mathbit{x}},\mathbit{x},\mathbit{u})=0	(4.83)

Before establishing, the cost function for the optimization problem, necessary parameters should be constructed. Translational dynamic, rotational dynamic, translational kinematic, and rotational kinematic cost function parameters are given in equations (4.84), (4.85), (4.86), and (4.87), respectively.
	f_{TD1}=\dot{u},\ \ f_{TD2}=\dot{v},\ \ f_{TD3}=\dot{w}\ 	(4.84)

	f_{RD1}=\dot{p},\ \ f_{RD2}=\dot{q},\ \ f_{RD3}=\dot{r}\ 	(4.85)

	f_{TK1}=\dot{x},\ \ f_{TK2}=\dot{y},\ \ f_{TK3}=\dot{z}\ 	(4.86)

	f_{RK1}=\dot{\phi},\ \ f_{RK2}=\dot{\theta},\ \ f_{RK3}=\dot{\psi}\ 	(4.87)

However, considering the construction of the trim problem in order to add flight path angle constraint to the cost function, f_{TK3} can be written as 
	f_{TK3}=sin\gamma-[(cos\alpha\ cos\beta\ sin\theta)-(sin\beta\ cos\theta\ sin\phi)-(sin\alpha\ cos\beta\ cos\theta\ cos\phi)]	(4.88)

These cost function parameters are obtained by using the equation of motion of the hybrid VTOL aircraft as it was discussed in section 4.2.1. For general straight flight trim analysis, the trim states are chosen as shown in equation (4.89).
	\mathbit{\zeta}={(V_T,\ \beta,\ \alpha,\ (u,\ v,\ w),\ p,\ q,\ r,\ \phi,\ \theta,\ \psi,\ h,\gamma,\ \delta_T,\ \delta e,\ \delta a,\ \delta r)}^T	(4.89)

Where specified and unknown trim state vectors are given in equations (4.90) and (4.91),
	\mathbit{\zeta}_{specified}={(V_T,\ h,\gamma,\ \phi)}^T	(4.90)

	\mathbit{\zeta}_{unknown}={(\ \beta,\ \alpha,\ p,\ q,\ r,\ \theta,\ \psi,\delta e,\ \delta a,\ \delta r,\delta_T)}^T	(4.91)

\zeta_{specified} is actually defined as the trim condition of the aircraft. h determines the atmospheric conditions, \gamma determines whether the aircraft descending or ascending. If a turn state is analyzed, \phi term can be given as input other than zero. The generalized trim cost function is defined in equation (4.92).
	J=f_{TD1}^2+f_{TD2}^2+f_{TD3}^2+f_{RD1}^2+f_{RD2}^2+f_{RD3}^2+f_{RK1}^2+f_{RK2}^2+f_{RK3}^2+f_{TK2}^2+f_{TK3}^2	(4.92)

Then, the optimization problem can be solved with 11 unknown and 11 known variables. The trim analysis is also run for transition dynamics for the aircraft. Additional variables are the propulsion system for the hover state of the aircraft. However, in order to make a number of unknown and known equal some changes should be done. Thus, the deflection of control surfaces is specified as 0 deflections. Also, one additional parameter which is \theta, is set to zero. Additionally, moment and forces are considered to be compensated by the hover propulsion system.
	\mathbit{\zeta}={(V_T,\ \beta,\ \alpha,\ (u,\ v,\ w),\ p,\ q,\ r,\ \phi,\ \theta,\ \psi,\ h,\gamma,\ \delta_T,u1,u2,u3,u4)}^T	(4.93)

Where specified and unknown trim state vectors are given in equations (4.94) and (4.95).
	\mathbit{\zeta}_{specified}={(V_T,\ h,\gamma,\ \phi,\ \theta=0,\ \delta e=0,\ \delta a=0,\ \delta r=0)}^T	(4.94)

	\mathbit{\zeta}_{unknown}={(\ \beta,\ \alpha,\ p,\ q,\ r,\ \theta,\ \psi,\delta e,\ \delta a,\ \delta r,\delta_T,u1,u2,u3,u4)}^T	(4.95)

The cost function for transition trim analysis is defined the same as the general straight flight trim analysis which is established in equation (4.92).
On the other hand, another trim analysis type which is steady-state turn flight is performed in this thesis. Additional to the general straight flight trim problem, the side-slip angle, \beta, is specified, body rates are removed, and yaw rate is left optional to user definition, regarding the trim state vector as it is seen in equation (4.96) and (4.97).
	\mathbit{\zeta}_{specified}={(V_T,\ h,\gamma,\beta,\ \phi\ (or\ \ \dot{\psi}))}^T	(4.96)

	\mathbit{\zeta}_{unknown}={(\alpha,\ \theta,\ \dot{\psi}\ (or\ \phi)\ ,\delta e,\ \delta a,\ \delta r,\delta_T)}^T	(4.97)

Therefore, in the trim state vector, there are seven unknown variables. Because turning concerns the variables in the planar position, related constraints are removed from the cost function as it is seen in equation (4.98).
	J=f_{TD1}^2+f_{TD2}^2+f_{TD3}^2+f_{RD1}^2+f_{RD2}^2+f_{RD3}^2+f_{TK3}^2	(4.98)

Therefore, for the optimization problem, there are seven unknown parameters to be solved by using another seven known parameters.
 
Figure 4.10: Trim analysis algorithm.
The trim analysis algorithm is illustrated in Figure 4.10. The optimization algorithm is selected as Thrust Region Method (Conn, Gould, & Toint, 2000). Firstly, the trim state vector is initialized with regard to the problem definition. Then, external forces and moments are computed in order to put them into the equation of motion. With obtained parameters, the cost function is calculated, and the optimization algorithm calculates the new trim state vector. If the trim state vector converges considering the related converging condition for the optimization algorithm, the problem is finalized. Otherwise, the trim state vector is again used to calculate external forces and moments.
As it is mentioned before, the aircraft trim analysis is performed for generalized trim analysis problems at different kinds of conditions. One of them is that wind velocity, V_T, is 20.5 m/s, altitude, h, is 0 meters, flight path angle, \gamma, is 0 degrees as it is seen in (4.99).
	\mathbit{\zeta}_{specified}={(20.5,\ 0,\ 0,\ 0)}^T	(4.99)

The initialization of the trim vector can be seen as it is in equation (4.100).
\mathbit{\zeta}_{unknown}={(\ 0.0,\ 0.02,\ 0.0,\ 0.0,\ 0.0,\ 0.02,\ 0.0,0.0,\ 0.0,\ 0.0,1.0)}^T	(4.100)

After the algorithm converges, the equation (4.101) is obtained as the solution to the problem.
\mathbit{\zeta}_{unknown}={(\ 0.0,\ 0.106,\ 0.0,\ 0.0,\ 0.0,\ 0.106,\ 0.0,-0.20,\ 0.0,\ 0.0,0.579)}^T	(4.101)

For the case of steady-turn trim problem definition, the trim state vector is defined in (4.102). wind velocity, V_T, is 20.5 m/s, altitude, h, is 0 meters, flight path angle, \gamma, is 0 and bank angle, \phi, is 45°.
	\mathbit{\zeta}_{specified}=(20.5,0,0,0,ϕ=45°)T
(4.102)

And the trim state vector is initialized as in equation (4.103).
\mathbit{\zeta}_{unknown}={(0.2,\ 0.2,\ 0.0,\ 0.0,\ 0.0,\ 0.0,\ 1)}^T	(4.103)

The solution of the steady-turn trim analysis is given in equation (4.104).
\mathbit{\zeta}_{unknown}={(0.020,\ 0.024,\ 0,0.0280,\ 0.002,\ -0.003,\ 0.505)}^T	(4.104)



Numerical linearization of flight and transition dynamics
There are six aircraft non-linear equations of motion that describes the motion of an aircraft. The non-linear system could be linearized around trim points in order to simplify the stability analysis and develop a control system for the vehicle in that operation region. Numerous engineering applications make use of linearization, and the linearized system is reliable enough to stand in for the original system close to the trim points. Different methods, such as obtaining partial derivatives of the equation of motion or small perturbation approaches, are used to linearize the aircraft's equations of motion. Beginning of the aircraft equations of motion linearization starts with the study of perturbed flight. Using a combination of steady state and perturbed variables for aircraft motion parameters as well as for forces and moments, the perturbed flight is described in relation to a steady state flight condition or trim conditions. The sum of a steady state value and a perturbed value is used to redefine each motion variable, Euler angle, force, and moment in the equation of motion. It can be started by defining the state equations. The explicit form of the nonlinear system is as in equation (4.121).
	\dot{\overline{x}}\ =f(\bar{x},\bar{u},\bar{d})	(4.121)

where \dot{\overline{x}}\ \epsilon\ \mathbb{R}^n, \bar{x}\ \epsilon\ \mathbb{R}^n, \bar{u}\ \epsilon\ \mathbb{R}^m and \bar{d}\ \epsilon\ \mathbb{R}^k. n is the number of states, m is the number of inputs and k is the number of disturbance variables. The explicit form of a nonlinear system can be written as the implicit form in the equation (4.122).
	0=f(\bar{x},\bar{u},\bar{d})-\dot{\overline{x}}\ =F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d})	(4.122)

If all variables are stacked into one called \eta, the equation (4.123) can be written.
	0=F(\bar{\eta}),\ \ \bar{\eta}\ =\ \left(\begin{matrix}\dot{\overline{x}}&\ \bar{x}&\bar{u}&\bar{d}\\\end{matrix}\right)^T	(4.123)

where \bar{\eta}\epsilon\ \mathbb{R}^{2n+m+k}. If the Taylor series expansion is written for F(\bar{\eta}), we can obtain equation (4.124).
	F(\bar{\eta})=F({\bar{\eta}}_1)+\left.\frac{\partial F(\bar{\eta})}{\partial\bar{\eta}}\right|_{{\bar{\eta}}_1}\left(\bar{\eta}-{\bar{\eta}}_1\right)+\left.\frac{\partial^2F(\bar{\eta})}{\partial{\bar{\eta}}^2}\right|_{{\bar{\eta}}_1}\left(\bar{\eta}-{\bar{\eta}}_1\right)^2+\cdots	(4.124)

And if the \bar{\eta} the term is expanded, the equation (4.125) can be written for the first gradient.
	F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d})=F({\dot{\overline{x}}}_1,{\bar{x}}_1,{\bar{u}}_1,{\bar{d}}_1)+\frac{\partial F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d})}{\partial\dot{\overline{x}}}\left(\dot{\overline{x}}-{\dot{\overline{x}}}_1\right)+\frac{\partial F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d})}{\partial\ \bar{x}}\left(\ \bar{x}-{\bar{x}}_1\right)+\frac{\partial F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d})}{\partial\ \bar{u}}\left(\ \bar{u}-{\bar{u}}_1\right)+\frac{\partial F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d})}{\partial\ \bar{d}}\left(\ \bar{d}-{\bar{d}}_1\right)+\cdots	(4.125)

In equation (4.125), the subscript “1” refers to the trim state, related to the steady-state flight. For the sake of simplicity, the difference term can be written as,
	∆x=x-x1
(4.126)

	∆x= x-x1
(4.127)

	∆u= u-u1
(4.128)

	∆d= d-d1
(4.129)

Then each Jacobian matrix can be written as follow,
	\frac{\partial F({\bar{\eta}}_1)}{\partial\dot{\overline{x}}}=\left(\begin{matrix}\frac{\partial F_1}{\partial\ {\dot{\overline{x}}}_1}&\ldots&\frac{\partial F_1}{\partial\ {\dot{\overline{x}}}_n}\\\vdots&\ddots&\vdots\\\frac{\partial F_n}{\partial\ {\dot{\overline{x}}}_1}&\ldots&\frac{\partial F_n}{\partial\ {\dot{\overline{x}}}_n}\\\end{matrix}\right)=\ E	
(4.130)

	\frac{\partial F({\bar{\eta}}_1)}{\partial\bar{x}}=\left(\begin{matrix}\frac{\partial F_1}{\partial\ {\bar{x}}_1}&\ldots&\frac{\partial F_1}{\partial\ {\bar{x}}_n}\\\vdots&\ddots&\vdots\\\frac{\partial F_n}{\partial\ {\bar{x}}_1}&\ldots&\frac{\partial F_n}{\partial\ {\bar{x}}_n}\\\end{matrix}\right)=\ A\prime	
(4.131)

	\frac{\partial F({\bar{\eta}}_1)}{\partial\bar{u}}=\left(\begin{matrix}\frac{\partial F_1}{\partial\ {\bar{u}}_1}&\ldots&\frac{\partial F_1}{\partial\ {\bar{u}}_n}\\\vdots&\ddots&\vdots\\\frac{\partial F_n}{\partial\ {\bar{u}}_1}&\ldots&\frac{\partial F_n}{\partial\ {\bar{u}}_n}\\\end{matrix}\right)=\ B\prime	
(4.132)

	\frac{\partial F({\bar{\eta}}_1)}{\partial\bar{d}}=\left(\begin{matrix}\frac{\partial F_1}{\partial\ {\bar{d}}_1}&\ldots&\frac{\partial F_1}{\partial\ {\bar{d}}_n}\\\vdots&\ddots&\vdots\\\frac{\partial F_n}{\partial\ {\bar{d}}_1}&\ldots&\frac{\partial F_n}{\partial\ {\bar{d}}_n}\\\end{matrix}\right)=\ D\prime	
(4.133)

Jacobians are given in equations (4.130), (4.131), (4.132), and (4.133). Regarding the equation (4.125), the terms F(\dot{\overline{x}},\ \bar{x},\bar{u},\bar{d}) and F({\dot{\overline{x}}}_1,{\bar{x}}_1,{\bar{u}}_1,{\bar{d}}_1) are expected to be zero because the flight conditions are steady-state flight conditions. Therefore, the equation can be established.
	0=E∆x+A'∆x+B'∆u+D'∆d
(4.134)

Using the equation (4.134), we can obtain state states form,
	∆x=-E-1A'∆x-E-1B'∆u-E-1D'∆d
(4.135)

Consequently, the state matrix, A, input matrix, B, and the disturbance matrix, D, can be obtained using equations (4.136), (4.137), and (4.138).
 	A=-E^{-1}A\prime	(4.136)

	B=-E^{-1}B\prime	(4.137)

	D={-E}^{-1}D\prime	(4.138)

The extraction of the Jacobians column by column creates the basis for the stated algorithm to linearize the system numerically. The discrepancy between the current column and the previously calculated column is controlled by the convergence tolerance value 1e-6. It moves on to the calculations for the following column if the difference is less than the tolerance value. 
 
Figure 4.12: Numerical linearization algorithm.
In reference of (Stevens, Lewis, & Johnson, 2015) algorithm of the numerical method which is also seen in the Figure 4.12 is described in detail. The user determines the precision value, state vector, control input vector and step size of the numerical derivation. Then, the algorithm starts with finding trim data and initial perturbation of the model regarding non-linear continuous state equation. Then, step size is reduced, and partial derivative calculation is performed. The algorithm adaptively reduces the step size during the numerical analysis and each step the precision is controlled by checking the relative error for each reduced step size adaptively. If the relative error is within the precision value given by the user, then the calculation is counted as converged and algorithm starts to calculate next derivative. This operation is done for matrix E, A\prime, C\prime and D\prime.
The linear state equations of the aircraft for are found by performing numerical linearization, while the aircraft mode and state matrices were found throughout the transition. Accordingly, the longitudinal and lateral state matrices are given in equations (4.139) and (4.140).
	A_{lon}=\left(\begin{matrix}X_u&X_w&X_q&-g\ cos\theta_1&0\\Z_u&Z_w&Z_q&-g\ sin\theta_1&0\\M_u&M_w&M_q&0&0\\0&0&1&0&0\\0&-1&0&V_1&0\\\end{matrix}\right)	
(4.139)

	A_{lat}=\left(\begin{matrix}Y_v&Y_p&Y_r&g\ cos\theta_1&0\\L_v&L_p&L_r&0&0\\N_v&N_p&N_r&0&0\\0&1&tan\theta_1&0&0\\0&0&1&0&0\\\end{matrix}\right)	
(4.140)

The uppercase symbols in the matrices indicate the force or moment derivatives, while the subscripts indicate the relevant state element. g is the gravitational acceleration, \theta_1 is the trim pitch and V_1 is the wind speed in the trim state. In addition, the states are as shown in equation (4.141).
	\mathbit{x}_{lon}=\left(\begin{matrix}u&w&\begin{matrix}q&\theta&h\\\end{matrix}\\\end{matrix}\right)^T,\ \ \mathbit{x}_{lat}=\left(\begin{matrix}v&p&\begin{matrix}r&\phi&\psi\\\end{matrix}\\\end{matrix}\right)^T	(4.141)

Input matrices are also obtained in numerical linearization. Below are the input matrices for longitudinal and lateral dynamics.
	B_{lon,ff}=\left(\begin{matrix}X_{\delta e}&X_T\\Z_{\delta e}&Z_T\\M_{\delta e}&M_T\\0&0\\0&0\\\end{matrix}\right)	
(4.142)

	B_{lat,ff}=\left(\begin{matrix}Y_{\delta a}&Y_{\delta r}\\L_{\delta a}&L_{\delta r}\\N_{\delta a}&N_{\delta r}\\0&0\\0&0\\\end{matrix}\right)	
(4.143)

Regarding the transition dynamics of the aircraft, there will be additional input variables obtained from quadrotor dynamics. Therefore, the matrices are 5x4 in size and there are eight control inputs in total. The linearized longitudinal and lateral control matrices are shown in (4.144) and (4.145).
	B_{lon,tr}=\left(\begin{matrix}X_{\delta e}&X_T&0&0\\Z_{\delta e}&Z_T&-\frac{1}{m}&0\\M_{\delta e}&M_T&0&\frac{a}{I_{yy}}\\0&0&0&0\\0&0&0&0\\\end{matrix}\right)	
(4.144)

	B_{lat,tr}=\left(\begin{matrix}Y_{\delta a}&Y_{\delta r}&0&0\\L_{\delta a}&L_{\delta r}&\frac{a}{I_{xx}}&0\\N_{\delta a}&N_{\delta r}&0&\frac{1}{I_{zz}}\\0&0&0&0\\0&0&0&0\\\end{matrix}\right)	
(4.145)

For the trim analysis whose conditions are given in equation (4.99), the non-linear state equations are linearized with regard to given numerical linearization techniques. The state matrix results for longitudinal and lateral dynamics are given in equations (4.146) and (4.147).
	A_{lon}=\left(\begin{matrix}-0.134&0.517&-2.030&-9.751&0\\-0.677&-2.623&19.027&-1.044&0\\0.263&-2.473&-5.197&0&0\\0&0&1&0&0\\0&-0.994&0&20.5&0\\\end{matrix}\right)	
(4.146)

	A_{lat}=\left(\begin{matrix}-0.298&2.207&-19.992&9.751&0\\0.624&-6.848&2.547&0&0\\0.997&-1.375&-0.859&0&0\\0&1&0.107&0&0\\0&0&1.006&0&0\\\end{matrix}\right)	
(4.147)

Also, the longitudinal and lateral input matrices are given in equations (4.148) and (4.149).
	B_{lon,ff}=\left(\begin{matrix}0&5.233\\-6.996&0\\-13.952&0\\0&0\\0&0\\\end{matrix}\right)	
(4.148)

	B_{lat,ff}=\left(\begin{matrix}0&3.079\\27.887&-0.420\\2.657&-8.965\\0&0\\0&0\\\end{matrix}\right)	
(4.149)


