# ciconia-trim-analysis

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

|     |                                                                               
       **x**<sub>**d**</sub>**=****(***u*, *v*, *w*, *p*, *q*, *r*)<sup>**T**</sup>   | (4.79) |
|-----|-------------------------------------------------------------------------------|--------|
|     |                                                                               
       **x**<sub>**k**</sub>**=****(***x*, *y*, *z*, *ϕ*, *θ*, *ψ*)<sup>**T**</sup>   | (4.80) |

**x**<sub>**d**</sub> is the dynamic state vector and
**x**<sub>**k**</sub> is a kinematic state vector. Also, the control
input is defined in equation (4.81), for the forward flight.

|     |                                                                    |        |
|-----|--------------------------------------------------------------------|--------|
|     |                                                                    
       **u**<sub>ff</sub> = (*δ*<sub>*T*</sub>, δe, δa, δr)<sup>*T*</sup>  | (4.81) |

Where *δ*<sub>*T*</sub> is throttle setting, δe is elevator deflection,
δa is aileron deflection and δr is rudder deflection. Because, in the
transition state, the propulsion system for the hover state will also be
used, the control input vector becomes as in equation (4.82).

|     |                                                                                            |        |
|-----|--------------------------------------------------------------------------------------------|--------|
|     |                                                                                            
       **u**<sub>tr</sub> = (*δ*<sub>*T*</sub>, δe, δa, δr, *u*1, *u*2, *u*3, *u*4)<sup>*T*</sup>  | (4.82) |

*u*1, *u*2, *u*3, and *u*4 are defined as *the* force in the x-axis, the
force that creates the pitching moment, and the force that creates the
rolling moment and yawing moment, respectively. Also, one can define the
implicit form of the non-linear aircraft state equation for the trim
state in equation (4.83).

|     |                                                       |        |
|-----|-------------------------------------------------------|--------|
|     |                                                       
       $$g(\\dot{\\mathbf{x}},\\mathbf{x},\\mathbf{u}) = 0$$  | (4.83) |

Before establishing, the cost function for the optimization problem,
necessary parameters should be constructed. Translational dynamic,
rotational dynamic, translational kinematic, and rotational kinematic
cost function parameters are given in equations (4.84), (4.85), (4.86),
and (4.87), respectively.

|     |                                                                                          
       *f*<sub>*T**D*1</sub> = *u̇*,  *f*<sub>*T**D*2</sub> = *v̇*,  *f*<sub>*T**D*3</sub> = *ẇ*   | (4.84) |
|-----|------------------------------------------------------------------------------------------|--------|
|     |                                                                                          
       *f*<sub>*R**D*1</sub> = *ṗ*,  *f*<sub>*R**D*2</sub> = *q̇*,  *f*<sub>*R**D*3</sub> = *ṙ*   | (4.85) |
|     |                                                                                          
       *f*<sub>*T**K*1</sub> = *ẋ*,  *f*<sub>*T**K*2</sub> = *ẏ*,  *f*<sub>*T**K*3</sub> = *ż*   | (4.86) |
|     |                                                                                          
       *f*<sub>*R**K*1</sub> = *ϕ̇*,  *f*<sub>*R**K*2</sub> = *θ̇*,  *f*<sub>*R**K*3</sub> = *ψ̇*   | (4.87) |

However, considering the construction of the trim problem in order to
add flight path angle constraint to the cost function,
*f*<sub>*T**K*3</sub> can be written as

|     |                                                                                                                                                                                        |        |
|-----|----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                                                                                        
       *f*<sub>*T**K*3</sub> = *s**i**n**γ* − \[(*c**o**s**α* *c**o**s**β* *s**i**n**θ*) − (*s**i**n**β* *c**o**s**θ* *s**i**n**ϕ*) − (*s**i**n**α* *c**o**s**β* *c**o**s**θ* *c**o**s**ϕ*)\]  | (4.88) |

These cost function parameters are obtained by using the equation of
motion of the hybrid VTOL aircraft as it was discussed in section 4.2.1.
For general straight flight trim analysis, the trim states are chosen as
shown in equation (4.89).

|     |                                                                                                                                             |        |
|-----|---------------------------------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                                             
       **ζ** = (*V*<sub>*T*</sub>, *β*, *α*, (*u*, *v*, *w*), *p*, *q*, *r*, *ϕ*, *θ*, *ψ*, *h*, *γ*, *δ*<sub>*T*</sub>, δe, δa, δr)<sup>*T*</sup>  | (4.89) |

Where specified and unknown trim state vectors are given in equations
(4.90) and (4.91),

|     |                                                                                                                     
       **ζ**<sub>specified</sub> = (*V*<sub>*T*</sub>, *h*, *γ*, *ϕ*)<sup>*T*</sup>                                         | (4.90) |
|-----|---------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                     
       **ζ**<sub>unknown</sub> = ( *β*, *α*, *p*, *q*, *r*, *θ*, *ψ*, *δ**e*, δa, *δ**r*, *δ*<sub>*T*</sub>)<sup>*T*</sup>  | (4.91) |

*ζ*<sub>specified</sub> is actually defined as the trim condition of the
aircraft. *h* determines the atmospheric conditions, *γ* determines
whether the aircraft descending or ascending. If a turn state is
analyzed, *ϕ* term can be given as input other than zero. The
generalized trim cost function is defined in equation (4.92).

|     |                                                                                                                                                                                                                                                                                                                                                                                                                 |        |
|-----|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                                                                                                                                                                                                                                                                                                                 
       *J* = *f*<sub>*T**D*1</sub><sup>2</sup> + *f*<sub>*T**D*2</sub><sup>2</sup> + *f*<sub>*T**D*3</sub><sup>2</sup> + *f*<sub>*R**D*1</sub><sup>2</sup> + *f*<sub>*R**D*2</sub><sup>2</sup> + *f*<sub>*R**D*3</sub><sup>2</sup> + *f*<sub>*R**K*1</sub><sup>2</sup> + *f*<sub>*R**K*2</sub><sup>2</sup> + *f*<sub>*R**K*3</sub><sup>2</sup> + *f*<sub>*T**K*2</sub><sup>2</sup> + *f*<sub>*T**K*3</sub><sup>2</sup>  | (4.92) |

Then, the optimization problem can be solved with 11 unknown and 11
known variables. The trim analysis is also run for transition dynamics
for the aircraft. Additional variables are the propulsion system for the
hover state of the aircraft. However, in order to make a number of
unknown and known equal some changes should be done. Thus, the
deflection of control surfaces is specified as 0 deflections. Also, one
additional parameter which is *θ*, is set to zero. Additionally, moment
and forces are considered to be compensated by the hover propulsion
system.

|     |                                                                                                                                                         |        |
|-----|---------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                                                         
       **ζ** = (*V*<sub>*T*</sub>, *β*, *α*, (*u*, *v*, *w*), *p*, *q*, *r*, *ϕ*, *θ*, *ψ*, *h*, *γ*, *δ*<sub>*T*</sub>, *u*1, *u*2, *u*3, *u*4)<sup>*T*</sup>  | (4.93) |

Where specified and unknown trim state vectors are given in equations
(4.94) and (4.95).

|     |                                                                                                                                             
       **ζ**<sub>specified</sub> = (*V*<sub>*T*</sub>, *h*, *γ*, *ϕ*, *θ* = 0, *δ**e* = 0, *δ**a* = 0, *δ**r* = 0)<sup>*T*</sup>                    | (4.94) |
|-----|---------------------------------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                                             
       **ζ**<sub>unknown</sub> = ( *β*, *α*, *p*, *q*, *r*, *θ*, *ψ*, *δ**e*, δa, *δ**r*, *δ*<sub>*T*</sub>, *u*1, *u*2, *u*3, *u*4)<sup>*T*</sup>  | (4.95) |

The cost function for transition trim analysis is defined the same as
the general straight flight trim analysis which is established in
equation (4.92).

On the other hand, another trim analysis type which is steady-state turn
flight is performed in this thesis. Additional to the general straight
flight trim problem, the side-slip angle, *β*, is specified, body rates
are removed, and yaw rate is left optional to user definition, regarding
the trim state vector as it is seen in equation (4.96) and (4.97).

|     |                                                                                                              
       **ζ**<sub>specified</sub> = (*V*<sub>*T*</sub>, *h*, *γ*, *β*, *ϕ* (*o**r*  *ψ̇*))<sup>*T*</sup>               | (4.96) |
|-----|--------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                              
       **ζ**<sub>unknown</sub> = (*α*, *θ*, *ψ̇* (*o**r* *ϕ*) , *δ**e*, δa, *δ**r*, *δ*<sub>*T*</sub>)<sup>*T*</sup>  | (4.97) |

Therefore, in the trim state vector, there are seven unknown variables.
Because turning concerns the variables in the planar position, related
constraints are removed from the cost function as it is seen in equation
(4.98).

|     |                                                                                                                                                                                                                                                                 |        |
|-----|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|--------|
|     |                                                                                                                                                                                                                                                                 
       *J* = *f*<sub>*T**D*1</sub><sup>2</sup> + *f*<sub>*T**D*2</sub><sup>2</sup> + *f*<sub>*T**D*3</sub><sup>2</sup> + *f*<sub>*R**D*1</sub><sup>2</sup> + *f*<sub>*R**D*2</sub><sup>2</sup> + *f*<sub>*R**D*3</sub><sup>2</sup> + *f*<sub>*T**K*3</sub><sup>2</sup>  | (4.98) |

Therefore, for the optimization problem, there are seven unknown
parameters to be solved by using another seven known parameters.

<img src="media/image1.png" style="width:5.66783in;height:1.06796in" alt="Graphical user interface, diagram Description automatically generated" />

**Figure** **4.10:** Trim analysis algorithm.

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
of them is that wind velocity, *V*<sub>*T*</sub>, is 20.5 m/s, altitude,
*h*, is 0 meters, flight path angle, *γ*, is 0 degrees as it is seen in
(4.99).

|     |                                                           |        |
|-----|-----------------------------------------------------------|--------|
|     |                                                           
       **ζ**<sub>specified</sub> = (20.5, 0, 0, 0)<sup>*T*</sup>  | (4.99) |

The initialization of the trim vector can be seen as it is in equation
(4.100).

|                                                                                                    |         |
|----------------------------------------------------------------------------------------------------|---------|
|                                                                                                    
 **ζ**<sub>unknown</sub> = ( 0.0, 0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.0, 1.0)<sup>*T*</sup>  | (4.100) |

After the algorithm converges, the equation (4.101) is obtained as the
solution to the problem.

|                                                                                                            |         |
|------------------------------------------------------------------------------------------------------------|---------|
|                                                                                                            
 **ζ**<sub>unknown</sub> = ( 0.0, 0.106, 0.0, 0.0, 0.0, 0.106, 0.0,  − 0.20, 0.0, 0.0, 0.579)<sup>*T*</sup>  | (4.101) |

For the case of steady-turn trim problem definition, the trim state
vector is defined in (4.102). wind velocity, *V*<sub>*T*</sub>, is 20.5
m/s, altitude, *h*, is 0 meters, flight path angle, *γ*, is 0 and bank
angle, *ϕ*, is 45<sup>∘</sup>.

|     |                                                                                 |         |
|-----|---------------------------------------------------------------------------------|---------|
|     |                                                                                 
       **ζ**<sub>specified</sub> = (20.5, 0, 0, 0, *ϕ* = 45<sup>∘</sup>)<sup>*T*</sup>  | (4.102) |

And the trim state vector is initialized as in equation (4.103).

|                                                                           |         |
|---------------------------------------------------------------------------|---------|
|                                                                           
 **ζ**<sub>unknown</sub> = (0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 1)<sup>*T*</sup>  | (4.103) |

The solution of the steady-turn trim analysis is given in equation
(4.104).

|                                                                                           |         |
|-------------------------------------------------------------------------------------------|---------|
|                                                                                           
 **ζ**<sub>unknown</sub> = (0.020, 0.024, 0, 0.0280, 0.002,  − 0.003, 0.505)<sup>*T*</sup>  | (4.104) |

### Numerical linearization of flight and transition dynamics

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

|     |                                                                            |         |
|-----|----------------------------------------------------------------------------|---------|
|     |                                                                            
       $$\\dot{\\overline{x}}\\  = f(\\overline{x},\\overline{u},\\overline{d})$$  | (4.121) |

where $\\dot{\\overline{x}}\\mathbf{\\ }\\epsilon\\ \\mathbb{R}^{n}$,
$\\overline{x}\\mathbf{\\ }\\epsilon\\ \\mathbb{R}^{n}$,
$\\overline{u}\\mathbf{\\ }\\epsilon\\ \\mathbb{R}^{m}$ and
$\\overline{d}\\mathbf{\\ }\\epsilon\\ \\mathbb{R}^{k}$. n is the number
of states, m is the number of inputs and k is the number of disturbance
variables. The explicit form of a nonlinear system can be written as the
implicit form in the equation (4.122).

|     |                                                                                                                                                       |         |
|-----|-------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                                                       
       $$0 = f(\\overline{x},\\overline{u},\\overline{d}) - \\dot{\\overline{x}}\\  = F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d})$$  | (4.122) |

If all variables are stacked into one called *η*, the equation (4.123)
can be written.

|     |                                                                              |         |
|-----|------------------------------------------------------------------------------|---------|
|     |                                                                              
       $$0 = F(\\overline{\\eta}),\\ \\ \\overline{\\eta}\\  = \\ \\begin{pmatrix}   
       \\dot{\\overline{x}} & \\ \\overline{x} & \\overline{u} & \\overline{d} \\\\  
       \\end{pmatrix}^{T}$$                                                          | (4.123) |

where $\\overline{\\eta}\\epsilon\\ \\mathbb{R}^{2n + m + k}$. If the
Taylor series expansion is written for $F(\\overline{\\eta})$, we can
obtain equation (4.124).

|     |                                                                                                                                                                                                                                                                                                                                                                                                                                                     |         |
|-----|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                                                                                                                                                                                                                                                                                                                                                     
       $$F(\\overline{\\eta}) = F({\\overline{\\eta}}\_{1}) + \\left. \\ \\frac{\\partial F(\\overline{\\eta})}{\\partial\\overline{\\eta}} \\right\|\_{{\\overline{\\eta}}\_{1}}\\left( \\overline{\\eta} - {\\overline{\\eta}}\_{1} \\right) + \\left. \\ \\frac{\\partial^{2}F(\\overline{\\eta})}{\\partial{\\overline{\\eta}}^{2}} \\right\|\_{{\\overline{\\eta}}\_{1}}\\left( \\overline{\\eta} - {\\overline{\\eta}}\_{1} \\right)^{2} + \\cdots$$  | (4.124) |

And if the $\\overline{\\eta}$ the term is expanded, the equation
(4.125) can be written for the first gradient.

|     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   |         |
|-----|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
       $$F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d}) = F({\\dot{\\overline{x}}}\_{1},{\\overline{x}}\_{1},{\\overline{u}}\_{1},{\\overline{d}}\_{1}) + \\frac{\\partial F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d})}{\\partial\\dot{\\overline{x}}}\\left( \\dot{\\overline{x}} - {\\dot{\\overline{x}}}\_{1} \\right) + \\frac{\\partial F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d})}{\\partial\\ \\overline{x}}\\left( \\ \\overline{x} - {\\overline{x}}\_{1} \\right) + \\frac{\\partial F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d})}{\\partial\\ \\overline{u}}\\left( \\ \\overline{u} - {\\overline{u}}\_{1} \\right) + \\frac{\\partial F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d})}{\\partial\\ \\overline{d}}\\left( \\ \\overline{d} - {\\overline{d}}\_{1} \\right) + \\cdots$$  | (4.125) |

In equation (4.125), the subscript “1” refers to the trim state, related
to the steady-state flight. For the sake of simplicity, the difference
term can be written as,

|     |                                                                                                                 
       $$\\mathrm{\\Delta}\\dot{\\overline{x}} = \\left( \\dot{\\overline{x}} - {\\dot{\\overline{x}}}\_{1} \\right)$$  | (4.126) |
|-----|-----------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                 
       $$\\mathrm{\\Delta}\\overline{x} = \\left( \\ \\overline{x} - {\\overline{x}}\_{1} \\right)$$                    | (4.127) |
|     |                                                                                                                 
       $$\\mathrm{\\Delta}\\overline{u} = \\left( \\ \\overline{u} - {\\overline{u}}\_{1} \\right)$$                    | (4.128) |
|     |                                                                                                                 
       $$\\mathrm{\\Delta}\\overline{d} = \\left( \\ \\overline{d} - {\\overline{d}}\_{1} \\right)$$                    | (4.129) |

Then each Jacobian matrix can be written as follow,

|     |                                                                                                                                                      
       $$\\frac{\\partial F({\\overline{\\eta}}\_{1})}{\\partial\\dot{\\overline{x}}} = \\begin{pmatrix}                                                     
       \\frac{\\partial F\_{1}}{\\partial\\ {\\dot{\\overline{x}}}\_{1}} & \\ldots & \\frac{\\partial F\_{1}}{\\partial\\ {\\dot{\\overline{x}}}\_{n}} \\\\  
        \\vdots & \\ddots & \\vdots \\\\                                                                                                                     
       \\frac{\\partial F\_{n}}{\\partial\\ {\\dot{\\overline{x}}}\_{1}} & \\ldots & \\frac{\\partial F\_{n}}{\\partial\\ {\\dot{\\overline{x}}}\_{n}} \\\\  
       \\end{pmatrix} = \\ E$$                                                                                                                               | (4.130) |
|-----|------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                                                      
       $$\\frac{\\partial F({\\overline{\\eta}}\_{1})}{\\partial\\overline{x}} = \\begin{pmatrix}                                                            
       \\frac{\\partial F\_{1}}{\\partial\\ {\\overline{x}}\_{1}} & \\ldots & \\frac{\\partial F\_{1}}{\\partial\\ {\\overline{x}}\_{n}} \\\\                
        \\vdots & \\ddots & \\vdots \\\\                                                                                                                     
       \\frac{\\partial F\_{n}}{\\partial\\ {\\overline{x}}\_{1}} & \\ldots & \\frac{\\partial F\_{n}}{\\partial\\ {\\overline{x}}\_{n}} \\\\                
       \\end{pmatrix} = \\ A'$$                                                                                                                              | (4.131) |
|     |                                                                                                                                                      
       $$\\frac{\\partial F({\\overline{\\eta}}\_{1})}{\\partial\\overline{u}} = \\begin{pmatrix}                                                            
       \\frac{\\partial F\_{1}}{\\partial\\ {\\overline{u}}\_{1}} & \\ldots & \\frac{\\partial F\_{1}}{\\partial\\ {\\overline{u}}\_{n}} \\\\                
        \\vdots & \\ddots & \\vdots \\\\                                                                                                                     
       \\frac{\\partial F\_{n}}{\\partial\\ {\\overline{u}}\_{1}} & \\ldots & \\frac{\\partial F\_{n}}{\\partial\\ {\\overline{u}}\_{n}} \\\\                
       \\end{pmatrix} = \\ B'$$                                                                                                                              | (4.132) |
|     |                                                                                                                                                      
       $$\\frac{\\partial F({\\overline{\\eta}}\_{1})}{\\partial\\overline{d}} = \\begin{pmatrix}                                                            
       \\frac{\\partial F\_{1}}{\\partial\\ {\\overline{d}}\_{1}} & \\ldots & \\frac{\\partial F\_{1}}{\\partial\\ {\\overline{d}}\_{n}} \\\\                
        \\vdots & \\ddots & \\vdots \\\\                                                                                                                     
       \\frac{\\partial F\_{n}}{\\partial\\ {\\overline{d}}\_{1}} & \\ldots & \\frac{\\partial F\_{n}}{\\partial\\ {\\overline{d}}\_{n}} \\\\                
       \\end{pmatrix} = \\ D'$$                                                                                                                              | (4.133) |

Jacobians are given in equations (4.130), (4.131), (4.132), and (4.133).
Regarding the equation (4.125), the terms
$F(\\dot{\\overline{x}},\\ \\overline{x},\\overline{u},\\overline{d})$
and
$F({\\dot{\\overline{x}}}\_{1},{\\overline{x}}\_{1},{\\overline{u}}\_{1},{\\overline{d}}\_{1})$
are expected to be zero because the flight conditions are steady-state
flight conditions. Therefore, the equation can be established.

|     |                                                                                                                                                         |         |
|-----|---------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                                                         
       $$0 = E\\mathrm{\\Delta}\\dot{\\overline{x}} + A'\\mathrm{\\Delta}\\overline{x} + B'\\mathrm{\\Delta}\\overline{u} + D'\\mathrm{\\Delta}\\overline{d}$$  | (4.134) |

Using the equation (4.134), we can obtain state states form,

|     |                                                                                                                                                                           |         |
|-----|---------------------------------------------------------------------------------------------------------------------------------------------------------------------------|---------|
|     |                                                                                                                                                                           
       $$\\mathrm{\\Delta}\\dot{\\overline{x}} = - E^{- 1}A'\\mathrm{\\Delta}\\overline{x} - E^{- 1}B'\\mathrm{\\Delta}\\overline{u} - E^{- 1}D'\\mathrm{\\Delta}\\overline{d}$$  | (4.135) |

Consequently, the state matrix, *A*, input matrix, *B*, and the
disturbance matrix, *D*, can be obtained using equations (4.136),
(4.137), and (4.138).

|     |                                 
       *A* =  − *E*<sup> − 1</sup>*A*′  | (4.136) |
|-----|---------------------------------|---------|
|     |                                 
       *B* =  − *E*<sup> − 1</sup>*B*′  | (4.137) |
|     |                                 
       *D* =  − *E*<sup> − 1</sup>*D*′  | (4.138) |

The extraction of the Jacobians column by column creates the basis for
the stated algorithm to linearize the system numerically. The
discrepancy between the current column and the previously calculated
column is controlled by the convergence tolerance value 1e-6. It moves
on to the calculations for the following column if the difference is
less than the tolerance value.

<img src="media/image2.png" style="width:4.16667in;height:5.10442in" alt="Diagram Description automatically generated" />

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
operation is done for matrix *E*, *A*′, *C*′ and *D*′.

The linear state equations of the aircraft for are found by performing
numerical linearization, while the aircraft mode and state matrices were
found throughout the transition. Accordingly, the longitudinal and
lateral state matrices are given in equations (4.139) and (4.140).

|     |                                                                 
       $$A\_{\\text{lon}} = \\begin{pmatrix}                            
       X\_{u} & X\_{w} & X\_{q} & - g\\ cos\\theta\_{1} & 0 \\\\        
       Z\_{u} & Z\_{w} & Z\_{q} & - g\\ sin\\theta\_{1} & 0 \\\\        
       M\_{u} & M\_{w} & M\_{q} & 0 & 0 \\\\                            
       0 & 0 & 1 & 0 & 0 \\\\                                           
       0 & - 1 & 0 & V\_{1} & 0 \\\\                                    
       \\end{pmatrix}$$                                                 | (4.139) |
|-----|-----------------------------------------------------------------|---------|
|     |                                                                 
       $$A\_{\\text{lat}} = \\begin{pmatrix}                            
       Y\_{v} & Y\_{p} & Y\_{r} & \\text{g\\ cos}\\theta\_{1} & 0 \\\\  
       L\_{v} & L\_{p} & L\_{r} & 0 & 0 \\\\                            
       N\_{v} & N\_{p} & N\_{r} & 0 & 0 \\\\                            
       0 & 1 & \\tan\\theta\_{1} & 0 & 0 \\\\                           
       0 & 0 & 1 & 0 & 0 \\\\                                           
       \\end{pmatrix}$$                                                 | (4.140) |

The uppercase symbols in the matrices indicate the force or moment
derivatives, while the subscripts indicate the relevant state element.
*g* is the gravitational acceleration, *θ*<sub>1</sub> is the trim pitch
and *V*<sub>1</sub> is the wind speed in the trim state. In addition,
the states are as shown in equation (4.141).

|     |                                                                        |         |
|-----|------------------------------------------------------------------------|---------|
|     |                                                                        
       $$\\mathbf{x}\_{\\text{lon}} = \\begin{pmatrix}                         
       u & w & \\begin{matrix}                                                 
       q & \\theta & h \\\\                                                    
       \\end{matrix} \\\\                                                      
       \\end{pmatrix}^{T},\\ \\ \\mathbf{x}\_{\\text{lat}} = \\begin{pmatrix}  
       v & p & \\begin{matrix}                                                 
       r & \\phi & \\psi \\\\                                                  
       \\end{matrix} \\\\                                                      
       \\end{pmatrix}^{T}$$                                                    | (4.141) |

Input matrices are also obtained in numerical linearization. Below are
the input matrices for longitudinal and lateral dynamics.

|     |                                        
       $$B\_{lon,ff} = \\begin{pmatrix}        
       X\_{\\text{δe}} & X\_{T} \\\\           
       Z\_{\\text{δe}} & Z\_{T} \\\\           
       M\_{\\text{δe}} & M\_{T} \\\\           
       0 & 0 \\\\                              
       0 & 0 \\\\                              
       \\end{pmatrix}$$                        | (4.142) |
|-----|----------------------------------------|---------|
|     |                                        
       $$B\_{lat,ff} = \\begin{pmatrix}        
       Y\_{\\text{δa}} & Y\_{\\text{δr}} \\\\  
       L\_{\\text{δa}} & L\_{\\text{δr}} \\\\  
       N\_{\\text{δa}} & N\_{\\text{δr}} \\\\  
       0 & 0 \\\\                              
       0 & 0 \\\\                              
       \\end{pmatrix}$$                        | (4.143) |

Regarding the transition dynamics of the aircraft, there will be
additional input variables obtained from quadrotor dynamics. Therefore,
the matrices are 5x4 in size and there are eight control inputs in
total. The linearized longitudinal and lateral control matrices are
shown in (4.144) and (4.145).

|     |                                                                         
       $$B\_{lon,tr} = \\begin{pmatrix}                                         
       X\_{\\text{δe}} & X\_{T} & 0 & 0 \\\\                                    
       Z\_{\\text{δe}} & Z\_{T} & - \\frac{1}{m} & 0 \\\\                       
       M\_{\\text{δe}} & M\_{T} & 0 & \\frac{a}{I\_{\\text{yy}}} \\\\           
       0 & 0 & 0 & 0 \\\\                                                       
       0 & 0 & 0 & 0 \\\\                                                       
       \\end{pmatrix}$$                                                         | (4.144) |
|-----|-------------------------------------------------------------------------|---------|
|     |                                                                         
       $$B\_{lat,tr} = \\begin{pmatrix}                                         
       Y\_{\\text{δa}} & Y\_{\\text{δr}} & 0 & 0 \\\\                           
       L\_{\\text{δa}} & L\_{\\text{δr}} & \\frac{a}{I\_{\\text{xx}}} & 0 \\\\  
       N\_{\\text{δa}} & N\_{\\text{δr}} & 0 & \\frac{1}{I\_{\\text{zz}}} \\\\  
       0 & 0 & 0 & 0 \\\\                                                       
       0 & 0 & 0 & 0 \\\\                                                       
       \\end{pmatrix}$$                                                         | (4.145) |

For the trim analysis whose conditions are given in equation (4.99), the
non-linear state equations are linearized with regard to given numerical
linearization techniques. The state matrix results for longitudinal and
lateral dynamics are given in equations (4.146) and (4.147).

|     |                                                
       $$A\_{\\text{lon}} = \\begin{pmatrix}           
        - 0.134 & 0.517 & - 2.030 & - 9.751 & 0 \\\\   
        - 0.677 & - 2.623 & 19.027 & - 1.044 & 0 \\\\  
       0.263 & - 2.473 & - 5.197 & 0 & 0 \\\\          
       0 & 0 & 1 & 0 & 0 \\\\                          
       0 & - 0.994 & 0 & 20.5 & 0 \\\\                 
       \\end{pmatrix}$$                                | (4.146) |
|-----|------------------------------------------------|---------|
|     |                                                
       $$A\_{\\text{lat}} = \\begin{pmatrix}           
        - 0.298 & 2.207 & - 19.992 & 9.751 & 0 \\\\    
       0.624 & - 6.848 & 2.547 & 0 & 0 \\\\            
       0.997 & - 1.375 & - 0.859 & 0 & 0 \\\\          
       0 & 1 & 0.107 & 0 & 0 \\\\                      
       0 & 0 & 1.006 & 0 & 0 \\\\                      
       \\end{pmatrix}$$                                | (4.147) |

Also, the longitudinal and lateral input matrices are given in equations
(4.148) and (4.149).

|     |                                  
       $$B\_{lon,ff} = \\begin{pmatrix}  
       0 & 5.233 \\\\                    
        - 6.996 & 0 \\\\                 
        - 13.952 & 0 \\\\                
       0 & 0 \\\\                        
       0 & 0 \\\\                        
       \\end{pmatrix}$$                  | (4.148) |
|-----|----------------------------------|---------|
|     |                                  
       $$B\_{lat,ff} = \\begin{pmatrix}  
       0 & 3.079 \\\\                    
       27.887 & - 0.420 \\\\             
       2.657 & - 8.965 \\\\              
       0 & 0 \\\\                        
       0 & 0 \\\\                        
       \\end{pmatrix}$$                  | (4.149) |

