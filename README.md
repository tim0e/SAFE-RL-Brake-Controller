# SAFE-RL-Brake-Controller
A RL braking controller that must stop a vehicle safely under varying road conditions (friction, slope, noise) without ever crashing, using a safety layer that overrides dangerous actions.

# Theory
1. What system are we controlling?
<br>
A 1-D vehicle that moves in a straight line toward an obstacle and is able to apply braking force.
<br>
The goal is to model the longitudal motion (speed + distance).
<br>
In extend this means
- State variables:
    - $v$: vehicle speed ($m/s$)
    - $d$: distance to the obstacle ($m$)
    - $\mu$: road friction coefficent
- Control input:
    - $a$: acceleration ($m/s^2$) (in this case the acceleration will be negative, due to braking)
 
The control problem can be defined as: 
<br>
"Given the current speed and distance to an obstacle, apply braking to stop safely and efficiently, under various road conditions"
<br>
2. Friction
Friction determines the maximum braking force.
$a_{max,friction} = \mu *g$
Due to friction the control problem becomes multi-conditional, e.g. good braking on dry asphalt can turn into sliding on ice.
<br>
3. Vehicle dynamics

Assuming the car is a simple point-mass model:
<br>
Velocity update:
<br>
$$v_{t+1} = v_t - a*\Delta t$$
Distance update:
$$d_{t+1} = d_t - v_t * \Delta t$$
where $\Delta t$ = simulation step

4. Stopping distance
<br>
The ideal stopping distance for constant deceleration is
$$d_stop = v^2/2a$$
If friction limits the acceleration to mu*g
$$d_{stop} = v^2/2 \mu *g$$

