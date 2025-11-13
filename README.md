# SAFE-RL-Brake-Controller
A RL braking controller that must stop a vehicle safely under varying road conditions (friction, slope, noise) without ever crashing, using a safety layer that overrides dangerous actions.

# Theory
1. What system are we controlling?
A 1-D vehicle that moves in a straight line toward an obstacle and is able to apply braking force.
The goal is to model the longitudal motion (speed + distance).

In extend this means
- State variables:
    - v: vehicle speed (m/s)
    - d: distance to the obstacle (m)
    - mu: road friction coefficent
- Control input:
    - a: acceleration (m/s^2) (in this case the acceleration will be negative, due to braking)
 
The control problem can be defined as: 
"Given the current speed and distance to an obstacle, apply braking to stop safely and efficiently, under various road conditions"

2. Friction
Friction determines the maximum braking force.
a_max,friction = mu*g
Due to friction the control problem becomes multi-conditional, e.g. good braking on dry asphalt can turn into sliding on ice.
