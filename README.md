# SAFE-RL-Brake-Controller
A RL braking controller that must stop a vehicle safely under varying road conditions (friction, slope, noise) without ever crashing, using a safety layer that overrides dangerous actions.

---

# Theory

## 1. What system are we controlling?

A 1-D vehicle that moves in a straight line toward an obstacle and is able to apply braking force.  
The goal is to model the longitudinal motion (speed + distance).

**State variables:**
- $v$: vehicle speed (m/s)
- $d$: distance to the obstacle (m)
- $\mu$: road friction coefficient

**Control input:**
- $a$: acceleration (m/s²) (in this case negative due to braking)

The control problem can be defined as:  
“Given the current speed and distance to an obstacle, apply braking to stop safely and efficiently, under various road conditions.”

---

## 2. Friction

Friction determines the maximum braking acceleration:

$$
a_{\max,\text{friction}} = \mu g
$$

Due to friction the control problem becomes multi-conditional, e.g. good braking on dry asphalt can turn into sliding on ice.

---

## 3. Vehicle Dynamics

Assuming the car is a simple point-mass model, we use discrete-time updates:

$$
\begin{aligned}
v_{t+1} &= v_t + a_t\Delta t \\
d_{t+1} &= d_t + v_t\Delta t
\end{aligned}
$$

where $\Delta t$ is the simulation time step.

---

## 4. Stopping Distance

The ideal stopping distance for constant deceleration is:

$$
d_{\text{stop}} = \frac{v^2}{2|a|}
$$

If friction limits the deceleration to $a = \mu g$, then:

$$
d_{\text{stop}} = \frac{v^2}{2 \mu g}
$$
