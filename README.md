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

### 4.1. Mathematic Background 

Position: $$x(t)$$ 
Velocity: $$v(t) = \frac{dx}{dt}$$ 
Acceleration: $$a(t) = \frac{dv}{dt}$$


For constant acceleration:
$$ a(t) = a = const $$

So we have the definition for acceleration:
$$ \frac{dv}{dt} = a $$

or rearranged
$$ dv = a dt $$

To get velocity as a function of time we need to integrate both sides from initial time $t_0$ within initial velocity $v_0$, to time $t$ with velocity $v$:


So we have the differential equation:
$$ \int_{v_0}^{v} dv= \int_{t_0}^{t} a dt $$

So (with a constant):
$$ v - v_0 = a(t - t_0) $$

Choosing $t_0 = 0$, the first standard kinematic equation is derived.
$$ \boxed{ v(t) = v_0 + at } $$

The next step is to relate velocity and position. We already know the definition of velocity:
$$ v = \frac{dx}{dt} $$

Input of the derived equation from earlier and we obtain:
$$ \frac{dx}{dt} = v_0 + at $$
$$ dx = (v_0 + at)dt $$

Integrate both sides again:
$$ \int_{x_0}^{x} dx= \int_{0}^{t} (v_0 + a\tau)d\tau $$
$$ x - x_0 = v_0t + \frac{1}{2}at^2 $$

And lets define displacement $ s = x - x_0 $:

$$ \boxed{ s = v_0t + \frac{1}{2}at^2 } $$

We have now the two equations we need to understand the braking distance. We want these to relate to each other - not depending on time anymore.

$$ v = v_0 + at \rightarrow t = \frac{v - v_0}{a} $$
$$ s = v_0\left(\frac{v - v_0}{a}\right) + \frac{1}{2}a\left(\frac{v - v_0}{a}\right)^2 $$
$$ s = \frac{1}{a} \left[ v_0(v - v_0) + \frac{1}{2}(v - v_0)^2 \right] $$
$$ s = \frac{1}{a} \left[ v_0v - v_0^2 + \left( \frac{1}{2}v^2 - v v_0 + \frac{1}{2}v_0^2 \right) \right] $$
$$ s = \frac{v^2 - v_0^2}{2a} $$

After rearranging we finally yield:
$$ \boxed{ v^2 = v_0^2 + 2as } $$

