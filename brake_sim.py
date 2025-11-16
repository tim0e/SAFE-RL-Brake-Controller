# ------------------------
# SAFE BRAKE RL CONTROLLER
# ------------------------

# initial state paramters
initial_state = {
    "v": 80,            # velocity [km/h]
    "d": 200,           # distance [m]
    "mu": 1.0,           # coefficient for road conditions 
}


# 1. Global Parameters
params = {
    "g": 9.81,          # gravity
    "dt": 0.05,         # time step delta t [s]
    "a_max": 8,         # maximum acceleration [m/s^2]
}

# 2. Dynamics x_{t+1} = f(x_t, a_t)
def step_function():
    """ One Step Simulation of the vehicle dynamics."""
    # unpack variables
    v = state["v"]
    d = state["d"]
    mu = state["mu"]

    # calculate the next step
    

