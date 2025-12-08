# ------------------------
# SAFE BRAKE RL CONTROLLER
# ------------------------

# km/h to m/s
unit_convert = 3.6

# initial state paramters
initial_state = {
    "v": 80 / unit_convert,            # velocity [km/h]
    "d": 200,               # distance [m]
    "mu": 1.0,              # coefficient for road conditions 
}


# 1. Global Parameters
params = {
    "g": 9.81,          # gravity
    "dt": 0.05,         # time step delta t [s]
    "a_max": 8,         # maximum acceleration [m/s^2]
}

# 2. Dynamics x_{t+1} = f(x_t, a_t)
def step_function(state, params, a_safe):
    """ One Step Simulation of the vehicle dynamics."""
    # unpack variables
    v = state["v"]
    d = state["d"]
    mu = state["mu"]

    dt = params["dt"]

    # update velocity and distance
    v_next = max(0, v - a_safe * dt)    # - due to deceleration
    d_next = d - v * dt                 # - due to deceleration

    # store velocity and distance in a dictionary
    next_state = {
        "v": v_next,
        "d": d_next,
        "mu": mu,
    }
    
    # return updated state values
    return next_state

# 3. Policy
def policy(state, params):
    v = state["v"]
    d = state["d"]
    a_max = params["a_max"]

    if d <= 0:
        return 1.0
    u_raw = 0.5
    return u_raw

# 4. Safety Layer
def safety_layer(state, params, u_raw):
    a_max = params["a_max"]
    g = params["g"]
    mu = state["mu"]

    a_raw = u_raw * a_max
    # friction limit
    a_phys_max = mu * g
    a_safe = min(a_raw, a_phys_max)
    
    return a_safe

def closed_loop_sim(initial_state, params, max_steps = 500):
    
    state = initial_state.copy()

    for step in range(max_steps):
        
        # current state
        v = state["v"]
        d = state["d"]
        print("Step No:", step, "Velocity:", v, "Distance:", d)

        # crash check
        if v <= 0.001:
            print("Stopped")
            break
        if d <= 0:
            print("CRASH!")
            break
        # policy and safety update
        u_raw = policy()
        a_safe = safety_layer(state, params, u_raw)
        next_state = step_function(state, params, a_safe)
        
        state = next_state

if __name__ == "__main__":
    closed_loop_sim(initial_state, params)