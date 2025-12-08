# ------------------------
# SAFE BRAKE RL CONTROLLER
# ------------------------
import random

d_safe = 0.05              # distance where the vehicle should stop [m]

# km/h to m/s
unit_convert = 3.6

# road conditions
road_condition = {
    "dry": 1.0,
    "wet": 0.5, 
    "ice": 0.1,
}

# initial state paramters
initial_state = {
    "v": 80 / unit_convert, # velocity [km/h]
    "d": 400,               # distance [m]
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
    d_eff = d - d_safe
    
    if d_eff <= 0:
        return 1.0

    a_required = v**2 / (2 * d_eff)

    u_raw = a_required / a_max

    u_raw = max(0.0, min(1.0, u_raw))

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

# 5. Can stop in time safety check
def can_stop_in_time(mu):
    v = initial_state["v"]
    d = initial_state["d"]
    g = params["g"]
    d_stop = v**2 / (2*mu*g)

    if d_stop > d:
        return False
    else:
        return True


def closed_loop_sim(initial_state, params, road_condition, max_steps = 1000):

    state = initial_state.copy()
    condition, mu = random.choice(list(road_condition.items()))
    state["mu"] = mu

    if not can_stop_in_time(mu):
        print("Physically impossible to brake")
        return 0

    for step in range(max_steps):
        
        # current state
        v = state["v"]
        d = state["d"]
        mu = state["mu"]
        print("Step No:", step, "Velocity:", v, "Distance:", d, "Road Condition:", condition)

        # crash check
        if v <= 0.001:
            print("Stopped")
            break
        if d <= 0:
            print("CRASH!")
            break
        # policy and safety update
        u_raw = policy(state, params)
        a_safe = safety_layer(state, params, u_raw)
        next_state = step_function(state, params, a_safe)
        
        state = next_state

if __name__ == "__main__":
    closed_loop_sim(initial_state, params, road_condition)