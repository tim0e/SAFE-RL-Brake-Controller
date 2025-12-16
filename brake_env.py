# ------------------------
# The environment in which the braking will happen
# ------------------------
import random
#from controllers import baseline_controller


#random.seed(1)

#-------------------------
# CONSTANTS
# ------------------------
d_safe = 1                 # distance where the vehicle should stop [m]
unit_convert = 3.6         # km/h to m/s
road_segments = 5           # Amount of road segments n

# Road conditions mu
road_condition = {
    "dry": 1.0,
    "slimy": 0.8,
    "muddy": 0.6,
    "wet": 0.3,  
    "ice": 0.1,
}


# ----------------------
# Generation of the Road
# ----------------------
def sample_random_road_profile(road_condition, segment_bounds):
    road_profile = []
    for (start, finish) in segment_bounds:
        road_state = random.choice(list(road_condition.keys()))
        mu = road_condition[road_state]
        road_profile.append((start, finish, mu))
    return road_profile


# ----------------------
# Updating the road profile according to segment bounds
# ----------------------
def update_road_profile(d, road_profile):
    for (d_start, d_end, mu) in road_profile:
        if d_end <= d <= d_start:
            return mu
    return road_condition["dry"]

# ----------------------
# Generation of the initial state
# ----------------------
def sample_initial_state(v_range, d_range):
    low_v, high_v = v_range
    low_d, high_d = d_range
    v0 = random.uniform(low_v, high_v)
    d0 = random.uniform(low_d, high_d)
    state = {
        "v": v0 / unit_convert, # velocity [km/h]
        "d": d0 ,               # distance [m]
        "mu": 1.0, 
    }
    return state

# ----------------------
# Generation of segment bounds
# ----------------------
def make_segment_bounds(d0, n_segments):
    seg_len = d0 / n_segments
    bounds = []
    for i in range(n_segments):
        d_start = d0 - i * seg_len
        d_end = d0 - (i + 1) * seg_len
        bounds.append((d_start, d_end))
    bounds[-1] = (bounds[-1][0], 0.0)
    return bounds


# ----------------------
# Dynamics
# ----------------------
def step_function(state, params, a_safe):
    """ One Step Simulation of the vehicle dynamics."""
    # unpack variables
    v = state["v"]
    d = state["d"]
    mu = state["mu"]

    dt = params["dt"]

    # update velocity and distance
    v_next = max(0, v - a_safe * dt)                     # - due to deceleration
    d_next = d - 0.5 * (v + v_next) * dt                 # distance to wall gets smaller

    # store velocity and distance in a dictionary
    next_state = {
        "v": v_next,
        "d": d_next,
        "mu": mu,
    }
    
    # return updated state values
    return next_state


# ----------------------
# Safety Layer
# ----------------------
def safety_layer(state, params, u_raw):
    a_max = params["a_max"]
    g = params["g"]
    mu = state["mu"]

    a_raw = u_raw * a_max
    # friction limit
    a_phys_max = mu * g
    a_safe = min(a_raw, a_phys_max)
    
    return a_safe


# ----------------------
# SIMULATION
# ----------------------
def sim(controller, params, seed, max_steps = 10000):
    if seed is not None:
        random.seed(seed)

    state = sample_initial_state((120, 140), (500, 800))
    segment_bounds = make_segment_bounds(state["d"], road_segments)
    road_profile = sample_random_road_profile(road_condition, segment_bounds)
    v0 = state["v"]
    d0 = state["d"]

    trajectory = {
        "t": [],
        "v": [],
        "d": [],
        "mu": [],
        "u_raw": [],
        "a_safe": [],
    }
    
    for step in range(max_steps):
        
        # current state
        v = state["v"]
        d = state["d"]
        state["mu"] = update_road_profile(d, road_profile)
        mu = state["mu"]

        # crash check
        if d <= 0:
            return {
                "outcome": "crash",
                "trajectory": trajectory,
                "t_final": step * params["dt"],
                "d_final": d,
                "v_final": v,
                "dist_error": abs(d_safe - d),
                "v0": v0,
                "d0": d0,
            }
        if v <= 0.001:
            return {
                "outcome": "stopped",
                "trajectory": trajectory,
                "t_final": step * params["dt"],
                "d_final": d,
                "v_final": v,
                "dist_error": abs(d_safe - d),
                "v0": v0,
                "d0": d0,
            }

        # policy and safety update
        observable_state = {
            "v": state["v"],
            "d": state["d"],
        }
        u_raw = controller(observable_state, params, d_safe)
        a_safe = safety_layer(state, params, u_raw)
        next_state = step_function(state, params, a_safe)

        trajectory["t"].append(step * params["dt"])
        trajectory["v"].append(v)
        trajectory["d"].append(d)
        trajectory["mu"].append(state["mu"])
        trajectory["u_raw"].append(u_raw)
        trajectory["a_safe"].append(a_safe)
        
        state = next_state