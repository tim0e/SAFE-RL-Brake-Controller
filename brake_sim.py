# ------------------------
# SAFE BRAKE RL CONTROLLER
# ------------------------
import random


random.seed(1)

# ------------------------
d_safe = 1                 # distance where the vehicle should stop [m]
unit_convert = 3.6         # km/h to m/s

# Road conditions mu
road_condition = {
    "muddy": 1.5,
    "dry": 1.0,
    "slimy":0.6,
    "wet": 0.3,  
    "ice": 0.1,
}

# segment bounds values
s0 = 1000
s1 = 800
s2 = 600
s3 = 400
s4 = 200
s5 = 0
segment_bounds = [
    # start, finish
    (s0, s1),
    (s1, s2),
    (s2, s3),
    (s3, s4),
    (s4, s5),
]

# Road profile
# road_profile = [
    # d_start, , d_end, mu
    #(500, 400, road_condition["dry"]),
    #(400, 300, road_condition["wet"]),
    #(300, 200, road_condition["ice"]),
    #(200, 100, road_condition["muddy"]),
    #(100, 0, road_condition["slimy"]),
#]

# Initial state paramters
#initial_state = {
    #"v": 80 / unit_convert, # velocity [km/h]
    #"d": 300,               # distance [m]
    #"mu": 1.0,              # coefficient for road conditions 
#}
# Global Parameters
params = {
    "g": 9.81,          # gravity
    "dt": 0.05,         # time step delta t [s]
    "a_max": 8,         # maximum acceleration [m/s^2]
}
# ----------------------

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
    d_next = d - v * dt                 # distance to wall gets smaller

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
    v = max(0.0, state["v"] + random.gauss(0, 0.1)) # introduced sensor noise
    d = max(0.0, state["d"] + random.gauss(0, 0.5)) # introduced sensor noise
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

def update_road_profile(d, road_profile):
    for (d_start, d_end, mu) in road_profile:
        if d_end <= d <= d_start:
            return mu
    return road_condition["dry"]
def closed_loop_sim(params, max_steps = 10000):

    state = sample_initial_state((80, 120), (800, 1000))
    road_profile = sample_random_road_profile(road_condition, segment_bounds)
    
    for step in range(max_steps):
        
        # current state
        v = state["v"]
        d = state["d"]
        state["mu"] = update_road_profile(d, road_profile)
        mu = state["mu"]

        # crash check
        if d <= 0:
            print("CRASH!")
            break
        if v <= 0.001:
            print("Stopped")
            break
        print("Step No:", step, "Velocity:", v, "Distance:", d, "Road Condition:", mu)

        # policy and safety update
        u_raw = policy(state, params)
        a_safe = safety_layer(state, params, u_raw)
        next_state = step_function(state, params, a_safe)
        
        state = next_state

if __name__ == "__main__":
    closed_loop_sim(params)