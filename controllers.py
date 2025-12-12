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