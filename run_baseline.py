import random 
import matplotlib.pyplot as plt
import json

from brake_env import sim
from controllers import baseline_controller


params = {
    "g": 9.81,          # gravity
    "dt": 0.02,         # time step delta t [s]
    "a_max": 8,         # maximum acceleration [m/s^2]
}

def build_baseline_dataset(n_episodes, base_seed=1):
    results = []
    for ep in range(n_episodes):
        seed = base_seed + ep
        res = sim(baseline_controller, params)
        results.append(res)
        res["episode"] = ep
        res["seed"] = seed
    return results

if __name__ == "__main__":
    data = build_baseline_dataset(1000)

    crashes = sum(1 for r in data if r["outcome"] == "crash")
    stops = sum(1 for r in data if r["outcome"] == "stopped")
    errors = [r["dist_error"] for r in data if r["outcome"] == "stopped"]

    print("Episodes:", len(data))
    print("Crashes:", crashes)
    print("Stopped:", stops)
    if errors:
        print("Mean stop error:", sum(errors) / len(errors))
