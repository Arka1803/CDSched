import math
import random
import numpy as np

# ---- Utility functions ----

def uunifast_distribute(n, U):
    utilizations = []
    sumU = U
    for i in range(1, n):
        nextU = sumU * (random.random() ** (1.0/(n - i)))
        utilizations.append(sumU - nextU)
        sumU = nextU
    utilizations.append(sumU)
    return np.array(utilizations)

def rm_rta(tasks):
    """Standard RM response-time analysis"""
    tasks_sorted = sorted(tasks, key=lambda x: x['T'])
    results = []
    schedulable = True
    for i, ti in enumerate(tasks_sorted):
        Ci = ti['C']
        Di = ti['D']
        hp = tasks_sorted[:i]
        R_prev = 0
        R = Ci
        while True:
            interference = sum(math.ceil(R / hj['T']) * hj['C'] for hj in hp)
            R_next = Ci + interference
            if abs(R_next - R_prev) < 1e-9:
                break
            if R_next > 1e5:  # runaway
                break
            R_prev = R_next
            R = R_next
        results.append({'T': ti['T'], 'C': ti['C'], 'R': R, 'D': Di})
        if R > Di:
            schedulable = False
    return schedulable, results

# ---- Generate and test ----

def generate_taskset(U_total, n):
    """Generate tasks with given total utilization"""
    u_list = uunifast_distribute(n, U_total)
    periods = np.random.choice([5, 10, 20, 50, 100, 200, 1000], size=n)
    tasks = []
    for i in range(n):
        Ci = u_list[i] * periods[i]
        tasks.append({'T': periods[i], 'C': Ci, 'D': periods[i]})
    return tasks

# ---- Example run ----

random.seed(0)
np.random.seed(0)

U_total = 0.9   # pick a high utilization to stress test
n_tasks = 6

tasks = generate_taskset(U_total, n_tasks)

print("Generated Task Set (unsorted):")
for i, t in enumerate(tasks):
    print(f" τ{i+1}: T={t['T']:4}, C={t['C']:.3f}, U={t['C']/t['T']:.3f}")

sched, results = rm_rta(tasks)

print("\n--- RM Response-Time Analysis ---")
for i, r in enumerate(sorted(results, key=lambda x: x['T'])):
    print(f"Task {i+1}: T={r['T']:4}, C={r['C']:.3f}, R={r['R']:.3f}, D={r['D']}  {'OK' if r['R']<=r['D'] else 'MISS'}")

print(f"\nSchedulable under RM?  →  {sched}")
