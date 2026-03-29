# save as solve_opt_delay.py
from math import gcd, ceil
from functools import reduce
import itertools
import numpy as np
import time
from gurobipy import Model, GRB, quicksum

def lcm(a,b): return a*b//gcd(a,b)
def lcm_list(lst): return reduce(lcm, lst, 1)

def compute_wcrt(fp_task, hp_tasks, C_self, D_self, max_iter=1000):
    """
    Fixed-priority WCRT computation (Liu & Layland style).
    Returns Ri if converges <= D_self; otherwise returns large value (unschedulable).
    hp_tasks: list of (Cj, Tj) for higher-priority tasks
    C_self: own WCET
    """
    R = C_self
    for _ in range(max_iter):
        interference = 0
        for (Cj, Tj) in hp_tasks:
            interference += ceil(R / Tj) * Cj
        R_next = C_self + interference
        if R_next == R:
            return R
        if R_next > D_self:
            return R_next  # unschedulable mark
        R = R_next
    return R

# ---------------------------
tasks = [
    ("tau1",10, 1, 10,  8, 3, "control"),   # TTC
    ("tau2",5, 1, 5,  7, 12,  "control"),   # ESP
    ("tau3",10, 1, 10, 9, 8,  "control"),   # CC
    ("tau4",100, 5,100,  None, None, "nonctrl"),
    ("tau5",20, 1 ,20,  None, None, "nonctrl"),
    ("tau6",100,  1, 100,   None, None, "nonctrl"),
    ("tau7",1000, 3,1000,  None, None, "nonctrl"),
    ("tau8",10, 1 ,10,  None, None, "nonctrl"),
    ("tau9",50, 10,50,   None, None, "nonctrl"),
    ("tau10",5000,1,5000,  None, None, "nonctrl"),
    ("tau11",100, 5,100,  None, None, "nonctrl"),
    ("tau12",20, 6 ,20,  None, None, "nonctrl"),
    ("tau13",500,  1, 500,   None, None, "nonctrl"),
    ("tau14",10000, 30,10000,  None, None, "nonctrl"),
    ("tau15",100, 5 ,100,  None, None, "nonctrl"),
    ("tau16",100, 10,100,   None, None, "nonctrl"),
    ("tau17",100,1,100,  None, None, "nonctrl"),
    ("tau18",20, 1,20,  None, None, "nonctrl"),
    ("tau19",50, 1 ,50,  None, None, "nonctrl"),
    ("tau20",50, 1 ,50,  None, None, "nonctrl")
]

# Partition sets
control_tasks = [t for t in tasks if t[6]=="control"]
untrusted_tasks = [t for t in tasks if t[6]!="control"]  # we treat these as untrusted in overlap calc


periods = [t[1] for t in tasks]
H = lcm_list(periods)
print("Hyperperiod H =", H, "ms")

# Build release lists for each task across hyperperiod
releases = {}  # task_name -> list of release times (ms)
for (name,T,C,D,Omega,Delta_peak,typ) in tasks:
    N = H // T
    releases[name] = [k*T for k in range(N)]

# Precompute higher-priority sets for WCRT (for each task, hp are tasks with smaller period -> higher priority)
# Rate Monotonic priorities: smaller period = higher priority
sorted_by_period = sorted(tasks, key=lambda x: x[1])  # ascending period => higher priority
priority_order = {t[0]: idx for idx,t in enumerate(sorted_by_period)}  # lower idx => higher priority

WCRT_upper = {}
for (name,T,C,D,Omega,Delta_peak,typ) in tasks:
    hp = []
    for (hn, hT, hC, hD, hOmega, hDelta_peak, htyp) in tasks:
        if priority_order[hn] < priority_order[name]:
            hp.append((hC, hT))
    R = compute_wcrt(name, hp, C, D)
    WCRT_upper[name] = R

print("Conservative WCRT (no delay-specific analysis) per task:")
for k,v in WCRT_upper.items():
    print(" ",k,":",v,"ms")

# ---------------------------
# MILP formulation for a chosen victim
# ---------------------------
def solve_for_victim(victim_name):
    start=time.time()
    # victim params
    v_task = next(t for t in tasks if t[0]==victim_name)
    _, Tv, Cv, Dv, Omega_v, Delta_peak_v, _ = v_task
    if Delta_peak_v is None:
        raise ValueError("No Delta_peak provided for victim")
    Ni = H // Tv
    # all victim release times r_{v,k}
    r_v = releases[victim_name]

    # prepare list of untrusted job instances (we will include tasks tau4,tau5,tau6)
    untrusted = [t for t in tasks if t[6]!="control"]  # list of tuples
    untrusted_jobs = []
    for (un_name, Tj, Cj, Dj, Oj, Dpk, typ) in untrusted:
        for m, rjm in enumerate(releases[un_name]):
            untrusted_jobs.append( (un_name, m, rjm, WCRT_upper[un_name]) )

    # precompute R_i_max and R_j_max: we use WCRT_upper as conservative approximation
    R_i_max = WCRT_upper[victim_name]
    # Build MILP
    M_big = H*10 + 1000  # sufficiently large big-M (ms)
    model = Model("cdsched_delay_opt")
    model.setParam('OutputFlag', 1)

    # Variables:
    # delta_k for k in 0..Ni-1
    delta = {}
    for k in range(Ni):
        delta[k] = model.addVar(lb=0.0, ub=Delta_peak_v, vtype=GRB.CONTINUOUS, name=f"delta_{k}")

    # For each (k, untrusted_job) create auxiliary variables for overlap computation
    z = {}  # overlap amount
    s_max = {}  # max(start_v, start_u)
    e_min = {}  # min(end_v, end_u)
    b_max = {}  # binary: 1 if start_v >= start_u
    b_min = {}  # binary: 1 if end_v <= end_u
    
    for k in range(Ni):
        for (un_name, m, rjm, Rj_max) in untrusted_jobs:
            key = (k, un_name, m)
            z[key] = model.addVar(lb=0.0, ub=M_big, vtype=GRB.CONTINUOUS, name=f"z_{k}_{un_name}_{m}")
            s_max[key] = model.addVar(lb=0.0, ub=M_big, vtype=GRB.CONTINUOUS, name=f"s_max_{k}_{un_name}_{m}")
            e_min[key] = model.addVar(lb=0.0, ub=M_big, vtype=GRB.CONTINUOUS, name=f"e_min_{k}_{un_name}_{m}")
            b_max[key] = model.addVar(vtype=GRB.BINARY, name=f"b_max_{k}_{un_name}_{m}")
            b_min[key] = model.addVar(vtype=GRB.BINARY, name=f"b_min_{k}_{un_name}_{m}")

    model.update()

    # Constraints: Compute overlap = max(0, min(end_v, end_u) - max(start_v, start_u))
    for k in range(Ni):
        for (un_name, m, rjm, Rj_max) in untrusted_jobs:
            key = (k, un_name, m)
            
            # Compute interval endpoints
            # Victim's AEW (Attack Effective Window): Omega window after execution completes
            start_v_expr = r_v[k] + delta[k] + R_i_max  # AEW starts when task completes
            end_v_expr = r_v[k] + delta[k] + R_i_max + (Omega_v if Omega_v is not None else 0)
            start_u = rjm
            end_u = rjm + Rj_max
            
            # s_max = max(start_v, start_u) using big-M and binary b_max
            # If b_max=1: start_v >= start_u, so s_max = start_v
            # If b_max=0: start_v < start_u, so s_max = start_u
            model.addConstr(s_max[key] >= start_v_expr)
            model.addConstr(s_max[key] >= start_u)
            model.addConstr(s_max[key] <= start_v_expr + M_big * (1 - b_max[key]))
            model.addConstr(s_max[key] <= start_u + M_big * b_max[key])
            model.addConstr(start_v_expr >= start_u - M_big * (1 - b_max[key]))
            
            # e_min = min(end_v, end_u) using big-M and binary b_min
            # If b_min=1: end_v <= end_u, so e_min = end_v
            # If b_min=0: end_v > end_u, so e_min = end_u
            model.addConstr(e_min[key] <= end_v_expr)
            model.addConstr(e_min[key] <= end_u)
            model.addConstr(e_min[key] >= end_v_expr - M_big * (1 - b_min[key]))
            model.addConstr(e_min[key] >= end_u - M_big * b_min[key])
            model.addConstr(end_v_expr <= end_u + M_big * (1 - b_min[key]))
            
            # z = max(0, e_min - s_max)
            model.addConstr(z[key] >= 0)
            model.addConstr(z[key] >= e_min[key] - s_max[key])

    # Objective: minimize total overlap
    model.setObjective(quicksum(z.values()), GRB.MINIMIZE)
    
    # Update model to finalize all constraints
    model.update()

    # Print model statistics
    num_vars = model.NumVars
    num_constrs = model.NumConstrs
    num_bins = model.NumBinVars
    num_continuous = num_vars - num_bins
    num_overlap_pairs = len(z)
    num_constrs_per_pair = 12  # 5 for s_max, 5 for e_min, 2 for z
    
    print(f"\nModel statistics for {victim_name}:")
    print(f"  Total variables: {num_vars} ({num_continuous} continuous + {num_bins} binary)")
    print(f"  Total constraints: {num_constrs}")
    print(f"  Decision variables (delta): {Ni}")
    print(f"  Overlap pairs (victim_job × untrusted_job): {num_overlap_pairs}")
    print(f"  Constraints per overlap pair: {num_constrs_per_pair}")
    print(f"  Expected total constraints: {num_overlap_pairs * num_constrs_per_pair}")

    # solve
    model.optimize()

    if model.status != GRB.OPTIMAL:
        print("Warning: model not optimal, status:", model.status)
        if model.status == GRB.INFEASIBLE:
            print("Model is infeasible. Computing IIS...")
            model.computeIIS()
            model.write("model.ilp")
            print("IIS written to model.ilp")
            return {
                "victim": victim_name,
                "Ni": Ni,
                "delta_sol": None,
                "overlap_opt": None,
                "overlap_base": None,
                "reduction_pct": None,
                "status": "INFEASIBLE"
            }

    end=time.time()
    print(f"Optimization time for {victim_name}: {end-start:.2f} seconds")

    # collect solution
    delta_sol = [delta[k].X for k in range(Ni)]
    total_overlap_opt = sum(z[key].X for key in z)
    
    # Verify the solution manually
    print(f"\nVerifying optimized solution for {victim_name}:")
    total_overlap_verify = 0.0
    for k in range(Ni):
        for (un_name, m, rjm, Rj_max) in untrusted_jobs:
            key = (k, un_name, m)
            start_v_opt = r_v[k] + delta[k].X + R_i_max
            end_v_opt = r_v[k] + delta[k].X + R_i_max + (Omega_v if Omega_v is not None else 0)
            start_u = rjm
            end_u = rjm + Rj_max
            overlap_verify = max(0.0, min(end_v_opt, end_u) - max(start_v_opt, start_u))
            total_overlap_verify += overlap_verify
            # Check what the solver thinks
            z_val = z[key].X
            if k == 0 and overlap_verify > 0.1:
                print(f"  Job {k} vs {un_name}_{m}: z={z_val:.2f}, actual_overlap={overlap_verify:.2f}")
    
    print(f"  Total overlap (verified): {total_overlap_verify:.2f} ms")
    print(f"  Total overlap (solver):   {total_overlap_opt:.2f} ms")
    
    # Print details for debugging
    print(f"\nDetailed solution for {victim_name}:")
    print(f"  WCRT (R_i_max) = {R_i_max} ms")
    print(f"  Omega_v (AEW duration) = {Omega_v} ms")
    print(f"  AEW starts at: release + delay + R_i_max")
    
    # compute baseline overlap with delta=0
    total_overlap_baseline = 0.0
    print(f"\nBaseline (delta=0) AEW overlap calculation:")
    for k in range(Ni):
        for (un_name, m, rjm, Rj_max) in untrusted_jobs:
            start_v0 = r_v[k] + 0.0 + R_i_max  # AEW starts after execution completes
            end_v0 = r_v[k] + 0.0 + R_i_max + (Omega_v if Omega_v is not None else 0)
            start_u = rjm
            end_u = rjm + Rj_max
            overlap = max(0.0, min(end_v0, end_u) - max(start_v0, start_u))
            if overlap > 0 and k < 2:  # Print first 2 victim jobs with overlap
                print(f"  Job {k}: victim AEW [{start_v0:.1f}, {end_v0:.1f}] vs {un_name}_{m} [{start_u:.1f}, {end_u:.1f}] → overlap = {overlap:.1f} ms")
            total_overlap_baseline += overlap

    return {
        "victim": victim_name,
        "Ni": Ni,
        "delta_sol": delta_sol,
        "overlap_opt": total_overlap_opt,
        "overlap_base": total_overlap_baseline,
        "reduction_pct": 100.0*(total_overlap_baseline - total_overlap_opt)/total_overlap_baseline if total_overlap_baseline>0 else 0.0
    }

# ---------------------------
# Run MILP for each control task
# ---------------------------
for (name,_,_,_,_,_,typ) in tasks:
    if typ!="control": continue
    # Focus on tau3 as mentioned by user
    if name != "tau3": continue
    print("\nSolving for victim:", name)
    res = solve_for_victim(name)
    print("Victim:", res["victim"])
    print("Number of victim jobs Ni:", res["Ni"])
    if res.get("status") == "INFEASIBLE":
        print("Solution: INFEASIBLE - cannot find valid delay schedule")
        continue
    print("Optimal delta_k sequence (ms):", ["{:.3f}".format(x) for x in res["delta_sol"]])
    print("Baseline total overlap (ms): {:.6f}".format(res["overlap_base"]))
    print("Optimal total overlap  (ms): {:.6f}".format(res["overlap_opt"]))
    print("Overlap reduction (%)   : {:.2f}%".format(res["reduction_pct"]))
