import sys
from math import gcd, ceil
from functools import reduce
import itertools
import numpy as np
from gurobipy import Model, GRB, quicksum

def lcm(a,b): return a*b//gcd(a,b)
def lcm_list(lst): return reduce(lcm, lst, 1)

def compute_wcrt(fp_task, hp_tasks, C_self, D_self, max_iter=1000):
    R = C_self
    for _ in range(max_iter):
        interference = 0
        for (Cj, Tj) in hp_tasks:
            interference += ceil(R / Tj) * Cj
        R_next = C_self + interference
        if R_next == R:
            return R
        if R_next > D_self:
            return R_next  
        R = R_next
    return R

def load_tasks_from_file(filepath):
    tasks = []
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = [p.strip() for p in line.split(',')]
            if len(parts) >= 7:
                name = parts[0]
                T = int(parts[1])
                C = int(parts[2])
                D = int(parts[3])
                Omega = int(parts[4]) if parts[4] != 'None' else None
                Delta = float(parts[5]) if parts[5] != 'None' else None
                typ = parts[6]
                tasks.append((name, T, C, D, Omega, Delta, typ))
    return tasks

def setup_globals(loaded_tasks):
    global tasks, periods, H, releases, WCRT_upper, priority_order
    tasks = loaded_tasks
    periods = [t[1] for t in tasks]
    H = lcm_list(periods)

    releases = {}
    for (name,T,C,D,Omega,Delta_peak,typ) in tasks:
        N = H // T
        releases[name] = [k*T for k in range(N)]

    sorted_by_period = sorted(tasks, key=lambda x: x[1])
    priority_order = {t[0]: idx for idx,t in enumerate(sorted_by_period)}

    WCRT_upper = {}
    for (name,T,C,D,Omega,Delta_peak,typ) in tasks:
        hp = []
        for (hn, hT, hC, hD, hOmega, hDelta_peak, htyp) in tasks:
            if priority_order[hn] < priority_order[name]:
                hp.append((hC, hT))
        WCRT_upper[name] = compute_wcrt(name, hp, C, D)

def solve_for_victim_multiple(victim_name, W, q, delta_max_override=None):
    v_task = next((t for t in tasks if t[0]==victim_name), None)
    if not v_task:
        raise ValueError(f"Task {victim_name} not found in tasks array.")
    _, Tv, Cv, Dv, Omega_v, Delta_peak_v, _ = v_task
    
    if delta_max_override is not None:
        Delta_peak_v = delta_max_override
        
    if Delta_peak_v is None:
        raise ValueError("No Delta_peak provided for victim in file or arguments.")
    Ni = H // Tv
    r_v = releases[victim_name]

    if q >= Ni:
        print(f"Warning: q ({q}) is >= number of instances H/Tv ({Ni}). Limiting q to {Ni}.")
        q = min(q, Ni)

    untrusted = [t for t in tasks if t[6]!="control"]
    untrusted_jobs = []
    for (un_name, Tj, Cj, Dj, Oj, Dpk, typ) in untrusted:
        for m, rjm in enumerate(releases[un_name]):
            untrusted_jobs.append( (un_name, m, rjm, WCRT_upper[un_name]) )
            # Add instances in the NEXT hyperperiod to catch overlaps spilling over the boundary H
            untrusted_jobs.append( (un_name, f"{m}_nextH", rjm + H, WCRT_upper[un_name]) )

    R_i_max = WCRT_upper[victim_name]
    M_big = H*10 + 1000

    # 0. Compute the very first initial baseline overlap with no delays
    initial_total_overlap = 0.0
    for k in range(Ni):
        sz = r_v[k] + 0.0 + R_i_max
        ez = sz + (Omega_v if Omega_v is not None else 0)
        for (un_name, m, rjm, Rj_max) in untrusted_jobs:
            su = rjm
            eu = rjm + Rj_max
            initial_total_overlap += max(0.0, min(ez, eu) - max(sz, su))
            
    print(f"\n[{victim_name}] Initial Baseline total overlap (0 delays): {initial_total_overlap:.2f} ms")

    sequences = []
    current_delays = [0.0] * Ni
    
    for seq_idx in range(W):
        print(f"\n--- [{victim_name}] Generating Sequence {seq_idx + 1}/{W} ---")
        
        # 1. Compute baseline overlap to sort and find top q highest overlaps
        # Uses current_delays, initially 0, meaning it acts as standard baseline for sequence 1.
        overlaps_k = []
        for k in range(Ni):
            job_overlap = 0.0
            for (un_name, m, rjm, Rj_max) in untrusted_jobs:
                start_v = r_v[k] + current_delays[k] + R_i_max
                end_v = r_v[k] + current_delays[k] + R_i_max + (Omega_v if Omega_v is not None else 0)
                start_u = rjm
                end_u = rjm + Rj_max
                overlap = max(0.0, min(end_v, end_u) - max(start_v, start_u))
                job_overlap += overlap
            overlaps_k.append((k, job_overlap))
            
        overlaps_sorted = sorted(overlaps_k, key=lambda x: x[1], reverse=True)
        top_q_k = [x[0] for x in overlaps_sorted[:q]]
        
        print(f"Top {q} vulnerable instance indices (Highest overlap): {top_q_k}")
        print(f"Their respective overlaps (ms): {[round(x[1], 2) for x in overlaps_sorted[:q]]}")
        
        # 2. Build and solve MILP for `top_q_k`
        model = Model(f"cdsched_delay_opt_{seq_idx}")
        model.setParam('OutputFlag', 0)
        
        delta = {}
        for k in top_q_k:
            delta[k] = model.addVar(lb=0.0, ub=Delta_peak_v, vtype=GRB.CONTINUOUS, name=f"delta_{k}")
            
        z = {}
        s_max = {}
        e_min = {}
        b_max = {}
        b_min = {}
        
        for k in top_q_k:
            for (un_name, m, rjm, Rj_max) in untrusted_jobs:
                key = (k, un_name, m)
                z[key] = model.addVar(lb=0.0, ub=M_big, vtype=GRB.CONTINUOUS, name=f"z_{k}_{un_name}_{m}")
                s_max[key] = model.addVar(lb=0.0, ub=M_big, vtype=GRB.CONTINUOUS, name=f"s_max_{k}_{un_name}_{m}")
                e_min[key] = model.addVar(lb=0.0, ub=M_big, vtype=GRB.CONTINUOUS, name=f"e_min_{k}_{un_name}_{m}")
                b_max[key] = model.addVar(vtype=GRB.BINARY, name=f"b_max_{k}_{un_name}_{m}")
                b_min[key] = model.addVar(vtype=GRB.BINARY, name=f"b_min_{k}_{un_name}_{m}")
                
        model.update()
        
        for k in top_q_k:
            for (un_name, m, rjm, Rj_max) in untrusted_jobs:
                key = (k, un_name, m)
                
                # Victim uses NEW delta from 0
                start_v_expr = r_v[k] + delta[k] + R_i_max
                end_v_expr = r_v[k] + delta[k] + R_i_max + (Omega_v if Omega_v is not None else 0)
                start_u = rjm
                end_u = rjm + Rj_max
                
                model.addConstr(s_max[key] >= start_v_expr)
                model.addConstr(s_max[key] >= start_u)
                model.addConstr(s_max[key] <= start_v_expr + M_big * (1 - b_max[key]))
                model.addConstr(s_max[key] <= start_u + M_big * b_max[key])
                model.addConstr(start_v_expr >= start_u - M_big * (1 - b_max[key]))
                
                model.addConstr(e_min[key] <= end_v_expr)
                model.addConstr(e_min[key] <= end_u)
                model.addConstr(e_min[key] >= end_v_expr - M_big * (1 - b_min[key]))
                model.addConstr(e_min[key] >= end_u - M_big * b_min[key])
                model.addConstr(end_v_expr <= end_u + M_big * (1 - b_min[key]))
                
                model.addConstr(z[key] >= 0)
                model.addConstr(z[key] >= e_min[key] - s_max[key])
                
        # Objective: minimize overlap
        model.setObjective(quicksum(z.values()), GRB.MINIMIZE)
        
        print("Optimizing...")
        model.optimize()
        
        if model.status != GRB.OPTIMAL:
            print("Warning: model not optimal, status:", model.status)
            if model.status == GRB.INFEASIBLE:
                print("Model is INFEASIBLE. Breaking sequence generation.")
            break
            
        # 3. Gather new absolute delays by padding remaining with 0
        new_delays = [0.0] * Ni
        for k in top_q_k:
            new_delays[k] = delta[k].X
            
        # Applying the delays to current iteration evaluater, so the next overlap calculation gets affected
        current_delays = new_delays.copy()
        
        # Verify and assess the new sequence total overlap with ALL jobs
        total_overlap = 0.0
        for k in range(Ni):
            sz = r_v[k] + current_delays[k] + R_i_max
            ez = sz + (Omega_v if Omega_v is not None else 0)
            for (un_name, m, rjm, Rj_max) in untrusted_jobs:
                su = rjm
                eu = rjm + Rj_max
                total_overlap += max(0.0, min(ez, eu) - max(sz, su))
                
        decrease_pct = 0.0
        if initial_total_overlap > 0:
            decrease_pct = 100.0 * (initial_total_overlap - total_overlap) / initial_total_overlap
                
        print(f"Total overlap over ALL instances after Sequence {seq_idx + 1}: {total_overlap:.2f} ms (Decrease: {decrease_pct:.2f}%)")
        print(f"Delay sequence: {['{:.3f}'.format(x) for x in current_delays]}")
        sequences.append((current_delays.copy(), decrease_pct, total_overlap))
        
    return sequences

if __name__ == "__main__":
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description="Multiple Sequence Optimization")
    parser.add_argument('-f', '--file', type=str, default="input.txt", help="Path to input tasks file")
    parser.add_argument('-v', '--victim', type=str, required=True, help="Victim task ID (e.g., tau1)")
    parser.add_argument('-q', type=int, required=True, help="Number of vulnerable instances to optimize")
    parser.add_argument('-W', type=int, required=True, help="Number of optimal sequences to generate")
    parser.add_argument('-d', '--delta_max', type=float, default=None, help="Override Delta_max value (otherwise reads 6th col from file)")
    
    args = parser.parse_args()
    
    filepath = args.file
    if not os.path.exists(filepath):
        # Fallback to appending script directory
        filepath = os.path.join(os.path.dirname(__file__), args.file)
        if not os.path.exists(filepath):
            print(f"Error: Could not find '{args.file}'.")
            sys.exit(1)
            
    loaded_tasks = load_tasks_from_file(filepath)
    setup_globals(loaded_tasks)
            
    print(f"\nRunning Multiple Sequence Optimization for {args.victim} with W={args.W}, q={args.q}")
    if args.delta_max is not None:
        print(f"Using explicitly provided Delta_max: {args.delta_max}")
        
    seqs = solve_for_victim_multiple(args.victim, args.W, args.q, delta_max_override=args.delta_max)
    
    print(f"\n===========================================")
    print(f"Final W={len(seqs)} Sequences for {args.victim}:")
    for i, (seq, dec_pct, tot_ov) in enumerate(seqs):
        print(f"Seq {i+1} [Overlap: {tot_ov:.2f} ms, Decrease: {dec_pct:.2f}%]: {['{:.3f}'.format(x) for x in seq]}")
    print(f"===========================================\n")
