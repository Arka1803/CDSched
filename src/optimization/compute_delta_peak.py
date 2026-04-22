# ============================================================
# Computes Δ_i^peak for control tasks using schedulability
# analysis as described in the CDSched paper (Eq. 9, 13, 15).
# ============================================================

from math import ceil, floor, gcd
from functools import reduce

def lcm(a, b): 
    return a * b // gcd(a, b)

def lcm_list(lst): 
    return reduce(lcm, lst, 1)

# ---------- Carry-in Interference (Eq. 12 from paper) ----------
def carry_in_I(r_v_k_prime, hp_tasks, debug=False):
    """
    Compute carry-in interference for victim job k.
    Eq. (12): I(k,i) = max(0, ⌈r'_{v,k}/T_i⌉ - ⌊(r'_{v,k}-C_i)/T_i⌋ - 1) * C_i
    Sum over all τ_i ∈ hp(τ_v) to get I(k) per text after Eq. 12.
    hp_tasks: list of tuples (Cj, Tj)
    """
    total = 0
    if debug:
        print(f"    Computing carry-in interference I(k) for r_v_k_prime={r_v_k_prime}:")
    for (Cj, Tj) in hp_tasks:
        # Each I(k,i): max(0, ceil(r'/T) - floor((r'-C)/T) - 1) * C
        term1 = ceil(r_v_k_prime / Tj)
        term2 = floor((r_v_k_prime - Cj) / Tj)
        jobs_count = max(0, term1 - term2 - 1)
        contrib = jobs_count * Cj
        if debug:
            print(f"      task(C={Cj},T={Tj}): ceil({r_v_k_prime}/{Tj}) - floor(({r_v_k_prime}-{Cj})/{Tj}) - 1 = {term1} - {term2} - 1 = {jobs_count}, I(k,i)={contrib}")
        total += contrib
    if debug:
        print(f"    Total I(k) = {total}")
    return total

# ---------- Victim task WCRT (Eq. 13) ----------
def compute_Rv_k(Cv, Dv_k, r_v_k_prime, hp_tasks, debug=False):
    """
    Compute response time R_{v,k} with carry-in interference
    using fixed-point iteration.
    """
    I_k = carry_in_I(r_v_k_prime, hp_tasks, debug=debug)
    if debug:
        print(f"    r_v_k_prime = {r_v_k_prime}, Cv = {Cv}, Dv_k = {Dv_k}")
    R = Cv
    max_iter = 1000
    for iteration in range(max_iter):
        interference = 0
        details = []
        for (Cj, Tj) in hp_tasks:
            count = ceil(R / Tj)
            contrib = count * Cj
            interference += contrib
            details.append(f"ceil({R}/{Tj})*{Cj} = {count}*{Cj} = {contrib}")
        R_next = Cv + I_k + interference
        if debug:
            print(f"    Iteration {iteration}: R={R}, interference={interference}")
            for d in details:
                print(f"      {d}")
            print(f"      R_next = {Cv} + {I_k} + {interference} = {R_next}")
        if R_next == R:
            if debug:
                print(f"    Converged: R = {R_next}")
            return R_next
        if R_next > Dv_k:
            if debug:
                print(f"    Exceeded deadline: {R_next} > {Dv_k}")
            return R_next
        R = R_next
    return R

# Lower-priority task WCRT
def compute_lower_task_wcrt(task, tasks, victim, delta):
    """
    Compute WCRT for lower-priority task τ_i under delay δ
    following Eq. (15).
    """
    name, T, C, D, _, _, _ = task
    vname, Tv, Cv, Dv, Ov, _, _ = victim

    # higher priority tasks
    hp_tasks = [(Cj, Tj) for (n, Tj, Cj, Dj, Oj, Dpk, typ) in tasks if Tj < T and n != vname]

    R = C
    max_iter = 1000
    for _ in range(max_iter):
        interference = 0
        for (Cj, Tj) in hp_tasks:
            interference += ceil(R / Tj) * Cj
        # Victim contribution (Eq. 15, using +δ)
        victim_terms = ceil((R + delta) / Tv) * Cv
        R_next = C + interference + victim_terms
        if R_next == R:
            return R_next
        if R_next > D:
            return R_next
        R = R_next
    return R

#peak computation 
def compute_delta_peak_for_taskset(tasks, step_ms=1):
    """
    tasks: list of tuples (name, T, C, D, Omega, Delta_peak_placeholder, type)
    step_ms: resolution of δ sweep in ms
    returns: dict{name: Δ^peak}
    """
    periods = [t[1] for t in tasks]
    H = lcm_list(periods)
    releases = {t[0]: [k * t[1] for k in range(H // t[1])] for t in tasks}

    results = {}
    for (vname, Tv, Cv, Dv, Omega_v, _, vtype) in tasks:
        if vtype != "control":
            continue

        # hp_tasks: all control tasks with higher priority than victim
        # Assuming priority order: tau1 > tau2 > tau3 (by task index, not period)
        victim_idx = next(i for i, t in enumerate(tasks) if t[0] == vname)
        hp_tasks = [(tasks[i][2], tasks[i][1]) for i in range(victim_idx) if tasks[i][6] == "control"]
        lp_tasks = [tasks[i] for i in range(victim_idx + 1, len(tasks)) if tasks[i][6] == "control"]

        max_delta = Tv - Cv
        feasible_delta = -1.0

        d = 0.0
        while d <= max_delta:
            delta = d
            ok = True
            Ni = H // Tv

            # check victim task instances (Eq. 13)
            for k in range(Ni):
                r_vk = releases[vname][k]
                r_v_k_prime = r_vk + delta
                Dv_k = Dv - delta
                debug_this = (vname == "tau3" and delta == 14 and k == 0)
                if debug_this:
                    print(f"\n=== Detailed WCRT computation for {vname}, delta={delta}, job k={k} ===")
                    print(f"  r_vk = {r_vk}, delta = {delta}, r_v_k_prime = {r_v_k_prime}")
                    print(f"  Dv = {Dv}, Dv_k = Dv - delta = {Dv_k}")
                    print(f"  Higher-priority tasks: {hp_tasks}")
                Rvk = compute_Rv_k(Cv, Dv_k, r_v_k_prime, hp_tasks, debug=debug_this)
                if vname == "tau3" and delta >= 13 and delta <= 15:  # debug tau3
                    print(f"  DEBUG {vname}: delta={delta}, job k={k}, r_vk={r_vk}, Dv_k={Dv_k}, Rvk={Rvk}, ok={Rvk <= Dv_k}")
                if Rvk > Dv_k:
                    ok = False
                    break

            # check lower-priority tasks (Eq. 15)
            if ok:
                for t in lp_tasks:
                    Ri = compute_lower_task_wcrt(t, tasks, (vname, Tv, Cv, Dv, Omega_v, None, vtype), delta)
                    if Ri > t[3]:
                        ok = False
                        break

            if ok:
                feasible_delta = delta  # still schedulable
            d += step_ms

        results[vname] = feasible_delta if feasible_delta >= 0 else None

    return results

import sys
import os

def load_tasks_from_file(filepath):
    """
    Parses an input file for tasks.
    Expected format per line:
    name, T, C, D, Omega, Delta_peak, type
    """
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
                Delta = int(parts[5]) if parts[5] != 'None' else None
                typ = parts[6]
                tasks.append((name, T, C, D, Omega, Delta, typ))
    return tasks

# Main function
if __name__ == "__main__":
    if len(sys.argv) > 1:
        filepath = sys.argv[1]
    else:
        filepath = os.path.join(os.path.dirname(__file__), "input.txt")

    if not os.path.exists(filepath):
        print(f"Error: Could not find '{filepath}'. Please create one or provide a valid path.")
        sys.exit(1)
        
    tasks = load_tasks_from_file(filepath)

    res = compute_delta_peak_for_taskset(tasks, step_ms=1)
    print("Computed Delta_peak per control task (ms):")
    for k, v in res.items():
        print(f"  {k} -> {v} ms")
