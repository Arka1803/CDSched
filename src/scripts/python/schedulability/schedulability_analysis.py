# cdsched_schedulability.py
# Generates task-sets and evaluates RM schedulability and CDSched (single-delta) feasibility
# Implements Eq.12-16 style checks from the paper (carry-in, victim RTA, lower-priority RTA).
# Dependencies: numpy, matplotlib, tqdm
import math
import random
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm

# ---------------------------
# Parameters (tweakable)
# ---------------------------
np.random.seed(0)
random.seed(0)

period_choices = np.array([5,10,20,50,100,200,1000], dtype=int)  # ms
n_tasks = 10                   # choose n = 5, 11, 15 for different plots (paper uses 5,11,15)
num_sets_per_group = 100
util_groups = [(0.01+0.1*i, 0.09+0.1*i) for i in range(10)]  # as in paper (first 5 displayed); if you want 10 groups extend i in range(10)
# NOTE: in paper they wrote ten groups: i={0..9}. adjust above range to 10 if you want ten groups.
# For full 10 groups:
# util_groups = [(0.02+0.2*i, 0.18+0.2*i) for i in range(10)]

delta_samples = 80   # number of δ values to try between (small positive) and Dv-Cv
eps_delta = 1e-6

# ---------------------------
# Helper functions
# ---------------------------

def uunifast_distribute(n, U):
    """UUniFast algorithm - returns n utilizations that sum to U"""
    utilizations = []
    sumU = U
    for i in range(1, n):
        nextU = sumU * (random.random() ** (1.0/(n - i)))
        utilizations.append(sumU - nextU)
        sumU = nextU
    utilizations.append(sumU)
    return np.array(utilizations)

def lcm(a, b):
    return a * b // math.gcd(a, b)

def hyperperiod(periods):
    H = 1
    for p in periods:
        H = lcm(H, p)
    return H

def generate_taskset_for_util(U_total, n):
    """
    Generate one task-set with total utilization approximately U_total.
    Returns list of tasks: [{'T':, 'C':, 'D':}]
    """
    u_list = uunifast_distribute(n, U_total)
    # assign periods randomly (uniform from choices)
    T = np.random.choice(period_choices, size=n)
    C = np.zeros(n, dtype=int)
    for i in range(n):
        ci = max(1, int(round(u_list[i] * T[i])))  # Use round() instead of floor()
        # ensure Ci < Ti
        if ci >= T[i]:
            ci = T[i] - 1
            if ci < 1:
                ci = 1
        C[i] = ci
    # If sum utilization is 0 due to rounding, adjust randomly
    if np.sum(C / T) <= 0:
        idx = np.random.randint(0, n)
        C[idx] = 1
    tasks = []
    for i in range(n):
        tasks.append({'T': int(T[i]), 'C': int(C[i]), 'D': int(T[i])})  # implicit deadlines
    return tasks

# RTA (standard fixed-priority) for RM ordering (sort by period ascending)
def rm_rta_schedulable(tasks):
    """
    Input: tasks = list of dicts with keys 'T','C','D'
    Returns: (schedulable_bool, R_list) where R_list are computed WCRTs in RM order
    """
    # Sort by period (RM)
    tasks_sorted = sorted(tasks, key=lambda x: x['T'])
    R = []
    for i, ti in enumerate(tasks_sorted):
        Ci = ti['C']; Di = ti['D']
        hp = tasks_sorted[:i]  # higher priority tasks
        Rk = Ci
        while True:
            interference = 0
            for hj in hp:
                interference += math.ceil(Rk / hj['T']) * hj['C']
            Rnext = Ci + interference
            if Rnext == Rk:
                break
            if Rnext > Di:
                Rk = Rnext
                break
            Rk = Rnext
        R.append(Rk)
        if Rk > Di:
            return False, R
    return True, R

# Eq.12 carry-in I(k)
def carry_in_I(rp_vk, hp_tasks):
    """
    rp_vk: delayed release time r'_{v,k} (float)
    hp_tasks: list of dict with 'T','C'
    Implements Eq.12: I(k,i) = max(0, ceil(r'/Ti) - floor((r'-Ci)/Ti) - 1) * Ci summed
    """
    total = 0
    for ti in hp_tasks:
        Ti = ti['T']; Ci = ti['C']
        val = math.ceil(rp_vk / Ti) - math.floor((rp_vk - Ci) / Ti) - 1
        if val < 0:
            val = 0
        total += val * Ci
    return total

# Eq.13 victim Rv,k
def victim_R_wcrt(Cv, hp_tasks, Ik, Dv_minus_delta, iter_limit=200):
    """
    Solve fixed-point R = Cv + I(k) + sum ceil(R/Tj)*Cj
    Return R and converged boolean.
    If R exceeds Dv_minus_delta we can return large value.
    """
    Rk = Cv
    for _ in range(iter_limit):
        interference = 0
        for tj in hp_tasks:
            interference += math.ceil(Rk / tj['T']) * tj['C']
        Rnext = Cv + Ik + interference
        if abs(Rnext - Rk) < 1e-9:
            return Rnext, True
        if Rnext > Dv_minus_delta + 1e-9:
            return Rnext, False
        Rk = Rnext
    return Rk, False

# Eq.15 lower-priority RTA accounting for victim jitter
def lower_priority_R_wcrt(task_i, hp_tasks_excluding_v, Cv, Tv, delta, Di, iter_limit=200):
    """
    R_{i}^{(k+1)} = Ci + sum_{hp j!=v} ceil(R/Tj)*Cj + max(0, ceil((R - delta)/Tv))*Cv
    """
    Ci = task_i['C']
    Rk = Ci
    for _ in range(iter_limit):
        interference = 0
        for hj in hp_tasks_excluding_v:
            interference += math.ceil(Rk / hj['T']) * hj['C']
        extra_from_v = 0
        if (Rk - delta) > 0:
            extra_from_v = math.ceil((Rk - delta) / Tv) * Cv
        Rnext = Ci + interference + max(0, extra_from_v)
        if abs(Rnext - Rk) < 1e-9:
            return Rnext, True
        if Rnext > Di + 1e-9:
            return Rnext, False
        Rk = Rnext
    return Rk, False

# Check existence of positive delta that satisfies Eqns 12-16 for a selected victim
def exists_positive_delta_schedulable(tasks, victim_index, delta_search_count=delta_samples):
    """
    tasks: list of dicts (unsorted)
    victim_index: index in tasks list (before RM ordering)
    returns: (found_boolean, found_delta)
    Uses single delta applied to all victim jobs and checks all Rv,k <= Dv - delta and all lp tasks schedulable with Eq.15
    """
    # Sort tasks by RM order but need to identify which index is the victim in sorted order
    tasks_sorted = sorted([(i, t) for i, t in enumerate(tasks)], key=lambda it: it[1]['T'])
    # find sorted index of victim
    sorted_indices = [it[0] for it in tasks_sorted]
    try:
        v_sorted_pos = sorted_indices.index(victim_index)
    except ValueError:
        return False, None
    # build ordered list of tasks for convenience
    ordered_tasks = [it[1] for it in tasks_sorted]
    Tv = ordered_tasks[v_sorted_pos]['T']; Cv = ordered_tasks[v_sorted_pos]['C']; Dv = ordered_tasks[v_sorted_pos]['D']
    # δ must satisfy 0 < δ < Dv - Cv (strictly positive). If Dv - Cv <= 0, impossible
    if Dv - Cv <= 1e-9:
        return False, None
    # hyperperiod
    T_list = [t['T'] for t in ordered_tasks]
    H = hyperperiod(T_list)
    # victim release times within hyperperiod
    Nv = H // Tv
    r_v_k = np.arange(0, Nv * Tv, Tv)  # releases at 0, Tv, 2Tv, ...
    # Precompute hp sets for each victim instance: hp are tasks with index < v_sorted_pos in ordered list
    hp_tasks = ordered_tasks[:v_sorted_pos]
    # Lower-priority tasks set:
    lp_tasks = ordered_tasks[v_sorted_pos+1:]
    # search δ on a grid (start from small positive)
    max_delta = Dv - Cv - 1e-9
    if max_delta <= 1e-9:
        return False, None
    delta_grid = np.linspace(1e-4, max_delta, delta_search_count)
    for delta in delta_grid:
        all_victim_ok = True
        # For each victim job instance, compute r' and I(k), then WCRT via Eq.13
        for rv in r_v_k:
            r_prime = rv + delta
            Ik = carry_in_I(r_prime, hp_tasks)  # Eq 12 sum over hp
            Rvk, conv = victim_R_wcrt(Cv, hp_tasks, Ik, Dv - delta)
            if (not conv) or (Rvk > Dv - delta + 1e-9):
                all_victim_ok = False
                break
        if not all_victim_ok:
            continue
        # check all lower-priority tasks using Eq.15 (delta = min(...)=delta here)
        all_lp_ok = True
        for li, task in enumerate(lp_tasks):
            # hp for this lower-priority task are tasks with higher priority than it (in ordered_tasks)
            idx_in_ordered = v_sorted_pos + 1 + li
            hp_for_i = ordered_tasks[:idx_in_ordered]  # includes victim; we will exclude victim in hp_excluding_v
            # create hp_excluding_v
            hp_excl_v = [t for t in hp_for_i if t is not ordered_tasks[v_sorted_pos]]
            Ri, conv_i = lower_priority_R_wcrt(task, hp_excl_v, Cv, Tv, delta, task['D'])
            if (not conv_i) or (Ri > task['D'] + 1e-9):
                all_lp_ok = False
                break
        if all_lp_ok:
            return True, delta
    return False, None

# ---------------------------
# Main experiment driver
# ---------------------------

def run_experiment(n_tasks, num_sets_per_group, util_groups, case_mode='all'):
    """
    case_mode:
        'all' : case 1 -> evaluate on all generated sets
        'rm_only' : case 2 -> evaluate only RM-schedulable sets
    Returns: results dict mapping group index to statistics
    """
    results = {}
    for g_idx, (u_low, u_high) in enumerate(util_groups):
        sets = []
        rm_sched_flags = []
        cdsched_flags_hp = []
        cdsched_flags_mp = []
        cdsched_flags_lp = []
        # generate sets
        for s in range(num_sets_per_group):
            U_total = random.uniform(u_low, u_high)
            tasks = generate_taskset_for_util(U_total, n_tasks)
            sets.append(tasks)
        # First pass: RM schedulability
        for tasks in sets:
            rm_ok, _ = rm_rta_schedulable(tasks)
            rm_sched_flags.append(rm_ok)
        # Decide which sets to evaluate for CDSched per case
        if case_mode == 'all':
            eval_indices = range(len(sets))
        elif case_mode == 'rm_only':
            eval_indices = [i for i, ok in enumerate(rm_sched_flags) if ok]
        else:
            raise ValueError("Invalid case_mode")

        # For each set to evaluate, pick one victim from each HP/MP/LP group and test existence of positive delta
        for i, tasks in enumerate(sets):
            # pre-fill false; if not evaluated (case rm_only and not rm_ok) they remain False
            cdsched_flags_hp.append(False)
            cdsched_flags_mp.append(False)
            cdsched_flags_lp.append(False)

        for idx in eval_indices:
            tasks = sets[idx]
            # RM sorted order
            tasks_sorted = sorted([(i, t) for i, t in enumerate(tasks)], key=lambda it: it[1]['T'])
            n = len(tasks)
            # group boundaries (integer division; paper uses n/3 boundaries)
            g1 = n // 3
            g2 = 2 * (n // 3)
            # if n not divisible, ensure groups non-empty
            if g1 == 0:
                g1 = 1
            if g2 <= g1:
                g2 = g1 + 1 if g1+1 < n else g1
            # indices in sorted order
            hp_range = list(range(0, g1))
            mp_range = list(range(g1, g2))
            lp_range = list(range(g2, n))
            # if any group empty, adjust by moving one element
            if len(mp_range) == 0 and len(hp_range) > 1:
                mp_range = [hp_range.pop()]
            if len(lp_range) == 0 and len(mp_range) > 1:
                lp_range = [mp_range.pop()]

            # pick a victim random from each group (map back to original index)
            # note: tasks_sorted is list of (orig_idx, task)
            if hp_range:
                chosen = random.choice(hp_range)
                victim_orig_idx = tasks_sorted[chosen][0]
                ok, delta = exists_positive_delta_schedulable(tasks, victim_orig_idx)
                cdsched_flags_hp[idx] = ok
            if mp_range:
                chosen = random.choice(mp_range)
                victim_orig_idx = tasks_sorted[chosen][0]
                ok, delta = exists_positive_delta_schedulable(tasks, victim_orig_idx)
                cdsched_flags_mp[idx] = ok
            if lp_range:
                chosen = random.choice(lp_range)
                victim_orig_idx = tasks_sorted[chosen][0]
                ok, delta = exists_positive_delta_schedulable(tasks, victim_orig_idx)
                cdsched_flags_lp[idx] = ok

        # compute fractions
        total_sets = len(sets)
        rm_frac = sum(rm_sched_flags) / total_sets if total_sets>0 else 0.0
        # For case_mode == 'rm_only', denominator for CDSched fractions is number of RM-sched sets (per your description)
        denom_all = total_sets
        denom_rm_only = max(1, sum(rm_sched_flags))  # avoid div by 0
        if case_mode == 'all':
            denom = denom_all
        else:
            denom = denom_rm_only

        hp_frac = sum(cdsched_flags_hp) / denom
        mp_frac = sum(cdsched_flags_mp) / denom
        lp_frac = sum(cdsched_flags_lp) / denom

        results[g_idx] = {
            'util_range': (u_low, u_high),
            'total_sets': total_sets,
            'rm_frac': rm_frac,
            'cds_hp': hp_frac,
            'cds_mp': mp_frac,
            'cds_lp': lp_frac
        }
        print(f"group {g_idx} util {u_low:.3f}-{u_high:.3f}: RM_frac={rm_frac:.3f}, CDS_hp={hp_frac:.3f}, CDS_mp={mp_frac:.3f}, CDS_lp={lp_frac:.3f}")
    return results

# ---------------------------
# Run both experiment modes and plot
# ---------------------------
if __name__ == "__main__":
    print("Running case 1: evaluate on all generated sets")
    res_all = run_experiment(n_tasks, num_sets_per_group, util_groups, case_mode='all')
    print("\nRunning case 2: evaluate only RM-schedulable sets")
    res_rmonly = run_experiment(n_tasks, num_sets_per_group, util_groups, case_mode='rm_only')

    # Prepare plot arrays
    x_labels = [f"{low:.2f}-{high:.2f}" for (low,high) in [res_all[i]['util_range'] for i in sorted(res_all.keys())]]
    x = np.arange(len(x_labels))
    rm_vals = [res_all[i]['rm_frac'] for i in sorted(res_all.keys())]
    cds_hp_vals = [res_all[i]['cds_hp'] for i in sorted(res_all.keys())]
    cds_mp_vals = [res_all[i]['cds_mp'] for i in sorted(res_all.keys())]
    cds_lp_vals = [res_all[i]['cds_lp'] for i in sorted(res_all.keys())]

    # For second case (rm-only) overlay as dashed lines
    rmonly_hp = [res_rmonly[i]['cds_hp'] for i in sorted(res_rmonly.keys())]
    rmonly_mp = [res_rmonly[i]['cds_mp'] for i in sorted(res_rmonly.keys())]
    rmonly_lp = [res_rmonly[i]['cds_lp'] for i in sorted(res_rmonly.keys())]

    plt.figure(figsize=(10,6))
    plt.plot(x, cds_hp_vals, label='CDSched v:HP (all)', marker='o')
    plt.plot(x, cds_mp_vals, label='CDSched v:MP (all)', marker='o')
    plt.plot(x, cds_lp_vals, label='CDSched v:LP (all)', marker='o')
    plt.plot(x, rm_vals, label='RM schedulability', marker='s', linewidth=2, color='k')
    # rm-only lines
    # plt.plot(x, rmonly_hp, '--', label='CDSched v:HP (RM-only)')
    # plt.plot(x, rmonly_mp, '--', label='CDSched v:MP (RM-only)')
    # plt.plot(x, rmonly_lp, '--', label='CDSched v:LP (RM-only)')

    plt.xticks(x, x_labels, rotation=30)
    plt.ylim(-0.05, 1.05)
    plt.xlabel('Utilization group')
    plt.ylabel('Schedulability fraction')
    plt.title(f'Schedulability results (n={n_tasks}) — RM and CDSched')
    plt.legend()
    plt.tight_layout()
    plt.savefig('fig8_like_schedulability.png', dpi=200)
    plt.show()
