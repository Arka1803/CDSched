"""
Experiment script for Δ_peak schedulability test (based on offset_paper.pdf)
Implements Rate-Monotonic priority assignment (smaller period = higher priority).

Steps:
  1. Generate 100 task sets for each utilization group using RandFixedSum-like method.
  2. Periods chosen from P_S, integer execution times from E_S=[1..50].
  3. After sorting by period (RMPA), divide into HP / MP / LP groups.
  4. Randomly select one victim per group and check schedulability:
        - Eq.12: Victim schedulability condition
        - Eq.17: Response time for lower-priority tasks
  5. Plot % schedulable vs utilization group for HP/MP/LP victims.
"""

import argparse
import numpy as np
import math
import random
import matplotlib.pyplot as plt
import pandas as pd
import subprocess
import os
from typing import List, Tuple

# === PARAMETERS you can edit ===
P_S = [5, 10, 20, 50, 100, 200, 500, 1000]  # allowed periods
E_MIN, E_MAX = 1, 30  # integer WCET range
NUM_SETS_PER_GROUP = 100
d_search_max = max(P_S)  # search δ from 0..d_search_max (integer)
# =================================

# ---------- Utility functions ----------

def sample_total_util_from_range(ut_low: float, ut_high: float) -> float:
    return random.uniform(ut_low, ut_high)

def rand_fixed_sum_dirichlet(n: int, U: float) -> np.ndarray:
    # Simple RandFixedSum-like via Dirichlet
    alpha = np.ones(n)
    x = np.random.gamma(shape=alpha, scale=1.0)
    x = x / np.sum(x)
    return (x * U)

def build_taskset_from_util_vector(uvec: np.ndarray, P_S: List[int]) -> List[dict]:
    n = len(uvec)
    tasks = []
    for i in range(n):
        Ti = int(random.choice(P_S))
        Ci = int(round(uvec[i] * Ti))
        Ci = max(E_MIN, min(E_MAX, Ci))
        Ci = min(Ci, Ti)  # Ensure C <= T
        Di = Ti  # implicit deadlines
        tasks.append({'id': i+1, 'C': Ci, 'T': Ti, 'D': Di})
    # Apply Rate-Monotonic priority ordering (smaller T = higher priority)
    tasks = sorted(tasks, key=lambda t: (t['T'], t['id']))
    for i, t in enumerate(tasks):
        t['id'] = i + 1  # reassign IDs by priority rank (1 = highest)
    return tasks

# ---------- Schedulability checks ----------

def calculate_victim_response_time(tasks: List[dict], victim_idx: int) -> float:
    """
    Calculate victim response time R_v using Equations 12, 14, 15.
    R_v = C_v + I_s1 + I_s2
    """
    v = tasks[victim_idx]
    Cv, Tv = v['C'], v['T']
    
    # Calculate R_v using fixed-point iteration (Equation 15)
    R_v = Cv
    for _ in range(1000):
        I_s1 = 0  # Interference from higher priority tasks
        
        # Interference from higher priority tasks
        for j, t in enumerate(tasks):
            if j == victim_idx:
                continue
            if t['T'] < v['T']:  # higher priority (Rate Monotonic)
                Tj, Cj = t['T'], t['C']
                I_s1 += math.ceil(R_v / Tj) * Cj
        
        # I_s2: Self-interference from victim's own subsequent jobs (Eq. 14)
        I_s2 = max(0, math.ceil((R_v - Cv) / Tv)) * Cv
        
        # Equation 15: R_v = C_v + I_s1 + I_s2
        R_v_new = Cv + I_s1 + I_s2
        
        if R_v_new == R_v:
            return R_v
        
        R_v = R_v_new
    
    return R_v

def is_victim_schedulable_with_delta(tasks: List[dict], victim_idx: int) -> Tuple[bool, int]:
    """
    Check if victim is schedulable and return delta_peak.
    Delta_peak = D_v - R_v
    Returns (is_schedulable, delta_peak)
    """
    v = tasks[victim_idx]
    Dv = v['D']
    
    R_v = calculate_victim_response_time(tasks, victim_idx)
    delta_peak = Dv - R_v
    
    if delta_peak > 0:
        return True, int(delta_peak)
    else:
        return False, -1

def compute_response_time_lower_task(task_i: dict, tasks: List[dict],
                                     victim_idx: int, delta: int, max_iters=1000) -> Tuple[float,bool]:
    """Implements Eq.17 iteratively for lower-priority tasks."""
    Ci, Di = task_i['C'], task_i['D']
    hp = [t for t in tasks if t['T'] < task_i['T']]  # higher priority set
    victim = tasks[victim_idx]
    Cv, Tv = victim['C'], victim['T']
    hp_non_v = [t for t in hp if t['T'] != Tv or t['C'] != Cv]
    R = Ci
    for _ in range(max_iters):
        newR = Ci
        for t in hp_non_v:
            newR += math.ceil(R / t['T']) * t['C']
        # victim contribution term
        if victim['T'] < task_i['T']:
            newR += math.ceil((R + delta) / Tv) * Cv
            ####
            
        if newR == R:
            return R, True
        if newR > Di:
            return newR, False
        R = newR
    return R, False

def is_lower_tasks_schedulable(tasks: List[dict], victim_idx: int, delta: int) -> bool:
    """Ensure all lower-priority tasks (T_i > T_v) satisfy Ri <= Di."""
    Tv = tasks[victim_idx]['T']
    for t in tasks:
        if t['T'] > Tv:
            R, conv = compute_response_time_lower_task(t, tasks, victim_idx, delta)
            if R > t['D']:
                return False
    return True

def find_delta_peak(tasks: List[dict], victim_idx: int, dmax_search: int) -> int:
    """
    Find delta_peak that ensures victim and all lower-priority tasks are schedulable.
    Based on Equations 15 and 17.
    """
    # First check if victim is schedulable and get delta_peak
    is_sched, delta_peak = is_victim_schedulable_with_delta(tasks, victim_idx)

    
    
    if not is_sched or delta_peak <= 0:
        return -1
    
    # Check if all lower-priority tasks are schedulable with this delta_peak
    if is_lower_tasks_schedulable(tasks, victim_idx, delta_peak):
        return delta_peak
    else:
        return -1

# ---------- Experiment driver ----------

def run_experiment(util_ranges: List[Tuple[float,float]], n_tasks:int,
                   P_S=P_S, sets_per_group=NUM_SETS_PER_GROUP,
                   d_search=d_search_max):
    results, labels = [], []
    util_midpoints = []  # Store midpoint of each utilization range
    
    for (low, high) in util_ranges:
        labels.append(f"{low:.2f}-{high:.2f}")
        util_midpoints.append((low + high) / 2)
        count_HP = count_MP = count_LP = 0
        for _ in range(sets_per_group):
            Utot = sample_total_util_from_range(low, high)
            uvec = rand_fixed_sum_dirichlet(n_tasks, Utot)
            tasks = build_taskset_from_util_vector(uvec, P_S)

            n = n_tasks
            g1, g2 = n // 3, 2 * (n // 3)
            if g1 == 0: g1 = 1
            if g2 <= g1: g2 = g1 + 1 if g1 + 1 < n else g1

            hp_choices = [t['id'] for t in tasks if t['id'] <= g1]
            mp_choices = [t['id'] for t in tasks if g1 < t['id'] <= g2]
            lp_choices = [t['id'] for t in tasks if t['id'] > g2]
            if len(hp_choices) == 0: hp_choices = [tasks[0]['id']]
            if len(mp_choices) == 0: mp_choices = [tasks[min(1, n-1)]['id']]
            if len(lp_choices) == 0: lp_choices = [tasks[-1]['id']]

            victim_hp = random.choice(hp_choices) - 1
            victim_mp = random.choice(mp_choices) - 1
            victim_lp = random.choice(lp_choices) - 1

            d_hp = find_delta_peak(tasks, victim_hp, d_search)
            d_mp = find_delta_peak(tasks, victim_mp, d_search)
            d_lp = find_delta_peak(tasks, victim_lp, d_search)

            if d_hp > 0: count_HP += 1
            if d_mp > 0: count_MP += 1
            if d_lp > 0: count_LP += 1

        results.append((count_HP/sets_per_group*100,
                        count_MP/sets_per_group*100,
                        count_LP/sets_per_group*100))
        print(f"Util {low:.3f}-{high:.3f}: HP {results[-1][0]:.1f}%  MP {results[-1][1]:.1f}%  LP {results[-1][2]:.1f}%")
    
    # Create DataFrame for CSV export
    df = pd.DataFrame({
        'Util_Range': labels,
        'Util_Midpoint': util_midpoints,
        'HP_Schedulability': [r[0] for r in results],
        'MP_Schedulability': [r[1] for r in results],
        'LP_Schedulability': [r[2] for r in results]
    })
    
    return labels, np.array(results), df

# ---------- Plotting ----------

def plot_results(labels, results_array):
    x = np.arange(len(labels))
    plt.figure(figsize=(10,5))
    plt.plot(x, results_array[:,0], marker='^', label='HP', linewidth=2, markersize=8, color='blue')
    plt.plot(x, results_array[:,1], marker='s', label='MP', linewidth=2, markersize=8, color='green')
    plt.plot(x, results_array[:,2], marker='o', label='LP', linewidth=2, markersize=8, color='red')
    plt.xticks(x, labels, rotation=45)
    plt.ylabel('% Schedulability (Δ_peak > 0)')
    plt.xlabel('Utilization group (U_total range)')
    plt.ylim(-5,105)
    plt.title('Percent schedulability vs Util group (victim from HP/MP/LP)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# ---------- Main ----------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Experiment script for Δ_peak schedulability test")
    parser.add_argument("n_tasks", type=int, help="Number of tasks per set")
    args = parser.parse_args()
    
    # Example usage (replace with your own ranges)
    util_ranges = [
        (0.02,0.08),(0.12,0.18),(0.22,0.28),
        (0.32,0.38),(0.42,0.48),(0.52,0.58),
        (0.62,0.68),(0.72,0.78),(0.82,0.88),(0.92,0.98)
    ]
    n_tasks = args.n_tasks
    labels, results, df = run_experiment(util_ranges, n_tasks, sets_per_group=100)
    
    # Save results to CSV robustly regardless of current execution directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(os.path.dirname(script_dir))
    output_dir = os.path.join(project_root, 'output')
    os.makedirs(output_dir, exist_ok=True)
    
    csv_filename = os.path.join(output_dir, f'schedulability_{n_tasks}.csv')
    df.to_csv(csv_filename, index=False)
    
    # Pre-format path safely for MATLAB to prevent escape character injection
    csv_filename_matlab = csv_filename.replace('\\', '/')
    print(f"\n✓ Results saved to {csv_filename}")
    
    # Plot results with matplotlib
    plot_results(labels, results)
    
    # Ask user if they want to run MATLAB script
    print("\n" + "="*60)
    run_matlab = input("Do you want to run MATLAB plotting script? (y/n): ").strip().lower()
    
    if run_matlab == 'y':
        matlab_script = 'plot_schedulability_results.m'
        matlab_script_path = os.path.join(script_dir, matlab_script)
        
        if not os.path.exists(matlab_script_path):
            print(f"✗ MATLAB script '{matlab_script}' not found in {script_dir}.")
        else:
            print(f"\nRunning MATLAB script: {matlab_script}")
            try:
                # Try to run MATLAB script
                # -batch runs without GUI, -r runs a command
                matlab_cmd = f"csv_filename='{csv_filename_matlab}'; plot_schedulability_results"
                result = subprocess.run(
                    ['matlab', '-batch', matlab_cmd],
                    cwd=script_dir,
                    capture_output=True,
                    text=True,
                    timeout=120  # 2 minute timeout
                )
                
                if result.returncode == 0:
                    print("✓ MATLAB script executed successfully!")
                    print("\nMATLAB Output:")
                    print(result.stdout)
                else:
                    print(f"✗ MATLAB script failed with error code {result.returncode}")
                    print("\nError output:")
                    print(result.stderr)
                    
            except FileNotFoundError:
                print("\n✗ MATLAB not found in system PATH.")
                print("   You can run the script manually in MATLAB:")
                print(f"   >> cd '{os.getcwd()}'")
                print("   >> plot_schedulability_results")
                
            except subprocess.TimeoutExpired:
                print("\n✗ MATLAB script timed out after 2 minutes.")
                
            except Exception as e:
                print(f"\n✗ Error running MATLAB: {e}")
                print("   You can run the script manually in MATLAB:")
                print(f"   >> cd '{os.getcwd()}'")
                print("   >> plot_schedulability_results")
    else:
        print("\nSkipping MATLAB plotting. You can run it manually in MATLAB:")
        print(f">> cd '{os.getcwd()}'")
        print(">> plot_schedulability_results")
    
    print("\n" + "="*60)
    print("Experiment completed!")
