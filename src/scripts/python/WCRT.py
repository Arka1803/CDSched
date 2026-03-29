import matplotlib.pyplot as plt
import pandas as pd
from math import ceil, floor, gcd

# Define example task set
tasks = {
    "tau1": {"C": 1, "T": 5, "D": 5},    # Highest priority non-victim
    "tauv": {"C": 3, "T": 10, "D": 10},  # Victim task
    "tau3": {"C": 3, "T": 15, "D": 15},  # Lower priority task
    "tau4": {"C": 2, "T": 20, "D": 20}   # Lowest priority task
}

# Higher priority tasks function
def hp(task_name):
    """Return list of higher priority tasks than the given task"""
    priority_order = ["tau1", "tauv", "tau3", "tau4"]  # Rate monotonic priority
    try:
        task_index = priority_order.index(task_name)
        return priority_order[:task_index]
    except ValueError:
        return []

# Simple WCRT computation based on simple_wcrt.py logic
def wcrt_victim(tasks, d):
    """Simple WCRT computation for victim with delay d"""
    Cv = tasks["tauv"]["C"]
    Dv = tasks["tauv"]["D"]
    
    R = Cv + d  # Base: execution time + delay
    
    # Fixed point iteration for interference
    for _ in range(10):
        R_new = Cv + d
        for jn in hp("tauv"):
            Cj, Tj = tasks[jn]["C"], tasks[jn]["T"]
            R_new += ceil(R / Tj) * Cj
        if R_new == R:
            break
        R = R_new
    
    schedulable = (R <= Dv)
    return int(R), schedulable

def wcrt_nonvictims(tasks, d):
    """Simple WCRT for other tasks based on simple_wcrt.py logic"""
    results = {}
    ok = True
    
    for name in tasks:
        if name == "tauv":
            continue
            
        Ci, Ti, Di = tasks[name]["C"], tasks[name]["T"], tasks[name]["D"]
        
        R = Ci
        
        for _ in range(10):
            R_new = Ci
            for jn in hp(name):
                Cj, Tj = tasks[jn]["C"], tasks[jn]["T"]
                if jn == "tauv":
                    # For victim task, add extra delay effect
                    R_new += ceil((R + d) / Tj) * Cj
                else:
                    R_new += ceil(R / Tj) * Cj
            if R_new == R:
                break
            R = R_new
        
        schedulable = (R <= Di)
        results[name] = (int(R), schedulable)
        if not schedulable:
            ok = False
    
    return results, ok

def schedulability_scan(tasks, max_d=20):
    scan = []
    for d in range(max_d+1):
        Rv, vok = wcrt_victim(tasks, d)
        nonv, nonv_ok = wcrt_nonvictims(tasks, d)
        all_ok = vok and nonv_ok
        scan.append({
            "d": d,
            "victim_R": Rv,
            "victim_ok": vok,
            **{f"{t}_R": R for t,(R,ok) in nonv.items()},
            **{f"{t}_ok": ok for t,(R,ok) in nonv.items()},
            "all_ok": all_ok
        })
    return pd.DataFrame(scan)

# Run the analysis
scan_df = schedulability_scan(tasks, max_d=10)

# Note: τ3 appears static because its interference from τv only increases 
# when (R+d) crosses multiples of τv's period (15). With R=5 for τ3,
# this happens when d > 10, which is outside our analysis range.

# Save results to CSV file
csv_filename = "wcrt_results.csv"
scan_df.to_csv(csv_filename, index=False)
print(f"Results saved to {csv_filename}")

# Print some key results to verify
print("Key WCRT Results:")
print("Delay | Victim_R | tau1_R | tau3_R | tau4_R")
for i, row in scan_df.iterrows():
    print(f"{row['d']:5d} | {row['victim_R']:8.1f} | {row['tau1_R']:6.1f} | {row['tau3_R']:6.1f} | {row['tau4_R']:6.1f}")
print()

# Plot
plt.figure(figsize=(10,6))

# Define colors for each task
task_colors = {
    "tauv": "red",
    "tau1": "blue", 
    "tau3": "green",
    "tau4": "orange"
}

# Plot victim response time
plt.plot(scan_df["d"], scan_df["victim_R"], marker="o", color=task_colors["tauv"], 
         linewidth=2, label="Victim τv Response Time")

# Plot all non-victim tasks response times
for tname in ["tau1", "tau3", "tau4"]:
    if f"{tname}_R" in scan_df.columns:
        plt.plot(scan_df["d"], scan_df[f"{tname}_R"], marker="o", 
                color=task_colors[tname], linewidth=2, label=f"{tname} Response Time")

# Plot deadline lines with matching colors and dotted style
for tname, t in tasks.items():
    display_name = "τv" if tname == "tauv" else tname
    plt.axhline(y=t["D"], linestyle=":", color=task_colors[tname], 
               linewidth=2, alpha=0.8, label=f"{display_name} Deadline D={t['D']}")

plt.xlabel("Delay d", fontsize=12)
plt.ylabel("WCRT", fontsize=12)
plt.ylim(0,30)
plt.title("WCRT vs job-level delay d", fontsize=14)
plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
plt.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Display results
print("Schedulability scan results:")
print(scan_df)
