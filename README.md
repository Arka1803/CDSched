# 🛡️ CDSched Optimization Scripts

> **Control-Task Delay Scheduling for Real-Time Security**

This module implements the core optimization algorithms for **CDSched**, a security-aware scheduling framework that protects control tasks from untrusted co-located workloads through strategic delay injection.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Project Structure](#project-structure)
- [Workflow](#workflow)
- [Core Components](#core-components)
- [Usage](#usage)
- [Dependencies](#dependencies)
- [Output Files](#output-files)

---

## 🎯 Overview

CDSched addresses a critical security challenge in mixed-criticality real-time systems: **protecting control tasks from timing-based attacks** originating from untrusted workloads. The framework strategically delays control task execution to minimize overlap between the **Attack Effective Window (AEW)** and untrusted task execution windows.

### Key Concepts

| Term | Description |
|------|-------------|
| **AEW (Attack Effective Window)** | Time window `Ω` during which a control task is vulnerable to attacks |
| **Δ_peak** | Maximum allowable delay for a control task while maintaining schedulability |
| **WCRT** | Worst-Case Response Time under fixed-priority scheduling |
| **Victim Task** | Control task being protected from overlap with untrusted tasks |

---

## 📁 Project Structure

```
optimization_script/
│
├── 📄 opt.py                    # Main MILP optimizer (small task sets)
├── 📄 opt_rover.py              # MILP optimizer for ArduPilot rover tasks
├── 📄 delta_peak.py             # Δ_peak computation (Eq. 9, 13, 15)
├── 📄 rover.cpp                 # Reference C++ rover scheduler
├── 📄 model.ilp                 # Gurobi IIS output (debugging)
│
├── 📂 schedulability/           # Schedulability analysis tools
│   ├── schedulability_analysis.py   # Full schedulability evaluation
│   ├── opt_rebutttal.py             # Extended optimizer (rebuttal)
│   └── outputs/                     # Generated results
│
├── 📂 rebuttal_plots/           # Visualization scripts
│   ├── plot.py                  # Plotting utilities
│   └── plot_20                  # Sample plot data
│
└── 📂 outputs/                  # Optimization results
```

---

## 🔄 Workflow

```
┌─────────────────────────────────────────────────────────────────────┐
│                        CDSched Pipeline                              │
└─────────────────────────────────────────────────────────────────────┘

          ┌─────────────────┐
          │  1. Task Set    │
          │   Definition    │
          │  (Period, WCET, │
          │   Deadline, Ω)  │
          └────────┬────────┘
                   │
                   ▼
     ┌─────────────────────────────┐
     │  2. Compute Δ_peak (Max     │
     │     Allowable Delay)        │◄─── delta_peak.py
     │  ────────────────────────── │
     │  • Carry-in interference    │
     │  • Victim WCRT (Eq. 13)     │
     │  • LP task validation       │
     └─────────────┬───────────────┘
                   │
                   ▼
     ┌─────────────────────────────┐
     │  3. MILP Optimization       │◄─── opt.py / opt_rover.py
     │  ────────────────────────── │
     │  • Minimize AEW overlap     │
     │  • Subject to Δ ≤ Δ_peak    │
     │  • Per-job delay variables  │
     └─────────────┬───────────────┘
                   │
                   ▼
     ┌─────────────────────────────┐
     │  4. Schedulability          │◄─── schedulability_analysis.py
     │     Verification            │
     │  ────────────────────────── │
     │  • RM RTA check             │
     │  • CDSched feasibility      │
     │  • Success rate evaluation  │
     └─────────────┬───────────────┘
                   │
                   ▼
          ┌─────────────────┐
          │  5. Output      │
          │  • δ_k schedule │
          │  • Overlap %    │
          │  • Plots        │
          └─────────────────┘
```

---

## 🧩 Core Components

### 1️⃣ `delta_peak.py` - Maximum Delay Computation

Computes the **maximum allowable delay** (Δ_peak) for each control task using schedulability analysis.

```python
# Key functions:
carry_in_I(r_v_k_prime, hp_tasks)      # Eq. 12: Carry-in interference
compute_Rv_k(Cv, Dv_k, r_v_k_prime)    # Eq. 13: Victim WCRT
compute_lower_task_wcrt(...)           # Eq. 15: LP task WCRT with jitter
compute_delta_peak_for_taskset(tasks)  # Main computation
```

**Equations Implemented:**
- **Eq. 12**: Carry-in interference for job k
- **Eq. 13**: Victim response time with delayed release
- **Eq. 15**: Lower-priority task response time accounting for jitter

---

### 2️⃣ `opt.py` / `opt_rover.py` - MILP Optimizer

Formulates and solves a **Mixed-Integer Linear Program** to find optimal delay assignments that minimize AEW overlap.

```python
# Decision Variables:
δ_k ∈ [0, Δ_peak]  for each victim job k

# Objective:
minimize Σ overlap(victim_job_k, untrusted_job_m)

# Overlap computation uses auxiliary variables:
z[k,m]    = overlap amount
s_max     = max(start_v, start_u)  via big-M
e_min     = min(end_v, end_u)      via big-M
```

**Task Set Format:**
```python
tasks = [
    # (name, period, wcet, deadline, Omega, Delta_peak, type)
    ("tau1", 10, 2, 10, 8, 3, "control"),
    ("tau4", 100, 5, 100, None, None, "nonctrl"),
    ...
]
```

---

### 3️⃣ `schedulability/schedulability_analysis.py` - Feasibility Evaluation

Generates random task sets and evaluates:
- **Standard RM schedulability** (Rate Monotonic RTA)
- **CDSched feasibility** (with positive delay)

```python
# Key functions:
uunifast_distribute(n, U)              # Task utilization generation
rm_rta_schedulable(tasks)              # Standard RM analysis
exists_positive_delta_schedulable()    # CDSched feasibility check
```

---

## 🚀 Usage

### Prerequisites

```bash
pip install numpy gurobipy matplotlib tqdm
```

> **Note:** Gurobi requires a valid license. Academic licenses are free.

### Running the Optimizer

```bash
# Basic optimization (small task set)
python opt.py

# ArduPilot rover task set
python opt_rover.py

# Compute Δ_peak values
python delta_peak.py

# Run schedulability analysis
cd schedulability
python schedulability_analysis.py
```

### Example Output

```
Hyperperiod H = 200 ms
Conservative WCRT per task:
  tau1 : 2 ms
  tau2 : 5 ms
  tau3 : 4 ms

Solving for victim: tau3
Model statistics for tau3:
  Total variables: 156 (108 continuous + 48 binary)
  Total constraints: 576
  Optimal delta_k sequence: ['2.500', '0.000', '1.250', ...]
  Baseline total overlap: 45.000 ms
  Optimal total overlap:  12.500 ms
  Overlap reduction: 72.22%
```

---

## 📦 Dependencies

| Package | Purpose | Version |
|---------|---------|---------|
| `numpy` | Numerical computations | ≥1.20 |
| `gurobipy` | MILP solver | ≥9.0 |
| `matplotlib` | Plotting | ≥3.0 |
| `tqdm` | Progress bars | ≥4.0 |

---

## 📊 Output Files

| File | Description |
|------|-------------|
| `model.ilp` | Gurobi IIS output for infeasible models |
| `outputs/` | Optimization results and logs |
| `schedulability/*.png` | Schedulability plots |
| `rebuttal_plots/` | Publication-ready figures |

---

## 📚 References

This implementation is based on the CDSched paper equations:

- **Eq. 9**: Δ_peak definition
- **Eq. 12**: Carry-in interference I(k,i)
- **Eq. 13**: Victim response time R_{v,k}
- **Eq. 15**: Lower-priority task response time
- **Eq. 16**: Overall schedulability condition

---

## 👤 Author

**Arkaprava Sain**

---

## 📝 License

Research/Academic Use
