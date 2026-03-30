# CDSched Optimization Scripts

> **Control-Task Delay Scheduling for Real-Time Security**

This module implements the core optimization algorithms for **CDSched**, a security-aware scheduling framework that protects control tasks from untrusted workloads through strategic delay injection.

---

## Table of Contents

- [Quick Start: Schedulability Analysis](#quick-start-schedulability-analysis)

## Quick Start: Schedulability Analysis

Follow these clear instructions to execute the schedulability tests (and their MATLAB plotting elements) natively from your main `CDSched` directory.

### 1. Install Requirements
Run standard pip instruction to install requisite python modules universally:
```bash
pip install -r requirements.txt
```

### 2. Run the Schedulability Script
The experiment runs directly via the main path bypassing CD navigation. Be sure to supply the desired number of tasks (`<n_tasks>`) as an argument.

**General Command:**
```bash
python src/schedulability/schedulability_fnal.py <n_tasks>
```

**Example (10 tasks per set):**
```bash
python src/schedulability/schedulability_fnal.py 10
```
- **What this does:** Computes the results via HP / MP / LP routines and evaluates their offset mapping directly. 
- **Outputs generated:** Your processed data `.csv` deposits directly cleanly into your internal `output/` directory (e.g., `output/schedulability_10.csv`).

### 3. Generate MATLAB Plots
Immediately following task completion, the script pauses gently and inquiries via the console:
`Do you want to run MATLAB plotting script? (y/n):`

Answering `y` engages MATLAB completely seamlessly. 
- **Outputs generated:** A collection of highly refined graphical maps (`.png`, `_hires.png`, and `.fig`) seamlessly route into the internal `plot/` folder identically formatted (e.g., `plot/schedulability_10_plot.png`).

### 4. Compute Delta Peak for Task Sets

You can additionally calculate the specific $\Delta^{peak}$ (Delta Peak) limits for control tasks interacting with untrusted scheduling environments using the `delta_peak.py` script. 

**Format Your Data (`src/optimization/input.txt`):**
Ensure your tasks are populated row-by-row mapping variables identical to this layout before execution.

```text
# Expected Format:
# name, T, C, D, Omega, Delta_peak, type
tau1, 10, 2, 10, 10, None, control
tau2, 40, 3, 40, 7,  None, control
tau3, 20, 2, 20, 5,  None, control
tau4, 100, 5, 100, None, None, nonctrl
tau5, 100, 4, 100, None, None, nonctrl
tau6, 40, 2, 40, None, None, nonctrl
```

**Execute the command:**
Run the evaluation providing the path directly to your loaded target input set:
```bash
python src/optimization/delta_peak.py src/optimization/input.txt
```
*(If you run the script without any arguments, it will automatically default to `src/optimization/input.txt`.)*


## License
Research/Academic Use
