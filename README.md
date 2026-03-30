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


## License
Research/Academic Use
