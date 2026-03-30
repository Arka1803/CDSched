# CDSched Optimization Scripts

> **Control-Task Delay Scheduling for Real-Time Security**

This module implements the core optimization algorithms for **CDSched**, a security-aware scheduling framework that protects control tasks from untrusted workloads through strategic delay injection.

---

## Quick Start Guide

### 0. Install Requirements
Run standard pip instruction to install requisite python modules globally for all scripts:
```bash
pip install -r requirements.txt
```

### Step 1: Compute Delta Peak for Task Sets (`delta_peak.py`)
Calculate the specific $\Delta^{peak}$ (Delta Peak) limits for control tasks interacting with untrusted scheduling environments.

**Format Your Data (`src/optimization/input.txt`):**
Ensure your tasks are populated row-by-row mapping variables identical to this layout before execution.
```text
# Expected Format:
# name, T, C, D, Omega, Delta_peak, type
tau1, 10, 2, 10, 10, None, control
tau2, 40, 3, 40, 7,  None, control
...
```

**Execute the command:**
Run the evaluation providing the path directly to your loaded target input set:
```bash
python src/optimization/delta_peak.py src/optimization/input.txt
```
*(If you run the script without any arguments, it will automatically default to `src/optimization/input.txt`.)*

### Step 2: Multi-Sequence Delay Optimization (`opt_multiple_solve.py`)
For comprehensive scenario mapping over victim functions leveraging discrete scheduling boundaries, utilize the multiple solver tool mapping natively onto your parsed target structure files limitlessly dynamically resolving constraints properly.

**Command Structure:**
```bash
python src/optimization/opt_multiple_solve.py -v <victim_task_id> -q <q> -W <W> -f <input_file.txt> [-d <delta_max_override>]
```

**Parameters Explained:**
- `-v`: Victim task identifier natively evaluated (e.g. `tau1`)
- `-q`: Vulnerable overlapping instances integer quantity.
- `-W`: Total sequence sequence length limits.
- `-f`: Explicit path identifying formatted inputs `input.txt`.
- `-d`: (Optional) Delta maximum parameter limits. If left blank, automatically fetches the explicit boundary defined within the **6th Column** of `input.txt`.

**Example:**
```bash
python src/optimization/opt_multiple_solve.py -v tau1 -q 2 -W 3 -f src/optimization/input.txt
```

### Step 3: Schedulability Analysis (`schedulability_fnal.py`)
Execute the schedulability tests (and optionally their MATLAB plotting elements) natively from your main `CDSched` directory.

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

**Generate MATLAB Plots:**
Immediately following task completion, the script pauses gently and inquiries via the console:
`Do you want to run MATLAB plotting script? (y/n):`
Answering `y` engages MATLAB completely seamlessly. 
- **Outputs generated:** A collection of highly refined graphical maps (`.png`, `_hires.png`, and `.fig`) seamlessly route into the internal `plot/` folder identically formatted (e.g., `plot/schedulability_10_plot.png`).

## License
Research/Academic Use
