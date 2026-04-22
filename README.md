# CDSched: Control-Task Delay Scheduling Framework

> **Control-Task Delay Scheduling for Real-Time Security**

This repository contains the core optimization framework behind **CDSched**, an scheduling framework designed to protect safety-critical control tasks from untrusted adversarial workloads through strategic computational delay injection. By calculating overlapping vulnerability intervals, the CDSched provides schedulability bounds for a given task set.

---

## System Requirements & Prerequisites

- **OS Environment:** Windows, macOS, or Linux.
- **Python:** Minimum version `3.10` or newer.
- **MATLAB:** Must be fully installed safely on your system, accessible smoothly via system PATH. You **must have active Academic or Commercial MATLAB License**
- **Gurobi Optimizer:** Requires an active Academic/Commercial license file (`gurobi.lic`) configured natively on your hardware. *(Note: Our environment structurally bounds Gurobi below v13.0 to guarantee backwards compatibility).*

---

## 0. Initial Virtual Environment Setup

Create a virtual environment 

**1. Generate the Virtual Environment:**
```bash
python -m venv venv
```

**2. Activate It:**
- **Windows:** `venv\Scripts\activate`
- **macOS/Linux:** `source venv/bin/activate`

**3. Install Dependencies:**
Load the target dependencies rigorously tracked within the deployment manifest:
```bash
pip install -r requirements.txt
```

---

## Steps to Run the Scripts:

Execute the scripts chronologically sequentially using the explicit terminal guidelines below.

### Step 1: Compute Base Delay Tolerances (`compute_delta_peak.py`)
This script evaluates the raw task properties securely determining absolute boundaries limiting delays analytically ($\Delta^{peak}$) per native control operation before logical instability occurs.

#### Configuration Formatting (`src/optimization/input.txt`)
Before running, you must declare your operational baseline configuration directly into the text bounds via comma-separated values:
- **Columns:** `Name`, `Period (T)`, `Compute Time (C)`, `Deadline (D)`, `Omega`, **`Delta_peak`**, `Type`
*(Set Attack Effective Window (AEW) Omega to `None` securely if evaluating standard non-control tasks!)*

#### Command Layout:
```bash
python src/optimization/compute_delta_peak.py [path_to_input]
```
- `[path_to_input]`: (Optional) Path to your formatted tracking input strings. If left completely blank, defaults inherently to `src/optimization/input.txt`.

**Example:**
```bash
python src/optimization/compute_delta_peak.py src/optimization/input.txt
```
> **What to expect:** The system will evaluate the baseline timing metrics structurally evaluating the longest theoretical stable overlapping delays ($\Delta^{peak}$) allowed, dropping evaluated limits logically out via Standard Terminal Output strictly.

---

### Step 2: Multi-Sequence Delay Optimization (`optimize_delay_sequences.py`)
This solver loads the established control configurations statically, identifying vulnerable overlap boundaries strictly resolving multi-interval mathematical delay mapping structurally through native Gurobi evaluation limits minimizing potential worst-case security exploits natively.

#### Command Layout:
```bash
python src/optimization/optimize_delay_sequences.py -v <TARGET> -q <Q> -W <W> -f <INPUT_FILE> [-d <OVERRIDE>]
```

#### Flags Explained:
- `-v` / `--victim`: Natively tracks the designated control target identifier logically (e.g. `tau1`).
- `-q`: **Overlap Instances.** Tracks exactly the physical quantity integer evaluating how many individual vulnerable execution overlaps you practically wish to evaluate per logical mapping sequence.
- `-W`: **Iterations / Length.** Evaluates the number of sequentially optimized output maps you explicitly want mapped globally resolving dynamic delays efficiently decreasing vulnerabilities across intervals.
- `-f` / `--file`: Path pointing smoothly to configurations identical to `input.txt`.
- `-d` / `--delta_max` *(Optional)*: Forces an absolute maximum theoretical delay tracking bound globally. If left unset, logically scales values reading identically out from your configuration text file's **6th column**.

**Example:**
```bash
python src/optimization/optimize_delay_sequences.py -v tau1 -q 2 -W 3 -f src/optimization/input.txt
```
> **What to expect:** It explicitly assesses native WCRT boundaries logically routing instances natively decreasing untargeted overlapping interactions accurately, producing terminal sequential delay constraints resolving overlapping offsets significantly!

---

### Step 3: Global Schedulability Routine (`evaluate_schedulability.py`)
Evaluates your computed parameters identically projecting them logically against mathematically rigid hardware boundaries tracking HP/MP/LP scheduling metrics reliably parsing values cleanly mathematically verifying the offsets statically structurally.

#### Command Layout:
```bash
python src/schedulability/evaluate_schedulability.py <n_tasks>
```

#### Flags Explained:
- `<n_tasks>`: Evaluates strictly generating configurations properly scaling internal parameters bounding structurally via `<n_tasks>` limits (e.g., `10` or `5`). 

**Example:**
```bash
python src/schedulability/evaluate_schedulability.py 10
```

> **What to expect:**
> 1. Displays evaluated `HP 100.0% | MP 95.0% | LP 80.0%` bounding matrix natively confirming analytical structures logically.
> 2. Pushes completed mapping values smoothly compiling data out natively mapping successfully bounded entries saving sequentially into `output/schedulability_<n_tasks>.csv`.
> 3. Natively queries your terminal safely asking: `Do you want to run MATLAB plotting script? (y/n):`. 
> 4. Typing `y` dynamically launches your global MATLAB instances accurately evaluating graphs correctly porting images cleanly out into your `plot/` structures smoothly!

---

## License & Intellectual Limits
Academic modeling restrictions apply natively to all evaluation algorithms strictly.
