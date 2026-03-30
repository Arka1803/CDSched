import subprocess
import argparse
import sys
import os
import re

def run_matlab_optimization(matlab_file):
    """
    Runs a MATLAB file using -batch mode and extracts delta_max from the console output.
    """
    if not os.path.exists(matlab_file):
        print(f"Error: {matlab_file} not found.")
        sys.exit(1)
        
    script_dir = os.path.dirname(os.path.abspath(matlab_file))
    script_base = os.path.basename(matlab_file)
    script_name, _ = os.path.splitext(script_base)
    
    print(f"Running MATLAB optimization: {script_name} from {script_dir}...")
    
    # Run MATLAB in batch mode from the script's directory
    cmd = ['matlab', '-batch', script_name]
    result = subprocess.run(cmd, cwd=script_dir, capture_output=True, text=True)
    
    output = result.stdout
    print(output)
    
    # Check for any fatal errors 
    if "Error" in output or result.returncode != 0:
        print("\n=> Error running MATLAB script.")
        print(result.stderr)
        
    # Search the output dynamically for 'delta_max' matching logic depending on how MATLAB prints it
    # E.g., "delta_max = 0.45" or "maximum delta : 0.45"
    match = re.search(r'(?i)(?:delta_?max|max(?:imum)?\s+delta).*?[=:]\s*([\d\.]+)', output)
    
    if match:
        delta_max = float(match.group(1))
        # Save to file
        output_file = os.path.join(script_dir, f"{script_name}_results.txt")
        with open(output_file, 'w') as f:
            f.write(f"delta_max={delta_max}\n")
        print(f"\n=> Successfully computed and stored delta_max = {delta_max} in {output_file}")
    else:
        print("\n=> Could not find a 'delta_max' or 'max delta' value in the MATLAB standard output!")
        print("   Please ensure your MATLAB script explicitly prints it out (e.g., fprintf('delta_max = %f\\n', delta_max)).")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run MATLAB CLF Optimization, parse, and store delta_max")
    parser.add_argument('matlab_file', help="Path to the MATLAB file (e.g., src/CLF/TTC.m)")
    args = parser.parse_args()
    
    run_matlab_optimization(args.matlab_file)
