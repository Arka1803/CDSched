"""
Helper function to run MATLAB script from Python
Can be imported or run standalone
"""

import subprocess
import os
import sys

def run_matlab_script(script_name='plot_schedulability_results', timeout=120, verbose=True):
    """
    Run a MATLAB script from Python
    
    Args:
        script_name: Name of MATLAB script (without .m extension)
        timeout: Maximum time to wait in seconds
        verbose: Print detailed output
    
    Returns:
        bool: True if successful, False otherwise
    """
    matlab_script = f"{script_name}.m"
    
    if not os.path.exists(matlab_script):
        if verbose:
            print(f"✗ MATLAB script '{matlab_script}' not found in current directory.")
        return False
    
    if verbose:
        print(f"Running MATLAB script: {matlab_script}")
    
    try:
        # Method 1: Try with -batch (MATLAB R2019a and later)
        result = subprocess.run(
            ['matlab', '-batch', script_name],
            capture_output=True,
            text=True,
            timeout=timeout
        )
        
        if result.returncode == 0:
            if verbose:
                print("✓ MATLAB script executed successfully!")
                if result.stdout:
                    print("\nMATLAB Output:")
                    print(result.stdout)
            return True
        else:
            if verbose:
                print(f"✗ MATLAB script failed with error code {result.returncode}")
                if result.stderr:
                    print("\nError output:")
                    print(result.stderr)
            return False
            
    except FileNotFoundError:
        if verbose:
            print("\n✗ MATLAB not found in system PATH.")
            print("   Alternative methods to run MATLAB script:")
            print("\n   Method 1: Add MATLAB to PATH and retry")
            print("   Method 2: Run manually in MATLAB:")
            print(f"      >> cd '{os.getcwd()}'")
            print(f"      >> {script_name}")
            print("\n   Method 3: Run from command line:")
            print(f"      matlab -batch \"{script_name}\"")
        return False
        
    except subprocess.TimeoutExpired:
        if verbose:
            print(f"\n✗ MATLAB script timed out after {timeout} seconds.")
        return False
        
    except Exception as e:
        if verbose:
            print(f"\n✗ Error running MATLAB: {e}")
            print("   You can run the script manually in MATLAB:")
            print(f"   >> cd '{os.getcwd()}'")
            print(f"   >> {script_name}")
        return False


def check_matlab_available():
    """Check if MATLAB is available in system PATH"""
    try:
        result = subprocess.run(
            ['matlab', '-batch', 'disp("MATLAB OK")'],
            capture_output=True,
            text=True,
            timeout=30
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, Exception):
        return False


if __name__ == "__main__":
    # Can be run standalone
    print("="*60)
    print("MATLAB Script Runner")
    print("="*60)
    
    if len(sys.argv) > 1:
        script_name = sys.argv[1].replace('.m', '')  # Remove .m if provided
    else:
        script_name = 'plot_schedulability_results'
    
    print(f"\nChecking if MATLAB is available...")
    if check_matlab_available():
        print("✓ MATLAB found in system PATH")
        print()
        success = run_matlab_script(script_name)
        sys.exit(0 if success else 1)
    else:
        print("✗ MATLAB not found in system PATH")
        print("\nTo add MATLAB to PATH on Windows:")
        print('  1. Search for "Environment Variables" in Start Menu')
        print('  2. Edit "Path" variable')
        print('  3. Add MATLAB bin directory, e.g.:')
        print('     C:\\Program Files\\MATLAB\\R2023a\\bin')
        sys.exit(1)
