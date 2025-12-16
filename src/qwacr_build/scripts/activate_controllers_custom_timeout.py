"""
Activate controllers via ros2 control set_controller_state CLI.
Bypasses ROS 2 service client issues by using subprocess.
"""

import time
import sys
import subprocess

def activate_controller(controller_name):
    """Activate a single controller using CLI. Returns True on success, False on failure."""
    cmd = [
        'ros2', 'control', 'set_controller_state',
        controller_name, 'active'
    ]
    
    print(f"Activating {controller_name}...")
    print(f"Running: {' '.join(cmd)}")
    
    try:
        result = subprocess.run(
            cmd,
            timeout=30.0,  # CLI should handle its own timeout
            capture_output=True,
            text=True,
            check=False
        )
        if result.returncode == 0:
            print(f"✓ {controller_name} activated successfully")
            if result.stdout:
                print(f"  Output: {result.stdout.strip()}")
            return True
        else:
            print(f"✗ {controller_name} activation failed with code {result.returncode}")
            if result.stderr:
                print(f"  Error: {result.stderr.strip()}")
            if result.stdout:
                print(f"  Output: {result.stdout.strip()}")
            return False
    except subprocess.TimeoutExpired:
        print(f"✗ {controller_name} activation timed out")
        return False
    except Exception as e:
        print(f"✗ Exception: {e}")
        return False


def main():
    # small delay to let controller_manager start
    time.sleep(5)

    # Activate joint_broad first
    if not activate_controller('joint_broad'):
        return 1

    # Small gap
    time.sleep(1.0)

    # Activate diff_cont
    if not activate_controller('diff_cont'):
        return 1

    print("✓ All controllers activated successfully!")
    return 0


if __name__ == '__main__':
    sys.exit(main())
