import subprocess
import time
import os
import sys

# CONFIGURATION
REPO_URL = "https://github.com/YOUR_USERNAME/YOUR_REPO.git"
SCRIPT_PATH = "./mqtt_server.py"         # Where the current script lives
CLONE_PATH = "./mqtt_server_temp"        # Temp folder for cloning updated version
UPDATE_EXIT_CODE = 10                   # Special code that means "update requested"

def update_script():
    print("[WRAPPER] Pulling latest script from GitHub...")

    # 1. Clone repo into a temporary folder
    subprocess.run(["git", "clone", REPO_URL, CLONE_PATH, "--depth", "1"])

    # 2. Replace the current mqtt_server.py with the one from the new repo
    os.replace(f"{CLONE_PATH}/mqtt_server.py", SCRIPT_PATH)

    # 3. Clean up
    subprocess.run(["rm", "-rf", CLONE_PATH])
    print("[WRAPPER] Script updated.")

# Main loop
while True:
    print("[WRAPPER] Starting agent...")

    # Run the main script and wait for it to finish
    result = subprocess.run([sys.executable, SCRIPT_PATH])

    # Check the exit code
    if result.returncode == UPDATE_EXIT_CODE:
        update_script()
        print("[WRAPPER] Restarting agent after update...")
        time.sleep(2)
    else:
        # Exit if the agent exited normally or crashed
        print(f"[WRAPPER] Agent exited with code {result.returncode}. Exiting.")
        break
