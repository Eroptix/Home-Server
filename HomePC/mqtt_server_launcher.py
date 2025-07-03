import subprocess
import time
import os
import sys
import shutil

# === CONFIGURATION ===
REPO_URL = "https://github.com/Eroptix/Home-Server.git"
SCRIPT_PATH = "./mqtt_server.py"               # Where the live script will run
CLONE_PATH = "./mqtt_repo_temp"                # Temporary clone directory
SCRIPT_IN_REPO = "HomePC/mqtt_server.py"       # Path to script inside repo
UPDATE_EXIT_CODE = 10                          # Used by agent to request update

def update_script():
    print("[LAUNCHER] Updating script from GitHub...")

    # 1. Clone repo into temp folder
    subprocess.run(["git", "clone", REPO_URL, CLONE_PATH, "--depth", "1"])

    # 2. Copy updated script to the main location
    source_script = os.path.join(CLONE_PATH, SCRIPT_IN_REPO)
    if os.path.exists(source_script):
        shutil.copyfile(source_script, SCRIPT_PATH)
        print("[LAUNCHER] Script updated successfully.")
    else:
        print(f"[ERROR] Cannot find {source_script}.")
        return

    # 3. Clean up cloned repo
    shutil.rmtree(CLONE_PATH)

def main_loop():
    while True:
        print("[LAUNCHER] Starting MQTT script...")
        result = subprocess.run([sys.executable, SCRIPT_PATH])

        if result.returncode == UPDATE_EXIT_CODE:
            print("[LAUNCHER] Script requested update. Pulling new version...")
            update_script()
            time.sleep(2)
        else:
            print(f"[LAUNCHER] Script exited with code {result.returncode}. Exiting.")
            break

if __name__ == "__main__":
    main_loop()
