
import subprocess
import time
import os
import sys
import shutil
import requests
import zipfile
import tempfile
import logging
from logging.handlers import RotatingFileHandler
import os

# === CONFIGURATION ===
REPO_URL = "https://github.com/Eroptix/Home-Server.git"
REPO_ZIP_URL = "https://github.com/Eroptix/Home-Server/archive/refs/heads/main.zip"
SCRIPT_PATH = "./mqtt_server.py"               # Where the live script will run
CLONE_PATH = "./mqtt_repo_temp"                # Temporary clone directory
SCRIPT_IN_REPO = "HomePC/mqtt_server.py"       # Path to script inside repo
UPDATE_EXIT_CODE = 10                          # Used by agent to request update

# === LOGGING ===
LOG_DIR = "./logs"
LOG_FILE = os.path.join(LOG_DIR, "launcher.log")
os.makedirs(LOG_DIR, exist_ok=True)

handler = RotatingFileHandler(LOG_FILE, maxBytes=1_000_000, backupCount=3)
logging.basicConfig(
    handlers=[handler],
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s"
)

def check_git_available():
    """Check if git is available in PATH"""
    try:
        result = subprocess.run(["git", "--version"], capture_output=True, text=True)
        return result.returncode == 0
    except FileNotFoundError:
        return False


def update_script_via_git():
    """Update script using git clone"""
    logging.info("Updating script from GitHub using git")

    try:
        # Clean up any existing clone directory
        if os.path.exists(CLONE_PATH):
            shutil.rmtree(CLONE_PATH)

        # Clone repo into temp folder
        result = subprocess.run(["git", "clone", REPO_URL, CLONE_PATH, "--depth", "1"],
                                capture_output=True, text=True)

        if result.returncode != 0:
            logging.error(f"Git clone failed: {result.stderr}")
            return False

        # Copy updated script to the main location
        source_script = os.path.join(CLONE_PATH, SCRIPT_IN_REPO)
        if os.path.exists(source_script):
            shutil.copyfile(source_script, SCRIPT_PATH)
            logging.info("Script updated successfully via git.")
        else:
            logging.error(f"Cannot find {source_script} in repository.")
            return False

        # Clean up cloned repo
        shutil.rmtree(CLONE_PATH)
        return True

    except Exception as e:
        logging.error(f"Git update failed: {e}")
        return False


def update_script_via_zip():
    """Update script by downloading ZIP archive"""
    logging.info("Updating script from GitHub using ZIP download...")

    try:
        # Create temp directory for download
        with tempfile.TemporaryDirectory() as temp_dir:
            zip_path = os.path.join(temp_dir, "repo.zip")
            extract_path = os.path.join(temp_dir, "extracted")

            # Download ZIP file
            logging.info("Downloading repository ZIP...")
            response = requests.get(REPO_ZIP_URL, timeout=30)
            response.raise_for_status()

            with open(zip_path, 'wb') as f:
                f.write(response.content)

            # Extract ZIP file
            logging.info("Extracting ZIP file...")
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(extract_path)

            # Find the extracted folder (usually repo-name-main)
            extracted_folders = [f for f in os.listdir(extract_path)
                                 if os.path.isdir(os.path.join(extract_path, f))]

            if not extracted_folders:
                logging.error("No folders found in extracted ZIP")
                return False

            repo_folder = os.path.join(extract_path, extracted_folders[0])
            source_script = os.path.join(repo_folder, SCRIPT_IN_REPO)

            if os.path.exists(source_script):
                shutil.copyfile(source_script, SCRIPT_PATH)
                logging.info("Script updated successfully via ZIP download.")
                return True
            else:
                logging.error(f"Cannot find {source_script} in downloaded repository.")
                return False

    except requests.RequestException as e:
        logging.error(f"Failed to download repository: {e}")
        return False
    except zipfile.BadZipFile:
        logging.error("Downloaded file is not a valid ZIP archive")
        return False
    except Exception as e:
        logging.error(f"ZIP update failed: {e}")
        return False


def update_script():
    """Update script from GitHub - try git first, fallback to ZIP"""
    logging.info("Checking for Git availability...")

    if check_git_available():
        logging.info("Git found, using git clone method...")
        if update_script_via_git():
            return True
        else:
            logging.info("Git method failed, trying ZIP download...")
    else:
        logging.info("Git not found in PATH, using ZIP download method...")

    # Fallback to ZIP download
    return update_script_via_zip()


def main_loop():
    """Main execution loop"""
    while True:
        logging.info("Starting MQTT script...")

        # Check if script exists
        if not os.path.exists(SCRIPT_PATH):
            logging.info(f"Script {SCRIPT_PATH} not found!")
            logging.info("Attempting to download initial script...")
            if not update_script():
                logging.info("Failed to download script. Exiting.")
                break

        try:
            result = subprocess.run([sys.executable, SCRIPT_PATH])

            if result.returncode == UPDATE_EXIT_CODE:
                logging.info("Script requested update. Pulling new version...")
                if update_script():
                    logging.info("Update successful, restarting script...")
                    time.sleep(2)
                else:
                    logging.info("Update failed, restarting with current version...")
                    time.sleep(5)
            else:
                logging.info(f"Script exited with code {result.returncode}.")
                if result.returncode != 0:
                    logging.info("Script crashed, restarting in 10 seconds...")
                    time.sleep(10)
                else:
                    logging.info("Script exited normally. Stopping launcher.")
                    break

        except KeyboardInterrupt:
            logging.info("Launcher interrupted by user. Exiting.")
            break
        except Exception as e:
            logging.error(f"Error running script: {e}")
            logging.info("Retrying in 10 seconds...")
            time.sleep(10)


if __name__ == "__main__":
    logging.info("=" * 60)
    logging.info("MQTT Server Launcher Starting...")
    logging.info(f"Python version: {sys.version}")
    logging.info(f"Script path: {SCRIPT_PATH}")
    logging.info(f"Repository: {REPO_URL}")

    try:
        main_loop()
    except KeyboardInterrupt:
        logging.info("Launcher stopped by user.")
    except Exception as e:
        logging.error(f"Fatal error: {e}")
        sys.exit(1)

    logging.info("Launcher exiting.")
