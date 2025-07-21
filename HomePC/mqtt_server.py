import paho.mqtt.client as mqtt
import requests
import tempfile
import os
import subprocess
import threading
import json
import logging
from logging.handlers import RotatingFileHandler
import psutil
import platform
import socket
import shutil
import time

# === Logging Setup with Rotation ===
LOG_DIR = "./logs"
LOG_FILE = os.path.join(LOG_DIR, "mqtt_server.log")
os.makedirs(LOG_DIR, exist_ok=True)

# Set up rotating handler
rotating_handler = RotatingFileHandler(
    LOG_FILE,
    maxBytes=1 * 1024 * 1024,  # 1 MB
    backupCount=5
)

# Format logs
formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
rotating_handler.setFormatter(formatter)

# Add console handler
console_handler = logging.StreamHandler()
console_handler.setFormatter(formatter)

# Configure root logger
logging.basicConfig(
    level=logging.INFO,
    handlers=[rotating_handler, console_handler]
)

# MQTT settings
MQTT_BROKER = "192.168.0.241"
MQTT_PORT = 1783

# Device information
DEVICE_NAME = "homeserver"
CURRENT_SW_VERSION = "1.1.4"
DEVICE_MODEL = "Home PC Server"
DEVICE_MANUFACTURER = "BTM Engineering"

# MQTT topics
BACKUP_RUN_TOPIC =                          f"home/{DEVICE_NAME}/backup/run"
BACKUP_STATUS_TOPIC =                       f"home/{DEVICE_NAME}/backup/status"
BACKUP_LOG_TOPIC =                          f"home/{DEVICE_NAME}/backup/log"
AVAILABILITY_TOPIC =                        f"home/{DEVICE_NAME}/available"
BLUETOOTH_STATUS_TOPIC =                    f"home/{DEVICE_NAME}/bluetooth/status"
BLUETOOTH_CONNECT_TOPIC =                   f"home/{DEVICE_NAME}/bluetooth/connect"
BLUETOOTH_DISCONNECT_TOPIC =                f"home/{DEVICE_NAME}/bluetooth/disconnect"
UPDATE_TOPIC =                              f"home/{DEVICE_NAME}/update"
TTS_TOPIC =                                 f"home/{DEVICE_NAME}/tts"
AUDIO_TOPIC =                               f"home/{DEVICE_NAME}/audio"
LOG_ERROR_TOPIC =                           f"home/{DEVICE_NAME}/log/error"
LOG_WARNING_TOPIC =                         f"home/{DEVICE_NAME}/log/warning"
LOG_INFO_TOPIC =                            f"home/{DEVICE_NAME}/log/info"
STATUS_UPTIME_TOPIC =                       f"home/{DEVICE_NAME}/status/uptime"
STATUS_VERSION_TOPIC =                      f"home/{DEVICE_NAME}/status/version"
STATUS_IP_TOPIC =                           f"home/{DEVICE_NAME}/status/ip"
STATUS_CPU_TEMP_TOPIC =                     f"home/{DEVICE_NAME}/status/cpu/temperature"
STATUS_LOAD_1MIN_TOPIC =                    f"home/{DEVICE_NAME}/status/cpu/load/1min"
STATUS_LOAD_5MIN_TOPIC =                    f"home/{DEVICE_NAME}/status/cpu/load/5min"
STATUS_LOAD_15MIN_TOPIC =                   f"home/{DEVICE_NAME}/status/cpu/load/15min"
STATUS_DISK_TOTAL_TOPIC =                   f"home/{DEVICE_NAME}/status/disk/total"
STATUS_DISK_USED_TOPIC =                    f"home/{DEVICE_NAME}/status/disk/used"
STATUS_DISK_FREE_TOPIC =                    f"home/{DEVICE_NAME}/status/disk/free"
STATUS_DISK_PERCENTAGE_TOPIC =              f"home/{DEVICE_NAME}/status/disk/percentage"
STATUS_EXTERNAL_TOTAL_TOPIC =               f"home/{DEVICE_NAME}/status/external/total"
STATUS_EXTERNAL_USED_TOPIC =                f"home/{DEVICE_NAME}/status/external/used"
STATUS_EXTERNAL_FREE_TOPIC =                f"home/{DEVICE_NAME}/status/external/free"
STATUS_EXTERNAL_PERCENTAGE_TOPIC =          f"home/{DEVICE_NAME}/status/external/percentage"
STATUS_MEMORY_TOTAL_TOPIC =                 f"home/{DEVICE_NAME}/status/memory/total"
STATUS_MEMORY_USED_TOPIC =                  f"home/{DEVICE_NAME}/status/memory/used"
STATUS_MEMORY_PERCENTAGE_TOPIC =            f"home/{DEVICE_NAME}/status/memory/percentage"

# Backup script
BACKUP_SCRIPT_URL = "https://raw.githubusercontent.com/Eroptix/Home-Server/refs/heads/main/HomePC/backup_script.sh"
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
BACKUP_SCRIPT_LOCAL = os.path.join(BASE_DIR, "backup.sh")

# TTS settings
MARYTTS_URL = "http://192.168.0.241:59125/process"
DEFAULT_VOICE = "cmu-slt-hsmm"

# Bluetooth device addresses
BT_SOUNDBAR_MAC = "68:52:10:57:03:58"


# === UTILITY FUNCTIONS ===
def remove_spaces(text):
    """Remove spaces and convert to lowercase for entity IDs"""
    return text.replace(" ", "_").lower()


def log(msg, level="info"):
    """Unified logger wrapper"""
    if level == "info":
        logging.info(msg)
        client.publish(LOG_INFO_TOPIC, msg, retain=False)
    elif level == "error":
        logging.error(msg)
        client.publish(LOG_ERROR_TOPIC, msg, retain=False)
    elif level == "warning":
        logging.warning(msg)
        client.publish(LOG_WARNING_TOPIC, msg, retain=False)
    elif level == "debug":
        logging.debug(msg)
    else:
        logging.info(msg)


def get_uptime():
    """Return system uptime as a string"""
    try:
        with open('/proc/uptime', 'r') as f:
            uptime_seconds = float(f.readline().split()[0])
            return str(timedelta(seconds=int(uptime_seconds)))
    except:
        return "unknown"


def get_ip_address():
    """Return the current IP address"""
    try:
        hostname = socket.gethostname()
        return socket.gethostbyname(hostname)
    except:
        return "unknown"


def get_cpu_temperature():
    """Try to read CPU temperature (Linux only)"""
    try:
        temp_paths = [
            "/sys/class/thermal/thermal_zone0/temp",
            "/sys/class/hwmon/hwmon0/temp1_input"
        ]
        for path in temp_paths:
            if os.path.exists(path):
                with open(path, "r") as f:
                    temp_raw = int(f.read().strip())
                    return round(temp_raw / 1000, 1)
    except:
        pass
    return None


def get_mounted_drives_info(exclude_roots=True):
    """Collect info about all mounted drives, excluding system volumes if needed"""
    external_drives = {}
    for part in psutil.disk_partitions(all=False):
        mountpoint = part.mountpoint
        fstype = part.fstype

        # Skip non-storage mounts
        if exclude_roots and (mountpoint.startswith("/boot") or mountpoint in ["/", "/snap", "/dev", "/proc", "/sys", "/run"]):
            continue
        if 'tmpfs' in part.opts or 'loop' in part.device:
            continue

        try:
            usage = psutil.disk_usage(mountpoint)
            external_drives[mountpoint] = {
                "total_gb": round(usage.total / (1024**3), 1),
                "used_gb": round(usage.used / (1024**3), 1),
                "free_gb": round(usage.free / (1024**3), 1),
                "percent_used": usage.percent
            }
        except Exception as e:
            external_drives[mountpoint] = {
                "error": str(e)
            }
    return external_drives


def publish_drive_status(path="/mnt/ssd", base_topic="home/homeserver/ssd"):
    try:
        usage = psutil.disk_usage(path)
        mqtt_client.publish(STATUS_EXTERNAL_TOTAL_TOPIC, round(usage.total / (1024**3), 1), retain=False)
        mqtt_client.publish(STATUS_EXTERNAL_USED_TOPIC, round(usage.used / (1024**3), 1), retain=False)
        mqtt_client.publish(STATUS_EXTERNAL_FREE_TOPIC, round(usage.free / (1024**3), 1), retain=False)
        mqtt_client.publish(STATUS_EXTERNAL_PERCENTAGE_TOPIC, usage.percent, retain=False)
    except Exception as e:
        mqtt_client.publish(LOG_ERROR_TOPIC, str(e), retain=False)


def collect_system_status():
    """Return a dictionary with various system status parameters"""
    status = {}

    # CPU
    load1, load5, load15 = os.getloadavg()
    status["cpu_load_1min"] = round(load1, 2)
    status["cpu_load_5min"] = round(load5, 2)
    status["cpu_load_15min"] = round(load15, 2)
    temp = get_cpu_temperature()
    status["cpu_temp"] = temp if temp is not None else "unavailable"

    # Memory
    mem = psutil.virtual_memory()
    status["memory_total_mb"] = round(mem.total / 1024 / 1024, 1)
    status["memory_used_mb"] = round(mem.used / 1024 / 1024, 1)
    status["memory_percent"] = mem.percent

    # Disk
    disk = shutil.disk_usage("/")
    status["disk_total_gb"] = round(disk.total / 1024 / 1024 / 1024, 1)
    status["disk_used_gb"] = round(disk.used / 1024 / 1024 / 1024, 1)
    status["disk_free_gb"] = round(disk.free / 1024 / 1024 / 1024, 1)
    status["disk_percent"] = round(disk.used / disk.total * 100, 1)

    # External Drives
    status["external_drives"] = get_mounted_drives_info()

    # Network / System Info
    status["ip_address"] = get_ip_address()
    status["uptime"] = get_uptime()
    status["os"] = platform.platform()
    status["hostname"] = socket.gethostname()

    return status

# === HOME ASSISTANT MQTT DISCOVERY FUNCTIONS ===
def publish_mqtt_sensor_discovery(name, state_topic, icon="", unit_of_measurement="",
                                  device_class="", state_class="", entity_category="",
                                  display_precision=None):
    """Create sensor discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/sensor/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "state_topic": state_topic,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    # Add optional parameters
    if icon: payload["icon"] = icon
    if unit_of_measurement: payload["unit_of_meas"] = unit_of_measurement
    if device_class: payload["device_class"] = device_class
    if state_class: payload["state_class"] = state_class
    if entity_category: payload["entity_category"] = entity_category
    if display_precision is not None: payload["suggested_display_precision"] = display_precision

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published sensor discovery: {name}")


def publish_mqtt_binary_sensor_discovery(name, state_topic, icon="", device_class="",
                                         entity_category=""):
    """Create binary sensor discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/binary_sensor/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "state_topic": state_topic,
        "payload_on": "ON",
        "payload_off": "OFF",
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    # Add optional parameters
    if icon: payload["icon"] = icon
    if device_class: payload["device_class"] = device_class
    if entity_category: payload["entity_category"] = entity_category

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published binary sensor discovery: {name}")


def publish_mqtt_switch_discovery(name, command_topic, state_topic, icon=""):
    """Create switch discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/switch/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "command_topic": command_topic,
        "state_topic": state_topic,
        "payload_on": "manual on",
        "payload_off": "manual off",
        "state_on": "ON",
        "state_off": "OFF",
        "retain": True,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    if icon: payload["icon"] = icon

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published switch discovery: {name}")


def publish_mqtt_select_discovery(name, command_topic, state_topic, options, icon=""):
    """Create select discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/select/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "command_topic": command_topic,
        "state_topic": state_topic,
        "options": options,
        "retain": True,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    if icon: payload["icon"] = icon

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published select discovery: {name}")


def publish_mqtt_button_discovery(name, command_topic, icon="", optimistic=False):
    """Create button discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/button/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "command_topic": command_topic,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    if optimistic: payload["optimistic"] = True
    if icon: payload["icon"] = icon

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published button discovery: {name}")


def publish_mqtt_number_discovery(name, command_topic, state_topic, min_value, max_value,
                                  step, icon="", unit="", optimistic=False):
    """Create number discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/number/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "command_topic": command_topic,
        "state_topic": state_topic,
        "min": min_value,
        "max": max_value,
        "step": step,
        "retain": True,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    if unit: payload["unit_of_meas"] = unit
    if icon: payload["icon"] = icon
    if optimistic: payload["optimistic"] = True

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published number discovery: {name}")


def publish_mqtt_climate_discovery(name, current_temp_topic, temp_state_topic,
                                   temp_command_topic, mode_state_topic, mode_command_topic):
    """Create climate discovery payload"""
    unique_id = f"climate-{DEVICE_NAME}"
    object_id = f"climate_{DEVICE_NAME}"
    config_topic = f"homeassistant/climate/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "availability_topic": AVAILABILITY_TOPIC,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "current_temperature_topic": current_temp_topic,
        "temperature_state_topic": temp_state_topic,
        "temperature_command_topic": temp_command_topic,
        "mode_state_topic": mode_state_topic,
        "mode_command_topic": mode_command_topic,
        "min_temp": 12,
        "max_temp": 30,
        "temp_step": 1.0,
        "precision": 0.1,
        "retain": True,
        "modes": ["off", "heat", "cool", "auto"],
        "device": {
            "identifiers": [DEVICE_NAME],
            "name": DEVICE_NAME,
            "model": DEVICE_MODEL,
            "manufacturer": DEVICE_MANUFACTURER
        }
    }

    client.publish(config_topic, json.dumps(payload), retain=True)
    log(f"Published climate discovery: {name}")


def setup_home_assistant_entities():
    """Setup all Home Assistant entities via MQTT discovery"""
    log("Setting up Home Assistant entities")

    # Buttons
    publish_mqtt_button_discovery("Update Script", UPDATE_TOPIC, icon="mdi:update", optimistic=True)
    publish_mqtt_button_discovery("Run Backup", BACKUP_RUN_TOPIC, icon="mdi:backup-restore", optimistic=True)
    publish_mqtt_button_discovery("Connect Bluetooth", BLUETOOTH_CONNECT_TOPIC, icon="mdi:bluetooth-connect", optimistic=True)
    publish_mqtt_button_discovery("Disconnect Bluetooth", BLUETOOTH_DISCONNECT_TOPIC, icon="mdi:bluetooth-off", optimistic=True)

    # Sensors
    publish_mqtt_sensor_discovery("Info Log", LOG_INFO_TOPIC, icon="mdi:information-outline", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Warning Log", LOG_WARNING_TOPIC, icon="mdi:shield-alert-outline", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Error Log", LOG_ERROR_TOPIC, icon="mdi:alert-circle-outline", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Backup Status", BACKUP_STATUS_TOPIC, icon="mdi:backup-restore", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Software Version", STATUS_VERSION_TOPIC, icon="mdi:text-box-outline", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Bluetooth Status", BLUETOOTH_STATUS_TOPIC, icon="mdi:bluetooth", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("SSD Free", STATUS_EXTERNAL_FREE_TOPIC, icon="mdi:harddisk-plus", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("SSD Used", STATUS_EXTERNAL_USED_TOPIC, icon="mdi:harddisk-remove", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("SSD Total", STATUS_EXTERNAL_TOTAL_TOPIC, icon="mdi:harddisk", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("SSD Percentage", STATUS_EXTERNAL_PERCENTAGE_TOPIC, icon="mdi:percent-outline", entity_category="diagnostic")

    # Binary Sensors
    publish_mqtt_binary_sensor_discovery("MQTT Server Status", AVAILABILITY_TOPIC, icon="mdi:server", device_class="connectivity")

    log("Home Assistant entities setup complete")


# === VERSION UPDATE ===
def handle_self_update():
    """Handle script update request"""
    log("Update requested via MQTT")
    client.disconnect()
    exit(10)  # Special code to tell launcher to update


# === SYSTEM BACKUP ===
def publish_backup_status(status, log_msg=None):
    """Publish backup status to MQTT"""
    client.publish(BACKUP_STATUS_TOPIC, status, retain=True)


def backup_thread():
    """Run backup in separate thread"""
    try:
        publish_backup_status("STARTED")
        log("Downloading backup script")

        r = requests.get(BACKUP_SCRIPT_URL, timeout=15)
        if r.status_code != 200:
            error_msg = f"Download failed: HTTP {r.status_code}"
            log(f"Download failed: HTTP {r.status_code}", "error")
            publish_backup_status("FAILED")
            return

        with open(BACKUP_SCRIPT_LOCAL, "wb") as f:
            f.write(r.content)

        # Fix line endings automatically to Unix style
        with open(BACKUP_SCRIPT_LOCAL, "r", encoding="utf-8") as f:
            content = f.read()

        content = content.replace('\r\n', '\n')

        with open(BACKUP_SCRIPT_LOCAL, "w", encoding="utf-8") as f:
            f.write(content)

        os.chmod(BACKUP_SCRIPT_LOCAL, 0o755)  # Make executable

        publish_backup_status("RUNNING")
        log(f"Running backup script at: {BACKUP_SCRIPT_LOCAL}")
        log(f"Current working directory: {os.getcwd()}")
        log(f"Script exists: {os.path.exists(BACKUP_SCRIPT_LOCAL)}")

        proc = subprocess.run(
            ['sudo', '-S', BACKUP_SCRIPT_LOCAL],
            input='bozso406\n',
            text=True,
            capture_output=True
        )

        if proc.returncode == 0:
            publish_backup_status("COMPLETED")
            log("Backup completed successfully")
        else:
            publish_backup_status("FAILED")
            log(f"Backup failed: {proc.stderr}", "error")

    except Exception as e:
        error_msg = str(e)
        publish_backup_status("FAILED")
        log(f"Backup exception: {error_msg}", "error")


def handle_backup():
    """Start backup in background thread"""
    thread = threading.Thread(target=backup_thread)
    thread.daemon = True
    thread.start()


# === TEXT TO SPEECH ===
def generate_tts(text):
    """Generate TTS audio file"""
    params = {
        "INPUT_TYPE": "TEXT",
        "OUTPUT_TYPE": "AUDIO",
        "AUDIO": "WAVE_FILE",
        "LOCALE": "en_US",
        "INPUT_TEXT": text,
        "VOICE": DEFAULT_VOICE,
    }
    log(f"Generating TTS audio file: {text}")

    try:
        response = requests.get(MARYTTS_URL, params=params, timeout=10)
        if response.status_code == 200:
            tmpfile = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
            tmpfile.write(response.content)
            tmpfile.close()
            log(f"TTS file created at {tmpfile.name}, size: {os.path.getsize(tmpfile.name)} bytes")
            return tmpfile.name
        else:
            log(f"MaryTTS error: {response.status_code}","error")
    except Exception as e:
        log(f"Exception in TTS: {e}","error")

    return None


def play_audio(file_path, cleanup=True):
    """Play audio file and optionally clean up"""
    try:
        # Set up environment variables for audio playback
        env = os.environ.copy()
        env["XDG_RUNTIME_DIR"] = "/run/user/1000"  # Adjust this if your UID is different
        env["DISPLAY"] = ":0"  # Often needed if PulseAudio or GUI context is used

        subprocess.Popen([
            "ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", file_path
        ], env=env)

        log(f"Audio played: {file_path}")
    except subprocess.CalledProcessError as e:
        log(f"Audio playback failed: {e}", "error")
    finally:
        if cleanup:
            try:
                os.remove(file_path)
            except OSError:
                pass


def handle_tts(payload):
    """Handle TTS request"""
    log(f"TTS requested: {payload}")
    if not payload.strip():
        log("Empty TTS payload, ignoring","warning")
        return

    audio_file = generate_tts(payload)
    if audio_file:
        play_audio(audio_file, False)
    else:
        log("Failed to generate TTS audio", "error")


# === BLUETOOTH ===
def btctl(command):
    """Execute bluetoothctl command safely using input"""
    try:
        result = subprocess.run(
            ["bluetoothctl"],
            input=command + "\n",
            capture_output=True,
            text=True,
            timeout=10
        )
        return result
    except subprocess.TimeoutExpired:
        log("Bluetooth command timed out", "error")
        return None


def handle_bluetooth_connect():
    """Connect to Bluetooth device"""
    log("Connecting to Bluetooth soundbar")
    result = btctl(f"connect {BT_SOUNDBAR_MAC}")
    if result:
        log(f"Bluetooth connect result:\n{result.stdout.strip()}", "info")
        if result.stderr:
            log(f"Bluetooth error:\n{result.stderr.strip()}", "error")

    time.sleep(5)  # Wait for connection to settle
    status = get_bt_connection_status()
    client.publish(BLUETOOTH_STATUS_TOPIC, status, retain=True)
    if status == "connected":
        log("Bluetooth device connected successfully")
    else:
        log("Bluetooth connection failed or not established", "warning")


def handle_bluetooth_disconnect():
    """Disconnect from Bluetooth device"""
    log("Disconnecting from Bluetooth soundbar")
    result = btctl(f"disconnect {BT_SOUNDBAR_MAC}")
    if result:
        log(f"Bluetooth disconnect result:\n{result.stdout.strip()}", "info")
        if result.stderr:
            log(f"Bluetooth error:\n{result.stderr.strip()}", "error")

    time.sleep(5)
    status = get_bt_connection_status()
    client.publish(BLUETOOTH_STATUS_TOPIC, status, retain=True)
    if status == "not connected":
        log("Bluetooth disconnected successfully")
    else:
        log("Unexpected status after disconnect: still connected", "warning")


def get_bt_connection_status():
    """
    Returns "connected" if any paired device is connected, otherwise "not connected".
    """
    try:
        result = subprocess.run(
            ['bluetoothctl', 'paired-devices'],
            capture_output=True,
            text=True,
            timeout=5
        )
        devices = result.stdout.strip().splitlines()
        if not devices:
            return "not connected"

        for line in devices:
            parts = line.strip().split()
            if len(parts) >= 2:
                mac = parts[1]
                info_result = subprocess.run(
                    ['bluetoothctl', 'info', mac],
                    capture_output=True,
                    text=True,
                    timeout=5
                )
                if "Connected: yes" in info_result.stdout:
                    return "connected"
    except Exception as e:
        log(f"Bluetooth status error: {e}", "error")

    return "not connected"


def bt_status_monitor_loop(interval=30):
    """Periodically check Bluetooth connection status and publish via MQTT."""
    last_status = None
    while True:
        publish_drive_status()
        status = get_bt_connection_status()
        if status != last_status:
            client.publish(BLUETOOTH_STATUS_TOPIC, status, retain=True)
            log(f"Bluetooth status updated: {status}")
            last_status = status
        time.sleep(interval)


# === MQTT FUNCTIONS ===
def on_connect(client, userdata, flags, rc):
    """Callback for MQTT connection"""
    if rc == 0:
        log("Connected to MQTT broker")
        client.subscribe(AUDIO_TOPIC)
        client.subscribe(UPDATE_TOPIC)
        client.subscribe(TTS_TOPIC)
        client.subscribe(BLUETOOTH_CONNECT_TOPIC)
        client.subscribe(BLUETOOTH_DISCONNECT_TOPIC)
        client.subscribe(BACKUP_RUN_TOPIC)

        # Publish availability
        client.publish(AVAILABILITY_TOPIC, "connected", retain=True)

        # Setup Home Assistant entities
        setup_home_assistant_entities()

        log("MQTT setup complete")
    else:
        log(f"Failed to connect to MQTT broker, result code {rc}", "error")


def on_disconnect(client, userdata, rc):
    """Callback for MQTT disconnection"""
    log(f"Disconnected from MQTT broker, result code {rc}", "error")


def on_message(client, userdata, msg):
    """Callback for MQTT message reception"""
    try:
        topic = msg.topic
        payload = msg.payload.decode().strip()
        log(f"Message received on {topic}: {payload}")

        if topic == TTS_TOPIC:
            handle_tts(payload)
        elif topic == BLUETOOTH_CONNECT_TOPIC:
            handle_bluetooth_connect()
        elif topic == BLUETOOTH_DISCONNECT_TOPIC:
            handle_bluetooth_disconnect()
        elif topic == UPDATE_TOPIC:
            handle_self_update()
        elif topic == BACKUP_RUN_TOPIC:
            handle_backup()
        elif topic == AUDIO_TOPIC:
            play_audio(payload, False)
        else:
            log(f"Unknown command topic: {topic}", "warning")

    except Exception as e:
        log(f"Error processing message: {e}", "error")


# === MAIN EXECUTION ===
def main():
    """Main function"""
    global client

    # Setup MQTT client
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    # Set will message for availability
    client.will_set(AVAILABILITY_TOPIC, "connection lost", retain=True)

    log("Starting MQTT Command Listener")
    log(f"Version: {CURRENT_SW_VERSION}")

    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.publish(STATUS_VERSION_TOPIC, CURRENT_SW_VERSION, retain=True)

        # Start background threads here
        threading.Thread(target=bt_status_monitor_loop, daemon=True).start()

        client.loop_forever()
    except KeyboardInterrupt:
        log("Shutting down")
        client.publish(AVAILABILITY_TOPIC, "connection lost", retain=True)
        client.disconnect()
    except Exception as e:
        log(f"Fatal error: {e}", "error")
        exit(1)


if __name__ == "__main__":
    main()