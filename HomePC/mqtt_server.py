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
import random

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
client = None 

# Device information
DEVICE_NAME = "homeserver"
CURRENT_SW_VERSION = "1.2.6"
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
STATUS_HOST_TOPIC =                         f"home/{DEVICE_NAME}/status/host"
STATUS_OS_TOPIC =                           f"home/{DEVICE_NAME}/status/os"
STATUS_CPU_TEMP_TOPIC =                     f"home/{DEVICE_NAME}/status/cpu/temperature"
STATUS_LOAD_1MIN_TOPIC =                    f"home/{DEVICE_NAME}/status/cpu/load/1min"
STATUS_LOAD_5MIN_TOPIC =                    f"home/{DEVICE_NAME}/status/cpu/load/5min"
STATUS_LOAD_15MIN_TOPIC =                   f"home/{DEVICE_NAME}/status/cpu/load/15min"
PLAY_ALBUM_TOPIC =                          f"home/{DEVICE_NAME}/music/play_album"

STATUS_MEDIA_STORAGE_TOTAL_TOPIC =          f"home/{DEVICE_NAME}/status/storage/media/total"
STATUS_MEDIA_STORAGE_USED_TOPIC =           f"home/{DEVICE_NAME}/status/storage/media/used"
STATUS_MEDIA_STORAGE_FREE_TOPIC =           f"home/{DEVICE_NAME}/status/storage/media/free"
STATUS_MEDIA_STORAGE_PERCENTAGE_TOPIC =     f"home/{DEVICE_NAME}/status/storage/media/percentage"

STATUS_SECURE_STORAGE_TOTAL_TOPIC =         f"home/{DEVICE_NAME}/status/storage/secure/total"
STATUS_SECURE_STORAGE_USED_TOPIC =          f"home/{DEVICE_NAME}/status/storage/secure/used"
STATUS_SECURE_STORAGE_FREE_TOPIC =          f"home/{DEVICE_NAME}/status/storage/secure/free"
STATUS_SECURE_STORAGE_PERCENTAGE_TOPIC =    f"home/{DEVICE_NAME}/status/storage/secure/percentage"

STATUS_BACKUP_STORAGE_TOTAL_TOPIC =         f"home/{DEVICE_NAME}/status/storage/backup/total"
STATUS_BACKUP_STORAGE_USED_TOPIC =          f"home/{DEVICE_NAME}/status/storage/backup/used"
STATUS_BACKUP_STORAGE_FREE_TOPIC =          f"home/{DEVICE_NAME}/status/storage/backup/free"
STATUS_BACKUP_STORAGE_PERCENTAGE_TOPIC =    f"home/{DEVICE_NAME}/status/storage/backup/percentage"

STATUS_INTERNAL_STORAGE_TOTAL_TOPIC =       f"home/{DEVICE_NAME}/status/storage/internal/total"
STATUS_INTERNAL_STORAGE_USED_TOPIC =        f"home/{DEVICE_NAME}/status/storage/internal/used"
STATUS_INTERNAL_STORAGE_FREE_TOPIC =        f"home/{DEVICE_NAME}/status/storage/internal/free"
STATUS_INTERNAL_STORAGE_PERCENTAGE_TOPIC =  f"home/{DEVICE_NAME}/status/storage/internal/percentage"

STATUS_ROOT_STORAGE_TOTAL_TOPIC =           f"home/{DEVICE_NAME}/status/storage/root/total"
STATUS_ROOT_STORAGE_USED_TOPIC =            f"home/{DEVICE_NAME}/status/storage/root/used"
STATUS_ROOT_STORAGE_FREE_TOPIC =            f"home/{DEVICE_NAME}/status/storage/root/free"
STATUS_ROOT_STORAGE_PERCENTAGE_TOPIC =      f"home/{DEVICE_NAME}/status/storage/root/percentage"

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


def publish_secure_drive_status(path="/mnt/secure"):
    global client
    try:
        usage = psutil.disk_usage(path)
        client.publish(STATUS_SECURE_STORAGE_TOTAL_TOPIC, round(usage.total / (1024**3), 1), retain=False)
        client.publish(STATUS_SECURE_STORAGE_USED_TOPIC, round(usage.used / (1024**3), 1), retain=False)
        client.publish(STATUS_SECURE_STORAGE_FREE_TOPIC, round(usage.free / (1024**3), 1), retain=False)
        client.publish(STATUS_SECURE_STORAGE_PERCENTAGE_TOPIC, usage.percent, retain=False)
    except Exception as e:
        client.publish(LOG_ERROR_TOPIC, str(e), retain=False)


def publish_media_drive_status(path="/mnt/media"):
    global client
    try:
        usage = psutil.disk_usage(path)
        client.publish(STATUS_MEDIA_STORAGE_TOTAL_TOPIC, round(usage.total / (1024**3), 1), retain=False)
        client.publish(STATUS_MEDIA_STORAGE_USED_TOPIC, round(usage.used / (1024**3), 1), retain=False)
        client.publish(STATUS_MEDIA_STORAGE_FREE_TOPIC, round(usage.free / (1024**3), 1), retain=False)
        client.publish(STATUS_MEDIA_STORAGE_PERCENTAGE_TOPIC, usage.percent, retain=False)
    except Exception as e:
        client.publish(LOG_ERROR_TOPIC, str(e), retain=False)


def publish_backup_drive_status(path="/mnt/backup"):
    global client
    try:
        usage = psutil.disk_usage(path)
        client.publish(STATUS_BACKUP_STORAGE_TOTAL_TOPIC, round(usage.total / (1024**3), 1), retain=False)
        client.publish(STATUS_BACKUP_STORAGE_USED_TOPIC, round(usage.used / (1024**3), 1), retain=False)
        client.publish(STATUS_BACKUP_STORAGE_FREE_TOPIC, round(usage.free / (1024**3), 1), retain=False)
        client.publish(STATUS_BACKUP_STORAGE_PERCENTAGE_TOPIC, usage.percent, retain=False)
    except Exception as e:
        client.publish(LOG_ERROR_TOPIC, str(e), retain=False)


def publish_root_drive_status(path="/"):
    global client
    try:
        usage = psutil.disk_usage(path)
        client.publish(STATUS_ROOT_STORAGE_TOTAL_TOPIC, round(usage.total / (1024**3), 1), retain=False)
        client.publish(STATUS_ROOT_STORAGE_USED_TOPIC, round(usage.used / (1024**3), 1), retain=False)
        client.publish(STATUS_ROOT_STORAGE_FREE_TOPIC, round(usage.free / (1024**3), 1), retain=False)
        client.publish(STATUS_ROOT_STORAGE_PERCENTAGE_TOPIC, usage.percent, retain=False)
    except Exception as e:
        client.publish(LOG_ERROR_TOPIC, str(e), retain=False)


def collect_system_status():
    """Return a dictionary with various system status parameters"""
    status = {}

    # CPU
    load1, load5, load15 = os.getloadavg()
    status["cpu_load_1min"] = round(load1, 2)
    status["cpu_load_5min"] = round(load5, 2)
    status["cpu_load_15min"] = round(load15, 2)

    client.publish(STATUS_CPU_TEMP_TOPIC, get_cpu_temperature(), retain=False)


    # Memory
    mem = psutil.virtual_memory()
    status["memory_total_mb"] = round(mem.total / 1024 / 1024, 1)
    status["memory_used_mb"] = round(mem.used / 1024 / 1024, 1)
    status["memory_percent"] = mem.percent

    # Network / System Info
    client.publish(STATUS_IP_TOPIC, get_ip_address(), retain=False)
    client.publish(STATUS_UPTIME_TOPIC, get_uptime(), retain=False)
    client.publish(STATUS_OS_TOPIC, platform.platform(), retain=False)
    client.publish(STATUS_HOST_TOPIC, socket.gethostname(), retain=False)

    return status


# === HOME ASSISTANT MQTT ===
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


def publish_mqtt_availability_binary_sensor_discovery(name, state_topic, icon="", device_class="connectivity",
                                         entity_category=""):
    """Create availability binary sensor discovery payload"""
    unique_id = f"{DEVICE_NAME}-{remove_spaces(name)}"
    object_id = f"{DEVICE_NAME}_{remove_spaces(name)}"
    config_topic = f"homeassistant/binary_sensor/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "state_topic": state_topic,
        "payload_on": "connected",
        "payload_off": "connection lost",
        "availability_topic": AVAILABILITY_TOPIC,
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
    publish_mqtt_sensor_discovery("IP Address", STATUS_IP_TOPIC, icon="mdi:bluetooth", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Host Name", STATUS_HOST_TOPIC, icon="mdi:bluetooth", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Uptime", STATUS_UPTIME_TOPIC, icon="mdi:bluetooth", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Platform", STATUS_OS_TOPIC, icon="mdi:bluetooth", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("CPU Temp", STATUS_CPU_TEMP_TOPIC, icon="mdi:bluetooth", entity_category="diagnostic")

    publish_mqtt_sensor_discovery("Secure Free", STATUS_SECURE_STORAGE_FREE_TOPIC, icon="mdi:harddisk-plus", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Secure Used", STATUS_SECURE_STORAGE_USED_TOPIC, icon="mdi:harddisk-remove", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Secure Total", STATUS_SECURE_STORAGE_TOTAL_TOPIC, icon="mdi:harddisk", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Secure Percentage", STATUS_SECURE_STORAGE_PERCENTAGE_TOPIC, icon="mdi:percent-outline", entity_category="diagnostic", unit_of_measurement="%", display_precision=0)

    publish_mqtt_sensor_discovery("Media Free", STATUS_MEDIA_STORAGE_FREE_TOPIC, icon="mdi:harddisk-plus", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Media Used", STATUS_MEDIA_STORAGE_USED_TOPIC, icon="mdi:harddisk-remove", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Media Total", STATUS_MEDIA_STORAGE_TOTAL_TOPIC, icon="mdi:harddisk", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Media Percentage", STATUS_MEDIA_STORAGE_PERCENTAGE_TOPIC, icon="mdi:percent-outline", entity_category="diagnostic", unit_of_measurement="%", display_precision=0)

    publish_mqtt_sensor_discovery("Backup Free", STATUS_BACKUP_STORAGE_FREE_TOPIC, icon="mdi:harddisk-plus", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Backup Used", STATUS_BACKUP_STORAGE_USED_TOPIC, icon="mdi:harddisk-remove", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Backup Total", STATUS_BACKUP_STORAGE_TOTAL_TOPIC, icon="mdi:harddisk", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Backup Percentage", STATUS_BACKUP_STORAGE_PERCENTAGE_TOPIC, icon="mdi:percent-outline", entity_category="diagnostic", unit_of_measurement="%", display_precision=0)
    
    publish_mqtt_sensor_discovery("Root Free", STATUS_ROOT_STORAGE_FREE_TOPIC, icon="mdi:harddisk-plus", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Root Used", STATUS_ROOT_STORAGE_USED_TOPIC, icon="mdi:harddisk-remove", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Root Total", STATUS_ROOT_STORAGE_TOTAL_TOPIC, icon="mdi:harddisk", entity_category="diagnostic", unit_of_measurement="GB", display_precision=0)
    publish_mqtt_sensor_discovery("Root Percentage", STATUS_ROOT_STORAGE_PERCENTAGE_TOPIC, icon="mdi:percent-outline", entity_category="diagnostic", unit_of_measurement="%", display_precision=0)

    # Binary Sensors
    publish_mqtt_availability_binary_sensor_discovery("MQTT Server Status", AVAILABILITY_TOPIC, icon="mdi:server", device_class="connectivity")

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


# === AUDIO/MUSIC ===
def play_audio(file_path, cleanup=True):
    """Play audio file and optionally clean up"""
    try:
        # Kill any existing ffplay processes
        stop_existing_audio()

        # Set up environment variables for audio playback
        env = os.environ.copy()
        env["XDG_RUNTIME_DIR"] = "/run/user/1000"
        env["DISPLAY"] = ":0"

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


def stop_existing_audio():
    """Stop all existing ffplay audio playback processes"""
    for proc in psutil.process_iter(['name', 'cmdline']):
        try:
            if 'ffplay' in proc.info['name'] or (
                proc.info['cmdline'] and 'ffplay' in proc.info['cmdline'][0]
            ):
                proc.terminate()  # Or use proc.kill() for immediate stop
                proc.wait(timeout=3)
                log("Stopped existing ffplay process", "info")
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.TimeoutExpired):
            pass


def select_random_song(folder_path):
    """
    Select a random .mp3 file from a folder.
    """
    if not os.path.exists(folder_path):
        log(f"Folder not found: {folder_path}","error")
        return None

    try:
        # List only mp3 files
        mp3_files = [f for f in os.listdir(folder_path) if f.lower().endswith(".mp3")]
        if not mp3_files:
            log(f"No mp3 files found in: {folder_path}","error")
            return None

        selected = random.choice(mp3_files)
        selected_path = os.path.join(folder_path, selected)
        log(f"Selected song: {selected_path}")
        return selected_path

    except Exception as e:
        log(f"Error reading folder {folder_path}: {e}","error")
        return None


def play_album(folder_path):
    """Play a random track from the album after Bluetooth is connected."""
    log(f"Starting album playback from: {folder_path}")

    # Select random song
    song_path = select_random_song(folder_path)
    if not song_path:
        log("No valid song selected. Aborting playback.", "error")
        return

    # Connect to Bluetooth soundbar
    handle_bluetooth_connect()

    # Optional extra wait (Bluetooth can be slow to fully connect audio sink)
    log("Waiting for Bluetooth audio to be ready...")
    for i in range(10):
        status = get_bt_connection_status()
        if status == "connected":
            break
        time.sleep(1)
    else:
        log("Bluetooth did not connect after 10 seconds", "error")
        return

    # Play selected audio
    play_audio(song_path, cleanup=False)


# === BLUETOOTH ===
def handle_bluetooth_connect():
    """Connect to Bluetooth device using D-Bus via busctl"""
    log("Connecting to Bluetooth soundbar...")

    device_path = f"/org/bluez/hci0/dev_{BT_SOUNDBAR_MAC.replace(':', '_')}"
    try:
        result = subprocess.run(
            ['busctl', 'call', 'org.bluez', device_path, 'org.bluez.Device1', 'Connect'],
            capture_output=True,
            text=True,
            timeout=10
        )

        log(f"D-Bus connect result:\n{result.stdout.strip()}", "info")
        if result.stderr.strip():
            log(f"D-Bus error:\n{result.stderr.strip()}", "error")

    except Exception as e:
        log(f"D-Bus connect command failed: {e}", "error")
        return

    time.sleep(5)

    status = get_bt_connection_status()
    client.publish(BLUETOOTH_STATUS_TOPIC, status, retain=True)

    if status == "connected":
        log("Bluetooth device connected successfully")
    else:
        log("Bluetooth connection failed or not established", "error")


def handle_bluetooth_disconnect():
    """Disconnect from Bluetooth device using D-Bus"""
    log("Disconnecting from Bluetooth soundbar...")

    device_path = f"/org/bluez/hci0/dev_{BT_SOUNDBAR_MAC.replace(':', '_')}"
    try:
        result = subprocess.run(
            ['busctl', 'call', 'org.bluez', device_path, 'org.bluez.Device1', 'Disconnect'],
            capture_output=True,
            text=True,
            timeout=10
        )

        log(f"D-Bus disconnect result:\n{result.stdout.strip()}", "info")
        if result.stderr.strip():
            log(f"D-Bus error:\n{result.stderr.strip()}", "error")

    except Exception as e:
        log(f"D-Bus disconnect command failed: {e}", "error")
        return

    time.sleep(5)

    status = get_bt_connection_status()
    client.publish(BLUETOOTH_STATUS_TOPIC, status, retain=True)

    if status == "not connected":
        log("Bluetooth disconnected successfully")
    else:
        log("Unexpected status after disconnect: still connected", "warning")


def get_bt_connection_status(mac_address=None):
    """
    Returns "connected" if the specified MAC address is connected.
    If mac_address is None, uses BT_SOUNDBAR_MAC.
    """
    if not mac_address:
        mac_address = BT_SOUNDBAR_MAC

    device_path = f"/org/bluez/hci0/dev_{mac_address.replace(':', '_')}"

    try:
        result = subprocess.run(
            ['busctl', 'get-property', 'org.bluez', device_path, 'org.bluez.Device1', 'Connected'],
            capture_output=True,
            text=True,
            timeout=5
        )

        if "true" in result.stdout:
            return "connected"
    except Exception as e:
        print(f"Bluetooth status check error: {e}")

    return "not connected"


def status_monitor_loop(interval=30):
    last_status = None
    global client
    while True:

        # Publish availability
        client.publish(AVAILABILITY_TOPIC, "connected", retain=True)

        # Publish system status
        collect_system_status()

        # Publish drive status
        publish_secure_drive_status()
        publish_media_drive_status()
        publish_backup_drive_status()
        publish_root_drive_status()

        # Check bluetooth status
        status = get_bt_connection_status()
        if status != last_status:
            client.publish(BLUETOOTH_STATUS_TOPIC, status, retain=True)
            last_status = status

        # Initiate sleep    
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
        client.subscribe(PLAY_ALBUM_TOPIC)

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
        elif topic == PLAY_ALBUM_TOPIC:
            play_album(payload)
        elif topic == TAILSCALE_CONTROL_TOPIC:
            handle_tailscale_control(payload)
        else:
            log(f"Unknown command topic: {topic}", "warning")

    except Exception as e:
        log(f"Error processing message: {e}", "error")


def handle_tailscale_control(payload):
    try:
        data = json.loads(payload)
        action = data.get("action")
        target = data.get("target")

        if action != "expose":
            log(f"Unknown Tailscale action: {action}", "warning")
            return

        # Stop any running serve config
        subprocess.run(["sudo", "tailscale", "serve", "--shutdown"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        if target == "bitwarden":
            subprocess.run(["sudo", "tailscale", "serve", "https", "/", "http://127.0.0.1:4080"], check=True)
            log("✅ Bitwarden exposed via Tailscale")
        elif target == "homeassistant":
            subprocess.run(["sudo", "tailscale", "serve", "https", "/", "http://127.0.0.1:8123"], check=True)
            log("✅ Home Assistant exposed via Tailscale")
        else:
            log(f"⚠️ Unknown target: {target}", "warning")

    except json.JSONDecodeError:
        log(f"Invalid JSON payload: {payload}", "error")
    except subprocess.CalledProcessError as e:
        log(f"Tailscale command failed: {e}", "error")
    except Exception as e:
        log(f"Error handling Tailscale control: {e}", "error")

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
        threading.Thread(target=status_monitor_loop, daemon=True).start()

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