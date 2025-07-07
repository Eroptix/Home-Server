import paho.mqtt.client as mqtt
import requests
import tempfile
import os
import subprocess
import threading
import json
import logging
from logging.handlers import RotatingFileHandler

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
CURRENT_SW_VERSION = "1.0.0"
DEVICE_MODEL = "Home PC Server"
DEVICE_MANUFACTURER = "BTM Engineering"

# Dynamic topic construction
BASE_COMMAND_TOPIC = f"home/{DEVICE_NAME}/command/"
BASE_RESPONSE_TOPIC = f"home/{DEVICE_NAME}/response/"
PARAMETER_REQUEST_TOPIC = f"home/{DEVICE_NAME}/parameters/request"
AVAILABILITY_TOPIC = f"home/{DEVICE_NAME}/available"
PARAMETER_RESPONSE_TOPIC = f"home/{DEVICE_NAME}/parameters"

# Backup script 
BACKUP_SCRIPT_URL = "https://raw.githubusercontent.com/YourUsername/YourRepo/main/tools/backup.sh"
BACKUP_SCRIPT_LOCAL = "./backup.sh"

# TTS settings
MARYTTS_URL = "http://localhost:59125/process"
DEFAULT_VOICE = "cmu-slt-hsmm"

# Bluetooth device addresses
BT_SOUNDBAR_MAC = "XX:XX:XX:XX:XX:XX"


# === UTILITY FUNCTIONS ===
def remove_spaces(text):
    """Remove spaces and convert to lowercase for entity IDs"""
    return text.replace(" ", "_").lower()

def log(msg, level="info"):
    """Unified logger wrapper"""
    if level == "info":
        logging.info(msg)
    elif level == "error":
        logging.error(msg)
    elif level == "warning":
        logging.warning(msg)
    elif level == "debug":
        logging.debug(msg)
    else:
        logging.info(msg)
     
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
    publish_mqtt_button_discovery("Update Script", f"{BASE_COMMAND_TOPIC}update",
                                  icon="mdi:update", optimistic=True)
    publish_mqtt_button_discovery("Run Backup", f"{BASE_COMMAND_TOPIC}backup",
                                  icon="mdi:backup-restore", optimistic=True)
    publish_mqtt_button_discovery("Connect Bluetooth", f"{BASE_COMMAND_TOPIC}bluetooth/connect",
                                  icon="mdi:bluetooth-connect", optimistic=True)
    publish_mqtt_button_discovery("Disconnect Bluetooth", f"{BASE_COMMAND_TOPIC}bluetooth/disconnect",
                                  icon="mdi:bluetooth-off", optimistic=True)

    # Sensors
    publish_mqtt_sensor_discovery("Backup Status", f"{BASE_RESPONSE_TOPIC}backup/status",
                                  icon="mdi:backup-restore", entity_category="diagnostic")
    publish_mqtt_sensor_discovery("Backup Log", f"{BASE_RESPONSE_TOPIC}backup/log",
                                  icon="mdi:text-box-outline", entity_category="diagnostic")

    # Binary Sensors
    publish_mqtt_binary_sensor_discovery("MQTT Server Status", AVAILABILITY_TOPIC,
                                         icon="mdi:server", device_class="connectivity")

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
    client.publish(f"{BASE_RESPONSE_TOPIC}backup/status", status, retain=True)
    if log_msg:
        client.publish(f"{BASE_RESPONSE_TOPIC}backup/log", log_msg, retain=True)


def backup_thread():
    """Run backup in separate thread"""
    try:
        publish_backup_status("started")
        log("Downloading backup script")

        r = requests.get(BACKUP_SCRIPT_URL, timeout=15)
        if r.status_code != 200:
            error_msg = f"Download failed: HTTP {r.status_code}"
            log(f"Download failed: HTTP {r.status_code}", "error")
            publish_backup_status("failed", error_msg)
            return

        with open(BACKUP_SCRIPT_LOCAL, "wb") as f:
            f.write(r.content)
        os.chmod(BACKUP_SCRIPT_LOCAL, 0o755)  # Make executable

        publish_backup_status("running")
        log("Running backup script")

        proc = subprocess.run([BACKUP_SCRIPT_LOCAL], capture_output=True, text=True)

        if proc.returncode == 0:
            publish_backup_status("success", proc.stdout)
            log("Backup completed successfully")
        else:
            publish_backup_status("failed", proc.stderr)
            log(f"Backup failed: {proc.stderr}", "error")

    except Exception as e:
        error_msg = str(e)
        publish_backup_status("failed", error_msg)
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
            return tmpfile.name
        else:
            log(f"MaryTTS error: {response.status_code}","error")
    except Exception as e:
        log(f"Exception in TTS: {e}","error")

    return None


def play_audio(file_path):
    """Play audio file and clean up"""
    try:
        subprocess.run(["aplay", file_path], check=True)
        log(f"Audio played: {file_path}")
    except subprocess.CalledProcessError as e:
        log(f"Audio playback failed: {e}","error")
    finally:
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
        play_audio(audio_file)
    else:
        log("Failed to generate TTS audio", "error")


# === BLUETOOTH ===
def btctl(command):
    """Execute bluetoothctl command"""
    full_command = f'echo -e "{command}" | bluetoothctl'
    try:
        result = subprocess.run(full_command, shell=True, capture_output=True, text=True, timeout=10)
        return result
    except subprocess.TimeoutExpired:
        log("Bluetooth command timed out", "error")
        return None


def handle_bluetooth_connect():
    """Connect to Bluetooth device"""
    log("Connecting to Bluetooth soundbar")
    result = btctl(f"connect {BT_DEVICE_MAC}")
    if result:
        log(f"Bluetooth connect result: {result.stdout}", "error")


def handle_bluetooth_disconnect():
    """Disconnect from Bluetooth device"""
    log("Disconnecting from Bluetooth soundbar")
    result = btctl(f"disconnect {BT_DEVICE_MAC}")
    if result:
        log(f"Bluetooth disconnect result: {result.stdout}", "error")


# === MQTT FUNCTIONS ===
def on_connect(client, userdata, flags, rc):
    """Callback for MQTT connection"""
    if rc == 0:
        log("Connected to MQTT broker")
        client.subscribe(f"{BASE_COMMAND_TOPIC}#")

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

        if topic == f"{BASE_COMMAND_TOPIC}tts":
            handle_tts(payload)
        elif topic == f"{BASE_COMMAND_TOPIC}bluetooth/connect":
            handle_bluetooth_connect()
        elif topic == f"{BASE_COMMAND_TOPIC}bluetooth/disconnect":
            handle_bluetooth_disconnect()
        elif topic == f"{BASE_COMMAND_TOPIC}update":
            handle_self_update()
        elif topic == f"{BASE_COMMAND_TOPIC}backup":
            handle_backup()
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