import paho.mqtt.client as mqtt
import requests
import tempfile
import os
import subprocess
import threading
import json

# === CONFIGURATION ===
MQTT_BROKER = "localhost"
BASE_COMMAND_TOPIC = "home/mini_pc/command/"
BASE_RESPONSE_TOPIC = "home/mini_pc/response/"

BACKUP_SCRIPT_URL = "https://raw.githubusercontent.com/YourUsername/YourRepo/main/tools/backup.sh"
BACKUP_SCRIPT_LOCAL = "./backup.sh"
MARYTTS_URL = "http://localhost:59125/process"
DEFAULT_VOICE = "cmu-slt-hsmm"
BT_DEVICE_MAC = "XX:XX:XX:XX:XX:XX"  # Replace this with your soundbar's MAC

# === VERSION UPDATE ===
def handle_self_update():
    log("Update requested via MQTT")
    client.disconnect()
    exit(10)  # Special code to tell launcher to update

# === SYSTEM BACKUP ===
def publish_backup_status(status, log=None):
    mqtt_client.publish(f"{BASE_RESPONSE_TOPIC}backup/status", status)
    if log:
        mqtt_client.publish(f"{BASE_RESPONSE_TOPIC}backup/log", log)

def backup_thread():
    try:
        publish_backup_status("started")
        r = requests.get(BACKUP_SCRIPT_URL, timeout=15)
        if r.status_code != 200:
            publish_backup_status("failed", f"Download failed: HTTP {r.status_code}")
            return
        with open(BACKUP_SCRIPT_LOCAL, "wb") as f:
            f.write(r.content)
        os.chmod(BACKUP_SCRIPT_LOCAL, 0o755)  # Make executable
        publish_backup_status("running")
        proc = subprocess.run([BACKUP_SCRIPT_LOCAL], capture_output=True, text=True)
        if proc.returncode == 0:
            publish_backup_status("success", proc.stdout)
        else:
            publish_backup_status("failed", proc.stderr)
    except Exception as e:
        publish_backup_status("failed", str(e))

def handle_backup():
    thread = threading.Thread(target=backup_thread)
    thread.start()

# === LOGGING ===
def log(msg):
    print(f"[LOG] {msg}")

# === TEXT TO SPEACH ===
def generate_tts(text):
    params = {
        "INPUT_TYPE": "TEXT",
        "OUTPUT_TYPE": "AUDIO",
        "AUDIO": "WAVE_FILE",
        "LOCALE": "en_US",
        "INPUT_TEXT": text,
        "VOICE": DEFAULT_VOICE,
    }
    try:
        response = requests.get(MARYTTS_URL, params=params, timeout=5)
        if response.status_code == 200:
            tmpfile = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
            tmpfile.write(response.content)
            tmpfile.close()
            return tmpfile.name
        else:
            log(f"MaryTTS error: {response.status_code}")
    except Exception as e:
        log(f"Exception in TTS: {e}")
    return None

def play_audio(file_path):
    try:
        subprocess.run(["aplay", file_path])
    finally:
        os.remove(file_path)

def handle_tts(payload):
    log(f"TTS requested: {payload}")
    audio_file = generate_tts(payload)
    if audio_file:
        play_audio(audio_file)

# === BLUETOOTH ===
def btctl(command):
    full_command = f'echo -e "{command}" | bluetoothctl'
    return subprocess.run(full_command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def handle_bluetooth_connect():
    log("Connecting to Bluetooth soundbar...")
    btctl(f"connect {BT_DEVICE_MAC}")

def handle_bluetooth_disconnect():
    log("Disconnecting from Bluetooth soundbar...")
    btctl(f"disconnect {BT_DEVICE_MAC}")

# === MQTT FUNCTIONS ===
def on_connect(client, userdata, flags, rc):
    log(f"Connected to MQTT with result code {rc}")
    client.subscribe(f"{BASE_COMMAND_TOPIC}#")

def on_message(client, userdata, msg):
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
        log(f"Unknown command topic: {topic}")

def publish_sensor_discovery(
    mqtt_client,
    name,
    state_topic,
    device_name,
    availability_topic,
    icon=None,
    unit_of_measurement=None,
    device_class=None,
    state_class=None,
    entity_category=None,
    display_precision=None,
    device_model="MQTT Server",
    device_manufacturer="YourName"
):
    def remove_spaces(text):
        return text.replace(" ", "_").lower()

    unique_id = f"{device_name}-{remove_spaces(name)}"
    object_id = f"{device_name}_{remove_spaces(name)}"
    config_topic = f"homeassistant/sensor/{unique_id}/config"

    payload = {
        "name": name,
        "unique_id": unique_id,
        "object_id": object_id,
        "state_topic": state_topic,
        "availability_topic": availability_topic,
        "payload_available": "connected",
        "payload_not_available": "connection lost",
        "device": {
            "identifiers": [device_name],
            "name": device_name,
            "model": device_model,
            "manufacturer": device_manufacturer
        }
    }

    if icon: payload["icon"] = icon
    if unit_of_measurement: payload["unit_of_meas"] = unit_of_measurement
    if device_class: payload["device_class"] = device_class
    if state_class: payload["state_class"] = state_class
    if entity_category: payload["entity_category"] = entity_category
    if display_precision is not None: payload["suggested_display_precision"] = display_precision

    mqtt_client.publish(config_topic, json.dumps(payload), retain=True)

# === MQTT CLIENT ===
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

log("Starting MQTT Command Listener...")
client.connect(MQTT_BROKER, 1883, 60)
client.loop_forever()
