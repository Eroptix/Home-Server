import paho.mqtt.client as mqtt
import requests
import tempfile
import os
import subprocess

# === CONFIGURATION ===
MQTT_BROKER = "localhost"
BASE_COMMAND_TOPIC = "mini_pc/command/"
BASE_RESPONSE_TOPIC = "mini_pc/response/"
MARYTTS_URL = "http://localhost:59125/process"
DEFAULT_VOICE = "cmu-slt-hsmm"
BT_DEVICE_MAC = "XX:XX:XX:XX:XX:XX"  # Replace this with your soundbar's MAC

# === VERSION UPDATE ===
def handle_self_update():
    log("Update requested via MQTT")
    client.disconnect()
    exit(10)  # Special code to tell launcher to update

# === LOGGING ===
def log(msg):
    print(f"[LOG] {msg}")

# === TTS ===
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

# === MQTT CALLBACKS ===
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
    else:
        log(f"Unknown command topic: {topic}")

# === MQTT CLIENT ===
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

log("Starting MQTT Command Listener...")
client.connect(MQTT_BROKER, 1883, 60)
client.loop_forever()
