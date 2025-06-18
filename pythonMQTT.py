import paho.mqtt.client as mqtt
import pymysql
import time
import os
from dotenv import load_dotenv

# === Load environment variables ===
load_dotenv()

# MQTT settings from .env
broker = os.getenv("MQTT_BROKER")
port = int(os.getenv("MQTT_PORT"))
client_name = os.getenv("MQTT_CLIENT_NAME")
username = os.getenv("MQTT_USERNAME")
password = os.getenv("MQTT_PASSWORD")

# Topics
topics = {
    "left_front": "tmellendijk/SmartCarT/Left-Front",
    "left_rear": "tmellendijk/SmartCarT/Left-Rear",
    "right_front": "tmellendijk/SmartCarT/Right-Front",
    "right_rear": "tmellendijk/SmartCarT/Right-Rear",
    "motor": "tmellendijk/SmartCarT/Motor",
    "speed": "tmellendijk/SmartCarT/RPM",
    "gforce": "tmellendijk/SmartCarT/G-kracht"
}

# Connect to MySQL using env vars
try:
    db = pymysql.connect(
        host=os.getenv("DB_HOST"),
        user=os.getenv("DB_USER"),
        password=os.getenv("DB_PASSWORD"),
        database=os.getenv("DB_NAME")
    )
    print("Successfully connected to the database.")
except pymysql.MySQLError as e:
    print(f"Error connecting to MySQL: {e}")

# Data structure to store sensor values
data = {key: None for key in topics}

# MQTT event handlers
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    if rc == 0:
        for topic in topics.values():
            client.subscribe(topic)
            print(f"Subscribed to topic: {topic}")
    else:
        print(f"Failed to connect with result code {rc}")

def on_message(client, userdata, msg):
    global data
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")
    cursor = db.cursor()
    try:
        for sensor_name, topic in topics.items():
            if msg.topic == topic:
                value = float(msg.payload.decode())
                data[sensor_name] = value
                print(f"Updated {sensor_name} with value {value}")

        if None not in data.values():
            print("All sensor data received, inserting into database...")
            cursor.execute(
                "INSERT INTO car1 (left_front, left_rear, right_front, right_rear, motor, speed, gforce) VALUES (%s, %s, %s, %s, %s, %s, %s)",
                (data["left_front"], data["left_rear"], data["right_front"], data["right_rear"], data["motor"], data["speed"], data["gforce"])
            )
            db.commit()
            print("Data inserted into database.")
            data = {key: None for key in data}

            cursor.execute("DELETE FROM smartcar.car1 ORDER BY id ASC LIMIT 1")
            db.commit()
            print("Line with the lowest ID deleted.")

    except Exception as e:
        print(f"Failed to execute MySQL: {e}")
    finally:
        cursor.close()

# Start MQTT client
client = mqtt.Client(client_name)
client.username_pw_set(username, password)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker, port, 60)

try:
    client.loop_forever()
except KeyboardInterrupt:
    db.close()
    print("Exiting...")
