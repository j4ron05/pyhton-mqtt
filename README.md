# SmartCar Telemetry System 🚗

A basic telemetry system using an **ESP32** to send sensor data to a **Raspberry Pi** via MQTT. The Raspberry Pi logs the data into a MySQL database.

---

## 🔧 Components

- **ESP32 (Arduino code)**  
  → Publishes sensor values over MQTT.

- **Raspberry Pi (Python code)**  
  → Listens to MQTT and stores the data in MySQL.

---

## 🌐 Setup

### 1. Create a `.env` file

Add your credentials like this:

MQTT

MQTT_BROKER=your_broker
MQTT_PORT=1883
MQTT_USERNAME=your_username
MQTT_PASSWORD=your_password
Database

DB_HOST=localhost
DB_USER=your_user
DB_PASSWORD=your_password
DB_NAME=smartcar
