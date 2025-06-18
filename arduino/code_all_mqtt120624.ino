// Include libraries
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "TCA9548.h"
#include <OneWire.h>
#include <WiFiManager.h>
#include <MPU9250_WE.h>
#include <WiFi.h>            // Include the WiFi library
#include <PubSubClient.h>    // Include the PubSubClient library

#define MULTIPLEXER_ADDRESS 0x70
#define MPU6500_ADDR 0X68

MPU6500_WE myMPU6500 = MPU6500_WE(MPU6500_ADDR);
WiFiClient espClient;  // Create a WiFiClient object
PubSubClient client(espClient);

const char* mqttServer = "";
const int mqttPort = 1883;
const char* mqttUsername = "";
const char* mqttPassword = "";
String mqttTopic_RPM = String(mqttUsername) + "/smartCarT/RPM";
String mqttTopic_LeftFront = String(mqttUsername) + "/smartCarT/Left-Front";
String mqttTopic_LeftRear = String(mqttUsername) + "/smartCarT/Left-Rear";
String mqttTopic_RightFront = String(mqttUsername) + "/smartCarT/Right-Front";
String mqttTopic_RightRear = String(mqttUsername) + "/smartCarT/Right-Rear";
String mqttTopic_Motor = String(mqttUsername) + "/smartCarT/Motor";
String mqttTopic_Bewegingsrichting = String(mqttUsername) + "/smartCarT/Bewegingsrichting";
String mqttTopic_Gkracht = String(mqttUsername) + "/smartCarT/G-kracht";


volatile float rev = 0;   // 'volatile' because it is modified in an ISR
volatile bool rpmError = false; // Flag to indicate RPM calculation error
float rpm = 0;
unsigned long oldtime = 0;   // Using unsigned long to store millis() values
unsigned long Time;

// Create sensor objects
Adafruit_MLX90614 mlx_left_front;
Adafruit_MLX90614 mlx_right_front;
Adafruit_MLX90614 mlx_left_rear;
Adafruit_MLX90614 mlx_right_rear;
Adafruit_MLX90614 mlx_motor;

void isr() // Interrupt Service Routine
{
  if (!rpmError) {
    rev++;
  }
}

void selectMultiplexerChannel(uint8_t channel) {
  if (channel > 7) return; // Prevent selecting invalid channel
  Wire.beginTransmission(MULTIPLEXER_ADDRESS);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Function to publish sensor data to MQTT broker
void publishSensorData(float rpm, double leftFront, double rightFront, double leftRear, double rightRear, double motor, String direction, float gKracht) {
  client.loop();
  if (client.connected()) {
    client.publish(mqttTopic_RPM, String(rpm).c_str());

    // Correctly convert double to string for temperature values
    client.publish(mqttTopic_LeftFront, String(leftFront, 2).c_str());
    client.publish(mqttTopic_LeftRear, String(leftRear, 2).c_str());
    client.publish(mqttTopic_RightFront, String(rightFront, 2).c_str());
    client.publish(mqttTopic_RightRear, String(rightRear, 2).c_str());
    client.publish(mqttTopic_Motor, String(motor, 2).c_str());

    client.publish(mqttTopic_Bewegingsrichting, direction.c_str());
    client.publish(mqttTopic_Gkracht, String(gKracht, 2).c_str());
  } else {
    Serial.println("MQTT not connected. Reconnecting...");
    if (client.connect("WiFiclient98765432", mqttUsername, mqttPassword)) {
      Serial.println("Reconnected to MQTT broker");
    } else {
      Serial.println("Reconnection failed");
    }
  }
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  WiFiManager wifiManager;
  wifiManager.autoConnect("AutoConnectAPTim"); // Connect to WiFi using WiFiManager
 
  client.setServer(mqttServer, mqttPort);      // Set MQTT broker server and port
  if (client.connect("WiFiclient", mqttUsername, mqttPassword)) {
    Serial.println("Connected to MQTT broker");
  } else {
    Serial.println("Failed to connect to MQTT broker");
  }

  // Initialize the multiplexer
  selectMultiplexerChannel(0);

  // Initialize the MLX90614 sensors
  if (!mlx_left_front.begin()) {
    Serial.println("Error connecting to left front MLX90614 sensor.");
    while (1);
  }
  if (!mlx_right_front.begin()) {
    Serial.println("Error connecting to right front MLX90614 sensor.");
    while (1);
  }
  if (!mlx_left_rear.begin()) {
    Serial.println("Error connecting to left rear MLX90614 sensor.");
    while (1);
  }
  if (!mlx_right_rear.begin()) {
    Serial.println("Error connecting to right rear MLX90614 sensor.");
    while (1);
  }
  if (!mlx_motor.begin()) {
    Serial.println("Error connecting to motor MLX90614 sensor.");
    while (1);
  }
  if (!myMPU6500.init()){
    Serial.println("MPU6500 does not respond");
  }
  else{
    Serial.println("MPU6500 is connected");
  }
  Serial.println("Position you MPU6500 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6500.autoOffsets();
  Serial.println("Done!");
  myMPU6500.setSampleRateDivider(5);
  myMPU6500.enableAccDLPF(true);
  myMPU6500.setAccDLPF(MPU6500_DLPF_6);
  delay(200);
  attachInterrupt(digitalPinToInterrupt(2), isr, RISING);  // Attaching the interrupt to pin 2

  Serial.println("Setup complete.");
}

void loop() {
  detachInterrupt(digitalPinToInterrupt(2));  // Detach the interrupt
  
  unsigned long startTime = millis();
  while (millis() - startTime < 1000) {  // Measure RPM for 1 second
    if (!rpmError) {
      Time = millis() - oldtime;  // Calculate elapsed time
      rpm = (rev / Time) * 60000;  // Calculate RPM
      rev = 0;  // Reset revolutions counter
      oldtime = millis();  // Update old time
    } else {
      // Handle RPM calculation error
      // Example: Reset variables
      rev = 0;
      oldtime = millis();
      rpm = 0;  // Optional: Set RPM to a default value or mark as invalid
    }
  }
  
  attachInterrupt(digitalPinToInterrupt(2), isr, RISING);  // Reattach interrupt

  selectMultiplexerChannel(0);
  double objectTemp_right_rear = mlx_right_rear.readObjectTempC();

  selectMultiplexerChannel(1);
  double objectTemp_left_rear = mlx_left_rear.readObjectTempC();

  selectMultiplexerChannel(2);
  double objectTemp_left_front = mlx_left_front.readObjectTempC(); 

  selectMultiplexerChannel(3);
  double objectTemp_motor = mlx_motor.readObjectTempC(); 

  selectMultiplexerChannel(4);
  double objectTemp_right_front = mlx_right_front.readObjectTempC(); 

  selectMultiplexerChannel(5);
  xyzFloat gValue = myMPU6500.getGValues();
  float resultantG = myMPU6500.getResultantG(gValue);
  float restG = myMPU6500.getResultantG(gValue)-2.1;
  // Bereken de hoeken
  float angleX = atan2(-gValue.y, gValue.z) * RAD_TO_DEG;
  float angleY = atan2(gValue.x, sqrt(gValue.y * gValue.y + gValue.z * gValue.z)) * RAD_TO_DEG;

  // Bepaal de richting van beweging
  String direction;

  if (resultantG > 1.0) { // Stel de drempel in voor detectie van beweging
    if (angleX > 4 && angleX < 150) {
      if (angleY > 4 && angleY < 150) {
        direction = "Linksvoor";
      } else if (angleY < -4 && angleY > -150) {
        direction = "Linksachter";
      } else {
        direction = "Links";
      }
    } else if (angleX < -4 && angleX > -150) {
      if (angleY > 4 && angleY < 150) {
        direction = "Rechtsvoor";
      } else if (angleY < -4 && angleY > -150) {
        direction = "Rechtsachter";
      } else {
        direction = "Rechts";
      }
    } else {
      if (angleY > 4 && angleY < 150) {
        direction = "Vooruit";
      } else if (angleY < -4 && angleY > -150) {
        direction = "Achteruit";
      } else {
        direction = "Stilstaand";
      }
    }
  } else {
    direction = "Stilstaand";
  }

  Serial.print("RPM: ");        // Print RPM value
  Serial.println(rpm);

  Serial.print("Left Front: ");
  Serial.print(objectTemp_left_front);
  Serial.println(" C");

  Serial.print("Right Front: ");
  Serial.print(objectTemp_right_front);
  Serial.println(" C");

  Serial.print("Left Rear: ");
  Serial.print(objectTemp_left_rear);
  Serial.println(" C"); 

  Serial.print("Right Rear: ");
  Serial.print(objectTemp_right_rear);
  Serial.println(" C");

  Serial.print("Motor: ");
  Serial.print(objectTemp_motor);
  Serial.println(" C");

  Serial.println("Bewegingsrichting: " + direction);
  Serial.print("Resultant G-Kracht: ");
  Serial.println(restG);

  Serial.println("------------------------------------");

  // Call the function to publish sensor data
  publishSensorData(rpm, objectTemp_left_front, objectTemp_right_front, objectTemp_left_rear, objectTemp_right_rear, objectTemp_motor, direction, restG);

  delay(1000);
}
