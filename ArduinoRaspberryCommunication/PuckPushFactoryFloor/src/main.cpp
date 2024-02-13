/* MQTT Subscriber */

#include <Arduino.h>
#include <Servo.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>

extern "C" {
  #include "freertos/FreeRTOS.h"
  #include "freertos/timers.h"
}

#define ONBOARD_LED 2

#define WIFI_SSID "DBL_group_22"
#define WIFI_PASSWORD "password123"

#define MQTT_BROKER IPAddress(192, 168, 8, 2)
#define MQTT_PORT 1883

#define NAME "group22"

const int SERVO_P1_PIN = 32;

const int PHOTO_S2_PIN = 35; 
const int S2_THRESHOLD = 50;

const int PHOTO_S3_PIN = 34;
Servo servoP1;

const int servoMoved = 60;

const int threshold = 50  ;

bool wasTaken1 = 0, wasTaken2 = 0, canProcess = 1;

int defaultS1, defaultS2;


int canIWork = 0;
bool reconnecting = false;
bool doNotReconnect = false;

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t a = mqttClient.subscribe("robots/allowed/group22", 1);
  uint16_t b = mqttClient.subscribe("robots/errors/group22", 1);

 
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  canIWork = false;
  if (WiFi.isConnected() && !doNotReconnect) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}



void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.print("received a message: ");
  canIWork = atoi(payload);
  Serial.println(atoi(payload));
  if (atoi(payload) == 1) {
    digitalWrite(ONBOARD_LED,HIGH);
  } else {
    digitalWrite(ONBOARD_LED,LOW);
  }
    
} 


void setup() {
  pinMode(ONBOARD_LED,OUTPUT);

  // Setup the servo's
  pinMode(SERVO_P1_PIN, OUTPUT);
  servoP1.attach(SERVO_P1_PIN);

  // Setup the phototransistor
  pinMode(PHOTO_S2_PIN, INPUT);
  
  pinMode(PHOTO_S3_PIN, INPUT);

  // Setup communication with the PiR180
  Serial.begin(9600);
  Serial.println("Ready");

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.setWill("disonnected", 1, false, NAME);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  connectToWifi(); 

}


void loop() {
  delay(100);
  // Read the phototransistor
  int photoS2 = analogRead(PHOTO_S2_PIN);
  
  int photoS3 = analogRead(PHOTO_S3_PIN);
  
  // Check if thee  default values are taken. If so, then if the 
  if(wasTaken1 == 1 && wasTaken2 == 1)
  {
    // if(abs(photoS2 - defaultS1) > threshold){
    //   Serial.print(photoS2);
    //   Serial.print(" ");
    //   Serial.print(defaultS1);
    //   Serial.print("\n");
    //   delay(750);
    //   photoS3 = analogRead(PHOTO_S3_PIN);
    if(photoS3 == 0 && canIWork == 1){
      Serial.print(photoS3);
      Serial.print(" ");
      Serial.print(defaultS2);
      Serial.print("\n");
      delay(1000);
      servoP1.write(servoMoved);
      delay(2000);
      servoP1.write(0);
      canIwork  = 0;
    }
    else Serial.println(-1);
    
  }


  // // Check if there is a default value for the phototransistors
  if(wasTaken1 == 0 && photoS2>0 && photoS2 < 1300){
    defaultS1 = photoS2;
    wasTaken1 = 1;
  }
  if(wasTaken2 == 0 && photoS3>0 && photoS3 < 1300){
    defaultS2 = photoS3;
    wasTaken2 = 1;
  }
  // Send the phototransistor value to the Pi
  Serial.print("S1: ");
  Serial.println(photoS2);
  Serial.print("S2: "); 
  Serial.println(photoS3);


  while (Serial.available() > 0) {
    // Read the serial input
    char input = Serial.read();

    // Check if the input is P2
    if (input == 'P') {
      // Read the angle
      int angle = Serial.parseInt();
      Serial.print("P2: ");
      Serial.println(angle);

      // Set the angle of the servo
      servoP1.write(angle);
    }

    //Check if the input is a delay program
    else if (input == 'D'){
      int delayAmount = Serial.parseInt();
      Serial.print("D: ");
      Serial.println(delayAmount);
      delay(delayAmount);
    }
    else if (input == 'C'){
      canIWork = 1;
    }
  }

  // Wait for 0.01 second
}
