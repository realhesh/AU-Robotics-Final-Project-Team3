#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>


#define EN_left 1
#define EN_right 2
#define IN1_left 3
#define IN1_right 4
#define IN2_left 5
#define IN2_right 6

const char* ssid = "ssid";
const char* password = "password";
const char* mqtt_server = "192.168.1.1"; //takes ip address for local broker or public broker
const int mqtt_port = 1883;
const char* mqtt_sub = "controls";
const char* mqtt_pub = "localization";

volatile float x = 0.0;
volatile float y = 0.0;
volatile float theta = 0.0;
volatile unsigned long last_millis = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

void reconnect();
void user_input(char* topic, byte* payload, unsigned int length);
void set_motor(float power, int en, int in1, int in2);

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
    }

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(user_input);
}

void loop() {
    if (!mqttClient.connected()) {
        reconnect();
    }
    mqttClient.loop();

    if (millis() - last_millis > 1000 ) {
        last_millis = millis();
        char msg[20];
        sprintf(msg, "%.2f,%.2f,%.2f", x, y, theta);
        mqttClient.publish(mqtt_pub, msg);
    }
}

void reconnect() {
    while (!mqttClient.connected()) {
        if (mqttClient.connect("ESP32Client", ssid, password)) {
            mqttClient.subscribe(mqtt_sub);
        }
    }
}

void user_input(char* topic, byte* payload, unsigned int length) {
    char msg[length + 1];
    memcpy(msg, payload, length);
    msg[length] = '\0';

    const char* left_ptr = strstr(msg, "left:");
    const char* right_ptr = strstr(msg, "right:");
    float right = strtof(right_ptr+6, NULL);
    float left = strtof(left_ptr+5, NULL);

    set_motor(left, EN_left, IN1_left, IN2_left);
    set_motor(right, EN_right, IN1_right, IN2_right);
}

void set_motor(float power, int en, int in1, int in2) {
    if (power > 0.15) {
        analogWrite(en, power * 255);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (power < -0.15) {
        analogWrite(en, -power * 255);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else {
        analogWrite(en, 0);
    }
}