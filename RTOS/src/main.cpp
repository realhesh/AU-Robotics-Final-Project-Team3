#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include "header.h"
#include "MPU9250.h"
#include "localization.h"

/* Tasks
emergency reset
localization
sensor reading
Motion control
communication
*/

QueueHandle_t left_encoder_queue;
QueueHandle_t right_encoder_queue;
QueueHandle_t imu_queue;
QueueHandle_t left_motor_queue;
QueueHandle_t right_motor_queue;
QueueHandle_t vertical_servo_queue;
QueueHandle_t horizontal_servo_queue;
QueueHandle_t localization_queue;
// QueueHandle_t communication_queue; // mesh fakra 3amalt el queue dah leeh bas 8aleban 3ashan el mode el hanakhdo men el console
// shakl mafeesh amal 8eer fy el manual :(

#define emergency_priority 5
#define localization_priority 4
#define sensor_priority 3
#define motion_priority 3
//#define communication_priority 4

#define wheel_radius 10
#define wheel_base 25
#define right_ticks_per_rev 40
#define left_ticks_per_rev 40

#define EN_left 15
#define EN_right 23
#define IN1_left 17
#define IN1_right 22
#define IN2_left 16
#define IN2_right 21
#define HS 33
#define VS 32

const char* ssid = "robot";
const char* password = "123456789";
const char* mqtt_server = "192.168.4.2"; //takes ip address for local broker or public broker
const int mqtt_port = 1883;
const char* mqtt_sub = "AUR/controls";
const char* mqtt_pub = "AUR/localization";

Servo horizontal_servo;
Servo vertical_servo;


WiFiClient espClient;
PubSubClient mqttClient(espClient);

int right_ticks = 0, left_ticks = 0; // kont 7attah gwa el task beta3 el sensors bas task el encoder haye7tago 3ashan ye increment number of ticks
// speeds
int leftspeed = 0;
int rightspeed = 0;
int vertical = 0;
int horizontal = 0;

typedef struct
{
    float x = 0.0, y = 0.0, heading = 0.0;
} localization_data;

void localization_task(void *pvParameters)
{
    localization localization(wheel_radius, wheel_base, right_ticks_per_rev, left_ticks_per_rev);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int right_ticks, left_ticks;
    float heading = 0.0;
    localization_data data;
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
        // read queue from encoder, and imu
        xQueueReceive(imu_queue, &heading, pdMS_TO_TICKS(5));
        xQueueReceive(left_encoder_queue, &left_ticks, pdMS_TO_TICKS(5));
        xQueueReceive(right_encoder_queue, &right_ticks, pdMS_TO_TICKS(5));

        localization.update(right_ticks, left_ticks, heading);
        data.x = localization.getX();
        data.y = localization.getY();
        data.heading = heading;
        xQueueSend(localization_queue, &data, 0);
    }
}

void sensor_reading_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

    // sensors initialise//fadel imu
    encoder left(left_encoder_A, left_encoder_B);
    encoder right(right_encoder_A, right_encoder_B);

    float heading = 0.0;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

        // sensor readings//fadel imu
        left_ticks = left.getTicks();
        right_ticks = right.getTicks();

        xQueueSend(imu_queue, &heading, pdMS_TO_TICKS(5)); // idk max tick count 3ala asas eh
        xQueueSend(left_encoder_queue, &left_ticks, pdMS_TO_TICKS(5));
        xQueueSend(right_encoder_queue, &right_ticks, pdMS_TO_TICKS(5));
    }
}

void set_motor(float power, int en, int in1, int in2) {
    if (power > 35) {
        analogWrite(en, power);
        digitalWrite(in1, HIGH);
        digitalWrite(in2, LOW);
    }
    else if (power < -35) {
        analogWrite(en, -power);
        digitalWrite(in1, LOW);
        digitalWrite(in2, HIGH);
    }
    else {
        analogWrite(en, 0);
    }
}

void motion_control_task(void *pvParameters)
{

    int left_speed = 0;
    int right_speed = 0;
    int vertical = 0;
    int horizontal = 0;
    while (1)
    {
        xQueueReceive(left_motor_queue, &left_speed, pdMS_TO_TICKS(5));
        xQueueReceive(right_motor_queue, &right_speed, pdMS_TO_TICKS(5));
        xQueueReceive(vertical_servo_queue, &vertical, pdMS_TO_TICKS(5));
        xQueueReceive(horizontal_servo_queue, &horizontal, pdMS_TO_TICKS(5));

        // write on motors
        set_motor(left_speed, EN_left, IN1_left, IN2_left);
        set_motor(right_speed, EN_right, IN1_right, IN2_right);
        horizontal_servo.write(horizontal);
        vertical_servo.write(vertical);
    }
}

void user_input(char* topic, byte* payload, unsigned int length) {
    char msg[100];
    memcpy(msg, payload, length);
    msg[length] = '\0';

    int left_motor, right_motor, vertical_ang, horizontal_ang;
    if (sscanf(msg, "%d %d %d %d", &left_motor, &right_motor, &vertical_ang, &horizontal_ang) == 4) {
        xQueueSendFromISR(left_motor_queue, &left_motor, NULL);
        xQueueSendFromISR(right_motor_queue, &right_motor, NULL);
        xQueueSendFromISR(vertical_servo_queue, &vertical_ang, NULL);
        xQueueSendFromISR(horizontal_servo_queue, &horizontal_ang, NULL);
    }
}

/*
void communication_task(void *pvParameters)
{
    nt left_speed = 0;
    int right_speed = 0;
    int vertival = 0;
    int horizontal = 0;

    while (1)
    {
        /*
        read mqtt
        write on motion queue
        read localization queue
        send localization data
        

        xQueueSend(left_motor_queue, &leftspeed, pdMS_TO_TICKS(5));
        xQueueSend(right_motor_queue, &rightspeed, pdMS_TO_TICKS(5));
        xQueueSend(vertical_servo_queue, &vertical, pdMS_TO_TICKS(5));
        xQueueSend(horizontal_servo_queue, &horizontal, pdMS_TO_TICKS(5));
    }
}
*/
void reconnect_task(void *pvParameters)
{
    while (true) {
        if (!mqttClient.connected()) {
            if (mqttClient.connect("ESP32Client")) {
                Serial.println("Connected");
                mqttClient.subscribe(mqtt_sub);
            } else {
                Serial.println("MQTT connection failed");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


void left_encoder_task()
{
    left_ticks++;
}

void right_encoder_task()
{
    right_ticks++;
}
void setup()
{
    WiFi.softAP(ssid, password);
//    while (WiFi.status() != WL_CONNECTED) {
//        delay(250);
//    }
    Serial.println("wifi connected");

    pinMode(EN_left, OUTPUT);
    pinMode(EN_right, OUTPUT);
    pinMode(IN1_left, OUTPUT);
    pinMode(IN2_left, OUTPUT);
    pinMode(IN1_right, OUTPUT);
    pinMode(IN2_right, OUTPUT);

    horizontal_servo.attach(HS);
    vertical_servo.attach(VS);

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(user_input);

    left_encoder_queue = xQueueCreate(10, sizeof(unsigned long));
    right_encoder_queue = xQueueCreate(10, sizeof(unsigned long));
    imu_queue = xQueueCreate(10, sizeof(float));
    left_motor_queue = xQueueCreate(10, sizeof(int));
    right_motor_queue = xQueueCreate(10, sizeof(int));
    vertical_servo_queue = xQueueCreate(10, sizeof(int));
    horizontal_servo_queue = xQueueCreate(10, sizeof(int));
    localization_queue = xQueueCreate(10, sizeof(localization_data));

    xTaskCreate(localization_task, "localization", 1024, NULL, localization_priority, NULL);
    xTaskCreate(sensor_reading_task, "sensor_readings", 1024, NULL, sensor_priority, NULL);
    xTaskCreate(motion_control_task, "motion_control", 1024, NULL, motion_priority, NULL);
    //xTaskCreate(communication_task, "communication", 1024, NULL, communication_priority, NULL);
    xTaskCreate(reconnect_task, "reset", 1024, NULL, emergency_priority, NULL);

    attachInterrupt(digitalPinToInterrupt(left_encoder_A), left_encoder_task, RISING);
    attachInterrupt(digitalPinToInterrupt(right_encoder_A), right_encoder_task, RISING);

    /*
    el ana fahmah en el mafrood a3mel interrupt 3ala pin el dakhelaha output A le kol encoder
    w lama ye7sal interrupt a3mel increment le number of ticks
    bas mesh 3arfa ezay a access ticks fy task mesh taba3ha
    ya3ny el ticks ma3molaha declare inside sensors task
    ezay a access w a8ayarha???

    */
}

void loop()
{
    mqttClient.loop();
}