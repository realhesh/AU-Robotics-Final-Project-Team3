#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
//#include "encoder.h"
#include <ESP32Encoder.h>
#include "MPU6050Yaw.h"
#include "Localization.h"

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
#define communication_priority 4

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

//place holder pins to avoid build errors
#define left_encoder_A 4
#define right_encoder_A 5
#define left_encoder_B 6
#define right_encoder_B 7

const char* ssid = "robot";
const char* password = "123456789";
const char* mqtt_server = "192.168.4.2"; //takes ip address for local broker or public broker
const int mqtt_port = 1883;
const char* mqtt_sub = "AUR/controls";
const char* mqtt_pub = "AUR/localization";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

volatile unsigned long right_ticks = 0;
volatile unsigned long left_ticks = 0; // kont 7attah gwa el task beta3 el sensors bas task el encoder haye7tago 3ashan ye increment number of ticks
// speeds
int leftspeed = 0;
int rightspeed = 0;
int vertical = 0;
int horizontal = 0;

typedef struct
{
    float x = 0.0, y = 0.0, heading = 0.0;
} localization_data;

ESP32Encoder leftEncoder;
ESP32Encoder rightEncoder;

    void localization_task(void *pvParameters)
{
    Localization localization(wheel_radius, wheel_base, right_ticks_per_rev, left_ticks_per_rev);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    int32_t right_ticks = 0, left_ticks = 0; // keep last known values
    float heading = 0.0;
    localization_data data;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

        // Try to get new IMU data (non-blocking)
        float new_heading;
        if (xQueueReceive(imu_queue, &new_heading, 0) == pdPASS) {
            heading = new_heading;
        }

        // Try to get new encoder ticks (non-blocking)
        int32_t left_delta, right_delta;
        if (xQueueReceive(left_encoder_queue, &left_delta, 0) == pdPASS) {
            left_ticks += left_delta;
        }
        if (xQueueReceive(right_encoder_queue, &right_delta, 0) == pdPASS) {
            right_ticks += right_delta;
        }

        // Update localization
        localization.update(right_ticks, left_ticks, heading);

        data.x = localization.getX();
        data.y = localization.getY();
        data.heading = heading;

        // Send updated localization to queue (non-blocking)
        xQueueSend(localization_queue, &data, 0);
    }
}


/*void sensor_reading_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    int left_t = left_ticks;
    int right_t = right_ticks;

    float heading = 0.0;

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

        // sensor readings//fadel imu
        left_ticks = left.getTicks();
        right_ticks = right.getTicks();

        xQueueSend(imu_queue, &heading, pdMS_TO_TICKS(5)); // idk max tick count 3ala asas eh
        xQueueSend(left_encoder_queue, &left_t, pdMS_TO_TICKS(5));
        xQueueSend(right_encoder_queue, &right_t, pdMS_TO_TICKS(5));
    }
}*/

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

void set_servo(int angle, int channel) {
  uint32_t duty = map(angle, 0, 180,
                      (uint32_t)(65535 * 0.025),
                      (uint32_t)(65535 * 0.12));
  ledcWrite(channel, duty);
}

void motion_control_task(void *pvParameters)
{

    int left_speed = 0;
    int right_speed = 0;
    float vertical = 0;
    float horizontal = 0;
    while (1)
    {
        xQueueReceive(left_motor_queue, &left_speed, 0);
        xQueueReceive(right_motor_queue, &right_speed, 0);
        xQueueReceive(vertical_servo_queue, &vertical, 0);
        xQueueReceive(horizontal_servo_queue, &horizontal, 0);

        // write on motors
        set_motor(left_speed, EN_left, IN1_left, IN2_left);
        set_motor(right_speed, EN_right, IN1_right, IN2_right);
        set_servo(vertical, 0);
        set_servo(horizontal, 1);

        vTaskDelay(pdTICKS_TO_MS(20));
    }
}

void user_input(char* topic, byte* payload, unsigned int length) {
    char msg[100];
    memcpy(msg, payload, length);
    msg[length] = '\0';
    Serial.println(msg);

    int left_motor, right_motor, vertical_ang, horizontal_ang;
    if (sscanf(msg, "%d %d %d %d", &left_motor, &right_motor, &vertical_ang, &horizontal_ang) == 4) {
        xQueueSendFromISR(left_motor_queue, &left_motor, NULL);
        xQueueSendFromISR(right_motor_queue, &right_motor, NULL);
        xQueueSendFromISR(vertical_servo_queue, &vertical_ang, NULL);
        xQueueSendFromISR(horizontal_servo_queue, &horizontal_ang, NULL);
    }
}

void communication_task(void *pvParameters)
{
    localization_data pub_data;

    while (1) {
    if (xQueueReceive(localization_queue, &pub_data, pdMS_TO_TICKS(5)) == pdPASS) {
        char msg[32];
        snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f", pub_data.x, pub_data.y, pub_data.heading);
        mqttClient.publish(mqtt_pub, msg);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // allow other tasks & feed WDT
}
}

void encoder_reading_task(void *pvParameters) {
  int32_t lastLeftCount = 0;
  int32_t lastRightCount = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));

    int32_t leftCount = leftEncoder.getCount();
    int32_t rightCount = rightEncoder.getCount();

    int32_t leftDelta = leftCount - lastLeftCount;
    int32_t rightDelta = rightCount - lastRightCount;

    lastLeftCount = leftCount;
    lastRightCount = rightCount;

    xQueueSend(left_encoder_queue, &leftDelta, 0);
    xQueueSend(right_encoder_queue, &rightDelta, 0);
  }
}

void mqtt_loop_task(void *pvParameters) {
    while (true) {
        mqttClient.loop();          // Keep MQTT client alive
        vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to yield to other tasks
    }
}

void reconnect_task(void *pvParameters)
{
    while (true)
    {
        if (!mqttClient.connected())
        {
            Serial.println("Attempting MQTT connection...");
            if (mqttClient.connect("ESP32Client"))
            {
                Serial.println("Connected to MQTT broker");
                mqttClient.subscribe(mqtt_sub); // subscribe after connecting
            }
            else
            {
                Serial.println("MQTT connection failed, will retry in 2s");
            }
        }

        // Delay inside the loop to avoid WDT reset
        vTaskDelay(pdMS_TO_TICKS(2000)); // 2-second delay between attempts
    }
}


void setup()
{
    Serial.begin(115200);

    WiFi.softAP(ssid, password);

//ESP32Encoder::useInternalWeakPullResistors = ESP32Encoder::puType::UP;
  // Setup encoders
    leftEncoder.attachSingleEdge(left_encoder_A, left_encoder_B);
    rightEncoder.attachSingleEdge(right_encoder_A, right_encoder_B);

  // Reset counts
    leftEncoder.setCount(0);
    rightEncoder.setCount(0);

    ledcAttach(VS, 50, 16);
    ledcAttach(HS, 50, 16);

    pinMode(EN_left, OUTPUT);
    pinMode(EN_right, OUTPUT);
    pinMode(IN1_left, OUTPUT);
    pinMode(IN2_left, OUTPUT);
    pinMode(IN1_right, OUTPUT);
    pinMode(IN2_right, OUTPUT);

    mqttClient.setServer(mqtt_server, mqtt_port);
    mqttClient.setCallback(user_input);

    left_encoder_queue = xQueueCreate(10, sizeof(int32_t));
    right_encoder_queue = xQueueCreate(10, sizeof(int32_t));
    imu_queue = xQueueCreate(10, sizeof(float));
    left_motor_queue = xQueueCreate(10, sizeof(int));
    right_motor_queue = xQueueCreate(10, sizeof(int));
    vertical_servo_queue = xQueueCreate(10, sizeof(float));
    horizontal_servo_queue = xQueueCreate(10, sizeof(float));
    localization_queue = xQueueCreate(10, sizeof(localization_data));

    xTaskCreatePinnedToCore(localization_task, "localization", 1024, NULL, localization_priority, NULL, 1);
    xTaskCreatePinnedToCore(encoder_reading_task, "sensor_readings", 1024, NULL, sensor_priority, NULL, 1);
    xTaskCreatePinnedToCore(mqtt_loop_task, "MQTT Loop", 2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(motion_control_task, "motion_control", 1024, NULL, motion_priority, NULL, 0);
    xTaskCreatePinnedToCore(communication_task, "communication", 1024, NULL, communication_priority, NULL, 0);
    xTaskCreatePinnedToCore(reconnect_task, "reconnect", 1024, NULL, emergency_priority, NULL, 0);

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
}