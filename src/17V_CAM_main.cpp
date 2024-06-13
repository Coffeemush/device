#include "esp_camera.h"
//#include "SensorAliases.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include "SHT2x.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_http_server.h>

// WiFi credentials
const char* ssid = "LAPTOP-INL7NVL4 4267";
const char* password = "023=26xC";

// MQTT broker details
const char* mqtt_server = "wouterpeetermans.com";
const int mqtt_port = 1884;
const char* mqtt_user = "your_MQTT_USERNAME";
const char* mqtt_password = "your_MQTT_PASSWORD";

// Device identifier
const String deviceId = "CM1";

// Light and temperature thresholds
const float LIGHT_THRESHOLD = 300.0;
const float TEMPERATURE_THRESHOLD = 30.0;
const float HUMIDITY_LOW_THRESHOLD = 70.0;
const float HUMIDITY_HIGH_THRESHOLD = 80.0;

// Create sensor instances
SHT2x sht;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);

// Define pin numbers
const int switchPin = 13; // Water float switch
const int fanPin = 4;    // Fan (changed from 4 to 16)
const int valvePin = 12;  // Solenoid valve

// WiFi and MQTT clients
WiFiClient espClient;
PubSubClient client(espClient);

// Timing variables
unsigned long previousMillis = 0;
const long interval = 2000; // Interval for loop execution

bool tempRequested = false;

// Variables to track last activation time
unsigned long lastValveActivation = 0;
unsigned long lastFanActivation = 0;

// Variables for operation duration
unsigned long fanOnTime = 0;
unsigned long valveOnTime = 0;
const unsigned long operationDuration = 5000; // Operation duration in milliseconds (5 seconds)

bool fanRunning = false;
bool valveRunning = false;

// Camera configuration
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

// Flash
#define LED_BUILTIN 4

// MQTT topics
const char *topic_PHOTO = "CM1/PICTURE";
const char *topic_PUBLISH = "CM1/CAM";
const char *topic_FLASH = "CM1/FLASH";
const int MAX_PAYLOAD = 60000;

bool flash;

void startCameraServer();
void callback(char* topic, byte* message, unsigned int length);
void take_picture();
void set_flash();
void sendMQTT(const uint8_t *buf, uint32_t len);

void displayLightSensorDetails() {
    sensor_t sensor;
    tsl.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");
    Serial.println("------------------------------------");
    Serial.println("");
}

void configureLightSensor() {
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    Serial.println("------------------------------------");
    Serial.print("Gain:         "); Serial.println("Auto");
    Serial.print("Timing:       "); Serial.println("13 ms");
    Serial.println("------------------------------------");
}

void setup_wifi() {
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
            Serial.println("connected");
            client.subscribe(topic_PHOTO);
            client.subscribe(topic_FLASH);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

void readTemperature() {
    if (tempRequested && sht.reqTempReady()) {
        sht.readTemperature();
        float temperature = sht.getTemperature();
        Serial.print("TEMP:\t");
        Serial.print(temperature, 1);
        Serial.println(" °C");
        client.publish((deviceId + "/temperature").c_str(), String(temperature, 1).c_str());

        if (temperature > TEMPERATURE_THRESHOLD) {
            unsigned long currentMillis = millis();
            if (currentMillis - lastFanActivation >= 0.1*60000) { // 5 minutes
                digitalWrite(fanPin, HIGH); // Turn on the fan for ventilation
                fanOnTime = currentMillis;
                fanRunning = true;
                lastFanActivation = currentMillis; // Update the last fan activation time
            }
        }

        sht.requestHumidity();
        tempRequested = false;
    }
}

void readHumidity() {
    if (!tempRequested && sht.reqHumReady()) {
        sht.readHumidity();
        float humidity = sht.getHumidity();
        Serial.print("HUM:\t");
        Serial.print(humidity, 1);
        Serial.println(" %");
        client.publish((deviceId + "/humidity").c_str(), String(humidity, 1).c_str());

        if (humidity < HUMIDITY_LOW_THRESHOLD) {
            unsigned long currentMillis = millis();
            if (currentMillis - lastValveActivation >= (0.1*60000)) { // 5 minutes
                digitalWrite(valvePin, HIGH); // Open water valve
                valveOnTime = currentMillis;
                valveRunning = true;
                lastValveActivation = currentMillis; // Update the last valve activation time
            }
        } else if (humidity > HUMIDITY_HIGH_THRESHOLD) {
            digitalWrite(valvePin, LOW); // Ensure water valve is closed
            valveRunning = false;
        }
        
        sht.requestTemperature();
        tempRequested = true;
    }
}

void readLight() {
    sensors_event_t event;
    tsl.getEvent(&event);

    if (event.light) {
        float light = event.light;
        Serial.print("LIGHT:\t");
        Serial.print(light);
        Serial.println(" lux");
        client.publish((deviceId + "/light").c_str(), String(light).c_str());

        if (light > LIGHT_THRESHOLD) {
            client.publish((deviceId + "/light_warning").c_str(), "1");
        } else {
            client.publish((deviceId + "/light_warning").c_str(), "2");
        }
    } else {
        Serial.println("Sensor overload");
    }
}

void readSwitch() {
    int switchState = digitalRead(switchPin);
    if (switchState == LOW) {
        Serial.println("Water available!");
        client.publish((deviceId + "/float_switch").c_str(), "Water available!");
    } else {
        Serial.println("No water available!");
        client.publish((deviceId + "/float_switch").c_str(), "No water available!");
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println(__FILE__);
    Serial.print("SHT2x_LIB_VERSION: \t");
    Serial.println(SHT2x_LIB_VERSION);

    Wire.begin(14, 15); // Initialize I2C with SDA on GPIO 14 and SCL on GPIO 15

    sht.begin();
    uint8_t stat = sht.getStatus();
    Serial.print(stat, HEX);
    Serial.println();

    sht.requestTemperature();
    tempRequested = true;

    Serial.println("Light Sensor Test");
    Serial.println("");
    if (!tsl.begin()) {
        Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }

    pinMode(fanPin, OUTPUT);
    digitalWrite(fanPin, LOW);

    pinMode(valvePin, OUTPUT);
    digitalWrite(valvePin, LOW);

    displayLightSensorDetails();
    configureLightSensor();

    pinMode(switchPin, INPUT);

    setup_wifi();
    client.setServer(mqtt_server, mqtt_port);
    client.setBufferSize(MAX_PAYLOAD); // This is the maximum payload length
    client.setCallback(callback);

    // Camera setup
    pinMode(LED_BUILTIN, OUTPUT);

    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        config.frame_size = FRAMESIZE_UXGA;
        config.jpeg_quality = 10;
        config.fb_count = 2;
    } else {
        config.frame_size = FRAMESIZE_SVGA;
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    flash = true;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t_esp *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    s->set_framesize(s, FRAMESIZE_QVGA);

    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    Serial.println("");
}

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        readTemperature();
        readHumidity();
        readLight();
        readSwitch();
    }

    // Check if fan should be turned off
    if (fanRunning && (millis() - fanOnTime >= operationDuration)) {
        digitalWrite(fanPin, LOW); // Turn off the fan
        fanRunning = false;
    }

    // Check if valve should be turned off
    if (valveRunning && (millis() - valveOnTime >= operationDuration)) {
        digitalWrite(valvePin, LOW); // Turn off the valve
        valveRunning = false;
    }
}

// Camera-related functions
void callback(char* topic, byte* message, unsigned int length) {
    String messageTemp;
    Serial.println(topic);
    for (int i = 0; i < length; i++) {
        messageTemp += (char)message[i];
    }
    if (String(topic) == topic_PHOTO) {
        take_picture();
    }
    if (String(topic) == topic_FLASH) {
        set_flash();
    }
}

void take_picture() {
    camera_fb_t *fb = NULL;
    set_flash();   
    Serial.println("Taking picture...");
    fb = esp_camera_fb_get(); // used to get a single picture.
    if (!fb) {
        Serial.println("Camera capture failed");
        return;
    }
    Serial.println("Picture taken");
    digitalWrite(LED_BUILTIN, LOW);
    sendMQTT(fb->buf, fb->len);
    esp_camera_fb_return(fb); // must be used to free the memory allocated by esp_camera_fb_get().
}

void set_flash() {
    flash = !flash;
    Serial.print("Setting flash to ");
    Serial.println(flash);
    if (!flash) {
        for (int i = 0; i < 6; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }
    if (flash) {
        for (int i = 0; i < 3; i++) {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
    }
}

void sendMQTT(const uint8_t *buf, uint32_t len) {
    Serial.println("Sending picture...");
    if (len > MAX_PAYLOAD) {
        Serial.print("Picture too large, increase the MAX_PAYLOAD value");
    } else {
        Serial.print("Picture sent ? : ");
        Serial.println(client.publish(topic_PUBLISH, buf, len, false));
    }
}

// HTTP Server code
static esp_err_t index_handler(httpd_req_t *req) {
    httpd_resp_send(req, "ESP32 Camera Web Server", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

static const httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
};

void startCameraServer() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_register_uri_handler(server, &index_uri);
    }
}