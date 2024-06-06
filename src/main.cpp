#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <esp_http_server.h>

// Select camera model
#define CAMERA_MODEL_AI_THINKER

#include "camera_pins.h"

// Flash
#define LED_BUILTIN 4

// WIFI config
const char *ssid = "LAPTOP-INL7NVL4 4267";
const char *password = "023=26xC";

// MQTT config
bool useMQTT = true;
const char *mqttServer = "wouterpeetermans.com";
const char *HostName = "Photobooth_Scaleway";
const char *mqttUser = "your-device-id";
const char *mqttPassword = "";
const char *topic_PHOTO = "CM1/PICTURE";
const char *topic_PUBLISH = "CM1/CAM";
const char *topic_FLASH = "CM1/FLASH";
const int MAX_PAYLOAD = 60000;

bool flash;

WiFiClient espClient;
PubSubClient client(espClient);

void startCameraServer();
void callback(char* topic, byte* message, unsigned int length);
void take_picture();
void set_flash();
void sendMQTT(const uint8_t *buf, uint32_t len);

void setup() {
    // Define Flash as an output
    pinMode(LED_BUILTIN, OUTPUT);

    // Initialise the Serial Communication
    Serial.begin(115200);
    Serial.setDebugOutput(true);
    Serial.println();

    // Config Camera Settings
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

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s->id.PID == OV3660_PID) {
        s->set_vflip(s, 1);       // flip it back
        s->set_brightness(s, 1);  // up the brightness just a bit
        s->set_saturation(s, -2); // lower the saturation
    }
    s->set_framesize(s, FRAMESIZE_QVGA);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");

    startCameraServer();

    Serial.print("Camera Ready! Use 'http://");
    Serial.print(WiFi.localIP());
    Serial.println("' to connect");

    client.setServer(mqttServer, 1884);
    client.setBufferSize(MAX_PAYLOAD); // This is the maximum payload length
    client.setCallback(callback);
}

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
    if (flash) {
        digitalWrite(LED_BUILTIN, HIGH);
    };
    Serial.println("Taking picture");
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

void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect(HostName, mqttUser, mqttPassword)) {
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

void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();
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
