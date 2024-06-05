#include <Arduino.h>
#include "WiFi.h"
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/rtc_io.h"
#include <SPIFFS.h>
#include <FS.h>
#include <PubSubClient.h>
#include <Wire.h>

const char *ssid = "LAPTOP-INL7NVL4 4267";
const char *password = "023=26xC";

const char *mqtt_server = "wouterpeetermans.com";
const int mqtt_port = 1884;
const char *mqtt_user = "YOUR_MQTT_USERNAME";     // if applicable
const char *mqtt_password = "YOUR_MQTT_PASSWORD"; // if applicable
const char *mqtt_topic = "cm/picture";

WiFiClient espClient;
PubSubClient client(espClient);

#define PWDN_GPIO_NUM 31
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#define photo_path "/image.jpg"
#define MAX_MQTT_PACKET_SIZE 128 // 128 bytes per chunk

#define BUTTON_PIN 4 // Change this to the pin your button is connected to

void setup_camera();
void connect_wifi();
void connect_mqtt();
void capture_and_send_photo();
bool check_photo(fs::FS &fs);
void capture_save_photo(void);
void send_mqtt_chunks(uint8_t *data, size_t len);
void setup_button();
void check_button();

void setup()
{
    Serial.begin(115200);
    
    setup_button(); // Initialize button
    
    connect_wifi();
    
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        ESP.restart();
    }
    else
    {
        delay(500);
        Serial.println("SPIFFS mounted successfully");
    }
    
    setup_camera();
    
    client.setServer(mqtt_server, mqtt_port);
    connect_mqtt();
}

void loop()
{
    if (!client.connected())
    {
        connect_mqtt();
    }
    client.loop();
    
    check_button(); // Check button state
}

void setup_camera()
{
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
    // Reduce the resolution to ensure smaller size images
    config.frame_size = FRAMESIZE_QVGA; // QVGA is 320x240 pixels
    config.jpeg_quality = 10;           // Higher number means lower quality
    config.fb_count = 1;
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK)
    {
        Serial.printf("Camera init failed with error 0x%x", err);
        ESP.restart();
    }
}

void connect_wifi()
{
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
}

void connect_mqtt()
{
    while (!client.connected())
    {
        Serial.println("Connecting to MQTT...");
        if (client.connect("ESP32CAMClient", mqtt_user, mqtt_password))
        {
            Serial.println("connected");
        }
        else
        {
            Serial.print("failed with state ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}

void capture_and_send_photo()
{
    capture_save_photo();
    
    if (check_photo(SPIFFS))
    {
        File photoFile = SPIFFS.open(photo_path, FILE_READ);
        
        if (photoFile)
        {
            size_t photoSize = photoFile.size();
            uint8_t *photoBuffer = (uint8_t *)malloc(photoSize);
            photoFile.read(photoBuffer, photoSize);
            photoFile.close();
            send_mqtt_chunks(photoBuffer, photoSize);
            free(photoBuffer);
        }
        else
        {
            Serial.println("Failed to open photo file");
        }
    }
    else
    {
        Serial.println("Failed to capture photo");
    }
}

void send_mqtt_chunks(uint8_t *data, size_t len)
{
    size_t chunkSize = MAX_MQTT_PACKET_SIZE;
    size_t offset = 0;
    while (offset < len)
    {
        size_t remaining = len - offset;
        if (remaining < chunkSize)
        {
            chunkSize = remaining;
        }
        if (client.publish(mqtt_topic, data + offset, chunkSize))
        {
            Serial.printf("Sent chunk: %d bytes\n", chunkSize);
        }
        else
        {
            Serial.println("Failed to send chunk to MQTT");
        }
        offset += chunkSize;
        delay(100); // Short delay to avoid flooding the broker
    }
}

bool check_photo(fs::FS &fs)
{
    File f_pic = fs.open(photo_path);
    unsigned int pic_sz = f_pic.size();
    f_pic.close();
    return (pic_sz > 100);
}

void capture_save_photo(void)
{
    camera_fb_t *fb = NULL;
    bool ok = 0;
    do
    {
        Serial.println("ESP32-CAM capturing photo...");
        fb = esp_camera_fb_get();
        if (!fb)
        {
            Serial.println("Failed to capture photo");
            return;
        }
        File file = SPIFFS.open(photo_path, FILE_WRITE);
        if (!file)
        {
            Serial.println("Failed to open file in writing mode");
        }
        else
        {
            file.write(fb->buf, fb->len);
            Serial.print("Photo saved in ");
            Serial.print(photo_path);
            Serial.print(" - Size: ");
            Serial.print(file.size());
            Serial.println("bytes");
        }
        file.close();
        esp_camera_fb_return(fb);
        ok = check_photo(SPIFFS);
    } while (!ok);
}

void setup_button()
{
    pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void check_button()
{
    static bool buttonWasPressed = false;
    bool buttonState = digitalRead(BUTTON_PIN);
    
    if (buttonState == LOW && !buttonWasPressed)
    {
        // Button pressed, trigger capture
        capture_and_send_photo();
        buttonWasPressed = true;
    }
    else if (buttonState == HIGH && buttonWasPressed)
    {
        // Button released, reset button press flag
        buttonWasPressed = false;
    }
}
