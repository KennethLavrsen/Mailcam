/* Using special Kenneth ESP32-CAM board
In ~arduino-1.8.X/portable/packages/esp32/hardware/esp32/1.0.4/boards.txt
Add this extra board to the end
##############################################################

kenneth32cam.name=Kenneth ESP32-CAM

kenneth32cam.upload.tool=esptool_py
kenneth32cam.upload.maximum_size=3145728
kenneth32cam.upload.maximum_data_size=327680
kenneth32cam.upload.wait_for_upload_port=true
kenneth32cam.upload.speed=460800

kenneth32cam.serial.disableDTR=true
kenneth32cam.serial.disableRTS=true

kenneth32cam.build.mcu=esp32
kenneth32cam.build.core=esp32
kenneth32cam.build.variant=esp32
kenneth32cam.build.board=ESP32_DEV
kenneth32cam.build.f_cpu=240000000L
kenneth32cam.build.flash_size=4MB
kenneth32cam.build.flash_freq=80m
kenneth32cam.build.flash_mode=dio
kenneth32cam.build.boot=qio
kenneth32cam.build.partitions=default
kenneth32cam.build.defines=-DBOARD_HAS_PSRAM -mfix-esp32-psram-cache-issue
kenneth32cam.build.code_debug=0

##############################################################

In a PHP script on webserver have this simple program mailcam.php
<?php
  $received = file_get_contents('php://input');
  $fileToWrite = "mail.jpg";
  if ( strlen($received) > 0 ) {
      file_put_contents($fileToWrite, $received);
  }
?>

*/

#include "esp_http_client.h"
#include "esp_camera.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "Arduino.h"
#include "driver/rtc_io.h"
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems
#include "driver/adc.h"        // Extra cover by butt in deepsleep sub
#include <esp_wifi.h>          // Extra cover by butt in deepsleep sub
#include <esp_bt.h>            // Extra cover by butt in deepsleep sub
#include "secrets-mailcam.h"

/****** All the configuration happens here ******/
// Wifi
const char* ssid     = WIFI_SSID;
const char* password = WIFI_PASSWORD;
IPAddress local_IP(192, 168, 1, 36);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8); //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

// MQTT
const char* mqttServer        = MQTT_SERVER;
const char* mqttUsername      = MQTT_USER;
const char* mqttPassword      = MQTT_PASSWORD;
const int   mqttPort          = 1883;
const char* mqttTopicAnnounce = "mailcam/announce";
const char* mqttTopicDebugSet = "mailcam/debug/set";
const char* mqttTopicDebug    = "mailcam/debug";
const char* mqttTopicReset    = "mailcam/reset";
const char* mqttTopicBattery  = "mailcam/battery";
const char* mqttTopicSnapshot = "mailcam/snapshot";

// Host name and OTA password
const char* hostName    = "mailcam";
const char* otaPassword = OTA_PASSWORD;

// Location where images are POSTED
const char *post_url = MAILCAM_URL;

// Define debug and normal wakeup intervals
#define REPORT_INTERVAL_DEBUG   1200  /* Debug interval is 20 minutes */
#define REPORT_INTERVAL_NORMAL  7200  /* Normal interval is 2 hours */

// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define LED 4
// 00EA 0000 0000 0000  E=empty GPIO13, A=arrive GPIO12
#define MAILARRIVE 0x01000
#define MAILEMPTY  0x02000
#define MAILARRIVEPORT 12
#define MAILEMPTYPORT  13
#define BATTERY        33

/****** End of configuration ******/

#define uS_TO_S_FACTOR 1000000ULL     /* Conversion factor for micro seconds to seconds */

bool internet_connected = false;
unsigned long mqttReconnectTimer = 0;
bool debugMode = false;
bool arrivalOpen = false;
bool emptyOpen = false;
bool snapshot = false;
long current_millis;
long last_capture_millis = 0;
RTC_DATA_ATTR uint32_t bootCount = 0; //times restarted
char mqttbuf[40];                     //buffer for MQTT messages to be sent

WiFiClient espClient;
PubSubClient client(espClient);


void setup()
{
    //Serial.begin(115200);
    pinMode(MAILARRIVEPORT,INPUT);
    pinMode(MAILEMPTYPORT,INPUT);
    pinMode(LED,OUTPUT);
    
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    rtc_gpio_hold_dis(GPIO_NUM_4); //Re-enable GPIO 4

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
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    /*!< Size of the output image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA  */
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;

    setup_wifi();

    // camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        //Serial.printf("Camera init failed with error 0x%x", err);
        deepSleep();
    }

    sensor_t * s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
        //Serial.println("I am an OV3660");
        s->set_vflip(s, 1);//flip it back
        s->set_brightness(s, 1);//up the blightness just a bit
        s->set_saturation(s, -2);//lower the saturation
    }

    // We connect to MQTT before we take a picture while we wait for camera to settle
    client.setServer(mqttServer, mqttPort);
    client.setCallback(mqttCallback);
    mqttReconnectTimer = 0;
    mqttConnect();

    uint64_t GPIO_reason = esp_sleep_get_ext1_wakeup_status();
    if ( debugMode ) {
        client.publish(mqttTopicDebug, "Waking up in debug mode");
    }

    if ( GPIO_reason == MAILARRIVE ) {
        if ( debugMode ) client.publish(mqttTopicDebug, "arrive");
        arrivalOpen = true;
    }
    
    if ( GPIO_reason == MAILEMPTY ) {
        if ( debugMode ) client.publish(mqttTopicDebug, "empty");
        emptyOpen = true;
    }

    float voltageAverage, voltages[5];
    for (int i = 0; i < 5; i++) {
        voltages[i] = analogRead(BATTERY);
    }
    std::sort(voltages, voltages + 5, std::greater<float>());
    voltageAverage = (voltages[1] + voltages[1] + voltages[1]) / 3;
    dtostrf(voltageAverage/993.7 + 0.7, 4, 2, mqttbuf); // Curvefit
    client.publish(mqttTopicBattery, mqttbuf, true );

    // Port defaults to 3232
    // ArduinoOTA.setPort(3232);

    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname(hostName);

    // No authentication by default
    ArduinoOTA.setPassword(otaPassword);

    ArduinoOTA
        .onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH)
                type = "sketch";
            else // U_SPIFFS
                type = "filesystem";

            // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        })
        .onEnd([]() {
        })
        .onProgress([](unsigned int progress, unsigned int total) {
        })
        .onError([](ota_error_t error) {
        });

    ArduinoOTA.begin();

}

void deepSleep(void)
{
        //WiFi.disconnect(true);
        //WiFi.mode(WIFI_OFF);
        //btStop();
        //adc_power_off();
        //esp_wifi_stop();
        //esp_bt_controller_disable();
        
        esp_sleep_enable_ext1_wakeup(MAILARRIVE + MAILEMPTY, ESP_EXT1_WAKEUP_ANY_HIGH);
        rtc_gpio_hold_en(GPIO_NUM_4); //Latch value when going into deep sleep
        if ( debugMode ) { 
            esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_DEBUG * uS_TO_S_FACTOR);
        } else {
            esp_sleep_enable_timer_wakeup(REPORT_INTERVAL_NORMAL * uS_TO_S_FACTOR);
        }
        esp_deep_sleep_start(); //Restart cam
}

bool setup_wifi()
{
    int connAttempts = 0;
    //Serial.println("\r\nConnecting to: " + String(ssid));
    WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED ) {
        delay(500);
        //Serial.print(".");
        if (connAttempts > 40) deepSleep();   // Go to sleep if Wifi is not present
        connAttempts++;
    }
    return true;
}

// MQTT Callback - when we receive mqtt
void mqttCallback(char* topic, byte* payload, unsigned int length) {

    // Creating safe local copies of topic and payload
    // enables publishing MQTT within the callback function
    // We avoid functions using malloc to avoid memory leaks
    
    char topicCopy[strlen(topic) + 1];
    strcpy(topicCopy, topic);
    
    char message[length + 1];
    for (int i = 0; i < length; i++) message[i] = (char)payload[i];
    
    message[length] = '\0';
  
    if ( strcmp(topicCopy, mqttTopicDebugSet) == 0 ) {
        if ( strcmp(message, "on") == 0 ) {
            debugMode = true;
            client.publish(mqttTopicDebug, "debug on");
        }
        if ( strcmp(message, "off") == 0 ) {
            debugMode = false;
            client.publish(mqttTopicDebug, "debug off");
        }
    }

    if ( strcmp(topicCopy, mqttTopicReset) == 0 ) {
        if ( strcmp(message, "reset") == 0 ) {
            ESP.restart();
        }
    }

    if ( strcmp(topicCopy, mqttTopicSnapshot) == 0 ) {
        if ( strcmp(message, "on") == 0 ) {
            snapshot = true;
        }
    }

}

bool mqttConnect() {

    //Serial.print("Attempting MQTT connection...");
    if (client.connect(hostName, mqttUsername, mqttPassword)) {
        //Serial.println("connected");
        // Once connected, publish an announcement...
        client.publish(mqttTopicAnnounce, "connected");
        client.subscribe(mqttTopicReset);
        client.subscribe(mqttTopicDebugSet);
        client.subscribe(mqttTopicSnapshot);
    }

    return client.connected();
}

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    return ESP_OK;
}

static esp_err_t take_send_photo(void)
{
    digitalWrite(LED, HIGH);
    //Delay is to let camera settle
    delay(3000);
    //Serial.println("Taking picture...");
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;

    fb = esp_camera_fb_get();
    if (!fb) {
        //Serial.println("Camera capture failed");
        esp_camera_fb_return(fb);
        return ESP_FAIL;
    }
  
    esp_http_client_handle_t http_client;
    
    esp_http_client_config_t config_client = {0};
    config_client.url = post_url;
    config_client.event_handler = _http_event_handler;
    config_client.method = HTTP_METHOD_POST;
    http_client = esp_http_client_init(&config_client);
    esp_http_client_set_post_field(http_client, (const char *)fb->buf, fb->len);
    esp_http_client_set_header(http_client, "Content-Type", "image/jpg");
    esp_err_t err = esp_http_client_perform(http_client);
    esp_http_client_cleanup(http_client);
    esp_camera_fb_return(fb);
    digitalWrite(LED, LOW);
}



void loop()
{

    if (WiFi.status() != WL_CONNECTED) {
        setup_wifi();
        return;
    }
    
    ArduinoOTA.handle();

    if (!client.connected()) {
        unsigned long currentTime = millis();
        if ( currentTime - mqttReconnectTimer > 5000 ) {
            mqttReconnectTimer = currentTime;
            if ( mqttConnect() ) {
                mqttReconnectTimer = 0;
            }
        }
    } else {
        client.loop();
    }

    if ( !digitalRead(MAILARRIVEPORT) && arrivalOpen ) {
        if ( debugMode ) client.publish("mailcam/mail", "mail");
        client.publish("mailserver/set", "on", false);         // Turn on Blue light
        client.publish("mailserver/state", "on", true);        // Turn on mail state for display and HA
        client.publish("mailserver/stateless", "on", false);   //Signal HA to announce mail
        take_send_photo();
        if ( debugMode ) client.publish(mqttTopicDebug, "Photo and sleep");
        delay(2000);
        deepSleep();
    }

    if ( !digitalRead(MAILEMPTYPORT) && emptyOpen ) {
        if ( debugMode ) client.publish("mailcam/mail", "empty");
        client.publish("mailserver/set", "off", false);
        client.publish("mailserver/state", "off", true);
        delay(2000);
        deepSleep();
    }

    if ( snapshot ) {
        client.publish(mqttTopicSnapshot, "off", true);
        if ( debugMode ) client.publish("mailcam/mail", "snapshot");
        take_send_photo();
        if ( debugMode ) client.publish(mqttTopicDebug, "Photo and sleep");
        delay(2000);
    }

    if ( !arrivalOpen && !emptyOpen ) {
        if ( debugMode ) client.publish(mqttTopicDebug, "To sleep");
        delay(2000);
        deepSleep();
    }
}
