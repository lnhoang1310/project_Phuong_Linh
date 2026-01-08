#include <Wire.h>
#include <MAX30105.h>
#include "heartRate.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

/* ================= CAMERA ================= */
#include "esp_camera.h"
#define CAMERA_MODEL_AI_THINKER
#include "camera_pins.h"

/* ================= WIFI ================= */
// #define WIFI_SSID   "Nhà 4"
// #define WIFI_PASS   "12348765"

#define WIFI_SSID   "Lâm Hoàng"
#define WIFI_PASS   "123456789"

/* ================= MQTT ================= */
#define MQTT_BROKER "06584af3887143f19a8ff4d8987d848f.s1.eu.hivemq.cloud"
#define MQTT_PORT   8883
#define MQTT_USER   "esp32cam_Duong"
#define MQTT_PASS   "Duong1111111111"
#define TOPIC_STATUS "driver_status"

/* ================= BUZZER ================= */
#define BUZZER_PIN 33

/* ================= MAX30102 ================= */
MAX30105 particleSensor;
float heartRate = 0;
float heartRate_Filter = 75;
float alpha = 0.3f;
long lastBeat = 0;
bool no_finger = true;

/* ================= MQTT ================= */
WiFiClientSecure espClient;
PubSubClient client(espClient);
uint32_t lastPublish = 0;

/* ================= BUZZER STATE ================= */
enum BuzzerMode {
  BUZZER_OFF,
  BUZZER_BUON_NGU,
  BUZZER_NGU_GAT
};

BuzzerMode buzzerMode = BUZZER_OFF;
bool buzzerState = false;
uint32_t buzzerTimer = 0;
uint8_t beepCount = 0;

/* ================= MQTT CALLBACK ================= */
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<128> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* status = doc["status"];
  if (!status) return;

  Serial.print("MQTT status: ");
  Serial.println(status);

  if (!strcmp(status, "Buon ngu")) {
    buzzerMode = BUZZER_BUON_NGU;
  }
  else if (!strcmp(status, "Ngu gat")) {
    buzzerMode = BUZZER_NGU_GAT;
  }
  else if (!strcmp(status, "Tinh tao")) {
    buzzerMode = BUZZER_OFF;
    digitalWrite(BUZZER_PIN, LOW);
  }
}

/* ================= MQTT RECONNECT ================= */
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    if (client.connect("ESP32CAM_MAX30102", MQTT_USER, MQTT_PASS)) {
      Serial.println("OK");
      client.subscribe(TOPIC_STATUS);
    } else {
      Serial.print("failed rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

/* ================= BUZZER TASK ================= */
void buzzerTask() {
  uint32_t now = millis();

  if (buzzerMode == BUZZER_OFF) {
    digitalWrite(BUZZER_PIN, LOW);
    return;
  }

  if (buzzerMode == BUZZER_NGU_GAT) {
    digitalWrite(BUZZER_PIN, HIGH);
    return;
  }

  if (buzzerMode == BUZZER_BUON_NGU) {
    if (beepCount < 2) {
      if (!buzzerState) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerTimer = now;
      }
      else if (now - buzzerTimer >= 150) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerTimer = now;
        beepCount++;
      }
    }
    else {
      if (now - buzzerTimer >= 2000) {
        beepCount = 0;
      }
    }
  }
}

/* ================= CAMERA SETUP ================= */
void startCamera() {
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
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG;  // for streaming
  //config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;
#if CONFIG_IDF_TARGET_ESP32S3
    config.fb_count = 2;
#endif
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        // flip it back
    s->set_brightness(s, 1);   // up the brightness just a bit
    s->set_saturation(s, -2);  // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if (config.pixel_format == PIXFORMAT_JPEG) {
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif
// Setup LED FLash if LED pin is defined in camera_pins.h
// #if defined(LED_GPIO_NUM)
//   setupLedFlash(LED_GPIO_NUM);
// #endif
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  espClient.setInsecure();
  client.setServer(MQTT_BROKER, MQTT_PORT);
  client.setCallback(mqttCallback);

  Wire.begin(14, 15);
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeGreen(0);

  startCamera();

  Serial.print("Camera Ready: http://");
  Serial.println(WiFi.localIP());
}

/* ================= LOOP ================= */
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  long irValue = particleSensor.getIR();
  if (irValue < 50000) {
    no_finger = true;
  } else if (checkForBeat(irValue)) {
    if (no_finger) {
      lastBeat = millis();
      no_finger = false;
    } else {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      heartRate = 60.0 / (delta / 1000.0);
      if (heartRate >= 50 && heartRate <= 120) {
        heartRate_Filter = alpha * heartRate + (1 - alpha) * heartRate_Filter;
      }
    }
  }

  if (!no_finger && millis() - lastPublish >= 1000) {
    lastPublish = millis();
    StaticJsonDocument<128> doc;
    doc["heart_rate"] = heartRate_Filter;
    char payload[128];
    serializeJson(doc, payload);
    client.publish(TOPIC_STATUS, payload);
    Serial.print("Publish MQTT: ");
    Serial.println(heartRate_Filter);
  }

  buzzerTask();
}
