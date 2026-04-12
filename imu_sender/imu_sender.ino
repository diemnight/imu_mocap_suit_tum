#include <SPI.h>
#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ── WiFi credentials ──────────────────────────────────────────────────────────
const char* SSID     = "IMU_network";
const char* PASSWORD = "imu12345";

// ── PC IP and port — change this to your PC's local IP ───────────────────────
const char* PC_IP   = "10.42.0.1";
const uint16_t PORT = 5005;     // destination port (PC receives on this)
const uint16_t SRC_PORT = 5006;   // source port (ESP32 sends from this)

// ── SPI + IMU pins ────────────────────────────────────────────────────────────
#define SPI_SCK      12
#define SPI_MISO     13
#define SPI_MOSI     11
#define BNO08X_CS    10
#define BNO08X_INT   9
#define BNO08X_RESET 5

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;
WiFiUDP udp;

float qw = 1, qi = 0, qj = 0, qk = 0;
float ax = 0, ay = 0, az = 0;

void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected — IP: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  connectWiFi();

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, BNO08X_CS);
  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT, &SPI)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }
  Serial.println("BNO08x found!");

  bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
  bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);

  udp.begin(SRC_PORT);
}

void sendPacket() {
  // packet: [0xAA][0xFF][board_id 1][ts 4][qw 4][qi 4][qj 4][qk 4][ax 4][ay 4][az 4]
  // total: 2 + 1 + 4 + 28 = 35 bytes
  uint8_t buf[35];
  uint32_t ts = micros();

  buf[0] = 0xAA;
  buf[1] = 0xFF;
  buf[2] = 0x01;  // board ID — change this per ESP32 (1-6)
  memcpy(buf + 3,  &ts, 4);
  memcpy(buf + 7,  &qw, 4);
  memcpy(buf + 11, &qi, 4);
  memcpy(buf + 15, &qj, 4);
  memcpy(buf + 19, &qk, 4);
  memcpy(buf + 23, &ax, 4);
  memcpy(buf + 27, &ay, 4);
  memcpy(buf + 31, &az, 4);

  udp.beginPacket(PC_IP, PORT);
  udp.write(buf, 35);
  int result = udp.endPacket();
  Serial.print("Sending to: ");
  Serial.print(PC_IP);
  Serial.print(":");
  Serial.println(PORT);
  Serial.print("endPacket: ");
  Serial.println(result);
}

void loop() {
  // Reconnect if WiFi drops
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost — reconnecting...");
    connectWiFi();
  }

  if (bno08x.wasReset()) {
    bno08x.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
    bno08x.enableReport(SH2_LINEAR_ACCELERATION, 5000);
  }

  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      case SH2_GAME_ROTATION_VECTOR:
        qw = sensorValue.un.gameRotationVector.real;
        qi = sensorValue.un.gameRotationVector.i;
        qj = sensorValue.un.gameRotationVector.j;
        qk = sensorValue.un.gameRotationVector.k;
        sendPacket();
        break;
      case SH2_LINEAR_ACCELERATION:
        ax = sensorValue.un.linearAcceleration.x;
        ay = sensorValue.un.linearAcceleration.y;
        az = sensorValue.un.linearAcceleration.z;
        break;
    }
  }
}