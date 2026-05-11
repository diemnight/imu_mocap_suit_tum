#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ── Pins ───────────────────────────────────────────────────────────────────────
#define SDA_PIN       8
#define SCL_PIN       9
#define MUX_RESET_PIN 4

// ── TCA9548A ───────────────────────────────────────────────────────────────────
#define MUX_ADDR 0x70
#define NUM_IMUS  6

// ── Single reusable IMU instance ──────────────────────────────────────────────
Adafruit_BNO08x imu(-1);
sh2_SensorValue_t sensorValue;

bool imu_ok[NUM_IMUS] = { false };

// ── Per-IMU state ──────────────────────────────────────────────────────────────
float qw[NUM_IMUS], qi[NUM_IMUS], qj[NUM_IMUS], qk[NUM_IMUS];
float ax[NUM_IMUS], ay[NUM_IMUS], az[NUM_IMUS];

// ── Select a TCA9548A channel (0-7) ───────────────────────────────────────────
void muxSelect(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(200);
}

// ── Reset the TCA9548A ────────────────────────────────────────────────────────
void muxReset() {
  digitalWrite(MUX_RESET_PIN, LOW);
  delay(10);
  digitalWrite(MUX_RESET_PIN, HIGH);
  delay(10);
}

// ── Enable rotation vector + linear accel ─────────────────────────────────────
void enableReports() {
  imu.enableReport(SH2_GAME_ROTATION_VECTOR, 5000);
  imu.enableReport(SH2_LINEAR_ACCELERATION,  5000);
}

// ── Send 35-byte binary packet over Serial ────────────────────────────────────
void sendPacket(uint8_t id,
                float qw_, float qi_, float qj_, float qk_,
                float ax_, float ay_, float az_) {
  uint8_t buf[35];
  uint32_t ts = micros();
  buf[0] = 0xAA;
  buf[1] = 0xFF;
  buf[2] = id;
  memcpy(buf + 3,  &ts,  4);
  memcpy(buf + 7,  &qw_, 4);
  memcpy(buf + 11, &qi_, 4);
  memcpy(buf + 15, &qj_, 4);
  memcpy(buf + 19, &qk_, 4);
  memcpy(buf + 23, &ax_, 4);
  memcpy(buf + 27, &ay_, 4);
  memcpy(buf + 31, &az_, 4);
  Serial.write(buf, 35);
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(921600);
  delay(1000);

  pinMode(MUX_RESET_PIN, OUTPUT);
  digitalWrite(MUX_RESET_PIN, HIGH);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  muxReset();
  delay(100);

  // Init using ONLY the first available IMU found
  bool driver_initialized = false;

  for (uint8_t ch = 0; ch < NUM_IMUS; ch++) {
    muxSelect(ch);
    delay(200);

    Wire.beginTransmission(0x4B);
    if (Wire.endTransmission() != 0) {
      Serial.print("IMU "); Serial.print(ch);
      Serial.println(" not visible, skipping");
      imu_ok[ch] = false;
      continue;
    }

    imu_ok[ch] = true;
    Serial.print("IMU "); Serial.print(ch); Serial.println(" visible");

    if (!driver_initialized) {
      // Only call begin_I2C once ever
      if (imu.begin_I2C(0x4B, &Wire)) {
        driver_initialized = true;
        enableReports();
        Serial.print("IMU "); Serial.print(ch); Serial.println(" driver initialized OK");
      } else {
        Serial.println("Driver init FAILED");
        imu_ok[ch] = false;
      }
    } else {
      // For subsequent IMUs, just enable reports — the chip is fresh
      enableReports();
      Serial.print("IMU "); Serial.print(ch); Serial.println(" reports enabled");
    }
    delay(100);
  }

  for (uint8_t ch = 0; ch < NUM_IMUS; ch++) {
    if (!imu_ok[ch]) continue;
    muxSelect(ch);
    delay(50);
    enableReports();
    delay(50);
  }
  Wire.setClock(400000);
  Serial.println("SETUP DONE --- streaming ---");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  for (uint8_t ch = 0; ch < NUM_IMUS; ch++) {
    if (!imu_ok[ch]) continue;

    muxSelect(ch);

    if (imu.wasReset()) enableReports();

    if (imu.getSensorEvent(&sensorValue)) {
      switch (sensorValue.sensorId) {

        case SH2_GAME_ROTATION_VECTOR:
          qw[ch] = sensorValue.un.gameRotationVector.real;
          qi[ch] = sensorValue.un.gameRotationVector.i;
          qj[ch] = sensorValue.un.gameRotationVector.j;
          qk[ch] = sensorValue.un.gameRotationVector.k;
          sendPacket(ch + 1,
                     qw[ch], qi[ch], qj[ch], qk[ch],
                     ax[ch], ay[ch], az[ch]);
          break;

        case SH2_LINEAR_ACCELERATION:
          ax[ch] = sensorValue.un.linearAcceleration.x;
          ay[ch] = sensorValue.un.linearAcceleration.y;
          az[ch] = sensorValue.un.linearAcceleration.z;
          break;
      }
    }
  }
}
