/*
  HILL AREA MONITORING - TX NODE
  ESP32 30-pin DevKit + Ai-Thinker RA-02 (SX1278) + MQ-2 + DHT12 + MPU6050

  Reads all 3 sensors, then sends the values as a compact packet over LoRa.

  Wiring:
  -------
  RA-02 (LoRa):
    3.3V -> 3V3        GND  -> GND
    NSS  -> GPIO5      SCK  -> GPIO18
    MISO -> GPIO19     MOSI -> GPIO23
    RST  -> GPIO14     DIO0 -> GPIO4

  MQ-2:
    VCC -> 5V (VIN)    GND -> GND    AO -> GPIO34

  DHT12 (3-pin digital / single-wire):
    +   -> 3.3V
    OUT -> GPIO25   *** moved off GPIO4 (used by LoRa DIO0) and off GPIO13 (unreliable on this board) ***
    -   -> GND
    (add 10k pull-up OUT-to-+ if your module doesn't already have one)

  MPU6050 (I2C):
    VCC -> 3.3V   GND -> GND   SDA -> GPIO21   SCL -> GPIO22

  Libraries needed:
    "LoRa" by Sandeep Mistry
    "DHT12 sensor library" by Renzo Mischianti  (NOT Rob Tillaart's I2C-only one)
    "Adafruit MPU6050", "Adafruit Unified Sensor", "Adafruit BusIO"
*/

#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <DHT12.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// ---------- LoRa pins ----------
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  4
#define LORA_FREQ  433E6

// ---------- Sensor pins ----------
#define MQ2_PIN    34
#define DHT12_PIN  25

// ---------- Alert thresholds ----------
const int   SMOKE_THRESHOLD     = 1500;
const float TEMP_HIGH           = 40.0;
const float HUMIDITY_HIGH       = 85.0;
const float VIBRATION_THRESHOLD = 2.0;

DHT12 dht12(DHT12_PIN, true);   // one-wire/digital mode
Adafruit_MPU6050 mpu;

const unsigned long SEND_INTERVAL = 5000; // ms, keep >=3-5s for duty-cycle
unsigned long lastSend = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("HILL MONITOR - TX NODE");

  // --- LoRa init ---
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("LoRa init failed. Check wiring.");
    while (true);
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  LoRa.setTxPower(10, PA_OUTPUT_PA_BOOST_PIN); // 10 dBm, EU 433MHz legal limit
  Serial.println("LoRa ready.");

  // --- Sensors ---
  Wire.begin(21, 22); // SDA, SCL for MPU6050
  if (!mpu.begin()) {
    Serial.println("ERROR: MPU6050 not found. Check wiring!");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("MPU6050 OK");

  pinMode(MQ2_PIN, INPUT);
  Serial.println("MQ-2 OK / DHT12 OK");
  Serial.println("System ready. Sending...\n");
}

void loop() {
  if (millis() - lastSend < SEND_INTERVAL) return;
  lastSend = millis();

  // ---------- Read sensors ----------
  int smokeValue = analogRead(MQ2_PIN);

  float temperature = dht12.readTemperature();
  float humidity     = dht12.readHumidity();

  sensors_event_t a, g, temp_mpu;
  mpu.getEvent(&a, &g, &temp_mpu);
  float totalAccel = sqrt(sq(a.acceleration.x) + sq(a.acceleration.y) + sq(a.acceleration.z));
  float vibration = abs(totalAccel - 9.8);

  // ---------- Alerts ----------
  int fireAlert = (smokeValue > SMOKE_THRESHOLD) ? 1 : 0;
  int envAlert  = (!isnan(temperature) && !isnan(humidity) &&
                    (temperature > TEMP_HIGH || humidity > HUMIDITY_HIGH)) ? 1 : 0;
  int rockAlert = (vibration > VIBRATION_THRESHOLD) ? 1 : 0;

  // Use -999 as a "sensor error" sentinel for NaN readings
  float tempOut = isnan(temperature) ? -999 : temperature;
  float humOut  = isnan(humidity)    ? -999 : humidity;

  // ---------- Build packet ----------
  // Format: S:<smoke>,T:<temp>,H:<hum>,V:<vib>,F:<0/1>,E:<0/1>,R:<0/1>
  String packet = "S:" + String(smokeValue)
                 + ",T:" + String(tempOut, 2)
                 + ",H:" + String(humOut, 2)
                 + ",V:" + String(vibration, 2)
                 + ",F:" + String(fireAlert)
                 + ",E:" + String(envAlert)
                 + ",R:" + String(rockAlert);

  // ---------- Send ----------
  LoRa.beginPacket();
  LoRa.print(packet);
  LoRa.endPacket();

  Serial.print("Sent: ");
  Serial.println(packet);
}
