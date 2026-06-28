#include <Wire.h>
#include <Adafruit_BME680.h>
#include <Adafruit_PM25AQI.h>
#include <SensirionI2cScd4x.h>
#include <Notecard.h>

// Uncomment for testing: sample every 1 min, sync every 4 min
// #define TEST_MODE

#define NOTECARD_PRODUCT_UID "me.proton.areskoch:riverscape_node"
#define usbSerial Serial

#ifdef TEST_MODE
  #define SLEEP_US  (1ULL * 60 * 1000000)             // 1 min
  #define VOUTBOUND "usb:2;high:4;normal:4;low:8;dead:0"
  #define VINBOUND  "usb:4;high:8;normal:8;low:16;dead:0"
#else
  #define SLEEP_US  (15ULL * 60 * 1000000)            // 15 min
  #define VOUTBOUND "usb:30;high:60;normal:240;low:720;dead:0"
  #define VINBOUND  "usb:60;high:120;normal:720;low:1440;dead:0"
#endif

Adafruit_BME680 bme;
Adafruit_PM25AQI pmsa = Adafruit_PM25AQI();
SensirionI2cScd4x scd41;
Notecard notecard;

#define PMS_POWER_PIN 10

void setup() {
  usbSerial.begin(115200);
  Wire.begin(3, 4);
  notecard.begin();

  // Voltage thresholds — tighter than lipo defaults so we back off before brownout
  J *vcfg = notecard.newRequest("card.voltage");
  JAddStringToObject(vcfg, "mode", "usb:4.6;high:4.2;normal:3.7;low:3.4;dead:0");
  notecard.sendRequest(vcfg);

  // Voltage-variable sync — Notecard picks cadence based on battery state
  J *req = notecard.newRequest("hub.set");
  JAddStringToObject(req, "product",   NOTECARD_PRODUCT_UID);
  JAddStringToObject(req, "mode",      "periodic");
  JAddStringToObject(req, "voutbound", VOUTBOUND);
  JAddStringToObject(req, "vinbound",  VINBOUND);
  notecard.sendRequest(req);

  // BME688
  if (!bme.begin()) usbSerial.println("BME688 not found");
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);

  // PMSA003I
  if (PMS_POWER_PIN >= 0) {
    pinMode(PMS_POWER_PIN, OUTPUT);
    digitalWrite(PMS_POWER_PIN, HIGH);
  }
  usbSerial.println("Warming up PMS fan (30s)...");
  delay(30000);
  if (!pmsa.begin_I2C()) usbSerial.println("PMSA003I not found");

  // SCD41
  scd41.begin(Wire, SCD41_I2C_ADDR_62);
  scd41.stopPeriodicMeasurement();
  delay(500);
  scd41.startPeriodicMeasurement();
  delay(5000);
}

void loop() {
  // ── BME688 ──
  if (!bme.performReading()) {
    usbSerial.println("BME688 read failed");
    delay(5000);
    return;
  }
  float humidity = bme.humidity;
  float pressure = bme.pressure / 100.0;
  float voc_raw  = bme.gas_resistance / 1000.0;

  // ── PMSA003I ──
  PM25_AQI_Data pms_data;
  float pm1 = 0, pm25 = 0, pm10 = 0;
  if (pmsa.read(&pms_data)) {
    pm1  = pms_data.pm10_standard;
    pm25 = pms_data.pm25_standard;
    pm10 = pms_data.pm100_standard;
  }

  // ── SCD41 ──
  uint16_t co2_raw = 0;
  float scd_temp = 0, scd_hum = 0;
  bool dataReady = false;
  scd41.getDataReadyStatus(dataReady);
  if (dataReady) {
    scd41.readMeasurement(co2_raw, scd_temp, scd_hum);
  }
  float temp_f = (scd_temp * 9.0 / 5.0) + 32.0;

  // ── Battery ──
  float batt_pct = 0;
  J *vreq = notecard.newRequest("card.voltage");
  JAddStringToObject(vreq, "mode", "lipo");
  J *vres = notecard.requestAndResponse(vreq);
  if (vres) {
    float voltage = JGetNumber(vres, "value");
    batt_pct = constrain((voltage - 3.4f) / (4.2f - 3.4f) * 100.0f, 0.0f, 100.0f);
    usbSerial.printf("BAT: %.2fV %.1f%%\n", voltage, batt_pct);
    notecard.deleteResponse(vres);
  }

  // ── Queue note — Notecard will sync on its own schedule ──
  J *req = notecard.newRequest("note.add");
  JAddStringToObject(req, "file", "riverscape_data.qo");
  JAddBoolToObject(req, "sync", false);
  J *body = JCreateObject();
  JAddNumberToObject(body, "temp_f",       temp_f);
  JAddNumberToObject(body, "humidity_pct", humidity);
  JAddNumberToObject(body, "pressure_hpa", pressure);
  JAddNumberToObject(body, "voc_kohm",     voc_raw);
  JAddNumberToObject(body, "co2_ppm",      co2_raw);
  JAddNumberToObject(body, "pm1",          pm1);
  JAddNumberToObject(body, "pm25",         pm25);
  JAddNumberToObject(body, "pm10",         pm10);
  JAddNumberToObject(body, "battery_pct",  batt_pct);
  JAddItemToObject(req, "body", body);
  notecard.sendRequest(req);

  usbSerial.printf("Queued → T:%.1fF H:%.1f P:%.1f VOC:%.2f PM1:%.1f PM2.5:%.1f PM10:%.1f CO2:%d BAT:%.1f%%\n",
    temp_f, humidity, pressure, voc_raw, pm1, pm25, pm10, co2_raw, batt_pct);

  // ── Power down PMS, deep sleep ──
  if (PMS_POWER_PIN >= 0) digitalWrite(PMS_POWER_PIN, LOW);
  esp_sleep_enable_timer_wakeup(SLEEP_US);
  esp_deep_sleep_start();
}