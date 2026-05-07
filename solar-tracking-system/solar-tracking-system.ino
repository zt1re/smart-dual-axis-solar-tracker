// BLYNK 
#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME ""
#define BLYNK_AUTH_TOKEN ""

#include <ESP32Servo.h>
#include <math.h>
#include <DHT.h>
#include <WiFi.h>
#include <time.h>
#include <SunPosition.h>
#include <BlynkSimpleEsp32.h>

// WIFI 
const char* ssid = "*****";
const char* password = "********";

// LOCATION 
const float LAT = 33.3152;
const float LON = 44.3661;

// SUN 
float sunAz = 0;
float sunEl = 0;

// SERVOS + LDR 
Servo horizontal;
Servo vertical;

int servohori = 90;
int servovert = 90;

const int servohoriLimitHigh = 180;
const int servohoriLimitLow  = 0;
const int servovertLimitHigh = 180;
const int servovertLimitLow  = 0;

const int LT = 35;
const int RT = 32;
const int LD = 34;
const int RD = 33;

float filt_lt, filt_rt, filt_ld, filt_rd;
const float lidf = 0.25f;

const float KpVert  = 0.002f; 
const float KpHoriz = 0.002f;

const int tolHigh = 120;
const int tolLow  = 80;
const int maxStep = 5;

const int invertVert  = -1;
const int invertHoriz = +1;

// DHT 
#define DHT_PIN 18
DHT dht(DHT_PIN, DHT11);

// DUST 
#define DUST_LED 25
#define DUST_ADC 39

// ALERT THRESHOLDS 
const float TEMP_HIGH = 35.0;
const float HUMIDITY_HIGH = 80.0;
const float DUST_HIGH = 150.0;

bool tempAlertSent = false;
bool humidityAlertSent = false;
bool dustAlertSent = false;

// SYSTEM MODES 
bool autoTrackingEnabled = true;
bool hybridMode = true;

// BLYNK CONTROL 
BLYNK_WRITE(V5) {
  autoTrackingEnabled = param.asInt();
}

BLYNK_WRITE(V7) {
  hybridMode = param.asInt();
}

// EFFICIENCY
float cleanLight = 8000;
int dirtyCounter = 0;

// TIMING
unsigned long lastSensorRead = 0;
unsigned long lastSunUpdate = 0;

const unsigned long sensorInterval = 2000;
const unsigned long sunInterval = 2000;

// DUST READ
float readDust() {
  digitalWrite(DUST_LED, LOW);
  delayMicroseconds(280);

  int adc = analogRead(DUST_ADC);

  digitalWrite(DUST_LED, HIGH);

  float voltage = adc * (3.3 / 4095.0);
  float dust = 170.0 * voltage - 0.1;

  return max(dust, 0.0f);
}

void setup() {
  Serial.begin(115200);

  // WIFI + BLYNK
  WiFi.begin(ssid, password);
  Blynk.config(BLYNK_AUTH_TOKEN);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  Blynk.connect();

  // NTP
  configTime(3 * 3600, 0, "pool.ntp.org");

  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    delay(500);
  }

  // LDR 
  analogReadResolution(12);
  analogSetPinAttenuation(LT, ADC_11db);
  analogSetPinAttenuation(RT, ADC_11db);
  analogSetPinAttenuation(LD, ADC_11db);
  analogSetPinAttenuation(RD, ADC_11db);

  // SERVOS
  horizontal.attach(14, 500, 2400);
  vertical.attach(27, 500, 2400);

  horizontal.write(90);
  vertical.write(90);

  filt_lt = analogRead(LT);
  filt_rt = analogRead(RT);
  filt_ld = analogRead(LD);
  filt_rd = analogRead(RD);

  // DHT
  dht.begin();

  // DUST
  pinMode(DUST_LED, OUTPUT);
  digitalWrite(DUST_LED, HIGH);
  analogSetPinAttenuation(DUST_ADC, ADC_11db);

}

void loop() {
  Blynk.run();

  unsigned long now = millis();

  // SUN UPDATE 
  if (now - lastSunUpdate >= sunInterval) {
    lastSunUpdate = now;

    time_t t = time(NULL);
    struct tm *timeinfo = localtime(&t);

    SunPosition sun(LAT, LON, t);
    sun.compute(LAT, LON, t);

    sunAz = sun.azimuth();
    sunEl = sun.altitude();

  }

  // NIGHT POSITION
  if (sunEl < 5) {
    horizontal.write(90);
    vertical.write(10);
  }

  // LDR + HYBRID TRACKING 
  if (autoTrackingEnabled && sunEl >= 5) {
    int lt_raw = analogRead(LT);
    int rt_raw = analogRead(RT);
    int ld_raw = analogRead(LD);
    int rd_raw = analogRead(RD);

    filt_lt = lidf * lt_raw + (1 - lidf) * filt_lt;
    filt_rt = lidf * rt_raw + (1 - lidf) * filt_rt;
    filt_ld = lidf * ld_raw + (1 - lidf) * filt_ld;
    filt_rd = lidf * rd_raw + (1 - lidf) * filt_rd;

    int lt = (int)filt_lt;
    int rt = (int)filt_rt;
    int ld = (int)filt_ld;
    int rd = (int)filt_rd;

    float avt = (lt + rt) / 2.0f;
    float avd = (ld + rd) / 2.0f;
    float avl = (lt + ld) / 2.0f;
    float avr = (rt + rd) / 2.0f;

    float dvert  = avt - avd;
    float dhoriz = avl - avr;

    static bool movingV = false;
    static bool movingH = false;

    if (!movingV && fabs(dvert) > tolHigh) movingV = true;
    if ( movingV && fabs(dvert) < tolLow)  movingV = false;

    if (!movingH && fabs(dhoriz) > tolHigh) movingH = true;
    if ( movingH && fabs(dhoriz) < tolLow) movingH = false;

    float deltaVert  = movingV ? invertVert  * KpVert  * dvert  : 0;
    float deltaHoriz = movingH ? invertHoriz * KpHoriz * dhoriz : 0;

    deltaVert  = constrain(deltaVert,  -maxStep, maxStep);
    deltaHoriz = constrain(deltaHoriz, -maxStep, maxStep);

    int baseH = map(sunAz, 0, 360, 0, 180);
    int baseV = map(sunEl, 0, 90, 0, 90);

    int targetV;
    int targetH;

    if (hybridMode) {
      targetV = constrain(baseV + round(deltaVert), servovertLimitLow, servovertLimitHigh);
      targetH = constrain(baseH + round(deltaHoriz), servohoriLimitLow, servohoriLimitHigh);
    } else {
      targetV = constrain(servovert + round(deltaVert), servovertLimitLow, servovertLimitHigh);
      targetH = constrain(servohori + round(deltaHoriz), servohoriLimitLow, servohoriLimitHigh);
    }

    if (targetV != servovert) {
      servovert = targetV;
      vertical.write(servovert);
    }

    if (targetH != servohori) {
      servohori = targetH;
      horizontal.write(servohori);
    }
  }

  // SENSOR + EFFICIENCY + BLYNK 
  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;

    float temp = dht.readTemperature();
    float hum  = dht.readHumidity();
    float dust = readDust();

    float lightSum = filt_lt + filt_rt + filt_ld + filt_rd;

    float expected = sin(radians(sunEl));
    float normalizedLight = lightSum / max(expected, 0.1f);

    if (sunEl > 50 && dust < 30) {
      cleanLight = 0.9 * cleanLight + 0.1 * normalizedLight;
    }

    float efficiency = normalizedLight / cleanLight;

    if (sunEl > 20 && efficiency < 0.7 && dust < 50) {
      dirtyCounter++;
    } else {
      dirtyCounter = 0;
    }

    if (dirtyCounter > 5) {
      Blynk.logEvent("dust_alert", "Panel needs cleaning!");
      dirtyCounter = 0;
    }

    // ALERTS 
    if (temp > TEMP_HIGH && !tempAlertSent) {
      Blynk.logEvent("high_temp", String("High Temperature: ") + String(temp) + "°C");
      tempAlertSent = true;
    } else if (temp <= TEMP_HIGH) {
      tempAlertSent = false;
    }

    if (hum > HUMIDITY_HIGH && !humidityAlertSent) {
      Blynk.logEvent("high_humidity", String("High Humidity: ") + String(hum) + "%");
      humidityAlertSent = true;
    } else if (hum <= HUMIDITY_HIGH) {
      humidityAlertSent = false;
    }

    if (dust > DUST_HIGH && !dustAlertSent) {
      Blynk.logEvent("high_dust", String("High Dust Level: ") + String(dust) + " µg/m³");
      dustAlertSent = true;
    } else if (dust <= DUST_HIGH) {
      dustAlertSent = false;
    }

    // SEND DATA TO BLYNK
    Blynk.virtualWrite(V0, temp);
    Blynk.virtualWrite(V1, hum);
    Blynk.virtualWrite(V2, dust);
    Blynk.virtualWrite(V3, servohori);
    Blynk.virtualWrite(V4, servovert);
    Blynk.virtualWrite(V6, efficiency * 100);
    Blynk.virtualWrite(V8, lightSum);

  }
}