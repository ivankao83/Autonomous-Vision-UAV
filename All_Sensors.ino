
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_QMC5883P.h>
#include <Bitcraze_PMW3901.h>
#include <TinyGPSPlus.h>
#include <VL53L1X.h>

// pin config

// TFT display
#define TFT_CS    8
#define TFT_DC    9
#define TFT_RST   10
#define TFT_MOSI  11
#define TFT_SCK   12

#define BTN_UP    1
#define BTN_DOWN  2

// Main I2C bus
#define I2C_SDA   21
#define I2C_SCL   20

#define BMI_ADDR  0x69
#define MAG_ADDR  0x2C
#define BARO_ADDR 0x76

// PMW3901 
#define FLOW_MOSI 3
#define FLOW_SCK  18
#define FLOW_MISO 13
#define FLOW_CS   7
#define FLOW_RST  6

// VL53L1X
#define TOF_SDA   4
#define TOF_SCL   5

// GPS
#define GPS_RX    44
#define GPS_TX    43

#define SCREEN_W 128
#define SCREEN_H 160

//objects

SPIClass tftSPI(HSPI);

Adafruit_ST7735 tft(
  &tftSPI, TFT_CS, TFT_DC, TFT_RST
);

Bitcraze_PMW3901 flow(FLOW_CS);

Adafruit_BMP280 bmp(&Wire);
Adafruit_QMC5883P compass;

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

TwoWire tofWire = TwoWire(1);
VL53L1X tof;

GFXcanvas16 canvas(SCREEN_W, SCREEN_H);

//status

bool imuOK = false;
bool magOK = false;
bool baroOK = false;
bool flowOK = false;
bool gpsReceiving = false;
bool tofOK = false;
bool tofValid = false;

int currentPage = 0;
const int PAGE_COUNT = 7;

//sensors data

// BMI160
float roll = 0;
float pitch = 0;
float gyroZ = 0;

// Compass
float magX = 0;
float magY = 0;
float magZ = 0;
float heading = 0;

// Compass calibration
const float OFFSET_X = 0.330500f;
const float OFFSET_Y = -0.049500f;
const float OFFSET_Z = -2.268000f;

const float SCALE_X = 1.042189f;
const float SCALE_Y = 1.081657f;
const float SCALE_Z = 0.896078f;

// BMP280
float temperatureC = 0;
float pressureHPa = 0;
float relativeAltitude = 0;
float referencePressure = 0;

// PMW3901
int16_t rawFlowX = 0;
int16_t rawFlowY = 0;

float smoothFlowX = 0;
float smoothFlowY = 0;

long totalFlowX = 0;
long totalFlowY = 0;

// GPS
uint32_t lastGPSData = 0;

// VL53L1X
float distanceCM = 0;
float filteredDistanceCM = 0;

const int FILTER_SIZE = 3;
float distanceHistory[FILTER_SIZE] = {0};

int historyCount = 0;
int historyIndex = 0;

bool filterReady = false;

uint32_t lastToFReading = 0;

const float SMOOTH_ALPHA = 0.5f;

const uint32_t TOF_HOLD_MS = 1000;


// Display
float displayFPS = 0;
uint32_t fpsCount = 0;

// Timing
uint32_t lastIMU = 0;
uint32_t lastIMUTimeUs = 0;
uint32_t lastMag = 0;
uint32_t lastBaro = 0;
uint32_t lastFlow = 0;
uint32_t lastToF = 0;
uint32_t lastDisplay = 0;
uint32_t lastFPS = 0;
uint32_t lastButton = 0;

bool prevUp = HIGH;
bool prevDown = HIGH;

const uint16_t SKY = 0x159F;
const uint16_t GROUND = 0xA305;
const uint16_t BG = 0x0841;

const uint16_t GREEN = 0x07E0;
const uint16_t CYAN = 0x07FF;
const uint16_t WHITE = 0xFFFF;
const uint16_t YELLOW = 0xFFE0;
const uint16_t RED = 0xF800;
const uint16_t GRAY = 0x8410;

// BMI160

uint8_t bmiRead(uint8_t reg) {
  Wire.beginTransmission(BMI_ADDR);
  Wire.write(reg);

  if (Wire.endTransmission(false) != 0) {
    return 0xFF;
  }

  if (Wire.requestFrom(BMI_ADDR, 1) != 1) {
    return 0xFF;
  }

  return Wire.read();
}

void bmiWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(BMI_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool updateBMI160() {
  Wire.beginTransmission(BMI_ADDR);
  Wire.write(0x0C);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(BMI_ADDR, 12) != 12) {
    return false;
  }

  uint8_t b[12];

  for (int i = 0; i < 12; i++) {
    b[i] = Wire.read();
  }

  int16_t gx = (int16_t)((b[1] << 8) | b[0]);
  int16_t gy = (int16_t)((b[3] << 8) | b[2]);
  int16_t gz = (int16_t)((b[5] << 8) | b[4]);

  int16_t ax = (int16_t)((b[7] << 8) | b[6]);
  int16_t ay = (int16_t)((b[9] << 8) | b[8]);
  int16_t az = (int16_t)((b[11] << 8) | b[10]);

  float axg = ax / 16384.0f;
  float ayg = ay / 16384.0f;
  float azg = az / 16384.0f;

  float gxDps = gx / 16.4f;
  float gyDps = gy / 16.4f;

  gyroZ = gz / 16.4f;

  uint32_t nowUs = micros();

  float dt =
    (nowUs - lastIMUTimeUs) / 1000000.0f;

  lastIMUTimeUs = nowUs;

  if (dt <= 0 || dt > 0.2f) {
    dt = 0.01f;
  }

  float accRoll =
    atan2f(ayg, azg) * 180.0f / PI;

  float accPitch =
    atan2f(
      -axg,
      sqrtf(ayg * ayg + azg * azg)
    ) * 180.0f / PI;

  roll = 0.98f * (roll + gxDps * dt)
       + 0.02f * accRoll;

  pitch = 0.98f * (pitch + gyDps * dt)
        + 0.02f * accPitch;

  return true;
}

//compass

void updateCompass() {
  if (!magOK) return;

  float x, y, z;

  if (!compass.getGaussField(&x, &y, &z)) {
    return;
  }

  magX = (x - OFFSET_X) * SCALE_X;
  magY = (y - OFFSET_Y) * SCALE_Y;
  magZ = (z - OFFSET_Z) * SCALE_Z;

  float angle =
    atan2f(magY, magX) * 180.0f / PI;

  if (angle < 0) {
    angle += 360.0f;
  }

  heading = angle;
}

//barometer

void updateBarometer() {
  if (!baroOK) return;

  temperatureC = bmp.readTemperature();
  pressureHPa = bmp.readPressure() / 100.0f;

  if (!isfinite(pressureHPa) ||
      pressureHPa <= 0) {
    return;
  }

  if (referencePressure <= 0) {
    referencePressure = pressureHPa;
  }

  relativeAltitude =
    44330.0f *
    (1.0f - powf(
      pressureHPa / referencePressure,
      0.1903f
    ));
}

//PMW3901

void updateOpticalFlow() {
  if (!flowOK) return;

  flow.readMotionCount(&rawFlowX, &rawFlowY);

static uint32_t lastPrint = 0;

  if (millis() - lastPrint >= 200) {
    lastPrint = millis();

    Serial.print("Raw X: ");
    Serial.print(rawFlowX);

    Serial.print(" | Raw Y: ");
    Serial.println(rawFlowY);
  }

  // noise reduction
  const float alpha = 0.15f;
  const int deadzone = 4;
  const int spikeLimit = 80;

  int16_t x = rawFlowX;
  int16_t y = rawFlowY;

  if (abs((int)x) > spikeLimit ||
      abs((int)y) > spikeLimit) {
    return;
  }

  if (abs((int)x) <= deadzone) {
    x = 0;
  }

  if (abs((int)y) <= deadzone) {
    y = 0;
  }


  smoothFlowX =
    alpha * x +
    (1.0f - alpha) * smoothFlowX;

  smoothFlowY =
    alpha * y +
    (1.0f - alpha) * smoothFlowY;

  if (fabsf(smoothFlowX) < 1.0f) {
    smoothFlowX = 0;
  }

  if (fabsf(smoothFlowY) < 1.0f) {
    smoothFlowY = 0;
  }

  totalFlowX += rawFlowX;
  totalFlowY += rawFlowY;
}

//GPS

void updateGPS() {
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();

    lastGPSData = millis();
    gps.encode(c);
  }

  gpsReceiving =
    lastGPSData != 0 &&
    millis() - lastGPSData < 3000;
}

//VL53L1X


void updateToF() {
  if (!tofOK) return;

  if (!tof.dataReady()) return;

  uint16_t distanceMM = tof.read(false);

  if (tof.timeoutOccurred()) {
    tofValid = false;
    return;
  }

  if (tof.ranging_data.range_status !=
      VL53L1X::RangeValid || distanceMM == 0) {
    return;
  }

  float newDistance = distanceMM / 10.0f;

  
  if (newDistance < 2.0f ||
      newDistance > 600.0f) {
    return;
  }

  lastToFReading = millis();

  distanceHistory[historyIndex] = newDistance;

  historyIndex =
    (historyIndex + 1) % FILTER_SIZE;

  if (historyCount < FILTER_SIZE) {
    historyCount++;
  }

  float sorted[FILTER_SIZE];

  for (int i = 0; i < historyCount; i++) {
    sorted[i] = distanceHistory[i];
  }

  for (int i = 0; i < historyCount - 1; i++) {
    for (int j = i + 1; j < historyCount; j++) {
      if (sorted[j] < sorted[i]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  float median;

  if (historyCount % 2 == 1) {
    median = sorted[historyCount / 2];
  } else {
    median =
      (sorted[historyCount / 2 - 1] +
       sorted[historyCount / 2]) / 2.0f;
  }

  if (!filterReady) {
    filteredDistanceCM = median;
    filterReady = true;
  } else {
    filteredDistanceCM =
      SMOOTH_ALPHA * median +
      (1.0f - SMOOTH_ALPHA) * filteredDistanceCM;
  }

  distanceCM = filteredDistanceCM;
  tofValid = true;
}





void textAt(
  int x,
  int y,
  const char* label,
  uint16_t color = WHITE,
  int size = 1
) {
  canvas.setTextColor(color);
  canvas.setTextSize(size);
  canvas.setCursor(x, y);
  canvas.print(label);
}

void pageHeader(const char* title) {
  canvas.fillRect(
    0, 0, 128, 18, 0x18E3
  );

  textAt(4, 5, title);
}

void pageFooter() {
  canvas.drawFastHLine(
    0, 143, 128, GRAY
  );

  canvas.setTextSize(1);
  canvas.setTextColor(GRAY);
  canvas.setCursor(4, 149);

  canvas.print("PAGE ");
  canvas.print(currentPage + 1);
  canvas.print("/");
  canvas.print(PAGE_COUNT);
  canvas.print(" UP/DOWN");
}

//attitude

void drawAttitude() {
  float r = constrain(
    roll, -75.0f, 75.0f
  );

  float p = constrain(
    pitch, -40.0f, 40.0f
  );

  float slope =
    tanf(r * PI / 180.0f);

  float horizon = 78 + p;

  for (int x = 0; x < 128; x++) {
    int hy = (int)(
      horizon + (x - 64) * slope
    );

    hy = constrain(hy, 19, 114);

    canvas.drawFastVLine(
      x, 19, hy - 19, SKY
    );

    canvas.drawFastVLine(
      x, hy, 114 - hy, GROUND
    );
  }

  canvas.drawFastHLine(
    30, 75, 25, YELLOW
  );

  canvas.drawFastHLine(
    73, 75, 25, YELLOW
  );

  canvas.fillCircle(
    64, 75, 3, YELLOW
  );

  pageHeader("1 DRONE ATTITUDE");

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);

  canvas.setCursor(3, 118);
  canvas.print("Roll: ");
  canvas.print(roll, 1);

  canvas.setCursor(3, 130);
  canvas.print("Pitch: ");
  canvas.print(pitch, 1);
}

//compass

void drawCompass() {
  pageHeader("2 MAGNETOMETER");

  if (!magOK) {
    textAt(
      8, 65,
      "COMPASS OFFLINE",
      RED
    );
    return;
  }

  int cx = 64;
  int cy = 76;
  int radius = 38;

  canvas.drawCircle(
    cx, cy, radius, GRAY
  );

  canvas.drawCircle(
    cx, cy, radius - 1, GRAY
  );

  textAt(
    cx - 3, cy - radius - 11,
    "N", RED
  );

  textAt(
    cx - 3, cy + radius + 4,
    "S"
  );

  textAt(
    cx + radius + 4, cy - 3,
    "E"
  );

  textAt(
    cx - radius - 10, cy - 3,
    "W"
  );

  float rad =
    heading * PI / 180.0f;

  int tipX =
    cx + sinf(rad) * 29;

  int tipY =
    cy - cosf(rad) * 29;

  canvas.drawLine(
    cx, cy, tipX, tipY, RED
  );

  canvas.fillCircle(
    cx, cy, 4, WHITE
  );

  canvas.setTextColor(CYAN);
  canvas.setTextSize(1);
  canvas.setCursor(5, 124);

  canvas.print("Heading: ");
  canvas.print(heading, 1);
  canvas.print(" deg");

  textAt(
    5, 135,
    "LEVEL HEADING",
    YELLOW
  );
}

//barometer
void drawAltitude() {
  pageHeader("3 BAROMETER");

  if (!baroOK) {
    textAt(
      8, 65,
      "BMP280 OFFLINE",
      RED
    );
    return;
  }

  textAt(
    7, 29,
    "RELATIVE ALTITUDE",
    GRAY
  );

  canvas.setTextColor(CYAN);
  canvas.setTextSize(2);
  canvas.setCursor(7, 45);

  canvas.print(relativeAltitude, 2);
  canvas.print("m");

  canvas.drawFastHLine(
    7, 75, 114, GRAY
  );

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);

  canvas.setCursor(7, 85);
  canvas.print("Pressure:");

  canvas.setCursor(7, 97);
  canvas.print(pressureHPa, 1);
  canvas.print(" hPa");

  canvas.setCursor(7, 116);
  canvas.print("Temperature:");

  canvas.setCursor(7, 128);
  canvas.print(temperatureC, 1);
  canvas.print(" C");
}

//PMW3901

void drawOpticalFlow() {
  pageHeader("4 OPTICAL FLOW");

  if (!flowOK) {
    textAt(
      8, 65,
      "PMW3901 OFFLINE",
      RED
    );
    return;
  }

  canvas.setTextColor(GREEN);
  canvas.setTextSize(2);

  canvas.setCursor(4, 26);
  canvas.print("X:");
  canvas.print((int)smoothFlowX);

  canvas.setCursor(4, 52);
  canvas.print("Y:");
  canvas.print((int)smoothFlowY);

  int cx = 64;
  int cy = 108;

  canvas.drawCircle(
    cx, cy, 26, WHITE
  );

  canvas.drawFastHLine(
    cx - 4, cy, 9, GRAY
  );

  canvas.drawFastVLine(
    cx, cy - 4, 9, GRAY
  );

  int dx = constrain(
    (int)smoothFlowX, -23, 23
  );

  int dy = constrain(
    (int)smoothFlowY, -23, 23
  );

  canvas.fillCircle(
    cx + dx,
    cy + dy,
    4,
    RED
  );

  textAt(
    3, 136,
    "Filtered motion counts",
    GRAY
  );
}

//system check

void statusLine(
  int y,
  const char* label,
  bool status
) {
  textAt(8, y, label, WHITE);

  textAt(
    91, y,
    status ? "OK" : "FAIL",
    status ? GREEN : RED
  );
}

void drawSystem() {
  pageHeader("5 SYSTEM STATUS");

  statusLine(22, "BMI160", imuOK);
  statusLine(36, "QMC5883P", magOK);
  statusLine(50, "BMP280", baroOK);
  statusLine(64, "PMW3901", flowOK);
  statusLine(78, "GPS UART", gpsReceiving);
  statusLine(92, "VL53L1X", tofOK);

  canvas.drawFastHLine(
    5, 108, 118, GRAY
  );

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);

  canvas.setCursor(6, 116);
  canvas.print("Display: ");
  canvas.print(displayFPS, 1);
  canvas.print(" FPS");

  canvas.setCursor(6, 129);
  canvas.print("Uptime: ");
  canvas.print(millis() / 1000);
  canvas.print(" s");
}

//gps

void drawGPS() {
  pageHeader("6 GPS NEO-6M");

  bool gpsFix =
    gpsReceiving &&
    gps.location.isValid() &&
    gps.location.age() < 5000;

  textAt(
    5, 24,
    "UART:",
    WHITE
  );

  textAt(
    44, 24,
    gpsReceiving ? "RECEIVING" : "NO DATA",
    gpsReceiving ? GREEN : RED
  );

  textAt(
    5, 39,
    "Fix:",
    WHITE
  );

  textAt(
    44, 39,
    gpsFix ? "VALID" : "WAITING",
    gpsFix ? GREEN : YELLOW
  );

  canvas.setTextSize(1);
  canvas.setTextColor(WHITE);

  canvas.setCursor(5, 55);
  canvas.print("Satellites: ");

  if (gpsReceiving &&
      gps.satellites.isValid() &&
      gps.satellites.age() < 5000) {

    canvas.print(
      gps.satellites.value()
    );

  } else {
    canvas.print("--");
  }

  canvas.setCursor(5, 71);
  canvas.print("Lat: ");

  if (gpsFix) {
    canvas.print(
      gps.location.lat(), 6
    );
  } else {
    canvas.print("--");
  }

  canvas.setCursor(5, 86);
  canvas.print("Lon: ");

  if (gpsFix) {
    canvas.print(
      gps.location.lng(), 6
    );
  } else {
    canvas.print("--");
  }

  canvas.setCursor(5, 103);
  canvas.print("GPS Alt: ");

  if (gpsFix &&
      gps.altitude.isValid() &&
      gps.altitude.age() < 5000) {

    canvas.print(
      gps.altitude.meters(), 1
    );

  } else {
    canvas.print("--");
  }

  canvas.print(" m");

  canvas.setCursor(5, 118);
  canvas.print("Speed: ");

  if (gpsFix &&
      gps.speed.isValid() &&
      gps.speed.age() < 5000) {

    canvas.print(
      gps.speed.kmph(), 1
    );

  } else {
    canvas.print("--");
  }

  canvas.print(" km/h");
}

//VL53L1X 

void drawDistance() {
  pageHeader("7 DISTANCE SENSOR");

  if (!tofOK) {
    textAt(
      10, 60,
      "VL53L1X OFFLINE",
      RED
    );

    textAt(
      10, 78,
      "CHECK I2C WIRING",
      YELLOW
    );

    return;
  }


  bool fresh =
    filterReady &&
    millis() - lastToFReading < TOF_HOLD_MS;


  textAt(
    6, 27,
    "DISTANCE",
    GRAY
  );

  canvas.setTextSize(2);

  canvas.setTextColor(
    fresh ? CYAN : YELLOW
  );

  canvas.setCursor(6, 47);

  if (fresh) {
    canvas.print(distanceCM, 1);
  } else {
    canvas.print("---");
  }

  canvas.print(" cm");

  canvas.drawRect(
    6, 85, 116, 15, GRAY
  );

  if (fresh) {
    float barCM = constrain(
      distanceCM,
      0.0f,
      400.0f
    );

    int barWidth =
      (int)(barCM / 400.0f * 112.0f);

    canvas.fillRect(
      8, 87,
      barWidth, 11,
      GREEN
    );
  }

  textAt(
    6, 105,
    "0",
    GRAY
  );

  textAt(
    82, 105,
    "400+ cm",
    GRAY
  );

  textAt(
    6, 121,
    "STATUS:",
    WHITE
  );

  textAt(
    54, 121,
    fresh ? "VALID" : "NO TARGET",
    fresh ? GREEN : YELLOW
  );

  textAt(
    6, 134,
    "MODE: LONG",
    CYAN
  );
}

//display render

void renderScreen() {
  canvas.fillScreen(BG);

  switch (currentPage) {
    case 0:
      drawAttitude();
      break;

    case 1:
      drawCompass();
      break;

    case 2:
      drawAltitude();
      break;

    case 3:
      drawOpticalFlow();
      break;

    case 4:
      drawSystem();
      break;

    case 5:
      drawGPS();
      break;

    case 6:
      drawDistance();
      break;
  }

  pageFooter();

  tft.drawRGBBitmap(
    0, 0,
    canvas.getBuffer(),
    SCREEN_W,
    SCREEN_H
  );

  fpsCount++;

  uint32_t now = millis();

  if (now - lastFPS >= 1000) {
    displayFPS =
      fpsCount * 1000.0f /
      (now - lastFPS);

    Serial.print("FPS: ");
    Serial.println(displayFPS, 1);

    fpsCount = 0;
    lastFPS = now;
  }
}

//display buttons
void handleButtons() {
  bool up = digitalRead(BTN_UP);
  bool down = digitalRead(BTN_DOWN);

  uint32_t now = millis();

  if (now - lastButton >= 200) {

    if (up == LOW && prevUp == HIGH) {
      currentPage =
        (currentPage + PAGE_COUNT - 1)
        % PAGE_COUNT;

      lastButton = now;
    }

    else if (
      down == LOW &&
      prevDown == HIGH
    ) {
      currentPage =
        (currentPage + 1) % PAGE_COUNT;

      lastButton = now;
    }
  }

  prevUp = up;
  prevDown = down;
}


void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("DRONE TELEMETRY START");

  // ========== GPS UART ==========

  gpsSerial.setRxBufferSize(1024);

  gpsSerial.begin(
    9600,
    SERIAL_8N1,
    GPS_RX,
    GPS_TX
  );

  Serial.println(
    "GPS UART INITIALIZED"
  );

  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);

  pinMode(FLOW_CS, OUTPUT);
  digitalWrite(FLOW_CS, HIGH);

  tftSPI.begin(
    TFT_SCK,
    -1,
    TFT_MOSI,
    TFT_CS
  );

  tft.initR(INITR_BLACKTAB);
  tft.setRotation(0);
  tft.setSPISpeed(16000000);
  tft.fillScreen(ST77XX_BLACK);

  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(1);
  tft.setCursor(5, 20);
  tft.print("INITIALIZING...");

  Wire.begin(
    I2C_SDA,
    I2C_SCL
  );

  Wire.setClock(100000);
  Wire.setTimeOut(20);

  imuOK = (bmiRead(0x00) == 0xD1);

  if (imuOK) {
    bmiWrite(0x7E, 0x11);
    delay(100);

    bmiWrite(0x7E, 0x15);
    delay(100);
  }

  Serial.println(
    imuOK ? "BMI160 OK" : "BMI160 FAILED"
  );

  magOK =
    compass.begin(MAG_ADDR, &Wire);

  if (magOK) {
    compass.setMode(
      QMC5883P_MODE_CONTINUOUS
    );

    compass.setODR(
      QMC5883P_ODR_50HZ
    );
  }

  Serial.println(
    magOK ? "QMC5883P OK" :
            "QMC5883P FAILED"
  );

  baroOK = bmp.begin(BARO_ADDR);

  if (baroOK) {
    bmp.setSampling(
      Adafruit_BMP280::MODE_NORMAL,
      Adafruit_BMP280::SAMPLING_X2,
      Adafruit_BMP280::SAMPLING_X16,
      Adafruit_BMP280::FILTER_X16,
      Adafruit_BMP280::STANDBY_MS_63
    );

    delay(150);

    referencePressure =
      bmp.readPressure() / 100.0f;
  }

  Serial.println(
    baroOK ? "BMP280 OK" :
             "BMP280 FAILED"
  );

  pinMode(FLOW_RST, OUTPUT);
  digitalWrite(FLOW_RST, HIGH);

  SPI.begin(
    FLOW_SCK,
    FLOW_MISO,
    FLOW_MOSI,
    FLOW_CS
  );

  flowOK = flow.begin();

  Serial.println(
    flowOK ? "PMW3901 OK" :
             "PMW3901 FAILED"
  );

  tofWire.begin(
    TOF_SDA,
    TOF_SCL
  );

  tofWire.setClock(100000);
  tofWire.setTimeOut(20);

  tof.setBus(&tofWire);
  tof.setTimeout(100);

  tofOK = tof.init();


  if (tofOK) {

    bool modeOK =
      tof.setDistanceMode(VL53L1X::Long);

    bool timingOK =
      tof.setMeasurementTimingBudget(50000);

    if (modeOK && timingOK) {

      tof.startContinuous(50);

      Serial.println("VL53L1X OK");

    } else {
      tofOK = false;
      Serial.println("VL53L1X CONFIG FAILED");
    }
  

  } else {
    Serial.println("VL53L1X FAILED");
    Serial.println(
      "Check SDA 4 / SCL 5"
    );
  }

  uint32_t now = millis();

  lastIMU = now;
  lastIMUTimeUs = micros();
  lastMag = now;
  lastBaro = now;
  lastFlow = now;
  lastToF = now;
  lastDisplay = now;
  lastFPS = now;

  Serial.println("SYSTEM READY");

  renderScreen();
}

void loop() {
  uint32_t now = millis();

  handleButtons();

  updateGPS();

  if (imuOK && now - lastIMU >= 10) {
    lastIMU = now;
    updateBMI160();
  }

  if (magOK && now - lastMag >= 50) {
    lastMag = now;
    updateCompass();
  }

  if (baroOK && now - lastBaro >= 100) {
    lastBaro = now;
    updateBarometer();
  }

  if (flowOK && now - lastFlow >= 50) {
    lastFlow = now;
    updateOpticalFlow();
  }

  if (tofOK && now - lastToF >= 5) {
    lastToF = now;
    updateToF();
  }

  if (now - lastDisplay >= 50) {
    lastDisplay = now;
    renderScreen();
  }

  delay(1);
}
