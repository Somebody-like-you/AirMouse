#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEHIDDevice.h>


#define SDA_PIN   21
#define SCL_PIN   22
#define BTN_LEFT  25
#define BTN_RIGHT 26


const float SENSITIVITY = 30.0;
const float DEADZONE    = 0.6;
const float SMOOTHING   = 0.7;
const int   POINT_AXIS  = 0;
const bool  INVERT_X    = true;
const bool  INVERT_Y    = false;
const int   LOOP_MS     = 8;



static const uint8_t HID_REPORT_MAP[] = {
  0x05, 0x01,
  0x09, 0x02,
  0xA1, 0x01,
  0x09, 0x01,
  0xA1, 0x00,
  0x85, 0x01,
  0x05, 0x09,
  0x19, 0x01,
  0x29, 0x03,
  0x15, 0x00,
  0x25, 0x01,
  0x95, 0x03,
  0x75, 0x01,
  0x81, 0x02,
  0x95, 0x01,
  0x75, 0x05,
  0x81, 0x03,
  0x05, 0x01,
  0x09, 0x30,
  0x09, 0x31,
  0x15, 0x81,
  0x25, 0x7F,
  0x75, 0x08,
  0x95, 0x02,
  0x81, 0x06,
  0xC0,
  0xC0
};

BLEHIDDevice*      hid        = nullptr;
BLECharacteristic* inputMouse = nullptr;
BLEServer*         bleServer  = nullptr;

volatile bool bleConnected  = false;
volatile bool needAdvertise = false;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s, esp_ble_gatts_cb_param_t* param) {
    bleConnected = true;
    Serial.println("BLE connected");


    esp_ble_conn_update_params_t cp;
    memcpy(cp.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
    cp.min_int = 0x06;
    cp.max_int = 0x0C;
    cp.latency = 0;
    cp.timeout = 400;
    esp_ble_gap_update_conn_params(&cp);
  }
  void onDisconnect(BLEServer* s) {
    bleConnected = false;
    needAdvertise = true;
    Serial.println("BLE disconnected");
  }
};

void bleMouseBegin() {
  BLEDevice::init("ESP32 Air Mouse");

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  hid = new BLEHIDDevice(bleServer);
  inputMouse = hid->inputReport(1);

  hid->manufacturer()->setValue("DIY");
  hid->pnp(0x02, 0xE502, 0xA111, 0x0210);
  hid->hidInfo(0x00, 0x01);
  hid->reportMap((uint8_t*)HID_REPORT_MAP, sizeof(HID_REPORT_MAP));


  BLESecurity* sec = new BLESecurity();
  sec->setAuthenticationMode(ESP_LE_AUTH_BOND);
  sec->setCapability(ESP_IO_CAP_NONE);
  sec->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  hid->startServices();
  hid->setBatteryLevel(100);

  BLEAdvertising* adv = bleServer->getAdvertising();
  adv->setAppearance(0x03C2);
  adv->addServiceUUID(hid->hidService()->getUUID());
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x0C);
  adv->start();
}


void sendReport(uint8_t buttons, int8_t dx, int8_t dy) {
  if (!bleConnected) return;
  uint8_t report[3] = { buttons, (uint8_t)dx, (uint8_t)dy };
  inputMouse->setValue(report, sizeof(report));
  inputMouse->notify();
}


#define MPU_ADDR 0x68
const float GYRO_SCALE = 65.5;

float offX = 0, offY = 0, offZ = 0;
float smX = 0, smY = 0;
float remX = 0, remY = 0;

unsigned long lastLoop = 0;


#define IDX_L 0
#define IDX_R 1

const uint8_t BTN_PINS[2]     = {BTN_LEFT, BTN_RIGHT};
bool          btnState[2]     = {false, false};
bool          btnLastRaw[2]   = {false, false};
unsigned long btnChangedAt[2] = {0, 0};

void updateButton(uint8_t i) {
  bool raw = (digitalRead(BTN_PINS[i]) == LOW);
  if (raw != btnLastRaw[i]) {
    btnLastRaw[i] = raw;
    btnChangedAt[i] = millis();
  }
  if (millis() - btnChangedAt[i] > 15) {
    btnState[i] = raw;
  }
}


void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}


bool readAll(int16_t a[3], int16_t g[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 14) != 14) return false;
  for (int i = 0; i < 3; i++) a[i] = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();
  for (int i = 0; i < 3; i++) g[i] = (Wire.read() << 8) | Wire.read();
  return true;
}

void calibrate() {
  const int N = 200;
  float sx = 0, sy = 0, sz = 0;
  int got = 0;
  for (int i = 0; i < N; i++) {
    int16_t a[3], g[3];
    if (readAll(a, g)) {
      sx += g[0]; sy += g[1]; sz += g[2];
      got++;
    }
    delay(5);
  }
  if (got > 0) {
    offX = sx / got;
    offY = sy / got;
    offZ = sz / got;
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(BTN_LEFT,  INPUT_PULLUP);
  pinMode(BTN_RIGHT, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  writeReg(0x6B, 0x00);
  writeReg(0x19, 0x07);
  writeReg(0x1A, 0x01);
  writeReg(0x1B, 0x08);

  delay(500);
  calibrate();

  bleMouseBegin();
  Serial.println("READY - pair 'ESP32 Air Mouse' in your Bluetooth settings");
  lastLoop = millis();
}


void loop() {

  if (needAdvertise) {
    needAdvertise = false;
    delay(300);
    bleServer->getAdvertising()->start();
    Serial.println("Advertising again");
  }

  if (millis() - lastLoop < LOOP_MS) return;
  float dt = (millis() - lastLoop) / 1000.0;
  lastLoop = millis();

  updateButton(IDX_L);
  updateButton(IDX_R);

  int16_t ra[3], rg[3];
  if (!readAll(ra, rg)) return;


  float g[3];
  g[0] = (rg[0] - offX) / GYRO_SCALE;
  g[1] = (rg[1] - offY) / GYRO_SCALE;
  g[2] = (rg[2] - offZ) / GYRO_SCALE;


  static float G[3] = {0, 0, 0};
  static bool gInit = false;
  float alen = sqrt((float)ra[0] * ra[0] + (float)ra[1] * ra[1] + (float)ra[2] * ra[2]);
  if (alen < 1) return;
  for (int i = 0; i < 3; i++) {
    float n = ra[i] / alen;
    G[i] = gInit ? (G[i] * 0.95f + n * 0.05f) : n;
  }
  gInit = true;
  float glen = sqrt(G[0] * G[0] + G[1] * G[1] + G[2] * G[2]);
  if (glen < 0.01f) return;
  float gu[3] = {G[0] / glen, G[1] / glen, G[2] / glen};


  float yaw = g[0] * gu[0] + g[1] * gu[1] + g[2] * gu[2];


  float P[3] = {0, 0, 0};
  P[POINT_AXIS] = 1;
  float L[3] = {
    P[1] * gu[2] - P[2] * gu[1],
    P[2] * gu[0] - P[0] * gu[2],
    P[0] * gu[1] - P[1] * gu[0]
  };
  float llen = sqrt(L[0] * L[0] + L[1] * L[1] + L[2] * L[2]);
  float pitch = 0;
  if (llen > 0.2f) {
    pitch = (g[0] * L[0] + g[1] * L[1] + g[2] * L[2]) / llen;
  }

  float h = yaw;
  float v = pitch;
  if (INVERT_X) h = -h;
  if (INVERT_Y) v = -v;


  if (fabs(h) < DEADZONE) h = 0;
  if (fabs(v) < DEADZONE) v = 0;


  smX = smX * (1.0f - SMOOTHING) + h * SMOOTHING;
  smY = smY * (1.0f - SMOOTHING) + v * SMOOTHING;


  remX += smX * SENSITIVITY * dt;
  remY += smY * SENSITIVITY * dt;
  int dx = (int)remX;  remX -= dx;
  int dy = (int)remY;  remY -= dy;


  if (dx > 127)  { remX += dx - 127;  dx = 127; }
  if (dx < -127) { remX += dx + 127;  dx = -127; }
  if (dy > 127)  { remY += dy - 127;  dy = 127; }
  if (dy < -127) { remY += dy + 127;  dy = -127; }


  static uint8_t lastBtn = 0;
  uint8_t buttons = (btnState[IDX_L] ? 1 : 0) | (btnState[IDX_R] ? 2 : 0);

  if (dx != 0 || dy != 0 || buttons != lastBtn) {
    sendReport(buttons, (int8_t)dx, (int8_t)dy);
    lastBtn = buttons;
  }
}


