#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <driver/ledc.h>
#include <DHT.h>

// -------------------- DHT SENSOR --------------------
#define DHT_PIN 19
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);

float T = 0, H = 0;

// -------------------- SENSOR PINS --------------------
#define SOIL_PIN 36
#define LIGHT_PIN 39 

// -------------------- PUMP DRIVER --------------------
#define PUMP_ENA 26
#define PUMP_IN1 32
#define PUMP_IN2 33

// -------------------- LED DRIVER ---------------------
#define LED_ENB 14
#define LED_IN3 13
#define LED_IN4 12

// -------------------- PWM CONFIG ---------------------
#define PWM_FREQ 5000
#define PWM_RES  8

// -------------------- BLE CONFIG ---------------------
#define DEVICE_NAME "SmartGardenESP32"
#define SERVICE_UUID "4e520001-3c8c-4824-8f7d-2b4421d8b63e"
#define DATA_UUID    "4e520002-3c8c-4824-8f7d-2b4421d8b63e"
#define CMD_UUID     "4e520003-3c8c-4824-8f7d-2b4421d8b63e"

BLECharacteristic *pDataChar;
bool deviceConnected = false;

// -------------------- THRESHOLDS ---------------------
int soilThresh  = 1700;
int lightThresh = 1000;

// -------------------- DUTY VALUES --------------------
int pumpDuty = 180;
int ledDuty  = 255;

// -------------------- STATES -------------------------
bool pumpOnState = false;
bool ledOnState  = false;
bool manualOverride = false;

// -------------------- SENSOR VALUES ------------------
int M = 0, L = 0;

// -------------------- APPLY PUMP ---------------------
void pumpApply(bool state) {
  pumpOnState = state;
  pumpDuty = constrain(pumpDuty, 0, 255);

  if (state) {
    digitalWrite(PUMP_IN1, HIGH);
    digitalWrite(PUMP_IN2, LOW);
    ledcWrite(PUMP_ENA, pumpDuty);
  } else {
    digitalWrite(PUMP_IN1, LOW);
    digitalWrite(PUMP_IN2, LOW);
    ledcWrite(PUMP_ENA, 0);
  }
}

// -------------------- APPLY LED ----------------------
void ledApply(bool state) {
  ledOnState = state;
  ledDuty = constrain(ledDuty, 0, 255);

  if (state) {
    digitalWrite(LED_IN3, HIGH);
    digitalWrite(LED_IN4, LOW);
    ledcWrite(LED_ENB, ledDuty);
  } else {
    digitalWrite(LED_IN3, LOW);
    digitalWrite(LED_IN4, LOW);
    ledcWrite(LED_ENB, 0);
  }
}

// -------------------- READ SENSORS -------------------
void readSensors() {
  M = map(analogRead(SOIL_PIN), 0, 4095, 4095, 0);
  L = analogRead(LIGHT_PIN);

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (!isnan(t)) T = t;
  if (!isnan(h)) H = h;
}

// -------------------- AUTOMATION ---------------------
void runAutomation() {
  if (manualOverride) return;

  bool dry  = M < soilThresh;
  bool dark = L < lightThresh;

  pumpApply(dry);
  ledApply(dark);
}

/**
 * @brief Send a command to the computer connected over serial.
 *
 * This function takes a command as a string and sends it over the serial connection
 * to the computer. The command will be sent as a JSON formatted string.
 *
 * @param command The command to send to the computer.
 */
void send_reading() {
  Serial.println(makeJSON());
}

// -------------------- JSON ---------------------------
String makeJSON() {
  return String("{\"M\":") + M +
         ",\"L\":" + L +
         ",\"T\":" + T +
         ",\"H\":" + H +
         ",\"P\":" + pumpDuty +
         ",\"P_ON\":" + (pumpOnState ? 1 : 0) +
         ",\"S\":" + ledDuty +
         ",\"S_ON\":" + (ledOnState ? 1 : 0) +
         ",\"O\":" + (manualOverride ? 1 : 0) +
         "}";
}

// -------------------- BLE CALLBACKS ------------------
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *p) { deviceConnected = true; }
  void onDisconnect(BLEServer *p) { deviceConnected = false; p->startAdvertising(); }
};

class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) {
    String cmd = String(c->getValue().c_str());
    Serial.println("CMD: " + cmd);

    if (cmd == "OVERRIDE_ON")  manualOverride = true;
    if (cmd == "OVERRIDE_OFF") manualOverride = false;

    if (cmd == "PUMP_ON")  pumpApply(true);
    if (cmd == "PUMP_OFF") pumpApply(false);
    if (cmd == "LED_ON")   ledApply(true);
    if (cmd == "LED_OFF")  ledApply(false);

    int idx = cmd.indexOf(':');
    if (idx > 0) {
      String key = cmd.substring(0, idx);
      float val  = cmd.substring(idx + 1).toFloat();

      if (key == "SET_MOISTURE_THRESH") soilThresh  = (int)val;
      if (key == "SET_LIGHT_THRESH")    lightThresh = (int)val;

      if (key == "SET_PUMP_SPEED") {
        pumpDuty = (int)val;
        pumpApply(pumpOnState);
      }

      if (key == "SET_LED_BRIGHTNESS") {
        ledDuty = (int)val;
        ledApply(ledOnState);
      }
    }

    pDataChar->setValue(makeJSON().c_str());
    pDataChar->notify();
  }
};

// -------------------- SETUP --------------------------
void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(PUMP_IN1, OUTPUT);
  pinMode(PUMP_IN2, OUTPUT);
  pinMode(LED_IN3, OUTPUT);
  pinMode(LED_IN4, OUTPUT);

  ledcSetClockSource(LEDC_USE_APB_CLK);
  ledcAttach(PUMP_ENA, PWM_FREQ, PWM_RES);
  ledcAttach(LED_ENB, PWM_FREQ, PWM_RES);

  pumpApply(false);
  ledApply(false);

  BLEDevice::init(DEVICE_NAME);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());

  BLEService *svc = server->createService(SERVICE_UUID);

  pDataChar = svc->createCharacteristic(
    DATA_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pDataChar->addDescriptor(new BLE2902());

  BLECharacteristic *cmd = svc->createCharacteristic(
    CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmd->setCallbacks(new CmdCB());

  svc->start();
  server->getAdvertising()->start();

  Serial.println("Smart Garden BLE Ready.");
}

// -------------------- LOOP ---------------------------
void loop() {
  static unsigned long lastRead = 0;

  send_reading();

  if (millis() - lastRead > 2000) {
    readSensors();
    runAutomation();

    if (deviceConnected) {
      String json = makeJSON();
      pDataChar->setValue(json.c_str());
      pDataChar->notify();
      Serial.println("Sent: " + json);
    }
    lastRead = millis();
  }
}