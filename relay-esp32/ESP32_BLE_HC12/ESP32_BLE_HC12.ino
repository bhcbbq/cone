#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial HC12(2);

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *txCharacteristic;

bool deviceConnected = false;
String hc12Buffer = "";

class RXCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {

    String value = pCharacteristic->getValue().c_str();
    value.trim();

    if (value.length() > 0) {
      Serial.print("Phone -> ESP32 : ");
      Serial.println(value);

      HC12.println(value);

      Serial.print("ESP32 -> HC12 : ");
      Serial.println(value);
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    deviceConnected = true;
    Serial.println("BLE CONNECTED");
  }

  void onDisconnect(BLEServer *pServer) override {
    deviceConnected = false;
    Serial.println("BLE DISCONNECTED");

    delay(200);
    BLEDevice::startAdvertising();

    Serial.println("BLE ADVERTISING RESTARTED");
  }
};

void sendToPhone(String message) {
  if (!deviceConnected) return;

  txCharacteristic->setValue(message.c_str());
  txCharacteristic->notify();

  Serial.print("ESP32 -> Phone : ");
  Serial.println(message);
}

void setup() {
  Serial.begin(115200);

  // HC-12
  HC12.begin(115200, SERIAL_8N1, 16, 17);

  // BLE
  BLEDevice::init("ESP32_CONE");

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service =
    server->createService(SERVICE_UUID);

  // Phone -> ESP32
  BLECharacteristic *rxCharacteristic =
    service->createCharacteristic(
      RX_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);

  rxCharacteristic->setCallbacks(new RXCallback());

  // ESP32 -> Phone
  txCharacteristic =
    service->createCharacteristic(
      TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY);

  txCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->start();

  Serial.println("BLE + HC12 READY");
}

void loop() {

  while (HC12.available()) {
    char c = HC12.read();

    if (c == '\n') {
      hc12Buffer.trim();

      if (hc12Buffer.length() > 0) {
        Serial.print("HC12 -> ESP32 : ");
        Serial.println(hc12Buffer);

        // HC-12 응답이 실제로 들어왔으므로 연결됨으로 판단
        sendToPhone("HC12_CONNECTED");

        // 원래 메시지도 그대로 앱으로 전달
        sendToPhone(hc12Buffer);
      }

      hc12Buffer = "";
    } else if (c != '\r') {
      hc12Buffer += c;
    }
  }

  delay(1);
}