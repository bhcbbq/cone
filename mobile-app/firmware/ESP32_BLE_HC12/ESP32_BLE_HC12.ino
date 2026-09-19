#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

HardwareSerial HC12(2);

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define RX_UUID      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *txCharacteristic;

class RXCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) override {

    String value = pCharacteristic->getValue().c_str();

    if (value.length() > 0) {
      Serial.print("Phone -> ESP32 : ");
      Serial.println(value);

      // BLE로 받은 명령을 HC-12로 그대로 전송
      HC12.println(value);

      Serial.print("ESP32 -> HC12 : ");
      Serial.println(value);
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    Serial.println("BLE CONNECTED");
  }

  void onDisconnect(BLEServer* pServer) override {
    Serial.println("BLE DISCONNECTED");

    delay(200);

    BLEDevice::startAdvertising();

    Serial.println("BLE ADVERTISING RESTARTED");
  }
};

void setup() {
  Serial.begin(115200);

  // HC-12
  HC12.begin(9600, SERIAL_8N1, 16, 17);

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
      BLECharacteristic::PROPERTY_WRITE
    );

  rxCharacteristic->setCallbacks(new RXCallback());

  // ESP32 -> Phone
  txCharacteristic =
    service->createCharacteristic(
      TX_UUID,
      BLECharacteristic::PROPERTY_NOTIFY
    );

  txCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("BLE + HC12 READY");
}

void loop() {

  // HC-12에서 들어오는 데이터 확인
  while (HC12.available()) {
    Serial.write(HC12.read());
  }
}