/*
 * GCM V1 — Bluetooth test
 * Sends "Hello" every second over BLE (Nordic UART Service) and over the TX/RX pins.
 *
 * The ESP32-S3 only has Bluetooth LE — classic Bluetooth serial (BluetoothSerial.h)
 * does NOT exist on this chip. Use a BLE terminal app on the phone:
 *   Android: "Serial Bluetooth Terminal" (Kai Morich)  ·  iOS: "Bluefruit Connect" (UART)
 * Connect to "GCM" and you'll see Hello once per second.
 */
#include <BLEDevice.h>
#include <BLEServer.h>
#ifndef CONFIG_BT_NIMBLE_ENABLED
#include <BLE2902.h>          // Bluedroid needs it; NimBLE adds it automatically
#endif

#define NUS_SERVICE "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_RX      "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"   // phone → robot
#define NUS_TX      "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"   // robot → phone

BLECharacteristic *tx;
bool connected = false;

class Conn : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { connected = true; }
  void onDisconnect(BLEServer *s) override { connected = false; s->startAdvertising(); }
};

void setup() {
  Serial.begin(115200);                      // USB
  Serial0.begin(115200);                     // TX/RX pins (TXD0 = GPIO43, RXD0 = GPIO44)

  BLEDevice::init("GCM");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new Conn());
  BLEService *svc = server->createService(NUS_SERVICE);
  tx = svc->createCharacteristic(NUS_TX, BLECharacteristic::PROPERTY_NOTIFY);
#ifndef CONFIG_BT_NIMBLE_ENABLED
  tx->addDescriptor(new BLE2902());
#endif
  svc->createCharacteristic(NUS_RX, BLECharacteristic::PROPERTY_WRITE);
  svc->start();
  BLEDevice::getAdvertising()->addServiceUUID(NUS_SERVICE);
  BLEDevice::startAdvertising();
}

void loop() {
  Serial.println("Hello");
  Serial0.println("Hello");
  if (connected) { tx->setValue("Hello\n"); tx->notify(); }
  delay(1000);
}
