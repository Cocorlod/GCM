#include "bluetooth.h"

#define SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic* pTxCharacteristic = nullptr;
bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* server) override {
        deviceConnected = true;
    }

    void onDisconnect(BLEServer* server) override {
        deviceConnected = false;
        BLEDevice::startAdvertising();
    }
};

void bluetoothInit() {
    BLEDevice::init("Micromouse");

    BLEServer* pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService* pService = pServer->createService(SERVICE_UUID);

    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY
    );

    pTxCharacteristic->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();
}

void debugPrint(const char* format, ...) {
    char buffer[512];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    Serial.print(buffer);

    if (!deviceConnected || pTxCharacteristic == nullptr) {
        return;
    }

    size_t length = strlen(buffer);

    const size_t chunkSize = 180;

    for (size_t i = 0; i < length; i += chunkSize) {
        size_t remaining = length - i;
        size_t currentSize = remaining < chunkSize ? remaining : chunkSize;

        String chunk = String(buffer + i).substring(0, currentSize);

        pTxCharacteristic->setValue(chunk.c_str());
        pTxCharacteristic->notify();

        delay(2);
    }
}