#include "bluetooth.hpp"

volatile bool bluetoothStart = false;
volatile bool bluetoothStop = false;
volatile RobotCommand bluetoothCommand = CMD_NONE;

#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_RX_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_TX_UUID "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* server = nullptr;
BLEService* service = nullptr;
BLECharacteristic* txCharacteristic = nullptr;
BLECharacteristic* rxCharacteristic = nullptr;

bool deviceConnected = false;

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) override {
        deviceConnected = true;
        Serial.println("BLE Connected");
    }

    void onDisconnect(BLEServer* pServer) override {
        deviceConnected = false;
        Serial.println("BLE Disconnected");

        delay(100);
        BLEDevice::startAdvertising();
    }
};

class RXCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) override {
        String cmd = characteristic->getValue().c_str();

        cmd.trim();
        cmd.toUpperCase();

        debugPrintf("RX: %s", cmd.c_str());

        if (cmd == "START") {
            bluetoothStart = true;
        } else if (cmd == "STOP") {
            bluetoothStop = true;
            bluetoothCommand = CMD_STOP;
        } else if (cmd == "FORWARD") {
            bluetoothCommand = CMD_FORWARD;
        } else if (cmd == "LEFT") {
            bluetoothCommand = CMD_LEFT;
        } else if (cmd == "RIGHT") {
            bluetoothCommand = CMD_RIGHT;
        } else if (cmd == "BACK") {
            bluetoothCommand = CMD_BACK;
        } else if (cmd == "RESET") {
            debugPrint("Restarting...");
            delay(100);
            ESP.restart();
        }
    }
};

void beginBluetooth() {
    BLEDevice::init("GCM Micromouse");

    server = BLEDevice::createServer();
    server->setCallbacks(new ServerCallbacks());

    service = server->createService(SERVICE_UUID);

    txCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_TX_UUID,
        BLECharacteristic::PROPERTY_NOTIFY
    );
    txCharacteristic->addDescriptor(new BLE2902());

    rxCharacteristic = service->createCharacteristic(
        CHARACTERISTIC_RX_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    rxCharacteristic->setCallbacks(new RXCallbacks());

    service->start();

    BLEAdvertising* advertising = BLEDevice::getAdvertising();
    advertising->addServiceUUID(SERVICE_UUID);
    advertising->setScanResponse(true);
    advertising->setMinPreferred(0x06);
    advertising->setMinPreferred(0x12);

    BLEDevice::startAdvertising();

    Serial.println("BLE Advertising Started");
}

void debugPrint(const char* msg) {
    Serial.println(msg);

    if (deviceConnected && txCharacteristic) {
        txCharacteristic->setValue((uint8_t*)msg, strlen(msg));
        txCharacteristic->notify();
    }
}

void debugPrintf(const char* format, ...) {
    char buffer[256];

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    debugPrint(buffer);
}