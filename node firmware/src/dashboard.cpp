//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DASHBOARD (BLUETOOTH LOW ENERGY)
//*************************************************************************************************************************************************

#include "system.h"
#include "dashboard.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

// Bluetooth Low Energy
BLEServer *bleServer = NULL;
BLECharacteristic *bleTx = NULL;

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// Setup
bool dashboardOK = false;
bool dashConnected = false;
int pendingDeliveries = 0;
float battery  = 0;

// Received data (written by the BLE task, read by dashRx())
String dashRxBuffer = "";
SemaphoreHandle_t dashRxMutex = NULL;

//*************************************************************************************************************************************************
// CALLBACKS
//*************************************************************************************************************************************************

// Tracks the dashboard connection
class DashServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) {
    dashConnected = true;
  }

  void onDisconnect(BLEServer *server) {
    dashConnected = false;
    server->startAdvertising();   // allow the dashboard to reconnect
  }
};

// Stores data written by the dashboard
class DashRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) {
    std::string value = characteristic->getValue();
    if (xSemaphoreTake(dashRxMutex, portMAX_DELAY) == pdTRUE) {
      dashRxBuffer += value.c_str();
      xSemaphoreGive(dashRxMutex);
    }
  }
};

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup Bluetooth Low Energy communication to TUKZIE dashboard
// Returns true if successful
bool dashInit(void) {

  dashRxMutex = xSemaphoreCreateMutex();
  if (dashRxMutex == NULL) {
    error = true;
    errMsg += "BLE: Rx mutex could not be created\n";
    return false;
  }

  BLEDevice::init(DASH_NAME);
  if (!BLEDevice::getInitialized()) {
    error = true;
    errMsg += "BLE: Bluetooth stack failed to start\n";
    return false;
  }

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new DashServerCallbacks());

  BLEService *service = bleServer->createService(NUS_SERVICE_UUID);

  // Node -> dashboard
  bleTx = service->createCharacteristic(NUS_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  bleTx->addDescriptor(new BLE2902());

  // Dashboard -> node
  BLECharacteristic *bleRx = service->createCharacteristic(NUS_RX_UUID,
                               BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  bleRx->setCallbacks(new DashRxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(NUS_SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  dashboardOK = true;
  return true;
}

// Transmits a table of nodes to the TUKZIE dashboard
// One node per line
// Returns false if no dashboard is connected
bool dashTx(Data table[], int size) {

  if (!dashConnected) {
    return false;
  }

  String payload = "";
  for (int i = 0; i < size; i++) {
    payload += node2Str(table[i]) + "\n";
  }

  // Send in chunks that fit within a single notification
  for (unsigned int i = 0; i < payload.length(); i += BLE_CHUNK) {
    size_t len = min((unsigned int)BLE_CHUNK, payload.length() - i);
    bleTx->setValue((uint8_t *)(payload.c_str() + i), len);
    bleTx->notify();
    delay(5);   // give the BLE stack time to send each chunk
  }
  return true;
}

// Receives and parses a packet from the TUKZIE dashboard
// Packets are terminated by '\n'
// Returns true when a complete packet has been parsed
bool dashRx(void) {

  String line = "";
  bool complete = false;

  if (xSemaphoreTake(dashRxMutex, 0) == pdTRUE) {
    int end = dashRxBuffer.indexOf('\n');
    if (end >= 0) {
      line = dashRxBuffer.substring(0, end);
      dashRxBuffer = dashRxBuffer.substring(end + 1);
      complete = true;
    }
    xSemaphoreGive(dashRxMutex);
  }

  if (!complete) {
    return false;
  }

  line.trim();   // drops any trailing '\r'

  int got = sscanf(line.c_str(),
  "%d,"                   // status
  "%d,"                   // deliveries
  "%f,"                   // battery
  "%d",                   // message
  &myself.status,
  &myself.tasks,
  &myself.battery,
  &myself.message);

  return got == 4;
}
