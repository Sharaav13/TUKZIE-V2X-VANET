//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - LoRa RADIO
//*************************************************************************************************************************************************

#include "LoRaRadio.h"
#include "GPS.h"       // datetime used by loraRxStr()/loraTxStr()
#include "Display.h"   // loraDebug() calls disp()

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

SPIClass spiLoRa(HSPI);

//*************************************************************************************************************************************************
// GLOBAL VARIABLES
//*************************************************************************************************************************************************

// LoRa
String message; // Make them volatile for interrupts.
char messageTx[FRAME_CSV_MAX];
char messageRx[FRAME_CSV_MAX];
int rssi;
float snr;
long freqErr;
uint32_t lastBeacon = 0;
// volatile bool cadBusy = false;
// volatile bool cadPending = false;
// uint32_t seq = 0;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// Setup LoRa
String loraInit(void) {

  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_NSS);
  LoRa.setSPI(spiLoRa);
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  LoRa.setSPIFrequency(1E6);        // 1E6 for breadboarding otherwise 8E6

  if (LoRa.begin(LORA_FREQ)) {

    LoRa.setSpreadingFactor(LORA_SF); // 12
    LoRa.setSignalBandwidth(LORA_BW); //125E3
    LoRa.setCodingRate4(LORA_CR);     // 8
    // LoRa.setPreambleLength(LORA_PREAMBLE);
    // LoRa.setSyncWord(LORA_SYNC);
    LoRa.setTxPower(LORA_TXPOWER, PA_OUTPUT_PA_BOOST_PIN);    // PA_BOOST output on the RA-02
    LoRa.enableCrc();
    LoRa.setSyncWord(0x12); // #
    // LoRa.onCadDone(onCadDoneISR);

    randomSeed(esp_random());
    lastBeacon = millis() - random(0, BEACON_INTVL);

    return "LoRa init pass.\n";
  } else {
    error = true;
    return "LoRa init failed.\n";
  }
}

// Procedure for receiving LoRa packet
String loraRx(void) {

  String msg = "";

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    while (LoRa.available()) {
      msg += String((char)LoRa.read());
    }
    rssi = LoRa.packetRssi();
    snr =  LoRa.packetSnr();
    freqErr = LoRa.packetFrequencyError();
  }
  return msg;
}

// Procedure for transmitting LoRa packet
void loraTx(String payload) {

  if (!LoRa.beginPacket()) {
    return; // radio busy
  } else {
    LoRa.print(payload);
    LoRa.endPacket(true); // true = async, completion via DIO0
  }
}

// Converts LoRa Rx information into a single string
size_t loraRxStr(char *out, size_t cap, String content) {
  return snprintf(out, cap,
        "%02d-%02d-%04d,\n"       // date
        "%02d:%02d:%02d,%lu,\n"   // time
        "%d,%.2f,%ld\n"           // RX diagnostics
        "%s\n",                 // received payload
        datetime[DAY], datetime[MONTH], datetime[YEAR],
        datetime[HOUR], datetime[MINUTE], datetime[SECOND], millis(),
        rssi, snr, freqErr,
        message.c_str());
}

// Converts LoRa Tx information into a single string
size_t loraTxStr(char *out, size_t cap, String content) {
  return snprintf(out, cap,
        "%02d-%02d-%04d,\n"       // date
        "%02d:%02d:%02d,%lu,\n"   // time
        "%s\n",                 // transmitted payload
        datetime[DAY], datetime[MONTH], datetime[YEAR],
        datetime[HOUR], datetime[MINUTE], datetime[SECOND], millis(),
        content.c_str());
}

// Shows LoRa Tx and Rx
void loraDebug(int select) {
  if ((select == 1) || (select == 2)){
    if ((millis() - lastBeacon) >= BEACON_INTVL) {
      String tx_msg = "Hello there, Bob!";
      loraTx(tx_msg);
      char tx_str[192];
      if (select != 2) {
        loraTxStr(tx_str, sizeof(tx_str), tx_msg);
        disp(tx_str, 1, false, false);
      }
      lastBeacon = millis();
    }
  }

  if ((select == 0) || (select == 2)){
    message = loraRx();
    if (message != "") {
      char rx_str[192];
      loraRxStr(rx_str, sizeof(rx_str), message);
      disp(rx_str, 1, false, false);
    }
  }
}
