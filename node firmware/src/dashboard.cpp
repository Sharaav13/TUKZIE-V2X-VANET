//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - DASHBOARD (UART)
//*************************************************************************************************************************************************

#include "system.h"
#include "dashboard.h"

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Dashboard dash;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

/*
  Setup dashboard communication and serial debugging messages
  Returns true if the dashboard UART driver was installed
*/
bool Dashboard::init(void) {
  uart.begin(DASH_BAUD, SERIAL_8N1, DASH_RX, DASH_TX);

  if (!uart) {
    reportError("Dashboard: UART driver failed to install\n");
    return false;
  }

  ok = true;
  return true;
}

/*
  Transmits myself and a table of nodes to the TUKZIE dashboard
  One node per line
*/
void Dashboard::tx(Data table[], uint8_t size) {

  String payload = myself.toStr() + "\n";
  for (uint8_t i = 0; i < size; i++) {
    payload += table[i].toStr() + "\n";
  }

  uart.println(payload);
}

/*
  Receives and parses a packet from the TUKZIE dashboard
  Packets are terminated by '\n'
  Returns true when a complete packet has been parsed
*/
bool Dashboard::rx(void) {

  bool complete = false;

  // Collect bytes until a full packet has arrived
  while (uart.available()) {
    char c = uart.read();

    if (c == '\n') {
      complete = true;
      break;    // leave any following packet in the UART buffer for the next call
    }

    if (c != '\r') {    // drops any '\r' before the '\n'
      rxBuffer[rxLen++] = c;
    }

    // Drop oversized/garbled packets (keeps room for the terminator)
    if (rxLen >= MSG_SIZE - 1) {
      rxLen = 0;
    }
  }

  if (!complete) {
    return false;
  }

  rxBuffer[rxLen] = '\0';
  rxLen = 0;    // ready for the next packet

  // sscanf %d needs int, so the codes are parsed into ints then narrowed
  int status, tasks, message;
  float battery;
  int got = sscanf(rxBuffer,
  "%d,"                   // status
  "%d,"                   // deliveries
  "%f,"                   // battery
  "%d",                   // message
  &status,
  &tasks,
  &battery,
  &message);

  if (got != 4) {
    return false;
  }

  myself.status = (uint8_t)status;
  myself.tasks = (uint8_t)tasks;
  myself.battery = battery;
  myself.message = (uint8_t)message;
  return true;
}
