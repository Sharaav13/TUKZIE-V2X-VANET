//*************************************************************************************************************************************************
// TUKZIE V2X VANET (TVV)
// NODE FIRMWARE - HOST (UART)
//*************************************************************************************************************************************************

#include "system.h"
#include "host.h"
#include "interpret.h"
#include <driver/gpio.h>

//*************************************************************************************************************************************************
// INSTANCES
//*************************************************************************************************************************************************

Host host;

//*************************************************************************************************************************************************
// FUNCTIONS
//*************************************************************************************************************************************************

// ------------------------------------------------------------------------------------------------------------------------------------------------
// Host
// ------------------------------------------------------------------------------------------------------------------------------------------------

/*
  Setup the host UART
  Returns true if the UART driver was installed
*/
bool Host::init(void) {
  uart.begin(HOST_BAUD, SERIAL_8N1, HOST_RX, HOST_TX);

  if (!uart) {
    reportError("Host: UART driver failed to install\n");
    return false;
  }

  // Keep RX high with no host connected, so a floating pin cannot read noise
  gpio_pullup_en((gpio_num_t)HOST_RX);
  return true;
}

/*
  Transmits myself and a table of nodes to the host, one packet per node: "$<node>\n"
  me: a copy of myself
*/
void Host::tx(const Data &me, Data table[], uint8_t size) {

  String payload = String(HOST_START) + me.toStr() + "\n";
  for (uint8_t i = 0; i < size; i++) {
    payload += String(HOST_START) + table[i].toStr() + "\n";
  }

  uart.print(payload);
}

/*
  Receives and parses a packet from the host into myself
  Packet: "$status,tasks,battery\n" (anything before the '$' is ignored)
  Returns true when a complete packet has been parsed
*/
bool Host::rx(void) {

  bool complete = false;

  // Collect bytes from a HOST_START until the '\n'
  while (uart.available()) {
    char c = uart.read();

    // Start (or restart) a packet, so a packet joined midway is never used
    if (c == HOST_START) {
      inPacket = true;
      rxLen = 0;
      continue;
    }

    // Ignore anything outside a packet
    if (!inPacket) {
      continue;
    }

    if (c == '\n') {
      inPacket = false;
      complete = true;
      break;    // leave the next packet for the next call
    }

    if (c != '\r') {    // drops any '\r' before the '\n'
      rxBuffer[rxLen++] = c;
    }

    // Drop oversized/garbled packets and wait for the next HOST_START (keeps room for the terminator)
    if (rxLen >= MSG_SIZE - 1) {
      inPacket = false;
      rxLen = 0;
    }
  }

  if (!complete) {
    return false;
  }

  rxBuffer[rxLen] = '\0';
  rxLen = 0;    // ready for the next packet

  // sscanf %d needs int
  int status, tasks;
  float battery;
  int got = sscanf(rxBuffer,
  "%d,"                   // status
  "%d,"                   // tasks
  "%f",                   // battery
  &status,
  &tasks,
  &battery);

  if (got != 3) {
    return false;
  }

  myself.status = (uint8_t)status;
  myself.tasks = (uint8_t)tasks;
  myself.battery = battery;
  lastRx = millis();
  return true;
}

/*
  Returns true if a valid host packet arrived within HOST_TIMEOUT
*/
bool Host::present(void) {
  return (lastRx != 0) && (millis() - lastRx < HOST_TIMEOUT);
}
