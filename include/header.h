#ifndef _HEADER_H_
#define _HEADER_H_

#include <SPI.h>
#include <Ethernet2.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 80, 17);
EthernetServer ethServer(502);
ModbusTCPServer modbusTCPServer;
const int ledPin = LED_BUILTIN;

#endif