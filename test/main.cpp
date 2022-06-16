
#include "header.h"
void setup()
{
  pinMode(PA0,INPUT);

  Serial.begin(9600);
  while (!Serial)
  {
    ;
  }
  Serial.println("Ethernet Modbus TCP Example");
  Ethernet.begin(mac, ip);
  ethServer.begin();
  Serial.println(Ethernet.localIP());
  if (!modbusTCPServer.begin())
  {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1)
      ;
  }
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  modbusTCPServer.configureCoils(0x00, 10);
}

void loop()
{
  EthernetClient client = ethServer.available();

  if (client)
  {

    Serial.println("new client");
    modbusTCPServer.accept(client);

    while (client.connected())
    {

      Serial.println(modbusTCPServer.coilRead(0));
      Serial.println(modbusTCPServer.coilRead(1));
      modbusTCPServer.poll();
    }

    Serial.println("client disconnected");
  }
}
