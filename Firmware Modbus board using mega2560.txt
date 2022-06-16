#include <ModbusRTU.h>
#include <Arduino.h>
#include <MCP3208.h>
#include <SPI.h>

// Firmware FOR BOARD 16IN_16OUT
// Using Uart2 for RS_485
#define REGN 10

// Cai dat cho ADC
MCP3208 adc(3);
// MCP3208 adc(ADC_VREF, SPI_CS);

//#define SLAVE_ID 1
// Define for board 16IN_16OUT
int input[] = {A8, A9, A10, A11, A12, A13, A14, A15,
               A3, A2, A1, A0, A4, A5, A6, A7};
int output[] = {37, 36, 35, 34, 33, 32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22};
#pragma region Define_ID_Pin_Baurate_Pin
// Define ID PinName
#define ID1 42
#define ID2 43
#define ID3 44
#define ID4 45
#define ID5 46
#define ID6 47
#define ID7 48
#define ID8 49
// Define Baurate PinName
#define BR1 4
#define BR2 13
#define BR3 12
#define BR4 9
#define BR5 8
#define BR6 7
#define BR7 6
#pragma endregion Define_ID_Pin_Baurate_Pin

int SLAVE_ID = 1;
long ModbusBaurate;
ModbusRTU mb;
void SetIDModbus()
{
    if (digitalRead(ID1) == HIGH)
        SLAVE_ID += 1;
    if (digitalRead(ID2) == HIGH)
        SLAVE_ID += 2;
    if (digitalRead(ID3) == HIGH)
        SLAVE_ID += 4;
    if (digitalRead(ID4) == HIGH)
        SLAVE_ID += 8;
    if (digitalRead(ID5) == HIGH)
        SLAVE_ID += 16;
    if (digitalRead(ID6) == HIGH)
        SLAVE_ID += 32;
    if (digitalRead(ID7) == HIGH)
        SLAVE_ID += 64;
    if (digitalRead(ID8) == HIGH)
        SLAVE_ID += 128;
}
int SetbaurateModbusRTU()
{
    // bool BR1, BR2, BR3 = true, BR4, BR5, BR6;
    if (digitalRead(BR1) == HIGH)
    {
        // Serial3.begin(14400);
        mb.setBaudrate(14400);
        ModbusBaurate = 14400;
    }
    if (digitalRead(BR2) == HIGH)
    {
        // Serial3.begin(19200);
        mb.setBaudrate(19200);
        ModbusBaurate = 19200;
    }
    if (digitalRead(BR3) == HIGH)
    {
        // Serial3.begin(38400);
        mb.setBaudrate(38400);
        ModbusBaurate = 38400;
    }
    if (digitalRead(BR4) == HIGH)
    {
        // Serial3.begin(56000);
        mb.setBaudrate(56000);
        ModbusBaurate = 56000;
    }
    if (digitalRead(BR5) == HIGH)
    {
        // Serial3.begin(57600);
        mb.setBaudrate(57600);
        ModbusBaurate = 57600;
    }
    if (digitalRead(BR6) == HIGH)
    {
        // Serial3.begin(115200);
        mb.setBaudrate(115200);
        ModbusBaurate = 115200;
    }
    if ((digitalRead(BR1) == LOW) & (digitalRead(BR2) == LOW) & (digitalRead(BR3) == LOW) & (digitalRead(BR4) == LOW) & (digitalRead(BR5) == LOW) & (digitalRead(BR6) == LOW))
    {
        // Serial3.begin(9600);
        mb.setBaudrate(9600);
        ModbusBaurate = 9600;
    }
    return ModbusBaurate;
}
void updateSensor()
{
    static unsigned long timeSampling = 0;
    // int val1 = adc.analogRead(4);

    // long val1=map(adc.analogRead(7), 0.0, 3189.00, 0.00, 20.00);
    float val1 = adc.analogRead(7) * (20.0 / 3189.0);
    float val2 = adc.analogRead(6) * (20.0 / 3189.0);
    float val3 = adc.analogRead(5) * (20.0 / 3189.0);
    float val4 = adc.analogRead(4) * (20.0 / 3189.0);

    // Sensor.UpdateData();
    if (millis() - timeSampling > 10)
    {
        // Serial.println((String)adc.analogRead(7)+"->"+(String)val1);

        mb.Hreg(0, val1 * 100); // Bắt đầu từ CH0 lưu vào Reg_1
        mb.Hreg(1, val2 * 100); // Bắt đầu từ CH1 lưu vào Reg_2
        mb.Hreg(2, val3 * 100); // Bắt đầu từ CH2 lưu vào Reg_3
        mb.Hreg(3, val4 * 100); // Bắt đầu từ CH3 lưu vào Reg_4
        timeSampling = millis();
    }
}
void checkconnect()
{
    static unsigned long timesample = 0;
    if (millis() - timesample > 10)
    {
        if (Serial2.available() > 0)
        {
        }
        else
        {
            // while (1)
            ;
        }
        timesample = millis();
    }
}
void setup()
{
#pragma region Declare_MPC3208
    // adc.begin(52, 51, 50, 3);
    adc.begin();
#pragma endregion Declare_MPC3208

    // initialize serial
    Serial.begin(9600);
    // 16Input
    for (int i = 0; i < 16; i++)
    {
        pinMode(input[i], INPUT_PULLUP);
    }
    // 16Output
    for (int i = 0; i < 16; i++)
    {
        pinMode(output[i], OUTPUT);
    }
    pinMode(ID1, INPUT_PULLUP);
    pinMode(ID2, INPUT_PULLUP);
    pinMode(ID3, INPUT_PULLUP);
    pinMode(ID4, INPUT_PULLUP);
    pinMode(ID5, INPUT_PULLUP);
    pinMode(ID6, INPUT_PULLUP);
    pinMode(ID7, INPUT_PULLUP);
    pinMode(ID8, INPUT_PULLUP);

    pinMode(BR1, INPUT_PULLUP);
    pinMode(BR2, INPUT_PULLUP);
    pinMode(BR3, INPUT_PULLUP);
    pinMode(BR4, INPUT_PULLUP);
    pinMode(BR5, INPUT_PULLUP);
    pinMode(BR6, INPUT_PULLUP);

    SetIDModbus();
    SetbaurateModbusRTU();
    Serial2.begin(ModbusBaurate, SERIAL_8N1);

    mb.begin(&Serial2);
    mb.setBaudrate(ModbusBaurate);
    mb.slave(SLAVE_ID);

    mb.addCoil(0, 0, 16); //  Thêm 100 Coils
    mb.addHreg(0, 0, 16); //  Thêm thanh ghi hoding register với địa chỉ bắt đầu = 0 và độ dài thanh ghi =100
    mb.addIsts(0, 0, 16); //  Thêm thanh ghi discrete với địa chỉ bắt đầu = 0, giá trị set ban đầu = false và độ dài thanh ghi = 100
    mb.addIreg(0, 0, 16); //  Thêm thanh ghi discrete với địa chỉ bắt đầu = 0, giá trị set ban đầu = false và độ dài thanh ghi = 100
                          //  mb.Ireg(0,1992);      //  Dùng cho xác thực board từ PLC
}
/*----------------------------------------------------------------*/
/*
void loop()
{
  //mb.Ists(0, 1);

  // for (int i = 0; i < 100; i++)
  // {
  //   mb.Ists(i, random(0, 2));
  // }
   for (int i = 0; i <= 100; i++)
   {
     mb.Ists(i, digitalRead(input[i]));
   }

  // for (int i = 0; i < 100; i++)
  // {
  //   mb.Hreg(i, random(0, 10));
  // }

  mb.task();
  yield();

  //For test ID
  //Serial.println(mb.Ireg(0));
}
*/
/*----------------------------------------------------------------*/
// Firmware for board 32Input
/*
void loop()
{
  //while (mb.Hreg(0) == HIGH992)
  {
    for (int i = 0; i <= 100; i++)
    {
      mb.Ists(i, digitalRead(input[i]));
    }

    mb.task();
    yield();
  }
  mb.task();
  yield();
  //Serial.println(SLAVE_ID);
  //Serial.println(ModbusBaurate);
}
*/
/*----------------------------------------------------------------*/
//// Firmware for board 16In/16Out
/*
void loop()
{
    // while (mb.Hreg(0) == HIGH992)
    {
        //Đọc input & gán vào thanh ghi
        // DI
        for (int i = 0; i < 16; i++)
        {
            mb.Ists(i, digitalRead(input[i]));
        }
        // DO
        for (int i = 0; i < 16; i++)
        {
            digitalWrite(output[i], mb.Coil(i));
        }
         //ADC
        //  for (int i = 0; i < 9; ++i)
        //  {
        //      mb.Hreg(i, adc.analogRead(i)); // Bắt đầu từ CH0 lưu vào Reg_1
        //  }
        //readAdc();
        mb.task();
        yield();
    }
    mb.task();
    yield();
}
*/
/*
// Test nhiễu điện từ
void loop()
{
    // while (mb.Hreg(0) == HIGH992)
    {
        for (int i = 0; i < 16; i++)
        {
            digitalWrite(output[i], HIGH);
            delay(100);
        }
        for (int i = 0; i < 16; i++)
        {
            digitalWrite(output[i], LOW);
            delay(10);
        }
    }
    mb.task();
    yield();
}
*/
///*
void loop()
{
    //Đọc input & gán vào thanh ghi
    for (int i = 0; i < 16; i++)
    {
        mb.Ists(i, digitalRead(input[i]));
    }
    // DO
    for (int i = 0; i < 16; i++)
    {
        digitalWrite(output[i], mb.Coil(i));
    }
    // ADC
    updateSensor();
    //checkconnect();
    //Serial.println(SLAVE_ID);
    //Serial.println(ModbusBaurate);

    mb.task();
    yield();
}
//*/