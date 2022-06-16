#ifndef _CONFIG_H_
#define _CONFIG_H_
#include <modbus.h>
#include <Arduino.h>
#include <config.h>

int ModbusID;
int ModbusBaurate;

int input[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17}; 
/*                  { 53, 52, 51, 50, 10, 11, 12, 13,
                    30, 31, 32, 22, 34, 35, 36, 37,                
                  21, 20, 19, 18, 38,
                  0, 1, 2, 3, 4, 5, 6, 7};
*/
//int coil[16] = {A8, A9, A10, A11, A12, A13, A14, A15, 22, 23, 24, 25, 26, 27, 28, 29};

#define ID1 22 //AD0
#define ID2 23 //AD1
#define ID3 24 //AD2
#define ID4 25 //AD3
#define ID5 26 //AD4
#define ID6 27 //AD5
#define ID7 28 //AD6
#define ID8 29 //AD7

#define BR1 8
#define BR2 9
#define BR3 10
#define BR4 11
#define BR5 12
#define BR6 13
#endif