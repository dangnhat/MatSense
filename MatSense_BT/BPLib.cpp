/*
 BPLib.cpp - Library for communication with RN-42 HID Bluetooth module
 Created by Basel Al-Rudainy, 6 april 2013.
 Ported to MSP432 by Nhat Pham, November 2018.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 */
#include <string.h>
#include <unistd.h>

#include "BPLib.h"

/* Debug header */
#define IES_DEBUG_EN (1)
#include "ies_debug.h"

BPLib::BPLib() {
  //TODO: add status pin.
//  pinMode(7, INPUT);
//  digitalWrite(7, LOW);
}

bool BPLib::begin(char BP_Mode[], char BP_Type[]) {
  // Reset BTSPP module
  GPIO_write(BP_RESET, 0);
  sleep(1);
  GPIO_write(BP_RESET, 1);
  sleep(1);

  // Init UART
  // Create a UART with data processing off.
  UART_Params_init(&uartParams);
  uartParams.writeDataMode = UART_DATA_BINARY;
  uartParams.readDataMode = UART_DATA_BINARY;
  uartParams.readReturnMode = UART_RETURN_FULL;
  uartParams.readEcho = UART_ECHO_OFF;
  uartParams.baudRate = 115200;
  // Open an instance of the UART drivers
  uart = UART_open(BP_UART, &uartParams);

//  /* Temporary increase UART baudrate */
//  sendString((char *)BP_MODE_COMMAND);
//  if (get((char *)BP_STAT_CMD, 5) != 1) {
//    return 0;
//  } //if
//
//  sendString((char *)BP_TEMP_CHANGE_BR);
//
//  if (get((char *)BP_STAT_ACK, 5) != 1) {
//    return 0;
//  } //if
//
//  UART_close(uart);
//  uartParams.baudRate = BP_UART_NBR;
//  // Open an instance of the UART drivers
//  uart = UART_open(BP_UART, &uartParams);
//
//  sleep(1);

  sendString((char *)BP_MODE_COMMAND);
  if (get((char *)BP_STAT_CMD, 5) != 1) {
    return 0;
  } //if

  sendString((char *)BP_MODE_EXITCOMMAND);
  if (get((char *)BP_STAT_END,  5) != 1) {
    return 0;
  } //if

  return 1;
}

bool BPLib::sendCmd(char BP_CMD[]) {
  sendString((char *)BP_MODE_COMMAND);
  if (get((char *)BP_STAT_CMD, 5) != 1) {
    return 0;
  } //if
  sendString((char *)BP_CMD);
  if (get((char *)BP_STAT_ACK, 5) != 1) {
    return 0;
  } //if
  sendString((char *)BP_MODE_EXITCOMMAND);
  if (get((char *)BP_STAT_END,  5) != 1) {
    return 0;
  } //if
  return 1;
}

uint8_t BPLib::readRaw(uint8_t *buffer, uint16_t size) {
  return UART_read(uart, buffer, size);
}

bool BPLib::get(char BP_STAT[], uint16_t strlen) {
  char buffer[strlen + 1];

  UART_read(uart, buffer, strlen);
  buffer[strlen] = 0;
  if (strcmp(buffer, BP_STAT) == 0) {
    return 1;
  } //if
  else {
    return 0;
  } //else
} //get

//void BPLib::keyboardPress(byte BP_KEY, byte BP_MOD) {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x9); //Length byte
//  Serial1.write(0x1); //Descriptor byte
//  Serial1.write(BP_MOD); //Modifier byte
//  Serial1.write(0x00); //-
//  Serial1.write(BP_KEY); //Send KEY
//  for (byte i = 0; i < 5; i++) { //Send five zero bytes
//    Serial1.write(0x00);
//  }
//
//}
//
//void BPLib::keyboardReleaseAll() {
//  keyboardPress(0x00, BP_MOD_NOMOD);
//}
//
//void BPLib::mouseClick(byte BP_BUTTON) {
//  mousePress(BP_BUTTON);
//  mouseReleaseAll();
//}
//void BPLib::mouseMove(signed int BP_X, signed int BP_Y) {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x5); //Length byte
//  Serial1.write(0x2); //Descriptor byte
//  Serial1.write(0x00); //Button byte
//  Serial1.write(BP_X);	//(-127 to 127)
//  Serial1.write(BP_Y);	//(-127 to 127)
//  Serial1.write(0x00);
//
//}
//void BPLib::mousePress(byte BP_BUTTON) {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x5); //Length byte
//  Serial1.write(0x2); //Descriptor byte
//  Serial1.write(BP_BUTTON); //Button byte
//  for (byte i = 0; i < 3; i++) { //Send three zero bytes
//    Serial1.write(0x00);
//  }
//}
//
//void BPLib::mouseReleaseAll() {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x5); //Length byte
//  Serial1.write(0x2); //Descriptor byte
//  for (byte i = 0; i < 4; i++) { //Send four zero bytes
//    Serial1.write(0x00);
//  }
//}
//
//void BPLib::mouseWheel(signed int BP_WHEEL) {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x5); //Length byte
//  Serial1.write(0x2); //Descriptor byte
//  for (byte i = 0; i < 3; i++) { //Send three zero bytes
//    Serial1.write(0x00);
//  }
//  Serial1.write(BP_WHEEL); //Wheel byte (-127 to 127)
//}

bool BPLib::changeName(char BP_NAME[]) {
  sendString((char *)BP_MODE_COMMAND);
  if (get((char *)BP_STAT_CMD, 5) != 1) {
    return 0;
  } //if
  sendString((char *)BP_CHANGE_NAME);
  sendString((char *)BP_NAME);
  sendString((char *)"\r\n");
  if (get((char *)BP_STAT_ACK, 5) != 1) {
    return 0;
  } //if
  sendString((char *)BP_MODE_EXITCOMMAND);
  if (get((char *)BP_STAT_END, 5) != 1) {
    return 0;
  } //if
  return 1;
}

void BPLib::sendByte(uint8_t byte) {
  UART_write(uart, &byte, 1);
}

void BPLib::sendBuffer(uint8_t *buffer, uint16_t size) {
  UART_write(uart, buffer, size);
}

void BPLib::sendString(char *string) {
  UART_write(uart, string, strlen(string));
}

//void BPLib::gameJoyPress(byte BP_ST_BTN, byte BP_ND_BTN) {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x6); //Length byte
//  Serial1.write(BP_ST_BTN); //First Button byte
//  Serial1.write(BP_ND_BTN); //Second Button byte
//  for (byte i = 0; i < 4; i++) { //Send four zero bytes
//    Serial1.write(0x00);
//  }
//}
//void BPLib::gameJoyMove(signed int BP_X1, signed int BP_Y1, signed int BP_X2,
//    signed int BP_Y2) {
//  Serial1.write(0xFD); //Start HID Report
//  Serial1.write(0x6); //Length byte
//  Serial1.write(BP_GAMEJOY_ST_NOBTN); //First Button byte
//  Serial1.write(BP_GAMEJOY_ND_NOBTN); //Second Button byte
//  Serial1.write(BP_X1 & 0xFF);	//First X coordinate
//  Serial1.write(BP_Y1 & 0xFF);	//First Y coordinate
//  Serial1.write(BP_X2 & 0xFF);	//Second X coordinate
//  Serial1.write(BP_Y2 & 0xFF);	//Second Y coordinate
//}
//void BPLib::gameJoyReleaseAll() {
//  gameJoyPress(BP_GAMEJOY_ST_NOBTN, BP_GAMEJOY_ND_NOBTN);
//}

bool BPLib::connected() {
  //TODO: return status pin
  return true;
}
