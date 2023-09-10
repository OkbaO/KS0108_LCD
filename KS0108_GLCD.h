/***************************************************************************************
* This is a library for KS0108 monochrome Graphics Liquid Crystal Display.
* This library works with GLCD modules with 2 or 3 KS0108 chips. A GLCD module with
*   two KS0108 chips (2 CS pins) resolution is 128x64 pixel, and a three-chip module
*   (3 CS pins) resolution is 192x64 pixel.
* This type of display uses parallel interface to communicate with master device and requires
*   14 or 15 GPIO pins mainly depending on the resolution of the display (# of KS0108 chips).
*
* https://simple-circuit.com/
*
*****************************************************************************************
* This library depends on Adafruit GFX library at:
*   https://github.com/adafruit/Adafruit-GFX-Library
*   being present on your system. Please make sure you have installed the latest
*   version before using this library.
* This library is not an official Adafruit Industries library, for more information
*   visit "Simple Circuit" website on:
* https://simple-circuit.com/
*
* BSD license, all text above and splash screen header file (splash.h) must be
*   included in any redistribution.
*
*****************************************************************************************/

#ifndef _KS0108_GLCD_H_
#define _KS0108_GLCD_H_

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
  #include "pins_arduino.h"
#endif

#include <Adafruit_GFX.h>


// uncomment the line below and the splash screen will not be used, frees some flash memory space
//#define KS0108_NO_SPLASH

#define KS0108_ON  1      // ON pixel
#define KS0108_OFF 0      // OFF pixel
#define KS0108_INVERSE 2  // inverse pixel

#define KS0108_CMD_DISPLAY_OFF   0x3E  // lcd off
#define KS0108_CMD_DISPLAY_ON    0x3F  // lcd on

#define KS0108_CMD_SET_DISP_START_LINE  0xC0   // start line address, 0x40
#define KS0108_CMD_SET_PAGE_ADDRESS     0xB8   // page address
#define KS0108_CMD_SET_COL_ADDRESS      0x40   // column address

#define KS0108_Chip_1     1
#define KS0108_Chip_2     2
#define KS0108_Chip_3     3
#define KS0108_Chip_All   255

#define KS0108_CS_ACTIVE_HIGH    HIGH
#define KS0108_CS_ACTIVE_LOW     LOW

#define OUTPUTS 0
#define INPUTS  1

#define KS0108_Data()        digitalWrite(di_pin, HIGH)
#define KS0108_Instruction() digitalWrite(di_pin, LOW)
#define KS0108_Read_Op()     digitalWrite(rw_pin, HIGH)
#define KS0108_Write_Op()    digitalWrite(rw_pin, LOW)

#define ks0108_swap(a, b) { uint8_t t = a; a = b; b = t; }


class KS0108_GLCD : public Adafruit_GFX {
 public:
  // two KS0108 chip, 128x64 pixel resolution
  KS0108_GLCD(uint8_t DI, uint8_t RW, uint8_t EN, uint8_t DB0, uint8_t DB1, uint8_t DB2, uint8_t DB3, uint8_t DB4,
             uint8_t DB5, uint8_t DB6, uint8_t DB7, uint8_t CS1, uint8_t CS2, uint8_t RES);
  // three KS0108 chip, 192x64 pixel resolution
  KS0108_GLCD(uint8_t DI, uint8_t RW, uint8_t EN, uint8_t DB0, uint8_t DB1, uint8_t DB2, uint8_t DB3, uint8_t DB4,
             uint8_t DB5, uint8_t DB6, uint8_t DB7, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES);

  void init(uint8_t DI, uint8_t RW, uint8_t EN, uint8_t DB0, uint8_t DB1, uint8_t DB2, uint8_t DB3, uint8_t DB4,
             uint8_t DB5, uint8_t DB6, uint8_t DB7, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES);

  bool begin(bool cs_active);
  void writeCommand(uint8_t c, uint8_t chip);
  void writeData(uint8_t d, uint8_t chip);
  uint8_t readData(uint8_t chip);
  uint8_t getStatus(uint8_t chip);  
  void ChipSelect(uint8_t chip);
  void ChipdeSelect(uint8_t chip);
  void EnablePulse();

  void drawPixel(int16_t x, int16_t y, uint16_t color);
  void display();
  void clearDisplay(void);
  void fillDisplay(void);
  void scrollRight(uint8_t s);
  void scrollLeft(uint8_t s);
  void scrollDown(uint8_t s);
  void scrollUp(uint8_t s);
  void invertDisp(bool inv);

  protected:
  void write_byte(uint8_t _byte);          // write one byte to data pins
  uint8_t read_byte();                     // read one byte from data pins
  void data_pins_config(bool _pin_dir);    // data pins configuration (inputs/outputs) according to variable _pin_dir, if 1 inputs and if 0 outputs
  void display_not_busy(uint8_t chip);     // waits for display chip(s) to be ready (not busy)
  uint8_t di_pin, rw_pin, en_pin, cs1_pin, cs2_pin, cs3_pin, res_pin;
  uint8_t data_pins[8];
  bool pin_dir;          // data pins direction variable, 0: outputs, 1: inputs
  bool ks0108_active;    // a variable to select or deselect a certain KS0108 chip, 0: not selected, 1: selected
  bool ks0108_inverted;  // display invert variable
  uint8_t *buffer;       // display buffer
  
};

#endif