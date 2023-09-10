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

#include <avr/pgmspace.h>
#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#ifdef __AVR__
  #include <util/delay.h>
#endif

#include <stdlib.h>
#include "KS0108_GLCD.h"
#include "splash.h"
#include <Adafruit_GFX.h>


// two KS0108 chip, 128x64 pixel resolution
KS0108_GLCD::KS0108_GLCD(uint8_t DI, uint8_t RW, uint8_t EN, uint8_t DB0, uint8_t DB1, uint8_t DB2, uint8_t DB3, uint8_t DB4,
             uint8_t DB5, uint8_t DB6, uint8_t DB7, uint8_t CS1, uint8_t CS2, uint8_t RES) : Adafruit_GFX(128, 64), buffer(NULL)
{
  init(DI, RW, EN, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, CS1, CS2, 255, RES);   // make CS3 pin = 255 (not used)
}
// three KS0108 chip, 192x64 pixel resolution
KS0108_GLCD::KS0108_GLCD(uint8_t DI, uint8_t RW, uint8_t EN, uint8_t DB0, uint8_t DB1, uint8_t DB2, uint8_t DB3, uint8_t DB4,
             uint8_t DB5, uint8_t DB6, uint8_t DB7, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES) : Adafruit_GFX(192, 64), buffer(NULL)
{
  init(DI, RW, EN, DB0, DB1, DB2, DB3, DB4, DB5, DB6, DB7, CS1, CS2, CS3, RES);
}

void KS0108_GLCD::init(uint8_t DI, uint8_t RW, uint8_t EN, uint8_t DB0, uint8_t DB1, uint8_t DB2, uint8_t DB3, uint8_t DB4,
             uint8_t DB5, uint8_t DB6, uint8_t DB7, uint8_t CS1, uint8_t CS2, uint8_t CS3, uint8_t RES)
{
  di_pin  = DI;
  rw_pin  = RW;
  en_pin  = EN;
  cs1_pin = CS1;
  cs2_pin = CS2;
  cs3_pin = CS3;
  res_pin = RES;

  data_pins[0] = DB0;
  data_pins[1] = DB1;
  data_pins[2] = DB2;
  data_pins[3] = DB3; 
  data_pins[4] = DB4;
  data_pins[5] = DB5;
  data_pins[6] = DB6;
  data_pins[7] = DB7; 
}

// KS0108 LCD initialize function
bool KS0108_GLCD::begin(bool cs_active)
{
  // allocate some RAM space for the display buffer, returns false if failed
  if ( (buffer = (uint8_t *)malloc((WIDTH * HEIGHT) / 8)) == NULL )
    return false;

  clearDisplay();   // clear display buffer

  // pin configuration
  pinMode(di_pin, OUTPUT);      // data/instruction pin
  pinMode(rw_pin, OUTPUT);      // read/write pin
  pinMode(en_pin, OUTPUT);      // enable pin
  pinMode(cs1_pin, OUTPUT);     // chip select pin chip 1
  pinMode(cs2_pin, OUTPUT);     // chip select pin for chip 2
  if (cs3_pin != 255)
    pinMode(cs3_pin, OUTPUT);   // chip select pin for chip 3 (if used)
  
  ks0108_active = cs_active;
  ChipdeSelect(KS0108_Chip_All);   // unselect all KS0108 LCD chip

  // initialize data pins, configure as outputs
  data_pins_config(OUTPUTS);

  //reset device
  delay(100);
  digitalWrite(res_pin, LOW);
  delay(10);
  digitalWrite(res_pin, HIGH);
  delay(10);

  writeCommand( KS0108_CMD_SET_DISP_START_LINE | 0, KS0108_Chip_All );  // set RAM start line to 0 (top of screen)
  writeCommand( KS0108_CMD_SET_PAGE_ADDRESS | 0, KS0108_Chip_All );     // set page address to 0
  writeCommand( KS0108_CMD_SET_COL_ADDRESS | 0, KS0108_Chip_All );      // set column address to 0
  writeCommand( KS0108_CMD_DISPLAY_ON, KS0108_Chip_All );               // turn the display ON
  
  setRotation(0);
  ks0108_inverted = false;   // initial state of the display, not inverted

  #ifndef KS0108_NO_SPLASH
  // draw bitmap (splash screen) on the display
  drawBitmap( (WIDTH - splash_width) / 2, (HEIGHT - splash_height) / 2,
               splash_data, splash_width, splash_height, 1);
  #endif

  return true;  // LCD has been initialized with success
}

// the most basic function, set a single pixel
void KS0108_GLCD::drawPixel(int16_t x, int16_t y, uint16_t color)
{
  if ((x >= 0) && (x < width()) && (y >= 0) && (y < height())) {
    // Pixel is in-bounds. Rotate coordinates if needed.
    switch (getRotation()) {
    case 1:
      ks0108_swap(x, y);
      x = WIDTH - x - 1;
      break;
    case 2:
      x = WIDTH - x - 1;
      y = HEIGHT - y - 1;
      break;
    case 3:
      ks0108_swap(x, y);
      y = HEIGHT - y - 1;
      break;
    }

    if (ks0108_inverted && color != KS0108_INVERSE) {
      color ^= 1;   // invert color
    }
    switch (color) {
      case KS0108_ON:
        buffer[x + (y / 8) * WIDTH] |= (1 << (y & 7));
        break;
      case KS0108_OFF:
        buffer[x + (y / 8) * WIDTH] &= ~(1 << (y & 7));
        break;
      case KS0108_INVERSE:
        buffer[x + (y / 8) * WIDTH] ^= (1 << (y & 7));
        break;
    }
  }
}

// write command to the display
void KS0108_GLCD::writeCommand(uint8_t c, uint8_t chip)
{
  display_not_busy(chip);  // wait for chip to be ready (not busy)

  // if data pins are configured as inputs, then configure them as outputs
  if (pin_dir == INPUTS)
    data_pins_config(OUTPUTS);

  KS0108_Instruction();   // instuction (command)
  KS0108_Write_Op();      // write operation
  ChipSelect(chip);       // select desired chip(s)

  write_byte(c);

  EnablePulse();
  ChipdeSelect(chip);     // deselect previously selected chip(s)
}

// write data to the display
void KS0108_GLCD::writeData(uint8_t d, uint8_t chip)
{
  // if data pins are configured as inputs, then configure them as outputs
  if (pin_dir == INPUTS)
    data_pins_config(OUTPUTS);

  KS0108_Data();        // display RAM data
  KS0108_Write_Op();    // write operation
  ChipSelect(chip);     // select desired chip(s)

  write_byte(d);

  EnablePulse();
  ChipdeSelect(chip);   // deselect previously selected chip(s)
}

// read data from the display
// only one chip at time should be selected
uint8_t KS0108_GLCD::readData(uint8_t chip)
{
  // if data pins are configured as outputs, then configure them as inputs
  if (pin_dir == OUTPUTS)
    data_pins_config(INPUTS);
  KS0108_Data();      // read data from display RAM
  KS0108_Read_Op();   // read operation
  ChipSelect(chip);   // select desired chip
  digitalWrite(en_pin, HIGH);   // raise enable pin for at least 320 ns
  delayMicroseconds(1);

  uint8_t d = read_byte();

  digitalWrite(en_pin, LOW);
  ChipdeSelect(chip);     // deselect previously selected chip(s)
  return d;
}

// read the status register of 1 specified KS0108 chip
// returns 8-bit number with:
// bit 7:    0 - ready   1 - busy
// bit 5:    0 - on      1 - off
// bit 4:    0 - normal  1 - reset
// bit 6 & bits 3-0:  all 0's
uint8_t KS0108_GLCD::getStatus(uint8_t chip)
{
  // if data pins are configured as outputs, then configure them as inputs
  if (pin_dir == OUTPUTS)
    data_pins_config(INPUTS);
  KS0108_Instruction();  // set for instruction (here to get status register)
  KS0108_Read_Op();      // read operation
  ChipSelect(chip);      // select desired chip
  
  digitalWrite(en_pin, HIGH);   // raise enable pin for at least 320 ns
  delayMicroseconds(1);

  uint8_t d = read_byte();

  digitalWrite(en_pin, LOW);
  ChipdeSelect(chip);     // deselect previously selected chip(s)
  return d;
}

// write display buffer in LCD hardware
void KS0108_GLCD::display(void)
{
  uint8_t ks0108_chips = 2;
  if (cs3_pin != 255)
    ks0108_chips = 3;

  writeCommand( KS0108_CMD_SET_COL_ADDRESS | 0, KS0108_Chip_All );  // column address = 0 (automatically incremented up to 63)
  for (uint8_t ch = 1; ch < ks0108_chips+1; ch++) {
    for(uint16_t p = 0; p < 8; p++) {
      writeCommand( KS0108_CMD_SET_PAGE_ADDRESS | p, ch );   // page address
      KS0108_Data();        // write data to display RAM
      ChipSelect(ch);       // select desired chip
      for(uint8_t col = 0; col < 64; col++) {
        write_byte( buffer[ (p * WIDTH) + (ch-1)*64 + col ] );
        EnablePulse();
      }
      ChipdeSelect(ch);     // unselect desired chip
    }
  }
}

// clear everything
void KS0108_GLCD::clearDisplay(void)
{
  if (ks0108_inverted)
    memset(buffer, 0xFF, WIDTH*HEIGHT/8);
  else
    memset(buffer, 0, WIDTH*HEIGHT/8);
  cursor_y = cursor_x = 0;
}

// fill all the display
void KS0108_GLCD::fillDisplay(void)
{
  if (ks0108_inverted)
    memset(buffer, 0, WIDTH*HEIGHT/8);
  else
    memset(buffer, 0xFF, WIDTH*HEIGHT/8);
  cursor_y = cursor_x = 0;
}

// scroll display buffer to the right by 's' pixels
void KS0108_GLCD::scrollRight(uint8_t s)
{
  s &= 0x7F;
  while (s--) {
    for(uint8_t p = 0; p < 8; p++) {
      uint8_t msbyte = buffer[ ((p + 1) * WIDTH) - 1 ];
      for(uint8_t col = (WIDTH - 1); col > 0; col--) {
        buffer[ (p * WIDTH) + col ] = buffer[ (p * WIDTH) + col - 1 ];
      }
      buffer[ (p * WIDTH) ] = msbyte;
    }
  }
}
// scroll display buffer to the left by 's' pixels
void KS0108_GLCD::scrollLeft(uint8_t s)
{
  s &= 0x7F;
  while (s--) {
    for(uint8_t p = 0; p < 8; p++) {
      uint8_t lsbyte = buffer[ p * WIDTH ];
      for(uint8_t col = 0; col < (WIDTH - 1); col++) {
        buffer[ (p * WIDTH) + col ] = buffer[ (p * WIDTH) + (col + 1) ];
      }
      buffer[ ((p + 1) * WIDTH) - 1 ] = lsbyte;
    }
  }
}
// scroll display buffer down by 's' pixels
void KS0108_GLCD::scrollDown(uint8_t s)
{
  s &= 0x3F;
  while (s--) {
    for(uint8_t col = 0; col < WIDTH; col++) {
      uint8_t msbit = buffer[ (7 * WIDTH) + col ] & 0x80;  // save lcd buffer ms bit (very last bit in the display)
      for(uint8_t p = 7; p > 0; p--) {
        buffer[ (p * WIDTH) + col ] = (buffer[ (p * WIDTH) + col ] << 1);  // shift left by 1
        uint8_t p_msb = buffer[ ((p-1) * WIDTH) + col ] & 0x80;  // save page byte ms bit
        buffer[ (p * WIDTH) + col ] |= (p_msb >> 7);  // update page byte ls bit
      }
      // first page byte update
      buffer[col] = (buffer[col] << 1);
      buffer[col] |= (msbit >> 7);
    }
  }
}
// scroll display buffer up by 's' pixels
void KS0108_GLCD::scrollUp(uint8_t s)
{
  s &= 0x3F;
  while (s--) {
    for(uint8_t col = 0; col < WIDTH; col++) {
      uint8_t lsbit = buffer[col] & 0x01;  // save lcd buffer ls bit (very first bit in the display)
      for(uint8_t p = 0; p < 7; p++) {
        buffer[ (p * WIDTH) + col ] = (buffer[ (p * WIDTH) + col ] >> 1);  // shift right by 1
        uint8_t p_lsb = buffer[ ((p+1) * WIDTH) + col ] & 0x01;  // save page byte ls bit
        buffer[ (p * WIDTH) + col ] |= (p_lsb << 7);  // update page byte ls bit
      }
      // first page byte update
      buffer[ (7 * WIDTH) + col ] = (buffer[ (7 * WIDTH) + col ] >> 1);
      buffer[ (7 * WIDTH) + col ] |= (lsbit << 7);
    }
  }
}

// invert the display
void KS0108_GLCD::invertDisp(bool inv)
{
  if (ks0108_inverted != inv) {
    for (uint16_t i = 0; i < WIDTH*HEIGHT/8; i++)
      buffer[i] = ~buffer[i];
    ks0108_inverted = inv;
  }
}

// waits for display desired chip(s) to be free (not busy)
void KS0108_GLCD::display_not_busy(uint8_t chip)
{
  if (chip != KS0108_Chip_All) {
    while ( getStatus(chip) & 0x80 ) ;
  }
  else {
    while ( getStatus(KS0108_Chip_1) & 0x80 ) ;    // wait for chip 1 to be ready
    if (cs2_pin != 255)
      while ( getStatus(KS0108_Chip_2) & 0x80 ) ;  // wait for chip 2 to be ready
    if (cs3_pin != 255)
      while ( getStatus(KS0108_Chip_3) & 0x80 ) ;  // wait for chip 3 to be ready
  }
}

// write byte to DB0 --> DB7 pins, MSB to DB7
void KS0108_GLCD::write_byte(uint8_t _byte)
{
  for (uint8_t i = 0; i < 8; i++, _byte >>= 1)
    digitalWrite(data_pins[i], _byte & 0x01);
}

// read & return one byte from DB0 --> DB7 pins
uint8_t KS0108_GLCD::read_byte()
{
  uint8_t _byte_ = 0;
  for (uint8_t i = 0; i < 8; i++)
    _byte_ |= ( digitalRead(data_pins[i]) << i );
  
  return _byte_;
}

// enable pulse
void KS0108_GLCD::EnablePulse()
{
  digitalWrite(en_pin, HIGH);
  delayMicroseconds(1);         // high time must be > 200 ns (data set-up time)
  digitalWrite(en_pin, LOW);
}

// chip select: select one or all KS0108 chips
void KS0108_GLCD::ChipSelect(uint8_t chip)
{
  switch (chip) {
    case KS0108_Chip_1:
      digitalWrite(cs1_pin, ks0108_active);
      break;
    case KS0108_Chip_2:
      digitalWrite(cs2_pin, ks0108_active);
      break;
    case KS0108_Chip_3:
      digitalWrite(cs3_pin, ks0108_active);
      break;
    case KS0108_Chip_All:
      digitalWrite(cs1_pin, ks0108_active);
      digitalWrite(cs2_pin, ks0108_active);
      if (cs3_pin != 255)
        digitalWrite(cs3_pin, ks0108_active);
  }
}
// chip deselect: deselect one or all KS0108 chips
void KS0108_GLCD::ChipdeSelect(uint8_t chip)
{
  switch (chip) {
    case KS0108_Chip_1:
      digitalWrite(cs1_pin, !ks0108_active);
      break;
    case KS0108_Chip_2:
      digitalWrite(cs2_pin, !ks0108_active);
      break;
    case KS0108_Chip_3:
      digitalWrite(cs3_pin, !ks0108_active);
      break;
    case KS0108_Chip_All:
      digitalWrite(cs1_pin, !ks0108_active);
      digitalWrite(cs2_pin, !ks0108_active);
      if (cs3_pin != 255)
        digitalWrite(cs3_pin, !ks0108_active);
  }
}

// data pins configuration function
// _pin_dir = 1: INPUTS
// _pin_dir = 0: OUTPUTS
void KS0108_GLCD::data_pins_config(bool _pin_dir)
{
  pin_dir = _pin_dir;
  if (pin_dir) {
    for (uint8_t i = 0; i < 8; i++)
      pinMode(data_pins[i], INPUT);
  }
  else {
    for (uint8_t i = 0; i < 8; i++)
      pinMode(data_pins[i], OUTPUT);
  }
}

// end of code.