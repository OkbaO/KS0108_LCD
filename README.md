# KS0108_LCD
Arduino library for KS0108 controller based monochrome LCD

This is a library for KS0108 monochrome Graphics Liquid Crystal Display.
This library works with GLCD modules with 2 or 3 KS0108 chips. A GLCD module with two KS0108 chips (2 CS pins) resolution is 128x64 pixel,
and a three-chip module (3 CS pins) resolution is 192x64 pixel.
This type of display uses parallel interface to communicate with master device and requires*   14 or 15 GPIO pins mainly depending on the
resolution of the display (# of KS0108 chips).
Website URL:
https://simple-circuit.com/

Example URL:
https://wp.me/p9n96B-2Oc

*****************************************************************************************
This library depends on Adafruit GFX library at:
https://github.com/adafruit/Adafruit-GFX-Library
being present on your system. Please make sure you have installed the latest version before using this library.
This library is not an official Adafruit Industries library, for more information visit "Simple Circuit" website on:
https://simple-circuit.com/

BSD license, all text above and splash screen header file (splash.h) must be included in any redistribution.
