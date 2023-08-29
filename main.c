//  ==========================================================================================
//  main.c for STM32F030-CMSIS-1-Wire-lib.c
//  ------------------------------------------------------------------------------------------
//  Implement the 1-Wire protocol in order to communicate with a single DS18B20 temperature
//  sensor and then eventually multiple sensors on a single interface. Will also use the I2C
//  LCD driver for output. Will focus on having the devices powered directly with VDD (not
//  "parasite power" via the data pin.) After confirming operation using an exteral pullup
//  resistor, attempt to use an internal pullup built into GPIO port.
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-1-Wire-lib
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//    Version 1.0   29 Aug 2023    Started port from STM32F103-CMSIS-1-Wire-lib
//  ------------------------------------------------------------------------------------------
//  Target Microcontroller and Devices:
//    STM32F030F4xx
//    DS18B20 1-Wire Temperature Sensor
//    16x2 LCD Driver Module based on the PCF8574
//    16x2 LCD
// =========================================================================================== 
// Hardware Setup:
//
//     
//
//                  I2C LCD Driver Module
//          ======================================
//          1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16
//          ======================================
//                     16x2 LCD Module
//
//
//          DS18B20   STM32F030F4x   POWER   I2C LCD Driver Module
//          =======   ============   =====   =====================
//                       GND ------- GND
//                       VCC ------- 3.3V
//
//         |  GND ------------------ GND
// DS18B20 | DATA --- PA5 (Pin 11)---------------------. 
//         |  VDD ------------------ 3.3V --- [5K6] ---'  (Pullup)
//
//
//                  | PA9  (Pin 17) ----------------- SCL
//                  | PA10 (Pin 18) ----------------- SDA
//                  |                  5V ----------- VCC
//    I2C LCD Module|                 GND ----------- GND
//                  |
//                  |                            LED Jumper -- [1K] --.
//                  |                                                 |
//                  |                            LED Jumper ----------'
//
// ===========================================================================================


#include <string.h>
#include <stdlib.h>
#include "stm32f030x6.h"                  // Primary CMSIS header file
#include "STM32F030-Delay-lib.c"          // Library for pause and delay_us functions
#include "STM32F030-CMSIS-1-WIRE-lib.c"   // 1-Wire device library
#include "STM32F030-CMSIS-I2C-LCD-lib.c"  // I2C LCD driver library


// ===========================================================================================
// main
// ===========================================================================================
int
main()
{
  
  char     myString[16];            // Will hold printable strings
  uint8_t  sensorData[9];           // Used to hold data read in from the DS18B20 sensor
  uint32_t tempWhole;               // Used to build the DS18B20 temperature value
  
  I2C_LCD_init( I2C1, 100e3 );      // Set the LCD interface to I2C1 at 100 kHz
  I2C_LCD_cmd( LCD_4B_58F_2L );     // Get LCD into 4-bit mode
  I2C_LCD_cmd( LCD_ON_NO_CURSOR );  // LCD ON, Cursor OFF
  I2C_LCD_cmd( LCD_CLEAR );         // Clear the LCD screen
  I2C_LCD_cmd( LCD_1ST_LINE + 1 );  // Position cursor starting position
  I2C_LCD_puts( "DS18B20 Sensor" ); // Display title text

  W1_init();                        // Initialize 1-Wire device

  while ( 1 )                       // Repeat this block forever
  {
    //  ---------------------------------------------------
    //  *** Start of loop to Read DS18B20 1-Wire Sensor ***

    //  W1_readROM( sensorData, myString );
    W1_resetPulse(); 
    W1_writeByte( W1_R_SKIP_ROM );  // Command to address all devices on 1-Wire bus
    W1_writeByte( W1_DS18_R_CNVT ); // Command to initialize DS18B20 temperature conversion
    delay_us( 750e3 );              // Wait 750 ms, assuming 12-bit conversion
   
    W1_resetPulse();
    W1_writeByte( W1_R_SKIP_ROM );        // Command to address all devices on 1-Wire bus
    W1_writeByte( W1_DS18_R_RD_SCRATCH ); // Command to retrieve DS18B20 Scratch Pad data
    for( uint8_t x=0; x<9; x++ )          // Read in 9 bytes from Scratch Pad
      sensorData[x] = W1_readByte();

    tempWhole = ( sensorData[1]<<4 ) + (( sensorData[0] & 0xf0 ) >> 4 );
    if( sensorData[1] & 0x80 )
      tempWhole |= 0xFFFFF000;  // Make upper padding match 2s complement sign

    tempWhole = ( tempWhole * 10000 + (( sensorData[0] & 0x0F ) * 625 )) / 100;

    i100toa( tempWhole, myString );     // Convert x100 value to readable string with one
                                        // decimal place.
    I2C_LCD_cmd( LCD_2ND_LINE + 5 );
    I2C_LCD_puts( myString );           // Display rounded single-decimal-place value
    I2C_LCD_putc( ' ' );
    I2C_LCD_putc( 0xDF );               // Display the degree character
    I2C_LCD_puts( "C   " );             // Some padding to erase old long data

    //  *** END of loop to Read DS18B20 1-Wire Sensor ***
    //  -------------------------------------------------


  } // END of main repeating block
  return 1;
}
