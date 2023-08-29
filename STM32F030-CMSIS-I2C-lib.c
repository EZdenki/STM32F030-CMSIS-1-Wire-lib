//  ==========================================================================================
//  STM32F030-CMSIS-I2C.lib.c
//  ------------------------------------------------------------------------------------------
//  Simple I2C library for the STM32F030 microcontroller running at 8 MHz.
//  ------------------------------------------------------------------------------------------
//  https://github.com/EZdenki/STM32F030-CMSIS-I2C-lib
//  Released under the MIT License
//  Copyright (c) 2023
//  Mike Shegedin, EZdenki.com
//  Version 1.0    5 Aug 2023   Updated init procedure
//  Version 0.9   23 Jul 2023   Start
//  ------------------------------------------------------------------------------------------
//  Target Device:
//    STM32F030Fxxx running at 8 MHz internal clock
//    Any standard I2C device
//  ------------------------------------------------------------------------------------------
//  Hardware Setup:
//
//                  STM32F030Fxxx
//                   .---. .---.                      ,---- [5.7 k] --- VCC
//             BOOT0 |1o  V  20| PA14 / SWCLK         |
//            OSC_IN |2      19| PA13 / SWDIO         |    .-------------. 
//           OSC_OUT |3      18| PA10 / I2C1_SDA | ---'--- | SDA  I2C    |
//              NRST |4      17| PA9  / I2C1_SCL | ---,--- | SCL  Device |
//              VDDA |5      16| VCC                  |    '-------------'
//               PA0 |6      15| GND                  |
//               PA1 |7      14| PB1                  '---- [5.7 k] --- VCC
//               PA2 |8      13| PA7
//               PA3 |9      12| PA6
//               PA4 |10     11| PA5
//                   '---------'
//
//  Note that the desired target I2C interface, I2C1, will be passed to *thisI2C. The
//  interface name is passed as-is, like "I2C_init( I2C1 );". Note that I2C2 is not
//  supported.
//
//  The following routines are supported:
//  ------------------------------------------------------------------------------------------
//
//  void
//  I2C_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed )
//    Initialize the specified I2C interface to operate at the specified speed. Note that
//    currently *only I2C1* is supported! I2C1 uses the following pins:
//        SCL: GPIO pin A9,  pin 17
//        SDA: GPIO pin A10, pin 18
//    Possible I2C speeds are from 10 kHz up to 400 kHz. Speeds below 10 kHz will default to
//    10 kHz and speeds above 400 kHz will default to 400 kHz.
// --------------------------------------------------------------------------------------------
//  void
//  I2C_start( I2C_TypeDef *thisI2C )
//    Set the start bit and wait for acknowledge that it was set.
// --------------------------------------------------------------------------------------------
//  void
//  I2C_setAddress( I2C_TypeDef *thisI2C, uint8_t address )
//    Write the address to the SADD bits of the CR2 register.
// --------------------------------------------------------------------------------------------
//   void
//   I2C_stop( I2C_TypeDef *thisI2C )
//     Set and then clear the stop bit.
// --------------------------------------------------------------------------------------------
//   void
//   I2C_setNBytes( I2C_TypeDef *thisI2C, uint8_t nBytes )
//     Set the number of bytes to be written.
// --------------------------------------------------------------------------------------------
//   void
//   I2C_write( I2C_TypeDef *thisI2C, uint8_t data )
//     Write a byte of data to the I2C interface.
// --------------------------------------------------------------------------------------------
//  void
//  I2C_stop( I2C_TypeDef *thisI2C )
//    Send the I2C stop bit and wait 20 us to allow enough time for next I2C command to occur
//    properly.
// --------------------------------------------------------------------------------------------
//   void
//   I2C_setReadMode( I2C_TypeDef *thisI2C )
//     Set the I2C interface into the read mode.
// --------------------------------------------------------------------------------------------
//   void
//   I2C_setWriteMode( I2C_TypeDef *thisI2C )
//     Set the I2C interface into the write mode.
// --------------------------------------------------------------------------------------------
//   uint8_t
//   I2C_read( I2C_TypeDef *thisI2C )
//     Read a byte from the I2C interface.
//
// --------------------------------------------------------------------------------------------
//
//  Normal Write Command Flow:
//  --------------------------
//  I2C_setAddress( I2C1, deviceI2CAddress )    (Set once per device)
//  I2C_setNBytes( I2C1, numberOfBytesToTransmit )
//  I2C_start( I2C1 )
//  I2C_write( I2C1, data )
//  (repeat write as required)
//  I2C_stop( I2C1 )
//
//  Normal Read Command Flow:
//  -------------------------
//  I2C_setAddress( I2C1, deviceI2CAddress )  (Set once per device)
//  I2C_setNBytes( I2C1, numberOfBytesToReceive )
//  I2C_setReadMode( I2C1 )
//  I2C_start( I2C1 )
//  data[0] = I2C_read( I2C1 )
//    (repeat read as required)
//  I2C_stop( I2C1 )
//  I2C_setWriteMode( I2C1 )
//
//  ==========================================================================================


#ifndef __STM32F030_CMSIS_I2C_LIB_C
#define __STM32F030_CMSIS_I2C_LIB_C


#include "stm32f030x6.h"          // Primary CMSIS header file


//  void
//  I2C_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed )
//    Initialize the specified I2C interface to operate at the specified speed. Note that
//    currently *only I2C1* is supported! I2C1 uses the following pins:
//        SCL: GPIO pin A9,  pin 17
//        SDA: GPIO pin A10, pin 18
//    Possible I2C speeds are from 10 kHz up to 400 kHz. Speeds below 10 kHz will default
//    to 10 kHz and speeds above 400 kHz will default to 400 kHz.
//    NOTE: The timing settings assume an 8 MHz clock.
void
I2C_init( I2C_TypeDef *thisI2C, uint32_t I2CSpeed )
{
  uint8_t I2CPresc, I2CSCLL, I2CSCLH;

  if( thisI2C != I2C1 )   // Currently, only I2C1 is implemented in this library. Add code
    thisI2C = I2C1;       // to support I2C2 if and when STM32F030 chips with more than
                          // 32 pins are to be supported.

  if( thisI2C == I2C1 )
  {
    RCC->AHBENR  |= RCC_AHBENR_GPIOAEN;   // Enable GPIO Port A clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;   // Enable the I2C1 clock
    
    // Set the GPIO A9 and A10 pins as alternate function
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER9_Pos) | (0b10 << GPIO_MODER_MODER10_Pos);
    
    // Set GPIO A9 and A10 as open-drain (not sure if this is needed)
    GPIOA->OTYPER |= GPIO_OTYPER_OT_9 | GPIO_OTYPER_OT_10;

    // Set GPIO A9 and A10 output speed to high (not sure if this is needed)
    GPIOA->OSPEEDR |= (0b10 << GPIO_OSPEEDR_OSPEEDR9_Pos) |
                      (0b10 << GPIO_OSPEEDR_OSPEEDR10_Pos);

    // Set GPIO A9 and A10 built-in pullups (internal pull-up approx. 40 k ohms)
    // This is disabeled as 40 k ohms is probably too small to be effective.
    // GPIOA->PUPDR |= (0b01 << GPIO_PUPDR_PUPDR9_Pos) | (0b01 << GPIO_PUPDR_PUPDR10_Pos);

    // Set GPIO A9 and A10 to Alternate Function 4
    GPIOA->AFR[1] |= (0b0100 << GPIO_AFRH_AFSEL9_Pos) | (0b0100 << GPIO_AFRH_AFSEL10_Pos);
  }

  // Perform a software reset on the I2C interface. Normally a SW reset would turn the I2C
  // interface off then on again. Here, the interface will be turned off, then the timings
  // set, then turned on, since timing settings must also occur when the interface is off.
  // This functionality is disabled since an initialization will only occure when the device
  // is first turned on.
  // thisI2C->CR1  &= ~I2C_CR1_PE;          // Disable the I2C interface
  // while( thisI2C->CR1 && I2C_CR1_PE ) ;  // Wait for PE bit to be cleared

  thisI2C->CR1 &= ~( I2C_CR1_DNF | I2C_CR1_ANFOFF | I2C_CR1_SMBHEN | I2C_CR1_SMBDEN );
  thisI2C->CR2 &= ~( I2C_CR2_RD_WRN | I2C_CR2_NACK | I2C_CR2_RELOAD | I2C_CR2_AUTOEND );
  
  if( I2CSpeed > 400E3 )      // Ensure that the specified speed is within operable bounds.
    I2CSpeed = 400E3;         // If not, then default to the high or low limit.
  else if( I2CSpeed < 10E3 )
    I2CSpeed = 10E3;

  if( I2CSpeed < 50E3 )       // Calculate lower speeds with the prescaler set to 1
  {
    I2CPresc = 1;
    I2CSCLH  = (uint32_t)2E6 / I2CSpeed - 5;
    I2CSCLL  = I2CSCLH + 3;
  }
  else                        // Calculte higher speeds with the prescaler set to 0
  {
    I2CPresc = 0;
    I2CSCLH  = (uint32_t)4E6 / I2CSpeed - 9;
    I2CSCLL  = I2CSCLH + 5;
  }

  #define I2C_SCLDEL 0x0      // Supposedly these can add a little speed in some cases,
  #define I2C_SDADEL 0x0      // but probably not strictly required.
  
  // Set the I2C timing values into the timing register
  
  thisI2C->TIMINGR |= (  I2CPresc << I2C_TIMINGR_PRESC_Pos)  |
                      (I2C_SCLDEL << I2C_TIMINGR_SCLDEL_Pos) |
                      (I2C_SDADEL << I2C_TIMINGR_SDADEL_Pos) |
                      (   I2CSCLH << I2C_TIMINGR_SCLH_Pos)   |
                      (   I2CSCLL << I2C_TIMINGR_SCLL_Pos);

  thisI2C->CR1  |= I2C_CR1_PE;            // Enable the I2C interface
}


//  void
//  I2C_start( I2C_TypeDef *thisI2c )
//  Set the start bit and wait for acknowledge that it was set.
void
I2C_start( I2C_TypeDef *thisI2C )
{
  thisI2C->CR2 |= I2C_CR2_START;          // Set START bit in I2C CR2 register
  while( thisI2C->CR2 & I2C_CR2_START ) ; // Wait until START bit is cleared
}


//  void
//  I2C_setAddress( I2C_TypeDef *thisI2C, uint8_t address )
//  Write the address to the SADD bits of the CR2 register.
void
I2C_setAddress( I2C_TypeDef *thisI2C, uint8_t address )
{
  thisI2C->CR2 &= ~I2C_CR2_SADD;                    // Clear Address bits in CR2 register
  thisI2C->CR2 |= (( address*2) << I2C_CR2_SADD_Pos );  // Write address to CR2 register
}


//  void
//  I2C_stop( I2C_TypeDef *thisI2C )
//  Set and then clear the stop bit.
void
I2C_stop( I2C_TypeDef *thisI2C )
{
  thisI2C->CR2 |= I2C_CR2_STOP;             // Set STOP bit in I2C CR2 register
  while( thisI2C->CR2 & I2C_CR2_STOP ) ;    // Wait until STOP bit is cleared

  thisI2C->ICR |= I2C_ICR_STOPCF;           // Clear the STOPF flag in the I2C ISR register
  while( thisI2C->ICR & I2C_ICR_STOPCF ) ;  // Wait until the STOPF flag is cleared. Note that
                                            // the stop flag is cleared by writing to the
                                            // ICR register but is read from the ISR register.
}


//  void
//  I2C_setNBytes( I2C_TypeDef *thisI2C, uint8_t nBytes )
//    Set the number of bytes to be transferred.
void
I2C_setNBytes( I2C_TypeDef *thisI2C, uint8_t nBytes )
{
  thisI2C->CR2 &= ~(I2C_CR2_NBYTES);            // Mask out byte-count bits
  thisI2C->CR2 |= (nBytes << I2C_CR2_NBYTES_Pos); // Set number of bytes to 1
}


//  void
//  I2C_write( I2C_TypeDef *thisI2C, uint8_t data )
//  Write a byte of data to the I2C interface.
void
I2C_write( I2C_TypeDef *thisI2C, uint8_t data )
{
  thisI2C->TXDR = (thisI2C->TXDR & 0xFFFFFF00) | data ;
  // Wait until both the the TXDR register is empty (TXIS=1) and the transfer-complete
  // flag (TC) is set, indicating the end of the transfer.
  while( !( thisI2C->ISR & ( I2C_ISR_TXIS | I2C_ISR_TC ))) ;
}


//  void
//  I2C_setReadMode( I2C_TypeDef *thisI2C )
//  Set the I2C interface into the read mode.
void
I2C_setReadMode( I2C_TypeDef *thisI2C )
{
  thisI2C->CR2 |= I2C_CR2_RD_WRN;               // Set I2C interface to read operation
}


//  void
//  I2C_setWriteMode( I2C_TypeDef *thisI2C )
//  Set the I2C interface into the write mode.
void
I2C_setWriteMode( I2C_TypeDef *thisI2C )
{
  thisI2C->CR2 &= ~I2C_CR2_RD_WRN;              // Restore read/write bit to write
}


//  uint8_t
//  I2C_read( I2C_TypeDef *thisI2C )
//  Read a byte from the I2C interface.
uint8_t
I2C_read( I2C_TypeDef *thisI2C )
{
  while( !( thisI2C->ISR & I2C_ISR_RXNE )) ;    // Wait for byte to appear
  uint8_t result = thisI2C->RXDR & 0xFF;        // Read received byte
  return result;
}


#endif /* __STM32F030_CMSIS_I2C_LIB.C */
