/************************************************************************
 * File : config.h                                                      *
 *  Configuration file for Kbots main shield.                           *
 *                                                                      *
 * Copyright : (c) 2014, Germain Haessig <ghaessig@crans.org>           *
 * Copyright : (c) 2014, Xavier Lagorce <Xavier.Lagorce@crans.org>      *
 ************************************************************************/

// Only include this file once
#ifndef __CONFIG_H
#define __CONFIG_H

/* ---------------- 
  MOTORS pinout
---------------- */
#define MOT1_1 4
#define MOT1_2 5
#define COD1_A 2
#define COD1_B 3
#define MOT1_EN 6

#define MOT2_1 7
#define MOT2_2 8
#define COD2_A 19
#define COD2_B 18
#define MOT2_EN 9

#define MOT3_1 10
#define MOT3_2 11
#define COD3_A 21
#define COD3_B 20
#define MOT3_EN 12


/* ------------ 
  IR Sensor
------------ */
#define SHARP_0 A1
#define SHARP_1 A2
#define SHARP_2 A3
#define SHARP_3 A4
#define SHARP_4 A5
#define SHARP_5 A6


/* -------------------- 
  BATTERY monitoring
-------------------- */
#define BATT_0 A10
#define BATT_1 A11
#define BATT_2 A12


/* ----------------
  COMMUNICATION
---------------- */
#define SCL_SW1 A14
#define SDA_SW1 A15

#define SCL_SW2 34
#define SDA_SW2 36

#define EXT_TX 14 //TX3
#define EXT_RX 15 //RX3

#define BT_RX 17
#define BT_TX 16

#define SPI_MOSI 51
#define SPI_MISO 50
#define SPI_SCK 42
#define SPI_SS0 53
#define SPI_SS1 48
#define SPI_SS2 46
#define SPI_SS3 44


/* ------------ 
  ILS pinout
------------ */
#define ILS_0 22
#define ILS_1 24
#define ILS_2 26
#define ILS_3 28
#define ILS_4 30
#define ILS_5 32


/* --------------------------------
  USER INTERFACE dedicated pins 
-------------------------------- */
#define UI_LED_CPA 23 //Charlieplexing A
#define UI_LED_CPB 25 //Charlieplexing B
#define UI_LED_CPC 27 //Charlieplexing C
#define UI_LED5 29
#define UI_LED4 31
#define UI_LED3 33
#define UI_LED2 35
#define UI_LED1 37
#define UI_LED0 39
#define UI_BTN_DOWN 41
#define UI_BTN_LEFT 43
#define UI_BTN_CENTER 45
#define UI_BTN_RIGHT 47
#define UI_BTN_UP 49


/* -------- 
  OTHERS
-------- */
#define LED_IND 13
#define EXT_DET A7
#define EXT_S1 A8
#define EXT_S2 A9
#define START_BTN A0
#define BUZZER A13

#endif



