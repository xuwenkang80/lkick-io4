//
// Created by i on 2022/4/13.
//

#ifndef LKICK_STDINCLUDE_H
#define LKICK_STDINCLUDE_H

#include <cstdio>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "bsp/board.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <ctype.h>
#include "tusb.h"

#include "FreeRTOS.h"
#include "task.h"

#include "io4_usb.h"
#include "ongeki_hardware.h"

#include "led_board.h"
#include "comio.h"
#include "serial.h"
#include "aime_reader.h"
#include "hardware/i2c.h"

#include <PicoLed.hpp>
#define USBD_STACK_SIZE    (3*configMINIMAL_STACK_SIZE/2) * (CFG_TUSB_DEBUG ? 2 : 1)

#include "hardware/flash.h"
#include "config.h"

#define BTN_L1_PIN 0
#define LED_L1_PIN 1
#define BTN_L2_PIN 2
#define LED_L2_PIN 3
#define BTN_L3_PIN 4
#define LED_L3_PIN 5

#define BTN_R1_PIN 6
#define LED_R1_PIN 7
#define BTN_R2_PIN 8
#define LED_R2_PIN 9
#define BTN_R3_PIN 10
#define LED_R3_PIN 11

#define BTN_LSIDE_PIN 12
#define LED_LSIDE_PIN 13
#define BTN_RSIDE_PIN 14
#define LED_RSIDE_PIN 15

// #define BTN_LMENU_PIN 16 // re-wire from 16
// #define BTN_RMENU_PIN 17 // re-wire from 17
#define BTN_LMENU_PIN 21 // re-wire from 16
#define BTN_RMENU_PIN 22 // re-wire from 17

#define BTN_SERV_PIN 18
#define BTN_TEST_PIN 19
#define BTN_COIN_PIN 20

#define LEVER_PIN 28

#define UART_TX_PIN 17 // rewire to new
#define UART_RX_PIN 16 // rewire to new
#define UART_ID uart0
#define BAUD_RATE 115200

#endif //LKICK_STDINCLUDE_H

