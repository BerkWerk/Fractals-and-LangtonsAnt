/*

    Titel: Embedded Systems Projekt - Visualization of Algorithms on TM4C1294NCPDT Display

    Inhalt:
    
    Ein großer Bestandteil dieses Projekts wurde aus anderen Libraries zusammengefasst und
    speziell für das  TM4C1294NCPDT Board mit Educational Booster Pack MKII umgeschrieben.

    Vieles wurde aus den Energia Libraries aus den Texas Instruments Beispiel Code, unter
    anderem für das TM4C1294NCPDT Board/ für den Educational Booster Pack MKII entnommen.
    Dabei wurden alle für die Ansteuerung des Displays notwendigen Libraries genau durchgelesen
    und hier zusammengefasst. Energia verwendet eine Mischung aus größtenteils C++/C und
    Arduino spezifische Funktionen, welche zum Teil relativ umständlich zu rekonstruieren sind.

    Dieses Projekt verwendet ausschließlich C, alle fehlenden Arduino Funktionen wurden
    durch eigene Lösungen substituiert. Alle redundanten Definitionen unter anderem für
    andere Boards wurden ignoriert.

    Da die Implementation von Schrift/Zeichen in Energia zu komplex schien wurde hierfür
    einer andere Library verwendet. Anfangs gab es Probleme die Schrift zu orientieren
    (gespiegelt/verkehrt), dieses Problem wurde behoben durch zusätzliche Cases in der Funktion
    'setOrientation'.
    Quelle: https://github.com/LePoloni/Texas-LaunchPad-and-BoosterPack-MK-II/tree/master/AulaMOC32

    Ziel des Projekts war es mit verschiedenen Algorithmen/Funktionen Grafiken auf dem
    Display zu erzeugen und dabei möglichst viele andere Interaktionsmöglichkeiten
    zu implementieren.

    Anleitung:

    Das fertige Projekt umfasst nun einen Mandelbrot und einen Langtons Ant Modus.
    Man kann zwischen den beiden Modi wechseln indem man beide Buttons auf dem
    Booster Pack gleichzeitig gedrückt hält.

    Mit dem auf dem Joystick auf dem Booster Pack kann man im Mandelbrot Modus
    navigieren. Mit den Buttons kann man rein und raus zoomen.

*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <inc/tm4c1294ncpdt.h>  /* Supplies SYSCTL_RCGCGPIO*, GPIO_PORT* macros */

#define TARGET_IS_SNOWFLAKE_RA0
#define PART_TM4C1294NCPDT          // necessary for certain libraries, board model

uint32_t g_ui32SysClock;
volatile uint32_t ui32PWMClock = 120000000;

#define BOOST_PACK_SPI 2

//#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"

#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include <driverlib/timer.h>
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/debug.h"
#include "driverlib/adc.h"


uint8_t SSIBitOrder = 1;

#define NOT_ACTIVE 0xA

//Energia.h
//definitions from the energia library; some of them are necessary for the SPI/display communication(lsb/msbfirst)
#define NOT_A_PORT 0
#define NOT_A_PIN 0
#define NOT_ON_TIMER 0
#define NOT_ON_ADC 0x10

#define CHANGE 4
#define FALLING 3
#define RISING 2
#define HIGH 0x1
#define LOW  0x0

#define LSBFIRST 0
#define MSBFIRST 1

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2
#define INPUT_PULLDOWN 0x3

#define SPI_LAST 0
#define SPI_CONTINUE 1

#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6
#define PG 7
#define PH 8
#define PJ 9
#define PK 10
#define PL 11
#define PM 12
#define PN 13
#define PP 14
#define PQ 15
#define PR 16
#define PS 17
#define PT 18

#define TIMA 0
#define TIMB 8

//#ifdef __TM4C1294NCPDT__
// from Table 13-2. General-Purpose Timers Signals (128TQFP)
#define T0CCP0_0 0
#define T0CCP0_1 1
#define T0CCP0_2 2

#define T0CCP1_0 3
#define T0CCP1_1 4
#define T0CCP1_2 5

#define T1CCP0_0 6
#define T1CCP0_1 7
#define T1CCP0_2 8

#define T1CCP1_0 9
#define T1CCP1_1 10
#define T1CCP1_2 11

#define T2CCP0_0 12
#define T2CCP0_1 13

#define T2CCP1_0 14
#define T2CCP1_1 15

#define T3CCP0_0 16
#define T3CCP0_1 17
#define T3CCP0_2 18

#define T3CCP1_0 19
#define T3CCP1_1 20
#define T3CCP1_2 21

#define T4CCP0_0 22
#define T4CCP0_1 23
#define T4CCP0_2 24

#define T4CCP1_0 25
#define T4CCP1_1 26
#define T4CCP1_2 27

#define T5CCP0_0 28
#define T5CCP0_1 29

#define T5CCP1_0 30
#define T5CCP1_1 31

#define TIMER0 0
#define TIMER1 1
#define TIMER2 2
#define TIMER3 3
#define TIMER4 4
#define TIMER5 5
#define WTIMER0 6 // this is needed PWMWrite, see TimerPrescaleSet
//end


//SPI.h
//definitions for the SPI connection
#define SPI_CLOCK_DIV2 2
#define SPI_CLOCK_DIV4 4
#define SPI_CLOCK_DIV8 8
#define SPI_CLOCK_DIV16 16
#define SPI_CLOCK_DIV32 32
#define SPI_CLOCK_DIV64 64
#define SPI_CLOCK_DIV128 128

#define SPI_MODE0 0x00
#define SPI_MODE1 0x80
#define SPI_MODE2 0x40
#define SPI_MODE3 0xC0

#define BOOST_PACK_SPI 2

#define MSBFIRST 1
#define LSBFIRST 0
//end

//Screen_HX8353E.h
//screen specific definitions
#define HX8353E_WIDTH  128
#define HX8353E_HEIGHT 128
#define HX8353E_MADCTL_MY  0x80
#define HX8353E_MADCTL_MX  0x40
#define HX8353E_MADCTL_MV  0x20
#define HX8353E_MADCTL_ML  0x10
#define HX8353E_MADCTL_RGB 0x08
#define HX8353E_MADCTL_MH  0x04
#define HX8353E_NOP     0x00
#define HX8353E_SWRESET 0x01
#define HX8353E_RDDID   0x04
#define HX8353E_RDDST   0x09
#define HX8353E_SLPIN   0x10
#define HX8353E_SLPOUT  0x11
#define HX8353E_PTLON   0x12
#define HX8353E_NORON   0x13
#define HX8353E_INVOFF  0x20
#define HX8353E_INVON   0x21
#define HX8353E_GAMSET  0x26
#define HX8353E_DISPOFF 0x28
#define HX8353E_DISPON  0x29
#define HX8353E_CASET   0x2A
#define HX8353E_RASET   0x2B
#define HX8353E_RAMWR   0x2C
#define HX8353E_RGBSET  0x2d
#define HX8353E_RAMRD   0x2E
#define HX8353E_PTLAR   0x30
#define HX8353E_MADCTL  0x36
#define HX8353E_COLMOD  0x3A
#define HX8353E_SETPWCTR 0xB1
#define HX8353E_SETDISPL 0xB2
#define HX8353E_FRMCTR3  0xB3
#define HX8353E_SETCYC   0xB4
#define HX8353E_SETBGP   0xb5
#define HX8353E_SETVCOM  0xB6
#define HX8353E_SETSTBA  0xC0
#define HX8353E_SETID    0xC3
#define HX8353E_GETHID   0xd0
#define HX8353E_SETGAMMA 0xE0

//these pins are important for SPI!
uint16_t _pinReset          = 17;
uint16_t _pinDataCommand    = 31;
uint16_t _pinChipSelect     = 13;
uint16_t _pinBacklight      = NULL;
//end

//start lcd_screen.h
//colours in binary
const uint16_t blackColour    = 0b0000000000000000;
const uint16_t whiteColour    = 0b1111111111111111;
const uint16_t redColour      = 0b1111100000000000;
const uint16_t greenColour    = 0b0000011111100000;
const uint16_t blueColour     = 0b0000000000011111;
const uint16_t yellowColour   = 0b1111111111100000;
const uint16_t cyanColour     = 0b0000011111111111;
const uint16_t orangeColour   = 0b1111101111100000;
const uint16_t magentaColour  = 0b1111100000001111;
const uint16_t violetColour   = 0b1111100000011111;
const uint16_t grayColour     = 0b0111101111101111;
const uint16_t darkGrayColour = 0b0011100111100111;



// array an farbwerten für das mandelbrot set mithilfe einer modulo funktion wird hierbei der farbwert bestimmt
// color= i%70;
const uint16_t colours[] = {
0x0000,//#define LCD_Black
0x0010,//#define LCD_Dark_Blue
0x0400,//#define LCD_Green
0x0410,//#define LCD_Teal
0x8000,//#define LCD_Dark_Red
0x8010,//#define LCD_Violet
0x8400,//#define LCD_Dark_Yellow
0x8410,//#define LCD_Gray50
0xC618,//#define LCD_Gray25
0x001F,//#define LCD_Blue
0x07E0,//#define LCD_Bright_Green
0x07FF,//#define LCD_Turquoise
0xF800,//#define LCD_Red
0xF81F,//#define LCD_Pink
0xFFE0,//#define LCD_Yellow
0xFFFF,//#define LCD_White
0x0000,//#define LCD_Black
0x0010,//#define LCD_Dark_Blue
0x0400,//#define LCD_Green
0x0410,//#define LCD_Teal
0x8000,//#define LCD_Dark_Red
0x8010,//#define LCD_Violet
0x8400,//#define LCD_Dark_Yellow
0x8410,//#define LCD_Gray50
0xC618,//#define LCD_Gray25
0x001F,//#define LCD_Blue
0x07E0,//#define LCD_Bright_Green
0x07FF,//#define LCD_Turquoise
0xF800,//#define LCD_Red
0xF81F,//#define LCD_Pink
0xFFE0,//#define LCD_Yellow
0xFFFF,//#define LCD_White
0x0000,//#define LCD_Black
0x0010,//#define LCD_Dark_Blue
0x0400,//#define LCD_Green
0x0410,//#define LCD_Teal
0x8000,//#define LCD_Dark_Red
0x8010,//#define LCD_Violet
1,//#define LCD4_Black
2,//#define LCD4_Dark_Blue
3,//#define LCD4_Green
4,//#define LCD4_Teal
5,//#define LCD4_Dark_Red
6,//#define LCD4_Violet
7,//#define LCD4_Dark_Yellow
8,//#define LCD4_Gray50
9,//#define LCD4_Gray25
10,//#define LCD4_Blue
11,//#define LCD4_Bright_Green
12,//#define LCD4_Turquoise
13,//#define LCD4_Red
14,//#define LCD4_Pink
15,//#define LCD4_Yellow
16,//#define LCD4_White
1,//#define LCD4_Black
2,//#define LCD4_Dark_Blue
3,//#define LCD4_Green
4,//#define LCD4_Teal
5,//#define LCD4_Dark_Red
6,//#define LCD4_Violet
7,//#define LCD4_Dark_Yellow
8,//#define LCD4_Gray50
9,//#define LCD4_Gray25
10,//#define LCD4_Blue
11,//#define LCD4_Bright_Green
12,//#define LCD4_Turquoise
13,//#define LCD4_Red
14,//#define LCD4_Pink
15,//#define LCD4_Yellow
16//#define LCD4_White
};


//font start
//
#define MAX_FONT_SIZE 1
#include "Terminal6e.h"

// standard ascii 5x7 font
// originally from glcdfont.c from Adafruit project
//font in hexadecimal
static const uint8_t Font[] = {
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x3E, 0x5B, 0x4F, 0x5B, 0x3E,
  0x3E, 0x6B, 0x4F, 0x6B, 0x3E,
  0x1C, 0x3E, 0x7C, 0x3E, 0x1C,
  0x18, 0x3C, 0x7E, 0x3C, 0x18,
  0x1C, 0x57, 0x7D, 0x57, 0x1C,
  0x1C, 0x5E, 0x7F, 0x5E, 0x1C,
  0x00, 0x18, 0x3C, 0x18, 0x00,
  0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
  0x00, 0x18, 0x24, 0x18, 0x00,
  0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
  0x30, 0x48, 0x3A, 0x06, 0x0E,
  0x26, 0x29, 0x79, 0x29, 0x26,
  0x40, 0x7F, 0x05, 0x05, 0x07,
  0x40, 0x7F, 0x05, 0x25, 0x3F,
  0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
  0x7F, 0x3E, 0x1C, 0x1C, 0x08,
  0x08, 0x1C, 0x1C, 0x3E, 0x7F,
  0x14, 0x22, 0x7F, 0x22, 0x14,
  0x5F, 0x5F, 0x00, 0x5F, 0x5F,
  0x06, 0x09, 0x7F, 0x01, 0x7F,
  0x00, 0x66, 0x89, 0x95, 0x6A,
  0x60, 0x60, 0x60, 0x60, 0x60,
  0x94, 0xA2, 0xFF, 0xA2, 0x94,
  0x08, 0x04, 0x7E, 0x04, 0x08,
  0x10, 0x20, 0x7E, 0x20, 0x10,
  0x08, 0x08, 0x2A, 0x1C, 0x08,
  0x08, 0x1C, 0x2A, 0x08, 0x08,
  0x1E, 0x10, 0x10, 0x10, 0x10,
  0x0C, 0x1E, 0x0C, 0x1E, 0x0C,
  0x30, 0x38, 0x3E, 0x38, 0x30,
  0x06, 0x0E, 0x3E, 0x0E, 0x06,
  0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x5F, 0x00, 0x00,
  0x00, 0x07, 0x00, 0x07, 0x00,
  0x14, 0x7F, 0x14, 0x7F, 0x14,
  0x24, 0x2A, 0x7F, 0x2A, 0x12,
  0x23, 0x13, 0x08, 0x64, 0x62,
  0x36, 0x49, 0x56, 0x20, 0x50,
  0x00, 0x08, 0x07, 0x03, 0x00,
  0x00, 0x1C, 0x22, 0x41, 0x00,
  0x00, 0x41, 0x22, 0x1C, 0x00,
  0x2A, 0x1C, 0x7F, 0x1C, 0x2A,
  0x08, 0x08, 0x3E, 0x08, 0x08,
  0x00, 0x80, 0x70, 0x30, 0x00,
  0x08, 0x08, 0x08, 0x08, 0x08,
  0x00, 0x00, 0x60, 0x60, 0x00,
  0x20, 0x10, 0x08, 0x04, 0x02,
  0x3E, 0x51, 0x49, 0x45, 0x3E, // 0
  0x00, 0x42, 0x7F, 0x40, 0x00, // 1
  0x72, 0x49, 0x49, 0x49, 0x46, // 2
  0x21, 0x41, 0x49, 0x4D, 0x33, // 3
  0x18, 0x14, 0x12, 0x7F, 0x10, // 4
  0x27, 0x45, 0x45, 0x45, 0x39, // 5
  0x3C, 0x4A, 0x49, 0x49, 0x31, // 6
  0x41, 0x21, 0x11, 0x09, 0x07, // 7
  0x36, 0x49, 0x49, 0x49, 0x36, // 8
  0x46, 0x49, 0x49, 0x29, 0x1E, // 9
  0x00, 0x00, 0x14, 0x00, 0x00,
  0x00, 0x40, 0x34, 0x00, 0x00,
  0x00, 0x08, 0x14, 0x22, 0x41,
  0x14, 0x14, 0x14, 0x14, 0x14,
  0x00, 0x41, 0x22, 0x14, 0x08,
  0x02, 0x01, 0x59, 0x09, 0x06,
  0x3E, 0x41, 0x5D, 0x59, 0x4E,
  0x7C, 0x12, 0x11, 0x12, 0x7C, // A
  0x7F, 0x49, 0x49, 0x49, 0x36, // B
  0x3E, 0x41, 0x41, 0x41, 0x22, // C
  0x7F, 0x41, 0x41, 0x41, 0x3E, // D
  0x7F, 0x49, 0x49, 0x49, 0x41, // E
  0x7F, 0x09, 0x09, 0x09, 0x01, // F
  0x3E, 0x41, 0x41, 0x51, 0x73, // G
  0x7F, 0x08, 0x08, 0x08, 0x7F, // H
  0x00, 0x41, 0x7F, 0x41, 0x00, // I
  0x20, 0x40, 0x41, 0x3F, 0x01, // J
  0x7F, 0x08, 0x14, 0x22, 0x41, // K
  0x7F, 0x40, 0x40, 0x40, 0x40, // L
  0x7F, 0x02, 0x1C, 0x02, 0x7F, // M
  0x7F, 0x04, 0x08, 0x10, 0x7F, // N
  0x3E, 0x41, 0x41, 0x41, 0x3E, // O
  0x7F, 0x09, 0x09, 0x09, 0x06, // P
  0x3E, 0x41, 0x51, 0x21, 0x5E, // Q
  0x7F, 0x09, 0x19, 0x29, 0x46, // R
  0x26, 0x49, 0x49, 0x49, 0x32, // S
  0x03, 0x01, 0x7F, 0x01, 0x03, // T
  0x3F, 0x40, 0x40, 0x40, 0x3F, // U
  0x1F, 0x20, 0x40, 0x20, 0x1F, // V
  0x3F, 0x40, 0x38, 0x40, 0x3F, // W
  0x63, 0x14, 0x08, 0x14, 0x63, // X
  0x03, 0x04, 0x78, 0x04, 0x03, // Y
  0x61, 0x59, 0x49, 0x4D, 0x43, // Z
  0x00, 0x7F, 0x41, 0x41, 0x41,
  0x02, 0x04, 0x08, 0x10, 0x20,
  0x00, 0x41, 0x41, 0x41, 0x7F,
  0x04, 0x02, 0x01, 0x02, 0x04,
  0x40, 0x40, 0x40, 0x40, 0x40,
  0x00, 0x03, 0x07, 0x08, 0x00,
  0x20, 0x54, 0x54, 0x78, 0x40, // a
  0x7F, 0x28, 0x44, 0x44, 0x38, // b
  0x38, 0x44, 0x44, 0x44, 0x28, // c
  0x38, 0x44, 0x44, 0x28, 0x7F, // d
  0x38, 0x54, 0x54, 0x54, 0x18, // e
  0x00, 0x08, 0x7E, 0x09, 0x02, // f
  0x18, 0xA4, 0xA4, 0x9C, 0x78, // g
  0x7F, 0x08, 0x04, 0x04, 0x78, // h
  0x00, 0x44, 0x7D, 0x40, 0x00, // i
  0x20, 0x40, 0x40, 0x3D, 0x00, // j
  0x7F, 0x10, 0x28, 0x44, 0x00, // k
  0x00, 0x41, 0x7F, 0x40, 0x00, // l
  0x7C, 0x04, 0x78, 0x04, 0x78, // m
  0x7C, 0x08, 0x04, 0x04, 0x78, // n
  0x38, 0x44, 0x44, 0x44, 0x38, // o
  0xFC, 0x18, 0x24, 0x24, 0x18, // p
  0x18, 0x24, 0x24, 0x18, 0xFC, // q
  0x7C, 0x08, 0x04, 0x04, 0x08, // r
  0x48, 0x54, 0x54, 0x54, 0x24, // s
  0x04, 0x04, 0x3F, 0x44, 0x24, // t
  0x3C, 0x40, 0x40, 0x20, 0x7C, // u
  0x1C, 0x20, 0x40, 0x20, 0x1C, // v
  0x3C, 0x40, 0x30, 0x40, 0x3C, // w
  0x44, 0x28, 0x10, 0x28, 0x44, // x
  0x4C, 0x90, 0x90, 0x90, 0x7C, // y
  0x44, 0x64, 0x54, 0x4C, 0x44, // z
  0x00, 0x08, 0x36, 0x41, 0x00,
  0x00, 0x00, 0x77, 0x00, 0x00,
  0x00, 0x41, 0x36, 0x08, 0x00,
  0x02, 0x01, 0x02, 0x04, 0x02,
  0x3C, 0x26, 0x23, 0x26, 0x3C,
  0x1E, 0xA1, 0xA1, 0x61, 0x12,
  0x3A, 0x40, 0x40, 0x20, 0x7A,
  0x38, 0x54, 0x54, 0x55, 0x59,
  0x21, 0x55, 0x55, 0x79, 0x41,
  0x21, 0x54, 0x54, 0x78, 0x41,
  0x21, 0x55, 0x54, 0x78, 0x40,
  0x20, 0x54, 0x55, 0x79, 0x40,
  0x0C, 0x1E, 0x52, 0x72, 0x12,
  0x39, 0x55, 0x55, 0x55, 0x59,
  0x39, 0x54, 0x54, 0x54, 0x59,
  0x39, 0x55, 0x54, 0x54, 0x58,
  0x00, 0x00, 0x45, 0x7C, 0x41,
  0x00, 0x02, 0x45, 0x7D, 0x42,
  0x00, 0x01, 0x45, 0x7C, 0x40,
  0xF0, 0x29, 0x24, 0x29, 0xF0,
  0xF0, 0x28, 0x25, 0x28, 0xF0,
  0x7C, 0x54, 0x55, 0x45, 0x00,
  0x20, 0x54, 0x54, 0x7C, 0x54,
  0x7C, 0x0A, 0x09, 0x7F, 0x49,
  0x32, 0x49, 0x49, 0x49, 0x32,
  0x32, 0x48, 0x48, 0x48, 0x32,
  0x32, 0x4A, 0x48, 0x48, 0x30,
  0x3A, 0x41, 0x41, 0x21, 0x7A,
  0x3A, 0x42, 0x40, 0x20, 0x78,
  0x00, 0x9D, 0xA0, 0xA0, 0x7D,
  0x39, 0x44, 0x44, 0x44, 0x39,
  0x3D, 0x40, 0x40, 0x40, 0x3D,
  0x3C, 0x24, 0xFF, 0x24, 0x24,
  0x48, 0x7E, 0x49, 0x43, 0x66,
  0x2B, 0x2F, 0xFC, 0x2F, 0x2B,
  0xFF, 0x09, 0x29, 0xF6, 0x20,
  0xC0, 0x88, 0x7E, 0x09, 0x03,
  0x20, 0x54, 0x54, 0x79, 0x41,
  0x00, 0x00, 0x44, 0x7D, 0x41,
  0x30, 0x48, 0x48, 0x4A, 0x32,
  0x38, 0x40, 0x40, 0x22, 0x7A,
  0x00, 0x7A, 0x0A, 0x0A, 0x72,
  0x7D, 0x0D, 0x19, 0x31, 0x7D,
  0x26, 0x29, 0x29, 0x2F, 0x28,
  0x26, 0x29, 0x29, 0x29, 0x26,
  0x30, 0x48, 0x4D, 0x40, 0x20,
  0x38, 0x08, 0x08, 0x08, 0x08,
  0x08, 0x08, 0x08, 0x08, 0x38,
  0x2F, 0x10, 0xC8, 0xAC, 0xBA,
  0x2F, 0x10, 0x28, 0x34, 0xFA,
  0x00, 0x00, 0x7B, 0x00, 0x00,
  0x08, 0x14, 0x2A, 0x14, 0x22,
  0x22, 0x14, 0x2A, 0x14, 0x08,
  0xAA, 0x00, 0x55, 0x00, 0xAA,
  0xAA, 0x55, 0xAA, 0x55, 0xAA,
  0x00, 0x00, 0x00, 0xFF, 0x00,
  0x10, 0x10, 0x10, 0xFF, 0x00,
  0x14, 0x14, 0x14, 0xFF, 0x00,
  0x10, 0x10, 0xFF, 0x00, 0xFF,
  0x10, 0x10, 0xF0, 0x10, 0xF0,
  0x14, 0x14, 0x14, 0xFC, 0x00,
  0x14, 0x14, 0xF7, 0x00, 0xFF,
  0x00, 0x00, 0xFF, 0x00, 0xFF,
  0x14, 0x14, 0xF4, 0x04, 0xFC,
  0x14, 0x14, 0x17, 0x10, 0x1F,
  0x10, 0x10, 0x1F, 0x10, 0x1F,
  0x14, 0x14, 0x14, 0x1F, 0x00,
  0x10, 0x10, 0x10, 0xF0, 0x00,
  0x00, 0x00, 0x00, 0x1F, 0x10,
  0x10, 0x10, 0x10, 0x1F, 0x10,
  0x10, 0x10, 0x10, 0xF0, 0x10,
  0x00, 0x00, 0x00, 0xFF, 0x10,
  0x10, 0x10, 0x10, 0x10, 0x10,
  0x10, 0x10, 0x10, 0xFF, 0x10,
  0x00, 0x00, 0x00, 0xFF, 0x14,
  0x00, 0x00, 0xFF, 0x00, 0xFF,
  0x00, 0x00, 0x1F, 0x10, 0x17,
  0x00, 0x00, 0xFC, 0x04, 0xF4,
  0x14, 0x14, 0x17, 0x10, 0x17,
  0x14, 0x14, 0xF4, 0x04, 0xF4,
  0x00, 0x00, 0xFF, 0x00, 0xF7,
  0x14, 0x14, 0x14, 0x14, 0x14,
  0x14, 0x14, 0xF7, 0x00, 0xF7,
  0x14, 0x14, 0x14, 0x17, 0x14,
  0x10, 0x10, 0x1F, 0x10, 0x1F,
  0x14, 0x14, 0x14, 0xF4, 0x14,
  0x10, 0x10, 0xF0, 0x10, 0xF0,
  0x00, 0x00, 0x1F, 0x10, 0x1F,
  0x00, 0x00, 0x00, 0x1F, 0x14,
  0x00, 0x00, 0x00, 0xFC, 0x14,
  0x00, 0x00, 0xF0, 0x10, 0xF0,
  0x10, 0x10, 0xFF, 0x10, 0xFF,
  0x14, 0x14, 0x14, 0xFF, 0x14,
  0x10, 0x10, 0x10, 0x1F, 0x00,
  0x00, 0x00, 0x00, 0xF0, 0x10,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
  0xFF, 0xFF, 0xFF, 0x00, 0x00,
  0x00, 0x00, 0x00, 0xFF, 0xFF,
  0x0F, 0x0F, 0x0F, 0x0F, 0x0F,
  0x38, 0x44, 0x44, 0x38, 0x44,
  0x7C, 0x2A, 0x2A, 0x3E, 0x14,
  0x7E, 0x02, 0x02, 0x06, 0x06,
  0x02, 0x7E, 0x02, 0x7E, 0x02,
  0x63, 0x55, 0x49, 0x41, 0x63,
  0x38, 0x44, 0x44, 0x3C, 0x04,
  0x40, 0x7E, 0x20, 0x1E, 0x20,
  0x06, 0x02, 0x7E, 0x02, 0x02,
  0x99, 0xA5, 0xE7, 0xA5, 0x99,
  0x1C, 0x2A, 0x49, 0x2A, 0x1C,
  0x4C, 0x72, 0x01, 0x72, 0x4C,
  0x30, 0x4A, 0x4D, 0x4D, 0x30,
  0x30, 0x48, 0x78, 0x48, 0x30,
  0xBC, 0x62, 0x5A, 0x46, 0x3D,
  0x3E, 0x49, 0x49, 0x49, 0x00,
  0x7E, 0x01, 0x01, 0x01, 0x7E,
  0x2A, 0x2A, 0x2A, 0x2A, 0x2A,
  0x44, 0x44, 0x5F, 0x44, 0x44,
  0x40, 0x51, 0x4A, 0x44, 0x40,
  0x40, 0x44, 0x4A, 0x51, 0x40,
  0x00, 0x00, 0xFF, 0x01, 0x03,
  0xE0, 0x80, 0xFF, 0x00, 0x00,
  0x08, 0x08, 0x6B, 0x6B, 0x08,
  0x36, 0x12, 0x36, 0x24, 0x36,
  0x06, 0x0F, 0x09, 0x0F, 0x06,
  0x00, 0x00, 0x18, 0x18, 0x00,
  0x00, 0x00, 0x10, 0x10, 0x00,
  0x30, 0x40, 0xFF, 0x01, 0x01,
  0x00, 0x1F, 0x01, 0x01, 0x1E,
  0x00, 0x19, 0x1D, 0x17, 0x12,
  0x00, 0x3C, 0x3C, 0x3C, 0x3C,
  0x00, 0x00, 0x00, 0x00, 0x00,
};
//font end


uint8_t      _fontX, _fontY, _fontSize;
uint8_t      _orientation;
bool         _penSolid, _fontSolid, _flagRead, _flagStorage;
uint16_t     _screenWidth, _screenHeigth;
uint8_t      _touchTrim;
uint16_t     _touchXmin, _touchXmax, _touchYmin, _touchYmax;

//end

//langtons ant definitions

uint8_t _map[2][128][128];
//end



//joystick global variables
uint32_t ui32ADCValues[4];
volatile uint32_t ui32X;
volatile uint32_t ui32Y;

//button global var
uint16_t buttonA;
uint16_t buttonB;

uint32_t ui32StrengthA;
uint32_t ui32PinTypeA;

uint32_t ui32StrengthB;
uint32_t ui32PinTypeB;


// function to turn leds on or off; useful for debugging certain parts of the code
void debugled(bool onoff){

    //SysCtlClockSet(SYSCTL_SYSDIV_2 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_25MHZ); //SYSCTL_XTAL_16MHZ             SYSCTL_SYSDIV_4

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);                  //for debugging delay frequency with oscilloscope
//    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);

    if(onoff == true){
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, GPIO_PIN_1);


        //ROM_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);  //pindebug
    }else{
        GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);


        //ROM_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0x00);        //pindebug
    }
}


// setup of the SPI connection
void SPIbegin() {
    uint32_t initialData = 0;

    // mostly copied from default SPI connection example, +/- some code snippets after hours of experimenting

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_2 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_25MHZ); // using the external crystal for improved accuracy

    //
    // The SSI0 peripheral must be enabled for use.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);

    //
    // For this example SSI0 is used with PortA[5:2].  The actual port and pins
    // used may be different on your part, consult the data sheet for more
    // information.  GPIO port A needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    //
    // Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //


    ROM_GPIOPinConfigure(GPIO_PD3_SSI2CLK);     //clock
    ROM_GPIOPinConfigure(GPIO_PD2_SSI2FSS);     //chipselect    x    GPIO_PD2_SSI2FSS
    ROM_GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);   //master out slave in
    ROM_GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);   //master in slave out


    // Configure the GPIO settings for the SSI pins.  This function also gives
    // control of these pins to the SSI hardware.  Consult the data sheet to
    // see which functions are allocated per pin.
    // The pins are assigned as follows:
    //      PA5 - SSI0Tx    transmitting pin(miso, masterout slave in)
    //      PA4 - SSI0Rx    receiving pin   (mosi, masterin slave out)
    //      PA3 - SSI0Fss   chipselect
    //      PA2 - SSI0CLK   clock pin               PD_3
    // TODO: change this to select the port/pin you are using.
    //

    ROM_GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);


    //
    // Configure and enable the SSI port for SPI master mode.  Use SSI0,
    // system clock supply, idle clock level low and active low clock in
    // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
    // For SPI mode, you can set the polarity of the SSI clock when the SSI
    // unit is idle.  You can also configure what clock edge you want to
    // capture data on.  Please reference the datasheet for more information on
    // the different SPI modes.
    //
    ROM_SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM); //ROM_SSIClockSourceSet(SSI2_BASE, SSI_CLOCK_SYSTEM);

    ROM_SSIConfigSetExpClk(SSI2_BASE, ui32PWMClock, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 8000000, 8);


    // Enable the SSI0 module.

    ROM_SSIEnable(SSI2_BASE);


    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    while(ROM_SSIDataGetNonBlocking(SSI2_BASE, &initialData)){
    }
}

// function to end SPI connection
void SPIend() {
    ROM_SSIDisable(SSI2_BASE);
}

// lsbfirst (least-significant bit first) or msbfirst(most ...)     determines the order of bits shifted out and into spi
void SPIsetBitOrder(uint8_t bitOrder) {
    SSIBitOrder = bitOrder;
}
void SPIsetBitOrder2(uint8_t ssPin, uint8_t bitOrder) {
    SSIBitOrder = bitOrder;
}


// sets the mode for spi communication   example: leading edge Sample rising, trailing edge Setup falling
void SPIsetDataMode(uint8_t mode) {
    HWREG(SSI2_BASE + SSI_O_CR0) &= ~(SSI_CR0_SPO | SSI_CR0_SPH);
    HWREG(SSI2_BASE + SSI_O_CR0) |= mode;
}

//setter function for the spi clockdivider
void SPIsetClockDivider(uint8_t divider){
  //value must be even
    HWREG(SSI2_BASE + SSI_O_CPSR) = divider;
}

//function to get the lower byte from 2 bytes       useful for sending 2 byte(16bit) data on 8 bit connection
uint8_t lowByte(uint16_t w){
    uint8_t low = w & 0xff;
    //printf("%x\n", lowByte((uint16_t) 0x00FF));
    return low;
}

//function to get the upper byte from 2 bytes
uint8_t highByte(uint16_t w){
    uint8_t high =(w>>8) & 0xff;
    //printf("%x\n", highByte((uint16_t) 0x11ff));
    return high;
}

// delay function; delays the systems functionality for a certain amount of time
void delayms(uint32_t n){

    ROM_SysCtlDelay(n*40000);    //5333@16000000  //400000@120MHZ(120000000)
    //16mhz processor  1cycle =  1second/16 000 000 = 0, 000 000 187 5 (1x3cycles); nanoseconds  1 000 000 000 (second/nanosecond) / 1 cycle  = 5 333 333  1S (5 333ms)

}

// this function activates the buzzer for a short amount of time;
void bleep(){

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    delayms(5);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
}

// function that actually transmits data to the display controller via SPI connection
uint8_t transfer(uint8_t data) {
    //unsigned long rxtxData;
    uint32_t rxtxData;
    rxtxData = data;

    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x000);

    ROM_SSIDataPut(SSI2_BASE, rxtxData);

    while(ROM_SSIBusy(SSI2_BASE)){};

    ROM_SSIDataGet(SSI2_BASE, &rxtxData);

    return (uint8_t) rxtxData;
}

// function to send command to the display
void writeCommand(uint8_t command8)
{
    //HWREG(GPIO_PORTF_BASE + (GPIO_O_DATA + ((GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3) << 2))) ^= ~0;

    //pinDataCommand LOW
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, 0x00);        //datacommand low   1: data   0: command
    //pinChipSelect LOW
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x000);        //chipselect low means writing
    //transfer
    transfer(command8);
    //pinChipSelect HIGH
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);  //chipselect high ... writing over
}

// function to send data to the display
void writeData(uint8_t data8)
{
//    digitalWrite(_pinDataCommand, HIGH);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);  //datacommand high means data transmission NOT command
//    digitalWrite(_pinChipSelect, LOW);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x000);        //chipselect low means writing
    //transfer
    transfer(data8);

//    digitalWrite(_pinChipSelect, HIGH);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);  //chipselect high ... writing over
}

// function for writing 16 bits of data by splitting it into 2 parts high and low
void writeData16(uint16_t data16)
{
//    digitalWrite(_pinDataCommand, HIGH);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
//    digitalWrite(_pinChipSelect, LOW);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x000);

    transfer(highByte(data16));
    transfer(lowByte(data16));

//    digitalWrite(_pinChipSelect, HIGH);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

// function for writing 8 bits 2 times in a row
void writeData88(uint8_t dataHigh8, uint8_t dataLow8)
{
//    digitalWrite(_pinDataCommand, HIGH);
    GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);


//    digitalWrite(_pinChipSelect, LOW);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, 0x000);
    transfer(dataHigh8);
    transfer(dataLow8);
//    digitalWrite(_pinChipSelect, HIGH);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
}

// function for writing 8 bits 4 times in a row
void writeData8888(uint8_t dataHigh8, uint8_t dataLow8, uint8_t data8_3, uint8_t data8_4)
{
    writeData(dataHigh8);
    writeData(dataLow8);
    writeData(data8_3);
    writeData(data8_4);
}

// writing on display registers
void writeRegister(uint8_t command8, uint8_t data8)
{
    writeCommand(command8);
    writeData(data8);
}

// setter function for the backlight pin
void setBacklight(bool flag)
{
    if( _pinBacklight!= NULL){
        if(flag == true){
            ROM_GPIOPinWrite(GPIO_PORTF_AHB_BASE, GPIO_PIN_2, GPIO_PIN_2);
        }else if(flag == false){
            ROM_GPIOPinWrite(GPIO_PORTF_AHB_BASE, GPIO_PIN_2, 0x00);    //backlight
        }
    }
}

void setDisplay(bool flag)
{
    if (_pinBacklight!=NULL) setBacklight(flag);
}


//sets the orientation of the display by writing data on the controller
void setOrientation(uint8_t orientation)
{
    _orientation = orientation % 6;
    writeCommand(HX8353E_MADCTL);
    switch (_orientation) {
        case 0:
            writeData(HX8353E_MADCTL_MX | HX8353E_MADCTL_ML | HX8353E_MADCTL_RGB);
            break;
        case 1:
            writeData(HX8353E_MADCTL_MY | HX8353E_MADCTL_ML | HX8353E_MADCTL_RGB);
            break;
        case 2:
            writeData(HX8353E_MADCTL_RGB);
            break;
        case 3:
            writeData(HX8353E_MADCTL_MX | HX8353E_MADCTL_MV | HX8353E_MADCTL_RGB);
            break;
        case 4:
            writeData(HX8353E_MADCTL_MX | HX8353E_MADCTL_MY | HX8353E_MADCTL_RGB);
            break;
        case 5: //orientation for text, not the best solution
            writeData(HX8353E_MADCTL_MX | HX8353E_MADCTL_MY | HX8353E_MADCTL_RGB);
            break;

/*
 * values for orientation
#define HX8353E_MADCTL_MY  0x80
#define HX8353E_MADCTL_MX  0x40
#define HX8353E_MADCTL_MV  0x20
#define HX8353E_MADCTL_ML  0x10
#define HX8353E_MADCTL_RGB 0x08
#define HX8353E_MADCTL_MH  0x04
*/
    }
}

// define the position/border of the window, on the display
void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
    switch (_orientation) {
        case 0:
            x0 += 2;
            y0 += 1;
            x1 += 2;
            y1 += 1;
            break;
        case 1:
            x0 += 3;
            y0 += 2;
            x1 += 3;
            y1 += 2;
            break;
        case 2:
            x0 += 2;
            y0 += 1;
            x1 += 2;
            y1 += 1;
            break;
        case 3:
            x0 += 1;
            y0 += 2;
            x1 += 1;
            y1 += 2;
            break;
        case 4:
            x0 += 2;
            y0 += 3;
            x1 += 2;
            y1 += 3;
            break;
        default:
            break;
    }
    writeCommand(HX8353E_CASET);
    writeData16(x0);
    writeData16(x1);
    writeCommand(HX8353E_RASET);
    writeData16(y0);
    writeData16(y1);
    writeCommand(HX8353E_RAMWR);
}

// this function draws a point on the screen
// x coord,  y coord,  colour
void setPoint(uint16_t x1, uint16_t y1, uint16_t colour)
{
    if( (x1 < 0) || (x1 >= 128) || (y1 < 0) || (y1 >= 128) ) return;
    setWindow(x1, y1, x1+1, y1+1);

    writeData16(colour);

}

//same as setpoint
void point(uint16_t x1, uint16_t y1, uint16_t colour)
{
    setPoint(x1, y1, colour);
}

void swap16(uint16_t *a, uint16_t *b)
{
    uint16_t w = *a;
    *a = *b;
    *b = w;
}

void swap8(uint8_t *a, uint8_t *b)
{
    uint8_t w = *a;
    *a = *b;
    *b = w;
}


// fills a rectangle with a colour;
void fastFill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour)
{
    if (x1 > x2){swap16(&x1, &x2);}
    if (y1 > y2){swap16(&y1, &y2);}

    uint32_t counter = (y2-y1+1)*(x2-x1+1);

    setWindow(x1, y1, x2, y2);
    ROM_GPIOPinWrite(GPIO_PORTL_BASE, GPIO_PIN_3, GPIO_PIN_3);
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);
    uint8_t hi = highByte(colour);
    uint8_t lo = lowByte(colour);

    for(counter; counter>0 ; counter--) {
        transfer(hi);
        transfer(lo);
    }
    ROM_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_2, GPIO_PIN_2);

}

// draws a line between 2 positions
void line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour)
{
    if ((x1 == x2) && (y1 == y2)) {
        setPoint(x1, y1, colour);
    } else if ((x1 == x2) || (y1 == y2)) {
        fastFill(x1, y1, x2, y2, colour);
    } else {
        uint16_t wx1 = (uint16_t)x1;
        uint16_t wx2 = (uint16_t)x2;
        uint16_t wy1 = (uint16_t)y1;
        uint16_t wy2 = (uint16_t)y2;
        bool flag = abs(wy2 - wy1) > abs(wx2 - wx1);
        if (flag) {
            swap16(&wx1, &wy1);
            swap16(&wx2, &wy2);
        }
        if (wx1 > wx2) {
            swap16(&wx1, &wx2);
            swap16(&wy1, &wy2);
        }
        int16_t dx = wx2 - wx1;
        int16_t dy = abs(wy2 - wy1);
        int16_t err = dx / 2;
        int16_t ystep;
        if (wy1 < wy2) ystep = 1;
        else ystep = -1;
        for (; wx1<=wx2; wx1++) {
            if (flag){setPoint(wy1, wx1, colour);}
            else{setPoint(wx1, wy1, colour);}
            err -= dy;
            if (err < 0) {
                wy1 += ystep;
                err += dx;
            }
        }
    }
}


//draws a line between 2 points
//              coords line 1               coords line 2               color
void dLine(uint16_t x0, uint16_t y0, uint16_t dx, uint16_t dy, uint16_t colour)
{
    line(x0, y0, x0+dx-1, y0+dy-1, colour);
}


//draws a rectangle between 2 points
void rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t colour)
{
    if (_penSolid == false) {
        line(x1, y1, x1, y2, colour);
        line(x1, y1, x2, y1, colour);
        line(x1, y2, x2, y2, colour);
        line(x2, y1, x2, y2, colour);
    } else {
        fastFill(x1, y1, x2, y2, colour);
    }
}

// pensolid fills the object with a certain colour
void setPenSolid(bool flag)
{
    _penSolid = flag;
}

// these functions provide the screen size
uint16_t screenSizeX()
{
    switch (_orientation) {
        case 0:
        case 2:
            return _screenWidth;
            break;
        case 1:
        case 3:
            return _screenHeigth;
            break;
    }
}
uint16_t screenSizeY()
{
    switch (_orientation) {
        case 0:
        case 2:
            return _screenHeigth;
            break;
        case 1:
        case 3:
            return _screenWidth;
            break;
    }
}

// reads the values from the 2 buttons on the educational booster pack mkII
void readButtons(){
//    buttonA = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_1);
//    buttonB = GPIOPinRead(GPIO_PORTL_BASE, GPIO_PIN_2);


    //uint8_t ui8toggle2 = (1<<0);

    GPIO_PORTL_DEN_R |=  (1<<1);
    GPIO_PORTL_DIR_R &= ~(1<<1);

    GPIO_PORTL_DEN_R |=  (1<<2);
    GPIO_PORTL_DIR_R &= ~(1<<2);

    /* Enable pull-up for button on pin 0 of port J */
    GPIO_PORTL_PUR_R |= (1<<1);
    GPIO_PORTL_PUR_R |= (1<<2);


    buttonA = HWREG(GPIO_PORTL_DATA_BITS_R + (1<<1));
    buttonB = HWREG(GPIO_PORTL_DATA_BITS_R + (1<<2));
}

// clear entire screen with 1 colour
void clear(uint16_t colour)
{
    uint8_t oldOrientation = _orientation;
    bool oldPenSolid = _penSolid;
    setOrientation(0);
    setPenSolid(true);
    rectangle(0, 0, screenSizeX()-1, screenSizeY()-1, colour);
    setOrientation(oldOrientation);
    setPenSolid(oldPenSolid);
}

/*

 starts SPI connection, initalizes screen,  turns on/off certain pins with certain delay to start communication
 with the display controller, writes settings and values for the screen(orientation, ...)

 */
void screenBegin(){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

//    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R12 | SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R10 | SYSCTL_RCGCGPIO_R7;
//    //                          12 N                5 F                     L10                 H7


    SPIbegin();
    SPIsetClockDivider(SPI_CLOCK_DIV2);
    SPIsetBitOrder(MSBFIRST);
    SPIsetDataMode(SPI_MODE0);          //SPI_MODE0   0x00

    if (_pinReset!=NULL){GPIOPinTypeGPIOOutput(GPIO_PORTH_AHB_BASE, GPIO_PIN_3);}//reset
    if (_pinBacklight!=NULL){GPIOPinTypeGPIOOutput(GPIO_PORTF_AHB_BASE, GPIO_PIN_2);}//backlight


    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_3);         //datacommand
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_2);         //chipselect

    if (_pinBacklight!=NULL){GPIOPinWrite(GPIO_PORTF_AHB_BASE, GPIO_PIN_2, GPIO_PIN_2);}//backlight     high                      GPIO_PIN_2               changed to high !!!!!
    if (_pinReset!=NULL){GPIOPinWrite(GPIO_PORTH_AHB_BASE, GPIO_PIN_3, GPIO_PIN_3);}//reset        high

    ////    uint8_t    _pinReset          = 17;//PH_3//pin17;
    ////    uint8_t    _pinDataCommand    = 31;//PL_3//pin31;
    ////    uint8_t    _pinChipSelect     = 13;//PN_2//pin13;
    ////    uint8_t    _pinBacklight      = NULL;  // PF2??

    //pulling certain pins down and up in a specific timeframe, allowws to read and write on specific registers on the display circuit
    //for further information address the ST7735S Datasheet, this is the datasheet for the display controller used on the EducationalBoosterPack MKII

    delayms(100);

    if (_pinReset!=NULL){GPIOPinWrite(GPIO_PORTH_AHB_BASE, GPIO_PIN_3, 0x00);}//reset        low

    delayms(50);

    if (_pinReset!=NULL){GPIOPinWrite(GPIO_PORTH_AHB_BASE, GPIO_PIN_3, GPIO_PIN_3);}//reset        high

    delayms(120);
    writeCommand(HX8353E_SWRESET);
    delayms(150);
    writeCommand(HX8353E_SLPOUT);
    delayms(200);
    writeRegister(HX8353E_GAMSET, 0x04);
    writeCommand(HX8353E_SETPWCTR);
    writeData88(0x0A, 0x14);
    writeCommand(HX8353E_SETSTBA);
    writeData88(0x0A, 0x00);
    writeRegister(HX8353E_COLMOD, 0x05);
    delayms(10);
    writeRegister(HX8353E_MADCTL, HX8353E_MADCTL_RGB);
    writeCommand(HX8353E_CASET);
    writeData8888(0x00, 0x00, 0x00, 0x79);
    writeCommand(HX8353E_RASET);
    writeData8888(0x00, 0x00, 0x00, 0x79);
    writeCommand(HX8353E_NORON);
    delayms(10);
    writeCommand(HX8353E_DISPON);
    delayms(120);
    writeCommand(HX8353E_RAMWR);
    setBacklight(true);
    setOrientation(0);
    _screenWidth  = HX8353E_WIDTH;
    _screenHeigth = HX8353E_HEIGHT;
    _penSolid  = false;
    _fontSolid = true;
    _flagRead  = false;
    _touchTrim = 0;
    clear(blackColour);
}


//  draws a single character at coordinates
void drawChar(int16_t x, int16_t y, char c, int16_t textColor, int16_t bgColor, uint8_t size){
  uint8_t line; // horizontal row of pixels of character
  int32_t col, row, i, j;// loop indices


  if(((x + 6*size - 1) >= HX8353E_WIDTH)  || // Clip right
     ((y + 8*size - 1) >= HX8353E_HEIGHT) || // Clip bottom
     ((x + 6*size - 1) < 0)        || // Clip left
     ((y + 8*size - 1) < 0)){         // Clip top
    return;
  }


  uint8_t oldOrientation = _orientation;

  setOrientation(5);

  setWindow(x, y, x+6*size-1, y+8*size-1);

  //setWindow(x+2*size, y+3*size, x, y);

  line = 0x01;        // print the top row first
  // print the rows, starting at the top
  for(row=0; row<8; row=row+1){
    for(i=0; i<size; i=i+1){
      // print the columns, starting on the left
      for(col=0; col<5; col=col+1){
        if(Font[(c*5)+col]&line){
          // bit is set in Font, print pixel(s) in text color
          for(j=0; j<size; j=j+1){
            writeData88(textColor,textColor);
          }
        } else{
          // bit is cleared in Font, print pixel(s) in background color
          for(j=0; j<size; j=j+1){
              writeData88(bgColor,bgColor);
          }
        }
      }
      // print blank column(s) to the right of character
      for(j=0; j<size; j=j+1){
          writeData88(bgColor,bgColor);
      }
    }
    line = line<<1;   // move up to the next row
  }
  setOrientation(oldOrientation);
}

// draws an array of characters
uint32_t drawString(uint16_t x, uint16_t y, char *pt, int16_t textColor, int16_t backgroundColor, int16_t textSize){
  uint32_t count = 0;
  if(y>12) return 0;
  while(*pt){
    drawChar(x+(6*x), y+(10*y), *pt, textColor, backgroundColor, textSize);
    //x*6, y*10,
    pt++;
    x = x+1;
    if(x>20) return count;  // number of characters printed
    count++;
  }
  return count;  // number of characters printed
}


// additional function for langtons ant, checks the colour of the field, inverts colour, draws on display
void antMove(uint16_t i, uint16_t j, uint16_t colour){

    if(colour == blackColour){
        _map[0][i][j]=1;
    }else if(colour == whiteColour){
        _map[0][i][j]=0;
    }
    setPoint(i,j, colour);
}


//function to start langtons ant mode
void langtonsAnt(){
/*
    Squares on a plane are colored variously either black or white. We arbitrarily identify one square as the "ant".
    The ant can travel in any of the four cardinal directions at each step it takes. The "ant" moves according to the rules below:

    At a white square, turn 90° clockwise, flip the color of the square, move forward one unit
    At a black square, turn 90° counter-clockwise, flip the color of the square, move forward one unit

    Source: https://en.wikipedia.org/wiki/Langton's_ant
*/

     //variables for the initial loop
    //init uint8_t _map[2][128][128];
     uint16_t i=0;
     uint16_t j=0;

     //field/map/screen is bein generated ... in white color
     for (i=0; i < 128; i++)
     {
         for (j=0; j < 128; j++)
         {
             antMove(i,j, whiteColour);

         }
     }

     //variables for the movement of the and and the status
     //number of round; a limit for the number of moves could be implemented here
     uint16_t roundsLimit= 50000;
     uint16_t roundsCurrent= 0;
     uint16_t posx= 64;
     uint16_t posy= 64;


     //the direction in which the ant moves
     uint16_t direction;
     //Status: ['1'black]['0'white] directions: '0'north '1'east '2'south '3'west

     direction = 1;  //starting direction

     //this is the initial position of the ant  64,64 ... middle of the screen
     antMove(posx,posy, blackColour); //setting start position

     while(roundsCurrent != roundsLimit){
        delayms(1);

        //checking which direction the ant is facing
        switch(direction)
        {
            case 1:     //North
                posy++;

                break;

            case 2:     //East
                posx++;

                break;

            case 3:     //South
                posy--;

                break;

            case 4:     //West
                posx--;

                break;
        }
//        At a white square, turn 90° clockwise, flip the color of the square, move forward one unit
//        At a black square, turn 90° counter-clockwise, flip the color of the square, move forward one unit
        if(_map[0][posx][posy]==1){
            direction--;
            antMove(posx,posy, whiteColour);
            if(direction==0){direction=4;}
        }else if(_map[0][posx][posy]==0){
            direction++;
            antMove(posx,posy, blackColour);
            if(direction==5){direction=1;}
        }
        roundsCurrent++;

        //checkin AB buttons so that the user can leave this mode
        readButtons();
        if(buttonA == 0 && buttonB == 0){
            bleep();
            break;
        }
     }
}

// picture array for mandelbulb
uint16_t picture[128][128];

//prints the mandelbulb colours from picture array on the display
void printMandelbrot(){

    uint16_t x;
    uint16_t y;
    for(x=0; x<128; x++){
        for(y=0; y<128; y++){
            //setPoint(x, y, picture[x][y]+0x44);  //fancy mode
            setPoint(x, y, colours[picture[x][y]]);
        }
    }
}

// this function generates the colours relative to the zoom and position
void mandelbrot(uint16_t widthArg,
                uint16_t heightArg,
                float offsetx,
                float offsety,
                float zoomlevel,
                uint8_t coloroffset){


    uint16_t zoomRes = 0;     //control the zoom

    uint16_t color;

    //skewed offsets WARNING
    float zoom = -14;           //zooming out from center ... relative to x and y offsets  NOT REAL ZOOM
    float x0 = 0.1*zoom;      //y size
    float y0 = 0.1*zoom;        //x size

    float x1 = -0.2;    //x axis offset         - moves right // + moves left
    float y1 = -0.85;    //y axis offset         - moves up    // + moves down
    //

    int maxi = 99;      // color generation?

    //x und y Koordinate im imaginären Bild, specs above affect x and y values bellow
    float x;
    float y;


    uint16_t iteratorX = 0;
    uint16_t iteratorY = 0;


    //Skaliere für jeden Punkt der Computergrafik den dazu gehörigen Punkt der imaginären Grafik
    for (iteratorY=0; iteratorY<128; iteratorY++) {

        x = (x0 + (x1 - x0) / ((float)80)*(float)iteratorY);

        x=x*zoomlevel+offsetx;//zoom level ...  + zoom out // - zoom in

        for (iteratorX=0; iteratorX<128; iteratorX++) {

            y = (y0 + (y1 - y0) / ((float)25)*(float)iteratorX);

            y=y*zoomlevel+offsety;

            float zr = 0;       // rotation
            float zi = 0;     // stretching
            int i;

            //calculating colours for 2 dimensional mandelbulb(picture) array
            for (i = 0; i < maxi; ++i) {
                // calculate next iteration
                // For the interested reader: z=zr+i*zi and c=x+i*y are complex numbers
                // In the next row we calculate z * z + c and check whether |z| > 2
                float nextzr = zr * zr - zi * zi + x;
                float nextzi = 2 * zr * zi + y;
                // are we done?
                if (nextzr * nextzr + nextzi * nextzi > 4) {
                    // TODO: set pixel to i % 70
                    // Insert here code to draw
                    //cout << "Pixel: " << t << "," << s << " ist " << i << "|";


                    break;
                }
                zr = nextzr;
                zi = nextzi;

            }

            color= i%70;    //  modulo 70 for the array of colours

            if(color>=69){
                color=69;
                color-=coloroffset;
            }else if(color<=0){
                color=0;
                color+=coloroffset;
            }

            picture[iteratorY][iteratorX]=color;

        }
    }
}

// initializes joystick,
void initJoystick(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_4);
    ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 1);
}

//  initializes microphone/ experimental
void initMicrophone(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_5 | GPIO_PIN_0);
    ADCSequenceConfigure(ADC1_BASE, 2, ADC_TRIGGER_PROCESSOR, 1);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 0, ADC_CTL_CH3);
    ADCSequenceStepConfigure(ADC1_BASE, 2, 1, ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 2);
}


uint16_t main(void)
{
    //setting cpu clock speed
    g_ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), ui32PWMClock);

    initJoystick();         //start joystick
    initMicrophone();       //start microphone  experimental!

    screenBegin();      //start screen

    delayms(10);

    //instructions      user has to reset device to read instructions again

    drawString(5, 1, "Mandelbrot", whiteColour, blackColour, 1);
    drawString(9, 2, "&", whiteColour, blackColour, 1);
    drawString(4, 3, "Langtons Ant", whiteColour, blackColour, 1);

    drawString(1, 5, "Instructions:", whiteColour, blackColour, 1);
    drawString(3, 6, "Joystick:Move", whiteColour, blackColour, 1);
    drawString(3, 7, "Button A:Zoom+", whiteColour, blackColour, 1);
    drawString(3, 8, "Button B:Zoom-", whiteColour, blackColour, 1);
    drawString(1, 10, "ChangeMode: A+B", whiteColour, blackColour, 1);


    delayms(8000);      //8 seconds delay to read instructions


    //uint16_t hue = 0;     // experimental value to pick colour with joystick(not implemented)

    // mandelbrot arguments
    float zoom = 1;
    float moveX = 0;
    float moveY = 0;
    uint8_t random =0;  // pseudo random/random number to alter colour ...  experimental


    //this loop keeps repeating during the operation
    while(1){

        uint32_t ui32ADCValues2[4];
        ADCIntClear(ADC1_BASE, 2);
        ADCProcessorTrigger(ADC1_BASE, 2);
        while (!ADCIntStatus(ADC1_BASE, 2, false))
        {
        }
        ADCSequenceDataGet(ADC1_BASE, 2, ui32ADCValues2); //mic recording
        printf("%d\n", ui32ADCValues2[2]);
        random = ui32ADCValues2[2]%16;


        // genereate/calculate 2dimensional array of colours
        mandelbrot(128,128,moveX,moveY, zoom, random);

        // print/show colours on display
        printMandelbrot();
        //clear(blackColour);



        //JOYSTICK              reading joystick values, determine if its up,down,left, right
        ADCIntClear(ADC0_BASE, 1);
        ADCProcessorTrigger(ADC0_BASE, 1);
        while (!ADCIntStatus(ADC0_BASE, 1, false))
        {
        }
        ADCSequenceDataGet(ADC0_BASE, 1, ui32ADCValues);
        ui32X = ui32ADCValues[1];
        ui32Y = ui32ADCValues[0];

        if(ui32X<600){
            moveX-= 0.1;
            bleep();
        }
        if(ui32X>3400){
            moveX+= 0.1;
            bleep();
        }
        if(ui32Y<600){
            moveY-= 0.1;
            bleep();
        }
        if(ui32Y>3400){
            moveY+= 0.1;
            bleep();
        }
        //JOYSTICK END


        //reading button values for zoom/ change mode
        readButtons();

        if(buttonA == 0){
//            delayms(20);
//            if(buttonA == 0){
            zoom-=0.1;
//            }
            bleep();
        }
        if(buttonB == 0){
//            delayms(20);
//            if(buttonB == 0){
            zoom+=0.1;
//            }
            bleep();
        }
        if(buttonA == 0 && buttonB == 0){       //changing mode
            langtonsAnt();
            bleep();
        }
    }


//    while(1){         // led debugging
//        delayms(1000);
//        debugled(true);
//        delayms(1000);
//        debugled(false);
//    }

    return 0;
}


