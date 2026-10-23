#pragma once

/* Pines GPIO del NodeMCU ESP8266 */
#define SENSOR   14  /* D5 */
#define INICIO   12  /* D6 */
#define CAL      13  /* D7 */

#define LCD_SDA   4  /* D2 */
#define LCD_SCL   5  /* D1 */
#define LCD_ADDR  0x27

#define ROJO      0  /* D3, activo en bajo */
#define VERDE     2  /* D4, activo en bajo */
#define AZUL     16  /* D0, activo en alto */
#define BUZZER   15  /* D8, mediante transistor */

/* Pulsador de reinicio conectado entre RST y GND. */
