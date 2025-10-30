// main.c
// Broderick Bownds
// brbownds@hmc.edu
// 10/20/2025
//
// This program controls an LED and a DS1722 temperature sensor over SPI,
// responding to web requests via the ESP8266 to make the HTML page showcasing temperature,
// LED status, resolution based on the most and least significant bits. 
#include <string.h>
#include <stdio.h>
#include "main.h"
#include "STM32L432KC_GPIO.h"
#include "STM32L432KC_SPI.h"
#include "STM32L432KC_TIM.h"

