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
#include "DS1722.h"


// HTML content
char* webpageStart =
"<!DOCTYPE html><html><head><title>E155 Web Server</title>\
<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0\"></head>\
<body><h1>Broderick Bownds: E155 IoT Temperature Server</h1>";

char* ledStr =
"<p>LED Control:</p>\
<form action=\"ledon\"><input type=\"submit\" value=\"Turn LED On!\"></form>\
<form action=\"ledoff\"><input type=\"submit\" value=\"Turn LED Off!\"></form>";

char* tempStr =
"<p>Temperature Resolution:</p>\
<form action=\"precision8\"><input type=\"submit\" value=\"8-bit\"></form>\
<form action=\"precision9\"><input type=\"submit\" value=\"9-bit\"></form>\
<form action=\"precision10\"><input type=\"submit\" value=\"10-bit\"></form>\
<form action=\"precision11\"><input type=\"submit\" value=\"11-bit\"></form>\
<form action=\"precision12\"><input type=\"submit\" value=\"12-bit\"></form>";

char* webpageEnd = "</body></html>";

// Checks if a substring exists inside a string
int inString(char request[], char des[]) {
    if (strstr(request, des) != NULL) {return 1;}
    return -1;
}

int led_status = 0;

// updateLEDStatus controls the LED based on the request
int updateLEDStatus(char request[])
{
    // The request has been received. now process to determine whether to turn the LED on or off
    if (inString(request, "ledoff")==1) {
        digitalWrite(LED_PIN, PIO_LOW);
        led_status = 0;
        return led_status;    // Ensures LED status is accurate when bit resolution is selected
    }
    else if (inString(request, "ledon")==1) {
        digitalWrite(LED_PIN, PIO_HIGH);
        led_status = 1;
        return led_status;
    }
    return led_status;
}




// Handles DS1722 precision requests
void newprecision(char request[]) {
	int precision = 9;

	if      (inString(request, "precision8") == 1) precision = 8;
	else if (inString(request, "precision9") == 1) precision = 9;
	else if (inString(request, "precision10") == 1) precision = 10;
	else if (inString(request, "precision11") == 1) precision = 11;
	else if (inString(request, "precision12") == 1) precision = 12;

	setprecision(precision);
}

/////////////////////////////////////////////////////////////////
// Main Function
/////////////////////////////////////////////////////////////////
int main(void) {
	configureFlash();
	configureClock();

	gpioEnable(GPIO_PORT_A);
	gpioEnable(GPIO_PORT_B);
	gpioEnable(GPIO_PORT_C);

	pinMode(LED_PIN, GPIO_OUTPUT);

	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
	initTIM(TIM15);

	// Initialize SPI for DS1722: CPOL=0, CPHA=1
	initSPI(0b111, 0, 1);
	setprecision(12); // Default: highest resolution


	USART_TypeDef *USART = initUSART(USART1_ID, 125000);
	printf("E155 IoT SPI Temperature Server Initialized\n");

        /*while(1){
        digitalWrite(SPI_CE, 1);
        spiSendReceive(0x80);
        spiSendReceive(0xE0);
        digitalWrite(SPI_CE, 0);

        digitalWrite(SPI_CE, 1);
        spiSendReceive(0x00);
        char test = spiSendReceive(0xE0);
        digitalWrite(SPI_CE, 0);
        printf("test = %X", test);
        }
        */


	while (1) {
		// Wait for ESP8266 request
		char request[64] = " ";
		int index = 0;

		while (inString(request, "\n") == -1) {
			while (!(USART->ISR & USART_ISR_RXNE));
			request[index++] = readChar(USART);
		}

		// Process web request
		int led_status = updateLEDStatus(request);
		newprecision(request);

                /*
                 uint8_t msb = 0xFF;
                 uint8_t lsb = 0x80;
                 int16_t raw = ((int16_t)msb << 8) | lsb;
                 float temp = (float)raw/ 256.0f;
                 */
		float temp = readTemp();

		// Prepare status messages
		char ledMsg[32];
		if (led_status) sprintf(ledMsg, "LED is ON");
		else sprintf(ledMsg, "LED is OFF");

		char tempMsg[48];
		sprintf(tempMsg, "Temperature: %.4f C " " (Resolution: %d bit)", temp, precisiondefault);

		// Send the full webpage back
		sendString(USART, webpageStart);
		sendString(USART, ledStr);
		sendString(USART, tempStr);
		sendString(USART, "<h2>Status</h2><p>");
		sendString(USART, ledMsg);
		sendString(USART, "<br>");
		sendString(USART, tempMsg);
		sendString(USART, "</p>");
		sendString(USART, webpageEnd);
	}
}

