#include "UartRingbuffer_multi.h"
#include "ESP8266_HAL.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "DFPLAYER_MINI.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

#define wif_uart &huart1
#define pcc_uart &huart2
//////////////
#define LED_ON_DURATION_MS 2000

// Variable global para rastrear el estado del LED y cuándo se encendió
uint32_t led1_turned_on_time = 0;
// Definir las constantes para representar el estado de los LED
#define LED1_OFF 0
#define LED1_ON 1
#define LED2_OFF 0
#define LED2_ON 1
#define LED3_OFF 0
#define LED3_ON 1
#define LED4_OFF 0
#define LED4_ON 1
#define LED5_OFF 0
#define LED5_ON 1

// Variables para almacenar el estado de los LED
int led1_state = LED1_OFF;
int led2_state = LED2_OFF;
int led3_state = LED3_OFF;
int led4_state = LED4_OFF;
int led5_state = LED5_OFF;
char buffer[20];

const char *Basic_inclusion =
		"<!DOCTYPE html>"
				"<html>"
				"<head>"
				"  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">"
				"  <title>ASCENSOR</title>"
				"  <style>"
				"    html { font-family: Helvetica, sans-serif; text-align: center; }"
				"    body { margin-top: 50px; }"
				"    h1 { color: #444444; margin: 50px auto 30px; }"
				"    h3 { color: #444444; margin-bottom: 50px; }"
				"    .container {"
				"      display: inline-block;"
				"      border: 2px solid #ccc;"
				"      padding: 20px;"
				"      border-radius: 8px;"
				"      margin-bottom: 30px;"
				"      background-color: #868686;"
				"    }"
				"    .button {"
				"      display: block;"
				"      width: 80px;"
				"      background-color: #1abc9c;"
				"      border: 2px solid #ccc;"
				"      color: white;"
				"      padding: 13px 30px;"
				"      text-decoration: none;"
				"      font-size: 25px;"
				"      margin: 10px auto;"
				"      cursor: pointer;"
				"      border-radius: 4px;"
				"    }"
				"    .button-on { background-color: #1abc9c; }"
				"    .button-on:active { background-color: #16a085; }"
				"    .button-off { background-color: #34495e; }"
				"    .button-off:active { background-color: #2c3e50; }"
				"    .button-red { background-color: #e74c3c; }"
				"    .button-red:active { background-color: #c0392b; }"
				"    .button-brown { background-color: #583737; }"
				"    .button-brown:active { background-color: #583737; }"
				"    p { font-size: 14px; color: #000000; margin-bottom: 10px; }"
				"  </style>"
				"</head>"
				"<body>"
				"  <h1>NIVELES DEL ASCENSOR</h1>";

char *Terminate = "</body></html>";

/*****************************************************************************************************************************************/

void ESP_Init(char *SSID, char *PASSWD) {
	char data[80];

	Ringbuf_init();

	Uart_sendstring("AT+RST\r\n", wif_uart);
	Uart_sendstring("RESETTING.", pcc_uart);
	for (int i = 0; i < 5; i++) {
		Uart_sendstring(".", pcc_uart);
		HAL_Delay(1000);
	}

	/********* AT **********/
	Uart_sendstring("AT\r\n", wif_uart);
	while (!(Wait_for("AT\r\r\n\r\nOK\r\n", wif_uart)))
		;
	Uart_sendstring("AT---->OK\n\n", pcc_uart);

	/********* AT+CWMODE=1 **********/
	Uart_sendstring("AT+CWMODE=1\r\n", wif_uart);
	while (!(Wait_for("AT+CWMODE=1\r\r\n\r\nOK\r\n", wif_uart)))
		;
	Uart_sendstring("CW MODE---->1\n\n", pcc_uart);

	/********* AT+CWJAP="SSID","PASSWD" **********/
	Uart_sendstring("connecting... to the provided AP\n", pcc_uart);
	sprintf(data, "AT+CWJAP=\"%s\",\"%s\"\r\n", SSID, PASSWD);
	Uart_sendstring(data, wif_uart);
	while (!(Wait_for("WIFI GOT IP\r\n\r\nOK\r\n", wif_uart)))
		;
	sprintf(data, "Connected to,\"%s\"\n\n", SSID);
	Uart_sendstring(data, pcc_uart);

	/********* AT+CIFSR **********/
	Uart_sendstring("AT+CIFSR\r\n", wif_uart);
	while (!(Wait_for("CIFSR:STAIP,\"", wif_uart)))
		;
	while (!(Copy_upto("\"", buffer, wif_uart)))
		;
	while (!(Wait_for("OK\r\n", wif_uart)))
		;
	int len = strlen(buffer);
	buffer[len - 1] = '\0';
	sprintf(data, "IP ADDR: %s\n\n", buffer);
	Uart_sendstring(data, pcc_uart);

	Uart_sendstring("AT+CIPMUX=1\r\n", wif_uart);
	while (!(Wait_for("AT+CIPMUX=1\r\r\n\r\nOK\r\n", wif_uart)))
		;
	Uart_sendstring("CIPMUX---->OK\n\n", pcc_uart);

	Uart_sendstring("AT+CIPSERVER=1,80\r\n", wif_uart);
	while (!(Wait_for("OK\r\n", wif_uart)))
		;
	Uart_sendstring("CIPSERVER---->OK\n\n", pcc_uart);

	Uart_sendstring("Now Connect to the IP ADRESS\n\n", pcc_uart);

}

int Server_Send(char *str, int Link_ID) {
	int len = strlen(str);
	char data[80];
	sprintf(data, "AT+CIPSEND=%d,%d\r\n", Link_ID, len);
	Uart_sendstring(data, wif_uart);
	while (!(Wait_for(">", wif_uart)))
		;

	Uart_sendstring(str, wif_uart);
	while (!(Wait_for("SEND OK", wif_uart)))
		;
	sprintf(data, "AT+CIPCLOSE=5\r\n");
	Uart_sendstring(data, wif_uart);
	while (!(Wait_for("OK\r\n", wif_uart)))
		;
	return 1;
}
void delay_ms(uint32_t milliseconds) {
    uint32_t tickstart = HAL_GetTick();
    while ((HAL_GetTick() - tickstart) < milliseconds) {}
}
// Función para generar la respuesta HTML para el estado de los LED
char* generateResponse() {
	char *response = (char*) malloc(1024); // Ajusta el tamaño del búfer según tus necesidades

	// Construir la respuesta HTML según el estado actual de los LED
	sprintf(response, "%s", Basic_inclusion);

	if (led1_state == LED1_ON) {

		strcat(response,
				"<p>NIVEL 1</p><a class=\"button button-off\" href=\"/ledoff\">-1-</a>");

	} else {
		//delay_ms(2000);
		strcat(response,
				"<p>NIVEL 1</p><a class=\"button button-on\" href=\"/ledon\">-1-</a>");

	}

	if (led2_state == LED2_ON) {
		strcat(response,
				"<p>NIVEL 2</p><a class=\"button button-off\" href=\"/led2off\">-2-</a>");
	} else {
		strcat(response,
				"<p>NIVEL 2</p><a class=\"button button-on\" href=\"/led2on\">-2-</a>");
	}
	if (led3_state == LED3_ON) {
		strcat(response,
				"<p>NIVEL 3</p><a class=\"button button-off\" href=\"/led3off\">-3-</a>");
	} else {
		strcat(response,
				"<p>NIVEL 3</p><a class=\"button button-on\" href=\"/led3on\">-3-</a>");
	}
	if (led4_state == LED4_ON) {
		strcat(response,
				"<p>ABRIR PUERTAS</p><a class=\"button button-brown\" href=\"/led4off\"><|></a>");
	} else {
		strcat(response,
				"<p>ABRIR PUERTAS</p><a class=\"button button-brown\" href=\"/led4on\"><|></a>");
	}
	if (led5_state == LED5_ON) {
		strcat(response,
				"<p>ALARMA</p><a class=\"button button-red\" href=\"/led5off\">EMERG.</a>");
	} else {
		strcat(response,
				"<p>ALARMA</p><a class=\"button button-red\" href=\"/led5on\">EMERG.</a>");
	}
	strcat(response, Terminate);
	return response;
}

void Server_Handle(char *str, int Link_ID) {
	char *datatosend = NULL;

	if (strcmp(str, "/ledon") == 0) {
		led1_state = LED1_ON;
	} else if (strcmp(str, "/ledoff") == 0) {
		led1_state = LED1_OFF;
	} else if (strcmp(str, "/led2on") == 0) {
		led2_state = LED2_ON;
	} else if (strcmp(str, "/led2off") == 0) {
		led2_state = LED2_OFF;
	} else if (strcmp(str, "/led3on") == 0) {
		led3_state = LED3_ON;
	} else if (strcmp(str, "/led3off") == 0) {
		led3_state = LED3_OFF;
	} else if (strcmp(str, "/led4on") == 0) {
		led4_state = LED4_ON;
	} else if (strcmp(str, "/led4off") == 0) {
		led4_state = LED4_OFF;
	} else if (strcmp(str, "/led5on") == 0) {
		led5_state = LED5_ON;
	} else if (strcmp(str, "/led5off") == 0) {
		led5_state = LED5_OFF;
	}

	    datatosend = generateResponse();
	    Server_Send(datatosend, Link_ID);

	    // Libera la memoria asignada para la respuesta
	    free(datatosend);
}

void Server_Start(void) {

	char buftocopyinto[64] = { 0 };
	char Link_ID;
	while (!(Get_after("+IPD,", 1, &Link_ID, wif_uart)))
		;
	Link_ID -= 48;
	while (!(Copy_upto(" HTTP/1.1", buftocopyinto, wif_uart)))
		;
	if (Look_for("/ledon", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
		Server_Handle("/ledon", Link_ID);


	} else if (Look_for("/ledoff", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);
		Server_Handle("/ledoff", Link_ID);


	} else if (Look_for("/led2on", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		Server_Handle("/led2on", Link_ID);


	} else if (Look_for("/led2off", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 1);
		Server_Handle("/led2off", Link_ID);

	} else if (Look_for("/led3on", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		Server_Handle("/led3on", Link_ID);

	} else if (Look_for("/led3off", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, 1);
		Server_Handle("/led3off", Link_ID);

	} else if (Look_for("/led4on", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
		Server_Handle("/led4on", Link_ID);

	} else if (Look_for("/led4off", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, 1);
		Server_Handle("/led4off", Link_ID);

	} else if (Look_for("/led5on", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
		Server_Handle("/led5on", Link_ID);

	} else if (Look_for("/led5off", buftocopyinto) == 1) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
		Server_Handle("/led5off", Link_ID);

	}

	else if (Look_for("/favicon.ico", buftocopyinto) == 1)
		;

	else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
        Server_Handle("/ ", Link_ID);

	}

}
