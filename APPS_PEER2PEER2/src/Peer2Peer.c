/**
 * \file Peer2Peer.c
 *
 * \brief Peer2Peer application implementation
 *
 * Copyright (c) 2014-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 *
 */

/**
 * \mainpage
 * \section preface Preface
 * This is the reference manual for the LWMesh Peer2Peer Application
 * The LightWeight Mesh Peer2Peer  implements a wireless UART application.Two nodes are used in this application
 * These two nodes must be configured with addresses 0x0001 and 0x0000 respectively.
 * To test this application,open a terminal for both the nodes.On entering text in the terminal the data is transmitted from one 
 * node to another node(0x0001 to 0x0000 and vice-versa)
 */
/*- Includes ---------------------------------------------------------------*/
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "sys.h"
#if SAMD || SAMR21 || SAML21
#include "system.h"
#else
#include "led.h"
#include "sysclk.h"
#endif
#include "phy.h"
#include "nwk.h"
#include "sysTimer.h"
#include "sio2host.h"
#include "asf.h"
////I2C
#include "twi_megarf.h"
#include "ioport.h"

/*- Definitions ------------------------------------------------------------*/
#ifdef NWK_ENABLE_SECURITY
  #define APP_BUFFER_SIZE     (NWK_MAX_PAYLOAD_SIZE - NWK_SECURITY_MIC_SIZE)
#else
  #define APP_BUFFER_SIZE     NWK_MAX_PAYLOAD_SIZE
#endif

static uint8_t rx_data[APP_RX_BUF_SIZE];

////////////////////CONFIGURACIÓN I2C


#define TWI_MASTER           &TWBR

#define TWI_SPEED            125000

#define TWI_SLAVE_ADDR       0x96

#define SLAVE_MEM_ADDR      0x01

#define SLAVE_MEM_ADDR_LENGTH   TWI_SLAVE_ONE_BYTE_SIZE

#define DATA_LENGTH  sizeof(conf_data)

#define ALARM_TIMER_PERIOD 1000
#define TEMP_TIMER_PERIOD 5000
#define ACK_PENDING_TIMER_PERIOD 1000
#define SINK_NUM 2
#define MAX_SENT_COUNT 3

//#define __MUTE_MESSAGES__


const uint8_t conf_data[] = {
	0x60
};

typedef struct temp_ctx_t{
	uint8_t readData[2];
} TEMP_CTX_T;
TEMP_CTX_T temp_ctx;


// Definimos la variable de la temperatura sacada del sensor externo
uint8_t *pData_auto;
uint8_t *pData_forzada;
volatile uint8_t CTC_flag;

/*- Types ------------------------------------------------------------------*/
typedef enum AppState_t {
	APP_STATE_INITIAL,
	APP_STATE_IDLE,
} AppState_t;

typedef enum pendingType_t {
	NONE,
	FORCED,
	AUTO,
	ALARM
} pendingType_t;

typedef struct nodeSink_t {
	uint8_t sentCounter;
	pendingType_t pendingType;
} nodeSink_t;

/*- Prototypes -------------------------------------------------------------*/
static void appSendData(int dstAddr, uint8_t* data_to_send);
static void temp5s_Handler(SYS_Timer_t *timer);
void timerConfig (void);

void sensor_conf (void);
uint8_t* read_temperature (void); //Función para leer la temperatura del sensor externo
void twi_init (void);

/*- Variables --------------------------------------------------------------*/
static AppState_t appState = APP_STATE_INITIAL;
static SYS_Timer_t appTimer;
static SYS_Timer_t temp5s, alarmTimer, ackPendingTimer; //Estructura del temporizador de 5 segundos para enviar temperatura automáticamente
static bool flag_5s=false;
static NWK_DataReq_t appDataReq;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appTempBuffer[] = "AL_ON";
static uint8_t appUartBufferPtr = 0;
static uint8_t sio_rx_length;
int dstAddrPrev=6;

volatile uint8_t watchBuffer[APP_BUFFER_SIZE];

///////////////////////////////IVAN////////////////////////////////

static const uint8_t tempThreshold = 30;// Umbral de temperatura máxima
//uint16_t destinationAddress = DANI;		// Dirección de destino del paquete de datos
char buffer[APP_BUFFER_SIZE];			// Buffer que contiene el paquete de datos a enviar
bool alarmFlag = 0;						// Flags que controlan eventos secundarios (parpadeo del led en alarma)
bool keyboardEntry = 1;					// Selector para controlar si el paquete a enviar procede de la entrada por teclado o si es por software
uint8_t tempMonitorData[2] = { 0, 0 };	// Vector que almacena la lectura de temperatura de la monitorización
uint8_t tempReqData[2] = { 0, 0 };		// Vector que almacena la lectura de temperatura de la solicitud (y no sobreescribir la de monitorización)
nodeSink_t ackPendingVect[SINK_NUM];	// Vector de booleanos que almacena si falta algún sink por confirmar el envío



bool btn_prev;
bool btn_now;
/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{
	appDataReqBusy = false;
	(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(int dstAddr, uint8_t* data_to_send)
{
	if (appDataReqBusy || 0 == appUartBufferPtr) {
		return;
	}

	memcpy(appDataReqBuffer, appUartBuffer, appUartBufferPtr);

	//appDataReq.dstAddr = 1 - APP_ADDR;
	appDataReq.dstAddr = dstAddr;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	appDataReq.data = data_to_send;
	appDataReq.size = sizeof(data_to_send);
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appUartBufferPtr = 0;
	appDataReqBusy = true;
	LED_Toggle(LED0);
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
	appSendData(dstAddrPrev, appDataReqBuffer);
	(void)timer;
}

static void alarmTimerHandler(SYS_Timer_t *timer) {
	LED_Toggle(LED0);
	(void)timer;
}

static void temp5s_Handler(SYS_Timer_t *timer) {
	printf("\n\r========================== MONITORIZACION AUTOMATICA ==========================");
	pData_auto = read_temperature(); //Lectura de la temperatura del sensor externo
	// LA IMPRIMIMOS
	printf("\n\r\t>> La temperatura de mi nodo es: %i,%i", pData_auto[0], pData_auto[1]);
	if(pData_auto[0] >= tempThreshold) {
		for(uint8_t i = 0; i < SINK_NUM; i++) ackPendingVect[i].pendingType = ALARM;
		printf("\n\r\t ## UMBRAL DE TEMPERATURA SUPERADO: SOLICITUD DE ACTIVACIÓN DE ALARMA ACTIVADA ##");
		} else {
		for(uint8_t i = 0; i < SINK_NUM; i++) {
			ackPendingVect[i].pendingType = AUTO;
			ackPendingVect[i].sentCounter = 0;
		}
	}
	printf("\n\r");
	//appSendDataMulti();
	SYS_TimerStop(&ackPendingTimer);
	SYS_TimerStart(&ackPendingTimer);	// Inicia el timer que reenviará los paquetes a los sinks que no hayan enviado su ACK
}

/*************************************************************************//**
*****************************************************************************/

void timerConfig(void)
{
	/* Configuración del timer de alarma */
	alarmTimer.interval = ALARM_TIMER_PERIOD;
	alarmTimer.mode = SYS_TIMER_PERIODIC_MODE;
	alarmTimer.handler = alarmTimerHandler;
	/* Configuración del timer de monitoreo */
	temp5s.interval = TEMP_TIMER_PERIOD;
	temp5s.mode = SYS_TIMER_PERIODIC_MODE;
	temp5s.handler = temp5s_Handler;
	SYS_TimerStart(&temp5s); //Inicialización del timer de 5 segundos
	/* Configuración del timer de pendiente de ACK */
	//ackPendingTimer.interval = ACK_PENDING_TIMER_PERIOD;
	//ackPendingTimer.mode = SYS_TIMER_PERIODIC_MODE;
	//ackPendingTimer.handler = ackPendingTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/

/** FUNCIÓN QUE ACTIVA LA ALARMA **/
static void activateAlarm(void) {
	/* Comprueba si la alarma estaba ya activada, y si no lo está, la activa */
	if(!alarmFlag) {
		SYS_TimerStart(&alarmTimer);
		alarmFlag = 1;
	} else {
		printf("\n\r\t:: ERROR - La alarma ya estaba activada ::");
	}
}

/** FUNCIÓN QUE DESACTIVA LA ALARMA **/
static void desactivateAlarm(void) {
/* Comprueba si la alarma estaba ya desactivada, y si no lo está, la desactiva */
	if(alarmFlag) {
		SYS_TimerStop(&alarmTimer);
		alarmFlag = 0;
	} else {
		printf("\n\r\t:: ERROR - La alarma ya estaba desactivada ::");
	}
}

/*************************************************************************//**
*****************************************************************************/
static bool appDataInd(NWK_DataInd_t *ind)
{
	/* SE IMPRIME POR PANTALLA QUIEN HA ENVIADO EL PAQUETE */
	printf("\n\r\tRecibido de: %i\n\r>> ", ind->srcAddr);
	printf("Command: ");
	for (uint8_t i = 0; i < ind->size; i++) sio2host_putchar(ind->data[i]);
	printf("\n");

	/* SE VERIFICA EL PRIMER CARACTER DEL PAQUETE */
	switch (ind->data[0]) {
		
		// ACTIVACIÓN DE LA ALARMA
		case 'A':
		case 'a': {
			
			if(ind->data[2] == '1'){
				printf("Activación de la alarma");
				// Verifica si el emisor del paquete es un sink
				activateAlarm();	// En caso de ser un sink, activa la alarma
			}else if (ind->data[2] == '0') {
				printf("Apagado de la alarma");
				desactivateAlarm();	// En caso de ser un sink, desactiva la alarma
				LED_Off(LED0);
			}
			
		} break;
		case 'W':
		case 'w': {
			printf("Activación de la alarma");
			// Verifica si el emisor del paquete es un sink
			activateAlarm();	// En caso de ser un sink, activa la alarma
		} break;
				
		
		// SOLICITUD DE TEMPERATURA
		case 'R':
		case 'r': {
			printf("Petición de temperatura");
			
			pData_forzada = read_temperature(); //Lectura de la temperatura del sensor externo
			// LA IMPRIMIMOS
			printf("\n\r\t:: La temperatura del sensor externo de mi nodo es: %i,%i ::" ,pData_forzada[0],pData_forzada[1]);
			if(pData_forzada[0] >= tempThreshold) {
				for(uint8_t i = 0; i < SINK_NUM; i++) ackPendingVect[i].pendingType = ALARM;
				printf("\n\n\r\t ## UMBRAL DE TEMPERATURA SUPERADO: SOLICITUD DE ACTIVACIÓN DE ALARMA ACTIVADA ##");
				//appSendDataMulti();
				} else {
				ackPendingVect[ind->srcAddr - 1].pendingType = FORCED;
				ackPendingVect[ind->srcAddr - 1].sentCounter = 0;
				sprintf((char*)appDataReqBuffer, "X|%i,%i", pData_forzada[0], pData_forzada[1]);
				//sink_addr = ind->srcAddr;
				//appSendData();
				
				/** REINICIO DEL TIMER DE MONITOREO Y DE ACKs, Y ACTUALIZACIÓN DEL VECTOR DE ACKs **/
				SYS_TimerStop(&temp5s);
				SYS_TimerStart(&temp5s);
				SYS_TimerStop(&ackPendingTimer);
				SYS_TimerStart(&ackPendingTimer);
			}
		} break;
		
		//RECEPCIÓN DE UN ACK
		case 'K':
		case 'k':{
			printf("ACK recibido");
			// En caso de ser un sink, actualiza en el vector de ACKs que deja de estar pendiente de enviarlo
			ackPendingVect[ind->srcAddr - 1].pendingType = NONE;
			// Muestra los restantes
			//showRemaining();
		} break;
		
		default: {
			printf("ERROR: Comando desconocido\n\r>> ");
			for (uint8_t i = 0; i < ind->size; i++) sio2host_putchar(ind->data[i]);
		}
	}
	
	printf("\n\r");
	return true;
}

/*************************************************************************//**
*****************************************************************************/
static void appInit(void)
{
	NWK_SetAddr(APP_ADDR);
	NWK_SetPanId(APP_PANID);
	PHY_SetChannel(APP_CHANNEL);
#ifdef PHY_AT86RF212
	PHY_SetBand(APP_BAND);
	PHY_SetModulation(APP_MODULATION);
#endif
	PHY_SetRxState(true);

	NWK_OpenEndpoint(APP_ENDPOINT, appDataInd);

	appTimer.interval = APP_FLUSH_TIMER_INTERVAL;
	appTimer.mode = SYS_TIMER_INTERVAL_MODE;
	appTimer.handler = appTimerHandler;
}

/*************************************************************************//**
*****************************************************************************/
static void APP_TaskHandler(void)
{
	switch (appState) {
	case APP_STATE_INITIAL:
	{
		appInit();
		appState = APP_STATE_IDLE;
	}
	break;

	case APP_STATE_IDLE:
		break;

	default:
		break;
	}
	
	memcpy(watchBuffer,appTempBuffer,APP_BUFFER_SIZE);
	
	sio_rx_length = sio2host_rx(rx_data, APP_RX_BUF_SIZE);
	if (sio_rx_length) {
		for (uint16_t i = 0; i < sio_rx_length; i++) {
			sio2host_putchar(rx_data[i]);
			if (appUartBufferPtr == sizeof(appUartBuffer)) {
				appSendData(dstAddrPrev,appDataReqBuffer);
			}

			if (appUartBufferPtr < sizeof(appUartBuffer)) {
				appUartBuffer[appUartBufferPtr++] = rx_data[i];
			}
		}

		
		SYS_TimerStop(&appTimer);
		SYS_TimerStart(&appTimer);
		
	}
	btn_prev=btn_now;
	btn_now=ioport_get_pin_level(GPIO_PUSH_BUTTON_0);
	if(!btn_now && btn_prev){
		
		appUartBufferPtr = sizeof(appTempBuffer)-1;
		appSendData(dstAddrPrev, appTempBuffer);	
			
		SYS_TimerStop(&appTimer);
		SYS_TimerStart(&appTimer);
		}
}

/*************************************************************************//**
*****************************************************************************/
//////////////////////////////////TEMPERATURA/////////////////////////////////////

/********************************************************************************
*********************************************************************************/
void sensor_conf (void){
/* configures the TWI configuration packet*/
twi_package_t packet = {
.addr[0] = (uint8_t) SLAVE_MEM_ADDR,
.addr_length = (uint8_t)SLAVE_MEM_ADDR_LENGTH,
.chip = TWI_SLAVE_ADDR,
.buffer = (void *)conf_data,
.length = DATA_LENGTH
};
/* Perform a multi-byte write access */
while (twi_master_write(TWI_MASTER,&packet) != TWI_SUCCESS) {
}
/* waits for write completion*/
delay_ms(5);
}
/*********************************************************************************
**********************************************************************************/

// FUNCIÓN PARA LEER LA TEMPERATURA DEL SENSOR EXTERNO

uint8_t* read_temperature (void){
	uint8_t dato_temp[2] = {0, 0};

	/* configures the TWI read packet*/
		twi_package_t packet_received = {
		.addr[0] = 0x00,
		.addr_length = (uint8_t)SLAVE_MEM_ADDR_LENGTH,
		.chip = TWI_SLAVE_ADDR,
		.buffer = dato_temp,
		.length = 2,
	};
	/* Perform a multi-byte read access*/
	while (twi_master_read(TWI_MASTER,&packet_received) != TWI_SUCCESS) {}
	temp_ctx.readData[0] = dato_temp[0];
	temp_ctx.readData[1] = dato_temp[1];

	return temp_ctx.readData;
}
/*********************************************************************************
**********************************************************************************/

void twi_init (void){
	/* TWI master initialization options. */
	twi_master_options_t m_options = {
		.speed      = TWI_SPEED,
		.chip  = TWI_SLAVE_ADDR,
	};
	m_options.baud_reg = TWI_CLOCK_RATE(sysclk_get_cpu_hz(), m_options.speed);
	/* Enable the peripheral clock for TWI module */
	sysclk_enable_peripheral_clock(TWI_MASTER);
	/* Initialize the TWI master driver. */
	twi_master_init(TWI_MASTER,&m_options);
}



/*********************************************************************************
**********************************************************************************/

/*************************************************************************//**
*****************************************************************************/
int main(void)
{
	irq_initialize_vectors();
	sysclk_init();
	board_init();
	SYS_Init();
	sysclk_set_prescalers(CONFIG_SYSCLK_PSDIV);
	ioport_init();
	twi_init();
	sensor_conf();
	sio2host_init();
	cpu_irq_enable();
	LED_On(LED0);
	timerConfig();
	while (1) {
		SYS_TaskHandler();
		APP_TaskHandler();
	}
}
