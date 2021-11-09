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
#ifndef SENSOR_H_
#define SENSOR_H_

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
uint16_t sink_addr;

#define BROADCAST 0xFFFF

////////////////////CONFIGURACIÓN I2C


#define TWI_MASTER           &TWBR

#define TWI_SPEED            125000

#define TWI_SLAVE_ADDR       0x96

#define SLAVE_MEM_ADDR      0x01

#define SLAVE_MEM_ADDR_LENGTH   TWI_SLAVE_ONE_BYTE_SIZE

#define DATA_LENGTH  sizeof(conf_data)


#endif /* SENSOR_H_ */

#define ALARM_TIMER_PERIOD 1000
#define TEMP_TIMER_PERIOD 5000
#define ACK_PENDING_TIMER_PERIOD 1000
#define SINK_NUM 2
#define MAX_SENT_COUNT 2

//#define __MUTE_MESSAGES__
static int sink_addrs[]={5,9};

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
static void appSendData(void);
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
static NWK_DataReq_t appDataReq, appDataReq1, appDataReq2, appDataReq3;
static bool appDataReqBusy = false;
static uint8_t appDataReqBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBuffer[APP_BUFFER_SIZE];
static uint8_t appUartBufferPtr = 0;
static uint8_t sio_rx_length;

///////////////////////////////IVAN////////////////////////////////

static const uint8_t tempThreshold = 30;// Umbral de temperatura máxima
//uint16_t destinationAddress = DANI;		// Dirección de destino del paquete de datos
char buffer[APP_BUFFER_SIZE];			// Buffer que contiene el paquete de datos a enviar
bool alarmFlag = 0;						// Flags que controlan eventos secundarios (parpadeo del led en alarma)
bool keyboardEntry = 1;					// Selector para controlar si el paquete a enviar procede de la entrada por teclado o si es por software
uint8_t tempMonitorData[2] = { 0, 0 };	// Vector que almacena la lectura de temperatura de la monitorización
uint8_t tempReqData[2] = { 0, 0 };		// Vector que almacena la lectura de temperatura de la solicitud (y no sobreescribir la de monitorización)
nodeSink_t ackPendingVect[SINK_NUM];	// Vector de booleanos que almacena si falta algún sink por confirmar el envío

/////////////////////////////IVAN//////////////////////////////////

/*- Implementations --------------------------------------------------------*/

/*************************************************************************//**
*****************************************************************************/
static void appDataConf(NWK_DataReq_t *req)
{   
	printf("\n\r\tMensaje enviado a %i", req->dstAddr);
	appDataReqBusy = false;
	(void)req;
}

/*************************************************************************//**
*****************************************************************************/
static void appSendData(void)   //Función para enviar datos por inalámbrico
{
	/****** CONFIGURACIÓN DEL DESTINO ******/
	appDataReq.dstAddr = sink_addr;
	appDataReq.dstEndpoint = APP_ENDPOINT;
	appDataReq.srcEndpoint = APP_ENDPOINT;
	appDataReq.options = NWK_OPT_ENABLE_SECURITY;
	/****** CONFIGURACIÓN DE LOS DATOS ******/
	appDataReq.data = appDataReqBuffer;
	appDataReq.size=sizeof(appDataReqBuffer);
	/****** CONFIRMACIÓN Y ENVÍO ******/
	appDataReq.confirm = appDataConf;
	NWK_DataReq(&appDataReq);

	appUartBufferPtr = 0;
	appDataReqBusy = true;
}

static void appSendDataMulti(void)   //Función para enviar datos por inalámbrico
{
	for(int i = 0; i<SINK_NUM; i++){
		if(ackPendingVect[i].pendingType != NONE) {
			/****** CONFIGURACIÓN DEL DESTINO ******/
			appDataReq.dstAddr = sink_addrs[i];
			appDataReq.dstEndpoint = APP_ENDPOINT;
			appDataReq.srcEndpoint = APP_ENDPOINT;
			appDataReq.options = NWK_OPT_ENABLE_SECURITY;
			/****** CONFIGURACIÓN DE LOS DATOS ******/
			if(ackPendingVect[0].pendingType == FORCED) sprintf((char*)appDataReqBuffer, "X|%i,%i", pData_forzada[0], pData_forzada[1]);
			else if(ackPendingVect[0].pendingType == AUTO) sprintf((char*)appDataReqBuffer, "T|%i,%i", pData_auto[0], pData_auto[1]);
			else sprintf((char*)appDataReqBuffer, "F            ");
			appDataReq.data = appDataReqBuffer;
			appDataReq.size=sizeof(appDataReqBuffer);
			/****** CONFIRMACIÓN Y ENVÍO ******/
			appDataReq.confirm = appDataConf;
			NWK_DataReq(&appDataReq);
		}
	}
	appUartBufferPtr = 0;
	appDataReqBusy = true;
}

/*************************************************************************//**
*****************************************************************************/
static void appTimerHandler(SYS_Timer_t *timer)
{
	appSendData();
	(void)timer;
}
/*************************************************************************//**
*****************************************************************************/
static void alarmTimerHandler(SYS_Timer_t *timer) {
	LED_Toggle(LED0);
	(void)timer;
}

/*************************************************************************//**
*****************************************************************************/

/** INTERRUPCIÓN DEL TIMER ackPendingTimer **/
static void ackPendingTimerHandler(SYS_Timer_t *timer) {
	/* Esta interrupción se genera cada 500 milisegundos, para volver a enviar los paquetes a aquellos sinks que no hayan enviado el ACK */
	printf("\n\r");
	for(uint8_t i = 0; i < SINK_NUM; i++) {	// Recorre todos los sinks
		if(ackPendingVect[i].pendingType != NONE) {
			if(ackPendingVect[i].sentCounter >= MAX_SENT_COUNT) {
				ackPendingVect[i].pendingType = NONE;
				printf("\n\r  ## EL SINK %i NO HA ENVIADO SU ACK -- COMUNICACION FALLIDA ##", i + 1);			// Avisa al usuario que el sink tiene pendiente el envío del ACK
			} else {
				ackPendingVect[i].sentCounter++;
				#ifndef __MUTE_MESSAGES__
				printf("\n\r  :: El sink %i no ha enviado su ACK -- Se reenvia el paquete ::", i + 1);
				#endif
			}
		}
	}
	printf("\n\r");
	appSendDataMulti();
}

/*************************************************************************//**
*****************************************************************************/

/** FUNCIÓN QUE COLOCA INTERRUPCIONES A TODOS LOS SINKS **/
static void ackPendingVectRaise(pendingType_t isRequest) {
	// Recorre todo el vector de ACKs
	for (uint8_t i = 0; i < SINK_NUM; i++) {
		ackPendingVect[i].sentCounter = 0;			// Activa el flag de pendiente de recibir ACK
		ackPendingVect[i].pendingType = AUTO;	// Selecciona si es un ACK de un envío de monitorización o si es un ACK de una petición expresa
	}
}

/*************************************************************************//**
*****************************************************************************/

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
	appSendDataMulti();
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
	ackPendingTimer.interval = ACK_PENDING_TIMER_PERIOD;
	ackPendingTimer.mode = SYS_TIMER_PERIODIC_MODE;
	ackPendingTimer.handler = ackPendingTimerHandler;
}
	
/*************************************************************************//**
*****************************************************************************/

/** FUNCIÓN QUE CONTROLA SI LA DIRECCIÓN QUE HA ENVIADO EL PAQUETE ES UN SINK **/
static bool verifySink(uint16_t scrAddr) {
	/* Comprueba si está fuera del rango de direcciones de los sinks */
	bool result = false;
	for(int i=0;i<SINK_NUM;i++){
		if(scrAddr == sink_addrs[i]) result = true;
	}
	return result;
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
static bool appDataInd(NWK_DataInd_t *ind) //en esta función se procesan los datos recibidos
{   
	/* SE IMPRIME POR PANTALLA QUIEN HA ENVIADO EL PAQUETE */
	printf("\n\r\tRecibido de: %i", ind->srcAddr);
	if(ind->dstAddr == BROADCAST) printf(" (via BROADCAST)");
	printf("\n\r>> ");

    /* SE VERIFICA EL PRIMER CARACTER DEL PAQUETE */
	switch (ind->data[0]) {
			
		// ACTIVACIÓN DE LA ALARMA
		case 'W':
		case 'w': {
			printf("Activación de la alarma");
			// Verifica si el emisor del paquete es un sink
			if(verifySink(ind->srcAddr) == false) printf("\n\r\t:: ERROR - Un nodo ha intentado activar la alarma ::");
			else activateAlarm();	// En caso de ser un sink, activa la alarma
		} break;	
		
		case 'A':
		case 'a': {
			if(ind->data[2]=='0'){
				printf("Apagado de la alarma");
				// Verifica si el emisor del paquete es un sink
				if(verifySink(ind->srcAddr) == false) printf("\n\r\t:: ERROR - Un nodo ha intentado desactivar la alarma ::");
				else {
					desactivateAlarm();	// En caso de ser un sink, desactiva la alarma
					LED_Off(LED0);
				}
			}else{
				printf("Activación de la alarma");
				// Verifica si el emisor del paquete es un sink
				if(verifySink(ind->srcAddr) == false) printf("\n\r\t:: ERROR - Un nodo ha intentado activar la alarma ::");
				else activateAlarm();	// En caso de ser un sink, activa la alarma
			}
		} break;
				
		// SOLICITUD DE TEMPERATURA
		case 'R':
		case 'r': {
			printf("Petición de temperatura");
			// Verifica si el emisor del paquete es un sink
			if(verifySink(ind->srcAddr) == false) printf("\n\r\t:: ERROR - Peticion de temperatura por parte de un nodo ::");
			else {
				pData_forzada = read_temperature(); //Lectura de la temperatura del sensor externo
				// LA IMPRIMIMOS
				printf("\n\r\t:: La temperatura del sensor externo de mi nodo es: %i,%i ::" ,pData_forzada[0],pData_forzada[1]);
				if(pData_forzada[0] >= tempThreshold) {
					for(uint8_t i = 0; i < SINK_NUM; i++) ackPendingVect[i].pendingType = ALARM;
					printf("\n\n\r\t ## UMBRAL DE TEMPERATURA SUPERADO: SOLICITUD DE ACTIVACIÓN DE ALARMA ACTIVADA ##");
					appSendDataMulti();
				} else {
					ackPendingVect[ind->srcAddr - 1].pendingType = FORCED;
					ackPendingVect[ind->srcAddr - 1].sentCounter = 0;
					sprintf((char*)appDataReqBuffer, "X|%i,%i", pData_forzada[0], pData_forzada[1]);
					sink_addr = ind->srcAddr;
					appSendData();
				}
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
			// Verifica si el emisor del paquete es un sink
			if(verifySink(ind->srcAddr) == false) printf("\n\r\t:: ERROR - ACK procedente de un nodo ::");
			else {
				// En caso de ser un sink, actualiza en el vector de ACKs que deja de estar pendiente de enviarlo
				ackPendingVect[ind->srcAddr - 1].pendingType = NONE;
				// Muestra los restantes
				//showRemaining();
			}
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
static void APP_TaskHandler(void) //es como la máquina de estados del sistema
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
	sio_rx_length = sio2host_rx(rx_data, APP_RX_BUF_SIZE);
	if (sio_rx_length) {
		for (uint16_t i = 0; i < sio_rx_length; i++) {
			sio2host_putchar(rx_data[i]);
			if (appUartBufferPtr == sizeof(appUartBuffer)) {
				appSendData();
			}

			if (appUartBufferPtr < sizeof(appUartBuffer)) {
				appUartBuffer[appUartBufferPtr++] = rx_data[i];
			}
		}

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
	while (twi_master_read(TWI_MASTER,&packet_received) != TWI_SUCCESS) {
	}
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
int main(void)
{
	irq_initialize_vectors();
	#if SAMD || SAMR21 || SAML21
	system_init();
	delay_init();
	#else
	sysclk_init();
	board_init();
	#endif
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

