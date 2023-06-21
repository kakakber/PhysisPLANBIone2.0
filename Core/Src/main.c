/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LORALIB/LoRa.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LORAPLEN 30

#define ADC16BIT 65535.0
#define ADCVDD 3.333

#define ADC1BUFSIZE 500
#define ADC1Channels 5

#define ADC2BUFSIZE 200
#define ADC2Channels 4

#define ADC3BUFSIZE 200
#define ADC3Channels 4

#define VOFF_ADC5V 0.0709
#define B_ADC5V 1.0146

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

FDCAN_HandleTypeDef hfdcan2;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart9;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
static void MX_UART7_Init(void);
static void MX_SPI2_Init(void);
static void MX_UART8_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_UART9_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//CAN
FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;

uint8_t TxDataCAN[8];
uint8_t TxData2CAN[8];
uint8_t RxDataCAN[8];
uint8_t RxData2CAN[8];
uint8_t RxData3CAN[8];
uint8_t RxData4CAN[8];
uint8_t RxData5CAN[8];
uint8_t RxData6CAN[8];
uint32_t TxMailbox;
int i = 0;

uint8_t count;

void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan1) {

	if (i == 0) {
		HAL_FDCAN_GetRxMessage(hcan1, FDCAN_RX_FIFO0, &RxHeader, RxDataCAN);
		i = 1;
	} else if (i == 1) {
		HAL_FDCAN_GetRxMessage(hcan1, FDCAN_RX_FIFO0, &RxHeader, RxData2CAN);
		i = 0;
	} else if (i == 2) {
		HAL_FDCAN_GetRxMessage(hcan1, FDCAN_RX_FIFO0, &RxHeader, RxData3CAN);
		i = 3;
	} else if (i == 3) {
		HAL_FDCAN_GetRxMessage(hcan1, FDCAN_RX_FIFO0, &RxHeader, RxData4CAN);
		i = 4;
	} else if (i == 4) {
		HAL_FDCAN_GetRxMessage(hcan1, FDCAN_RX_FIFO0, &RxHeader, RxData5CAN);
		i = 5;
	} else if (i == 5) {
		HAL_FDCAN_GetRxMessage(hcan1, FDCAN_RX_FIFO0, &RxHeader, RxData6CAN);
		i = 0;
	}

}

//DATA STRUCTURES
//We get this data through UART requests
typedef struct BMSStruct {
	//0x90 Data request
	int voltage; //byte 0 and 1 V*10
	int current; //byte 4 and 5 I*10 (remember 3000 offset)
	int soc; //byte 6,7 SOC*10
	//0x94: we get status info, in particular the number of cells (byte 0)
	//and the number of temp sensors (byte 1), which tell us how many bytes to take the temp from (best to do avg)
	uint8_t noOfCells;
	uint8_t noOfTempSensors;
	//0x96
	int temperature; //byte 1->7: from multiple sensors

} BMSStruct;

typedef struct DCDCStruct {
	unsigned short outVoltage; //*10
	unsigned short outCurrent; //*10
	unsigned short outPower;

	unsigned short inputTension;
	signed short dissipatoreTemp;
} DCDCStruct;

typedef struct PSStruct {
	float ps1;
	float ps2;
	float ps3;
} PSStruct;

typedef struct ControllerStruct {
	//for now we can only detect temperature
	int temp;
} ControllerStruct;

typedef struct EngineInfoStruct {
	int BUSVoltage;
	int BUSCurrent;
	int engineSpeed;
	int alarm; //?????? è questo?
} EngineInfoStruct;

typedef struct vitalSystemInfoStruct {
	EngineInfoStruct engineInfo;
	BMSStruct BMSData;
	DCDCStruct DCDCData;
	float h2SensorValue;
	PSStruct psValues;
	ControllerStruct controllerData;
} vitalSystemInfoStruct;

vitalSystemInfoStruct vitalSystemInfo;

//LORA

void assignUShort(uint8_t *out_packet, int *ind, unsigned short ass) {
	union {
		unsigned short a;
		uint8_t bytes[2];
	} thing;
	thing.a = ass;
	for (int i = 0; i < 2; i++) {
		out_packet[i + *ind] = thing.bytes[i];
	}
	(*ind) += 2;
}

void createLoRaPacket(uint8_t *out_packet) {
	//turns the struct into composed uint array. Total 30 bytes
	int ind = 0;
	assignUShort(out_packet, &ind, vitalSystemInfo.DCDCData.outVoltage);
	assignUShort(out_packet, &ind, vitalSystemInfo.DCDCData.outCurrent);
	assignUShort(out_packet, &ind, vitalSystemInfo.DCDCData.outPower);
	assignUShort(out_packet, &ind, vitalSystemInfo.DCDCData.inputTension);
	assignUShort(out_packet, &ind, vitalSystemInfo.DCDCData.dissipatoreTemp);

	//TODO: SAVE THE RAW, DONT DO CALCULATIONS
	assignUShort(out_packet, &ind,
			(unsigned short) ((vitalSystemInfo.psValues.ps1 * ADC16BIT) / ADCVDD));
	assignUShort(out_packet, &ind,
			(unsigned short) ((vitalSystemInfo.psValues.ps2 * ADC16BIT) / ADCVDD));
	assignUShort(out_packet, &ind,
			(unsigned short) ((vitalSystemInfo.psValues.ps3 * ADC16BIT) / ADCVDD));

	assignUShort(out_packet, &ind, vitalSystemInfo.controllerData.temp);
	assignUShort(out_packet, &ind, vitalSystemInfo.controllerData.temp); //TODO: put speed here

	assignUShort(out_packet, &ind,
			(unsigned short) ((vitalSystemInfo.h2SensorValue * ADC16BIT)
					/ ADCVDD));

	assignUShort(out_packet, &ind, vitalSystemInfo.BMSData.voltage);
	assignUShort(out_packet, &ind, vitalSystemInfo.BMSData.current);
	assignUShort(out_packet, &ind, vitalSystemInfo.BMSData.soc);
	assignUShort(out_packet, &ind, vitalSystemInfo.BMSData.temperature);
}

//UART

static unsigned short bytesToUShort(uint8_t b1, uint8_t b2) {
	unsigned short wd = ((unsigned short) b1 << 8) | b2;
	return wd;
}

static signed short bytesToSShort(uint8_t b1, uint8_t b2) {
	signed short wd = ((signed short) b1 << 8) | b2;
	return wd;
}

static int bytesToInt(uint8_t b1, uint8_t b2) {
	int wd = ((int) b1 << 8) | b2;
	return wd;
}

void decodeDCDCReceivedData(uint8_t *rdata) {
	//i get 10 bytes
	vitalSystemInfo.DCDCData.outVoltage = bytesToUShort(rdata[2], rdata[1]);
	vitalSystemInfo.DCDCData.outCurrent = bytesToUShort(rdata[4], rdata[3]);
	vitalSystemInfo.DCDCData.outPower = bytesToUShort(rdata[6], rdata[5]);
	vitalSystemInfo.DCDCData.inputTension = bytesToUShort(rdata[8], rdata[7]);
	vitalSystemInfo.DCDCData.dissipatoreTemp = bytesToSShort(rdata[10],
			rdata[9]);

}

uint8_t receivedDCDCUartData[50];
uint8_t receivedControllerUARTData[8];
uint8_t receivedBMSUartData[13];

//BMS
#define XFER_BUFFER_LENGTH 13
#define BMS_CURRENT_OFFSET 30000

void readBMSData() {
	//HAL_UART_Receive(&huart5, receivedBMSUartData, 13, 1000);
	if (receivedBMSUartData[0] == 0xA5) {
		//right data
		switch (receivedBMSUartData[2]) {
		case 0x90:
			//requested v, i, soc
			vitalSystemInfo.BMSData.voltage = bytesToInt(receivedBMSUartData[4],
					receivedBMSUartData[5]);
			vitalSystemInfo.BMSData.current = bytesToInt(receivedBMSUartData[8],
					receivedBMSUartData[9]) - BMS_CURRENT_OFFSET;
			vitalSystemInfo.BMSData.soc = bytesToInt(receivedBMSUartData[10],
					receivedBMSUartData[11]);
			break;
		case 0x96:
			//requested temp
			//TODO: Maybe do avg
			vitalSystemInfo.BMSData.temperature = receivedBMSUartData[4];
			break;
		case 0x94:
			//requested n of cells, nb of sensors
			vitalSystemInfo.BMSData.noOfCells = receivedBMSUartData[4];
			vitalSystemInfo.BMSData.noOfTempSensors = receivedBMSUartData[5];
			break;
		}
	}
}

int BMSDataDelay;
bool tempDataReq;

void requestBMSData() {
	//Requesting

	if (tempDataReq == false) {
		//first here

		uint8_t txData[13];
		txData[0] = 0xA5;
		txData[1] = 0x40;
		txData[2] = 0x90;
		txData[3] = 0x08;

		txData[4] = 0x00;
		txData[5] = 0x00;
		txData[6] = 0x00;
		txData[7] = 0x00;
		txData[8] = 0x00;
		txData[9] = 0x00;
		txData[10] = 0x00;
		txData[11] = 0x00;

		txData[12] = 0x7d;			//checksum
		HAL_UART_Transmit(&huart5, txData, XFER_BUFFER_LENGTH, 30);
		BMSDataDelay = HAL_GetTick();
		tempDataReq == true;
	}

	if ((HAL_GetTick() - BMSDataDelay >= 30) && (tempDataReq == true)) {
		uint8_t txData[13];
		txData[0] = 0xA5;
		txData[1] = 0x40;
		txData[2] = 0x96;
		txData[3] = 0x08;

		txData[4] = 0x00;
		txData[5] = 0x00;
		txData[6] = 0x00;
		txData[7] = 0x00;
		txData[8] = 0x00;
		txData[9] = 0x00;
		txData[10] = 0x00;
		txData[11] = 0x00;

		txData[12] = 0x83;			//checksum
		HAL_UART_Transmit(&huart5, txData, XFER_BUFFER_LENGTH, 30);
		tempDataReq = false;
	}

}

//SD CARD

FATFS FatFs; //Fatfs handle
FIL fil; //File handle
FRESULT fres; //Result after operations

void mount_sd_card() {
	fres = f_mount(&FatFs, "", 1); //1=mount now
	if (fres != FR_OK) {
		//printf("No SD Card found : (%i)\r\n", fres);
		return;
	} else {
		int a = 0;
	}
}

int setFilename = 0;
char filename[20];

int BUSVoltage;
int BUSCurrent;
int engineSpeed;
int alarm; //?????? è questo?

void write_to_file() {

	do {
		uint32_t currentTime = HAL_GetTick();
		char dataString[200];
		char headerString[327];
		sprintf(dataString,
				"%d, %d, %d, %d, %d, %d, %d, %d, %d, %f, %f, %f, %f, %d, %d, %d, %d, %d, %lu\n",
				vitalSystemInfo.BMSData.current,
				vitalSystemInfo.BMSData.voltage,
				vitalSystemInfo.BMSData.temperature,
				vitalSystemInfo.BMSData.soc,
				vitalSystemInfo.DCDCData.outVoltage,
				vitalSystemInfo.DCDCData.outCurrent,
				vitalSystemInfo.DCDCData.outPower,
				vitalSystemInfo.DCDCData.inputTension,
				vitalSystemInfo.DCDCData.dissipatoreTemp,
				vitalSystemInfo.h2SensorValue, vitalSystemInfo.psValues.ps1,
				vitalSystemInfo.psValues.ps2, vitalSystemInfo.psValues.ps3,
				vitalSystemInfo.controllerData.temp,
				vitalSystemInfo.engineInfo.BUSVoltage,
				vitalSystemInfo.engineInfo.BUSCurrent,
				vitalSystemInfo.engineInfo.engineSpeed,
				vitalSystemInfo.engineInfo.alarm, currentTime);
		bool newFileTitle = false;
		if (!setFilename) {
			//begin
			newFileTitle = true;
			snprintf(filename, sizeof(filename), "%ld.txt", HAL_GetTick());
			setFilename = 1;
		}

		//Open the file
		fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_APPEND);
		if (fres != FR_OK) {
			//printf("File creation/open Error : (%i)\r\n", fres);
			//HAL_Delay(700);
			mount_sd_card();
			break;
		}

		if (newFileTitle) {
			newFileTitle = false;
			sprintf(headerString,
					"BMSData.current, BMSData.voltage, BMSData.temperature, BMSData.soc, DCDCData.outVoltage, DCDCData.outCurrent, DCDCData.outPower, DCDCData.inputTension, DCDCData.dissipatoreTemp, h2SensorValue, ps1, ps2, ps3, controllerData.temp, engineInfo.BUSVoltage, engineInfo.BUSCurrent, engineInfo.engineSpeed, engineInfo.alarm, timestamp\n");
			f_puts(headerString, &fil);
		}

		f_puts(dataString, &fil);

		f_close(&fil);

	} while (0);
}

//TODO: What is it
#define H2TANKSTEMP 27

int remainingH2KW() {
	//gives kW remaining fC
	//TODO: Have exact formula
	int ps1Rem = vitalSystemInfo.psValues.ps1;
	int ps2Rem = vitalSystemInfo.psValues.ps2;
	int ps3Rem = vitalSystemInfo.psValues.ps3;
	return (ps1Rem + ps2Rem + ps3Rem);
}

/*
 void calculateOptimalPower(){
 //get voltage and current from BMS and DCDC (output)
 //From that calculate the remaining time from that power
 int batteryPower = vitalSystemInfo.BMSData.current * vitalSystemInfo.BMSData.voltage;
 int FCPower = vitalSystemInfo.DCDCData.outPower;
 int totalPowerNOW = batteryPower + FCPower;

 int remainingH2KW = remainingH2();
 //TODO: How to calculate this
 int remainingBatteryW = vitalSystemInfo.BMSData.soc;

 //int totalRemainingW = remainingH2W + remainingBatteryW;

 float hoursRemaining = totalRemainingW/totalPowerNOW;
 }*/

//UART callbacks
/*UART received data handling*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart == &huart7) {
		//received form dcdc uart
		if (receivedDCDCUartData[0] == 0x18) {
			//we received the data we wanted
			decodeDCDCReceivedData(receivedDCDCUartData);
		}
		HAL_UARTEx_ReceiveToIdle_IT(&huart7, receivedDCDCUartData, 20);
		//HAL_UART_Receive_IT(&huart7, receivedDCDCUartData, 50);
	} else if (huart == &huart4) {
		//Data from the controller, we always get 8 bytes, the last one is the temp*2
		vitalSystemInfo.controllerData.temp = receivedControllerUARTData[7] / 2;
		HAL_UART_Receive_IT(&huart4, receivedControllerUARTData, 12);
	} else if (huart == &huart5) {
		//Data from BMS
		readBMSData();
		HAL_UARTEx_ReceiveToIdle_IT(&huart5, receivedBMSUartData, 13);
	} else if (huart == &huart8) {
		//Data from GPS
	}
}

/*ADC Data handling*/

uint16_t adc1Buff[500]; //100 reads per adc channel, 16 bits prec
uint16_t adc2Buff[200]; //50 reads per adc channel, 16 bits prec
uint16_t adc3Buff[200]; //50 reads per adc channel, 16 bits prec

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc == &hadc1) {
		//adc 1 gets ps1, ps2,ps3, psfc1, h2
		float ps1sum = 0;
		float ps2sum = 0;
		float ps3sum = 0;
		float psfcsum = 0;
		float h2sum = 0;

		for (int i = 0; i < ADC1BUFSIZE; i += ADC1Channels) {
			ps1sum += adc1Buff[i + 0];
			ps2sum += adc1Buff[i + 2];
			ps3sum += adc1Buff[i + 1];
			h2sum += adc1Buff[i + 3];
			psfcsum += adc1Buff[i + 4];
		}
		//we got the values, the adc is 16 bits
		//computes ((sum /100)/65535)*3.3 50 microvolts precision!

		vitalSystemInfo.psValues.ps1 = ((((ps1sum
				/ ((ADC1BUFSIZE / ADC1Channels))) / ADC16BIT) * 5.0) - 0.0709)
				/ 1.0146;
		vitalSystemInfo.psValues.ps2 =
				((ps2sum / ((ADC1BUFSIZE / ADC1Channels))) / ADC16BIT) * 5.0;
		vitalSystemInfo.psValues.ps3 =
				((ps3sum / ((ADC1BUFSIZE / ADC1Channels))) / ADC16BIT) * 5.0;
		vitalSystemInfo.h2SensorValue =
				((h2sum / ((ADC1BUFSIZE / ADC1Channels))) / ADC16BIT) * 5.0;

	}

}

enum NUCLEOSTate {
	nucleoInit,
	nominal,
	h2Emergency,
	PBEmergency,
	killcordEmergency,
	waitingStep
};
enum NUCLEOSTate status;

bool fansOn = false;
long int fansOnCounter;

void emergencyGeneralActions() {
	//start the fans and open the relay
	fansOn = true;
	fansOnCounter = HAL_GetTick();
	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
}

int ff = 0;
int dd = 0;
void generateFakeH2Eme() {
	int ff = 0;
	if ((HAL_GetTick() > dd + 10000) && ff == 0) {
		ff = 1;
		status = h2Emergency;
		emergencyGeneralActions();
	}
}

//3.3 is 1000 ppm.
#define H2_ALARM_TRESHOLD 4.5 //900 ppm
#define H2_ALARM_RESET_VALUE 2.0 //400 ppm

void checkAlarms() {

	if (vitalSystemInfo.h2SensorValue > H2_ALARM_TRESHOLD) {
		//h2 over the limit
		HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
		emergencyGeneralActions();
		status = h2Emergency;
	}


	//TODO: check killcord activation (probably to manage in callback)

}

void backToNominalState() {
	//switch power on
	HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	//switch controller on, switching for 3 secs (relay2)
	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_RESET);
	HAL_Delay(3000);
	HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
	status = nominal;
}



void writeDCDCData() {
	char dataString[60];
	int v = 70;
	int c = 20;
	unsigned int p = 1800;
	sprintf(dataString, "%d %d %u\n", v, c, p);
	size_t dataLength = strlen(dataString); // Calculate the length of the data string

	HAL_UART_Transmit(&huart9, (uint8_t*) dataString, dataLength, 100);
	HAL_Delay(10);
}

void readDCDCDataPolling() {
	HAL_UART_Receive(&huart7, receivedDCDCUartData, 11, 5);
	if (receivedDCDCUartData[0] == 0x18) {
		decodeDCDCReceivedData(receivedDCDCUartData);
	}

}

#define EMERGENCY_FANS_TIME_MS 60000 //60 secs delay for fans after emergency

void manageFans() {
	if (fansOn == true) {
		if (HAL_GetTick() > fansOnCounter + EMERGENCY_FANS_TIME_MS) {
			//stop fans
			fansOn = false;
			HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_SET);
		}
		HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_RESET);
	} else {
		HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_SET);
	}
}

int cntLOR;
bool loraActive;




void manageLoRaTransmission(){
	if (loraActive == true) {
		if (loRaIsTrasmitting() == FALSE) {
			uint8_t buff[30];
			createLoRaPacket(buff);
			loRaClearIRQReceive();
			loRaSendPacket(TRUE, buff, sizeof(buff));
			loRaSetSpreadingFactor(cntLOR);
		}
	}
}


/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* Configure the peripherals common clocks */
	PeriphCommonClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_SPI4_Init();
	MX_UART4_Init();
	MX_UART5_Init();
	MX_UART7_Init();
	MX_SPI2_Init();
	MX_UART8_Init();
	MX_FDCAN2_Init();
	MX_UART9_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */

	HAL_Delay(300);

	//Initialize UART IT

	//HAL_UART_Receive_IT(&huart4, receivedControllerUARTData, 4);
	//HAL_UART_Receive_IT(&huart7, receivedDCDCUartData, 3);
	//HAL_UART_Receive_DMA (&huart4, receivedDCDCUartData, 3);

	status = nucleoInit;

	loraActive = false;

	if (loRaInit(&hspi2, LORA_CS_GPIO_Port, LORA_CS_Pin, LORA_DIO0_GPIO_Port,
	LORA_DIO0_Pin, LORA_RST_GPIO_Port, LORA_RST_Pin)) {
		//LoRaSuccess
		loRaFullyTransmitMode();
		loRaEnableCRC();
		loRaSetSyncWord(0xF4);
		loRaSetTxPower(21, TRUE); //boost the signal to TRUW
		loRaSetSpreadingFactor(7);
		loRaSetSignalBandwidth(250E3);
		loRaSetCodingRate(9);
		//start sending packet
		uint8_t buff = 0x34;
		loRaSendPacket(TRUE, &buff, sizeof(buff));
		loraActive = true;
	} else {

		//LoRaFail
		//stays in loop while it cannot initialize the lora module
		//TODO: REMOVE THIS BLOCKING BIT!
		do {
			HAL_Delay(1000);
		} while (!loRaInit(&hspi2, LORA_CS_GPIO_Port, LORA_CS_Pin,
		LORA_DIO0_GPIO_Port, LORA_DIO0_Pin, LORA_RST_GPIO_Port,
		LORA_RST_Pin));

	}

	//Initialize ADC DMA
	HAL_ADC_Start_DMA(&hadc1, adc1Buff, sizeof(adc1Buff) / sizeof(adc1Buff[0]));
	//HAL_ADC_Start_DMA(&hadc2, adc2Buff, sizeof(adc2Buff)/sizeof(adc2Buff[0]));
	//HAL_ADC_Start_DMA(&hadc3, adc3Buff, sizeof(adc3Buff)/sizeof(adc3Buff[0]));

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	cntLOR = 7;

	dd = HAL_GetTick();
	fansOn = false;
	tempDataReq = false;

	mount_sd_card();

	HAL_UARTEx_ReceiveToIdle_IT(&huart5, receivedBMSUartData, 13);

	HAL_FDCAN_Start(&hfdcan2);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	/*
	 TxHeader.DLC=8;
	 TxHeader.ExtId=0;
	 TxHeader.IDE=CAN_ID_STD;
	 TxHeader.RTR=CAN_RTR_DATA;
	 TxHeader.StdId= 0x103;
	 TxHeader.TransmitGlobalTime= DISABLE;
	 */

	while (1) {
		//readDCDCDataPolling();
		//writeDCDCData();

		switch (status) {
		case nucleoInit:
			// Code for nucleoInit state
			HAL_GPIO_WritePin(RELAY1_GPIO_Port, RELAY1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RELAY2_GPIO_Port, RELAY2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_SET);
			//generateFakeH2Eme();
			if (HAL_GPIO_ReadPin(SYSTEM_BUTTON_GPIO_Port, SYSTEM_BUTTON_Pin)
					== GPIO_PIN_RESET) {
				backToNominalState();
			}
			break;
		case nominal:
			// Code for nominal state
			//int g = 0;
			readDCDCDataPolling();
			writeDCDCData();
			//calculateOptimalPower();
			//HAL_GPIO_WritePin(RELAY3_GPIO_Port, RELAY3_Pin, GPIO_PIN_SET);
			break;
		case h2Emergency:
			// Code for h2Emergency state
			//keep checking h2 values

			if (vitalSystemInfo.h2SensorValue < H2_ALARM_RESET_VALUE) {
				//can restart
				status = waitingStep;
			}
			break;
		case killcordEmergency:
			//TODO: Code for killcordEmergency state
			status = killcordEmergency;
			break;
		case waitingStep:
			if (HAL_GPIO_ReadPin(SYSTEM_BUTTON_GPIO_Port, SYSTEM_BUTTON_Pin)
					== GPIO_PIN_RESET && !fansOn) {
				backToNominalState();
			}
			break;
		default:
			// Code for handling unknown state, should never happen
			backToNominalState();
			break;
		}

		//check for alarms
		checkAlarms();

		//activate fans if needed
		manageFans();


		//ask bms for data
		requestBMSData();
		//readBMSDataPolling();

		//write data to sd card log
		write_to_file();

		//send lora packet
		manageLoRaTransmission();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/*AXI clock gating */
	RCC->CKGAENR = 0xFFFFFFFF;

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = 64;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Peripherals Common Clock Configuration
 * @retval None
 */
void PeriphCommonClock_Config(void) {
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = { 0 };

	/** Initializes the peripherals clock
	 */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
	PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV32;
	hadc1.Init.Resolution = ADC_RESOLUTION_16B;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	sConfig.OffsetSignedSaturation = DISABLE;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief FDCAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_FDCAN2_Init(void) {

	/* USER CODE BEGIN FDCAN2_Init 0 */

	/* USER CODE END FDCAN2_Init 0 */

	/* USER CODE BEGIN FDCAN2_Init 1 */

	/* USER CODE END FDCAN2_Init 1 */
	hfdcan2.Instance = FDCAN2;
	hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
	hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
	hfdcan2.Init.AutoRetransmission = DISABLE;
	hfdcan2.Init.TransmitPause = DISABLE;
	hfdcan2.Init.ProtocolException = DISABLE;
	hfdcan2.Init.NominalPrescaler = 16;
	hfdcan2.Init.NominalSyncJumpWidth = 1;
	hfdcan2.Init.NominalTimeSeg1 = 2;
	hfdcan2.Init.NominalTimeSeg2 = 2;
	hfdcan2.Init.DataPrescaler = 18;
	hfdcan2.Init.DataSyncJumpWidth = 1;
	hfdcan2.Init.DataTimeSeg1 = 1;
	hfdcan2.Init.DataTimeSeg2 = 1;
	hfdcan2.Init.MessageRAMOffset = 0;
	hfdcan2.Init.StdFiltersNbr = 0;
	hfdcan2.Init.ExtFiltersNbr = 0;
	hfdcan2.Init.RxFifo0ElmtsNbr = 0;
	hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxFifo1ElmtsNbr = 0;
	hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.RxBuffersNbr = 0;
	hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
	hfdcan2.Init.TxEventsNbr = 0;
	hfdcan2.Init.TxBuffersNbr = 0;
	hfdcan2.Init.TxFifoQueueElmtsNbr = 0;
	hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
	hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
	if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN FDCAN2_Init 2 */

	/* USER CODE END FDCAN2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 0x0;
	hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi2.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi2.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief SPI4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI4_Init(void) {

	/* USER CODE BEGIN SPI4_Init 0 */

	/* USER CODE END SPI4_Init 0 */

	/* USER CODE BEGIN SPI4_Init 1 */

	/* USER CODE END SPI4_Init 1 */
	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.Mode = SPI_MODE_MASTER;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_SOFT;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 0x0;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi4.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi4.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI4_Init 2 */

	/* USER CODE END SPI4_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief UART5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART5_Init(void) {

	/* USER CODE BEGIN UART5_Init 0 */

	/* USER CODE END UART5_Init 0 */

	/* USER CODE BEGIN UART5_Init 1 */

	/* USER CODE END UART5_Init 1 */
	huart5.Instance = UART5;
	huart5.Init.BaudRate = 9600;
	huart5.Init.WordLength = UART_WORDLENGTH_8B;
	huart5.Init.StopBits = UART_STOPBITS_1;
	huart5.Init.Parity = UART_PARITY_NONE;
	huart5.Init.Mode = UART_MODE_TX_RX;
	huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart5.Init.OverSampling = UART_OVERSAMPLING_16;
	huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart5) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART5_Init 2 */

	/* USER CODE END UART5_Init 2 */

}

/**
 * @brief UART7 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART7_Init(void) {

	/* USER CODE BEGIN UART7_Init 0 */

	/* USER CODE END UART7_Init 0 */

	/* USER CODE BEGIN UART7_Init 1 */

	/* USER CODE END UART7_Init 1 */
	huart7.Instance = UART7;
	huart7.Init.BaudRate = 115200;
	huart7.Init.WordLength = UART_WORDLENGTH_8B;
	huart7.Init.StopBits = UART_STOPBITS_1;
	huart7.Init.Parity = UART_PARITY_NONE;
	huart7.Init.Mode = UART_MODE_RX;
	huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart7.Init.OverSampling = UART_OVERSAMPLING_16;
	huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart7.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart7) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart7, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart7, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart7) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART7_Init 2 */

	/* USER CODE END UART7_Init 2 */

}

/**
 * @brief UART8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART8_Init(void) {

	/* USER CODE BEGIN UART8_Init 0 */

	/* USER CODE END UART8_Init 0 */

	/* USER CODE BEGIN UART8_Init 1 */

	/* USER CODE END UART8_Init 1 */
	huart8.Instance = UART8;
	huart8.Init.BaudRate = 115200;
	huart8.Init.WordLength = UART_WORDLENGTH_8B;
	huart8.Init.StopBits = UART_STOPBITS_1;
	huart8.Init.Parity = UART_PARITY_NONE;
	huart8.Init.Mode = UART_MODE_TX_RX;
	huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart8.Init.OverSampling = UART_OVERSAMPLING_16;
	huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart8) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART8_Init 2 */

	/* USER CODE END UART8_Init 2 */

}

/**
 * @brief UART9 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART9_Init(void) {

	/* USER CODE BEGIN UART9_Init 0 */

	/* USER CODE END UART9_Init 0 */

	/* USER CODE BEGIN UART9_Init 1 */

	/* USER CODE END UART9_Init 1 */
	huart9.Instance = UART9;
	huart9.Init.BaudRate = 38400;
	huart9.Init.WordLength = UART_WORDLENGTH_8B;
	huart9.Init.StopBits = UART_STOPBITS_1;
	huart9.Init.Parity = UART_PARITY_NONE;
	huart9.Init.Mode = UART_MODE_TX;
	huart9.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart9.Init.OverSampling = UART_OVERSAMPLING_16;
	huart9.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart9.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart9.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart9) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart9, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart9, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart9) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART9_Init 2 */

	/* USER CODE END UART9_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, SD_CARD_CS_Pin | RELAY1_Pin | RELAY3_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, ENABLE_RELAYS_Pin | LORA_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD, RELAY4_Pin | RELAY2_Pin | LORA_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : SD_CARD_CS_Pin RELAY1_Pin RELAY3_Pin */
	GPIO_InitStruct.Pin = SD_CARD_CS_Pin | RELAY1_Pin | RELAY3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : ENABLE_RELAYS_Pin */
	GPIO_InitStruct.Pin = ENABLE_RELAYS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ENABLE_RELAYS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_CS_Pin */
	GPIO_InitStruct.Pin = LORA_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SYSTEM_BUTTON_Pin */
	GPIO_InitStruct.Pin = SYSTEM_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(SYSTEM_BUTTON_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : RELAY4_Pin RELAY2_Pin LORA_RST_Pin */
	GPIO_InitStruct.Pin = RELAY4_Pin | RELAY2_Pin | LORA_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : LORA_DIO0_Pin */
	GPIO_InitStruct.Pin = LORA_DIO0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
