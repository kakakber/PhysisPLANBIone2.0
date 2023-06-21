/**
  ******************************************************************************
  * @file    LoRa.c
  * @author  Enrico Alberti, Physis PEB 2023
  * @brief   C file of LORA module.
  ******************************************************************************
  */


#include "LoRa.h"


/*'private' functions*/
static void loRaWriteRegister(uint8_t, uint8_t);
static uint8_t loRaReadRegister(uint8_t);
static void loRaSetOCP(uint8_t);
static void loRaSetLDOFlag();

//ports

static GPIO_TypeDef* LORA_SPI_CS_Port;
static uint16_t LORA_SPI_CS_Pin;

static GPIO_TypeDef* LORA_DI0_Port;
static uint16_t LORA_DIO_Pin;

static GPIO_TypeDef* LORA_RST_Port;
static uint16_t LORA_RST_Pin;

static SPI_HandleTypeDef* loRa_hspi;

static uint8_t FifoTxBaseAddr = 0x80;


/*
enum operationType {
  TX_MODE,
  RX_MODE
};*/





//datasheet da sx a dx 7...0
/**
 * Takes a defined register as input, puts a '1' first to signal write mode to lora module
 * ex: reg=0x42 -> 1 0x42 = 11000010
 */
void loRaWriteRegister(uint8_t reg, uint8_t data){
	//writes to register, pooling mode
	uint8_t inpData[2];
	inpData[0] = reg | 0x80, //sets first bit to 0
	inpData[1] = data;

	HAL_GPIO_WritePin(LORA_SPI_CS_Port, LORA_SPI_CS_Pin, GPIO_PIN_RESET);

	//read the register in pooling mode and not interrupting
	HAL_SPI_Transmit(loRa_hspi, inpData, 2, 10);

	//end of transaction
	HAL_GPIO_WritePin(LORA_SPI_CS_Port, LORA_SPI_CS_Pin, GPIO_PIN_SET);

}
//cad callback has a boolean parameter
/**
 *
 */
void loRaDIO0InterruptHandler(void (*TXcallback)(void), void (*RXcallback)(void), void (*CADCallback)(int), void (*CRCErrorCallback)(void)){
	//calls callback when needed
	//read irq flags to know the state
	uint8_t irqFlags = loRaReadRegister(REG_IRQ_FLAGS);

	//clear irq flags, writing one on the 1's clears the irq
	loRaWriteRegister(REG_IRQ_FLAGS, irqFlags);
	/*
	 * CAD detection:
	 * If interrupt has cad done the cad operation has completed, check cadDetected to see if radio trasmission was detected on the channel
	 */
	if((irqFlags & IRQ_CAD_DONE_MASK) == IRQ_CAD_DONE_MASK){
		//channel activity detection done
		int cadDetected = irqFlags & IRQ_CAD_DETECTED_MASK;
		if(CADCallback){
			//if there is a completion handler
			CADCallback(cadDetected);
		}
		return;
	}

	if((irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK) == IRQ_PAYLOAD_CRC_ERROR_MASK){
		if(CRCErrorCallback){
			CRCErrorCallback();
		}
		return;
	}

	if((irqFlags & IRQ_RX_DONE_MASK) == IRQ_RX_DONE_MASK){
		//packet received
		//TODO: handle packet received
		return;
	}
	if((irqFlags & IRQ_TX_DONE_MASK) == IRQ_TX_DONE_MASK){
		if(TXcallback){
			TXcallback();
		}
		return;
	}
	return;
}

/*
 * Set up to be asynchronous, sends packet and the response to the di0 txdone interrupt HAS to be handled by the user
 * In order to write packet data into FIFO user should: (page 36)
	1 Set FifoPtrAddr to FifoTxPtrBase.
	2 Write PayloadLength bytes to the FIFO (RegFifo)
 */
int loRaSendPacket(int explicitHeader, const uint8_t *buffer, size_t size){
	//first check if not already transmitting

	if ((loRaReadRegister(REG_OP_MODE) & MODE_TX) == MODE_TX) {
	    return 0;//cannot send packet, already sending!
	}
	if (loRaReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) {
	    // clear IRQ's
	    loRaWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	}
	loRaIdle();
	if(explicitHeader){
		loRaWriteRegister(REG_MODEM_CONFIG_1, loRaReadRegister(REG_MODEM_CONFIG_1) | 0x00);
	}else{
		/*
		 *In certain scenarios, where the payload, coding rate and CRC presence are fixed or known in advance, it may be advantageous
		 *to reduce transmission time by invoking implicit header mode. In this mode the header is removed from the packet.
		 *to In this case the payload length, error coding rate and presence of the payload CRC must be manually configured on both sides of the radio link.
		 */
		loRaWriteRegister(REG_MODEM_CONFIG_1, loRaReadRegister(REG_MODEM_CONFIG_1) | 0x01);
	}
	/*The actual location to be read from, or written to, over the SPI interface is defined by the address pointer FifoAddrPtr.
	 * Before any read or write operation it is hence necessary to initialise this pointer to the corresponding base value.
	*/
	//every time we transmit we initialize the FIFO buffer, the uC will handle bigger buffers itself
	loRaWriteRegister(REG_FIFO_ADDR_PTR, 0);
	loRaWriteRegister(REG_PAYLOAD_LENGTH, 0);

	//send the data
	//int actualLen = readRegister(REG_PAYLOAD_LENGTH);//not useful for now
	if(size > MAX_PKT_LENGTH){
		return 1; //fail
	}

	//write data to fifo
	/*
	 * From docs: Upon reading or writing to the FIFO data buffer (RegFifo) the address pointer will then increment automatically.
	 */
	 for (size_t i = 0; i < size; i++) {
	    loRaWriteRegister(REG_FIFO, buffer[i]);
	  }
	 //update the payload
	 loRaWriteRegister(REG_PAYLOAD_LENGTH, size);

	 //be sure that DIO0 has the right mapping, otherwise no interrupt and tx state forever!
	 //in tx has to be mapped 01 => updates on tx done
	 loRaWriteRegister(REG_DIO_MAPPING_1, 0x40);
	 //set mode to tx
	 loRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
	 return 1;

}

void loRaClearIRQ(){
	uint8_t irqFlags = loRaReadRegister(REG_IRQ_FLAGS);

	//clear irq flags, writing one on the 1's clears the irq
	loRaWriteRegister(REG_IRQ_FLAGS, irqFlags);
}


int loRaIsTrasmitting(){
	if((loRaReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) != IRQ_TX_DONE_MASK){
		//trasmitting
		return 1;
	}else{
		return 0;
	}
}

/*
 * uint8_t irqFlags = loRaReadRegister(REG_IRQ_FLAGS);

	//clear irq flags, writing one on the 1's clears the irq
	loRaWriteRegister(REG_IRQ_FLAGS, irqFlags);
 */

int loRaSendPacketSYNC(int explicitHeader, const uint8_t *buffer, size_t size){
	 loRaSendPacket(explicitHeader, buffer, size);
	 // wait for TX done
	 while ((loRaReadRegister(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) != IRQ_TX_DONE_MASK);
	 // clear IRQ's
	 loRaWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
	 return 1;

}

void loRaClearIRQReceive(){
	loRaWriteRegister(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

int loRaSendPacketSynchronous(int explicitHeader, const uint8_t *buffer, size_t size){

	 return 1;

}



void loRaFullyTransmitMode(){
	//the user just wants to transmit, set the base of the FIFO buffer to 0x00 to use all the available 256 byte buffer
	loRaWriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
	FifoTxBaseAddr = 0x00;
}

void loRaFullyReceiveMode(){
	//the user just wants to receive, set the base of the FIFO buffer to 0x00 (which is already the default)
	loRaWriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
}




/**
 * Takes a defined register as input, puts a '0' first to signal read mode to lora module
 * ex: reg=0x42 -> 0 0x42 = 01000010
 * Puts 8 more dummy bits while receiving in MISO
 */
uint8_t loRaReadRegister(uint8_t reg){
	//accepts the 7 bits of address, MSB first
	uint8_t inpData[2];
	inpData[0] = reg & 0x7F; //sets first bit to 0 and the rest as requested
	inpData[1] = 0x00; //dummy data when receiving
	uint8_t rxData[2];


	//lora CS to low to begin
	HAL_GPIO_WritePin(LORA_SPI_CS_Port, LORA_SPI_CS_Pin, GPIO_PIN_RESET);

	//read the register in pooling mode and not interrupting
	HAL_SPI_TransmitReceive(loRa_hspi, inpData, rxData, 2, 10);

	//end of transaction
	HAL_GPIO_WritePin(LORA_SPI_CS_Port, LORA_SPI_CS_Pin, GPIO_PIN_SET);

	return rxData[1];
}


/**
 * Sets over current protection, right trim for chosen maximum mA
 * Trimming of OCP current:
 * Imax = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) / Imax = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240 mA)
 */
void loRaSetOCP(uint8_t mA){
	uint8_t ocpTrim = 27;
	if (mA <= 120) {
	   ocpTrim = (mA - 45) / 5;
	 } else if (mA <=240) {
	   ocpTrim = (mA + 30) / 10;
	}
	loRaWriteRegister(REG_OCP, 0x20 | (0x1F & ocpTrim));
}



void loRaSetSyncWord(int syncWord){
	loRaWriteRegister(REG_SYNC_WORD, syncWord);
}



/**
 * Initializes the lora for trasmit ops
 */
int loRaInit(SPI_HandleTypeDef* hspi_in, GPIO_TypeDef* spi_cs_port, uint16_t spi_cs_pin, GPIO_TypeDef* di0_port, uint16_t di0_pin, GPIO_TypeDef* rst_port, uint16_t rst_pin){
	loRa_hspi = hspi_in;

	LORA_SPI_CS_Port = spi_cs_port;
	LORA_SPI_CS_Pin = spi_cs_pin;

	LORA_DI0_Port = di0_port;
	LORA_DIO_Pin = di0_pin;

	LORA_RST_Port = rst_port;
	LORA_RST_Pin = rst_pin;

	//important! pu after gpio and spi init
	//put cs high (dont write)
	HAL_GPIO_WritePin(LORA_RST_Port, LORA_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LORA_RST_Port, LORA_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(LORA_SPI_CS_Port, LORA_SPI_CS_Pin, GPIO_PIN_SET);
	//random check to see if chip is working, otherwise done
	while (loRaReadRegister(REG_VERSION) != 0x12){

		return 0;
	}
	//MODES: 000 SLEEP 001 STDBY 010 Frequency synthesis TX (FSTX) 011 Transmit (TX) 100 Frequency synthesis RX (FSRX) 101 Receive continuous (RXCONTINUOUS) 110receive single (RXSINGLE) 111Channel activity detection (CAD)
	loRaSleep();
	loRaSetFrequency(0);

	//LoRa fifo base addr
	//loRaWriteRegister(REG_FIFO_TX_BASE_ADDR, 0);
	//loRaWriteRegister(REG_FIFO_RX_BASE_ADDR, 0);

	//set LNA boost
	loRaWriteRegister(REG_LNA, 0x23); //maximum gain, default current, boost on TO CHECK!!

	//set modem config (Low data rate optimuze enabled?)
	loRaWriteRegister(REG_MODEM_CONFIG_3, 0x04);

	//set tx power, defaults at 15 dbm (boost set to true!)
	loRaSetTxPower(15, TRUE);

	// put in standby mode
	loRaIdle();
	return 1;
}


void loRaEnableCRC(){
  uint8_t prevValue = loRaReadRegister(REG_MODEM_CONFIG_2);
  loRaWriteRegister(REG_MODEM_CONFIG_2, prevValue | 0x04);
}

/**
 * Handling SF 6 (not usable in europe)
 * from datasheet:
 * SF = 6 Is a special use case for the highest data rate transmission possible with the LoRa modem.
 * 	- To this end several settings must be activated in the RFM95W/96W/98W registers when it is in use:
 * 	- Set SpreadingFactor = 6 in RegModemConfig2
 * 	- The header must be set to Implicit mode
 * 	- Write bits 2-0 of register address 0x31 to value "0b101"
 * 	- Write register address 0x37 to value 0x0C
 */

void loRaSetSpreadingFactor(int sf){
	if(sf<6){
		sf = 6;
	}else if (sf > 12){
		sf = 12;
	}

	if(sf == 6){
		loRaWriteRegister(REG_DETECTION_OPTIMIZE, 0xc5);//or maybe 0x05
		loRaWriteRegister(REG_DETECTION_THRESHOLD, 0x0C);
		//set header to implicit mode
	}else{
		loRaWriteRegister(REG_DETECTION_OPTIMIZE, 0xc3);//or maybe 0x05
		loRaWriteRegister(REG_DETECTION_THRESHOLD, 0x0a);
	}

	uint8_t prevReg = loRaReadRegister(REG_MODEM_CONFIG_2);

	loRaWriteRegister(REG_MODEM_CONFIG_2, (prevReg & 0x0f) | ((sf << 4) & 0xf0));//same last 4 and shift sf log2 in first 4 bits

	loRaSetLDOFlag();
}

uint8_t loRaReadConfig1(){
	return loRaReadRegister(REG_MODEM_CONFIG_1);
}

/**
 * Error coding rate bit 3-1 of 0x1D reg
 * - 001 4/5
 * - 010 4/6
 * - 011 4/7
 * - 100 4/8
 */

void loRaSetCodingRate(int denom){
	if (denom < 5) {
		denom = 5;
	  } else if (denom > 8) {
		  denom = 8;
	  }

	  int cr = denom - 4;

	  loRaWriteRegister(REG_MODEM_CONFIG_1, (loRaReadRegister(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * set low data rate optimization flag
 */
void loRaSetLDOFlag(){
	  // Section 4.1.1.5
	  long symbolDuration = 1000 / ( loRaGetSignalBandwidth() / (1L << loRaGetSpreadingFactor()) ) ;

	  // Section 4.1.1.6

	  uint8_t config3 = loRaReadRegister(REG_MODEM_CONFIG_3);
	  if(symbolDuration > 16){
		  config3 |= 0x08; //set LowDataRateOptimize to true
	  }else{
		  config3 &= 0xF7; //set LowDataRateOptimize to false
	  }
	  loRaWriteRegister(REG_MODEM_CONFIG_3, config3);
}


void loRaSetSignalBandwidth(long sbw){
  int bw;

  if (sbw <= 7.8E3) {
    bw = 0;
  } else if (sbw <= 10.4E3) {
    bw = 1;
  } else if (sbw <= 15.6E3) {
    bw = 2;
  } else if (sbw <= 20.8E3) {
    bw = 3;
  } else if (sbw <= 31.25E3) {
    bw = 4;
  } else if (sbw <= 41.7E3) {
    bw = 5;
  } else if (sbw <= 62.5E3) {
    bw = 6;
  } else if (sbw <= 125E3) {
    bw = 7;
  } else if (sbw <= 250E3) {
    bw = 8;
  } else{
    bw = 9;
  }

  loRaWriteRegister(REG_MODEM_CONFIG_1, (loRaReadRegister(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
  loRaSetLDOFlag();
}


long loRaGetSignalBandwidth(){

	  uint8_t bw = (loRaReadRegister(REG_MODEM_CONFIG_1) >> 4);

	  switch (bw) {
	    case 0: return 7.8E3;
	    case 1: return 10.4E3;
	    case 2: return 15.6E3;
	    case 3: return 20.8E3;
	    case 4: return 31.25E3;
	    case 5: return 41.7E3;
	    case 6: return 62.5E3;
	    case 7: return 125E3;
	    case 8: return 250E3;
	    case 9: return 500E3;
	  }
	  return -1;
}

int loRaGetSpreadingFactor(){
	return loRaReadRegister(REG_MODEM_CONFIG_2) >> 4;
}


//TODO: think about continuous mode


void loRaSetTxPower(int level, int boosted){
	//we will set 1 as PA output pin (PA BOOST with output limited to 20 dbm)
	/*
	 * From specs:
	 * bit 7: 1 with PA_BOOST enabled (always enabled in our case) - 0 with RFO pin
	 * 		RFM95W/96W/98W feature three distinct RF power amplifiers. Two of those, connected to RFO_LF and RFO_HF,
	 * 		can deliver up to +14 dBm, are unregulated for high power efficiency and can be connected directly to their
	 * 		respective RF receiver inputs via a pair of passive components to form a single antenna port high efficiency transceiver.
	 * 		The third PA, connected to the PA_BOOST pin and can deliver up to +20 dBm via a dedicated matching network.
	 * bit 6 to 4: max output power: Pmax=10.8+0.6*MaxPower[dBm]
	 * bit 3 to 0: output power:
	 * 			with RFO: Pout=Pmax-(15-OutputPower)
	 * 			with PA_BOOST: Pout=17-(15-OutputPower)
	 *
	 * high and power settings from page 80 of datasheet (REG_PA_DAC settings)
	 */
	if(boosted == FALSE){
		if(level < 0){
			level = 0;
		}else if (level > 14){
			level = 14;
		}
		loRaWriteRegister(REG_PA_CONFIG, 0x70 | level);
	}else{
		//BOOST ON (default)
		if (level > 17) {
		   if (level > 20) {
			   level = 20;
		   }
		   level = level - 3;
		   loRaWriteRegister(REG_PA_DAC, 0x87);
		   loRaSetOCP(140);
		}else{
		   if (level < 2) {
			  level = 2;
		   }
		   loRaWriteRegister(REG_PA_DAC, 0x84);
		   loRaSetOCP(100);
		}
		loRaWriteRegister(REG_PA_CONFIG, PA_BOOST | (level - 2));
	}
}


void loRaSleep(){
	//sets the reg op to sleep (001 on last 3 bits)
	loRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

void loRaIdle(){
	//sets the reg op to idle (001 on last 3 bits)
	loRaWriteRegister(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}




/**
 * input is the value of .x MHZ ex: loRaSetFrequency(3) sets 868.3MHZ working freq
 */
void loRaSetFrequency(uint8_t value){
	//only to set when in SLEEP or STBY
	//formula: f=F(xosc)*frf/2^19 where F(xosc)=32Mhz
	uint32_t frf = (((uint64_t)(LORA_BASE_FREQ + value*100000))<<19)/32000000;

	//the freq is set in 3 registers, MSB, MID, LSB
	//24 bit di dato 0x112233
	loRaWriteRegister(REG_FRF_MSB, (uint8_t)(frf>>16)); //11 in casting
	loRaWriteRegister(REG_FRF_MID, (uint8_t)(frf>>8)); //22 in casting
	loRaWriteRegister(REG_FRF_LSB, (uint8_t)(frf>>0)); //33 in casting
}


