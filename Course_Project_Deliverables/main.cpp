/* 

ECEN 5023 - Course Project
Saksham Sabharwal

*/


/* Adding header files */
#include "mbed.h"
#include "em_chip.h"
#include "em_emu.h"
#include "em_letimer.h"
#include "em_int.h"
#include "em_leuart.h"
#include "em_acmp.h"
#include "em_adc.h"
#include "em_dma.h"
#include "em_i2c.h"
#include "sleepmodes.h"
#include "ZX_Sensor.h"
#include "EFM32_CapSenseSlider.h"
#include "em_device.h"
#include "em_msc.h"
#include "em_msc.h"
#include "em_assert.h"
#include <stdbool.h>
#include <string.h>
#include <string.h>
#include <stdlib.h>

/* Defining macros */

#if defined (_EFM32_GECKO_FAMILY) || defined(_EFM32_TINY_FAMILY)
#define PAGE_SIZE                        0x200
#elif defined(_EFM32_GIANT_FAMILY)
#define PAGE_SIZE                       ((FLASH_SIZE >= 0x80000) ? 0x1000 : 0x800)
#else
#pragma error "Unknown page size"
#endif

#define DEFAULT_NUM_PAGES          2
#define MAX_NUM_PAGES              (FLASH_SIZE/PAGE_SIZE)

#define SIZE_OF_DATA                     2                                        /* 2 bytes */
#define SIZE_OF_VIRTUAL_ADDRESS          2                                        /* 2 bytes */
#define SIZE_OF_VARIABLE                 (SIZE_OF_DATA + SIZE_OF_VIRTUAL_ADDRESS) /* 4 bytes */

#define MAX_ACTIVE_VARIABLES             (PAGE_SIZE / SIZE_OF_VARIABLE)

#define PAGE_STATUS_ERASED               0xFF
#define PAGE_STATUS_RECEIVING            0xAA
#define PAGE_STATUS_ACTIVE               0x00

#define WRITE_EEPROM true

#define Debug 	false
#define BUFF_RCV_LENGTH	8
#define GESTURE_ADDRESS	0x10 << 1
#define DMA_CHANNEL_RX	0
#define DMA_CHANNEL_TX	2
#define DMA_CHANNEL_ADC	1
#define DMA_CHANNELS	3
#define PROGRAM_BLE			true
#define PROGRAM_BLE_RESET	false
#define BLE_COMMAND_MODE	"+++\n"
#define BLE_DISABLE_ECHO	"ATE=0\n"
#define BLE_COMMAND_TEST	"AT\n"
#define BLE_COMMAND_INFO	"ATI\n"
#define BLE_FACTORY_RESET	"AT+FACTORYRESET\n"
#define BLE_COMMAND_RESET	"ATZ\n"
#define BLE_COMMAND_1		"AT+GAPINTERVALS=50,4000,10230,50\n"
#define BLE_COMMAND_2		"AT+HWMODELED=DISABLE\n"
#define BLE_COMMAND_3		"AT+BLEPOWERLEVEL=-8\n"
#define BLE_TX	PD4
#define BLE_RX	PD5
#define BLE_BAUD_RATE 		9600
#define BLE_DATA_BITS 		8
#define BLE_PARITY 			SerialBase::None
#define BLE_STOP_BITS 		1
#define LEUART_PORT 		gpioPortD
#define LEUART_PIN_CTS 		15
#define LEUART_PIN_TXO 		4
#define LEUART_PIN_RXI 		5
#define LEUART_BAUD_RATE 	9600
#define LEUART_DATA_BITS	leuartDatabits8
#define LEUART_PARITY 		leuartNoParity
#define LEUART_STOP_BITS 	leuartStopbits1
#define DESIRED_PERIOD 4
#define DUTY_CYCLE	99.9
#define LIGHTSENSE_EXCITE_PIN 6
#define LIGHTSENSE_EXCITE_PORT gpioPortD

#define ADCSAMPLES 	200
#define _T_ACQ 		adcAcqTime2
#define ADCFREQ_PR 	875000
#define LOWER_TEMP_LIMIT 	0
#define UPPER_TEMP_LIMIT 	30
#define BUFF_MSG_LEN 	40
#define BUFF_MSG_STR 	31
#define BUFF_TMP_LEN 	9

/* Defining types */

typedef enum {
	eePageStatusErased = PAGE_STATUS_ERASED,
	eePageStatusReceiving = PAGE_STATUS_RECEIVING,
	eePageStatusActive = PAGE_STATUS_ACTIVE,
} EE_PageStatus_TypeDef;

typedef struct {
	/* Each variable is assigned a unique virtual address automatically when first
	 * written to, or when using the declare function. */
	uint16_t virtualAddress;
} EE_Variable_TypeDef;

typedef struct {
	uint32_t *startAddress;
	uint32_t *endAddress;
} EE_Page_TypeDef;

/* Function prototypes */

void ADC_TempSetup(void);
void ADC_TransferComplete(unsigned int, bool, void *);
float convertToCelsius(int32_t);

void DMA_AllSetup(void);

void TX_DataSent(unsigned int, bool, void *);
void RX_DataReceived(unsigned int channel, bool primary, void *user);

void I2C_Initialize(void);
extern char I2C_Read(char);
extern void I2C_Write(char, char);

void BSP_TraceSwoSetup(void);

void LETIMER0_Init();
void LEUART0_RxInterrupt(void);
void LEUART_Init(void);
void BLE_Configure(void);
void gesture_init(void);
void gesture_read(void);
void proximity_interrupt1(void);
void proximity_interrupt2(void);
void touch(void);

bool EE_Init(uint32_t);
bool EE_Format(uint32_t);
bool EE_Read(EE_Variable_TypeDef *var, uint16_t *readData);
void EE_Write(EE_Variable_TypeDef *var, uint16_t writeData);
bool EE_DeclareVariable(EE_Variable_TypeDef *var);
void EE_DeleteVariable(EE_Variable_TypeDef *var);
uint32_t EE_GetEraseCount(void);
void moveInterruptVectorToRam(void);
void EEPROM_MyWrite(void);

/* Defining variables */

EE_Variable_TypeDef tempup, templow;
uint16_t readtempup, readtemplow;

DigitalOut myLed0(LED0);
DigitalOut myLed1(LED1);

Serial pc(USBTX, USBRX);

Serial ble_program(BLE_TX, BLE_RX);

ZX_Sensor zx_sensor(GESTURE_ADDRESS);
InterruptIn gesture_int(PC3);
InterruptIn proximity_int(PA12);

silabs::EFM32_CapSenseSlider capsense;

char Below[BUFF_MSG_LEN] = "Temperature under set minimum = XX.XC\n\r";
char Above[BUFF_MSG_LEN] = "Temperature over set maximum = XX.XC\n\r";
char Tempr[BUFF_TMP_LEN] = " XX.XC\n\r";
char Receive[BUFF_RCV_LENGTH + 1] = { 0 };

char RightMsg[] = "Swipe Right\n\r";
char LeftMsg[] = "Swipe Left\n\r";
char UpMsg[] = "Swipe Up\n\r";

char Lit[] = "Illuminated\n\r";
char Dark[] = "Dark\n\r";
bool light_detected = false;

char NoGMsg[] = "NoSwipe\n\r";

char NearMsg[] = "Proximity = Near\n\r";
char FarMsg[] = "Proximity = Nothing\n\r";
bool proximity_detected = false;

char NoTouch[] = "No touch\n\r";
char Touch[] = "Touch sensed!\n\r";

DMA_CB_TypeDef cb_ADC;
DMA_CB_TypeDef cb_RX;
DMA_CB_TypeDef cb_TX;

uint8_t cmd_array[1];
uint8_t data_array[10];
uint16_t ADC_Buffer[ADCSAMPLES] = { 0 };
volatile bool ADC_TransferActive;

void BLE_Configure(void) {
	if (PROGRAM_BLE) {
		char str[256];
#define WaitForOK {memset(str,0,256);while(strcmp(str,"OK")){ble_program.scanf("%s",str);}}
		ble_program.baud(BLE_BAUD_RATE);
		ble_program.format(BLE_DATA_BITS, BLE_PARITY, BLE_STOP_BITS);

		ble_program.printf(BLE_COMMAND_MODE);
		WaitForOK

		ble_program.printf(BLE_DISABLE_ECHO);
		WaitForOK

		ble_program.printf(BLE_COMMAND_1);
		WaitForOK
		ble_program.printf(BLE_COMMAND_2);
		WaitForOK
		ble_program.printf(BLE_COMMAND_3);
		WaitForOK

		ble_program.printf(BLE_COMMAND_RESET);
		WaitForOK
	}
	ble_program.attach(&LEUART0_RxInterrupt, Serial::RxIrq);
	delete ble_program;
}

void LEUART_Init(void) {
	CMU_ClockEnable(cmuClock_LEUART0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);

	GPIO_PinModeSet(LEUART_PORT, LEUART_PIN_CTS, gpioModePushPull, 0);
	GPIO_PinOutClear(LEUART_PORT, LEUART_PIN_CTS);

	GPIO_PinModeSet(LEUART_PORT, LEUART_PIN_TXO, gpioModePushPull, 1);
	GPIO_PinModeSet(LEUART_PORT, LEUART_PIN_RXI, gpioModeInputPull, 1);

	LEUART_Init_TypeDef leuart0Init;
	leuart0Init.enable = leuartEnable;
	leuart0Init.refFreq = 0;
	leuart0Init.baudrate = LEUART_BAUD_RATE;
	leuart0Init.databits = LEUART_DATA_BITS;
	leuart0Init.parity = LEUART_PARITY;
	leuart0Init.stopbits = LEUART_STOP_BITS;

	LEUART_Reset(LEUART0 );
	LEUART_Init(LEUART0, &leuart0Init);

	LEUART0 ->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN
			| LEUART_ROUTE_LOCATION_LOC0;

	LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU | LEUART_CTRL_RXDMAWU;
	LEUART0 ->SIGFRAME = '!';

	LEUART_IntEnable(LEUART0, LEUART_IEN_SIGF);

	NVIC_EnableIRQ(LEUART0_IRQn);
}

void gesture_init(void) {
	uint8_t ver;
	if (!zx_sensor.init(GESTURE_INTERRUPTS)) {
		/* Something went wrong during ZX Sensor init!*/
		while (1)
			;
	}
	/* ZX Sensor initialization complete */

	ver = zx_sensor.getModelVersion();
	if (ver == ZX_ERROR) {
		/* Error reading model version number */while (1)
			;
	}
	if (ver != ZX_MODEL_VER) {
		/* Model version needs to be ZX_MODEL_VER to work with this library. Stopping */
		while (1)
			;
	}

	gesture_int.rise(&gesture_read);
	zx_sensor.clearInterrupt();
}

void LEUART0_RxInterrupt(void) {
#define BUFF_ERR_LEN	33
	static char Error[BUFF_ERR_LEN] = "RetTT! ERROR: Invalid input!\n\r";
	uint32_t leuartif;
	leuartif = LEUART_IntGet(LEUART0 );
	LEUART_IntClear(LEUART0, leuartif);
	if (leuartif & LEUART_IF_SIGF) {
		LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
		if (strstr(Receive, "RetTemp!")) {
			DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
					(void*) (&LEUART0 ->TXDATA), Tempr, strlen(Tempr) - 1);
		} else if (strstr(Receive, "RetDark!")) {
			if (light_detected) {
				DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
						(void*) (&LEUART0 ->TXDATA), Lit, strlen(Lit) - 1);
			} else {
				DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
						(void*) (&LEUART0 ->TXDATA), Dark, strlen(Dark) - 1);
			}
		} else if (strstr(Receive, "RetProx!")) {
			if (proximity_detected) {
				DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
						(void*) (&LEUART0 ->TXDATA), NearMsg,
						strlen(NearMsg) - 1);
			} else {
				DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
						(void*) (&LEUART0 ->TXDATA), FarMsg,
						strlen(FarMsg) - 1);
			}
		} else {
			DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
					(void*) (&LEUART0 ->TXDATA), Error, strlen(Error) - 1);
		}
		memset(Receive, 0, BUFF_RCV_LENGTH);
	}
	DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, Receive,
			(void *) &(LEUART0 ->RXDATA), BUFF_RCV_LENGTH - 1);
}

void gesture_read(void) {
	GestureType gesture;

	zx_sensor.clearInterrupt();
	gesture = zx_sensor.readGesture();

	switch (gesture) {
	case NO_GESTURE:
		break;
	case RIGHT_SWIPE:
		LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), RightMsg, strlen(RightMsg) - 1);
		break;
	case LEFT_SWIPE:
		LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), LeftMsg, strlen(LeftMsg) - 1);
		break;
	case UP_SWIPE:
		LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), UpMsg, strlen(UpMsg) - 1);
		break;
	default:
		break;
	}
}

void proximity_interrupt1(void) {
	proximity_detected = true;
}
void proximity_interrupt2(void) {
	proximity_detected = false;
}

void touch(void) {
	LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
	if (!capsense.isTouched()) {
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), NoTouch, strlen(NoTouch) - 1);
	} else {
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), Touch, strlen(Touch) - 1);
	}
}


void TX_DataSent(unsigned int channel, bool primary, void *user) {
	(void) channel;
	(void) primary;
	(void) user;

	/* Disable DMA wake-up from LEUART0 TX */
	LEUART0 ->CTRL &= ~LEUART_CTRL_TXDMAWU;
}

void RX_DataReceived(unsigned int channel, bool primary, void *user) {
	(void) channel;
	(void) primary;
	(void) user;

	DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, Receive,
			(void *) &(LEUART0 ->RXDATA), BUFF_RCV_LENGTH - 1);
}

void DMA_AllSetup(void) {
	CMU_ClockEnable(cmuClock_DMA, true);

	DMA_Init_TypeDef dmaInit;
	DMA_CfgChannel_TypeDef rxChnlCfg;
	DMA_CfgChannel_TypeDef txChnlCfg;
	DMA_CfgChannel_TypeDef adcChnlCfg;
	DMA_CfgDescr_TypeDef rxDescrCfg;
	DMA_CfgDescr_TypeDef txDescrCfg;
	DMA_CfgDescr_TypeDef adcDescrCfg;

	dmaInit.hprot = 0;
	dmaInit.controlBlock = dmaControlBlock;
	DMA_Init(&dmaInit);

	cb_RX.cbFunc = RX_DataReceived;
	cb_RX.userPtr = NULL;

	rxChnlCfg.highPri = true;
	rxChnlCfg.enableInt = false;
	rxChnlCfg.select = DMAREQ_LEUART0_RXDATAV;
	rxChnlCfg.cb = &cb_RX;
	DMA_CfgChannel(DMA_CHANNEL_RX, &rxChnlCfg);

	rxDescrCfg.dstInc = dmaDataInc1;
	rxDescrCfg.srcInc = dmaDataIncNone;
	rxDescrCfg.size = dmaDataSize1;
	rxDescrCfg.arbRate = dmaArbitrate1;
	rxDescrCfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL_RX, true, &rxDescrCfg);

	cb_TX.cbFunc = TX_DataSent;
	cb_TX.userPtr = NULL;

	txChnlCfg.highPri = false;
	txChnlCfg.enableInt = false;
	txChnlCfg.select = DMAREQ_LEUART0_TXBL;
	txChnlCfg.cb = &cb_TX;
	DMA_CfgChannel(DMA_CHANNEL_TX, &txChnlCfg);

	txDescrCfg.dstInc = dmaDataIncNone;
	txDescrCfg.srcInc = dmaDataInc1;
	txDescrCfg.size = dmaDataSize1;
	txDescrCfg.arbRate = dmaArbitrate1;
	txDescrCfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL_TX, true, &txDescrCfg);

	DMA ->IEN |= DMA_IEN_CH0DONE;
	DMA ->IEN |= DMA_IEN_CH2DONE;

	NVIC_EnableIRQ(DMA_IRQn);

	cb_ADC.cbFunc = ADC_TransferComplete;
	cb_ADC.userPtr = NULL;

	adcChnlCfg.highPri = false;
	adcChnlCfg.enableInt = true;
	adcChnlCfg.select = DMAREQ_ADC0_SINGLE;
	adcChnlCfg.cb = &cb_ADC;
	DMA_CfgChannel(DMA_CHANNEL_ADC, &adcChnlCfg);

	adcDescrCfg.dstInc = dmaDataInc2;
	adcDescrCfg.srcInc = dmaDataIncNone;
	adcDescrCfg.size = dmaDataSize2;
	adcDescrCfg.arbRate = dmaArbitrate1;
	adcDescrCfg.hprot = 0;
	DMA_CfgDescr(DMA_CHANNEL_ADC, true, &adcDescrCfg);
}

void I2C_Initialize(void) {
	CMU_ClockEnable(cmuClock_I2C1, true);
	I2C_Init_TypeDef i2cInit = I2C_INIT_DEFAULT;
	I2C_Init(I2C1, &i2cInit);

	GPIO_PinModeSet(gpioPortE, 2, gpioModePushPull, 0);
	GPIO_PinModeSet(gpioPortC, 4, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortC, 5, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortD, 6, gpioModePushPull, 0);
	GPIO_PinOutSet(gpioPortD, 6);
	I2C1 ->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN
			| (_I2C_ROUTE_LOCATION_LOC0);

}

void i2c_transfer(uint16_t device_addr, uint8_t cmd_array[],
		uint8_t data_array[], uint16_t cmd_len, uint16_t data_len,
		uint8_t flag) {
	I2C_TransferSeq_TypeDef i2cTransfer;

	I2C_TransferReturn_TypeDef result;
	i2cTransfer.addr = device_addr;
	i2cTransfer.flags = flag;
	i2cTransfer.buf[0].data = cmd_array;
	i2cTransfer.buf[0].len = cmd_len;

	i2cTransfer.buf[1].data = data_array;
	i2cTransfer.buf[1].len = data_len;

	result = I2C_TransferInit(I2C1, &i2cTransfer);

	while (result != i2cTransferDone) {
		if (result != i2cTransferInProgress) {
			break;
		}
		result = I2C_Transfer(I2C1 );
	}
}

void I2C_Write(char reg_offset, char write_data) {
	cmd_array[0] = reg_offset;
	data_array[0] = write_data;
	i2c_transfer(GESTURE_ADDRESS, cmd_array, data_array, 1, 1,
			I2C_FLAG_WRITE_WRITE);
}

char I2C_Read(char reg_offset) {
	cmd_array[0] = reg_offset;
	i2c_transfer(GESTURE_ADDRESS, cmd_array, data_array, 1, 1,
			I2C_FLAG_WRITE_READ);
	return data_array[0];
}
void ACMP0_IRQHandler(void) {
	ACMP_IntClear(ACMP0, ACMP_IF_WARMUP);
	static bool light_acmp = true;
	static bool light_once = false;
	for (int i = 1000; i; i--)
		;
	if (light_acmp == true) {
		if (ACMP0 ->STATUS & ACMP_STATUS_ACMPOUT) {
			if (light_once)
				light_detected = true;
			ACMP_ChannelSet(ACMP0, acmpChannelVDD, acmpChannel6); //set to 3.3V
			light_acmp = false; //toggle the flag
			light_once = true;
		}
	} else {
		if ((ACMP0 ->STATUS & ACMP_STATUS_ACMPOUT)== 0){
			light_detected = false;
			ACMP_ChannelSet(ACMP0, acmpChannel1V25, acmpChannel6);
			light_acmp = true;
			light_once = false;
		} else {
			light_detected = true;
			light_once = true;
		}
	}
	GPIO_PinOutClear(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN);
}

void ACMP_Setup(void) {
	/* Enable clock for ACMP0. */
	CMU_ClockEnable(cmuClock_ACMP0, true);
	/* ACMP configuration constant table. */
	static const ACMP_Init_TypeDef acmp0_init = { false, /* Full bias current*/
	true, /* Half bias current - expect half the current to be used*/
	0, /* Biasprog current configuration - least current draw configuration from the AN*/
	false, /* Enable interrupt for falling edge */
	false, /* Enable interrupt for rising edge */
	acmpWarmTime256, /* Warm-up time in clock cycles, >140 cycles for 10us with 14MHz - tried with 128 as well */
	acmpHysteresisLevel0, /* Hysteresis configuration - Increasing the level improved the energy score by a small factor */
	0, /* Inactive comparator output value */
	true, /* Enable low power mode */
	(int) (2.0 * 63 / 3.3), /* Vdd reference scaling */
	true, /* Enable ACMP */
	};

	/* Configure ACMP. */
	ACMP_Init(ACMP0, &acmp0_init);
	/* Disable ACMP0 out to a pin. */
	ACMP_GPIOSetup(ACMP0, 0, false, false);
	// Set up ACMP 0  to 1.25V
	ACMP_ChannelSet(ACMP0, acmpChannel1V25, acmpChannel6);
	// enabling when warmup is complete
	ACMP_IntEnable(ACMP0, ACMP_IF_WARMUP);

	NVIC_EnableIRQ(ACMP0_IRQn);
}

void LETIMER0_Init() {
	CMU_ClockEnable(cmuClock_LETIMER0, true);

	NVIC_EnableIRQ(LETIMER0_IRQn);
	LETIMER_IntEnable(LETIMER0, LETIMER_IF_UF);
	uint16_t TimerTotalValue = 0;
	uint16_t TimerComp1Value = 0;
	uint32_t LETIMER0_Clock = 0;

	LETIMER0_Clock = CMU_ClockFreqGet(cmuClock_LETIMER0);
	TimerTotalValue = (uint16_t) (DESIRED_PERIOD * (LETIMER0_Clock / 1));
	LETIMER_CompareSet(LETIMER0, 0, TimerTotalValue);
	TimerComp1Value = (uint16_t) ((TimerTotalValue * (100 - DUTY_CYCLE) / 100));
	LETIMER_CompareSet(LETIMER0, 1, TimerComp1Value);
	LETIMER_Init_TypeDef Letimer_Init_Config;
	Letimer_Init_Config.enable = true;
	Letimer_Init_Config.debugRun = false;
	Letimer_Init_Config.rtcComp0Enable = false;
	Letimer_Init_Config.rtcComp1Enable = false;
	Letimer_Init_Config.comp0Top = true;
	Letimer_Init_Config.bufTop = false;
	Letimer_Init_Config.out0Pol = 0;
	Letimer_Init_Config.out1Pol = 0;
	Letimer_Init_Config.ufoa0 = letimerUFOANone;
	Letimer_Init_Config.ufoa1 = letimerUFOANone;
	Letimer_Init_Config.repMode = letimerRepeatFree;

	LETIMER_Init(LETIMER0, &Letimer_Init_Config);
}

void LETIMER0_IRQHandler(void) {
	if (LETIMER_IntGet(LETIMER0 ) & LETIMER_IF_COMP1) {
		LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
		ACMP_Disable(ACMP0 );
		GPIO_PinOutSet(LIGHTSENSE_EXCITE_PORT, LIGHTSENSE_EXCITE_PIN);
	} // Handle comparison interrupt
	else if (LETIMER_IntGet(LETIMER0 ) & LETIMER_IF_UF) {
		LETIMER_IntClear(LETIMER0, LETIMER_IF_UF);
		ACMP_Enable(ACMP0 );
		ADC_TransferActive = true;
		DMA_ActivateBasic(DMA_CHANNEL_ADC, true, false, ADC_Buffer,
				(void *) &(ADC0 ->SINGLEDATA), ADCSAMPLES - 1);
	}
}

void ADC_TempSetup(void) {
	CMU_ClockEnable(cmuClock_ADC0, true);
	ADC_Init_TypeDef init = ADC_INIT_DEFAULT;
	ADC_InitSingle_TypeDef sInit = ADC_INITSINGLE_DEFAULT;
	init.timebase = ADC_TimebaseCalc(0);
	init.prescale = ADC_PrescaleCalc(ADCFREQ_PR, 0);
	ADC_Init(ADC0, &init);
	sInit.acqTime = _T_ACQ;
	sInit.reference = adcRef1V25;
	sInit.input = adcSingleInpTemp;
	sInit.rep = true;
	sInit.resolution = adcRes12Bit;
	ADC_InitSingle(ADC0, &sInit);
	ADC_Start(ADC0, adcStartSingle);
}

void ADC_TransferComplete(unsigned int channel, bool primary, void *user) {
	(void) channel;
	(void) primary;
	(void) user;

	float CTemp;
	int CTempInt;
	uint32_t Sum;

	Sum = 0;
	for (int i = 0; i < ADCSAMPLES; i++) {
		Sum += ADC_Buffer[i];
	}
	CTemp = (convertToCelsius(Sum / ADCSAMPLES));
	if (CTemp < 0) {
		Tempr[0] = '-';
		CTempInt = -1 * (CTemp * 10);
	} else {
		Tempr[0] = '+';
		CTempInt = (CTemp * 10);
	}

	Tempr[1] = '0' + (CTempInt / 100);
	Tempr[2] = '0' + (CTempInt % 100) / 10;
	Tempr[4] = '0' + (CTempInt % 10);

	if (CTemp > readtempup) {
		Above[BUFF_MSG_STR + 0] = Tempr[0];
		Above[BUFF_MSG_STR + 1] = Tempr[1];
		Above[BUFF_MSG_STR + 2] = Tempr[2];
		Above[BUFF_MSG_STR + 4] = Tempr[4];
		LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), Above, strlen(Above) - 1);

	} else if (CTemp < readtemplow) {
		Below[BUFF_MSG_STR + 0] = Tempr[0];
		Below[BUFF_MSG_STR + 1] = Tempr[1];
		Below[BUFF_MSG_STR + 2] = Tempr[2];
		Below[BUFF_MSG_STR + 4] = Tempr[4];
		LEUART0 ->CTRL |= LEUART_CTRL_TXDMAWU;
		DMA_ActivateBasic(DMA_CHANNEL_TX, true, false,
				(void*) (&LEUART0 ->TXDATA), Below, strlen(Below) - 1);
	}

	ADC_TransferActive = false;
}

void BSP_TraceSwoSetup(void) {
	/* Enable GPIO clock */
	CMU ->HFPERCLKEN0 |= CMU_HFPERCLKEN0_GPIO;

	/* Enable Serial wire output pin */GPIO ->ROUTE |= GPIO_ROUTE_SWOPEN;

	/* Set correct location */
	/* This location is valid for GG, LG and WG! */GPIO ->ROUTE = (GPIO ->ROUTE
			& ~(_GPIO_ROUTE_SWLOCATION_MASK)) | GPIO_ROUTE_SWLOCATION_LOC0;

	/* Enable output on correct pin. */
	/* This pin is valid for GG, LG and WG! */GPIO ->P[5].MODEL &=
			~(_GPIO_P_MODEL_MODE2_MASK);
	GPIO ->P[5].MODEL |= GPIO_P_MODEL_MODE2_PUSHPULL;

	/* Enable debug clock AUXHFRCO */CMU ->OSCENCMD = CMU_OSCENCMD_AUXHFRCOEN;

	/* Wait until clock is ready */
	while (!(CMU ->STATUS & CMU_STATUS_AUXHFRCORDY))
		;

	/* Enable trace in core debug */CoreDebug ->DHCSR |=
			CoreDebug_DHCSR_C_DEBUGEN_Msk;
	CoreDebug ->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* Enable PC and IRQ sampling output */DWT ->CTRL = 0x400113FF;

	/* Set TPIU prescaler to 16. */TPI ->ACPR = 15;

	/* Set protocol to NRZ */TPI ->SPPR = 2;

	/* Disable continuous formatting */TPI ->FFCR = 0x100;

	/* Unlock ITM and output data */ITM ->LAR = 0xC5ACCE55;
	ITM ->TCR = 0x10009;

	/* ITM Channel 0 is used for UART output */
	ITM ->TER |= (1UL << 0);
}

float convertToCelsius(int32_t adcSample) {
	float temp;
	float cal_temp_0 = (float) ((DEVINFO ->CAL & _DEVINFO_CAL_TEMP_MASK)
			>> _DEVINFO_CAL_TEMP_SHIFT);
	float cal_value_0 = (float) ((DEVINFO ->ADC0CAL2
			& _DEVINFO_ADC0CAL2_TEMP1V25_MASK)
			>> _DEVINFO_ADC0CAL2_TEMP1V25_SHIFT);
	float t_grad = -6.27;
	temp = (cal_temp_0 - ((cal_value_0 - adcSample) / t_grad));
	return temp;
}

static uint32_t pageStatusActiveValue = ((uint32_t) PAGE_STATUS_ACTIVE << 24)
		| 0x00FFFFFF;
static uint32_t pageStatusReceivingValue = ((uint32_t) PAGE_STATUS_RECEIVING
		<< 24) | 0x00FFFFFF;
/* Variables to track which pages are active and receiving. */
static int activePageNumber = -1;
static int receivingPageNumber = -1;

static bool initialized = false;

static int16_t numberOfVariablesDeclared = 0;
static int16_t numberOfActiveVariables = 0;
static int16_t numberOfPagesAllocated;

/* Array of all pages allocated to the eeprom */
static EE_Page_TypeDef pages[MAX_NUM_PAGES];

__STATIC_INLINE EE_PageStatus_TypeDef EE_getPageStatus(EE_Page_TypeDef *page) {
	return (EE_PageStatus_TypeDef) ((*(page->startAddress) >> 24) & 0xFF);
}

__STATIC_INLINE msc_Return_TypeDef EE_setPageStatusActive(
		EE_Page_TypeDef *page) {
	return MSC_WriteWord(page->startAddress, &pageStatusActiveValue,
			SIZE_OF_VARIABLE);
}

__STATIC_INLINE msc_Return_TypeDef EE_setPageStatusReceiving(
		EE_Page_TypeDef *page) {
	return MSC_WriteWord(page->startAddress, &pageStatusReceivingValue,
			SIZE_OF_VARIABLE);
}

#define VECTOR_SIZE (16+38)

/* Align the RAM vector table */
uint32_t vectorTable[VECTOR_SIZE] __attribute__ ((aligned(256)));

/* SysTick interrupt handler. Counts ms ticks */
volatile uint32_t msTicks = 0;

/* Place interrupt handler in RAM */
__attribute__ ((section(".ram")))

void SysTick_Handler(void) {
	msTicks++;
}

void moveInterruptVectorToRam(void) {
	memcpy(vectorTable, (uint32_t*) SCB ->VTOR, sizeof(uint32_t) * VECTOR_SIZE);
	SCB ->VTOR = (uint32_t) vectorTable;
}

static bool EE_validateIfErased(EE_Page_TypeDef *page) {
	uint32_t *address = page->startAddress;

	/* Iterate through all the words of the page, and validate that all bits are set. */
	while (address <= page->endAddress) {
		if (*address != 0xFFFFFFFF) {
			/* 0 bit detected */
			return false;
		}
		address++;
	}
	/* All bits are 1's. */
	return true;
}

static bool EE_WriteToPage(EE_Page_TypeDef *page, uint16_t virtualAddress,
		uint16_t writeData) {
	/* Start at the second word. The fist one is reserved for status and erase count. */
	uint32_t *address = page->startAddress + 1;
	uint32_t virtualAddressAndData;

	/* Iterate through the page from the beginning, and stop at the fist empty word. */
	while (address <= page->endAddress) {
		/* Empty word found. */
		if (*address == 0xFFFFFFFF) {
			/* The virtual address and data is combined to a full word. */
			virtualAddressAndData = ((uint32_t) (virtualAddress << 16)
					& 0xFFFF0000) | (uint32_t) (writeData);

			/* Make sure that the write to flash is a success. */
			if (MSC_WriteWord(address, &virtualAddressAndData, SIZE_OF_VARIABLE)
					!= mscReturnOk) {
				/* Write failed. Halt for debug trace, if enabled. */
				EFM_ASSERT(0);
				return false;
			}
			/* Data written successfully. */
			return true;
		} else {
			address++;
		}
	}
	/* Reached the end of the page without finding any empty words. */
	return false;
}

bool EE_Format(uint32_t numberOfPages) {
	uint32_t eraseCount = 0xFF000001;
	int i;
	msc_Return_TypeDef retStatus;

	/* Make the number of pages allocated accessible throughout the file. */
	numberOfPagesAllocated = numberOfPages;

	/* Initialize the address of each page */
	for (i = 0; i < numberOfPagesAllocated; i++) {
		pages[i].startAddress = (uint32_t *) (FLASH_SIZE - i * PAGE_SIZE
				- PAGE_SIZE);
		pages[i].endAddress = (uint32_t *) (FLASH_SIZE - i * PAGE_SIZE - 4);
	}

	/* Erase all pages allocated to the eeprom emulator*/
	for (i = numberOfPagesAllocated - 1; i >= 0; i--) {
		/* Validate if the page is already erased, and erase it if not. */
		if (!EE_validateIfErased(&pages[i])) {
			/* Erase the page, and return the status if the erase operation is unsuccessful. */
			retStatus = MSC_ErasePage(pages[i].startAddress);
			if (retStatus != mscReturnOk) {
				return false;
			}
		}
	}

	/* Page 0 is the active page. */
	activePageNumber = 0;

	/* There should be no receiving page. */
	receivingPageNumber = -1;

	/* Write erase count of 1 to the page 0 head. */
	retStatus = MSC_WriteWord(pages[activePageNumber].startAddress, &eraseCount,
			4);
	if (retStatus != mscReturnOk) {
		return false;
	}

	/* Set page status active to page 0. */
	retStatus = EE_setPageStatusActive(&pages[activePageNumber]);
	if (retStatus != mscReturnOk) {
		return false;
	}

	/** Successfully formatted pages */
	return true;
}

static msc_Return_TypeDef EE_TransferPage(EE_Variable_TypeDef *var,
		uint16_t writeData) {
	msc_Return_TypeDef retStatus;
	uint32_t *activeAddress;
	uint32_t *receivingAddress;
	bool newVariable;
	uint32_t eraseCount;

	/* If there is no receiving page predefined, set it to cycle through all allocated pages. */
	if (receivingPageNumber == -1) {
		receivingPageNumber = activePageNumber + 1;

		if (receivingPageNumber >= numberOfPagesAllocated) {
			receivingPageNumber = 0;
		}

		/* Check if the new receiving page really is erased. */
		if (!EE_validateIfErased(&pages[receivingPageNumber])) {
			/* If this page is not truly erased, it means that it has been written to
			 * from outside this API, this could be an address conflict. */
			EFM_ASSERT(0);
			MSC_ErasePage(pages[receivingPageNumber].startAddress);
		}
	}

	/* Set the status of the receiving page */
	EE_setPageStatusReceiving(&pages[receivingPageNumber]);

	/* If a variable was specified, write it to the receiving page */
	if (var != NULL) {
		EE_WriteToPage(&pages[receivingPageNumber], var->virtualAddress,
				writeData);
	}

	/* Start at the last word. */
	activeAddress = pages[activePageNumber].endAddress;

	/* Iterate through all words in the active page. Each time a new virtual
	 * address is found, write it and it's data to the receiving page */
	while (activeAddress > pages[activePageNumber].startAddress) {
		/* 0x0000 and 0xFFFF are not valid virtual addresses. */
		if ((uint16_t) (*activeAddress >> 16) == 0x0000
				|| (uint16_t) (*activeAddress >> 16) == 0xFFFF) {
			newVariable = false;
		}
		/* Omit when transfer is initiated from inside the EE_Init() function. */
		else if (var != NULL
				&& (uint16_t) (*activeAddress >> 16)
						> numberOfVariablesDeclared) {
			/* A virtual address outside the virtual address space, defined by the
			 * number of variables declared, are considered garbage. */
			newVariable = false;
		}

		else {
			receivingAddress = pages[receivingPageNumber].startAddress + 1;

			/* Start at the beginning of the receiving page. Check if the variable is
			 * already transfered. */
			while (receivingAddress <= pages[receivingPageNumber].endAddress) {
				/* Variable found, and is therefore already transferred. */
				if ((uint16_t) (*activeAddress >> 16)
						== (uint16_t) (*receivingAddress >> 16)) {
					newVariable = false;
					break;
				}
				/* Empty word found. All transferred variables are checked.  */
				else if (*receivingAddress == 0xFFFFFFFF) {
					newVariable = true;
					break;
				}
				receivingAddress++;
			}
		}

		if (newVariable) {
			/* Write the new variable to the receiving page. */
			EE_WriteToPage(&pages[receivingPageNumber],
					(uint16_t) (*activeAddress >> 16),
					(uint16_t) (*activeAddress));
		}
		activeAddress--;
	}

	/* Update erase count */
	eraseCount = EE_GetEraseCount();

	/* If a new page cycle is started, increment the erase count. */
	if (receivingPageNumber == 0)
		eraseCount++;

	/* Set the first byte, in this way the page status is not altered when the erase count is written. */
	eraseCount = eraseCount | 0xFF000000;

	/* Write the erase count obtained to the active page head. */
	retStatus = MSC_WriteWord(pages[receivingPageNumber].startAddress,
			&eraseCount, 4);
	if (retStatus != mscReturnOk) {
		return retStatus;
	}

	/* Erase the old active page. */
	retStatus = MSC_ErasePage(pages[activePageNumber].startAddress);
	if (retStatus != mscReturnOk) {
		return retStatus;
	}

	/* Set the receiving page to be the new active page. */
	retStatus = EE_setPageStatusActive(&pages[receivingPageNumber]);
	if (retStatus != mscReturnOk) {
		return retStatus;
	}

	activePageNumber = receivingPageNumber;
	receivingPageNumber = -1;

	return mscReturnOk;
}

bool EE_Init(uint32_t numberOfPages) {
	/* Make sure that the eeprom emulator is only initialized once. More that one
	 * initialization may result in undefined behavior. */
	EFM_ASSERT(!initialized);

	initialized = true;

	/* Number of pages must be at least 2. */
	if (numberOfPages < 2) {
		numberOfPages = DEFAULT_NUM_PAGES;
	}

	/* Make the number of pages allocated accessible throughout the file. */
	numberOfPagesAllocated = numberOfPages;

	/* Initialize the address of each page */
	uint32_t i;
	for (i = 0; i < numberOfPages; i++) {
		pages[i].startAddress = (uint32_t *) (FLASH_SIZE - i * PAGE_SIZE
				- PAGE_SIZE);
		pages[i].endAddress = (uint32_t *) (FLASH_SIZE - i * PAGE_SIZE - 4);
	}

	/* Check status of each page */
	for (i = 0; i < numberOfPages; i++) {
		switch (EE_getPageStatus(&pages[i])) {
		case eePageStatusActive:
			if (activePageNumber == -1) {
				activePageNumber = i;
			} else {
				/* More than one active page found. This is an invalid system state. */
				return false;
			}
			break;
		case eePageStatusReceiving:
			if (receivingPageNumber == -1) {
				receivingPageNumber = i;
			} else {
				/* More than one receiving page foudn. This is an invalid system state. */
				return false;
			}
			break;
		case eePageStatusErased:
			/* Validate if the page is really erased, and erase it if not. */
			if (!EE_validateIfErased(&pages[i])) {
				MSC_ErasePage(pages[i].startAddress);
			}
			break;
		default:
			/* Undefined page status, erase page. */
			MSC_ErasePage(pages[i].startAddress);
			break;
		}
	}

	/* No receiving or active page found. This is an invalid system state. */
	if (receivingPageNumber == -1 && activePageNumber == -1) {
		return false;
	}

	/* One active page only. All good. */
	if (receivingPageNumber == -1) {
		return true;
	}

	/* One receiving page only. */
	else if (activePageNumber == -1) {
		/* Set current receiving page as active. */
		activePageNumber = receivingPageNumber;
		receivingPageNumber = -1;
		EE_setPageStatusActive(&pages[receivingPageNumber]);
	}
	/* Found exactly one active and one receiving page. */
	else {
		/* Transfer variables from active to receiving page. */
		EE_TransferPage(NULL, 0);
	}

	/* Initialization completed successfully */
	return true;
}

bool EE_Read(EE_Variable_TypeDef *var, uint16_t *readData) {
	/* Make sure that the eeprom emulator is initialized. */
	EFM_ASSERT(initialized);

	uint32_t *address;

	address = (pages[activePageNumber].endAddress);

	/* 0x0000 and 0xFFFF are illegal addresses. */
	if (var->virtualAddress != 0x0000 && var->virtualAddress != 0xFFFF) {
		/* Iterate through the active page, starting from the end. */
		while (address > pages[activePageNumber].startAddress) {
			/* Check if the stored virtual address matches the one wanted. */
			if ((uint16_t) (*address >> 16) == var->virtualAddress) {
				/* Correct virtual address found, return the corresponding data. */
				*readData = (uint16_t) (*address);
				return true;
			}
			address--;
		}
	}
	/* Variable not found, return null value. */
	*readData = 0x0000;
	return false;
}

void EE_Write(EE_Variable_TypeDef *var, uint16_t writeData) {
	/* Make sure that the eeprom emulator is initialized. */
	EFM_ASSERT(initialized);

	/* Make sure that the virtual address is declared and valid. */
	EFM_ASSERT(var->virtualAddress <= numberOfVariablesDeclared);

	uint16_t readData;

	/* Check whether the variable already has a value associated to it. */
	if (EE_Read(var, &readData)) {
		/* Do not write if data is duplicate. */
		if (readData == writeData) {
			return;
		}
	}
	/* Write to flash. */
	if (!EE_WriteToPage(&pages[activePageNumber], var->virtualAddress,
			writeData)) {
		/* The write was not successful, which indicates that the active page is full. */
		EE_TransferPage(var, writeData);
	}
}

void EE_DeleteVariable(EE_Variable_TypeDef *var) {
	/* If the eeprom emulator is not initialized, this function has no meaning. */
	if (!initialized) {
		/* Halt for debug trace. */
		EFM_ASSERT(0);
		return;
	}

	uint32_t deleteData = 0x0000FFFF;

	uint32_t *address = (pages[activePageNumber].endAddress);

	/* Keep track if we actually removed a variable */bool varDeleted = false;

	/* Iterate through the active page from the end. */
	while (address > pages[activePageNumber].startAddress) {
		/* Write the virtual address 0x0000 to all instances of the chosen variable.
		 * Since 0x0000 is not a valid virtual address, the variable will not
		 * transferred to a new page on the next page transfer. */
		if ((uint16_t) (*address >> 16) == var->virtualAddress) {
			MSC_WriteWord(address, &deleteData, sizeof deleteData);
			varDeleted = true;
		}
		address--;
	}

	if (varDeleted) {
		numberOfActiveVariables--;
	}
}

bool EE_DeclareVariable(EE_Variable_TypeDef *var) {
	if (numberOfActiveVariables < MAX_ACTIVE_VARIABLES) {

		/* The virtual addresses are assigned according to the order of declaration. */
		var->virtualAddress = ++numberOfVariablesDeclared;

		numberOfActiveVariables++;

		return true;
	} else {
		return false;
	}
}

uint32_t EE_GetEraseCount(void) {
	/* Make sure that there is an active page */
	EFM_ASSERT(activePageNumber != -1);

	uint32_t eraseCount;

	/* The number of erase cycles is the 24 LSB of the first word of the active page. */
	eraseCount = (*(pages[activePageNumber].startAddress) & 0x00FFFFFF);

	/* if the page has never been erased, return 0. */
	if (eraseCount == 0xFFFFFF) {
		return 0;
	}

	return eraseCount;
}

void EEPROM_MyWrite(void) {
	/* Move the interrupt vector table to RAM to safely handle interrupts
	 * while performing write/erase operations on flash */
	moveInterruptVectorToRam();

	/* Enables the flash controller for writing. */
	MSC_Init();

	/* Initialize the eeprom emulator using 3 pages. */
	if (!EE_Init(3)) {

		/* If the initialization fails we have to take some measure
		 * to obtain a valid set of pages. In this example we simply
		 * format the pages */
		EE_Format(3);
	}

	EE_DeclareVariable(&tempup);
	EE_DeclareVariable(&templow);
	if (WRITE_EEPROM) {
		EE_Write(&tempup, UPPER_TEMP_LIMIT);
		EE_Write(&templow, LOWER_TEMP_LIMIT);
	}
	EE_Read(&tempup, &readtempup);
	EE_Read(&templow, &readtemplow);
}

int main(void) {
	CHIP_Init();

	blockSleepMode(EM2);
	CMU_HFRCOBandSet(cmuHFRCOBand_14MHz);
	CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
	#if !Debug
	#endif

	BSP_TraceSwoSetup();

	BLE_Configure();

	LEUART_Init();
	ADC_TempSetup();
	DMA_AllSetup();

	DMA_ActivateBasic(DMA_CHANNEL_RX, true, false, Receive,
			(void *) &(LEUART0 ->RXDATA), BUFF_RCV_LENGTH - 1);

	I2C_Initialize();

	gesture_init();

	proximity_int.fall(&proximity_interrupt1);
	proximity_int.rise(&proximity_interrupt2);

	capsense.start();
	capsense.attach_touch(touch);

	ACMP_Setup();

	EEPROM_MyWrite();

	LETIMER0_Init();
	blockSleepMode(EM2);
	while (1) {
		while (1) {
			INT_Disable();
			if (ADC_TransferActive) {
				EMU_EnterEM1();
			}
			INT_Enable();
			if (!ADC_TransferActive) {
				break;
			}
		}
		sleep();
	}
}