#ifndef PN532_NFC_H
#define PN532_NFC_H

#ifdef LINUX
	
#else
	#include "hal_uart.h"
#endif
//---------------------------------------------------------------------------
// Global Variables Declarations
//
#ifdef LINUX
	int fd;
#endif

#define NFC_SUCCESS 0
#define NFC_FAIL -1
#define RX_BUFFER_LEN 300
#define MI 0x20
//---------------------------------------------------------------------------
// Error Codes Definiions
// mode definition, only for Pn532Mode
#define STANDBY 0
#define LOWVBAT 1
#define POWERDOWN 2

//---------------------------------------------------------------------------
// Error Codes Definiions
//
#define ERROR_TIMEOUT					0x01
#define ERROR_CRC						0x02
#define ERROR_PARITY					0x03
#define ERROR_ERR_BIT_COUNT				0x04
#define ERROR_FRAMING					0x05
#define ERROR_BIT_COLLISION				0x06
#define ERROR_BUFFER_INSUFFICIENT		0x07
#define ERROR_RFBUF_OVERFLOW			0x09
#define ERROR_RFFIELD_NOT_SWITCHED_ON	0x0A
#define ERROR_RF_PROTOCOL				0x0B
#define ERROR_OVERHEATING				0x0D
#define ERROR_INTBUF_OVERFLOW			0x0E
#define ERROR_INVALID_PARAMETER			0x10
#define ERROR_COMMAND_NOT_SUPPORTED		0x12
#define ERROR_DATA_FORMAT_NOT_MATCHED	0x13
#define ERROR_AUTHENTICATION			0x14
#define ERROR_UID_CHECKBYTE_WRONG		0x23
#define ERROR_DEVICE_STATE_INVALID		0x25
#define ERROR_OP_NOT_ALLOWED			0x26
#define ERROR_COMMAND_NOT_ACCEPTABLE	0x27
#define ERROR_TARGET_RELEASED			0x29
#define ERROR_CARD_ID_NOT_MATCHED		0x2A
#define ERROR_CARD_DISAPPEARED			0x2B
#define ERROR_NFCID3_MISMATCH			0x2C
#define ERROR_OVERCURRENT				0x2D
#define ERROR_NAD_MISSING				0x2E

//---------------------------------------------------------------------------
// PN532 communication External References & Function Declarations:
// L4 functions
extern int PN532InitAsInitiator();
extern int PN532InitAsTarget();
extern int PN532TargetDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn);
extern int PN532InitiatorDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn);
extern void UARTcallback(unsigned char task_id, unsigned int events);

//---------------------------------------------------------------------------
// L0 functions
//
extern void nfcUARTOpen();
extern int UARTsend(unsigned char *pBuffer, int length);
extern int UARTreceive(unsigned char *pBuffer, int length);
extern int UARTflushRxBuf(void);

//---------------------------------------------------------------------------
// L1 functions
//
extern int PN532sendFrame(unsigned char* PData,unsigned int PDdataLEN);
extern int PN532sendNACKFrame(void);
extern int PN532sendACKFrame(void);
extern int PN532receiveFrame(unsigned char* Receive);
extern int PN532transceive(unsigned char* Input, int InputLen, unsigned char* Output);

//---------------------------------------------------------------------------
// L2 functions
// Miscellaneous Commands
extern int PN532diagnose(unsigned char NumTst, unsigned char* InParam,unsigned int InParamLen, unsigned char* OutParam);
extern int PN532getFirmwareVersion(unsigned char* OutParam);
extern int PN532getGeneralStatus(unsigned char* OutParam);
extern int PN532readRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL);
extern int PN532writeRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL);
extern int PN532readGPIO(unsigned char* GpioVal);
extern int PN532writeGPIO(unsigned char P3, unsigned char P7);
extern int PN532setSerialBaudRate(unsigned char BR);
extern int PN532setParameters(unsigned char Flags, unsigned char* OutParam);
extern int PN532SAMconfiguration(unsigned char Mode, unsigned char Timeout, unsigned char* IRQ);
extern int PN532powerDown(unsigned char WakeUpEnable, unsigned char* GenarateIRQ, unsigned char* OutParam);
extern int PN532RFConfiguration(unsigned char CfgItem, unsigned char* ConfigurationData, int CfgDataLen, unsigned char* OutParam);
extern int PN532RegulationTest(unsigned char TxMode);

//---------------------------------------------------------------------------
// Initiator Commands
//
extern int inJumpForDEP(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, unsigned char* NFCID3i, unsigned char* Gi,int GiLen, unsigned char* OutParam);
extern int inJumpForPSL(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, unsigned char* NFCID3i, unsigned char* Gi, unsigned int GiLen, unsigned char* OutParam);
extern int inListPassiveTarget(unsigned char MaxTg, unsigned char BrTy, unsigned char* InitiatorData);
extern int inATR(unsigned char Tg, unsigned char Next, unsigned char* NFCID3i, unsigned char* Gi);
extern int inPSL(unsigned char Tg, unsigned char BRit, unsigned char BRti);
extern int inDataExchange(unsigned char Tg, unsigned char* DataOut, unsigned int DataOutLen, unsigned char* OutParam);
extern int inCommunicateThru(unsigned char* DataOut);
extern int inDeselect(unsigned char Tg);
extern int inRelease(unsigned char Tg, unsigned char* OutParam);
extern int inSelect(unsigned char Tg);
extern int inAutoPoll(unsigned char PollNr, unsigned char Period, unsigned char* Type);

//---------------------------------------------------------------------------
// Target Commands
//
extern int tgInitAsTarget(unsigned char Mode, unsigned char* MifareParams, unsigned char* FeliCaParams, unsigned char* NFCID3t, unsigned char LENGt, unsigned char* Gt, unsigned char LENTk, unsigned char* Tk, unsigned char* OutParam);
extern int tgSetGeneralBytes(unsigned char* Gt);
extern int tgGetData(unsigned char* OutParam);
extern int tgSetData(unsigned char* DataOut, int DataOutLen, unsigned char* OutParam);
extern int tgSetMetaData(unsigned char* DataOut, int DataOutLen, unsigned char* OutParam);
extern int tgGetInitiatorCommand(void);
extern int tgResponseToInitiator(unsigned char* TgResponse);
extern int tgGetTargetStatus(unsigned char* OutParam);

#endif
