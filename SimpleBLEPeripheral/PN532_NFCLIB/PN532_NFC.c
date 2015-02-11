#include "PN532_NFC.h"
#include <string.h>

#ifdef LINUX
	#include <stdio.h>
	#include <stdlib.h>
	#include <time.h>
	#include <unistd.h>
	#include <sys/types.h>
	#include <sys/stat.h>
	#include <fcntl.h>
	#include <termios.h>
	#include <errno.h>

	void *osal_mem_alloc(unsigned int size){
		return malloc(size);
	}

	void osal_mem_free(void *ptr){
		free(ptr);
	}

	unsigned int osal_rand(void){
		srand((unsigned int)time(NULL));
		return (unsigned int)rand();
	}
#endif

//---------------------------------------------------------------------------
// Global Variables Definiions
//
unsigned char Pn532PowerMode = LOWVBAT;
int NfcRole = -1;
//---------------------------------------------------------------------------
// level-3 functions
//	随机机制

// NfcInit() Status: untested
// desc:		初始化PN532,随机初始化为initiator或者target
// input:		void
// output:	失败NFC_FAIL, 成功NFC_SUCCESS
int NfcInit(void){
	unsigned int rNumber = osal_rand();
	int res = 0;
	if(rNumber%2 == 0){
		res = PN532InitAsInitiator();
		if(res == 0){
			//error handling
			return NFC_FAIL;
		}
		NfcRole = INITIATOR;
	}else{
		res = PN532InitAsTarget();
		if(res == 0){
			//error handling
			return NFC_FAIL;
		}
		NfcRole = TARGET;
	}
	return NFC_SUCCESS;
}

// NfcDataExchange() Status: untested
// desc:	使用PN532进行数据交换, 依靠NfcInit初始化成的角色分别进行不同的数据交换
//	input:	DataOut: 输出数据
//				DataOutLen: Input的长度
//				DataIn: 存放输入数据的容器, 长度应合适, 内存分配由上级函数实现.
//	output: 失败NFC_FAIL,成功则为DataIn的实际长度.
int NfcDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn){
	switch(NfcRole){
	case INITIATOR:{
		return PN532InitiatorDataExchange(DataOut, DataOutLen, DataIn);
	}break;
	case TARGET:{
		return PN532TargetDataExchange(DataOut, DataOutLen, DataIn);
	}break;
	default:{
		//PN532 not initialized
	}
	}
	return NFC_FAIL;
}

// PN532InitAsInitiator() Status: tested
// desc:		将PN532初始化为initiator
// input:		void
// output:	失败NFC_FAIL, 成功NFC_SUCCESS
int PN532InitAsInitiator(void){
	//TODO: test
	nfcUARTOpen();
	retVal* res = inJumpForDEP(0x01, 0x02, 0x00, NULL, NULL, NULL, 0);
	if(res == (retVal*) NFC_FAIL){	//low level error
#ifdef LINUX
		printf("low level error\n");
#else
		HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
		return NFC_FAIL;
	}else if(res->Rcv[0] != 0){	//app level error
#ifdef LINUX
		printf("app level error\n");
#else
		HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
		osal_mem_free(res);
		return NFC_FAIL;
	}

	//deal with junks
	osal_mem_free(res);
	return NFC_SUCCESS;
}

// PN532InitAsTarget() Status: tested
// desc:		将PN532初始化为target
// input:		void
// output:	失败NFC_FAIL, 成功NFC_SUCCESS
int PN532InitAsTarget(void){
	//TODO: test
	nfcUARTOpen();
	unsigned char MifareParams[6] = {0, 0, 0, 0, 0, 0x40};
	unsigned char FelicaParams[18] = {0};
	unsigned char NFCID3t[10] = {0};
	retVal* res = tgInitAsTarget(0, MifareParams, FelicaParams, NFCID3t, 0, NULL, 0, NULL);
	if(res == (retVal*) NFC_FAIL){	//low level error
#ifdef LINUX
		printf("low level error\n");
#else
		HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
		return NFC_FAIL;
	}else if(res->Rcv[0] == 0x7F){	//syntax error
#ifdef LINUX
		printf("syntax error\n");
#else
		HalLcdWriteString( "syntax error", HAL_LCD_LINE_8 );
#endif
		osal_mem_free(res);
		return NFC_FAIL;
	}

	//deal with junks
	osal_mem_free(res);
	return NFC_SUCCESS;
}

// PN532TargetDataExchange() Status: tested
// desc:		使用PN532作为target进行数据交换
//	input:		DataOut: 输出数据
//					DataOutLen: Input的长度
//					DataIn: 存放输入数据的容器, 长度应合适, 内存分配由上级函数实现.
//	output: 	失败NFC_FAIL,成功则为DataIn的实际长度.
int PN532TargetDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn){
	//TODO: test
	int DataOutLenRest = DataOutLen;
	retVal* res = NULL;
	int DataInPos = 0;

	//使用chaining mechanism接收数据
	res = tgGetData();
	if(res == (retVal*) NFC_FAIL){//error handling
		//low level error
#ifdef LINUX
		printf("low level error\n");
#else
		HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
		return NFC_FAIL;
	}else if( (res->Rcv[0]&0x3F) != 0){
		//app level error
#ifdef LINUX
		printf("app level error\n");
#else
		HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
		osal_mem_free(res);
		return NFC_FAIL;
	}
	while( (res->Rcv[0]&MI) != 0){
		memcpy(&DataIn[DataInPos], &res->Rcv[1], 262);
		res = tgGetData();
		if(res == (retVal*) NFC_FAIL){//error handling
			//low level error
#ifdef LINUX
			printf("low level error\n");
#else
			HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
			return NFC_FAIL;
		}else if( (res->Rcv[0]&0x3F) != 0){
			//app level error
#ifdef LINUX
			printf("app level error\n");
#else
			HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			return NFC_FAIL;
		}
	}
	memcpy(&DataIn[DataInPos], &res->Rcv[1], res->length-1);
	DataInPos = DataInPos + res->length-1;

	//使用chaining mechanism发送数据
	while(DataOutLenRest > 262){
		res = tgSetMetaData(&DataOut[DataOutLen - DataOutLenRest], 262);
		if(res == (retVal*) NFC_FAIL){//error handling
			//low level error
#ifdef LINUX
			printf("low level error\n");
#else
			HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
			return NFC_FAIL;
		}else if( (res->Rcv[0]&0x3F) != 0){
			//app level error
#ifdef LINUX
			printf("app level error\n");
#else
			HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			return NFC_FAIL;
		}
		DataOutLenRest = DataOutLenRest - 262;
	}
	res = tgSetData(&DataOut[DataOutLen - DataOutLenRest], DataOutLenRest);
	if(res == (retVal*) NFC_FAIL){//error handling
		//low level error
#ifdef LINUX
		printf("low level error\n");
#else
		HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
		return NFC_FAIL;
	}else if( (res->Rcv[0]&0x3F) != 0){
		//app level error
#ifdef LINUX
		printf("app level error\n");
#else
		HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
		osal_mem_free(res);
		return NFC_FAIL;
	}

	//deal with junks
	osal_mem_free(res);
	return DataInPos;
}

// PN532InitiatorDataExchange() Status: tested
// desc:	使用PN532作为target进行数据交换
//	input:	DataOut: 输出数据
//				DataOutLen: Input的长度
//				DataIn: 存放输入数据的容器, 长度应合适
//	output: 失败NFC_FAIL,成功则为DataIn的实际长度.
int PN532InitiatorDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn){
	//TODO: test
	int DataOutLenRest = DataOutLen;
	retVal* res = NULL;
	int DataInPos = 0;

	//使用chaining mechanism发送数据
	while(DataOutLenRest > 262){
		res = inDataExchange(0x01 | MI, &DataOut[DataOutLen - DataOutLenRest], 262);
		if(res == (retVal*) NFC_FAIL){//error handling
			//low level error
#ifdef LINUX
			printf("low level error\n");
#else
			HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
			return NFC_FAIL;
		}else if( (res->Rcv[0]&0x3F) != 0){
			//app level error
#ifdef LINUX
			printf("app level error\n");
#else
			HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			return NFC_FAIL;
		}
		DataOutLenRest = DataOutLenRest - 262;
	}
	res = inDataExchange(0x01, &DataOut[DataOutLen - DataOutLenRest], DataOutLenRest);
	if(res == (retVal*) NFC_FAIL){//error handling
		//low level error
#ifdef LINUX
		printf("low level error\n");
#else
		HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
		return NFC_FAIL;
	}else if( (res->Rcv[0]&0x3F) != 0){
		//app level error
#ifdef LINUX
		printf("app level error\n");
#else
		HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
		osal_mem_free(res);
		return NFC_FAIL;
	}

	//使用chaining mechanism接收数据
	while( (res->Rcv[0]&MI) != 0){
		memcpy(&DataIn[DataInPos], &res->Rcv[1], 262);
		DataInPos = DataInPos + 262;
		res = inDataExchange(0x01, NULL, 0);
		if(res == (retVal*) NFC_FAIL){//error handling
			//low level error
#ifdef LINUX
			printf("low level error\n");
#else
			HalLcdWriteString( "low level error", HAL_LCD_LINE_8 );
#endif
			return NFC_FAIL;
		}else if( (res->Rcv[0]&0x3F) != 0){
			//app level error
#ifdef LINUX
			printf("app level error\n");
#else
			HalLcdWriteString( "app level error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			return NFC_FAIL;
		}
	}
	memcpy(&DataIn[DataInPos], &res->Rcv[1], res->length-1);
	DataInPos = DataInPos + res->length - 1;

	//deal with junks
	osal_mem_free(res);
	return DataInPos;
}

// DelayMs() Status: untested
// desc:		进行毫秒级延时,仅用于CC254x平台.
//	input:		msec: 延时毫秒数
//	output: 	void.
void DelayMs(unsigned int msec){
	unsigned int i,j;
	for(i=0; i<msec; i++)
		for(j=0; j<535; j++);
}

void UARTcallback(unsigned char task_id, unsigned int events){
	//void
}
// end of level-4 functions
//---------------------------------------------------------------------------
// level0 functions
// hardware-oriented functions
// replace registers with HAL functions

// nfcUARTOpen() Status: tested
// desc:	init and open UART.
// input:	void
// output:	void
void nfcUARTOpen(){
	/* obsolete init process
	PERCFG &= ~0x01;	//peripheral control set USART0 I/O location alt.1
	//gpio configuration
	P0SEL |= 0x0C;		//USART0 pin peripheral selection
						//P0_3 TX P0_2 RX
	P2DIR &= ~0xC0;		//P0 PRIP0 = 00 -> USART0 first
	//uart0 configuration
	U0CSR |= 0x80;		//UART MODE
						//Receive disabled
	U0UCR = 0x02;		//no flow control, no parity, 8bits,
						//1 stop bit, low start bit, high stop bit
	U0GCR = 0x0B;		//LSB first, BAUD_E = 11
	U0BAUD = 0xD8;		//BAUD_M = 216,	baud rate 115200
	//interrupt setting
	EA = 0;				//disable global interrupt
	UTX0IF = 0;			//clear int flag
	URX0IF = 0;
	URX0IE = 0;			//disable USART0 RX int	
						//doesn't need RX interrupt
	EA = 1;				//Enable global interrupt
	U0CSR |= 0x40;		//receive enable*/
#ifdef LINUX
	fd = open("/dev/ttyUSB0",O_RDWR | O_NOCTTY | O_NDELAY);
	if(fd == -1){
		perror("unable to open ttyUSB0");
	}
	struct termios Opt;
	tcgetattr(fd, &Opt);
	cfsetispeed(&Opt, B115200);// Baud rate at 115200
	cfsetospeed(&Opt, B115200);
	Opt.c_cflag &= ~PARENB;//8bits, no parity, one stop bit
	Opt.c_cflag &= ~CSTOPB;
	Opt.c_cflag &= ~CSIZE;
	Opt.c_cflag |= CS8;
	Opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//raw mode
	Opt.c_oflag &= ~OPOST;
	tcsetattr(fd, TCSANOW, &Opt);
#else
	halUARTCfg_t halUARTCfg;
	halUARTCfg.baudRate = HAL_UART_BR_115200;
	halUARTCfg.flowControl = FALSE;
	halUARTCfg.callBackFunc =  (halUARTCBack_t)UARTcallback;
	HalUARTInit();
	//TODO: change the UART port
	HalUARTOpen(HAL_UART_PORT_0, &halUARTCfg);
#endif
}

// UARTsend() Status: tested
// desc:	send data through UART.
// input:	unsigned char *pBuffer: the buffer of data to be sent
//				int length: the length of pBuffer
// output:	NFC_FAIL(-1) when failed and NFC_SUCCESS(0) when successed.
int UARTsend(unsigned char *pBuffer, int length){
	/*obsolete send process
	U0DBUF = dat;
	while(UTX0IF == 0);
	UTX0IF = 0;*/
	int temp = 0;
#ifdef LINUX
	temp = write(fd, pBuffer, length);
	if(temp < 0){
		printf("send error\n");
		return NFC_FAIL;
	}else if(temp != length){
		printf("sent data less than required length\n");
		return NFC_FAIL;
	}
	else{
		return NFC_SUCCESS;
	}
#else
	//TODO: change the UART port
	temp = HalUARTWrite(HAL_UART_PORT_0, pBuffer, length);
	if(temp != length){
		HalLcdWriteString( "HalUARTWriteError", HAL_LCD_LINE_3 );
		return NFC_FAIL;
	}else{
		return NFC_SUCCESS;
	}
#endif
}

// UARTreceive() Status: tested
// desc:	receive data through UART.
// input:	length: the length of bytes to be received
// output:	NFC_FAIL(-1) when failed and struct retVal when successed
retVal* UARTreceive(int length){
	int temp = 0;
	retVal* RetVal = (retVal*) osal_mem_alloc ( sizeof(retVal) + length );
	if(RetVal == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("retValMemAllocError\n");
#else
		HalLcdWriteString( "retValMemAllocError", HAL_LCD_LINE_3 );
#endif
		goto err;
	}
#ifdef LINUX
	temp = read(fd, RetVal->Rcv, length);
	if(temp <= 0){	//error handling
		printf("receive error\n");
		goto err;
	}else if(temp != length){
		printf("received data less than required length\n");
		goto err;
	}
#else
	//TODO: change the UART port
	temp = HalUARTRead(HAL_UART_PORT_0, RetVal->Rcv, length);
	if(temp != length){	//error handling
		HalLcdWriteString( "HalUARTReadError", HAL_LCD_LINE_3 );
		goto err;
	}
#endif

	RetVal->length = length;
	return RetVal;

err:
	//deal with junks
	osal_mem_free(RetVal);
	return (retVal*) NFC_FAIL;
}

// UARTflushRxBuf() Status: untested
// desc:	flush the RX buffer
// input:	void
// output:	NFC_SUCCESS when successed and NFC_FAIL when failed
int UARTflushRxBuf(void){
#ifdef LINUX
	unsigned char buf[RX_BUFFER_LEN] = {0};
	read(fd,buf,RX_BUFFER_LEN);
	return NFC_SUCCESS;
#else
	unsigned char buf[RX_BUFFER_LEN] = {0};
	HalUARTRead(HAL_UART_PORT_0,buf,RX_BUFFER_LEN);
	return NFC_SUCCESS;
#endif
}
// end of level-0 functions
//---------------------------------------------------------------------------
// level-1 functions
// basic implement of the Host controller communication protocol

// PN532sendFrame() Status: tested
// desc:	send information frame built from PData. pre and postambles are omitted.
// input:	PData:the data to be send, include TFI and PD; 
//			PDdataLEN:the length of PData.
// output:	NFC_SUCCESS when successed and NFC_FAIL when failed
int PN532sendFrame(unsigned char* PData,unsigned int PDdataLEN){
	unsigned char *Frame = NULL;
	retVal* RetVal = NULL;
	int res = 0;
	//handle mode of PN532
	switch (Pn532PowerMode){
		case LOWVBAT: {
			// PN532 wakeup.
			// According to PN532 application note, C106 appendix: to go out Low Vbat mode and enter in normal mode we need to send a SAMConfiguration command
			unsigned char pn532_wakeup_outLVbat_preamble[26] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};
			res = UARTsend(pn532_wakeup_outLVbat_preamble, 26);
			if(res == NFC_FAIL){
#ifdef LINUX
		printf("LOWVBSendError\n");
#else
		HalLcdWriteString( "LOWVBSendError", HAL_LCD_LINE_5 );
#endif
				goto err;
			}
			//receiving ACK and SAM info frame
			//short delay
#ifdef LINUX
			usleep(5000);
#else
			DelayMs(5);
#endif
			//receive ACK frame
			RetVal = PN532receiveFrame();
			if(RetVal == (retVal*) NFC_FAIL){		//error handling
#ifdef LINUX
				printf("LOWVBAckRcvError\n");
#else
				HalLcdWriteString( "LOWVBAckRcvError", HAL_LCD_LINE_5 );
#endif
				//deal with the junks
				osal_mem_free(Frame);
				return NFC_FAIL;
			}
			//check ACK
			if(RetVal->Rcv[0] != 0x00 || RetVal->Rcv[1] != 0xFF){
#ifdef LINUX
		printf("LOWVBAckError\n");
#else
		HalLcdWriteString( "LOWVBAckError", HAL_LCD_LINE_5 );
#endif
				goto err;
			}
			//receive info frame
			osal_mem_free(RetVal);
			RetVal = PN532receiveFrame();
			if(RetVal == (retVal*) NFC_FAIL){		//error handling
#ifdef LINUX
				printf("LOWVBInfoRcvError\n");
#else
				HalLcdWriteString( "LOWVBInfoRcvError", HAL_LCD_LINE_5 );
#endif
				//deal with the junks
				osal_mem_free(Frame);
				return NFC_FAIL;
			}
			Pn532PowerMode = STANDBY; // PN532 should now be awake
		}break;
		case POWERDOWN:{
			unsigned char pn532_wakeup_preamble[16] = { 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
			int res = UARTsend(pn532_wakeup_preamble, 16);
			if(res == NFC_FAIL){
#ifdef LINUX
		printf("PowDownSendError\n");
#else
		HalLcdWriteString( "PowDownSendError", HAL_LCD_LINE_5 );
#endif
				goto err;
			}
			Pn532PowerMode = STANDBY; // PN532 should now be awake
		}	break;
		case STANDBY:
			// Nothing to do :)
			break;
	};
	//flush the ACK and SAMconfigure answer frames
	//UARTflushRxBuf();
	//build frame
	int i = 0;unsigned char LCS, DCS;
	unsigned int frameLen = 0;
	if(PDdataLEN <= 0xFF){			//build normal Frame
		//allocate memory for built frame
		Frame = osal_mem_alloc(4+PDdataLEN+1);
		if(Frame == NULL){
			//memory allocation unsuccess
#ifdef LINUX
		printf("FrameMemAllocError\n");
#else
		HalLcdWriteString( "FrameMemAllocError", HAL_LCD_LINE_5 );
#endif
			goto err;
		}
		//build frame
		Frame[0] = 0x00;
		Frame[1] = 0xFF;
		Frame[2] = PDdataLEN;
		LCS = 0 - PDdataLEN;
		Frame[3] = LCS;
		DCS = 0;
		for(i = 0;i < PDdataLEN;i++){
			Frame[4+i] = PData[i];
			DCS = DCS - PData[i];
		}
		Frame[4+PDdataLEN] = DCS;
		frameLen = 5+PDdataLEN;
	}else if(PDdataLEN <= 265){		//build extended Frame
		//allocate memory for built frame
		Frame = osal_mem_alloc(7+PDdataLEN+1);
		if(Frame == NULL){
			//memory allocation unsuccess
#ifdef LINUX
		printf("FrameMemAllocError\n");
#else
		HalLcdWriteString( "FrameMemAllocError", HAL_LCD_LINE_5 );
#endif
			goto err;
		}
		//build frame
		Frame[0] = 0x00;
		Frame[1] = 0xFF;
		Frame[2] = 0xFF;
		Frame[3] = 0xFF;
		Frame[4] = (unsigned char) ((PDdataLEN>>8) & 0x00FF);
		Frame[5] = (unsigned char) (PDdataLEN & 0x00FF);
		LCS = 0 - (unsigned char) (PDdataLEN & 0x00FF) - (unsigned char) ((PDdataLEN>>8) & 0x00FF);	//LCS
		Frame[6] = LCS;
		DCS = 0;
		for(i = 0;i < PDdataLEN;i++){
			Frame[7+i] = PData[i];
			DCS = DCS - PData[i];
		}
		Frame[7+PDdataLEN] = DCS;
		frameLen = 5+PDdataLEN;
	}else{							//return error
#ifdef LINUX
		printf("PDdataLENIllegal\n");
#else
		HalLcdWriteString( "PDdataLENIllegal", HAL_LCD_LINE_5 );
#endif
		goto err;
	}
	res = UARTsend(Frame, frameLen);
	if(res == NFC_FAIL){
		//Frame send error
#ifdef LINUX
		printf("FrameSendError\n");
#else
		HalLcdWriteString( "FrameSendError", HAL_LCD_LINE_5 );
#endif
		goto err;
	}

	//deal with the junks
	osal_mem_free(Frame);
	osal_mem_free(RetVal);
	return NFC_SUCCESS;

err:
	//deal with the junks
	osal_mem_free(Frame);
	osal_mem_free(RetVal);
	return NFC_FAIL;
}

// PN532sendNACKFrame() Status:unimplemented
// desc:	send a NACK frame.
// input:	void.
// output:	NFC_SUCCESS(0) or the error code.
int PN532sendNACKFrame(void){
	return NFC_SUCCESS;
}

// PN532sendACKFrame() Status:untested
// desc:	send a ACK frame.
// input:	void.
// output:	NFC_SUCCESS(0) or the error code.
int PN532sendACKFrame(void){
	unsigned char Frame[6] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
	//send
	return UARTsend(Frame, 6);
}

// PN532receiveFrame() Status: tested
// input:		void
// output: 	NFC_FAIL when failed and struct retVal when succeed
retVal* PN532receiveFrame(void){
	retVal *temp = NULL;
	retVal *RetVal = NULL;
	int i = 0;		//for local count use
	unsigned char DCS = 0;
	int LEN = 0;

	temp = UARTreceive(3);
	if(temp == (retVal*) NFC_FAIL){	//error handling
#ifdef LINUX
		printf("PreambleRcvError\n");
#else
		HalLcdWriteString( "PreambleRcvError", HAL_LCD_LINE_4 );
#endif
		return (retVal*) NFC_FAIL;
	}
	
	if(temp->Rcv[0] != 0x00 || temp->Rcv[1] != 0x00 || temp->Rcv[2] != 0xFF){
		//preamble not matched
#ifdef LINUX
		printf("PreambleNotMatched\n");
#else
		HalLcdWriteString( "PreambleNotMatched", HAL_LCD_LINE_4 );
#endif
		UARTflushRxBuf();
		osal_mem_free(temp);
		return (retVal*) NFC_FAIL;
	}

	osal_mem_free(temp);
	temp = UARTreceive(2);
	if(temp == (retVal*) NFC_FAIL){	//error handling
		//LEN LCS receive error
#ifdef LINUX
		printf("LENLCSRcvError\n");
#else
		HalLcdWriteString( "LENLCSRcvError", HAL_LCD_LINE_4 );
#endif
		return (retVal*) NFC_FAIL;
	}
	
	if(temp->Rcv[0] == 0x00 && temp->Rcv[1] == 0xFF){
		//receive ACK frame

		//receive postamble
		osal_mem_free(temp);
		temp = UARTreceive(1);
		if(temp == (retVal*) NFC_FAIL){	//error handling
#ifdef LINUX
			printf("AckPostambleRcvError\n");
#else
			HalLcdWriteString( "AckPostambleRcvError", HAL_LCD_LINE_4 );
#endif
			return (retVal*) NFC_FAIL;
		}
		//allocate memory for Receive buffer
		RetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + 2);
		if(RetVal == NULL){
			//memory allocation unsuccess
#ifdef LINUX
			printf("AckReceiveMemAllocError\n");
#else
			HalLcdWriteString( "AckReceiveMemAllocError", HAL_LCD_LINE_4 );
#endif
			osal_mem_free(temp);
			return (retVal*) NFC_FAIL;
		}
		RetVal->Rcv[0] = 0x00;
		RetVal->Rcv[1] = 0xFF;
		RetVal->length = 2;
	}else if(temp->Rcv[0] == 0xFF && temp->Rcv[1] == 0xFF){
		//receive extended frame
		//LENm+LENl+LCS
		osal_mem_free(temp);
		temp = UARTreceive(3);
		if(temp == (retVal*) NFC_FAIL){	//error handling
#ifdef LINUX
			printf("ExLENmLENlLCSRcvError\n");
#else
			HalLcdWriteString( "ExLENmLENlLCSRcvError", HAL_LCD_LINE_4 );
#endif
			return (retVal*) NFC_FAIL;
		}
		//check LEN + LCS
		if(((unsigned char)(temp->Rcv[0] + temp->Rcv[1] + temp->Rcv[2])) != 0x00){
			//LEN + LCS error
#ifdef LINUX
			printf("ExLEN+LCSError\n");
#else
			HalLcdWriteString( "ExLEN+LCSError", HAL_LCD_LINE_4 );
#endif
			UARTflushRxBuf();
			osal_mem_free(temp);
			return (retVal*) NFC_FAIL;
		}
		LEN = temp->Rcv[0]*256 + temp->Rcv[1];

		//receive TFI and PD and DCS and postamble
		osal_mem_free(temp);
		temp = UARTreceive(LEN + 2);
		if(temp == (retVal*) NFC_FAIL){
#ifdef LINUX
			printf("ExTFIPDDCSPostambleRcvError\n");
#else
			HalLcdWriteString( "ExTFIPDDCSPostambleRcvError", HAL_LCD_LINE_4 );
#endif
			return (retVal*) NFC_FAIL;
		}
		//check DCS
		for(i = 0; i < LEN; i++){
			DCS = DCS - temp->Rcv[i];
		}
		if(DCS != temp->Rcv[LEN]){
			//DCS not matched
#ifdef LINUX
			printf("ExDCSError\n");
#else
			HalLcdWriteString( "ExDCSError", HAL_LCD_LINE_4 );
#endif
			UARTflushRxBuf();
			osal_mem_free(temp);
			return (retVal*) NFC_FAIL;
		}
		//allocate memory for Receive buffer
		RetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + LEN);
		if(RetVal == NULL){	//memory allocation unsuccess
#ifdef LINUX
			printf("ExReceiveMemAllocError\n");
#else
			HalLcdWriteString( "ExReceiveMemAllocError", HAL_LCD_LINE_4 );
#endif
			osal_mem_free(temp);
			return (retVal*) NFC_FAIL;
		}
		//successful received
		memcpy(RetVal->Rcv, temp->Rcv, LEN);
		RetVal->length = LEN;
	}else{
		//receive normal/error frame
		//temp[3] is LEN, temp[4] is LCS
		//check LEN + LCS
		if( ((unsigned char)(temp->Rcv[0] + temp->Rcv[1])) != 0){
			//LEN + LCS error
#ifdef LINUX
			printf("NormLEN+LCSError\n");
#else
			HalLcdWriteString( "NormLEN+LCSError", HAL_LCD_LINE_4 );
#endif
			UARTflushRxBuf();
			osal_mem_free(temp);
			return (retVal*) NFC_FAIL;
		}
		LEN = temp->Rcv[0];
		//receive TFI and PD and DCS and postamble
		osal_mem_free(temp);
		temp = UARTreceive(LEN + 2);
		if(temp == (retVal*) NFC_FAIL){	//TFI PD DCS postamble receive error
#ifdef LINUX
			printf("NormTFIPDDCSPostambleRcvError\n");
#else
			HalLcdWriteString( "NormTFIPDDCSPostambleRcvError", HAL_LCD_LINE_4 );
#endif
			return (retVal*) NFC_FAIL;
		}
		//check DCS
		for(i = 0; i < LEN; i++){
			DCS = DCS - temp->Rcv[i];
		}
		if(DCS != temp->Rcv[LEN]){
			//DCS error
#ifdef LINUX
			printf("NormDCSError\n");
#else
			HalLcdWriteString( "NormDCSError", HAL_LCD_LINE_4 );
#endif
			UARTflushRxBuf();
			osal_mem_free(temp);
			return (retVal*) NFC_FAIL;
		}
		//allocate memory for Receive buffer
		RetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + LEN);
		if(RetVal == NULL){	//memory allocation unsuccess
#ifdef LINUX
			printf("NormReceiveMemAllocError\n");
#else
			HalLcdWriteString( "NormReceiveMemAllocError", HAL_LCD_LINE_4 );
#endif
			return (retVal*) NFC_FAIL;
		}
		//successful received
		memcpy(RetVal->Rcv, temp->Rcv, LEN);
		RetVal->length = LEN;
	}

	//deal with the junks
	osal_mem_free(temp);
	return RetVal;
}
// UARTtransceive() Status: tested
// desc :	data transfer. Input<->Output
// input:	Input: the data array to be transferred from host controller.
//				InputLen: the length of Input.
// output:	NFC_FAIL when failed or struct retVal when succeed
//					return 00 FF when receiving ACK frame,
//					return TF + PD when receiveing information frame,
//					return only error code when receiving error frame.
retVal* PN532transceive(unsigned char* Input, int InputLen){
	retVal* Receive = NULL;
	retVal* RetVal = NULL;
	int res = 0;

	//send frame
	res = PN532sendFrame(Input, InputLen);
	if(res == NFC_FAIL){
		//error handling
#ifdef LINUX
		printf("SendFrameError\n");
#else
		HalLcdWriteString( "SendFrameError", HAL_LCD_LINE_6 );
#endif
		return (retVal*) NFC_FAIL;
	}

	//short delay
#ifdef LINUX
	usleep(10000);
#else
	DelayMs(10);
#endif

	//receive ACK frame
	Receive = PN532receiveFrame();
	if(Receive == (retVal*) NFC_FAIL){	//error handling
#ifdef LINUX
		printf("AckRcvError\n");
#else
		HalLcdWriteString( "AckRcvError", HAL_LCD_LINE_6 );
#endif
		return (retVal*) NFC_FAIL;
	}

	//check ACK
	if(Receive->Rcv[0] != 0x00 || Receive->Rcv[1] != 0xFF){	//ack error
#ifdef LINUX
		printf("AckError\n");
#else
		HalLcdWriteString( "AckError", HAL_LCD_LINE_6 );
#endif
		return (retVal*) NFC_FAIL;
	}

	//short delay
#ifdef LINUX
	sleep(1);
#else
	DelayMs(1000);
#endif

	//receive info frame
	osal_mem_free(Receive);
	Receive = PN532receiveFrame();
	if(Receive == (retVal*) NFC_FAIL){	//error handling
#ifdef LINUX
		printf("InfoRcvError\n");
#else
		HalLcdWriteString( "InfoRcvError", HAL_LCD_LINE_6 );
#endif
		return (retVal*) NFC_FAIL;
	}

	//load info frame into Output
	if(Receive->Rcv[0] == 0x7F){
		RetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + 1);
		if(RetVal == NULL){	//memory allocation unsuccess
#ifdef LINUX
			printf("OutpMemoAllocError\n");
#else
			HalLcdWriteString( "OutpMemoAllocError", HAL_LCD_LINE_6 );
#endif
			return (retVal*) NFC_FAIL;
		}
		//syntax error
		RetVal->Rcv[0] = 0x7F;
		RetVal->length = 1;
	}else{
		RetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + Receive->length-2);
		if(RetVal == NULL){	//memory allocation unsuccess
#ifdef LINUX
		printf("OutpMemoAllocError\n");
#else
		HalLcdWriteString( "OutpMemoAllocError", HAL_LCD_LINE_6 );
#endif
		return (retVal*) NFC_FAIL;
		}
		//info frame
		memcpy(RetVal->Rcv, &Receive->Rcv[2], Receive->length-2);
		RetVal->length = Receive->length-2;
	}

	//deal with junks
	osal_mem_free(Receive);
	return RetVal;
}
// end of level-1 functions
//---------------------------------------------------------------------------
// level-2 functions
// the commands

//---------------------------------------------------------------------------
// Miscellaneous Commands
//

// 	PN532diagnose() Status: tested
//	input:		NumTst:the test code of diagnose; InParamLen:the length of InParam;
//					InParam: the input parameter of NumTst;
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* PN532diagnose(unsigned char NumTst, unsigned char* InParam,unsigned int InParamLen){
	//TODO: test
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3 + InParamLen;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x00;
	PData[2] = NumTst;
	memcpy(&PData[3], InParam, InParamLen);

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// PN532getFirmwareVersion() Status: tested
//	input:		void
//	output: 	NFC_FAIL when failed or struct retVal when succeed
//					returns are{IC Ver REV Support}.
retVal* PN532getFirmwareVersion(void){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x02;

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// PN532getGeneralStatus() Status: tested
//	input:		void
//	output: 	NFC_FAIL when failed or struct retVal when succeed
//					returns are{Err Field NbTg [Tg1] [BrRx1] [BrTx1] [Type1]
//											   [Tg2] [BrRx2] [BrTx2] [Type2]
//											   [SAM status]}.
retVal* PN532getGeneralStatus(void){
	//TODO: test
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x04;

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// PN532readRegister() Status: unimplemented
//	input:		ADDR:the 2byte address of the registers to be read.
//					ADDRLen:the length of ADDR.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
//					please refer to the UM for the returned values
retVal* PN532readRegister(int* ADDR, unsigned int ADDRLen){
	return NFC_SUCCESS;
}

// PN532writeRegister() Status: unimplemented
//	input:	ADDR:the 2byte address of the registers to be read.
//			ADDRLen:the length of ADDR.
//			VAL:the value of the ADDR. length should be ADDRLen.
//	output: NFC_SUCCESS(0) or the error code.
int PN532writeRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL){
	return NFC_SUCCESS;
}

// PN532readGPIO() Status: unimplemented
//	input:		void.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* PN532readGPIO(void){
	return NFC_SUCCESS;
}

// PN532writeGPIO() Status: unimplemented
//	input:		P3:please refer to the User Manual
//					P7:please refer to the User Manual
//	output: 	NFC_SUCCESS(0) or the error code.
int PN532writeGPIO(unsigned char P3, unsigned char P7){
	return NFC_SUCCESS;
}

// PN532setSerialBaudRate() Status: unimplemented
//	input:		BR:the baud rate of HSU.
//	output: 	NFC_SUCCESS(0) or the error code.
int PN532setSerialBaudRate(unsigned char BR){
	return NFC_SUCCESS;
}

// PN532setParameters() Status: untested
//	input:		Flags:the parameters to be set.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* PN532setParameters(unsigned char Flags){
	//TODO: test
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x12;
	PData[2] = Flags;

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// PN532SAMconfiguration() Status:unimplemented
//	input:		Mode:the way of using SAM.
//					Timeout:please refer to the User Manual.
//					IRQ:please refer to the User Manual.
//	output: 	NFC_SUCCESS(0) or the error code.
int PN532SAMconfiguration(unsigned char Mode, unsigned char Timeout, unsigned char* IRQ){
	return NFC_SUCCESS;
}

// PN532powerDown() Status: untested
//	input:		WakeUpEnable:please refer to the User Manual.
//					GenarateIRQ:please refer to the User Manual.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* PN532powerDown(unsigned char WakeUpEnable, unsigned char* GenarateIRQ){
	//TODO: test
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3;
	if(GenarateIRQ != NULL){
		PDataLen = PDataLen +1;
	}
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){	//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x16;
	PData[2] = WakeUpEnable;
	if(GenarateIRQ != NULL){
		PData[3] = *GenarateIRQ;
	}

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// PN532RFConfiguration() Status: untested
//	input:		CfgItem:please refer to the User Manual.
//					ConfigurationData:please refer to the User Manual.
//					CfgDataLen:the length of ConfigurationData.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* PN532RFConfiguration(unsigned char CfgItem, unsigned char* ConfigurationData, int CfgDataLen){
	//TODO: test
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3 + CfgDataLen;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x32;
	PData[2] = CfgItem;
	memcpy(&PData[3], ConfigurationData, CfgDataLen);

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// PN532RegulationTest() Status: unimplemented
//	input:		TxMode:please refer to the User Manual.
//	output: 	NFC_SUCCESS(0) because there are no output of this command
int PN532RegulationTest(unsigned char TxMode){
	return NFC_SUCCESS;
}
//---------------------------------------------------------------------------
// Initiator Commands
//

// inJumpForDEP() Status: untested
//	input:		ActPass:active or passive.
//					BR:106, 212 or 424kbps.
//					Next:indicate the next bytes. please refer to the User Manual.
//					PassiveInitiatorData: should be 4 bytes when BR = 106
//														  should be 5 bytes when BR = 212/424
//					NFCID3i:should be 10 bytes long.
//					Gi:general bytes.
//					GiLen: the length of Gi.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* inJumpForDEP(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, unsigned char* NFCID3i, unsigned char* Gi, int GiLen){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 5;
	int i = 0;	//counter of PData index
	if(Next & 0x01){
		if(BR == 0x00){
			PDataLen = PDataLen + 4;
		}else{
			PDataLen = PDataLen + 5;
		}
	}
	if(Next & 0x02){
		PDataLen = PDataLen + 10;
	}
	if(Next & 0x04){
		PDataLen = PDataLen + GiLen;
	}
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x56;
	PData[2] = ActPass;
	PData[3] = BR;
	PData[4] = Next;
	i = 5;
	if(Next & 0x01){
		if(BR == 0x00){
			memcpy(&PData[i], PassiveInitiatorData, 4);
			i = i + 4;
		}else{
			memcpy(&PData[i], PassiveInitiatorData, 5);
			i = i + 5;
		}
	}
	if(Next & 0x02){
		memcpy(&PData[i], NFCID3i, 10);
		i = i + 10;
	}
	if(Next & 0x04){
		memcpy(&PData[i], Gi, GiLen);
		i = i + GiLen;
	}

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// inJumpForPSL() Status:unimplemented
//	input:		ActPass:active or passive.
//					BR:106, 212 or 424kbps.
//					Next:indicate the next bytes. please refer to the User Manual.
//					PassiveInitiatorData: should be 4 bytes when BR = 106
//								 						 should be 5 bytes when BR = 212/424
//					NFCID3i:should be 10 bytes long.
//					Gi:general bytes.
//					GiLen: the length of Gi.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* inJumpForPSL(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, 	unsigned char* NFCID3i, unsigned char* Gi, unsigned int GiLen){
	return (retVal*) NFC_FAIL;
}

int inListPassiveTarget(unsigned char MaxTg, unsigned char BrTy, unsigned char* InitiatorData){
	return NFC_SUCCESS;
}

int inATR(unsigned char Tg, unsigned char Next, unsigned char* NFCID3i, unsigned char* Gi){
	return NFC_SUCCESS;
}

int inPSL(unsigned char Tg, unsigned char BRit, unsigned char BRti){
	return NFC_SUCCESS;
}

// inDataExchange() Status: untested
//	input:		Tg:the target number. bit 6 is MI.
//					DataOut:the data to be transfer out.
//					DataOutLen:the length of DataOut.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* inDataExchange(unsigned char Tg, unsigned char* DataOut, unsigned int DataOutLen){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = DataOutLen + 3;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x40;
	PData[2] = Tg;
	memcpy(&PData[3], DataOut, DataOutLen);

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

int inCommunicateThru(unsigned char* DataOut){
	return NFC_SUCCESS;
}

int inDeselect(unsigned char Tg){
	return NFC_SUCCESS;
}

// inRelease() Status: untested
//	input:		Tg:the target number.
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* inRelease(unsigned char Tg){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x52;
	PData[2] = Tg;

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

int inSelect(unsigned char Tg){
	return NFC_SUCCESS;
}

int inAutoPoll(unsigned char PollNr, unsigned char Period, unsigned char* Type){
	return NFC_SUCCESS;
}
//---------------------------------------------------------------------------
// Target Commands
//

// tgInitAsTarget() Status: untested
//	input:		Mode: target mode. please refer to the User Manual.
//					MifareParams: should be 6 bytes long. please refer to the User Manual.
//					FeliCaParams: should be 18 bytes long. please refer to the User Manual.
//					NFCID3t: should be 10 bytes long. please refer to the User Manual.
//					LENGt: the length of Gt.
//					Gt: general bytes of target. (max. 47 bytes) please refer to the User Manual.
//					LENTk: the length of Tk.
//					Tk: (max. 48 bytes) please refer to the User Manual
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* tgInitAsTarget(unsigned char Mode, unsigned char* MifareParams, unsigned char* FeliCaParams, unsigned char* NFCID3t, unsigned char LENGt, unsigned char* Gt, unsigned char LENTk, unsigned char* Tk){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 39 + LENGt + LENTk;
	int pointer = 38;			//array index for count use
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x8C;
	PData[2] = Mode;
	memcpy(&PData[3], MifareParams, 6);
	memcpy(&PData[9], FeliCaParams, 18);
	memcpy(&PData[27], NFCID3t, 10);
	PData[37] = LENGt;
	if(LENGt != 0){
		memcpy(&PData[pointer], Gt, LENGt);
		pointer = pointer + LENGt;
	}
	PData[pointer] = LENTk;
	pointer++;
	if(LENTk != 0){
		memcpy(&PData[pointer], Tk, LENTk);
	}

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

int tgSetGeneralBytes(unsigned char* Gt){
	return NFC_SUCCESS;
}

// tgGetData() Status: untested
//	input:		void
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* tgGetData(void){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x86;
	
	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// tgSetData() Status: untested
//	input:		DataOut: the data to be transfer out from the target;
//					DataOutLen: the length of DataOut;
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* tgSetData(unsigned char* DataOut, int DataOutLen){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = DataOutLen + 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x8E;
	memcpy(&PData[2], DataOut, DataOutLen);

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

// tgSetMetaData() Status: untested
//	input:		DataOut: the data to be transfer out from the target;
//					DataOutLen: the length of DataOut;
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* tgSetMetaData(unsigned char* DataOut, int DataOutLen){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = DataOutLen + 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x94;
	memcpy(&PData[2], DataOut, DataOutLen);

	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;
}

int tgGetInitiatorCommand(void){
	return NFC_SUCCESS;
}

int tgResponseToInitiator(unsigned char* TgResponse){
	return NFC_SUCCESS;
}

// tgGetTargetStatus() Status: untested
//	input:		void
//	output: 	NFC_FAIL when failed or struct retVal when succeed
retVal* tgGetTargetStatus(void){
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
#ifdef LINUX
		printf("PDataMemoAllocError\n");
#else
		HalLcdWriteString( "PDataMemoAllocError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}
	PData[0] = 0xD4;
	PData[1] = 0x8A;
	
	//transmit data
	retVal* OutParam = NULL;
	OutParam = PN532transceive(PData, PDataLen);
	if(OutParam == (retVal*) NFC_FAIL){	//transcevie error
#ifdef LINUX
		printf("TransceiveError\n");
#else
		HalLcdWriteString( "TransceiveError", HAL_LCD_LINE_7 );
#endif
		goto err;
	}

	//deal with junks
	osal_mem_free(PData);
	return OutParam;

err:
	//deal with junks
	osal_mem_free(PData);
	return (retVal*) NFC_FAIL;

}
// end of level-2 functions--------------------------------------------------------------------
