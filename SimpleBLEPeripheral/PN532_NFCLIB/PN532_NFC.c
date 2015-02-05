#include "PN532_NFC.h"
#include <string.h>

#ifdef LINUX
	#include <stdio.h>
	#include <stdlib.h>
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
#endif

//---------------------------------------------------------------------------
// Global Variables Definiions
//
unsigned char Pn532PowerMode = STANDBY;

//---------------------------------------------------------------------------
// level-3 functions
//

// PN532InitAsInitiator() Status: untested
// desc:		将PN532初始化为initiator
// input:		void
// output:	失败NFC_FAIL, 成功NFC_SUCCESS
int PN532InitAsInitiator(void){
	//TODO: test
	nfcUARTOpen();
	unsigned char *Output = NULL;
	int res = inJumpForDEP(0x01, 0x02, 0x00, NULL, NULL, NULL, 0, Output);
	if(res == NFC_FAIL){
		//low level error
		return NFC_FAIL;
	}else if(Output[0] != 0){
		//app level error
		return NFC_FAIL;
	}

	//deal with junks
	osal_mem_free(Output);
	return NFC_SUCCESS;
}

// PN532InitAsTarget() Status: untested
// desc:		将PN532初始化为target
// input:		void
// output:	失败NFC_FAIL, 成功NFC_SUCCESS
int PN532InitAsTarget(void){
	//TODO: test
	nfcUARTOpen();
	unsigned char *Output= NULL;
	unsigned char MifareParams[6] = {0, 0, 0, 0, 0, 0x40};
	unsigned char FelicaParams[18] = {0};
	unsigned char NFCID3t[10] = {0};
	int res = tgInitAsTarget(0, MifareParams, FelicaParams, NFCID3t, 0, NULL, 0, NULL, Output);
	if(res == NFC_FAIL){
		//low level error
		return NFC_FAIL;
	}else if(Output[0] == 0x7F){
		//syntax error
		return NFC_FAIL;
	}

	//deal with junks
	osal_mem_free(Output);
	return NFC_SUCCESS;
}

// PN532TargetDataExchange() Status: untested
// desc:	使用PN532作为target进行数据交换
//	input:	DataOut: 输出数据
//				DataOutLen: Input的长度
//				DataIn: 存放输入数据的容器, 长度应合适
//	output: 失败NFC_FAIL,成功则为DataIn的实际长度.
int PN532TargetDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn){
	//TODO: test
	unsigned char *OutputTemp = NULL;
	int DataOutLenRest = DataOutLen;
	int res = 0;
	int DataInPos = 0;

	//使用chaining mechanism接收数据
	res = tgGetData(OutputTemp);
	if(res == NFC_FAIL){//error handling
		//low level error
		return NFC_FAIL;
	}else if( (OutputTemp[0]&0x3F) != 0){
		//app level error
		return NFC_FAIL;
	}
	while( (OutputTemp[0]&MI) != 0){
		memcpy(&DataIn[DataInPos], &OutputTemp[1], 262);
		res = tgGetData(OutputTemp);
		if(res == NFC_FAIL){//error handling
			//low level error
			return NFC_FAIL;
		}else if( (OutputTemp[0]&0x3F) != 0){	//deal with junks
			osal_mem_free(OutputTemp);
			//app level error
			return NFC_FAIL;
		}
	}
	memcpy(&DataIn[DataInPos], &OutputTemp[1], res-1);
	DataInPos = DataInPos + res - 1;

	//使用chaining mechanism发送数据
	while(DataOutLenRest > 262){
		res = tgSetMetaData(&DataOut[DataOutLen - DataOutLenRest], 262, OutputTemp);
		if(res == NFC_FAIL){//error handling
			//low level error
			return NFC_FAIL;
		}else if( (OutputTemp[0]&0x3F) != 0){
			//app level error
			return NFC_FAIL;
		}
		DataOutLenRest = DataOutLenRest - 262;
	}
	res = tgSetData(&DataOut[DataOutLen - DataOutLenRest], DataOutLenRest, OutputTemp);
	if(res == NFC_FAIL){//error handling
		//low level error
		return NFC_FAIL;
	}else if( (OutputTemp[0]&0x3F) != 0){
		//app level error
		return NFC_FAIL;
	}

	//deal with junks
	osal_mem_free(OutputTemp);
	return DataInPos;
}

// PN532InitiatorDataExchange() Status: untested
// desc:	使用PN532作为target进行数据交换
//	input:	DataOut: 输出数据
//				DataOutLen: Input的长度
//				DataIn: 存放输入数据的容器, 长度应合适
//	output: 失败NFC_FAIL,成功则为DataIn的实际长度.
int PN532InitiatorDataExchange(unsigned char* DataOut, int DataOutLen, unsigned char* DataIn){
	//TODO: test
	unsigned char *OutputTemp = NULL;
	int DataOutLenRest = DataOutLen;
	int res = 0;
	int DataInPos = 0;

	//使用chaining mechanism发送数据
	while(DataOutLenRest > 262){
		res = inDataExchange(0x01 | MI, &DataOut[DataOutLen - DataOutLenRest], 262, OutputTemp);
		if(res == NFC_FAIL){//error handling
			//low level error
			return NFC_FAIL;
		}else if( (OutputTemp[0]&0x3F) != 0){
			//app level error
			return NFC_FAIL;
		}
		DataOutLenRest = DataOutLenRest - 262;
	}
	res = inDataExchange(0x01, &DataOut[DataOutLen - DataOutLenRest], DataOutLenRest, OutputTemp);
	if(res == NFC_FAIL){//error handling
		//low level error
		return NFC_FAIL;
	}else if( (OutputTemp[0]&0x3F) != 0){
		//app level error
		return NFC_FAIL;
	}

	//使用chaining mechanism接收数据
	while( (OutputTemp[0]&MI) != 0){
		memcpy(&DataIn[DataInPos], &OutputTemp[1], 262);
		DataInPos = DataInPos + 262;
		res = inDataExchange(0x01, NULL, 0, OutputTemp);
		if(res == NFC_FAIL){//error handling
			//low level error
			return NFC_FAIL;
		}else if( (OutputTemp[0]&0x3F) != 0){
			//app level error
			return NFC_FAIL;
		}
	}
	memcpy(&DataIn[DataInPos], &OutputTemp[1], res-1);
	DataInPos = DataInPos + res - 1;

	//deal with junks
	osal_mem_free(OutputTemp);
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

// UARTsend() Status:tested
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
#ifdef Debug
				printf("send error\n");
#endif
		return NFC_FAIL;
	}else if(temp != length){
#ifdef Debug
				printf("sent data less than required length\n");
#endif
		return NFC_FAIL;
	}
	else{
		return NFC_SUCCESS;
	}
#else
	//TODO: change the UART port
	temp = HalUARTWrite(HAL_UART_PORT_0, pBuffer, length);
	if(temp != length){
		return NFC_FAIL;
	}else{
		return NFC_SUCCESS;
	}
#endif
}

// UARTreceive() Status:tested
// desc:	receive data through UART.
// input:	unsigned char *pBuffer: the buffer of data to be received
//			int length: the length of bytes to be received
// output:	NFC_FAIL(-1) when failed and the bytes received when successed
int UARTreceive(unsigned char *pBuffer, int length){
	/*obsolete receive process
	unsigned char temp = 0;
	while(URX0IF == 0);
	temp = U0DBUF;
	URX0IF = 0;
	return temp;*/
	int temp = 0;
	osal_mem_free(pBuffer);
	pBuffer = osal_mem_alloc(length);
	if(pBuffer == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
#ifdef LINUX
	temp = read(fd, pBuffer, length);//读了2个但是pbuffer都是0
	if(temp <= 0){
#ifdef Debug
				printf("receive error\n");
#endif
		return NFC_FAIL;
	}else if(temp != length){
#ifdef Debug
				printf("received data less than required length\n");
#endif
		return NFC_FAIL;
	}
	else{
		return NFC_SUCCESS;
	}
#else
	//TODO: change the UART port
	temp = HalUARTRead(HAL_UART_PORT_0, pBuffer, length);
	if(temp != length){
		return NFC_FAIL;
	}else{
		return NFC_SUCCESS;
	}
#endif
}

// UARTflushRxBuf() Status:untested
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

// PN532sendFrame() Status:tested
// desc:	send information frame built from PData. pre and postambles are omitted.
// input:	PData:the data to be send, include TFI and PD; 
//			PDdataLEN:the length of PData.
// output:	NFC_SUCCESS when successed and NFC_FAIL when failed
int PN532sendFrame(unsigned char* PData,unsigned int PDdataLEN){
	unsigned char *Frame = NULL;
	//handle mode of PN532
	switch (Pn532PowerMode){
		case LOWVBAT: {
			// PN532 wakeup.
			// According to PN532 application note, C106 appendix: to go out Low Vbat mode and enter in normal mode we need to send a SAMConfiguration command
			unsigned char pn532_wakeup_outLVbat_preamble[26] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};
			int res = UARTsend(pn532_wakeup_outLVbat_preamble, 26);
			if(res == NFC_FAIL){
				return NFC_FAIL;
			}
			//receiving ACK and SAM info frame
			//short delay
			long int len = 0;
#ifdef LINUX
			usleep(5000);
#else
			DelayMs(5);
#endif
			//receive ACK frame
			len = PN532receiveFrame(Frame);
			if(len == NFC_FAIL){		//error handling
				return NFC_FAIL;
			}
			//check ACK
			if(Frame[0] != 0x00 || Frame[1] != 0xFF){
				return NFC_FAIL;
			}
			//receive info frame
			len = PN532receiveFrame(Frame);
			if(len == NFC_FAIL){		//error handling
				return NFC_FAIL;
			}
			Pn532PowerMode = STANDBY; // PN532 should now be awake
		}break;
		case POWERDOWN:{
			unsigned char pn532_wakeup_preamble[16] = { 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
			int res = UARTsend(pn532_wakeup_preamble, 16);
			if(res == NFC_FAIL){
				return NFC_FAIL;
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
			return NFC_FAIL;
		}
		//build frame
		Frame[0] = 0x00;
		Frame[1] = 0xFF;
		Frame[2] = PDdataLEN;
		LCS = 0 - PDdataLEN;
		Frame[3] = LCS;
		DCS = 0;
		//for(i = 0;i < PDdataLEN;i++){
		//	Frame[4+i] = PData[i];
		//	DCS = DCS - PData[i];
		//}
		Frame[4+PDdataLEN] = DCS;
		frameLen = 5+PDdataLEN;
	}else if(PDdataLEN <= 265){		//build extended Frame
		//allocate memory for built frame
		Frame = osal_mem_alloc(7+PDdataLEN+1);
		if(Frame == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
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
		return NFC_FAIL;
	}

	//send frame and deal with the junks
	osal_mem_free(Frame);
	return UARTsend(Frame, frameLen);
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

// PN532receiveFrame() Status:tested
// input:	Receive:the container of received data and should be inited with the length of 265
//				remember to use free(Receive) to release the memory when using malloc.
//				return 00 FF when receiving ACK frame,
//				return TF + PD when receiveing information frame,
//				return only error code when receiving error frame.
// output:	the actual length of the Receive for further use or NFC_FAIL when failed
int PN532receiveFrame(unsigned char* Receive){
	osal_mem_free(Receive);	//free the Receive buffer
	unsigned char *temp = NULL;
	int i = 0;		//for local count use
	unsigned char DCS = 0;
	int LEN = 0;
	
	if(NFC_FAIL == UARTreceive(temp, 3)){
		return NFC_FAIL;
	}
	
	if(temp[0] != 0x00 || temp[1] != 0x00 || temp[2] != 0xFF){
		//preamble not matched
		UARTflushRxBuf();
		return NFC_FAIL;
	}
	
	if(NFC_FAIL == UARTreceive(temp, 2)){//读了2个0进来
		return NFC_FAIL;
	}
	
	if(temp[0] == 0x00 && temp[1] == 0xFF){
		//receive ACK frame
		if(NFC_FAIL == UARTreceive(temp, 1)){
				//receive postamble
				return NFC_FAIL;
		}
		//allocate memory for Receive buffer
		Receive = osal_mem_alloc(2);
		if(Receive == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		Receive[0] = 0x00;
		Receive[1] = 0xFF;
		LEN = 2;
	}else if(temp[0] == 0xFF && temp[1] == 0xFF){
		//receive extended frame
		if(NFC_FAIL == UARTreceive(temp, 3)){	//LENm+LENl+LCS
			return NFC_FAIL;
		}
		//check LEN + LCS
		if(((unsigned char)(temp[0] + temp[1] + temp[2])) != 0x00){
			return NFC_FAIL;
		}
		LEN = temp[0]*256 + temp[1];
		//receive TFI and PD and DCS and postamble
		if(NFC_FAIL == UARTreceive(temp, LEN + 2)){
			return NFC_FAIL;
		}
		//check DCS
		for(i = 0; i < LEN; i++){
			DCS = DCS - temp[i];
		}
		if(DCS != temp[LEN]){
			return NFC_FAIL;
		}
		//allocate memory for Receive buffer
		Receive = osal_mem_alloc(LEN);
		if(Receive == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//successful received
		memcpy(Receive, temp, LEN);
	}else{
		//receive normal/error frame
		//temp[3] is LEN, temp[4] is LCS
		//check LEN + LCS
		if( ((unsigned char)(temp[0] + temp[1])) != 0){
			return NFC_FAIL;
		}
		LEN = temp[0];
		//receive TFI and PD and DCS and postamble
		if(NFC_FAIL == UARTreceive(temp, LEN + 2)){
			return NFC_FAIL;
		}
		//check DCS
		for(i = 0; i < LEN; i++){
			DCS = DCS - temp[i];
		}
		if(DCS != temp[LEN]){
			return NFC_FAIL;
		}
		//allocate memory for Receive buffer
		Receive = osal_mem_alloc(LEN);
		if(Receive == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//successful received
		memcpy(Receive, temp, LEN);
	}

	//deal with the junks
	osal_mem_free(temp);
	return LEN;	//return length of received data
}
// UARTtransceive() Status:tested
// desc :	data transfer. Input<->Output
// input:	Input: the data array to be transferred from host controller.
//			InputLen: the length of Input.
//			Output:the container of received data and should be inited with the length of 265
//				remember to use free(Output) to release the memory when using malloc.
//				return 00 FF when receiving ACK frame,
//				return TF + PD when receiveing information frame,
//				return only error code when receiving error frame.
// output:	the actual length of the Output for further use or NFC_FAIL when failed
int PN532transceive(unsigned char* Input, int InputLen, unsigned char* Output){
	//free the Output buffer
	osal_mem_free(Output);

	unsigned char *Receive = NULL;
	int len = 0;
	//send frame
	len = PN532sendFrame(Input, InputLen);
	if(len == NFC_FAIL){		//error handling
		return NFC_FAIL;
	}
	//short delay
#ifdef LINUX
	usleep(10000);
#else
	DelayMs(10);
#endif
	//receive ACK frame
	len = PN532receiveFrame(Receive);
	if(len == NFC_FAIL){		//error handling
		return NFC_FAIL;
	}
	//check ACK
	if(Receive[0] != 0x00 || Receive[1] != 0xFF){
		return NFC_FAIL;
	}
	//short delay
#ifdef LINUX
	sleep(1);
#else
	DelayMs(1000);
#endif
	//receive info frame
	len = PN532receiveFrame(Receive);
	if(len == NFC_FAIL){		//error handling
		return NFC_FAIL;
	}

	//load data into Output
	Output = Receive;
	return len;
}
// end of level-1 functions
//---------------------------------------------------------------------------
// level-2 functions
// the commands

//---------------------------------------------------------------------------
// Miscellaneous Commands
//

// 	PN532diagnose() Status: untested
//	input:	NumTst:the test code of diagnose; InParamLen:the length of InParam;
//			InParam: the input parameter of NumTst;
//			OutParam:the container of OutParam and should be inited with the length of 262.
//					remember to use free(InParam) to release the memory when using malloc.
//	output: the actual length of the OutParam.
int PN532diagnose(unsigned char NumTst, unsigned char* InParam,unsigned int InParamLen, unsigned char* OutParam){
	//TODO: test
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);

	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3 + InParamLen;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x00;
	PData[2] = NumTst;
	memcpy(&PData[3], InParam, InParamLen);

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	PN532getFirmwareVersion() Status:tested
//	input:	OutParam:
//					remember to use free(OutParam) to release the memory when using malloc.
//					returns are{IC Ver REV Support}.
//	output: the actual length of the OutParam for further use or the error code.
//					or NFC_FAIL when failed
int PN532getFirmwareVersion(unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x02;

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	PN532getGeneralStatus() Status:unimplemented
//	input:	OutParam:the container of OutParam
//					remember to use free(OutParam) to release the memory
//					returns are{Err Field NbTg [Tg1] [BrRx1] [BrTx1] [Type1]
//											   [Tg2] [BrRx2] [BrTx2] [Type2]
//											   [SAM status]}.
//	output: the actual length of the OutParam for further use.
int PN532getGeneralStatus(unsigned char* OutParam){
	//TODO: test
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x04;

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	PN532readRegister() Status:unimplemented
//	input:	ADDR:the 2byte address of the registers to be read.
//			ADDRLen:the length of ADDR.
//			VAL:the container of VAL and should be inited with the length of N.
//					N is the amount of registers to be read, the amount of ADDR.
//					remember to use free(VAL) to release the memory when using malloc.
//					returns are the value of registers in the order of ADDR.
//	output: the actual length of the VAL for further use.
//			or the error code.
int PN532readRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL){
	return NFC_SUCCESS;
}

// 	PN532writeRegister() Status:unimplemented
//	input:	ADDR:the 2byte address of the registers to be read.
//			ADDRLen:the length of ADDR.
//			VAL:the value of the ADDR. length should be ADDRLen.
//	output: NFC_SUCCESS(0) or the error code.
int PN532writeRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL){
	return NFC_SUCCESS;
}

// 	PN532readGPIO() Status:unimplemented
//	input:GpioVal:the container of value of gpios and should be inited with the length of 3.
//					remember to use free(GpioVal) to release the memory when using malloc.
//					returns are the value of P3 P7 I0I1.
//	output: the length of GpioVal (3) or the error code.
int PN532readGPIO(unsigned char* GpioVal){
	return NFC_SUCCESS;
}

// 	PN532writeGPIO() Status:unimplemented
//	input:	P3:please refer to the User Manual
//			P7:please refer to the User Manual
//	output: NFC_SUCCESS(0) or the error code.
int PN532writeGPIO(unsigned char P3, unsigned char P7){
	return NFC_SUCCESS;
}

// 	PN532setSerialBaudRate() Status:unimplemented
//	input:	BR:the baud rate of HSU.
//	output: NFC_SUCCESS(0) or the error code.
int PN532setSerialBaudRate(unsigned char BR){
	return NFC_SUCCESS;
}

// 	PN532setParameters() Status: untested
//	input:	Flags:the parameters to be set.
//				OutParam:the container of return valus and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the output please refer to the User Manual.
//	output: the actual length of OutParam.
int PN532setParameters(unsigned char Flags, unsigned char* OutParam){
	//TODO: test
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x12;
	PData[2] = Flags;

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	PN532SAMconfiguration() Status:unimplemented
//	input:	Mode:the way of using SAM.
//			Timeout:please refer to the User Manual.
//			IRQ:please refer to the User Manual.
//	output: NFC_SUCCESS(0) or the error code.
int PN532SAMconfiguration(unsigned char Mode, unsigned char Timeout, unsigned char* IRQ){
	return NFC_SUCCESS;
}

// 	PN532powerDown() Status: untested
//	input:	WakeUpEnable:please refer to the User Manual.
//				GenarateIRQ:please refer to the User Manual.
//				OutParam:the container of return valus and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the output please refer to the User Manual.
//	output: the actual length of OutParam.
int PN532powerDown(unsigned char WakeUpEnable, unsigned char* GenarateIRQ, unsigned char* OutParam){
	//TODO: test
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3;
	if(GenarateIRQ != NULL){
		PDataLen = PDataLen +1;
	}
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x16;
	PData[2] = WakeUpEnable;
	if(GenarateIRQ != NULL){
		PData[3] = *GenarateIRQ;
	}

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	PN532RFConfiguration() Status: untested
//	input:	CfgItem:please refer to the User Manual.
//				ConfigurationData:please refer to the User Manual.
//				CfgDataLen:the length of ConfigurationData.
//				OutParam:the container of return valus and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the output please refer to the User Manual.
//	output: the actual length of OutParam.
int PN532RFConfiguration(unsigned char CfgItem, unsigned char* ConfigurationData, int CfgDataLen, unsigned char* OutParam){
	//TODO: test
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3 + CfgDataLen;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x32;
	PData[2] = CfgItem;
	memcpy(&PData[3], ConfigurationData, CfgDataLen);

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	PN532RegulationTest() Status:unimplemented
//	input:	TxMode:please refer to the User Manual.
//	output: NFC_SUCCESS(0) because there are no output of this command
int PN532RegulationTest(unsigned char TxMode){
	return NFC_SUCCESS;
}
//---------------------------------------------------------------------------
// Initiator Commands
//

// 	inJumpForDEP() Status: untested
//	input:	ActPass:active or passive.
//			BR:106, 212 or 424kbps.
//			Next:indicate the next bytes. please refer to the User Manual.
//			PassiveInitiatorData: should be 4 bytes when BR = 106
//								  should be 5 bytes when BR = 212/424
//			NFCID3i:should be 10 bytes long.
//			Gi:general bytes.
//			GiLen: the length of Gi.
//			OutParam:the container of return valus and should be inited with the length of 66.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the output please refer to the User Manual.
//	output: the length of OutParam.
int inJumpForDEP(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, unsigned char* NFCID3i, unsigned char* Gi, int GiLen, unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
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
		return NFC_FAIL;
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
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	inJumpForPSL() Status:unimplemented
//	input:	ActPass:active or passive.
//			BR:106, 212 or 424kbps.
//			Next:indicate the next bytes. please refer to the User Manual.
//			PassiveInitiatorData: should be 4 bytes when BR = 106
//								  should be 5 bytes when BR = 212/424
//			NFCID3i:should be 10 bytes long.
//			Gi:general bytes.
//			GiLen: the length of Gi.
//			OutParam:the container of return vals and should be inited with the length of 65.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or the error code.
int inJumpForPSL(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, 	unsigned char* NFCID3i, unsigned char* Gi, unsigned int GiLen, unsigned char* OutParam){
	return NFC_SUCCESS;
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

// 	inDataExchange() Status: untested
//	input:	Tg:the target number. bit 6 is MI.
//			DataOut:the data to be transfer out.
//			DataOutLen:the length of DataOut.
//			OutParam:the container of return vals and should be inited with the length of 263.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the OutParam contains a status byte and DataIn bytes.
//	output: the actual length of OutParam.
int inDataExchange(unsigned char Tg, unsigned char* DataOut, unsigned int DataOutLen, unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = DataOutLen + 3;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x40;
	PData[2] = Tg;
	memcpy(&PData[3], DataOut, DataOutLen);


	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

int inCommunicateThru(unsigned char* DataOut){
	return NFC_SUCCESS;
}

int inDeselect(unsigned char Tg){
	return NFC_SUCCESS;
}

// 	inRelease() Status: untested
//	input:	Tg:the target number.
//				OutParam: the container of return vals and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the OutParam contains a status byte and DataIn bytes.
//	output: the length of OutParam(1).
int inRelease(unsigned char Tg, unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 3;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x52;
	PData[2] = Tg;

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
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

// 	tgInitAsTarget() Status: untested
//	input:	Mode: target mode. please refer to the User Manual.
//			MifareParams: should be 6 bytes long. please refer to the User Manual.
//			FeliCaParams: should be 18 bytes long. please refer to the User Manual.
//			NFCID3t: should be 10 bytes long. please refer to the User Manual.
//			LENGt: the length of Gt.
//			Gt: general bytes of target. (max. 47 bytes) please refer to the User Manual.
//			LENTk: the length of Tk.
//			Tk: (max. 48 bytes) please refer to the User Manual
//			OutParam:the container of return vals and should be inited with the length of 265.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam.
int tgInitAsTarget(unsigned char Mode, unsigned char* MifareParams, unsigned char* FeliCaParams, unsigned char* NFCID3t, unsigned char LENGt, unsigned char* Gt, unsigned char LENTk, unsigned char* Tk, unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 39 + LENGt + LENTk;
	int pointer = 38;			//array index for count use
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
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
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

int tgSetGeneralBytes(unsigned char* Gt){
	return NFC_SUCCESS;
}

// 	tgGetData() Status: untested
//	input:	OutParam:the container of return vals and should be inited with the length of 263.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgGetData(unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x86;
	
	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

// 	tgSetData() Status: untested
//	input:	DataOut: the data to be transfer out from the target;
//			DataOutLen: the length of DataOut;
//			OutParam:the container of return vals and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgSetData(unsigned char* DataOut, int DataOutLen, unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = DataOutLen + 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x8E;
	memcpy(&PData[2], DataOut, DataOutLen);

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

//tgSetMetaData() Status: untested
//	input:	DataOut: the data to be transfer out from the target;
//			DataOutLen: the length of DataOut;
//			OutParam:the container of return vals and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgSetMetaData(unsigned char* DataOut, int DataOutLen, unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = DataOutLen + 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x94;
	memcpy(&PData[2], DataOut, DataOutLen);

	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}

int tgGetInitiatorCommand(void){
	return NFC_SUCCESS;
}

int tgResponseToInitiator(unsigned char* TgResponse){
	return NFC_SUCCESS;
}

// 	tgGetTargetStatus() Status:tested
//	input:	OutParam:the container of return vals and should be inited with the length of 2.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgGetTargetStatus(unsigned char* OutParam){
	//make sure the OutParam is NULL
	osal_mem_free(OutParam);
	//build TFI + PData
	unsigned char *PData = NULL;
	int PDataLen = 2;
	PData = osal_mem_alloc(PDataLen);
	if(PData == NULL){
		//memory allocation unsuccess
		return NFC_FAIL;
	}
	PData[0] = 0xD4;
	PData[1] = 0x8A;
	
	//transmit data
	unsigned char *Receive = NULL;
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		OutParam = osal_mem_alloc(1);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		OutParam = osal_mem_alloc(ReceiveLen-2);
		if(OutParam == NULL){
			//memory allocation unsuccess
			return NFC_FAIL;
		}
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}

	//deal with junks
	osal_mem_free(PData);
	osal_mem_free(Receive);
	return OutParamLen;
}
// end of level-2 functions
