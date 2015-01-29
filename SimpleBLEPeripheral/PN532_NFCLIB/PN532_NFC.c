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
#endif

//---------------------------------------------------------------------------
// Global Variables Definiions
//
unsigned char Pn532PowerMode = STANDBY;
//---------------------------------------------------------------------------
// level-3 functions
//
void inInitNFC(){
	
}

void tgInitNFC(){
	
}

void UARTcallback(unsigned char task_id, unsigned int events){
	
}
// end of level-4 functions
//---------------------------------------------------------------------------
// level0 functions
// hardware-oriented functions
// replace registers with HAL functions

// nfcUARTOpen() Status:tested
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
	HalUARTOpen(HAL_UART_PORT_0, &halUARTCfg);
#endif
}

// UARTsend() Status:tested
// desc:	send data through UART.
// input:	unsigned char *pBuffer: the buffer of data to be sent
//			int length: the length of pBuffer
// output:	NFC_FAIL(-1) when failed and the bytes sent when successed
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
	temp = HalUARTWrite(HAL_UART_PORT_0, pBuffer, length);
	if(temp == 0){
		return NFC_FAIL;
	}else{
		return temp;
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
	temp = HalUARTRead(HAL_UART_PORT_0, pBuffer, length)
	if(temp == 0){
		return NFC_FAIL;
	}else{
		return temp;
	}
#endif
}

// UARTflushRxBuf() Status:untested
// desc:	flush the RX buffer
// input:	void
// output:	NFC_SUCCESS when successed and NFC_FAIL when failed
int UARTflushRxBuf(void){
#ifdef LINUX
	unsigned char buf[300] = {0};
	read(fd,buf,300);
	return NFC_SUCCESS;
#else
	unsigned char buf[HAL_UART_ISR_RX_MAX] = {0};
	HalUARTRead(HAL_UART_PORT_0,buf,HAL_UART_ISR_RX_MAX);
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
	unsigned char Frame[273] = {0};
	//handle mode of PN532
	switch (Pn532PowerMode){
		case LOWVBAT: {
			// PN532 wakeup.
			// According to PN532 application note, C106 appendix: to go out Low Vbat mode and enter in normal mode we need to send a SAMConfiguration command
			unsigned char pn532_wakeup_outLVbat_preamble[26] = {0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x17, 0x00};
			int res = UARTsend(pn532_wakeup_outLVbat_preamble, 26);
			if(res == NFC_FAIL){
#ifdef Debug
				printf("error sending SAM\n");
#endif
				return NFC_FAIL;
			}
			//receiving ACK and SAM info frame
			//short delay
			long int len = 0;
#ifdef LINUX
			usleep(5000);
#else
			//TODO: short delay
#endif
			//receive ACK frame
			len = PN532receiveFrame(Frame);
			if(len == NFC_FAIL){		//error handling
#ifdef Debug
				printf("error receiving SAM ACK\n");
#endif
				return NFC_FAIL;
			}
			//check ACK
			if(Frame[0] != 0x00 || Frame[1] != 0xFF){
#ifdef Debug
				printf("error checking SAM ACK\n");
#endif
				return NFC_FAIL;
			}
			//receive info frame
			len = PN532receiveFrame(Frame);
			if(len == NFC_FAIL){		//error handling
#ifdef Debug
				printf("error receiving SAM info\n");
#endif
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
	//send frame
	return UARTsend(Frame, frameLen);
}

// PN532sendNACKFrame() Status:unimplemented
// desc:	send a NACK frame.
// input:	void.
// output:	NFC_SUCCESS(0) or the error code.
unsigned int PN532sendNACKFrame(void){
	return NFC_SUCCESS;
}

// PN532sendACKFrame() Status:untested
// desc:	send a ACK frame.
// input:	void.
// output:	NFC_SUCCESS(0) or the error code.
unsigned int PN532sendACKFrame(void){
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
	unsigned char temp[275] = {0};
	int i = 0;		//for local count use
	unsigned char DCS = 0;
	int LEN = 0;
	
	if(NFC_FAIL == UARTreceive(temp, 3)){
#ifdef Debug
		printf("no start code received\n");
#endif
		return NFC_FAIL;
	}
	
	if(temp[0] != 0x00 || temp[1] != 0x00 || temp[2] != 0xFF){
#ifdef Debug
		printf("start code doesn't match\n");
#endif
		//preamble not matched
		UARTflushRxBuf();
		return NFC_FAIL;
	}
	
	if(NFC_FAIL == UARTreceive(&temp[3], 1)){//读了2个0进来
#ifdef Debug
		printf("error receiving LEN\n");
#endif
		return NFC_FAIL;
	}
	if(NFC_FAIL == UARTreceive(&temp[4], 1)){
#ifdef Debug
		printf("error receiving LCS\n");
#endif
			return NFC_FAIL;
	}
	
	if(temp[3] == 0x00 && temp[4] == 0xFF){
		//receive ACK frame
		if(NFC_FAIL == UARTreceive(&temp[5], 1)){
#ifdef Debug
		printf("error receiving ACK postamble\n");
#endif
				//receive postamble
				return NFC_FAIL;
		}
		Receive[0] = 0x00;
		Receive[1] = 0xFF;
		LEN = 2;
	}else if(temp[3] == 0xFF && temp[4] == 0xFF){
		//receive extended frame
		if(NFC_FAIL == UARTreceive(&temp[5], 3)){	//LENm+LENl+LCS
#ifdef Debug
		printf("error receiving LENm+LENl+LCS in extended frame\n");
#endif
			return NFC_FAIL;
		}
		//check LEN + LCS
		if(((unsigned char)(temp[5] + temp[6] + temp[7])) != 0x00){
#ifdef Debug
		printf("LEN+LCS checksum error\n");
#endif
			return NFC_FAIL;
		}
		LEN = temp[5]*256 + temp[6];
		//receive TFI and PD and postamble
		if(NFC_FAIL == UARTreceive(&temp[8], LEN + 2)){
#ifdef Debug
		printf("error receiving TFI+PD in extended frame\n");
#endif
			return NFC_FAIL;
		}
		//check DCS
		for(i = 0; i < LEN; i++){
			DCS = DCS - temp[8+i];
		}
		if(DCS != temp[8 + LEN]){
#ifdef Debug
		printf("DCS checksum error\n");
#endif
			return NFC_FAIL;
		}
		//successful received
		memcpy(Receive, &temp[8], LEN);
	}else{
		//receive normal/error frame
		//temp[3] is LEN, temp[4] is LCS
		//check LEN + LCS
		if( ((unsigned char)(temp[3] + temp[4])) != 0){
#ifdef Debug
		printf("LEN+LCS checksum error\n");
#endif
			return NFC_FAIL;
		}
		LEN = temp[3];
		//receive TFI and PD and postamble
		if(NFC_FAIL == UARTreceive(&temp[5], LEN + 2)){
#ifdef Debug
		printf("error receiving TFI+PD in normal/error frame\n");
#endif
			return NFC_FAIL;
		}
		//check DCS
		for(i = 0; i < LEN; i++){
			DCS = DCS - temp[5+i];
		}
		if(DCS != temp[5 + LEN]){
#ifdef Debug
		printf("DCS checksum error\n");
#endif
			return NFC_FAIL;
		}
		//successful received
		memcpy(Receive, &temp[5], LEN);
	}
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
	unsigned char Receive[275] = {0};
	int len = 0;
	//send frame
	len = PN532sendFrame(Input, InputLen);
	if(len == NFC_FAIL){		//error handling
#ifdef Debug
		printf("error sending command\n");
#endif
		return NFC_FAIL;
	}
	//short delay
#ifdef LINUX
	usleep(10000);
#else
	//TODO: short delay
#endif
	//receive ACK frame
	len = PN532receiveFrame(Receive);
	if(len == NFC_FAIL){		//error handling
#ifdef Debug
		printf("error receiving command ACK\n");
#endif
		return NFC_FAIL;
	}
	//check ACK
	if(Receive[0] != 0x00 || Receive[1] != 0xFF){
#ifdef Debug
		printf("error checking command ACK\n");
#endif
		return NFC_FAIL;
	}
	//short delay
#ifdef LINUX
	sleep(1);
#else
	//TODO: short delay
#endif
	//receive info frame
	len = PN532receiveFrame(Receive);
	if(len == NFC_FAIL){		//error handling
#ifdef Debug
		printf("error receiving command info\n");
#endif
		return NFC_FAIL;
	}
	//load data into Output
	memcpy(Output, Receive, len);
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
	//build TFI + PData
	unsigned char PData[265] = {0xD4, 0x00, 0x00};
	PData[2] = NumTst;
	memcpy(&PData[3], InParam, InParamLen);
	int PDataLen = 3 + InParamLen;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

// 	PN532getFirmwareVersion() Status:tested
//	input:	OutParam:the container of OutParam and should be inited with the length of 4.
//					remember to use free(OutParam) to release the memory when using malloc.
//					returns are{IC Ver REV Support}.
//	output: the actual length of the OutParam for further use or the error code.
//					or NFC_FAIL when failed
int PN532getFirmwareVersion(unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[2] = {0xD4, 0x02};
	int PDataLen = 2;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

// 	PN532getGeneralStatus() Status:unimplemented
//	input:	OutParam:the container of OutParam and should be inited with the length of 12.
//					remember to use free(OutParam) to release the memory when using malloc.
//					returns are{Err Field NbTg [Tg1] [BrRx1] [BrTx1] [Type1]
//											   [Tg2] [BrRx2] [BrTx2] [Type2]
//											   [SAM status]}.
//	output: the actual length of the OutParam for further use.
int PN532getGeneralStatus(unsigned char* OutParam){
	//TODO: test
	//build TFI + PData
	unsigned char PData[2] = {0xD4, 0x04};
	int PDataLen = 2;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
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
unsigned int PN532readRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL){
	return NFC_SUCCESS;
}

// 	PN532writeRegister() Status:unimplemented
//	input:	ADDR:the 2byte address of the registers to be read.
//			ADDRLen:the length of ADDR.
//			VAL:the value of the ADDR. length should be ADDRLen.
//	output: NFC_SUCCESS(0) or the error code.
unsigned int PN532writeRegister(int* ADDR, unsigned int ADDRLen, unsigned char* VAL){
	return NFC_SUCCESS;
}

// 	PN532readGPIO() Status:unimplemented
//	input:GpioVal:the container of value of gpios and should be inited with the length of 3.
//					remember to use free(GpioVal) to release the memory when using malloc.
//					returns are the value of P3 P7 I0I1.
//	output: the length of GpioVal (3) or the error code.
unsigned int PN532readGPIO(unsigned char* GpioVal){
	return NFC_SUCCESS;
}

// 	PN532writeGPIO() Status:unimplemented
//	input:	P3:please refer to the User Manual
//			P7:please refer to the User Manual
//	output: NFC_SUCCESS(0) or the error code.
unsigned int PN532writeGPIO(unsigned char P3, unsigned char P7){
	return NFC_SUCCESS;
}

// 	PN532setSerialBaudRate() Status:unimplemented
//	input:	BR:the baud rate of HSU.
//	output: NFC_SUCCESS(0) or the error code.
unsigned int PN532setSerialBaudRate(unsigned char BR){
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
	//build TFI + PData
	unsigned char PData[3] = {0xD4, 0x12, 0x00};
	PData[2] = Flags;
	int PDataLen = 3;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
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
	//build TFI + PData
	unsigned char PData[4] = {0xD4, 0x16, 0x00, 0x00};
	PData[2] = WakeUpEnable;
	int PDataLen = 3;
	if(GenarateIRQ != NULL){
		memcpy(&PData[3], GenarateIRQ, 1);
		PDataLen = PDataLen +1;
	}

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
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
	//build TFI + PData
	unsigned char PData[14] = {0xD4, 0x32};
	PData[2] = CfgItem;
	int PDataLen = 3;
	memcpy(&PData[3], ConfigurationData, CfgDataLen);
	PDataLen = PDataLen +CfgDataLen;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

// 	PN532RegulationTest() Status:unimplemented
//	input:	TxMode:please refer to the User Manual.
//	output: NFC_SUCCESS(0) because there are no output of this command
unsigned int PN532RegulationTest(unsigned char TxMode){
	return NFC_SUCCESS;
}
//---------------------------------------------------------------------------
// Initiator Commands
//

// 	inJumpForDEP() Status: tested
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
	//build TFI + PData
	unsigned char PData[68] = {0};
	PData[0] = 0xD4;
	PData[1] = 0x56;
	PData[2] = ActPass;
	PData[3] = BR;
	PData[4] = Next;
	int PDataLen = 5;
	if(Next & 0x01){
		if(BR == 0x00){
			memcpy(&PData[5], PassiveInitiatorData, 4);
			PDataLen = PDataLen + 4;
		}else{
			memcpy(&PData[5], PassiveInitiatorData, 5);
			PDataLen = PDataLen + 5;
		}
	}
	if(Next & 0x02){
		memcpy(&PData[PDataLen], NFCID3i, 10);
		PDataLen = PDataLen + 10;
	}
	if(Next & 0x04){
		memcpy(&PData[PDataLen], Gi, GiLen);
		PDataLen = PDataLen + GiLen;
	}

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
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
unsigned int inJumpForPSL(unsigned char ActPass, unsigned char BR, unsigned char Next, unsigned char* PassiveInitiatorData, 	unsigned char* NFCID3i, unsigned char* Gi, unsigned int GiLen, unsigned char* OutParam){
	return NFC_SUCCESS;
}

unsigned int inListPassiveTarget(unsigned char MaxTg, unsigned char BrTy, unsigned char* InitiatorData){
	return NFC_SUCCESS;
}

unsigned int inATR(unsigned char Tg, unsigned char Next, unsigned char* NFCID3i, unsigned char* Gi){
	return NFC_SUCCESS;
}

unsigned int inPSL(unsigned char Tg, unsigned char BRit, unsigned char BRti){
	return NFC_SUCCESS;
}

// 	inDataExchange() Status: tested
//	input:	Tg:the target number. bit 6 is MI.
//			DataOut:the data to be transfer out.
//			DataOutLen:the length of DataOut.
//			OutParam:the container of return vals and should be inited with the length of 263.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the OutParam contains a status byte and DataIn bytes.
//	output: the actual length of OutParam.
int inDataExchange(unsigned char Tg, unsigned char* DataOut, unsigned int DataOutLen, unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[265] = {0};
	PData[0] = 0xD4;
	PData[1] = 0x40;
	PData[2] = Tg;
	memcpy(&PData[3], DataOut, DataOutLen);
	int PDataLen = DataOutLen + 3;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

unsigned int inCommunicateThru(unsigned char* DataOut){
	return NFC_SUCCESS;
}

unsigned int inDeselect(unsigned char Tg){
	return NFC_SUCCESS;
}

// 	inRelease() Status: tested
//	input:	Tg:the target number.
//				OutParam: the container of return vals and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the OutParam contains a status byte and DataIn bytes.
//	output: the length of OutParam(1).
int inRelease(unsigned char Tg, unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[3] = {0};
	PData[0] = 0xD4;
	PData[1] = 0x52;
	PData[2] = Tg;
	int PDataLen = 3;

	//transmit data
	unsigned char Receive[3] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}

	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

unsigned int inSelect(unsigned char Tg){
	return NFC_SUCCESS;
}

unsigned int inAutoPoll(unsigned char PollNr, unsigned char Period, unsigned char* Type){
	return NFC_SUCCESS;
}
//---------------------------------------------------------------------------
// Target Commands
//

// 	tgInitAsTarget() Status:tested
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
	//build TFI + PData
	unsigned char PData[134] = {0};
	PData[0] = 0xD4;
	PData[1] = 0x8C;
	PData[2] = Mode;
	memcpy(&PData[3], MifareParams, 6);
	memcpy(&PData[9], FeliCaParams, 18);
	memcpy(&PData[27], NFCID3t, 10);
	PData[37] = LENGt;
	unsigned char pointer = 38;			//array index for count use
	if(LENGt != 0){
		memcpy(&PData[pointer], Gt, LENGt);
		pointer = pointer + LENGt;
	}
	PData[pointer] = LENTk;
	pointer++;
	if(LENTk != 0){
		memcpy(&PData[pointer], Tk, LENTk);
	}
	int PDataLen = 39 + LENGt + LENTk;
	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}
	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

unsigned int tgSetGeneralBytes(unsigned char* Gt){
	return NFC_SUCCESS;
}

// 	tgGetData() Status:tested
//	input:	OutParam:the container of return vals and should be inited with the length of 263.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgGetData(unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[2] = {0xD4, 0x86};
	
	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, 2, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}
	
	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

// 	tgSetData() Status:tested
//	input:	DataOut: the data to be transfer out from the target;
//			DataOutLen: the length of DataOut;
//			OutParam:the container of return vals and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgSetData(unsigned char* DataOut, int DataOutLen, unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[264] = {0};
	PData[0] = 0xD4;
	PData[1] = 0x8E;
	memcpy(&PData[2], DataOut, DataOutLen);
	int PDataLen = DataOutLen + 2;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}
	
	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

//tgSetMetaData() Status:untested
//	input:	DataOut: the data to be transfer out from the target;
//			DataOutLen: the length of DataOut;
//			OutParam:the container of return vals and should be inited with the length of 1.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgSetMetaData(unsigned char* DataOut, int DataOutLen, unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[264] = {0};
	PData[0] = 0xD4;
	PData[1] = 0x94;
	memcpy(&PData[2], DataOut, DataOutLen);
	int PDataLen = DataOutLen + 2;

	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, PDataLen, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}
	
	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}

unsigned int tgGetInitiatorCommand(void){
	return NFC_SUCCESS;
}

unsigned int tgResponseToInitiator(unsigned char* TgResponse){
	return NFC_SUCCESS;
}

// 	tgGetTargetStatus() Status:tested
//	input:	OutParam:the container of return vals and should be inited with the length of 2.
//				remember to use free(OutParam) to release the memory when using malloc.
//				the order of the OutParam please refer to the User Manual.
//	output: the length of OutParam or NFC_FAIL(-1).
int tgGetTargetStatus(unsigned char* OutParam){
	//build TFI + PData
	unsigned char PData[2] = {0xD4, 0x8A};
	
	//transmit data
	unsigned char Receive[265] = {0};
	int ReceiveLen = 0;
	ReceiveLen = PN532transceive(PData, 2, Receive);
	if(ReceiveLen == NFC_FAIL){
		return NFC_FAIL;
	}
	
	//load info frame into OutParam
	int OutParamLen = 0;
	if(Receive[0] == 0x7F){
		//syntax error
		OutParam[0] = 0x7F;
		OutParamLen = 1;
	}else{
		//info frame
		memcpy(OutParam, &Receive[2], ReceiveLen-2);
		OutParamLen = ReceiveLen-2;
	}
	return OutParamLen;
}
// end of level-2 functions
