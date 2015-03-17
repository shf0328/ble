/**************************************************************************************************
  Filename:       NfcTask.c
 
  Description:    This file contains the NFC application for use with PN532.

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "PN532_NFC.h"
#include <string.h>
#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"

#include "gatt.h"
#include "Osal_snv.h"
#include "Osal_Nv.h"
#include "hci.h"
#include "flash_operate.h"

#include "gapgattserver.h"
#include "gattservapp.h"
#include "devinfoservice.h"
#include "simpleGATTprofile.h"

#if defined( CC2540_MINIDK )
  #include "simplekeys.h"
#endif

#include "peripheral.h"

#include "gapbondmgr.h"

#include "NfcTask.h"

#if defined FEATURE_OAD
  #include "oad.h"
  #include "oad_target.h"
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */


#define NFC_PERIODIC_EVT_PERIOD 1000
#define SOCIAL_INIT_MAX_TRY_COUNT 200
 /*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

uint8 next = CARD_MODE;
uint8 now = CARD_MODE;
//card mode

//social mode
uint16 SocialInitRestTryCnt = SOCIAL_INIT_MAX_TRY_COUNT;
uint8 SocialToRcvInitInfo = 0;
retVal* SocialRcvRetVal = NULL;
//reader mode card info
uint8 Key[6] = {0};
uint8 SerialNum[4] = {0};
/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS PROTOTYPES
 */

/*********************************************************************
 * FUNCTIONS DEFINITIONS
 */
 
/*********************************************************************
 * @fn      NfcTask_Init
 *
 * @brief   Initialization function for the NFC App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void NfcTask_Init( uint8 task_id ){
	NfcTask_TaskID = task_id;
	osal_start_timerEx( NfcTask_TaskID, NFC_PERIODIC_EVT, NFC_PERIODIC_EVT_PERIOD );
}

/*********************************************************************
 * @fn      NfcTask_ProcessEvent
 *
 * @brief   NFC Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 NfcTask_ProcessEvent( uint8 task_id, uint16 events ){
	VOID task_id;
	
	//process messages
	if(events & SYS_EVENT_MSG){
		uint8 *pMsg;
		if ( (pMsg = osal_msg_receive( NfcTask_TaskID )) != NULL ){
			//process message
			switch ( ((osal_event_hdr_t*)pMsg)->event ){
				case CARD:
					next = CARD_MODE;
					break;
					
				case SOCIAL:
					next = SOCIAL_MODE;
					SocialInitRestTryCnt = SOCIAL_INIT_MAX_TRY_COUNT;
					break;
					
				case READER:
					next = READER_MODE;
					break;
					
				default:
					// do nothing
					break;
			}
			// Release the OSAL message
			VOID osal_msg_deallocate( pMsg );
		}
		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}
	
	//process periodic event
	if(events & NFC_PERIODIC_EVT){
		// Restart timer
		if ( NFC_PERIODIC_EVT_PERIOD ){
		osal_start_timerEx( NfcTask_TaskID, NFC_PERIODIC_EVT, NFC_PERIODIC_EVT_PERIOD );
		}
		
		switch(next){
			case CARD_MODE:
				/*if( events & NFC_CARD_MODE_INIT_EVT ){
					//do nothing
				}else if( events & NFC_CARD_MODE_DE_EVT ){
					//do nothing
				}else{
					//start the event to init pn532 as card
					osal_set_event(NfcTask_TaskID, NFC_CARD_MODE_INIT_EVT);
				}
				now = CARD_MODE;*/
				break;
			
			case SOCIAL_MODE:
				if(SocialInitRestTryCnt == 0){
					SocialInitRestTryCnt = SOCIAL_INIT_MAX_TRY_COUNT;
					next = CARD_MODE;
				}
				else{
					//start the event to init pn532 as card
					if(events & NFC_SOCIAL_MODE_DE_EVT){
					
					}else if(events & NFC_SOCIAL_RCV_EVT){
						
					}else{
						osal_set_event(NfcTask_TaskID, NFC_SOCIAL_MODE_INIT_EVT);
					}
					SocialInitRestTryCnt--;
					now = SOCIAL_MODE;
				}
				break;
			
			case READER_MODE:
				if( events & NFC_READER_MODE_INIT_EVT ){
					//do nothing
				}else if( events & NFC_READER_MODE_AUTH_EVT ){
					//do nothing
				}else if( events & NFC_READER_MODE_READ_EVT ){
					//do nothing
				}else{
					//start the event to init pn532 as card
					osal_set_event(NfcTask_TaskID, NFC_READER_MODE_INIT_EVT);
				}
				now = READER_MODE;
				break;
		}
		return (events ^ NFC_PERIODIC_EVT);
	}
	
	//process card mode initialization event
	if(events & NFC_CARD_MODE_INIT_EVT){
		nfcUARTOpen();
		//init as mifare card
		//TODO: card info
		unsigned char MifareParams[6] = {0, 0, 0, 0, 0, 0x40};
		unsigned char FelicaParams[18] = {0};
		unsigned char NFCID3t[10] = {0};
		retVal* res = tgInitAsTarget(5, MifareParams, FelicaParams, NFCID3t, 0, NULL, 0, NULL, 1);
		if(res == (retVal*) NFC_FAIL){	//low level error
#ifdef DEBUG
			HalLcdWriteString( "card init low level error", HAL_LCD_LINE_8 );
#endif
			NfcRelease();
			goto card_init_fail;
		}else if(res->Rcv[0] == 0x7F){	//syntax error
#ifdef DEBUG
			HalLcdWriteString( "card init syntax error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			NfcRelease();
			goto card_init_fail;
		}
		//TODO: deal with first cmd
		
		//deal with junks
		osal_mem_free(res);
		osal_set_event(NfcTask_TaskID, NFC_CARD_MODE_DE_EVT);
		
card_init_fail:
		return (events ^ NFC_CARD_MODE_INIT_EVT);
	}
	
	//process card mode data exchange event
	if(events & NFC_CARD_MODE_DE_EVT){
		//TODO: mifare card data exchange
		//using tgGetInitiatorData and tgResponseToInitiator
		return (events ^ NFC_CARD_MODE_DE_EVT);
	}
	
	//process social mode event
	if(events & NFC_SOCIAL_MODE_INIT_EVT){
	
		//check if it's in social mode
		if(now != SOCIAL_MODE){
			goto social_init_fail;
		}
		
		nfcUARTOpen();
		NfcRelease();
		int res = PN532InitAsInitiator();
		if(res == NFC_FAIL){
			NfcRelease();
			goto social_init_fail;
		}
		SocialToRcvInitInfo = 1;
		osal_start_timerEx( NfcTask_TaskID, NFC_SOCIAL_RCV_EVT, 100 );
social_init_fail:
		return (events ^ NFC_SOCIAL_MODE_INIT_EVT);
	}
	
	//process social mode initialization receive event
	if(events & NFC_SOCIAL_RCV_EVT){
		
		if(SocialToRcvInitInfo == 1){
			//to receive social init info
			//short delay
			DelayMs(50);
			//receive info frame
			retVal* Receive = PN532receiveFrame();
			if(Receive == (retVal*) NFC_FAIL){	//error handling
#ifdef LINUX
				printf("InfoRcvError\n");
#else
				HalLcdWriteString( "InfoRcvError", HAL_LCD_LINE_6 );
#endif
				NfcRelease();
				goto social_rcv_fail;
			}

			//load info frame into Output
			if(Receive->Rcv[0] == 0x7F){
				SocialRcvRetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + 1);
				if(SocialRcvRetVal == NULL){	//memory allocation unsuccess
#ifdef LINUX
					printf("OutpMemoAllocError\n");
#else
					HalLcdWriteString( "OutpMemoAllocError", HAL_LCD_LINE_6 );
#endif
					osal_mem_free(Receive);
					NfcRelease();
					goto social_rcv_fail;
				}
				//syntax error
				SocialRcvRetVal->Rcv[0] = 0x7F;
				SocialRcvRetVal->length = 1;
			}else{
				SocialRcvRetVal = (retVal *) osal_mem_alloc (sizeof (retVal) + Receive->length-2);
				if(SocialRcvRetVal == NULL){	//memory allocation unsuccess
#ifdef LINUX
					printf("OutpMemoAllocError\n");
#else
					HalLcdWriteString( "OutpMemoAllocError", HAL_LCD_LINE_6 );
#endif
					osal_mem_free(Receive);
					NfcRelease();
					goto social_rcv_fail;
				}
				//info frame
				memcpy(SocialRcvRetVal->Rcv, &Receive->Rcv[2], Receive->length-2);
				SocialRcvRetVal->length = Receive->length-2;
			}
			//init success
			osal_set_event(NfcTask_TaskID, NFC_SOCIAL_MODE_DE_EVT);
			SocialToRcvInitInfo = 0;
		}
		
social_rcv_fail:
		return (events ^ NFC_SOCIAL_RCV_EVT);
	}
	
	//process social mode data exchange event
	if(events & NFC_SOCIAL_MODE_DE_EVT){
		//check if it's in social mode
		if(now != SOCIAL_MODE){
			goto social_de_end;
		}
			
		uint8 send[50]={0};
		uint8 rec[50]={0};
		flash_Tinfo_all_read(send);
		int res = NfcDataExchange(send, 50, rec);
		if(res == NFC_FAIL){
			HalLcdWriteString( "FAIL", HAL_LCD_LINE_5 );
			goto social_de_fail;
		}
		flash_Rinfo_all_write(rec);
		HalLcdWriteString( "SUCCESS", HAL_LCD_LINE_5 );
social_de_fail:
		NfcRelease();
		next = CARD_MODE;
		SocialInitRestTryCnt = SOCIAL_INIT_MAX_TRY_COUNT;
social_de_end:
		return (events ^ NFC_SOCIAL_MODE_DE_EVT);
	}
	
	//process reader mode initialization event
	if(events & NFC_READER_MODE_INIT_EVT){
		nfcUARTOpen();
		retVal* res = inListPassiveTarget(1, 0, NULL, 0);
		if(res == (retVal*) NFC_FAIL){	//low level error
#ifdef DEBUG
			HalLcdWriteString( "reader init low level error", HAL_LCD_LINE_8 );
#endif
			goto reader_init_fail;
		}else if(res->Rcv[0] == 0x7F){	//syntax error
#ifdef DEBUG
			HalLcdWriteString( "reader init low level error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			NfcRelease();
			goto reader_init_fail;
		}
		
		if(res->Rcv[0] == 0){
#ifdef DEBUG
			HalLcdWriteString( "no card present", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			NfcRelease();
			goto reader_init_fail;
		}
		
		//TODO: deal with card info
		memcpy(SerialNum, &res->Rcv[6], 4);
		//deal with junks
		osal_mem_free(res);
		osal_set_event(NfcTask_TaskID, NFC_READER_MODE_AUTH_EVT);
reader_init_fail:
		return (events ^ NFC_READER_MODE_INIT_EVT);
	}
	
	//authenticate mifare card
	if(events & NFC_READER_MODE_AUTH_EVT){
		//Authentication
		int DataOutLen = 18;
		uint8* DataOut = osal_mem_alloc(DataOutLen);
		DataOut[0] = 0x60;	//Authentication A
		//TODO: set address and key
		
		memcpy(&DataOut[8], SerialNum, 4);
		retVal* res = inDataExchange(1, DataOut, DataOutLen);
		if(res == (retVal*) NFC_FAIL){
			//low level error
			NfcRelease();
			goto reader_auth_fail;
		}else if( (res->Rcv[0]&0x3F) != 0 ){
			//app level error
			NfcRelease();
			goto reader_auth_fail;
		}
		
		osal_set_event(NfcTask_TaskID, NFC_READER_MODE_READ_EVT);
		osal_mem_free(DataOut);
		osal_mem_free(res);
reader_auth_fail:
		return (events ^ NFC_READER_MODE_AUTH_EVT);
	}
	
	//read mifare card
	if(events & NFC_READER_MODE_READ_EVT){
		//TODO: reader mode data exchange using inDataExchange
		
		//Reading
		int DataOutLen = 2;
		uint8* DataOut = osal_mem_alloc(DataOutLen);
		DataOut[0] = 0x30;	//16-bytes reading
		//TODO: set address
		
		retVal* res = inDataExchange(1, DataOut, DataOutLen);
		if(res == (retVal*) NFC_FAIL){
			//low level error
			goto reader_read_fail;
		}else if( (res->Rcv[0]&0x3F) != 0 ){
			//app level error
			goto reader_read_fail;
		}
		//TODO: handle received data
		
		next = CARD_MODE;
		NfcRelease();
		osal_mem_free(DataOut);
		osal_mem_free(res);
reader_read_fail:
		return (events ^ NFC_READER_MODE_READ_EVT);
	}
	
	// Discard unknown events
	return 0;
}

/*********************************************************************
*********************************************************************/