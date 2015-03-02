/**************************************************************************************************
  Filename:       NfcTask.c
 
  Description:    This file contains the NFC application for use with PN532.

**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "PN532_NFC.h"
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
#define CARD_MODE 0
#define SOCIAL_MODE 1
#define READER_MODE 2

#define NFC_PERIODIC_EVT_PERIOD 1000
#define SOCIAL_INIT_MAX_TRY_COUNT 10000
 /*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */
static uint8 NfcTask_TaskID;   // Task ID for internal task/event processing
uint8 next = CARD_MODE;
uint8 now = CARD_MODE;
uint16 SocialInitRestTryCnt = SOCIAL_INIT_MAX_TRY_COUNT;
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
	VOID task_id;
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
				/*case KEY_CHANGE:
					simpleBLEPeripheral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
					break;*/
				
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
				if(events & NFC_CARD_MODE_DE_EVT){
					//do nothing
				}else{
					//start the event to init pn532 as card
					osal_set_event(NfcTask_TaskID, NFC_CARD_MODE_INIT_EVT);
				}
				now = CARD_MODE;
				break;
			
			case SOCIAL_MODE:
				if(SocialInitRestTryCnt == 0){
					SocialInitRestTryCnt = SOCIAL_INIT_MAX_TRY_COUNT;
					next = CARD_MODE;
				}
				else{
					if(events & NFC_SOCIAL_MODE_DE_EVT){
						//do nothing
					}else{
						osal_set_event(NfcTask_TaskID, NFC_SOCIAL_MODE_INIT_EVT);
					}
					SocialInitRestTryCnt--;
					now = SOCIAL_MODE;
				}
				break;
			
			case READER_MODE:
				if(events & NFC_READER_MODE_DE_EVT){
					//do nothing
				}else{
					osal_set_event(NfcTask_TaskID, NFC_READER_MODE_INIT_EVT);
				}
				now = READER_MODE;
				break;
		}
		return (events ^ NFC_PERIODIC_EVT);
	}
	
	//process card mode initialization event
	if(events & NFC_CARD_MODE_INIT_EVT){
		NfcRelease();
		nfcUARTOpen();
		unsigned char MifareParams[6] = {0, 0, 0, 0, 0, 0x40};
		unsigned char FelicaParams[18] = {0};
		unsigned char NFCID3t[10] = {0};
		retVal* res = tgInitAsTarget(0, MifareParams, FelicaParams, NFCID3t, 0, NULL, 0, NULL);
		if(res == (retVal*) NFC_FAIL){	//low level error
#ifdef LINUX
			printf("low level error\n");
#else
			HalLcdWriteString( "tg init low level error", HAL_LCD_LINE_8 );
#endif
			goto fail;
		}else if(res->Rcv[0] == 0x7F){	//syntax error
#ifdef LINUX
			printf("syntax error\n");
#else
			HalLcdWriteString( "tg init syntax error", HAL_LCD_LINE_8 );
#endif
			osal_mem_free(res);
			goto fail;
		}
success:
		//deal with junks
		osal_mem_free(res);
		osal_set_event(NfcTask_TaskID, NFC_CARD_MODE_DE_EVT);
		return (events ^ NFC_CARD_MODE_INIT_EVT);
		
fail:
		return (events ^ NFC_CARD_MODE_INIT_EVT);
	}
	
	//process card mode data exchange event
	if(events & NFC_CARD_MODE_DE_EVT){
	
		return (events ^ NFC_CARD_MODE_DE_EVT);
	}
	
	//process social mode initialization event
	if(events & NFC_SOCIAL_MODE_INIT_EVT){
	
		return (events ^ NFC_SOCIAL_MODE_INIT_EVT);
	}
	
	//process social mode data exchange event
	if(events & NFC_SOCIAL_MODE_DE_EVT){
	
		return (events ^ NFC_SOCIAL_MODE_DE_EVT);
	}
	
	//process reader mode initialization event
	if(events & NFC_READER_MODE_INIT_EVT){
	
		return (events ^ NFC_READER_MODE_INIT_EVT);
	}
	
	//process reader mode data exchange event
	if(events & NFC_READER_MODE_DE_EVT){
	
		return (events ^ NFC_READER_MODE_DE_EVT);
	}
	
	// Discard unknown events
	return 0;
}

/*********************************************************************
*********************************************************************/