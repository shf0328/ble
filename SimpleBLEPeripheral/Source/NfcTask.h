/**************************************************************************************************
	Filename:       NfcTask.h

	Description:    This file contains the NFC application definitions and prototypes.

**************************************************************************************************/

#ifndef _NFCTASK_H
#define _NFCTASK_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

/*********************************************************************
 * CONSTANTS
 */


// Simple BLE Peripheral Task Events
#define NFC_CARD_MODE_INIT_EVT		0x0001
#define NFC_CARD_MODE_DE_EVT		0x0002
#define NFC_SOCIAL_MODE_INIT_EVT	0x0004
#define NFC_SOCIAL_MODE_DE_EVT		0x0008
#define NFC_READER_MODE_INIT_EVT	0x0010
#define NFC_READER_MODE_DE_EVT		0x0020
#define NFC_PERIODIC_EVT			0x0040

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * FUNCTIONS' PROTOTYPES
 */

/*
 * Task Initialization for the BLE Application
 */
extern void NfcTask_Init( uint8 task_id );

/*
 * Task Event Processor for the BLE Application
 */
extern uint16 NfcTask_ProcessEvent( uint8 task_id, uint16 events );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _NFCTASK_H */
