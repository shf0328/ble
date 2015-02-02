#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "osal_snv.h"

extern uint8 flash_pwd_init( void );

extern uint8 flash_pwd_write(void *pBuf);

extern uint8 flash_pwd_read(void *pBuf);

extern uint8 flash_pwd_delete(void);

extern uint8 flash_info_init( void );

extern uint8 flash_Rinfo_all_read(void *pBuf);

extern uint8 flash_Tinfo_all_read(void *pBuf);

extern uint8 flash_Rinfo_all_write(void *pBuf);

extern uint8 flash_Tinfo_all_write(void *pBuf);




