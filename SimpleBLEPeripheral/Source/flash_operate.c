#include "hal_adc.h"
#include "hal_flash.h"
#include "hal_types.h"
#include "comdef.h"
#include "OSAL.h"
#include "osal_snv.h"
#include "hal_assert.h"
#include "saddr.h"


uint8 flash_pwd_init( void )
{
	uint8 pwd[8]= { 0, 0, 0, 0, 0, 0, 0, 0};
	int8 ret8 = osal_snv_read(0x80, 8, pwd);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED ,
    // 我们利用这个特点作为第一次烧录后的运行， 从而设置参数的出厂设置
    if(NV_OPER_FAILED == ret8)
    {
        // 把数据结构保存到flash
        osal_memset(pwd, 0, 8);
        osal_snv_write(0x80, 8, pwd); 
        osal_snv_read(0x80, 8, pwd);
    }
	return SUCCESS;
}

uint8 flash_pwd_write(void *pBuf)
{
	return osal_snv_write(0x80, 8, pBuf);
}

uint8 flash_pwd_read(void *pBuf)
{
	return osal_snv_read(0x80, 8, pBuf);
}

uint8 flash_pwd_delete(void)
{
	uint8 pwd[8]= { 0, 0, 0, 0, 0, 0, 0, 0};
	return osal_snv_write(0x80, 8, pwd);
}

uint8 flash_info_init( void )
{
	uint8 T_info[50] ={0};
	//地址0x82是发送出去的信息
	int8 ret8 = osal_snv_read(0x82, 50, T_info);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // 把数据结构保存到flash
        osal_memset(T_info, 0, 50);
        osal_snv_write(0x82, 50, T_info); 
        osal_snv_read(0x82, 50, T_info);
    }
	//地址0x84是接受的信息
	int8 ret9 = osal_snv_read(0x84, 50, T_info);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret9)
    {
        // 把数据结构保存到flash
        osal_memset(T_info, 0, 50);
        osal_snv_write(0x84, 50, T_info); 
        osal_snv_read(0x84, 50, T_info);
    }
	return SUCCESS;
}


uint8 flash_Rinfo_all_read(void *pBuf)
{
	return osal_snv_read(0x84, 50, pBuf);
}

uint8 flash_Tinfo_all_read(void *pBuf)
{
	return osal_snv_read(0x82, 50, pBuf);
}

uint8 flash_Rinfo_all_write(void *pBuf)
{
	return osal_snv_write(0x84, 50, pBuf);
}

uint8 flash_Tinfo_all_write(void *pBuf)
{
	return osal_snv_write(0x82, 50, pBuf);
}