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
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED ,
    // ������������ص���Ϊ��һ����¼������У� �Ӷ����ò����ĳ�������
    if(NV_OPER_FAILED == ret8)
    {
        // �����ݽṹ���浽flash
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
	//��ַ0x82�Ƿ��ͳ�ȥ����Ϣ
	int8 ret8 = osal_snv_read(0x82, 50, T_info);
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // �����ݽṹ���浽flash
        osal_memset(T_info, 0, 50);
        osal_snv_write(0x82, 50, T_info); 
        osal_snv_read(0x82, 50, T_info);
    }
	//��ַ0x84�ǽ��ܵ���Ϣ
	int8 ret9 = osal_snv_read(0x84, 50, T_info);
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret9)
    {
        // �����ݽṹ���浽flash
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