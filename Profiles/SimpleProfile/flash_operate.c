#include "hal_adc.h"
#include "hal_flash.h"
#include "hal_types.h"
#include "comdef.h"
#include "OSAL.h"
#include "osal_snv.h"
#include "hal_assert.h"
#include "saddr.h"
#include "flash_operate.h"
#include "bcomdef.h"
#include "OSAL_PwrMgr.h"

#include "OnBoard.h"
#include "hal_adc.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "flash_operate.h"

  
/*************************************
* uint8 flash_pwd_init( void )
* ��flash�ڲ�����ĳ�ʼ������
* ���ڶ�Ӧλ������Ӧ�ĳ�ʼ���룬8��0
**************************************/
uint8 flash_pwd_init( void )
{
	uint8 pwd[8]= { 1, 2, 3, 4, 5, 6, 7, 8};
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

/*************************************
* uint8 flash_pwd_write(void *pBuf)
* ��flash�ڲ�д������
* ����pBufΪһ��8λuint8������ĵ�ַ
**************************************/
uint8 flash_pwd_write(void *pBuf)
{
	return osal_snv_write(0x80, 8, pBuf);
}

/**************************************
* uint8 flash_pwd_read(void *pBuf)
* ��ȡflash�ڲ����������
* ����pBufΪһ��8λuint8����ĵ�ַ
**************************************/
uint8 flash_pwd_read(void *pBuf)
{
	return osal_snv_read(0x80, 8, pBuf);
}

/**************************************
* uint8 flash_pwd_delete(void *pBuf)
* ���flash�ڲ����������
**************************************/
uint8 flash_pwd_delete(void)
{
	uint8 pwd[8]= { 0, 0, 0, 0, 0, 0, 0, 0};
	return osal_snv_write(0x80, 8, pwd);
}



/**************************************
* uint8 flash_info_init( void )
* ��flash�ڲ���ʼ�����գ��������ݵĴ洢����
* ��flash�ڲ���ʼ�����գ��������ݵĳ��ȵĴ洢
**************************************/
uint8 flash_info_init( void )
{
    uint8 T_info[INFO_LENGTH] ={0};
    //��ַ0x82�Ƿ��ͳ�ȥ����Ϣ
    int8 ret8 = osal_snv_read(0x82, INFO_LENGTH, T_info);
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // �����ݽṹ���浽flash
        osal_memset(T_info, 0, INFO_LENGTH);
        osal_snv_write(0x82, INFO_LENGTH, T_info); 
        osal_snv_read(0x82, INFO_LENGTH, T_info);
    }
    //��ʼ��������Ϣ�ĳ���
    flash_Tinfo_Length_init();
    
    //��ַ0x84�ǽ��ܵ���Ϣ
    int8 ret9 = osal_snv_read(0x84, INFO_LENGTH, T_info);
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret9)
    {
        // �����ݽṹ���浽flash
        osal_memset(T_info, 0, INFO_LENGTH);
        osal_snv_write(0x84, INFO_LENGTH, T_info); 
        osal_snv_read(0x84, INFO_LENGTH, T_info);
    }
    //��ʼ��������Ϣ�ĳ���
    flash_Rinfo_Length_init();
    
    return SUCCESS;
}

/**************************************
* uint8 flash_Rinfo_all_read(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ����ȡ
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
uint8 flash_Rinfo_all_read(void *pBuf)
{
	return osal_snv_read(0x84, INFO_LENGTH, pBuf);
}

/**************************************
* uint8 flash_Tinfo_all_read(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ����ȡ
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
uint8 flash_Tinfo_all_read(void *pBuf)
{
	return osal_snv_read(0x82, INFO_LENGTH, pBuf);
}

/**************************************
* uint8 flash_Rinfo_all_write(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ��д��
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
uint8 flash_Rinfo_all_write(void *pBuf)
{
	return osal_snv_write(0x84, INFO_LENGTH, pBuf);
}

/**************************************
* uint8 flash_Tinfo_all_write(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ��д��
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
uint8 flash_Tinfo_all_write(void *pBuf)
{
	return osal_snv_write(0x82, INFO_LENGTH, pBuf);
}




/**************************************
* uint8 flash_Rinfo_single_read( uint8 seq )
* ��ȡflash�ڲ��������ݵĴ洢����ĵ�seq��λ���������
* ��seqδ�����洢����ĳ��ȣ����ض�Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
uint8 flash_Rinfo_single_read(uint8 seq)
{
	uint8 temp[INFO_LENGTH]={0};
        osal_snv_read(0x84, INFO_LENGTH, temp);
        if(seq<INFO_LENGTH)
        {
          return temp[seq];
        }else{
          return 0xFF;
        }
}




/**************************************
* uint8 flash_Tinfo_single_read( uint8 seq )
* ��ȡflash�ڲ��������ݵĴ洢����ĵ�seq��λ���������
* ��seqδ�����洢����ĳ��ȣ����ض�Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
uint8 flash_Tinfo_single_read(uint8 seq)
{
	uint8 temp[INFO_LENGTH]={0};
        osal_snv_read(0x82, INFO_LENGTH, temp);
	if(seq<INFO_LENGTH)
        {
          return temp[seq];
        }else{
          return 0xFF;
        }
}





/**************************************
* uint8 flash_Rinfo_single_write(uint8 seq, uint8 value)
* ��flash�ڲ��������ݵĴ洢����ĵ�seqλ��������value
* ��seqδ�����洢����ĳ��ȣ����ض�д�ɹ���Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
uint8 flash_Rinfo_single_write(uint8 seq, uint8 value)
{
	uint8 temp[INFO_LENGTH]={0};
        osal_snv_read(0x84, INFO_LENGTH, temp);
        if(seq<INFO_LENGTH)
        {
          temp[seq]=value;
          return osal_snv_write(0x84, INFO_LENGTH, temp);
        }else
        {
          return 0xFF;
        }
}

/**************************************
* uint8 flash_Tinfo_single_write(uint8 seq, uint8 value)
* ��flash�ڲ��������ݵĴ洢����ĵ�seqλ��������value
* ��seqδ�����洢����ĳ��ȣ����ض�д�ɹ���Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
uint8 flash_Tinfo_single_write(uint8 seq, uint8 value)
{
	uint8 temp[INFO_LENGTH]={0};
        osal_snv_read(0x82, INFO_LENGTH, temp);
        if(seq<INFO_LENGTH)
        {
          temp[seq]=value;
          return osal_snv_write(0x82, INFO_LENGTH, temp);
        }else
        {
          return 0xFF;
        }
}


/**************************************
* uint8 flash_Tinfo_Length_init(void��
* ��flash�ڲ���ʼ���������ݵĳ��ȴ洢λ
**************************************/
uint8 flash_Tinfo_Length_init(void)
{
    uint8 temp=0;
    //��ַ0x83�Ƿ������ݵĳ��ȵĴ洢����
    int8 ret8 = osal_snv_read(0x83, 1, &temp);
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // �����ݽṹ���浽flash
        osal_snv_write(0x83, 1, &temp); 
        osal_snv_read(0x83, 1, &temp);
    }
    return SUCCESS;
}




/**************************************
* uint8 flash_Tinfo_Length_set(uint8 length)
* ��flash�ڲ�д�뷢�����ݵĳ��ȴ洢λ
**************************************/
uint8 flash_Tinfo_Length_set(uint8 length)
{
	uint8 temp=0;
        temp=length;
        return osal_snv_write(0x83, 1, &temp);
}




/**************************************
* uint8 flash_Tinfo_Length_get(void)
* ��flash�ڲ���ȡ�������ݵĳ��ȴ洢λ
**************************************/
uint8 flash_Tinfo_Length_get(void)
{
        uint8 temp=1;
        osal_snv_read(0x83, 1, &temp);
        return temp;
}




/**************************************
* uint8 flash_Tinfo_short_write(void *pBuf, uint8 len)
* ��flash�ڲ�������������Ľ������ݵĳ���Ϊs
* ����s����д�볤��Ϊlen�����飬��ַ��pBuf
* �������洢���ȵ����ݲ�д
**************************************/
uint8 flash_Tinfo_short_write(void *pBuf, uint8 len)
{
        uint8 length=0;
        length=flash_Tinfo_Length_get();
        uint8 inMem[INFO_LENGTH]={0};
        
        osal_snv_read(0x82, INFO_LENGTH, inMem);
        uint8 i=0;
        for(i=0;i<len;i++)
        {
          if((length+i)<INFO_LENGTH)
          {
            inMem[length+i]=((uint8 *)pBuf)[i];
          }else
          {
            break;
          }
        }
        length=length+len;
        if(length>INFO_LENGTH)
        {
          length=INFO_LENGTH;
        }
	#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "LVALUE = ", length, 10, HAL_LCD_LINE_6 );
	#endif
        flash_Tinfo_Length_set(length);
	return osal_snv_write(0x82, INFO_LENGTH, inMem);
}



/**************************************
* uint8 flash_Tinfo_short_read(void *pBuf, uint8 seq)
* ��flash�ڲ�������������ĵ�seq����ʼΪ��0λ����󣨰���seq����ȡ����10������
* �������洢���ȣ������������
* ��ֵ��pBuf��
**************************************/
uint8 flash_Tinfo_short_read(void *pBuf, uint8 seq)
{
  uint8 inMem[INFO_LENGTH]={0};
  osal_snv_read(0x82, INFO_LENGTH, inMem);
  
  uint8 temp[10]={0};
  
  
  uint8 i=0;
  for(i=0;i<10;i++)
  {
    if((seq+i)<INFO_LENGTH)
    {
      temp[i]=inMem[seq+i];
    }else
    {
      break;
    }
  }
  
  for(i=0;i<10;i++)
  {
    ((uint8 *)pBuf)[i]=temp[i];
  }
  return 0;
}






/**************************************
* uint8 flash_Rinfo_Length_init(void��
* ��flash�ڲ���ʼ���������ݵĳ��ȴ洢λ
**************************************/
uint8 flash_Rinfo_Length_init(void)
{
    uint8 temp=0;
    //��ַ0x85�ǽ�����Ϣ�ĳ��ȵĴ洢����
    int8 ret8 = osal_snv_read(0x85, 1, &temp);
    // ����ö��ڴ�δ��д������ݣ� ֱ�Ӷ����᷵�� NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // �����ݽṹ���浽flash
        osal_snv_write(0x85, 1, &temp); 
        osal_snv_read(0x85, 1, &temp);
    }
    return SUCCESS;
}




/**************************************
* uint8 flash_Rinfo_Length_set(uint8 length)
* ��flash�ڲ�д��������ݵĳ��ȴ洢λ
**************************************/
uint8 flash_Rinfo_Length_set(uint8 length)
{
	uint8 temp=0;
        temp=length;
        return osal_snv_write(0x85, 1, &temp);
}




/**************************************
* uint8 flash_Rinfo_Length_get(void)
* ��flash�ڲ���ȡ�������ݵĳ��ȴ洢λ
**************************************/
uint8 flash_Rinfo_Length_get(void)
{
        uint8 temp=1;
        osal_snv_read(0x85, 1, &temp);
        return temp;
}






/**************************************
* uint8 flash_Rinfo_short_write(void *pBuf, uint8 len)
* ��flash�ڲ�������������Ľ������ݵĳ���Ϊs
* ����s����д�볤��Ϊlen�����飬��ַ��pBuf
* �������洢���ȵ����ݲ�д
**************************************/
uint8 flash_Rinfo_short_write(void *pBuf, uint8 len)
{
        uint8 length=0;
        length=flash_Rinfo_Length_get();
        uint8 inMem[INFO_LENGTH]={0};
        
        osal_snv_read(0x84, INFO_LENGTH, inMem);
        uint8 i=0;
        for(i=0;i<len;i++)
        {
          if((length+i)<INFO_LENGTH)
          {
            inMem[length+i]=((uint8 *)pBuf)[i];
          }else
          {
            break;
          }
        }
        length=length+len;
        if(length>INFO_LENGTH)
        {
          length=INFO_LENGTH;
        }
	#if (defined HAL_LCD) && (HAL_LCD == TRUE)
        HalLcdWriteStringValue( "LVALUE = ", length, 10, HAL_LCD_LINE_6 );
	#endif
        flash_Rinfo_Length_set(length);
	return osal_snv_write(0x84, INFO_LENGTH, inMem);
}

/**************************************
* uint8 flash_Rinfo_short_read(void *pBuf, uint8 seq)
* ��flash�ڲ�������������ĵ�seq����ʼΪ��0λ����󣨰���seq����ȡ����10������
* �������洢���ȣ������������
* ��ֵ��pBuf��
**************************************/
uint8 flash_Rinfo_short_read(void *pBuf, uint8 seq)
{
  uint8 inMem[INFO_LENGTH]={0};
  osal_snv_read(0x84, INFO_LENGTH, inMem);
  
  uint8 temp[10]={0};
  
  
  uint8 i=0;
  for(i=0;i<10;i++)
  {
    if((seq+i)<INFO_LENGTH)
    {
      temp[i]=inMem[seq+i];
    }else
    {
      break;
    }
  }
  
  for(i=0;i<10;i++)
  {
    ((uint8 *)pBuf)[i]=temp[i];
  }
  return 0;
}

