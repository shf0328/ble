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
* 在flash内部密码的初始化函数
* 即在对应位存入相应的初始密码，8个0
**************************************/
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

/*************************************
* uint8 flash_pwd_write(void *pBuf)
* 向flash内部写入密码
* 参数pBuf为一个8位uint8的数组的地址
**************************************/
uint8 flash_pwd_write(void *pBuf)
{
	return osal_snv_write(0x80, 8, pBuf);
}

/**************************************
* uint8 flash_pwd_read(void *pBuf)
* 读取flash内部存入的密码
* 参数pBuf为一个8位uint8数组的地址
**************************************/
uint8 flash_pwd_read(void *pBuf)
{
	return osal_snv_read(0x80, 8, pBuf);
}

/**************************************
* uint8 flash_pwd_delete(void *pBuf)
* 清空flash内部存入的密码
**************************************/
uint8 flash_pwd_delete(void)
{
	uint8 pwd[8]= { 0, 0, 0, 0, 0, 0, 0, 0};
	return osal_snv_write(0x80, 8, pwd);
}



/**************************************
* uint8 flash_info_init( void )
* 在flash内部初始化接收，发送数据的存储区域
* 在flash内部初始化接收，发送数据的长度的存储
**************************************/
uint8 flash_info_init( void )
{
    uint8 T_info[INFO_LENGTH] ={0};
    //地址0x82是发送出去的信息
    int8 ret8 = osal_snv_read(0x82, INFO_LENGTH, T_info);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // 把数据结构保存到flash
        osal_memset(T_info, 0, INFO_LENGTH);
        osal_snv_write(0x82, INFO_LENGTH, T_info); 
        osal_snv_read(0x82, INFO_LENGTH, T_info);
    }
    //初始化发送信息的长度
    flash_Tinfo_Length_init();
    
    //地址0x84是接受的信息
    int8 ret9 = osal_snv_read(0x84, INFO_LENGTH, T_info);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret9)
    {
        // 把数据结构保存到flash
        osal_memset(T_info, 0, INFO_LENGTH);
        osal_snv_write(0x84, INFO_LENGTH, T_info); 
        osal_snv_read(0x84, INFO_LENGTH, T_info);
    }
    //初始化接收信息的长度
    flash_Rinfo_Length_init();
    
    return SUCCESS;
}

/**************************************
* uint8 flash_Rinfo_all_read(void *pBuf)
* 在flash内部接收数据的全部读取
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
uint8 flash_Rinfo_all_read(void *pBuf)
{
	return osal_snv_read(0x84, INFO_LENGTH, pBuf);
}

/**************************************
* uint8 flash_Tinfo_all_read(void *pBuf)
* 在flash内部发送数据的全部读取
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
uint8 flash_Tinfo_all_read(void *pBuf)
{
	return osal_snv_read(0x82, INFO_LENGTH, pBuf);
}

/**************************************
* uint8 flash_Tinfo_all_read(void *pBuf)
* 在flash内部发送数据的全部写入
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
uint8 flash_Rinfo_all_write(void *pBuf)
{
	return osal_snv_write(0x84, INFO_LENGTH, pBuf);
}

/**************************************
* uint8 flash_Tinfo_all_read(void *pBuf)
* 在flash内部发送数据的全部写入
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
uint8 flash_Tinfo_all_write(void *pBuf)
{
	return osal_snv_write(0x82, INFO_LENGTH, pBuf);
}




/**************************************
* uint8 flash_Rinfo_single_read( uint8 seq )
* 读取flash内部接收数据的存储区域的第seq个位存入的数据
* 若seq未超过存储区域的长度，返回对应值
* 若seq超过存储区域的长度，返回0xFF
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
* 读取flash内部发送数据的存储区域的第seq个位存入的数据
* 若seq未超过存储区域的长度，返回对应值
* 若seq超过存储区域的长度，返回0xFF
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
* 向flash内部接收数据的存储区域的第seq位存入数据value
* 若seq未超过存储区域的长度，返回读写成功对应值
* 若seq超过存储区域的长度，返回0xFF
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
* 向flash内部发送数据的存储区域的第seq位存入数据value
* 若seq未超过存储区域的长度，返回读写成功对应值
* 若seq超过存储区域的长度，返回0xFF
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
* uint8 flash_Tinfo_Length_init(void）
* 在flash内部初始化发送数据的长度存储位
**************************************/
uint8 flash_Tinfo_Length_init(void)
{
    uint8 temp=0;
    //地址0x85是接收信息的长度的存储区域
    int8 ret8 = osal_snv_read(0x83, 1, &temp);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // 把数据结构保存到flash
        osal_snv_write(0x83, 1, &temp); 
        osal_snv_read(0x83, 1, &temp);
    }
    return SUCCESS;
}




/**************************************
* uint8 flash_Tinfo_Length_set(uint8 length)
* 在flash内部写入发送数据的长度存储位
**************************************/
uint8 flash_Tinfo_Length_set(uint8 length)
{
	uint8 temp=0;
        temp=length;
        return osal_snv_write(0x83, 1, &temp);
}




/**************************************
* uint8 flash_Tinfo_Length_get(void)
* 在flash内部读取发送数据的长度存储位
**************************************/
uint8 flash_Tinfo_Length_get(void)
{
        uint8 temp=1;
        osal_snv_read(0x83, 1, &temp);
        return temp;
}




/**************************************
* uint8 flash_Tinfo_short_write(void *pBuf, uint8 len)
* 在flash内部发送数据区域的接收数据的长度为s
* 则向s后面写入长度为len的数组，地址是pBuf
* 若超过存储长度的数据不写
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
        HalLcdWriteStringValue( "LVALUE = ", length, 10, HAL_LCD_LINE_6 );
        flash_Rinfo_Length_set(length);
	return osal_snv_write(0x82, INFO_LENGTH, inMem);
}



/**************************************
* uint8 flash_Tinfo_short_write(void *pBuf, uint8 seq)
* 在flash内部发送数据区域的第seq处开始为第0位，向后（包括seq）读取长度5的数组
* 若超过存储长度，则在数组后补零
* 赋值给pBuf处
**************************************/
uint8 flash_Tinfo_short_read(void *pBuf, uint8 seq)
{
  uint8 inMem[INFO_LENGTH]={0};
  osal_snv_read(0x84, INFO_LENGTH, inMem);
  
  uint8 temp[5]={0};
  
  
  uint8 i=0;
  for(i=0;i<5;i++)
  {
    if((seq+i)<INFO_LENGTH)
    {
      temp[i]=inMem[seq+i];
    }else
    {
      break;
    }
  }
  
  for(i=0;i<5;i++)
  {
    ((uint8 *)pBuf)[i]=temp[i];
  }
  return 0;
}






/**************************************
* uint8 flash_Rinfo_Length_init(void）
* 在flash内部初始化发送数据的长度存储位
**************************************/
uint8 flash_Rinfo_Length_init(void)
{
    uint8 temp=0;
    //地址0x85是接收信息的长度的存储区域
    int8 ret8 = osal_snv_read(0x85, 1, &temp);
    // 如果该段内存未曾写入过数据， 直接读，会返回 NV_OPER_FAILED 
    if(NV_OPER_FAILED == ret8)
    {
        // 把数据结构保存到flash
        osal_snv_write(0x85, 1, &temp); 
        osal_snv_read(0x85, 1, &temp);
    }
    return SUCCESS;
}




/**************************************
* uint8 flash_Rinfo_Length_set(uint8 length)
* 在flash内部写入接收数据的长度存储位
**************************************/
uint8 flash_Rinfo_Length_set(uint8 length)
{
	uint8 temp=0;
        temp=length;
        return osal_snv_write(0x85, 1, &temp);
}




/**************************************
* uint8 flash_Rinfo_Length_get(void)
* 在flash内部读取接收数据的长度存储位
**************************************/
uint8 flash_Rinfo_Length_get(void)
{
        uint8 temp=1;
        osal_snv_read(0x85, 1, &temp);
        return temp;
}






/**************************************
* uint8 flash_Rinfo_short_write(void *pBuf, uint8 len)
* 在flash内部接收数据区域的接收数据的长度为s
* 则向s后面写入长度为len的数组，地址是pBuf
* 若超过存储长度的数据不写
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
        HalLcdWriteStringValue( "LVALUE = ", length, 10, HAL_LCD_LINE_6 );
        flash_Rinfo_Length_set(length);
	return osal_snv_write(0x84, INFO_LENGTH, inMem);
}

/**************************************
* uint8 flash_Rinfo_short_write(void *pBuf, uint8 seq)
* 在flash内部接收数据区域的第seq处开始为第0位，向后（包括seq）读取长度5的数组
* 若超过存储长度，则在数组后补零
* 赋值给pBuf处
**************************************/
uint8 flash_Rinfo_short_read(void *pBuf, uint8 seq)
{
  uint8 inMem[INFO_LENGTH]={0};
  osal_snv_read(0x84, INFO_LENGTH, inMem);
  
  uint8 temp[5]={0};
  
  
  uint8 i=0;
  for(i=0;i<5;i++)
  {
    if((seq+i)<INFO_LENGTH)
    {
      temp[i]=inMem[seq+i];
    }else
    {
      break;
    }
  }
  
  for(i=0;i<5;i++)
  {
    ((uint8 *)pBuf)[i]=temp[i];
  }
  return 0;
}

