#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "osal_snv.h"
#define INFO_LENGTH   50  
   
/*************************************
* uint8 flash_pwd_init( void )
* 在flash内部密码的初始化函数
* 即在对应位存入相应的初始密码，8个0
**************************************/
extern uint8 flash_pwd_init( void );




/*************************************
* uint8 flash_pwd_write(void *pBuf)
* 向flash内部写入密码
* 参数pBuf为一个8位uint8的数组的地址
**************************************/
extern uint8 flash_pwd_write(void *pBuf);




/**************************************
* uint8 flash_pwd_read(void *pBuf)
* 读取flash内部存入的密码
* 参数pBuf为一个8位uint8数组的地址
**************************************/
extern uint8 flash_pwd_read(void *pBuf);




/**************************************
* uint8 flash_pwd_delete(void *pBuf)
* 清空flash内部存入的密码
**************************************/
extern uint8 flash_pwd_delete(void);




/**************************************
* uint8 flash_info_init( void )
* 在flash内部初始化接收，发送数据的存储区域
* 在flash内部初始化接收，发送数据的长度的存储
**************************************/
extern uint8 flash_info_init( void );




/**************************************
* uint8 flash_Rinfo_all_read(void *pBuf)
* 在flash内部接收数据的全部读取
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
extern uint8 flash_Rinfo_all_read(void *pBuf);






/**************************************
* uint8 flash_Tinfo_all_read(void *pBuf)
* 在flash内部发送数据的全部读取
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
extern uint8 flash_Tinfo_all_read(void *pBuf);





/**************************************
* uint8 flash_Tinfo_all_write(void *pBuf)
* 在flash内部发送数据的全部写入
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
extern uint8 flash_Rinfo_all_write(void *pBuf);





/**************************************
* uint8 flash_Tinfo_all_write(void *pBuf)
* 在flash内部发送数据的全部写入
* 参数是一个长度INFO_LENGTH的数组地址
* 返回值是osal flash操作的值，具体参见API文档
* 一般使用成功是SUCCESS
**************************************/
extern uint8 flash_Tinfo_all_write(void *pBuf);




/**************************************
* uint8 flash_Rinfo_single_read( uint8 seq )
* 读取flash内部接收数据的存储区域的第seq个位存入的数据
* 若seq未超过存储区域的长度，返回对应值
* 若seq超过存储区域的长度，返回0xFF
**************************************/
extern uint8 flash_Rinfo_single_read(uint8 seq);





/**************************************
* uint8 flash_Tinfo_single_read( uint8 seq )
* 读取flash内部发送数据的存储区域的第seq个位存入的数据
* 若seq未超过存储区域的长度，返回对应值
* 若seq超过存储区域的长度，返回0xFF
**************************************/
extern uint8 flash_Tinfo_single_read(uint8 seq);




/**************************************
* uint8 flash_Rinfo_single_write(uint8 seq, uint8 value)
* 向flash内部接收数据的存储区域的第seq位存入数据value
* 若seq未超过存储区域的长度，返回读写成功对应值
* 若seq超过存储区域的长度，返回0xFF
**************************************/
extern uint8 flash_Rinfo_single_write(uint8 seq, uint8 value);




/**************************************
* uint8 flash_Tinfo_single_write(uint8 seq, uint8 value)
* 向flash内部发送数据的存储区域的第seq位存入数据value
* 若seq未超过存储区域的长度，返回读写成功对应值
* 若seq超过存储区域的长度，返回0xFF
**************************************/
extern uint8 flash_Tinfo_single_write(uint8 seq, uint8 value);





/**************************************
* uint8 flash_Tinfo_Length_init(void）
* 在flash内部初始化发送数据的长度存储位
**************************************/
extern uint8 flash_Tinfo_Length_init(void);




/**************************************
* uint8 flash_Tinfo_Length_set(uint8 length)
* 在flash内部写入发送数据的长度存储位
**************************************/
extern uint8 flash_Tinfo_Length_set(uint8 length);





/**************************************
* uint8 flash_Tinfo_Length_get(void)
* 在flash内部读取发送数据的长度存储位
**************************************/
extern uint8 flash_Tinfo_Length_get(void);





/**************************************
* uint8 flash_Tinfo_short_write(void *pBuf, uint8 len)
* 在flash内部发送数据区域的接收数据的长度为s
* 则向s后面写入长度为len的数组，地址是pBuf
* 若超过存储长度的数据不写
**************************************/
extern uint8 flash_Tinfo_short_write(void *pBuf, uint8 len);


/**************************************
* uint8 flash_Tinfo_short_write(void *pBuf, uint8 seq)
* 在flash内部发送数据区域的第seq处开始为第0位，向后（包括seq）读取长度5的数组
* 若超过存储长度，则在数组后补零
* 赋值给pBuf处
**************************************/
extern uint8 flash_Tinfo_short_read(void *pBuf, uint8 seq);



/**************************************
* uint8 flash_Rinfo_Length_init(void）
* 在flash内部初始化发送数据的长度存储位
**************************************/
extern uint8 flash_Rinfo_Length_init(void);





/**************************************
* uint8 flash_Rinfo_Length_set(uint8 length)
* 在flash内部写入接收数据的长度存储位
**************************************/
extern uint8 flash_Rinfo_Length_set(uint8 length);





/**************************************
* uint8 flash_Rinfo_Length_get(void)
* 在flash内部读取接收数据的长度存储位
**************************************/
extern uint8 flash_Rinfo_Length_get(void);




/**************************************
* uint8 flash_Rinfo_short_write(void *pBuf, uint8 len)
* 在flash内部接收数据区域的接收数据的长度为s
* 则向s后面写入长度为len的数组，地址是pBuf
* 若超过存储长度的数据不写
**************************************/
extern uint8 flash_Rinfo_short_write(void *pBuf, uint8 len);




/**************************************
* uint8 flash_Rinfo_short_write(void *pBuf, uint8 seq)
* 在flash内部接收数据区域的第seq处开始为第0位，向后（包括seq）读取长度5的数组
* 若超过存储长度，则在数组后补零
* 赋值给pBuf处
**************************************/
extern uint8 flash_Rinfo_short_read(void *pBuf, uint8 seq);
