

/*********************************************************************
 * INCLUDES
 */
#include "hal_types.h"
#include "osal_snv.h"
#define INFO_LENGTH   50  
   
/*************************************
* uint8 flash_pwd_init( void )
* ��flash�ڲ�����ĳ�ʼ������
* ���ڶ�Ӧλ������Ӧ�ĳ�ʼ���룬8��0
**************************************/
extern uint8 flash_pwd_init( void );




/*************************************
* uint8 flash_pwd_write(void *pBuf)
* ��flash�ڲ�д������
* ����pBufΪһ��8λuint8������ĵ�ַ
**************************************/
extern uint8 flash_pwd_write(void *pBuf);




/**************************************
* uint8 flash_pwd_read(void *pBuf)
* ��ȡflash�ڲ����������
* ����pBufΪһ��8λuint8����ĵ�ַ
**************************************/
extern uint8 flash_pwd_read(void *pBuf);




/**************************************
* uint8 flash_pwd_delete(void *pBuf)
* ���flash�ڲ����������
**************************************/
extern uint8 flash_pwd_delete(void);




/**************************************
* uint8 flash_info_init( void )
* ��flash�ڲ���ʼ�����գ��������ݵĴ洢����
* ��flash�ڲ���ʼ�����գ��������ݵĳ��ȵĴ洢
**************************************/
extern uint8 flash_info_init( void );




/**************************************
* uint8 flash_Rinfo_all_read(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ����ȡ
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
extern uint8 flash_Rinfo_all_read(void *pBuf);






/**************************************
* uint8 flash_Tinfo_all_read(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ����ȡ
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
extern uint8 flash_Tinfo_all_read(void *pBuf);





/**************************************
* uint8 flash_Rinfo_all_write(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ��д��
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
extern uint8 flash_Rinfo_all_write(void *pBuf);





/**************************************
* uint8 flash_Tinfo_all_write(void *pBuf)
* ��flash�ڲ��������ݵ�ȫ��д��
* ������һ������INFO_LENGTH�������ַ
* ����ֵ��osal flash������ֵ������μ�API�ĵ�
* һ��ʹ�óɹ���SUCCESS
**************************************/
extern uint8 flash_Tinfo_all_write(void *pBuf);




/**************************************
* uint8 flash_Rinfo_single_read( uint8 seq )
* ��ȡflash�ڲ��������ݵĴ洢����ĵ�seq��λ���������
* ��seqδ�����洢����ĳ��ȣ����ض�Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
extern uint8 flash_Rinfo_single_read(uint8 seq);





/**************************************
* uint8 flash_Tinfo_single_read( uint8 seq )
* ��ȡflash�ڲ��������ݵĴ洢����ĵ�seq��λ���������
* ��seqδ�����洢����ĳ��ȣ����ض�Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
extern uint8 flash_Tinfo_single_read(uint8 seq);




/**************************************
* uint8 flash_Rinfo_single_write(uint8 seq, uint8 value)
* ��flash�ڲ��������ݵĴ洢����ĵ�seqλ��������value
* ��seqδ�����洢����ĳ��ȣ����ض�д�ɹ���Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
extern uint8 flash_Rinfo_single_write(uint8 seq, uint8 value);




/**************************************
* uint8 flash_Tinfo_single_write(uint8 seq, uint8 value)
* ��flash�ڲ��������ݵĴ洢����ĵ�seqλ��������value
* ��seqδ�����洢����ĳ��ȣ����ض�д�ɹ���Ӧֵ
* ��seq�����洢����ĳ��ȣ�����0xFF
**************************************/
extern uint8 flash_Tinfo_single_write(uint8 seq, uint8 value);





/**************************************
* uint8 flash_Tinfo_Length_init(void��
* ��flash�ڲ���ʼ���������ݵĳ��ȴ洢λ
**************************************/
extern uint8 flash_Tinfo_Length_init(void);




/**************************************
* uint8 flash_Tinfo_Length_set(uint8 length)
* ��flash�ڲ�д�뷢�����ݵĳ��ȴ洢λ
**************************************/
extern uint8 flash_Tinfo_Length_set(uint8 length);





/**************************************
* uint8 flash_Tinfo_Length_get(void)
* ��flash�ڲ���ȡ�������ݵĳ��ȴ洢λ
**************************************/
extern uint8 flash_Tinfo_Length_get(void);





/**************************************
* uint8 flash_Tinfo_short_write(void *pBuf, uint8 len)
* ��flash�ڲ�������������Ľ������ݵĳ���Ϊs
* ����s����д�볤��Ϊlen�����飬��ַ��pBuf
* �������洢���ȵ����ݲ�д
**************************************/
extern uint8 flash_Tinfo_short_write(void *pBuf, uint8 len);


/**************************************
* uint8 flash_Tinfo_short_read(void *pBuf, uint8 seq)
* ��flash�ڲ�������������ĵ�seq����ʼΪ��0λ����󣨰���seq����ȡ����5������
* �������洢���ȣ������������
* ��ֵ��pBuf��
**************************************/
extern uint8 flash_Tinfo_short_read(void *pBuf, uint8 seq);



/**************************************
* uint8 flash_Rinfo_Length_init(void��
* ��flash�ڲ���ʼ���������ݵĳ��ȴ洢λ
**************************************/
extern uint8 flash_Rinfo_Length_init(void);





/**************************************
* uint8 flash_Rinfo_Length_set(uint8 length)
* ��flash�ڲ�д��������ݵĳ��ȴ洢λ
**************************************/
extern uint8 flash_Rinfo_Length_set(uint8 length);





/**************************************
* uint8 flash_Rinfo_Length_get(void)
* ��flash�ڲ���ȡ�������ݵĳ��ȴ洢λ
**************************************/
extern uint8 flash_Rinfo_Length_get(void);




/**************************************
* uint8 flash_Rinfo_short_write(void *pBuf, uint8 len)
* ��flash�ڲ�������������Ľ������ݵĳ���Ϊs
* ����s����д�볤��Ϊlen�����飬��ַ��pBuf
* �������洢���ȵ����ݲ�д
**************************************/
extern uint8 flash_Rinfo_short_write(void *pBuf, uint8 len);




/**************************************
* uint8 flash_Rinfo_short_read(void *pBuf, uint8 seq)
* ��flash�ڲ�������������ĵ�seq����ʼΪ��0λ����󣨰���seq����ȡ����5������
* �������洢���ȣ������������
* ��ֵ��pBuf��
**************************************/
extern uint8 flash_Rinfo_short_read(void *pBuf, uint8 seq);
