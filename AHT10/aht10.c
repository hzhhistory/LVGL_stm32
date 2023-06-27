#include "aht10.h"
#include "delay.h"
#include "myiic.h"

/*********************************************************************************
			  ___   _     _____  _____  _   _  _____  _____  _   __
			 / _ \ | |   |_   _||  ___|| \ | ||_   _||  ___|| | / /
			/ /_\ \| |     | |  | |__  |  \| |  | |  | |__  | |/ /
			|  _  || |     | |  |  __| | . ` |  | |  |  __| |    \
			| | | || |_____| |_ | |___ | |\  |  | |  | |___ | |\  \
			\_| |_/\_____/\___/ \____/ \_| \_/  \_/  \____/ \_| \_/

 *	******************************************************************************
 *	������ֻ��ѧϰʹ�ã�δ���������ɣ��������������κ���;
 *	ALIENTEK Pandora STM32L475 IOT������
 *	AHT10��������
 *	����ԭ��@ALIENTEK
 *	������̳:www.openedv.com
 *	��������:2018/10/27
 *	�汾��V1.0
 *	��Ȩ���У�����ؾ���
 *	Copyright(C) �������������ӿƼ����޹�˾ 2014-2024
 *	All rights reserved
 *	******************************************************************************
 *	��ʼ�汾
 *	******************************************************************************/

/**
 * @brief	��ATH10д������
 *
 * @param   cmd		����
 * @param   data	Ҫд�������
 * @param   len		д�����ݴ�С
 *
 * @return  u8		0,����,����,�������
 */
u8 AHT10_Write_Data(u8 cmd, u8 *data, u8 len)
{
    IIC_Start();
    IIC_Send_Byte((AHT10_IIC_ADDR << 1) | 0); //����������ַ+д����

    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    IIC_Send_Byte(cmd);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��

    for(u8 i = 0; i < len; i++)
    {
        IIC_Send_Byte(data[i]);     //��������
        IIC_Wait_Ack();				//�ȴ�Ӧ��
    }

    IIC_Stop();
    return 0;
}


/**
 * @brief	��һ���ֽ�
 *
 * @param   void
 *
 * @return  u8		����������
 */
u8 AHT10_ReadOneByte(void)
{
    u8 res = 0;
    IIC_Start();
    IIC_Send_Byte((AHT10_IIC_ADDR << 1) | 0X01); //����������ַ+������

    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    res = IIC_Read_Byte(0);		//������,����nACK
    IIC_Stop();                 //����һ��ֹͣ����
    return res;
}

/**
 * @brief	������
 *
 * @param   data	���ݻ���
 * @param   len		�����ݴ�С
 *
 * @return  u8		0,����,����,�������
 */
u8 AHT10_Read_Data(u8 *data, u8 len)
{
    IIC_Start();
    IIC_Send_Byte((AHT10_IIC_ADDR << 1) | 0x01); //����������ַ+������

    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }

    for(u8 i = 0; i < len; i++)
    {
        if(i == (len - 1))
            data[i] = IIC_Read_Byte(0);		//������,����nACK

        else
            data[i] = IIC_Read_Byte(1);		//������,����ACK
    }

    IIC_Stop();
    return 0;
}

/**
 * @brief	��ȡ�¶�����
 *
 * @param   void
 *
 * @return  float	�¶����ݣ���λ�����϶ȣ�
 */
float AHT10_Read_Temperature(void)
{
    u8 res = 0;
    u8 cmd[2] = {0, 0};
    u8 temp[6];
    float cur_temp;

    res = AHT10_Write_Data(AHT10_GET_DATA, cmd, 2); //���Ͷ�ȡ��������

    if(res)	return 1;

    res = AHT10_Read_Data(temp, 6);				//��ȡ����

    if(res)	return 1;

    cur_temp = ((temp[3] & 0xf) << 16 | temp[4] << 8 | temp[5]) * 200.0 / (1 << 20) - 50;

    return cur_temp;
}

/**
 * @brief	��ȡʪ������
 *
 * @param   void
 *
 * @return  float	ʪ�����ݣ���λ��%RH��
 */
float AHT10_Read_Humidity(void)
{
    u8 res = 0;
    u8 cmd[2] = {0, 0};
    u8 humi[6];
    float cur_humi;

    res = AHT10_Write_Data(AHT10_GET_DATA, cmd, 2); //���Ͷ�ȡ��������

    if(res)	return 1;

    res = AHT10_Read_Data(humi, 6);				//��ȡ����

    if(res)	return 1;

    cur_humi = ((humi[1]) << 12 | humi[2] << 4 | (humi[3] & 0xF0)) * 100.0 / (1 << 20);

    return cur_humi;
}

/**
 * @brief	ATH10��������ʼ��
 *
 * @param   void
 *
 * @return  u8		0,��ʼ���ɹ�������,ʧ��
 */
u8 AHT10_Init(void)
{
    u8 res;
    u8 temp[2] = {0, 0};

    IIC_Init();		//��ʼ��IIC�ӿڣ�ע�������IIC����Ϊ��SCL-PD6 SDA-PC1

    res = AHT10_Write_Data(AHT10_NORMAL_CMD, temp, 2);

    if(res != 0)	return 1;

    delay_ms(300);

    temp[0] = 0x08;
    temp[1] = 0x00;
    res = AHT10_Write_Data(AHT10_CALIBRATION_CMD, temp, 2);

    if(res != 0)	return 1;

    delay_ms(300);

    return 0;
}


