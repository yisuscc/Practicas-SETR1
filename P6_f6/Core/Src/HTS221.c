/*******************************************
 * @breif	HTS221�����ļ�
 * @author	Mculover666(www.mculover666.cn)
 * @date	2019-12-27
 * @version	1.0.0
********************************************/
#include "main.h"
#include "HTS221.h"

extern I2C_HandleTypeDef hi2c2;

/* ���ù���ģʽ����ʼ��HTS221 */
void HTS221_Init()
{
	uint8_t cmd = 0;
	
	//���÷ֱ���
	cmd = 0x3F;
	HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDR_WR, 0x10, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0xFFFF);
	
	//���õ�Դ�����ݿ����ģʽ�������������
	cmd = 0x84;
	HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDR_WR, HTS221_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0xFFFF);
	
	//�������ݴ洢�鸴λģʽ���ر��ڲ�����
	cmd = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDR_WR, HTS221_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0xFFFF);
	
	//�ر������������ź�
	cmd = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDR_WR, HTS221_CTRL_REG3, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 0xFFFF);

}

/* HTS221����һ��ת�� */
static void HTS221_Start()
{
	uint8_t dat = 0;
	
	//��ȡREG2�Ĵ����е�ֵ����ֹ������Ϣ���ƻ�
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &dat, 1, 0xFFFF);
	
	//����һ��ת��
	dat |= 0x01;
	HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDR_WR, HTS221_CTRL_REG2, I2C_MEMADD_SIZE_8BIT, &dat, 1, 0xFFFF);

}

/* ����һ��ת������ȡУ�����¶�ֵ */
/* note����API��ȡ��ֵ��10������   */
uint8_t	HTS221_Get_Temperature(int16_t* temperature)
{
	uint8_t status_dat = 0;
	int16_t T0_degC, T1_degC;
	int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	uint8_t T0_degC_x8, T1_degC_x8, tmp;
	uint8_t buffer[4];
	int32_t tmp32;
	
	/*1. ��ȡT0_degC_x8 �� T1_degC_x8 У��ֵ */
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x32, I2C_MEMADD_SIZE_8BIT, &T0_degC_x8, 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x33, I2C_MEMADD_SIZE_8BIT, &T1_degC_x8, 1, 0xFFFF);
	
	/*2. ��ȡT1_degC �� T0_degC �����λ*/
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x35, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 0xFFFF);
	
	// ����T0_degC and T1_degC ֵ */
	T0_degC_x8_u16 = (((uint16_t)(tmp & 0x03)) << 8) | ((uint16_t)T0_degC_x8);
	T1_degC_x8_u16 = (((uint16_t)(tmp & 0x0C)) << 6) | ((uint16_t)T1_degC_x8);
	T0_degC = T0_degC_x8_u16>>3;
	T1_degC = T1_degC_x8_u16>>3;
	
	/*3. ��ȡ T0_OUT �� T1_OUT ֵ */
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x3C, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x3D, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x3E, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x3F, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	
	T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	T1_out = (((uint16_t)buffer[3])<<8) | (uint16_t)buffer[2];
	
	/* 4. ����ת�����ȴ���ɺ��ȡת�����ֵT_OUT */
	HTS221_Start();
	while(status_dat != 0x03)
	{
		HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status_dat, 1, 0xFFFF);
	}
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_TEMP_OUT_L, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_TEMP_OUT_H, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	
	T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	
	/* 5. ʹ�����Բ�ֵ�����㵱ǰ��Ӧ���¶�ֵ */
	tmp32 = ((int32_t)(T_out - T0_out)) * ((int32_t)(T1_degC - T0_degC)*10);
	*temperature = tmp32 /(T1_out - T0_out) + T0_degC*10;
	
	return 0;
}

/* ����һ��ת������ȡУ����ʪ��ֵ */
/* note����API��ȡ��ֵ��10������   */
uint8_t	HTS221_Get_Humidity(int16_t* humidity)
{
	uint8_t status_dat = 0;
	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t H0_rh, H1_rh;
	uint8_t buffer[2];
	int32_t tmp;
	
	
	/* 1. ��ȡH0_rH and H1_rH У��ֵ */
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x30, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x31, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	H0_rh = buffer[0] >> 1;
	H1_rh = buffer[1] >> 1;
	
	/*2. ��ȡ H0_T0_OUT У��ֵ */
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x36, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x37, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	H0_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];
	
	/*3. ��ȡ H1_T0_OUT У��ֵ */
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x3A, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, 0x3B, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	H1_T0_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*4. ����ת�����ȴ���ɺ��ȡת�����ֵ */
	HTS221_Start();
	while(status_dat != 0x03)
	{
		HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_STATUS_REG, I2C_MEMADD_SIZE_8BIT, &status_dat, 1, 0xFFFF);
	}
	
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_HUMIDITY_OUT_L, I2C_MEMADD_SIZE_8BIT, &buffer[0], 1, 0xFFFF);
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_RD, HTS221_HUMIDITY_OUT_H, I2C_MEMADD_SIZE_8BIT, &buffer[1], 1, 0xFFFF);
	H_T_out = (((uint16_t)buffer[1])<<8) | (uint16_t)buffer[0];

	/*5. ʹ�����Բ�ֵ������ʪ��ֵ RH [%] */
	tmp = ((int32_t)(H_T_out - H0_T0_out)) * ((int32_t)(H1_rh - H0_rh)*10);
	*humidity = (tmp/(H1_T0_out - H0_T0_out) + H0_rh*10);
	//ʪ����ֵ�˲�
	if(*humidity>1000)
	{		
		*humidity = 1000;
	}
	
	return 0;
}
