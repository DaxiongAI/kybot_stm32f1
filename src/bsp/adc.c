//{{{ include
#include "adc.h"
//}}}

//{{{ define
#define ADC1_DR_Address         ((u32)0x4001244C)
#define BATTERY_CHANNEL         ADC_Channel_1  
#define BATTERY_ADC_GPIO        GPIOA  
#define BATTERY_ADC_PIN         GPIO_Pin_1

// ��������ֵ����λK������ʹ�ø���Ƿ���ͳһ�Ŵ���
// 40.2K +/-1%
#define BATTERY_RESI_H		402
// 10K +/-1%
#define BATTERY_RESI_L		100

__IO u16 ADC_ConvertedValue;
//}}}

//{{{ init
static void ADC1_GPIO_Config(void)
{
        GPIO_InitTypeDef GPIO_InitStructure;

        GPIO_InitStructure.GPIO_Pin = BATTERY_ADC_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
        GPIO_Init(BATTERY_ADC_GPIO, &GPIO_InitStructure);				
}

static void ADC1_Mode_Config(void)
{
        DMA_InitTypeDef DMA_InitStructure;
        ADC_InitTypeDef ADC_InitStructure;

        /* DMA channel1 configuration */
        DMA_DeInit(DMA1_Channel1);
        DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
        DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        DMA_InitStructure.DMA_BufferSize = 1;
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;
        DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
        DMA_Init(DMA1_Channel1, &DMA_InitStructure);

        /* Enable DMA channel1 */
        DMA_Cmd(DMA1_Channel1, ENABLE);

        /* ADC1 configuration */
        ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ ÿ��ADC��������
        ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ʹ��ɨ��ģʽ  scanλ����
        ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;// contλ���� ����ת��ģʽ
        ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;//EXTSEL ѡ����������ͨ����ת�����ⲿ�¼� ���ó����������
        ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//���ݶ��� �������λ�����   �������ó��Ҷ���
        ADC_InitStructure.ADC_NbrOfChannel = 1;	//����ͨ�����г��� ��Щλ����������ڹ���ͨ��ת�������е�ͨ����Ŀ 1��ת�� ָ���ɶ��ٸ�ͨ����ת��

        ADC_Init(ADC1, &ADC_InitStructure);

        /* ADC1 regular channel11 configuration */ 
        ADC_RegularChannelConfig(ADC1, BATTERY_CHANNEL, 1, ADC_SampleTime_239Cycles5);	//ת��ʱ����55.5������

        /* Enable ADC1 DMA */
        ADC_DMACmd(ADC1, ENABLE);

        /* Enable ADC1 */
        ADC_Cmd(ADC1, ENABLE);

        /* Enable ADC1 reset calibaration register */   
        ADC_ResetCalibration(ADC1);
        /* Check the end of ADC1 reset calibration register */
        while(ADC_GetResetCalibrationStatus(ADC1));

        /* Start ADC1 calibaration */
        ADC_StartCalibration(ADC1);
        /* Check the end of ADC1 calibration */
        while(ADC_GetCalibrationStatus(ADC1));

        /* Start ADC1 Software Conversion */ 
        ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}
void adc1_init(void)
{
        ADC1_GPIO_Config();
        ADC1_Mode_Config();
}
//}}}

//{{{ getter
// ��ȡ��ص�ѹ
// ��������ʹ�ø���˳���
// 0.1V
uint8_t get_battery_vol(void)
{
	uint32_t value = (uint32_t)ADC_ConvertedValue*33;
	return value*(BATTERY_RESI_H + BATTERY_RESI_L)/(BATTERY_RESI_L*4096);
}
// ��ȡ���adcת��ֵ
uint32_t get_battery_value(void)
{
	return ADC_ConvertedValue;
}
//}}}
