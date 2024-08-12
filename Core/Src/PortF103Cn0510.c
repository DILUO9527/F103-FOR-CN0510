#include "spi.h"
#include "PortF103Cn0510.h"
#include "gpio.h"
#include "tim.h"

/******

basic code

********/
volatile uint8_t ucInterrupted = 0;


void AD5940_ReadWriteNBytes(unsigned char *pSendBuffer, unsigned char *pRecvBuffer, unsigned long length) {
  HAL_SPI_TransmitReceive(&hspi1, pSendBuffer, pRecvBuffer, length, HAL_MAX_DELAY);
 delay_us(1);//Ҫ����

}

void AD5940_CsClr(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}


void AD5940_CsSet(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}


void AD5940_RstSet(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 1); // ���� RST �������ӵ� PB0
}




void AD5940_RstClr(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, 0); // ���� RST �������ӵ� PB0
}


void AD5940_Delay10us(uint32_t time) {

    delay_us(10);//ʹ���˶�ʱ��4��1us������ʱ

}

uint32_t AD5940_GetMCUIntFlag(void) {
    return ucInterrupted;
}

uint32_t AD5940_ClrMCUIntFlag(void) {
    ucInterrupted = 0; // �������жϱ�־
  return 1;
}



uint32_t AD5940_MCUResourceInit(void *pCfg) {
    // ��ʼ�� GPIO
//    __HAL_RCC_GPIOA_CLK_ENABLE();
//    __HAL_RCC_GPIOB_CLK_ENABLE();

//    GPIO_InitTypeDef GPIO_InitStruct = {0};

//    // ��ʼ�� CS ���� (PA4)
//    GPIO_InitStruct.Pin = GPIO_PIN_4;
//    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//    AD5940_CsSet();

//    // ��ʼ�� RST ���� (�������ӵ� PB0)
//    GPIO_InitStruct.Pin = GPIO_PIN_0;
//    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//    AD5940_RstSet();

//    // ��ʼ�� SPI (�����Ѿ�ͨ�� HAL ���ʼ���� hspi1)

//    // ��ʼ���ⲿ�ж� (�������ӵ� PA0)
//    GPIO_InitStruct.Pin = GPIO_PIN_0;
//    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//    GPIO_InitStruct.Pull = GPIO_NOPULL;
//    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

//    // ʹ�� NVIC �е��ⲿ�ж�
//    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
//    HAL_NVIC_EnableIRQ(EXTI0_IRQn);


//��ʼ��exti reset_low����
MX_GPIO_Init();
return 1;
}




//void EXTI0_IRQHandler(void) {
//    if (__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_0) != RESET) {
//        __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
//        ucInterrupted = 1;
//    }
//}
//�жϻص���main.c



/******

run code

********/