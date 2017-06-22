#include <stm32f0xx.h>
#include "usb_bsp.h"

void USB_BSP_Init(USB_CORE_HANDLE *pdev)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;
	RCC->CFGR3 |= RCC_CFGR3_USBSW;  	
}

/**
* @brief  Enable USB Global interrupt
* @param  None
* @retval None
*/
void USB_BSP_EnableInterrupt(USB_CORE_HANDLE *pdev)
{
  NVIC_EnableIRQ(USB_IRQn);
	NVIC_SetPriority(USB_IRQn,1);
}
