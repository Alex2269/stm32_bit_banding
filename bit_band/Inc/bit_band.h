#ifndef __bit_band_H
#define __bit_band_H

/* bitband type */
typedef volatile uint32_t * const bitband_t;

/* base address for bit banding */
#define BITBAND_SRAM_REF              (0x20000000)
/* base address for bit banding */
#define BITBAND_SRAM_BASE             (0x22000000)
/* base address for bit banding */
#define BITBAND_PERIPH_REF            (0x40000000)
/* base address for bit banding */
#define BITBAND_PERIPH_BASE           (0x42000000)

/* sram bit band */
#define BITBAND_SRAM(address, bit)     ((void*)(BITBAND_SRAM_BASE +   \
                (((uint32_t)address) - BITBAND_SRAM_REF) * 32 + (bit) * 4))

/* periph bit band */
#define BITBAND_PERIPH(address, bit)   ((void *)(BITBAND_PERIPH_BASE + \
                (((uint32_t)address) - BITBAND_PERIPH_REF) * 32 + (bit) * 4))

void pin_mode(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint32_t GPIO_Mode)
{
  GPIO_InitTypeDef GPIO_InitStruct;
  // GPIOx Periph clock enable
  if(GPIOx==GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
  }
  else if(GPIOx==GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
  }
  else if(GPIOx==GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
  }
  else if(GPIOx==GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
  }
  else if(GPIOx==GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
  }
  GPIO_InitStruct.Pin = GPIO_Pin;
  GPIO_InitStruct.Mode = GPIO_Mode;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

#endif /* __bit_band_H */
