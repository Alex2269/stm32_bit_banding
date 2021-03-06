#ifndef __BBAND_H
#define __BBAND_H

#define PA0 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (0*4))))
#define PA1 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (1*4))))
#define PA2 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (2*4))))
#define PA3 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (3*4))))
#define PA4 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (4*4))))
#define PA5 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (5*4))))
#define PA6 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (6*4))))
#define PA7 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (7*4))))
#define PA8 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (8*4))))
#define PA9 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (9*4))))
#define PA10 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (10*4))))
#define PA11 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (11*4))))
#define PA12 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (12*4))))
#define PA13 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (13*4))))
#define PA14 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (14*4))))
#define PA15 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1080C*32)) + (15*4))))
//
#define PB0 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (0*4))))
#define PB1 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (1*4))))
#define PB2 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (2*4))))
#define PB3 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (3*4))))
#define PB4 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (4*4))))
#define PB5 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (5*4))))
#define PB6 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (6*4))))
#define PB7 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (7*4))))
#define PB8 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (8*4))))
#define PB9 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (9*4))))
#define PB10 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (10*4))))
#define PB11 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (11*4))))
#define PB12 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (12*4))))
#define PB13 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (13*4))))
#define PB14 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (14*4))))
#define PB15 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x10C0C*32)) + (15*4))))

#define PC0 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (0*4))))
#define PC1 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (1*4))))
#define PC2 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (2*4))))
#define PC3 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (3*4))))
#define PC4 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (4*4))))
#define PC5 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (5*4))))
#define PC6 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (6*4))))
#define PC7 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (7*4))))
#define PC8 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (8*4))))
#define PC9 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (9*4))))
#define PC10 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (10*4))))
#define PC11 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (11*4))))
#define PC12 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (12*4))))
#define PC13 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (13*4))))
#define PC14 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (14*4))))
#define PC15 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1100C*32)) + (15*4))))

#define PD0 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (0*4))))
#define PD1 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (1*4))))
#define PD2 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (2*4))))
#define PD3 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (3*4))))
#define PD4 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (4*4))))
#define PD5 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (5*4))))
#define PD6 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (6*4))))
#define PD7 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (7*4))))
#define PD8 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (8*4))))
#define PD9 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (9*4))))
#define PD10 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (10*4))))
#define PD11 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (11*4))))
#define PD12 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (12*4))))
#define PD13 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (13*4))))
#define PD14 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (14*4))))
#define PD15 (*((volatile unsigned long *) ((PERIPH_BB_BASE + (0x1140C*32)) + (15*4))))

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

#endif /* __BBAND_H */
