
  while (1)
  {

    float j;
    for(j=70;j>20;j-=0.5)
    {
     GPIOA->ODR = 0x55;
     DelayMicro(j*100);
     GPIOA->ODR = 0xAA;
     DelayMicro(j*100);
    }
    for(j=20;j<70;j+=0.5)
    {
     GPIOA->ODR = 0x55;
     DelayMicro(j*100);
     GPIOA->ODR = 0xAA;
     DelayMicro(j*100);
    }
    GPIOA->ODR = 0xFF;

    for(j=70;j>20;j-=0.5)
    {
     GPIOB->ODR = 0x55;
     DelayMicro(j*100);
     GPIOB->ODR = 0xAA;
     DelayMicro(j*100);
    }
    for(j=20;j<70;j+=0.5)
    {
     GPIOB->ODR = 0x55;
     DelayMicro(j*100);
     GPIOB->ODR = 0xAA;
     DelayMicro(j*100);
    }
    GPIOB->ODR = 0xFF;

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
