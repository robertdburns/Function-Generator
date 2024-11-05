/*
 * dac.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Robert
 */


void DAC_init(void) {                           // Initialize DAC with SPI
  // Enable GPIOC Clock
  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOAEN);

  // PA4 Set to General Output (CS)
  GPIOA->MODER &= ~(GPIO_MODER_MODE4);
  GPIOA->MODER |= (GPIO_MODER_MODE4_0);
  // PA5 (SCK), PA7 (MOSI) Set to Alt Function
  GPIOA->MODER   |= (GPIO_MODER_MODE5 | GPIO_MODER_MODE7);
  GPIOA->MODER   &= ~(GPIO_MODER_MODE5_0 | GPIO_MODER_MODE7_0);
  GPIOA->OTYPER  &= ~(GPIO_OTYPER_OT4 | GPIO_OTYPER_OT5 | GPIO_OTYPER_OT7);
  GPIOA->OSPEEDR |=  (GPIO_OSPEEDR_OSPEED4 | GPIO_OSPEEDR_OSPEED5
                        | GPIO_OSPEEDR_OSPEED7);
  GPIOA->PUPDR   &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD7);
  // Alt Function 5 For SPI1 Functionality on PA5 & PA7
  GPIOA->AFR[0]   &= ~(GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL7);
  GPIOA->AFR[0]   |= ((5 << GPIO_AFRL_AFSEL5_Pos) | (5 << GPIO_AFRL_AFSEL7_Pos));
  // Set Chip Select Low - Active Low
  GPIOA->ODR |= (GPIO_ODR_OD4);

  // SPI Registers Configuration
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR1 &= ~(SPI_CR1_BR);
  SPI1->CR1 |= (SPI_CR1_BR_0);                // Baud Rate f/4
  SPI1->CR1 |= SPI_CR1_CPOL;                  // CPOL = 0 Clock Polarity Low
  SPI1->CR1 |= (SPI_CR1_CPHA);                // CPHA = 1 Clock Phase
  SPI1->CR1 &= ~(SPI_CR1_RXONLY);             // Full-duplex Mode
  SPI1->CR1 &= ~(SPI_CR1_LSBFIRST);           // Set to MSB first
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;     // Set to Software control
  SPI1->CR1 |= SPI_CR1_MSTR;                  // Set as Controller
  SPI1->CR2 |= (15 << SPI_CR2_DS_Pos);        // 16-bit data size
  SPI1->CR1 |= SPI_CR1_SPE;                   // Enable SPI
}

void DAC_write(uint16_t inpVoltage) {         // Write a 12-bit value to the DAC

  // FIFO mask first 3 bits for DAC (1110XXXXDATAXXXX)
  const uint16_t FIFOEMPTYMASK = 14 << 11;
  GPIOA->ODR &= ~(GPIO_ODR_OD4);              // Set chip select high - active low
  SPI1->DR = inpVoltage |= FIFOEMPTYMASK;     // write value to DAC
  while (SPI1->SR & SPI_SR_BSY) {}            // Wait until data is fully transmitted
  GPIOA->ODR |= (GPIO_ODR_OD4);               // Set chip select back to low
}

uint16_t DAC_volt_conv(uint16_t inpVoltage) { // Convert a voltage value into a 12-bit value to control the DAC

  const uint16_t MAX12BITVAL = 4095;          // Max 12 bit val, max value of the input
  const uint16_t DACLIMITVOLTAGE = 3300;      // Max DAC volage (3300mV = 3.3V)

  // Return input voltage percentage of max 12 bit val as a percentage of 3.3V
  return ((inpVoltage * MAX12BITVAL) / DACLIMITVOLTAGE);
}
