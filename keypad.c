/*
 * keypad.c
 *
 *  Created on: Oct 30, 2024
 *      Author: Robert
 */

void keypad_init(void)
{
  // Enable GPIOC Clock
  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOCEN);
  // Configure keyboard column (output) pins: PC10, PC11, PC12
  GPIOC->MODER   &= ~(GPIO_MODER_MODE10 | GPIO_MODER_MODE11 | GPIO_MODER_MODE12);
  GPIOC->MODER   |=  ((1 << GPIO_MODER_MODE10_Pos) | (1 << GPIO_MODER_MODE11_Pos)
                        | (1 << GPIO_MODER_MODE12_Pos));
  // Configure keyboard row (input) pins: PC0, PC1, PC2, PC3
  GPIOC->MODER   &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
  GPIOC->PUPDR   &= ~(GPIO_PUPDR_PUPD0 | GPIO_PUPDR_PUPD1 | GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3);
  GPIOC->PUPDR   |= ((2 << GPIO_PUPDR_PUPD0_Pos) | (2 << GPIO_PUPDR_PUPD1_Pos)
                      | (2 << GPIO_PUPDR_PUPD2_Pos) | (2 << GPIO_PUPDR_PUPD3_Pos));
  // Set all columns high
  GPIOC->ODR |= ((1 << GPIO_ODR_OD10_Pos) | (1 << GPIO_ODR_OD11_Pos) | (1 << GPIO_ODR_OD12_Pos));
}

int16_t keypad_get_button(void)
{
  uint16_t curr_row;
  uint16_t curr_col;
  uint16_t IDR;
  IDR = (GPIOC->IDR & 15);
  if(GPIOC->IDR != 0)
  {
    for (curr_col = 0; curr_col < 3; curr_col++)
    {
      GPIOC->ODR &= ~(7 << GPIO_ODR_OD10_Pos);                // Set columns low
      GPIOC->ODR |= (1 << (curr_col + GPIO_ODR_OD10_Pos));
      for (curr_row = 0; curr_row < 4; curr_row++){
        IDR = GPIOC->IDR & (1 << (curr_row + GPIO_IDR_ID0_Pos));
        if(IDR != 0)
        {
          GPIOC->ODR |= ((1 << GPIO_ODR_OD10_Pos) | (1 << GPIO_ODR_OD11_Pos)
                          | (1 << GPIO_ODR_OD12_Pos));        // Set columns high
          uint8_t value = ((curr_row * 3) + curr_col + 1);    // Value to be returned
//          if (value > 10) {   // if value is double digits, return 0 instead
//            value = 0;
//          }
          return value;               // Return final keypad value
        }
      }
    }
  }
  return -1; // Return no-press
}
