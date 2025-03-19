#include <Arduino.h>

#define PWM_PIN PA2 // Define the pin for PWM



void setup() {
  pinMode(PWM_PIN, OUTPUT);   // sets the LED pin as output

  
}

void loop() {
  

  digitalWrite(PWM_PIN, HIGH);  // analogRead values go from 0
                                      // to 4095, pwmWrite values
                                      // from 0 to 65535, so scale roughly
  delay(10);
}

// Manual PWM set up  

/*
void setup() {
  // Enable clock for GPIOA and Timer 2
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;  // Enable GPIOA clock
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;  // Enable Timer 2 clock

  // Configure PA2 as alternate function push-pull
  GPIOA->CRL &= ~(GPIO_CRL_MODE2 | GPIO_CRL_CNF2);  // Reset PA2 configuration
  GPIOA->CRL |= GPIO_CRL_MODE2_1 | GPIO_CRL_MODE2_0;  // Output mode, max 50 MHz
  GPIOA->CRL |= GPIO_CRL_CNF2_1;  // Alternate function output push-pull

  // Configure Timer 2 for 16-bit PWM
  TIM2->PSC = 10;         // Prescaler: 72 MHz / (71 + 1) = 1 MHz timer clock
  TIM2->ARR = 65535;      // Auto-reload value for 16-bit resolution (0-65535)

  // Configure PWM mode on Channel 3 (PA2)
  TIM2->CCMR2 &= ~TIM_CCMR2_OC3M;    // Clear output compare mode
  TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2;  // PWM Mode 1
  TIM2->CCMR2 |= TIM_CCMR2_OC3PE;   // Preload enable

  // Enable PWM output on Channel 3 (PA2)
  TIM2->CCER |= TIM_CCER_CC3E;

  // Start the timer
  TIM2->CR1 |= TIM_CR1_ARPE;  // Auto-reload preload enable
  TIM2->CR1 |= TIM_CR1_CEN;   // Enable the timer
}

void loop() {
  // Gradually increase the duty cycle (16-bit range)
  for (uint16_t duty = 0; duty <= 40000; duty += 1000) {
    TIM2->CCR3 = duty;  // Set duty cycle (Channel 3)
    delay(30);          // Small delay for smooth transition
  }

  // // Gradually decrease the duty cycle
  for (uint16_t ruty = 40000; ruty > 0; ruty -= 1000) {
    TIM2->CCR3 = ruty;  // Set duty cycle (Channel 3)
    delay(30);          // Small delay for smooth transition
  }
}
*/
