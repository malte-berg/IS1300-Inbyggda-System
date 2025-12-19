#include "Test.h"

void Test_program(void) {
    Test_Leds();
    Test_OLED();
    HAL_Delay(500);
}

void Test_Leds(void) {
    HAL_GPIO_WritePin(SR_Reset_GPIO_Port, SR_Reset_Pin, GPIO_PIN_SET);
    /* Set instruction to turn on specific LEDS */
    int32_t instruction = 0x800000;

    for(int j = 0; j < 24; j++) {
        Send_Instruction(instruction);
        HAL_Delay(200);
        instruction = instruction >> 1; // Loop through all leds
    }
}

void Test_OLED(void) {
  uint8_t bar_vals[8];
  for (int i = BAR_Y1; i >= BAR_Y2; i--) {
    for(int j = 0; j < 8; j++) {
      bar_vals[j] = i;
    }
    drawBars(bar_vals);
    HAL_Delay(10);
  }
  for (int i = BAR_Y2; i <= BAR_Y1; i++) {
    for(int j = 0; j < 8; j++) {
      bar_vals[j] = i;
    }
    drawBars(bar_vals);
    HAL_Delay(10);
  }
  for(int j = 0; j < 8; j++) {
    for (int i = BAR_Y1; i >= BAR_Y2; i--) {
        bar_vals[j] = i;
        drawBars(bar_vals);
        HAL_Delay(10);
    }
  }
  for(int j = 0; j < 8; j++) {
    for (int i = BAR_Y2; i <= BAR_Y1; i++) {
        bar_vals[j] = i;
        drawBars(bar_vals);
        HAL_Delay(10);
    }
  }
}

