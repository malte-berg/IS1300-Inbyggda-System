#include "FreeRTOS.h"
#include "Test.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "Traffic.h"
#include "event_groups.h"
#include "cmsis_os.h"

void Test_program(void) {
    Test_Leds();
}

void Test_Leds(void) {
    /* Set instruction to turn on specific LEDS */
    int32_t instruction = 0x800000;

    for(int j = 0; j < 24; j++) {
        Send_Instruction(instruction);
        HAL_Delay(100);
        instruction = instruction >> 1;
    }
}

void Test_EventGroup(EventGroupHandle_t eventGroup) {
  /*
    EventBits_t receivedBits;
    receivedBits = xEventGroupWaitBits(eventGroup, (Event_PL1 | Event_PL2 | Event_TL1_Switch | Event_TL2_Switch | Event_TL3_Switch | Event_TL4_Switch), pdTRUE, pdFALSE, portMAX_DELAY);

    uint32_t instruction = 0x0;

    if(receivedBits & Event_PL1)
      instruction = PL1_Green;
    else if(receivedBits & Event_PL2)
      instruction = PL2_Green;
    else if(receivedBits & Event_TL1_Switch)
      instruction = TL1_Green;
    else if(receivedBits & Event_TL2_Switch)
      instruction = TL2_Green;
    else if(receivedBits & Event_TL3_Switch)
      instruction = TL3_Green;
    else if(receivedBits & Event_TL4_Switch)
      instruction = TL4_Green;

    Send_Instruction(instruction);
    */
}

