#include "main.h"
#include "traffic_functions.h"
#include "traffic.h"
#include <spi.h>
#include <stdlib.h>

// send the isntruction to the shift register using SPI
void Send_Instruction(uint32_t instruction) {
    /* Make sure Reset and Enable are set to HIGH and LOW respectively */
    uint8_t *data = malloc(sizeof(uint8_t)*(3));
    
    data[0] = instruction;
    data[1] = instruction >> 8;
    data[2] = instruction >> 16;

    HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);
    
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_RESET);
    free(data);
}

// updates the current instruction according to which light group should be midified, while keeping eveyrthing else the same
uint32_t update_instruction(uint32_t current_instruction, uint32_t instruction, ledType led) {
    uint32_t zeroing;
    switch (led) {
    case TL:
        zeroing = ~(TL_EW_Red | TL_EW_Yellow | TL_EW_Green | TL_NS_Red | TL_NS_Yellow | TL_NS_Green);
        break;
    case PL:
        zeroing = ~(PL1_Green | PL1_Red | PL2_Green | PL2_Red);
        break;
    case PLB:
        zeroing = ~(PL1_Blue | PL2_Blue);
        break;
    default:
        break;
    }
    current_instruction &= zeroing; // save the other light states
    current_instruction |= instruction;
    Send_Instruction(current_instruction);
    return current_instruction;
}

