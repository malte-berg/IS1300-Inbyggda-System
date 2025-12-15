#include "main.h"
#include "traffic_functions.h"
#include "traffic.h"

// send the isntruction to the shift register using SPI
void Send_Instruction(uint32_t instruction) {
    /* Make sure Reset and Enable are set to HIGH and LOW respectively */
    HAL_GPIO_WritePin(SR_Reset_GPIO_Port, SR_Reset_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_Enable_GPIO_Port, SR_Enable_Pin, GPIO_PIN_RESET);

    for(int i = 0; i < 24; i++) {
        bool lsb = instruction & 1;
        HAL_GPIO_WritePin(SR_DS_GPIO_Port, SR_DS_Pin, lsb ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(SR_SHCP_GPIO_Port, SR_SHCP_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(SR_SHCP_GPIO_Port, SR_SHCP_Pin, GPIO_PIN_RESET);
        instruction = instruction >> 1;
    }
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_STCP_GPIO_Port, SR_STCP_Pin, GPIO_PIN_RESET);
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