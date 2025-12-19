#ifndef INC_TEST_H_
#define INC_TEST_H_

#include "FreeRTOS.h"
#include "event_groups.h"
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "Traffic.h"
#include "cmsis_os.h"
#include "oled_functions.h"
#include "traffic_functions.h"


void Test_program(void);

void Test_Leds(void);

void Test_OLED(void);

#endif /* INC_TEST_H_ */

