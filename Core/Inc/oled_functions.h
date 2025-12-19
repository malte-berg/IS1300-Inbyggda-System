#ifndef INC_OLED_FUNCTIONS_H_
#define INC_OLED_FUNCTIONS_H_

#include <stdint.h>
#include <stdbool.h>
#include "ssd1306.h"
#include "semphr.h"

void startBar(uint8_t barIndex, TickType_t duration);

void stopBar(uint8_t barIndex);

extern SemaphoreHandle_t oledMutex;

typedef struct {
    TickType_t startTick;
    TickType_t durationTick;
    uint8_t active;
} oledBarStruct;

#define NUM_BARS 8
extern oledBarStruct oledBars[NUM_BARS];

#define MAX_BAR_HEIGHT 50

// OLED is (0,0) top left => Higher Y = lower on screen

#define BAR_Y1 52 // bottom height of the bars
#define BAR_Y2 2  // top
#define BAR_X1 3
#define BAR_X2 13
#define BAR_PADDING 6

#define TEXT_Y 55

void drawTitles(void);
void drawBarOutlines(void);
void drawBars(uint8_t *screenArray);

#endif /* INC_OLED_FUNCTIONS_H_ */
