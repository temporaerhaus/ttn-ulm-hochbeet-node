#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#ifndef HOCHBEET_DISPLAY_H
#define HOCHBEET_DISPLAY_H

#define MAX_DISPLAY_BUFFER_SIZE 15000ul // ~15k is a good compromise
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))\

GxEPD2_BW<GxEPD2_154, MAX_HEIGHT(GxEPD2_154)> epaper(GxEPD2_154(/*CS=4*/ 22, /*DC=*/ 13, /*RST=*/ 12, /*BUSY=*/ 11));

void updateEpaper(float temperature, int humidity, float tensio1, float tensio2, int tank, int bed, int pump, char time[]);
void setupDisplay();
void loopDisplay();

#endif
