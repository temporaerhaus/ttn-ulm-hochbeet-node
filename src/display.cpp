// base class GxEPD2_GFX can be used to pass references or pointers to the display instance as parameter, uses ~1.2k more code
// enable or disable GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 1

#include "display.h"

const int buttonPin = 9;
int buttonState = 0;
int buttonDown = 0;

void setupDisplay()
{
    epaper.init(115200);
    // first update should be full refresh

    epaper.setRotation(1);
    epaper.setFont(&FreeMonoBold9pt7b);
    epaper.setTextColor(GxEPD_BLACK);

    char time[] = "in 4:30h";
    updateEpaper(25.8, 50, 77.0, 99.0, 1, 0, 0, time);
    delay(2000);

    pinMode(buttonPin, INPUT);
}

void loopDisplay() {
    //*******************
    // Button press
    //*******************
    delay(10); // needed?
    buttonState = digitalRead(buttonPin);
    if (buttonState == HIGH) {
        buttonDown = 1;
    } else if (buttonDown == 1 && buttonState == LOW) { // on release
        Serial.println("Button released");
        buttonDown = 0;
        //char time[] = "in 4:10h";
        //updateEpaper(25.8, 23, 88.0, 99.0, 1, 0, 0, time);
    }

}

void updateEpaper(float temperature, int humidity, float tensio1, float tensio2, int tank, int bed, int pump, char time[]) {

    int16_t tbx, tby;
    uint16_t tbw, tbh;

    //uint16_t leftStart = 10;
    uint16_t rightStart = (epaper.width() / 2) + 10;

    //**********************
    // Line 1 + 2
    //**********************
    char temperatureTitle[] = "Temp.";
    epaper.getTextBounds(temperatureTitle, 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t l1lx = 10;
    uint16_t l1ly = 15;

    char tempStr[6]; // 4 letters, example 25.8
    sprintf(tempStr, "%4.1f", temperature);
    //display.getTextBounds(tempStr, 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t l2lx = 10;
    uint16_t l2ly = l1ly + tbh + 4; // 4 margin

    char humidityTitle[] = "Hum.";
    uint16_t l1rx = rightStart;
    uint16_t l1ry = 15;

    char humidityStr[6];
    sprintf(humidityStr, "%i%%", humidity);
    uint16_t l2rx = rightStart;
    uint16_t l2ry = l1ly + tbh + 4; // 4 margin


    //**********************
    // Line 3 + 4
    //**********************

    char tensio1Title[] = "Tensio1";
    //display.getTextBounds(tensio1Title, 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t l3lx = 10;
    uint16_t l3ly = l2ry + tbh + 12;

    char tensio1str[6]; // 4 letters, example 25.8
    sprintf(tensio1str, "%4.1f", tensio1);
    //display.getTextBounds(tensio1str, 0, 0, &tbx, &tby, &tbw, &tbh);
    uint16_t l4lx = 10;
    uint16_t l4ly = l3ly + tbh + 4; // 4 margin


    char tensio2Title[] = "Tensio2";
    uint16_t l3rx = rightStart;
    uint16_t l3ry = l2ry + tbh + 12;

    char tensio2Str[6];
    sprintf(tensio2Str, "%4.1f", tensio2);
    uint16_t l4rx = rightStart;
    uint16_t l4ry = l3ry + tbh + 4; // 4 margin


    //**********************
    // Line 5 + 6
    //**********************
    char tankTitle[] = "Tank";
    uint16_t l5lx = 10;
    uint16_t l5ly = l4ly + tbh + 12;

    char tankStr[6];
    if (tank == 1) {
        strncpy(tankStr, "An", 6);
    } else {
        strncpy(tankStr, "Aus", 6);
    }
    uint16_t l6lx = 10;
    uint16_t l6ly = l5ly + tbh + 4; // 4 margin


    char bedTitle[] = "Beet";
    uint16_t l5rx = rightStart;
    uint16_t l5ry = l4ry + tbh + 12;

    char bedStr[6];
    if (bed == 1) {
        strncpy(bedStr, "An", 6);
    } else {
        strncpy(bedStr, "Aus", 6);
    }
    uint16_t l6rx = rightStart;
    uint16_t l6ry = l5ry + tbh + 4; // 4 margin



    //**********************
    // Line 7 + 8
    //**********************
    char pumpTitle[] = "Pumpe";
    uint16_t l7lx = 10;
    uint16_t l7ly = l6ly + tbh + 12;

    char pumpStr[6];
    if (pump == 1) {
        strncpy(pumpStr, "An", 6);
    } else {
        strncpy(pumpStr, "Aus", 6);
    }
    uint16_t l8lx = 10;
    uint16_t l8ly = l7ly + tbh + 4; // 4 margin


    char timeTitle[] = "Next";
    uint16_t l7rx = rightStart;
    uint16_t l7ry = l6ry + tbh + 12;

    char timeStr[8]; // 14:14:00
    strncpy(timeStr, time, 8);
    uint16_t l8rx = rightStart;
    uint16_t l8ry = l7ry + tbh + 4; // 4 margin



    // render
    epaper.setFullWindow();
    epaper.firstPage();
    do {
        epaper.fillScreen(GxEPD_WHITE);

        // line 1 + 2
        // temp
        epaper.setCursor(l1lx, l1ly);
        epaper.print(temperatureTitle);
        epaper.setCursor(l2lx, l2ly);
        epaper.print(tempStr);
        // hum
        epaper.setCursor(l1rx, l1ry);
        epaper.print(humidityTitle);
        epaper.setCursor(l2rx, l2ry);
        epaper.print(humidityStr);

        // line 3 + 4
        // tensio1
        epaper.setCursor(l3lx, l3ly);
        epaper.print(tensio1Title);
        epaper.setCursor(l4lx, l4ly);
        epaper.print(tensio1str);
        // tensio2
        epaper.setCursor(l3rx, l3ry);
        epaper.print(tensio2Title);
        epaper.setCursor(l4rx, l4ry);
        epaper.print(tensio2Str);

        // line 5 + 6
        // tank
        epaper.setCursor(l5lx, l5ly);
        epaper.print(tankTitle);
        epaper.setCursor(l6lx, l6ly);
        epaper.print(tankStr);
        // bed
        epaper.setCursor(l5rx, l5ry);
        epaper.print(bedTitle);
        epaper.setCursor(l6rx, l6ry);
        epaper.print(bedStr);

        // line 7 + 8
        // pump
        epaper.setCursor(l7lx, l7ly);
        epaper.print(pumpTitle);
        epaper.setCursor(l8lx, l8ly);
        epaper.print(pumpStr);
        // time
        epaper.setCursor(l7rx, l7ry);
        epaper.print(timeTitle);
        epaper.setCursor(l8rx, l8ry);
        epaper.print(timeStr);


    } while (epaper.nextPage());
}