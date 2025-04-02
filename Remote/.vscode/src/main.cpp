#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TFT_eWidget.h>



//Initialize TFT
TFT_eSPI tft = TFT_eSPI();

//Initialize Sprites
TFT_eSprite knob = TFT_eSprite(&tft); // Sprite for the slide knob


//Initialize Widgets
SliderWidget s1 = SliderWidget(&tft, &knob);    // Slider 1 widget
SliderWidget s2 = SliderWidget(&tft, &knob);    // Slider 2 widget
SliderWidget s3 = SliderWidget(&tft, &knob);    // Slider 3 widget

//Set REPEAT_CAL to false to stop calivrating again
#define CALIBRATION_FILE "/calibrationData"
#define REPEAT_CAL false


//Variables
static uint16_t color;
uint16_t x, y;
uint16_t x2, y2;
int lasttouch = 0;

//Funktions
void touch_calibrate();


void setup(void) {
  uint16_t calibrationData[5];
  uint8_t calDataOK = 0;

  Serial.begin(115200);
  Serial.println("starting");

  tft.init();

  tft.setRotation(3);
  tft.fillScreen((TFT_BLACK));
  Serial.println("Screen filled");
  Serial.println("calibration run");

  // Calibrate the touch screen and retrieve the scaling factors
  if (REPEAT_CAL) {
    touch_calibrate();
    tft.fillScreen(TFT_BLACK);
  }

  // Create a parameter set for the slider
  slider_t param;

  // Slider slot parameters
  param.slotWidth = 9;           // Note: ends of slot will be rounded and anti-aliased
  param.slotLength = 200;        // Length includes rounded ends
  param.slotColor = TFT_BLUE;    // Slot colour
  param.slotBgColor = TFT_BLACK; // Slot background colour for anti-aliasing
  param.orientation = H_SLIDER;  // sets it "true" for horizontal

  // Slider control knob parameters (smooth rounded rectangle)
  param.knobWidth = 20;          // Always along x axis
  param.knobHeight = 20;         // Always along y axis
  param.knobRadius = 5;          // Corner radius
  param.knobColor = TFT_WHITE;   // Anti-aliased with slot backgound colour
  param.knobLineColor = TFT_WHITE; // Colour of marker line (set to same as knobColor for no line)

  // Slider range and movement speed
  param.sliderLT = 1;            // Left side for horizontal, top for vertical slider
  param.sliderRB = 270;          // Right side for horizontal, bottom for vertical slider
  param.startPosition = 50;      // Start position for control knob
  param.sliderDelay = 0;         // Microseconds per pixel movement delay (0 = no delay)

//Slider1
  // Create slider using parameters and plot at 0,0
  tft.setCursor(0, 0, 2);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.printf("Side");
  s1.drawSlider(0, 20, param);

  // Show bounding box (1 pixel outside slider working area)
  int16_t x, y;    // x and y can be negative
  uint16_t w, h;   // Width and height
  s1.getBoundingRect(&x, &y, &w, &h);     // Update x,y,w,h with bounding box
  tft.drawRect(x, y, w, h, TFT_DARKGREY); // Draw rectangle outline

  delay(1000);
  s1.setSliderPosition(50);
  delay(1000);
  s3.setSliderPosition(100);
  /*
//Slider2
  tft.setCursor(0, 50, 2);
  tft.printf("Shoulder");
  s2.drawSlider(0, 70, param);

  s2.getBoundingRect(&x, &y, &w, &h);
  tft.drawRect(x, y, w, h, TFT_DARKGREY);

  // Move slider under software control
  delay(1000);
  s2.setSliderPosition(50);
  delay(1000);
  s3.setSliderPosition(100);

//Slider3
  tft.setCursor(0, 100, 2);
  tft.printf("Knee");
  s3.drawSlider(0, 120, param);

  s3.getBoundingRect(&x, &y, &w, &h);
  tft.drawRect(x, y, w, h, TFT_DARKGREY);

  // Move slider under software control
  delay(1000);
  s3.setSliderPosition(50);
  delay(1000);
  s3.setSliderPosition(100);
  */
}

void loop() {
  /*
  if (tft.getTouch(&x, &y)) {
    Serial.println("touched");
    tft.setCursor(5, 5, 2);
    tft.printf("x: %i     ", x);
    tft.setCursor(5, 20, 2);
    tft.printf("y: %i    ", y);

    tft.drawPixel(x, y, TFT_BLACK);
    if(x2 != 0 && y2 != 0 && millis() - lasttouch < 100) {
      tft.drawLine(x, y, x2, y2, TFT_BLACK);
    }
    x2 = x;
    y2 = y;
    lasttouch = millis();
  }
  */
  static uint32_t scanTime = millis();
  uint16_t t_x = 9999, t_y = 9999; // To store the touch coordinates

  // Scan for touch every 50ms
  if (millis() - scanTime >= 20) {
    // Pressed will be set true if there is a valid touch on the screen
    if( tft.getTouch(&t_x, &t_y, 250) ) {
      if (s1.checkTouch(t_x, t_y)) {
        Serial.print("Slider 1 = "); Serial.println(s1.getSliderPosition());
      }
      if (s2.checkTouch(t_x, t_y)) {
        Serial.print("Slider 2 = "); Serial.println(s2.getSliderPosition());
      }
    }
    scanTime = millis();
  }
}

void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  // check file system exists
  if (!LittleFS.begin()) {
    Serial.println("Formating file system");
    LittleFS.format();
    LittleFS.begin();
  }

  // check if calibration file exists and size is correct
  if (LittleFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      LittleFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = LittleFS.open(CALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 14) == 14)
          calDataOK = 1;
        f.close();
      }
    }
  }

  if (calDataOK && !REPEAT_CAL) {
    // calibration data valid
    tft.setTouch(calData);
  } else {
    // data not valid so recalibrate
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(20, 0);
    tft.setTextFont(2);
    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.println("Touch corners as indicated");

    tft.setTextFont(1);
    tft.println();

    if (REPEAT_CAL) {
      tft.setTextColor(TFT_RED, TFT_BLACK);
      tft.println("Set REPEAT_CAL to false to stop this running again!");
    }

    tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.println("Calibration complete!");

    // store data
    File f = LittleFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}