//Mac Adresse Remote: 30:C9:22:FF:71:F4

#include "FS.h"
#include <SPI.h>
#include <TFT_eSPI.h>
#include <Arduino.h>
#include <lvgl.h>
#include <ui.h>
#include <funktions.h>
#include <LoRa.h>
#include <esp_sleep.h>


//Battery
#define VBAT_PIN 34

//runEvery lastruntimes
unsigned long lastRunTimeBattery = 0;
unsigned long lastRunTimeController = 0;
unsigned long nextLvTimerRun = 0;

//Set REPEAT_CAL to false to stop calibrating again
static const String CALIBRATION_FILE =  "/calibrationData";
static const bool REPEAT_CAL = false;

/*screen resolution*/
static const uint16_t screenWidth  = 480;
static const uint16_t screenHeight = 320;

//buffers lvgl
void* lv_mem_pool;
#define LV_BUF_SIZE (screenWidth * screenHeight) // Define the size of the screen buffer

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];
void* buf1;
void* buf2;


TFT_eSPI tft = TFT_eSPI(screenHeight, screenWidth); /* TFT instance */
// SPIClass tftSPI = SPIClass(VSPI);

//Joystick

#define xPinRight A4  // xPin Joystick A
#define yPinRight A3  // yPin Joystick A
#define yPinLeft A0  // yPin Joystick B
#define xPinLeft A1  // xPin Joystick B
uint8_t joyLeft = 0;
uint8_t joyRight = 0;

unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 100; // Interval in milliseconds

// LoRa
#define loraDIO0 D3
#define loraCS 21
#define loraFrequency 433E6
SPIClass SPILora = SPIClass(HSPI);

//Variables
uint16_t posFLS = 0;
uint16_t posFLT = 0;
uint16_t posFLB = 0;
uint16_t posFRS = 0;
uint16_t posFRT = 0;
uint16_t posFRB = 0;
uint16_t posBRS = 0;
uint16_t posBRT = 0;
uint16_t posBRB = 0;
uint16_t posBLS = 0;
uint16_t posBLT = 0;
uint16_t posBLB = 0;
uint16_t lastPosCheck = 0;


//Variables Communication
const uint8_t sit = 1;
const uint8_t stand = 2;
const uint8_t codeHump = 3;
const uint8_t FLS = 10;
const uint8_t FLT = 11;
const uint8_t FLB = 12;
const uint8_t FRS = 20;
const uint8_t FRT = 21;
const uint8_t FRB = 22;
const uint8_t BRS = 30;
const uint8_t BRT = 31;
const uint8_t BRB = 32;
const uint8_t BLS = 40;
const uint8_t BLT = 41;
const uint8_t BLB = 42;
const uint8_t isStanding = 69;
const uint8_t gimmePosLegs = 250;
const uint8_t reset = 255;
int msgArray[2];
bool warningON = false;

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p );

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data );

//Funktions
void touch_calibrate();
void manageSend();
bool is_screen_active(lv_obj_t *screen) { return lv_scr_act() == screen; }


void setup()
{
  Serial.begin( 9600 ); /* prepare for possible serial debug */
  // Serial2.begin( 9600 ); //connection to second esp

    //Joystick setup
  pinMode(xPinRight, INPUT);
  pinMode(yPinRight, INPUT);
  pinMode(xPinLeft, INPUT);
  pinMode(yPinLeft, INPUT);
  pinMode(VBAT_PIN, INPUT_PULLDOWN);

  
  tft.begin();          /* TFT init */
  tft.setRotation( 1 ); /* Landscape orientation, flipped */

  tft.fillScreen(TFT_BLACK);
  tft.setCursor(20, 0);
  tft.setTextFont(2);
  tft.setTextSize(3);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.println("Starting...");


  touch_calibrate();

  // //Lora Setup
  delay(1000);
  Serial.println("LoRa Start");
  LoRa.setSPI(SPILora);
  LoRa.setPins( loraCS , -1 , loraDIO0 );
  delay(500);
  if (!LoRa.begin(loraFrequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);    
  }
  Serial.println("LoRa init succeeded.");

  // Check if PSRAM is available
  if (!psramFound()) {
      Serial.println("PSRAM not found");
      while (true); // Halt execution if PSRAM is not found
  }

  // Allocate memory from PSRAM
  lv_mem_pool = ps_malloc(LV_MEM_SIZE);
  if (lv_mem_pool == NULL) {
      Serial.println("Failed to allocate memory from PSRAM");
      while (true); // Halt execution if allocation fails
  } else {
      Serial.println("Memory allocated from PSRAM");
  }

  // Allocate screen buffers from PSRAM
  buf1 = ps_malloc(LV_BUF_SIZE * sizeof(lv_color_t));
  buf2 = ps_malloc(LV_BUF_SIZE * sizeof(lv_color_t));
  if (buf1 == NULL || buf2 == NULL ) { // || buf2 == NULL
      Serial.println("Failed to allocate screen buffers from PSRAM");
      while (true); // Halt execution if allocation fails
  } else {
      Serial.println("Screen buffers allocated from PSRAM");
  }

  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  Serial.println( LVGL_Arduino );
  Serial.println( "I am LVGL_Arduino" );

  lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

  /*Initialize the display buffer*/
  lv_disp_draw_buf_init( &draw_buf, buf1, buf2 , LV_BUF_SIZE);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init( &disp_drv );
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register( &disp_drv );

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init( &indev_drv );
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register( &indev_drv );


  ui_init();

  Serial.println( " LVGL Setup done, starting ESP-Now" );

  Serial.println("Setup done");
}

//LOOP /////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  // parse for a packet, and call onReceive with the result:
  int* message = stringToIntArray(onReceive(LoRa.parsePacket()));
  unsigned long currentTime = millis();

  if(message[0] != 0){

    Serial.printf("%d %d \n", message[0], message[1]);

    switch(message[0])
    {
      case FLS:
        lv_slider_set_value(ui_flSideSlider, message[1], LV_ANIM_OFF);
        // Serial.println("FLS");
        break;
      case FLT:
        lv_slider_set_value(ui_flTopSlider, message[1], LV_ANIM_OFF);
        // Serial.println("FLt");
        break;
      case FLB:
        lv_slider_set_value(ui_flBotSlider, message[1], LV_ANIM_OFF);
        // Serial.println("FLb");
        break;
      case FRS:
        lv_slider_set_value(ui_flSideSlider2, message[1], LV_ANIM_OFF);
        // Serial.println("FrS");
        break;
      case FRT: 
        lv_slider_set_value(ui_flTopSlider2, message[1], LV_ANIM_OFF);
        // Serial.println("Frt");
        break;
      case FRB:
        lv_slider_set_value(ui_flBotSlider2, message[1], LV_ANIM_OFF);
        // Serial.println("Frb");
        break;
      case BLS:
        lv_slider_set_value(ui_flSideSlider1, message[1], LV_ANIM_OFF);
        // Serial.println("bLS");
        break;
      case BLT:
        lv_slider_set_value(ui_flTopSlider1, message[1], LV_ANIM_OFF);
        // Serial.println("blt");
        break;
      case BLB:
        lv_slider_set_value(ui_flBotSlider1, message[1], LV_ANIM_OFF);
        // Serial.println("blb");
        break;
      case BRS:
        lv_slider_set_value(ui_flSideSlider3, message[1], LV_ANIM_OFF);
        // Serial.println("brs");
        break;
      case BRT:
        lv_slider_set_value(ui_flTopSlider3, message[1], LV_ANIM_OFF);   
        // Serial.println("brt");
        break;
      case BRB:
        lv_slider_set_value(ui_flBotSlider3, message[1], LV_ANIM_OFF);  
        break;
      default:
        break;
        ;
    }
  }

  if(runEvery(10000, lastRunTimeBattery))
  {
    int rawValue = analogRead(VBAT_PIN);  // Read the raw ADC value
    Serial.println(rawValue);

    // Convert the raw ADC value to the actual battery voltage
    float voltage = rawValue * (3.3 / 4095.0) * 2;  // Scale to the actual battery voltage

    Serial.print("Battery Voltage: ");
    Serial.print(voltage);
    Serial.println(" V");

    // Check for overdischarge
    if (voltage < 3.0) {
        //Serial.println("Warning: Battery voltage is below the overdischarge detection voltage!");
        showBatteryWarning("Achtung Batterie leer, Gerät schaltet sich ab.");
        delay(5000);
        esp_deep_sleep_start();
    }
    // Calculate the battery percentage
    int percentage = (voltage - 3.0) / (4.2 - 3.0) * 100;
    if (percentage > 100) percentage = 100;
    if (percentage < 0) percentage = 0;
    Serial.println(percentage);
    update_battery_status(percentage);

  }

  if(is_screen_active(ui_WalkScreen))
  {
    if(message[0] == 200)
    {
      lv_bar_set_value(ui_DistanceBar, message[1], LV_ANIM_ON);
      if(message[1] <= 15)
      {
        show_warning_label("Colission Warning! \n Back up!");
        warningON = true;
      }else if(warningON && (message[1] > 17))
      {
        lv_obj_del(warning_container);
      }
    }

    if(runEvery(500, lastRunTimeController))
    {
      // Serial.println("Read JoystickRight\n");
      // Serial.printf("xPin: %d, yPin: %d", analogRead(xPinRight), analogRead(yPinRight));
      joyLeft = readJoystick(xPinRight,yPinRight);
      joyLeft += 10;
      // Serial.println("Read JoystickLeft\n");
      // Serial.printf("xPin: %d, yPin: %d", analogRead(xPinLeft), analogRead(yPinLeft));
      joyRight = readJoystick(xPinLeft,yPinLeft);
      joyRight += 10;
      manageSend(joyLeft, joyRight);
    }
  }
  //reset message
  message = 0;

  //LVGL
    // Run LVGL timer handler only when needed
  if (currentTime >= nextLvTimerRun) {
    nextLvTimerRun = currentTime + lv_timer_handler();
  }
  delay(5);
}

// TFT Screen /////////////////////////////////////////////////////////////////////////////////////
void touch_calibrate()
{
  uint16_t calData[5];
  uint8_t calDataOK = 0;

  Serial.println("Starting cal");
  tft.fillScreen(TFT_BLACK);

  // check file system
  if (!SPIFFS.begin()) {
    Serial.println("formatting file system");

    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(CALIBRATION_FILE)) {
    if (REPEAT_CAL)
    {
      // Delete if we want to re-calibrate
      SPIFFS.remove(CALIBRATION_FILE);
    }
    else
    {
      File f = SPIFFS.open(CALIBRATION_FILE, "r");
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
    File f = SPIFFS.open(CALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 14);
      f.close();
    }
  }
}

void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;

    if( !tft.getTouch( &touchX, &touchY, 600 ) )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

    }
}

void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );


    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

//Event functions/////////////////////////////////////////////////////////////////////////////////////////////

void getPositionLegs(lv_event_t * e)
{

  LoRa_sendMessage(String(gimmePosLegs));
}

void sendHump(lv_event_t * e)
{
  LoRa_sendMessage(String(codeHump));
}

//Positions/////////////////////////////////////////////////////////////////////////////////////////////////////
void sendSit1(lv_event_t * e){
  Serial.println("Sit1");
  LoRa_sendMessage(String(sit));
}

void standSend1(lv_event_t * e){
  Serial.println("Stand1");
  LoRa_sendMessage(String(stand));
}


void resetPositionDog(lv_event_t * e){
  Serial.println("Reset");
}


//LEGS////////////////////////////////////////////////////////////////////////////////////////////////////////
//Front left leg /////////////////////////////////////////////////////////////////////////////////////////////
void flSideChangeVal(lv_event_t * e)
{
  Serial.printf("posFLS: %d\n", lv_slider_get_value(e->target));
  posFLS = lv_slider_get_value(e->target);
  msgArray[0] = FLS;
  msgArray[1] = posFLS;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void flTopChangeVal(lv_event_t * e)
{
  Serial.printf("posFLT: %d\n", lv_slider_get_value(e->target));
  posFLT = lv_slider_get_value(e->target);
  msgArray[0] = FLT;
  msgArray[1] = posFLT;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void flBotChangeVal(lv_event_t * e)
{
  Serial.printf("posFLB: %d\n", lv_slider_get_value(e->target));
  posFLB = lv_slider_get_value(e->target);
  msgArray[0] = FLB;
  msgArray[1] = posFLB;
  LoRa_sendMessage(intArraytoString(msgArray));
}

//Front right leg /////////////////////////////////////////////////////////////////////////////////////////////
void frSideChangeVal(lv_event_t * e)
{
  Serial.printf("posFRS: %d\n", lv_slider_get_value(e->target));
  posFRS = lv_slider_get_value(e->target);
  msgArray[0] = FLB;
  msgArray[1] = posFLB;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void frTopChangeVal(lv_event_t * e)
{
  Serial.printf("posFRT: %d\n", lv_slider_get_value(e->target));
  posFRT = lv_slider_get_value(e->target);
  msgArray[0] = FRT;
  msgArray[1] = posFRT;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void frBotChangeVal(lv_event_t * e)
{
  Serial.printf("posFRB: %d\n", lv_slider_get_value(e->target));
  posFRB = lv_slider_get_value(e->target);
  msgArray[0] = FRB;
  msgArray[1] = posFRB;
  LoRa_sendMessage(intArraytoString(msgArray));
}


//Back left leg/////////////////////////////////////////////////////////////////////////////////////////////

void blSideChangeVal(lv_event_t * e)
{
  Serial.printf("posBLS: %d\n", lv_slider_get_value(e->target));
  posBLS = lv_slider_get_value(e->target);
  msgArray[0] = BLS;
  msgArray[1] = posBLS;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void blTopChangeVal(lv_event_t * e)
{
  Serial.printf("posBLT: %d\n", lv_slider_get_value(e->target));
  posBLT = lv_slider_get_value(e->target);
  msgArray[0] = BLT;
  msgArray[1] = posBLT;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void blBotChangeVal(lv_event_t * e)
{
  Serial.printf("posBLB: %d\n", lv_slider_get_value(e->target));
  posBLB = lv_slider_get_value(e->target);
  msgArray[0] = BLB;
  msgArray[1] = posBLB;
  LoRa_sendMessage(intArraytoString(msgArray));
}


//Back right leg/////////////////////////////////////////////////////////////////////////////////////////////
void brSideChangeVal(lv_event_t * e)
{
  Serial.printf("posBRS: %d\n", lv_slider_get_value(e->target));
  posBRS = lv_slider_get_value(e->target);
  msgArray[0] = BRS;
  msgArray[1] = posBRS;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void brTopChangeVal(lv_event_t * e)
{
  Serial.printf("posBRT: %d\n", lv_slider_get_value(e->target));
  posBRT = lv_slider_get_value(e->target);
  msgArray[0] = BRT;
  msgArray[1] = posBRT;
  LoRa_sendMessage(intArraytoString(msgArray));
}

void brBotChangeVal(lv_event_t * e)
{
  Serial.printf("posBRB: %d\n", lv_slider_get_value(e->target));
  posBRB = lv_slider_get_value(e->target);
  msgArray[0] = BRB;
  msgArray[1] = posBRB;
  LoRa_sendMessage(intArraytoString(msgArray));
}

