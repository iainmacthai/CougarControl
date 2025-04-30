// CougarControlV3.ino

#include <Arduino.h>
#include <SPI.h>
#include <lvgl.h>               //Using LVGL with Arduino requires some extra steps: https://docs.lvgl.io/master/get-started/platforms/arduino.html
#include <TFT_eSPI.h>
#include "ui.h"
#include <XPT2046_Touchscreen.h>

#include "RGBledDriver.h"       //Onboard LED's

#include "unit_byte.hpp"        //Needed for M5Unit-ByteSwitch / ByteButton
#include "UNIT_8ENCODER.h"      //Needed for M5Unit-8Encoder
#include <Wire.h>               //I2C Bus

UnitByte deviceSwitch;          // ByteSwitch
const uint8_t switchId = 0x46;  // ByteSwitch

UnitByte deviceButton;          // ByteButton
const uint8_t buttonId = 0x47;  // ByteButton

UNIT_8ENCODER sensor;           // Unit_8Encoder

/*Global variables Encoder*/
int encoderValues[8] = {0};             // Sets Encoders to a value of zer0 (0)
uint8_t encoderbuttonStates[8] = {0};   // Stores raw encoder button states (0 = pressed)
bool encoderswitchState = false;        // Stores encoder switch state
int32_t prevEncoderRaw[8] = {0};        // Store previous raw values
/*Encoder Mapping Scale*/
const float ENCODER_SCALE = 0.5;        // 0.5 = 1 per click 1.0 - 2 per click (1.0 = 1:1 mapping, increase for more sensitivity)
/*Global variables switches and buttons*/
uint8_t switchStates[8] = {0};         // Stores switch states (0=Off, 1=On)
uint8_t buttonStates[8] = {0};         // Stores button states (1=Pressed, 0=Released)

// XPT2046_Touchscreen.h A library for interfacing with the touch screen: https://github.com/PaulStoffregen/XPT2046_Touchscreen
// The CYD touch uses some non default SPI Pins
// SPI pins
// ----------------------------
// Touch Screen pins
// ----------------------------
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// SPIClass mySpi = SPIClass(HSPI); // touch does not work with this setting
SPIClass mySpi = SPIClass(VSPI); // critical to get touch working
XPT2046_Touchscreen ts(XPT2046_CS, XPT2046_IRQ);

/*Change to your screen resolution*/
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];
TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushColors((uint16_t *)&color_p->full, w * h, true);
    tft.endWrite();
    lv_disp_flush_ready(disp_drv);
}

/*Read the touchpad*/
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    uint16_t touchX, touchY;
    bool touched = (ts.tirqTouched() && ts.touched()); // this is the version needed for XPT2046 touchscreen
    if (!touched)
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        // the following three lines are specific for using the XPT2046 touchscreen
        TS_Point p = ts.getPoint();
        touchX = map(p.x, 200, 3700, 1, screenWidth);  /* Touchscreen X calibration */
        touchY = map(p.y, 240, 3800, 1, screenHeight); /* Touchscreen X calibration */
        data->state = LV_INDEV_STATE_PR;
        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;
       // Serial.print("Data x ");
       // Serial.println(touchX);
       // Serial.print("Data y ");
       // Serial.println(touchY);
    }
}

void setup()
{
    Serial.begin(115200); 
    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();
    Serial.println(LVGL_Arduino);
    Serial.println("I am LVGL_Arduino");
    initRGBled();
    ChangeRGBColor(RGB_COLOR_1); //Red
    delay(500);
    ChangeRGBColor(RGB_COLOR_2); //Green
    delay(500);
    ChangeRGBColor(RGB_COLOR_3); //Blue 
    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif
    mySpi.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS); /* Start second SPI bus for touchscreen */
    ts.begin(mySpi);                                                  /* Touchscreen init */
    ts.setRotation(1);                                                /* Landscape orientation */
    tft.begin();        /* TFT init */
    tft.setRotation(1); // Landscape orientation  1 =  CYC usb on right, 2 for vertical
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);
    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);
    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
    /* Uncomment to create simple label */
    // lv_obj_t *label = lv_label_create( lv_scr_act() );
    // lv_label_set_text( label, "Hello Ardino and LVGL!");
    // lv_obj_align( label, LV_ALIGN_CENTER, 0, 0 );
    ui_init();

// Initialize Devices on I2C Bus
    Wire.begin(27, 22);
    /*Initialize Byte-Switch*/
    deviceSwitch.begin(&Wire, switchId, 27, 22, 100000);  // ByteSwitch
    deviceSwitch.setLEDShowMode(BYTE_LED_MODE_DEFAULT);
    for (int i = 0; i < 8; i++) {
        deviceSwitch.setSwitchOffRGB888(i, 0xFF0000);     // OFF RED
        deviceSwitch.setSwitchOnRGB888(i, 0x00FF00);      // ON GREEN
    }
    deviceSwitch.setFlashWriteBack();                     // SAVE SETTINGS
    /*Initialize Byte-Button*/
    deviceButton.begin(&Wire, buttonId, 27, 22, 100000);  // ByteButton
    deviceButton.setLEDShowMode(BYTE_LED_MODE_DEFAULT);
    for (int i = 0; i < 8; i++) {
        deviceButton.setSwitchOffRGB888(i, 0x00FF00);     // OFF GREEN - INVERTED LOGIC
        deviceButton.setSwitchOnRGB888(i, 0xFF0000);      // ON RED - INVERTED LOGIC
    }
    deviceButton.setFlashWriteBack();                     // SAVE SETTINGS
    /*Initialize Unit_8Encoder*/
    sensor.begin(&Wire, ENCODER_ADDR, 27, 22, 100000UL);  // UL denotes unsigned long
    Serial.println("Setup done");
}

// Function prototype declarations
// void TaskA();
// void TaskB();
// void TaskC();

/////////////////////////////////////////////  READ 8Ch Encoder //////////////////////// 
void ReadEncoder() {
    for(int i = 0; i < 8; i++) {
        int32_t currentRaw = sensor.getEncoderValue(i);
        int32_t delta = currentRaw - prevEncoderRaw[i];
        prevEncoderRaw[i] = currentRaw;
        // Update value with scaled delta and clamp
        encoderValues[i] = constrain(encoderValues[i] + (delta * ENCODER_SCALE), 0, 100);
    }
    // Read buttons and switch
    for(int i = 0; i < 8; i++) {
        encoderbuttonStates[i] = sensor.getButtonStatus(i);
    }
    encoderswitchState = sensor.getSwitchStatus();
}

///////////////////////////////////////////// READ 8Ch Byte Switch ////////////////////////
void ReadSwitches() {
    for (uint8_t i = 0; i < 8; i++) {
        switchStates[i] = deviceSwitch.getSwitchStatus(i);
    }
}

///////////////////////////////////////////// READ 8Ch Byte Button ////////////////////////
void ReadButtons() {
    for (uint8_t i = 0; i < 8; i++) {
        // Invert logic during read (0=Pressed becomes 1=Pressed)
        buttonStates[i] = 1 - deviceButton.getSwitchStatus(i);
    }
}

///////////////////////////////////////////// PRINT Byte Switch /////////////////////////
void PrintSwitches() {
    Serial.print("Byte - Switches: ");
        for (uint8_t i = 0; i < 8; i++) {
        Serial.print(switchStates[i]);
        Serial.print(i == 7 ? "\n" : ", ");
    }
}

///////////////////////////////////////////// PRINT Byte Button ///////////////////////
void PrintButtons() {
    Serial.print("Byte - Buttons:  ");
        for (uint8_t i = 0; i < 8; i++) {
        Serial.print(buttonStates[i]);
        Serial.print(i == 7 ? "\n" : ", ");
    }
}

///////////////////////////////////////////// PRINT 8Ch Encoder ////////////////////////
void PrintEncoder() {    
    Serial.print("Encoder Buttons: ");
    for(int i = 0; i < 8; i++) {
        Serial.print(encoderbuttonStates[i] ? "0" : "1"); // Invert display (0 = pressed)
        Serial.print(i == 7 ? "\n" : ", ");
    }
    Serial.print("Encoders: ");
    for(int i = 0; i < 8; i++) {
        Serial.print(encoderValues[i]);
        Serial.print(i == 7 ? "\n" : ", ");
    }
    Serial.print("Encoder Switch: ");
    Serial.println(encoderswitchState ? "ON" : "OFF");
    Serial.println("-------------------");
}

void UpdateUI() {
 ///////////////////////////////////////////// Switch 0 Main Contactor ////////////////////////       
            if (switchStates[0]) {
            lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);   // If switch 0 is ON then HIDE the white OFF indicator
            lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);  // If switch 0 is ON then SHOW the white ON indicator
            
                        
        } else {
            lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);    // If switch 0 is OFF then HIDE the white ON indicator
            lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN); // If switch 0 is OFF then SHOW the white OFF indicator
            lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set The PSU Indicator Led to Grey
          
        }
 ///////////////////////////////////////////// Switch 1 Start Button ////////////////////////       
            if (switchStates[1]) {             
            lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // If switch 1 is ON Set The PSU Indicator Led to Green
            
        } else {
            lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // If switch 1 is OFF Set The PSU Indicator Led to Grey
          
        }




       
}

void loop()
{
    lv_timer_handler();
    ReadEncoder();
    ReadSwitches();
    ReadButtons();
    UpdateUI(); 
    delay(5);
}
