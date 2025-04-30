// CougarControlV3.ino

#include <Arduino.h>
#include <SPI.h>
#include <lvgl.h>                               //Using LVGL with Arduino requires some extra steps: https://docs.lvgl.io/master/get-started/platforms/arduino.html
#include <TFT_eSPI.h>
#include "ui.h"
#include <XPT2046_Touchscreen.h>
#include "RGBledDriver.h"                       //Onboard LED's
#include "unit_byte.hpp"                        //Needed for M5Unit-ByteSwitch / ByteButton
#include "UNIT_8ENCODER.h"                      //Needed for M5Unit-8Encoder
#include <Wire.h>                               //I2C Bus

UnitByte deviceSwitch;                          // ByteSwitch
const uint8_t switchId = 0x46;                  // ByteSwitch
UnitByte deviceButton;                          // ByteButton
const uint8_t buttonId = 0x47;                  // ByteButton
UNIT_8ENCODER sensor;                           // Unit_8Encoder

/*Global variables Encoder*/
int encoderValues[8] = {0};                     // Sets Encoders to a value of zer0 (0)
uint8_t encoderbuttonStates[8] = {0};           // Stores raw encoder button states (0 = pressed)
bool encoderswitchState = false;                // Stores encoder switch state
int32_t prevEncoderRaw[8] = {0};                // Store previous raw values
/*Encoder Mapping Scale*/
const float ENCODER_SCALE = 0.5;                // 0.5 = 1 per click 1.0 - 2 per click (1.0 = 1:1 mapping, increase for more sensitivity)
/*Global variables switches and buttons*/
uint8_t switchStates[8] = {0};                  // Stores switch states 
uint8_t buttonStates[8] = {0};                  // Stores button states (1=Pressed, 0=Released)

// Define system states based on the truth table
enum SystemState {
  SYSTEM_SHUTDOWN,
  MAIN_CONTACTOR_TRIPPED,  // New state Modification #1
  AC_POWER_AVAILABLE,
  AC_POWER_STARTED,
  AC_SURFACE_ON,
  AC_VEHICLE_ON,
  DC1_SELECTED,
  DC2_SELECTED,
  DC1_AND_2_SELECTED,
  MASTER_DC_ON_THRUSTERS_DISABLED,
  MASTER_DC_ON_DC1_ONLY,
  MASTER_DC_ON_DC2_ONLY,
  THRUSTERS_ENABLED_DC1_ONLY,
  THRUSTERS_ENABLED_DC2_ONLY,
  THRUSTERS_ENABLED_SYSTEM_READY,
  UNKNOWN_STATE
};

// Function to return state name as a string
const char* getStateName(SystemState state) {
  switch (state) {
    case SYSTEM_SHUTDOWN: return "System Shutdown";
    case AC_POWER_AVAILABLE: return "AC Power Available";
    case MAIN_CONTACTOR_TRIPPED: return "Main Contactor Tripped";  // New case Modification #1
    case AC_POWER_STARTED: return "AC Power Started";
    case AC_SURFACE_ON: return "AC Surface ON";
    case AC_VEHICLE_ON: return "AC Vehicle ON";
    case DC1_SELECTED: return "DC 1 Selected";
    case DC2_SELECTED: return "DC 2 Selected";
    case DC1_AND_2_SELECTED: return "DC 1 & 2 Selected";
    case MASTER_DC_ON_THRUSTERS_DISABLED: return "Master DC ON - Thrusters Disabled";
    case MASTER_DC_ON_DC1_ONLY: return "Master DC ON - DC1 Only – Thrusters Disabled";
    case MASTER_DC_ON_DC2_ONLY: return "Master DC ON - DC2 Only – Thrusters Disabled";
    case THRUSTERS_ENABLED_DC1_ONLY: return "Thrusters Enabled – DC 1 Only";
    case THRUSTERS_ENABLED_DC2_ONLY: return "Thrusters Enabled – DC 2 Only";
    case THRUSTERS_ENABLED_SYSTEM_READY: return "Thrusters Enabled – System Ready";
    default: return "Unknown State";
  }
}

SystemState getCurrentState() {
  bool encoderOn = encoderswitchState;

  // Check states in descending order of specificity
  // ----------------------------------------------------------------------------
  // 1. Thrusters Enabled – System Ready (Sw1-7 ON, Sw8 ignored)
  if (encoderOn && 
      switchStates[0] == 1 &&  // Sw1
      switchStates[1] == 1 &&  // Sw2
      switchStates[2] == 1 &&  // Sw3
      switchStates[3] == 1 &&  // Sw4 (DC1)
      switchStates[4] == 1 &&  // Sw5 (DC2)
      switchStates[5] == 1 &&  // Sw6 (Master DC)
      switchStates[6] == 1) {  // Sw7 (Thrusters)
    return THRUSTERS_ENABLED_SYSTEM_READY;
  }
  // 2. Thrusters Enabled – DC 1 Only (Sw1-4 ON, Sw6-7 ON; Sw5 OFF)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 1 &&  // Sw4 (DC1)
           switchStates[4] == 0 &&  // Sw5 (DC2 OFF)
           switchStates[5] == 1 &&  // Sw6 (Master DC)
           switchStates[6] == 1) {  // Sw7 (Thrusters)
    return THRUSTERS_ENABLED_DC1_ONLY;
  }
  // 3. Thrusters Enabled – DC 2 Only (Sw1-3 ON, Sw5-7 ON; Sw4 OFF)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 0 &&  // Sw4 (DC1 OFF)
           switchStates[4] == 1 &&  // Sw5 (DC2)
           switchStates[5] == 1 &&  // Sw6 (Master DC)
           switchStates[6] == 1) {  // Sw7 (Thrusters)
    return THRUSTERS_ENABLED_DC2_ONLY;
  }
  // 4. Master DC ON - DC1 Only (Sw1-4,6 ON; Sw5,7 OFF)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 1 &&  // Sw4 (DC1)
           switchStates[4] == 0 &&  // Sw5 (DC2 OFF)
           switchStates[5] == 1 &&  // Sw6 (Master DC)
           switchStates[6] == 0) {  // Sw7 (Thrusters OFF)
    return MASTER_DC_ON_DC1_ONLY;
  }
  // 5. Master DC ON - DC2 Only (Sw1-3,5-6 ON; Sw4,7 OFF)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 0 &&  // Sw4 (DC1 OFF)
           switchStates[4] == 1 &&  // Sw5 (DC2)
           switchStates[5] == 1 &&  // Sw6 (Master DC)
           switchStates[6] == 0) {  // Sw7 (Thrusters OFF)
    return MASTER_DC_ON_DC2_ONLY;
  }
  // 6. Master DC ON - Thrusters Disabled (Sw1-6 ON; Sw7 OFF)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 1 &&  // Sw4 (DC1)
           switchStates[4] == 1 &&  // Sw5 (DC2)
           switchStates[5] == 1 &&  // Sw6 (Master DC)
           switchStates[6] == 0) {  // Sw7 (Thrusters OFF)
    return MASTER_DC_ON_THRUSTERS_DISABLED;
  }
  // 7. DC 1 & 2 Selected (Sw1-5 ON)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 1 &&  // Sw4 (DC1)
           switchStates[4] == 1) {   // Sw5 (DC2)
    return DC1_AND_2_SELECTED;
  }
  // 8. DC 2 Selected (Sw1-3,5 ON; Sw4 OFF)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 0 &&  // Sw4 (DC1 OFF)
           switchStates[4] == 1) {  // Sw5 (DC2)
    return DC2_SELECTED;
  }
  // 9. DC 1 Selected (Sw1-4 ON)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1 &&  // Sw3
           switchStates[3] == 1) {  // Sw4 (DC1)
    return DC1_SELECTED;
  }
  // 10. AC Vehicle ON (Sw1-3 ON)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1 &&  // Sw2
           switchStates[2] == 1) {  // Sw3
    return AC_VEHICLE_ON;
  }
  // 11. AC Surface ON (Sw1-2 ON)
  else if (encoderOn && 
           switchStates[0] == 1 &&  // Sw1
           switchStates[1] == 1) {  // Sw2
    return AC_SURFACE_ON;
  }
  // 12. AC Power Started (Sw1 ON)
  else if (encoderOn && 
           switchStates[0] == 1) {  // Sw1
    return AC_POWER_STARTED;
  }
  // 13. AC Power Available (Encoder ON, no switches)
  else if (encoderOn) {
    return AC_POWER_AVAILABLE;
  }
  // 14. System Shutdown (All switches OFF)
  else if (!encoderOn && 
           switchStates[0] == 0 && 
           switchStates[1] == 0 && 
           switchStates[2] == 0 && 
           switchStates[3] == 0 && 
           switchStates[4] == 0 && 
           switchStates[5] == 0 && 
           switchStates[6] == 0) {
    return SYSTEM_SHUTDOWN;
  }
  // 15. Main Contactor Tripped (Encoder OFF + any switch ON)
  else if (!encoderOn) {
    return MAIN_CONTACTOR_TRIPPED;
  }
  // 16. Unknown state
  return UNKNOWN_STATE;
}

// ----------------------------
// XPT2046 Touch Screen SPI pins
// ----------------------------
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33
// SPIClass mySpi = SPIClass(HSPI);             // touch does not work with this setting
SPIClass mySpi = SPIClass(VSPI);                // critical to get touch working
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

    ui_init();

// Initialize Devices on I2C Bus
    Wire.begin(27, 22);
    /*Initialize Byte-Switch*/
    deviceSwitch.begin(&Wire, switchId, 27, 22, 100000);  // ByteSwitch
    deviceSwitch.setLEDShowMode(BYTE_LED_USER_DEFINED);   // ByteSwitch - Set Leds To User Defined Mode    
    for (int i = 0; i < 8; i++) {
        deviceSwitch.setLEDBrightness(i, 250);            // ByteSwitch - Set Leds Max Brightness 
        deviceSwitch.setRGB888(i, 0x0000FF);              // ByteSwitch - Test ALL LEDS BLUE
        delay(100);           
    }
    for (int i = 0; i < 8; i++) {
        deviceSwitch.setRGB888(i, 0x000000);              // ByteSwitch - Turn OFF ALL LEDS            
    }
    deviceSwitch.setFlashWriteBack();                     // SAVE SETTINGS
    /*Initialize Byte-Button*/
    deviceButton.begin(&Wire, buttonId, 27, 22, 100000);  // ByteButton
    deviceButton.setLEDShowMode(BYTE_LED_USER_DEFINED);   // ByteButton - Set Leds To User Defined Mode 
    for (int i = 0; i < 8; i++) {
        deviceButton.setLEDBrightness(i, 250);            // ByteButton - Set Leds Max Brightness
        deviceButton.setRGB888(i, 0x0000FF);              // ByteButton - Test ALL LEDS BLUE
        delay(100);                
    }
    for (int i = 0; i < 8; i++) {
        deviceButton.setRGB888(i, 0x000000);              // ByteButton - Turn OFF ALL LEDS
    }                  
    deviceButton.setFlashWriteBack();                     // SAVE SETTINGS
    /*Initialize Unit_8Encoder*/
    sensor.begin(&Wire, ENCODER_ADDR, 27, 22, 100000UL);  // UL denotes unsigned long
    for (int i = 0; i < 8; i++) {
        sensor.setLEDColor(i, 0x0000FF);                  // Encoder - Test ALL LEDS BLUE
        delay(100);                
    }
    sensor.setAllLEDColor(0x000000);                      // Encoder - Turn OFF ALL LEDS
    Serial.println("Setup done");
}


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
      // Invert logic: 0 becomes 1 (Down=ON), 1 becomes 0 (Up=OFF)
    switchStates[i] = 1 - deviceSwitch.getSwitchStatus(i); // <-- Added inversion here
    }
}

///////////////////////////////////////////// READ 8Ch Byte Button ////////////////////////
void ReadButtons() {
    for (uint8_t i = 0; i < 8; i++) {
        // Invert logic during read (0=Pressed becomes 1=Pressed)
        buttonStates[i] = 1 - deviceButton.getSwitchStatus(i);
    }
}

///////////////////////////////////////////// MAIN LOOP ///////////////////////////////////
void loop() {
  lv_timer_handler();
  ReadEncoder();
  ReadSwitches();
  ReadButtons();

  static SystemState previousState = UNKNOWN_STATE;
  SystemState currentState = getCurrentState();

  // Log state changes
  if (currentState != previousState) {
    Serial.print("State changed to: ");
    Serial.println(getStateName(currentState));
    previousState = currentState;
  }

  // Execute tasks based on the current state
  switch (currentState) {
    case SYSTEM_SHUTDOWN:

      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      //sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      //deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      //deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      //deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      //lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      //lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor          
     
      break;
    case AC_POWER_AVAILABLE:
      
      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      //sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      //deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      //deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      //deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      //lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case AC_POWER_STARTED:

      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      //sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      //deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      //deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      //lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case AC_SURFACE_ON:
      
      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      //deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor
      
      break;
    case AC_VEHICLE_ON:
      
      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case DC1_SELECTED:
      
      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case DC2_SELECTED:

// ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
        lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case DC1_AND_2_SELECTED:
      
// ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case MASTER_DC_ON_THRUSTERS_DISABLED: // DC1 & DC2 ARE ON (Naming of the case should be clearer)

// ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case MASTER_DC_ON_DC1_ONLY:

      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case MASTER_DC_ON_DC2_ONLY:

      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case THRUSTERS_ENABLED_DC1_ONLY:

      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      //deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case THRUSTERS_ENABLED_DC2_ONLY:

      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      //deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case THRUSTERS_ENABLED_SYSTEM_READY:
      
// ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      //deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      //deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      //deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      //deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      //deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      //deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      //deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      //lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      //lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      //lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case MAIN_CONTACTOR_TRIPPED:  // <-- NEW CASE Modification #1
      
      // ENCODER INDICATOR LED SETTINGS
      // ENCODER INDICATOR LED - OFF STATE
      sensor.setLEDColor(8, 0xFF0000);                                                                              // ENCODER INDICATOR LED - Set Led 1 RED
      // ENCODER INDICATOR LED - ON STATE
      //sensor.setLEDColor(8, 0x00FF00);                                                                              // ENCODER INDICATOR LED - Set Led 1 GREEN
      // BYTE SWITCH SETTINGS
      // BYTE SWITCH LEDS - OFF STATES
      deviceSwitch.setRGB888(0, 0x000000);                                                                          // ByteSwitch - Set Led 1 OFF
      deviceSwitch.setRGB888(1, 0x000000);                                                                          // ByteSwitch - Set Led 2 OFF
      deviceSwitch.setRGB888(2, 0x000000);                                                                          // ByteSwitch - Set Led 3 OFF
      deviceSwitch.setRGB888(3, 0x000000);                                                                          // ByteSwitch - Set Led 4 OFF
      deviceSwitch.setRGB888(4, 0x000000);                                                                          // ByteSwitch - Set Led 5 OFF
      deviceSwitch.setRGB888(5, 0x000000);                                                                          // ByteSwitch - Set Led 6 OFF
      deviceSwitch.setRGB888(6, 0x000000);                                                                          // ByteSwitch - Set Led 7 OFF
      deviceSwitch.setRGB888(7, 0x000000);                                                                          // ByteSwitch - Set Led 8 OFF
      // BYTE SWITCH LEDS - RED STATES
      //deviceSwitch.setRGB888(0, 0xFF0000);                                                                          // ByteSwitch - Set Led 1 RED
      //deviceSwitch.setRGB888(1, 0xFF0000);                                                                          // ByteSwitch - Set Led 2 RED
      //deviceSwitch.setRGB888(2, 0xFF0000);                                                                          // ByteSwitch - Set Led 3 RED
      //deviceSwitch.setRGB888(3, 0xFF0000);                                                                          // ByteSwitch - Set Led 4 RED
      //deviceSwitch.setRGB888(4, 0xFF0000);                                                                          // ByteSwitch - Set Led 5 RED
      //deviceSwitch.setRGB888(5, 0xFF0000);                                                                          // ByteSwitch - Set Led 6 RED
      //deviceSwitch.setRGB888(6, 0xFF0000);                                                                          // ByteSwitch - Set Led 7 RED
      //deviceSwitch.setRGB888(7, 0xFF0000);                                                                          // ByteSwitch - Set Led 8 RED
      // BYTE SWITCH LEDS - GREEN STATES
      //deviceSwitch.setRGB888(0, 0x00FF00);                                                                          // ByteSwitch - Set Led 1 GREEN
      //deviceSwitch.setRGB888(1, 0x00FF00);                                                                          // ByteSwitch - Set Led 2 GREEN
      //deviceSwitch.setRGB888(2, 0x00FF00);                                                                          // ByteSwitch - Set Led 3 GREEN
      //deviceSwitch.setRGB888(3, 0x00FF00);                                                                          // ByteSwitch - Set Led 4 GREEN
      //deviceSwitch.setRGB888(4, 0x00FF00);                                                                          // ByteSwitch - Set Led 5 GREEN
      //deviceSwitch.setRGB888(5, 0x00FF00);                                                                          // ByteSwitch - Set Led 6 GREEN
      //deviceSwitch.setRGB888(6, 0x00FF00);                                                                          // ByteSwitch - Set Led 7 GREEN
      //deviceSwitch.setRGB888(7, 0x00FF00);                                                                          // ByteSwitch - Set Led 8 GREEN
      // USER INTERFACE SETTINGS 
      // UI LEDS - OFF STATES
      lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to Grey
      lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x5e5d62), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to Grey
      // UI LEDS - ON STATES
      //lv_obj_set_style_bg_color(ui_PSUPOWERLED, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Power Led indicator on UI to GREEN
      //lv_obj_set_style_bg_color(ui_ACSurfaceLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Surface Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ACVehicleLED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);          // Set AC Vehicle Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC1LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 1 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_DC2LED, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);                // Set DC 2 Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_MasterDCLed, lv_color_hex(0xFF0000), LV_PART_MAIN | LV_STATE_DEFAULT);           // Set Master DC Power Led indicator on UI to RED
      //lv_obj_set_style_bg_color(ui_ThrustersEnabledLed, lv_color_hex(0x00FF00), LV_PART_MAIN | LV_STATE_DEFAULT);   // Set Master DC Power Led indicator on UI to GREEN
      // SWITCHES - OFF STATES
      lv_obj_clear_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                     // Set AC Surface Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                     // Set AC Vehicle Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC1Switch, LV_STATE_CHECKED);                                                           // SetDC 1 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_DC2Switch, LV_STATE_CHECKED);                                                           // Set DC 2 Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                      // Set Master DC Switch to UNCHECKED state (OFF)
      lv_obj_clear_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                              // Set Thrusters Enabled Switch to UNCHECKED state (OFF)
      // SWITCHES - ON STATES
      //lv_obj_add_state(ui_ACSurfaceSwitch, LV_STATE_CHECKED);                                                       // Set AC Surface Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ACVehicleSwitch, LV_STATE_CHECKED);                                                       // Set AC Vehicle Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC1Switch, LV_STATE_CHECKED);                                                             // SetDC 1 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_DC2Switch, LV_STATE_CHECKED);                                                             // Set DC 2 Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_MasterDCSwitch, LV_STATE_CHECKED);                                                        // Set Master DC Switch to CHECKED state (ON)
      //lv_obj_add_state(ui_ThrustersEnabledSwitch, LV_STATE_CHECKED);                                                // Set Thrusters Enabled Switch to CHECKED state (ON)
      // MAIN CONTACTOR HIDE / SHOW FLAGS                                                                                   
      lv_obj_add_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                          // HIDE the white ON indicator on The Main Contctor
      //lv_obj_add_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                         // HIDE the white OFF indicator  on The Main Contctor
      // MAIN CONTACTOR ADD FLAGS                                                                                    
      //lv_obj_clear_flag(ui_MainContactorIndicatorBarON, LV_OBJ_FLAG_HIDDEN);                                        // SHOW the white ON indicator on The Main Contctor
      lv_obj_clear_flag(ui_MainContactorIndicatorBarOFF, LV_OBJ_FLAG_HIDDEN);                                       // SHOW the white OFF indicator  on The Main Contctor

      break;
    case UNKNOWN_STATE:
      // Add tasks 
      break;
      // Add cases for other states as needed
    default:
      break;
  }

  delay(5);
}
