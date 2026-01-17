#include <Wire.h>
#include <Arduino.h>
#include "pin_config.h"
#include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "HWCDC.h"
HWCDC USBSerial;

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
                                      0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);

void Arduino_IIC_Touch_Interrupt(void);

std::unique_ptr<Arduino_IIC> CST816T(new Arduino_CST816x(IIC_Bus, CST816T_DEVICE_ADDRESS,
                                                         TP_RST, TP_INT, Arduino_IIC_Touch_Interrupt));

void Arduino_IIC_Touch_Interrupt(void) {
  CST816T->IIC_Interrupt_Flag = true;
}

// Define GPIO pins for power control and button
#define SYS_EN_GPIO 41
#define SYS_OUT_GPIO 40

// Variables for button press detection
uint8_t key_long_press_cnt = 0;

// Variables to store the last touch coordinates
static int32_t lastTouchX = -1, lastTouchY = -1;

// Variables for power state management
bool powerState = true; // true = ON, false = OFF
bool keyReleased = true; // Track if the button has been released

void setup() {
  USBSerial.begin(115200);
  Wire.begin(IIC_SDA, IIC_SCL);

  while (CST816T->begin() == false) {
    USBSerial.println("CST816T initialization fail");
    delay(2000);
  }
  USBSerial.println("CST816T initialization successfully");

  CST816T->IIC_Write_Device_State(CST816T->Arduino_IIC_Touch::Device::TOUCH_DEVICE_INTERRUPT_MODE,
                                  CST816T->Arduino_IIC_Touch::Device_Mode::TOUCH_DEVICE_INTERRUPT_PERIODIC);

  gfx->begin();
  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, HIGH);
  gfx->fillScreen(WHITE);

  for (int i = 0; i <= 255; i++)  //0-255
  {
    gfx->Display_Brightness(i);
    gfx->setCursor(20, 100);
    gfx->setTextColor(BLUE);
    gfx->setTextSize(2);
    gfx->println("Loading board");
    delay(3);
  }
  delay(500);
  gfx->fillScreen(WHITE);

  // Initialize power control GPIO
  pinMode(SYS_EN_GPIO, OUTPUT);
  digitalWrite(SYS_EN_GPIO, HIGH); // Enable power by default

  // Initialize button GPIO
  pinMode(SYS_OUT_GPIO, INPUT_PULLUP);
}

void loop() {
  int32_t touchX = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_X);
  int32_t touchY = CST816T->IIC_Read_Device_Value(CST816T->Arduino_IIC_Touch::Value_Information::TOUCH_COORDINATE_Y);

  USBSerial.printf("Touch X:%d Y:%d\n", touchX, touchY);

  if (touchX > 20 && touchY > 20) {
    if (lastTouchX != -1 && lastTouchY != -1) {
      // Draw a thicker line between the last touch point and the current touch point
      gfx->drawLine(lastTouchX, lastTouchY, touchX, touchY, BLUE);
      gfx->fillCircle(lastTouchX, lastTouchY, 5, BLUE); // Add thickness to the start of the line
      gfx->fillCircle(touchX, touchY, 5, BLUE); // Add thickness to the end of the line
    }
    // Update the last touch coordinates
    lastTouchX = touchX;
    lastTouchY = touchY;
  } else {
    // Reset the last touch coordinates if no valid touch is detected
    lastTouchX = -1;
    lastTouchY = -1;
  }

  // Power button logic
  if (digitalRead(SYS_OUT_GPIO) == LOW) { // Button is pressed
    if (keyReleased) {
      key_long_press_cnt++;
    }
  } else {
    key_long_press_cnt = 0;
    keyReleased = true; // Reset when button is released
  }

  if (key_long_press_cnt > 50 && keyReleased) { // Long press detected (5 seconds)
    powerState = !powerState; // Toggle power state
    digitalWrite(SYS_EN_GPIO, powerState ? HIGH : LOW); // Update power state
    digitalWrite(LCD_BL, powerState ? HIGH : LOW); // Update backlight state
    keyReleased = false; // Wait for button release
  }

  delay(10); // Reduce delay to improve touch responsiveness
}
