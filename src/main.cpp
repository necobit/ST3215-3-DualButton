#include <Arduino.h>
#include <SCServo.h>
#include "M5_EXTIO2.h"

// Serial pins for STS3215 communication
#define STS_TX_PIN 9
#define STS_RX_PIN 7
#define STS_BAUDRATE 1000000

// Button pins (active LOW with internal pull-up)
#define BTN_SERVO1_CW 2  // Servo 1 forward
#define BTN_SERVO1_CCW 1 // Servo 1 reverse
#define BTN_SERVO2_CW 3  // Servo 2 forward
#define BTN_SERVO2_CCW 4 // Servo 2 reverse
#define BTN_SERVO3_CW 5  // Servo 3 forward
#define BTN_SERVO3_CCW 6 // Servo 3 reverse

// EXTIO2 I2C pins
#define EXTIO2_SDA 13
#define EXTIO2_SCL 15

// Servo IDs
#define SERVO1_ID 1
#define SERVO2_ID 2
#define SERVO3_ID 3

// Speed settings (0-3000, positive = CW, negative = CCW)
#define SERVO_SPEED 1500
#define SERVO_ACC 30 // Max 255

SMS_STS sts;
M5_EXTIO2 extio;

// Button state tracking
struct ButtonState
{
  int cwPin;
  int ccwPin;
  int servoId;
  uint8_t fwdLimitPin; // EXTIO2 pin for forward limit
  uint8_t revLimitPin; // EXTIO2 pin for reverse limit
  bool lastCwState;
  bool lastCcwState;
  bool lastFwdLimit; // previous forward limit state
  bool lastRevLimit; // previous reverse limit state
};

ButtonState buttons[3] = {
    {BTN_SERVO1_CW, BTN_SERVO1_CCW, SERVO1_ID, 0, 1, true, true, false, false},
    {BTN_SERVO2_CW, BTN_SERVO2_CCW, SERVO2_ID, 2, 3, true, true, false, false},
    {BTN_SERVO3_CW, BTN_SERVO3_CCW, SERVO3_ID, 4, 5, true, true, false, false}};

void setup()
{
  Serial.begin(115200);
  Serial.println("STS3215 Dual Button Controller");

  // Initialize serial for STS3215
  Serial1.begin(STS_BAUDRATE, SERIAL_8N1, STS_RX_PIN, STS_TX_PIN);
  sts.pSerial = &Serial1;
  delay(100);

  // Initialize EXTIO2
  while (!extio.begin(&Wire, EXTIO2_SDA, EXTIO2_SCL, 0x45))
  {
    Serial.println("EXTIO2 not found, retrying...");
    delay(500);
  }
  Serial.println("EXTIO2 initialized (FW: " + String(extio.getVersion()) + ")");
  extio.setAllPinMode(DIGITAL_INPUT_MODE);

  // Initialize button pins
  pinMode(BTN_SERVO1_CW, INPUT_PULLUP);
  pinMode(BTN_SERVO1_CCW, INPUT_PULLUP);
  pinMode(BTN_SERVO2_CW, INPUT_PULLUP);
  pinMode(BTN_SERVO2_CCW, INPUT_PULLUP);
  pinMode(BTN_SERVO3_CW, INPUT_PULLUP);
  pinMode(BTN_SERVO3_CCW, INPUT_PULLUP);

  // Ping each servo to check communication
  Serial.println("Checking servo communication...");
  for (int i = 0; i < 3; i++)
  {
    int id = buttons[i].servoId;
    int result = sts.Ping(id);
    Serial.print("Servo ");
    Serial.print(id);
    if (result != -1)
    {
      Serial.print(": OK (ID=");
      Serial.print(result);
      Serial.println(")");
    }
    else
    {
      Serial.println(": NOT FOUND");
    }
    delay(50);
  }

  // Set all servos to wheel mode (continuous rotation)
  Serial.println("Setting wheel mode...");
  for (int i = 0; i < 3; i++)
  {
    int id = buttons[i].servoId;
    sts.WheelMode(id);
    delay(50);
  }

  Serial.println("Initialization complete");
}

void loop()
{
  for (int i = 0; i < 3; i++)
  {
    bool cwPressed = (digitalRead(buttons[i].cwPin) == LOW);
    bool ccwPressed = (digitalRead(buttons[i].ccwPin) == LOW);

    // Read limit switches from EXTIO2 (LOW = limit reached)
    bool fwdLimit = (extio.getDigitalInput(buttons[i].fwdLimitPin) == 0);
    bool revLimit = (extio.getDigitalInput(buttons[i].revLimitPin) == 0);

    // Determine desired speed
    int16_t speed = 0;
    if (cwPressed && !ccwPressed)
    {
      speed = SERVO_SPEED;
    }
    else if (ccwPressed && !cwPressed)
    {
      speed = -SERVO_SPEED;
    }

    // Apply limit switch constraints
    // Forward limit blocks CW (positive speed), reverse limit blocks CCW (negative speed)
    if (fwdLimit && speed > 0)
    {
      speed = 0;
    }
    if (revLimit && speed < 0)
    {
      speed = 0;
    }

    // Update servo if button or limit state changed
    bool stateChanged = (cwPressed != !buttons[i].lastCwState) ||
                        (ccwPressed != !buttons[i].lastCcwState) ||
                        (fwdLimit != buttons[i].lastFwdLimit) ||
                        (revLimit != buttons[i].lastRevLimit);

    if (stateChanged)
    {
      int id = buttons[i].servoId;
      sts.WriteSpe(id, speed, SERVO_ACC);
      buttons[i].lastCwState = !cwPressed;
      buttons[i].lastCcwState = !ccwPressed;
      buttons[i].lastFwdLimit = fwdLimit;
      buttons[i].lastRevLimit = revLimit;

      // Serial output: servo action + all IO states
      Serial.print("Servo ");
      Serial.print(id);
      if (speed > 0)
        Serial.print(": CW");
      else if (speed < 0)
        Serial.print(": CCW");
      else
        Serial.print(": STOP");

      Serial.print(" | CW_BTN:");
      Serial.print(cwPressed ? "L" : "H");
      Serial.print(" CCW_BTN:");
      Serial.print(ccwPressed ? "L" : "H");
      Serial.print(" FWD_LIM:");
      Serial.print(fwdLimit ? "L" : "H");
      Serial.print(" REV_LIM:");
      Serial.println(revLimit ? "L" : "H");
    }
  }

  delay(10);
}
