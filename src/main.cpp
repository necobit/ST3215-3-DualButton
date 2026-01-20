#include <Arduino.h>
#include <SCServo.h>

// Serial pins for STS3215 communication
#define STS_TX_PIN 15
#define STS_RX_PIN 13
#define STS_BAUDRATE 1000000

// Button pins (active LOW with internal pull-up)
#define BTN_SERVO1_CW 2  // Servo 1 forward
#define BTN_SERVO1_CCW 1 // Servo 1 reverse
#define BTN_SERVO2_CW 3  // Servo 2 forward
#define BTN_SERVO2_CCW 4 // Servo 2 reverse
#define BTN_SERVO3_CW 5  // Servo 3 forward
#define BTN_SERVO3_CCW 6 // Servo 3 reverse

// Servo IDs
#define SERVO1_ID 1
#define SERVO2_ID 2
#define SERVO3_ID 3

// Speed settings (0-3000, positive = CW, negative = CCW)
#define SERVO_SPEED 1500
#define SERVO_ACC 254 // Max 255

SMS_STS sts;

// Load monitoring
unsigned long lastLoadPrint = 0;
#define LOAD_PRINT_INTERVAL 200 // ms

// Overload protection (CCW direction only)
#define LOAD_THRESHOLD 510
#define REVERSE_DURATION 4000 // ms
unsigned long reverseStartTime[3] = {0, 0, 0};
bool inReverse[3] = {false, false, false};
bool waitForRelease[3] = {false, false, false}; // Wait for button release after reverse

// Button state tracking
struct ButtonState
{
  int cwPin;
  int ccwPin;
  int servoId;
  bool lastCwState;
  bool lastCcwState;
};

ButtonState buttons[3] = {
    {BTN_SERVO1_CW, BTN_SERVO1_CCW, SERVO1_ID, true, true},
    {BTN_SERVO2_CW, BTN_SERVO2_CCW, SERVO2_ID, true, true},
    {BTN_SERVO3_CW, BTN_SERVO3_CCW, SERVO3_ID, true, true}};

void setup()
{
  Serial.begin(115200);
  Serial.println("STS3215 Dual Button Controller");

  // Initialize serial for STS3215
  Serial1.begin(STS_BAUDRATE, SERIAL_8N1, STS_RX_PIN, STS_TX_PIN);
  sts.pSerial = &Serial1;
  delay(100);

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

    // Verify by reading current speed
    int pos = sts.ReadPos(id);
    Serial.print("Servo ");
    Serial.print(id);
    Serial.print(": Pos=");
    Serial.println(pos);
  }

  Serial.println("Initialization complete");
}

void loop()
{
  for (int i = 0; i < 3; i++)
  {
    bool cwPressed = (digitalRead(buttons[i].cwPin) == LOW);
    bool ccwPressed = (digitalRead(buttons[i].ccwPin) == LOW);

    // Skip button handling during reverse operation
    if (inReverse[i])
    {
      continue;
    }

    // Wait for button release after reverse completes
    if (waitForRelease[i])
    {
      if (!cwPressed && !ccwPressed)
      {
        waitForRelease[i] = false;
        buttons[i].lastCwState = true; // Reset to "not pressed"
        buttons[i].lastCcwState = true;
        Serial.print("S");
        Serial.print(buttons[i].servoId);
        Serial.println(" Button released, ready");
      }
      continue;
    }

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

    // Update servo if state changed
    bool stateChanged = (cwPressed != !buttons[i].lastCwState) ||
                        (ccwPressed != !buttons[i].lastCcwState);

    if (stateChanged)
    {
      int id = buttons[i].servoId;
      sts.WriteSpe(id, speed, SERVO_ACC);
      buttons[i].lastCwState = !cwPressed;
      buttons[i].lastCcwState = !ccwPressed;

      // Serial output
      Serial.print("Servo ");
      Serial.print(id);
      Serial.print(" WriteSpe(");
      Serial.print(speed);
      Serial.print(", ");
      Serial.print(SERVO_ACC);
      Serial.print(") -> ");

      // Read back to verify
      delay(10);
      int readSpeed = sts.ReadSpeed(id);
      int readLoad = sts.ReadLoad(id);
      Serial.print("Speed=");
      Serial.print(readSpeed);
      Serial.print(", Load=");
      Serial.println(readLoad);
    }
  }

  // Periodic load monitoring and overload protection
  if (millis() - lastLoadPrint >= LOAD_PRINT_INTERVAL)
  {
    lastLoadPrint = millis();
    for (int i = 0; i < 3; i++) // All servos
    {
      int id = buttons[i].servoId;
      int load = sts.ReadLoad(id);
      int spd = sts.ReadSpeed(id);
      bool cw = (digitalRead(buttons[i].cwPin) == LOW);
      bool ccw = (digitalRead(buttons[i].ccwPin) == LOW);

      // Check for overload in CCW direction (negative speed)
      if (!inReverse[i] && spd < 0 && load > LOAD_THRESHOLD)
      {
        // Overload detected - stop and reverse
        inReverse[i] = true;
        reverseStartTime[i] = millis();
        sts.WriteSpe(id, SERVO_SPEED, SERVO_ACC); // Reverse to CW
        Serial.print("*** S");
        Serial.print(id);
        Serial.println(" OVERLOAD! Reversing ***");
      }

      // Check if reverse duration has elapsed
      if (inReverse[i] && (millis() - reverseStartTime[i] >= REVERSE_DURATION))
      {
        inReverse[i] = false;
        waitForRelease[i] = true;       // Require button release before next operation
        sts.WriteSpe(id, 0, SERVO_ACC); // Stop
        Serial.print("*** S");
        Serial.print(id);
        Serial.println(" Reverse complete, release button ***");
      }

      Serial.print("S");
      Serial.print(id);
      Serial.print("[");
      if (inReverse[i])
        Serial.print("REV");
      else if (waitForRelease[i])
        Serial.print("WAIT");
      else if (cw)
        Serial.print("CW");
      else if (ccw)
        Serial.print("CCW");
      else
        Serial.print("--");
      Serial.print("] Spd=");
      Serial.print(spd);
      Serial.print(" Load=");
      Serial.print(load);
      Serial.print("  ");
    }
    Serial.println();
  }

  delay(10);
}
