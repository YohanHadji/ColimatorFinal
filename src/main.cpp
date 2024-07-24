#include <Arduino.h>
#include <PWMServo.h>
#include <capsule.h>

// Define the servo pins
const int servoPin1 = 3;
const int servoPin2 = 4;

float modelS1(float CamY);
float modelS2(float CamX);

#define SERVO_POWER_PIN 5
#define SERVO_POWER_TIMER 5000

static unsigned long lastTimeLightInView = millis();

// void map_quadrilateral_to_rectangle(float src_x, float src_y, float& dest_x, float& dest_y);
void handleRpi(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic rpi(handleRpi);

struct CameraErrorPacket {
  char name[4];
  int32_t isVisible;
  int32_t Cx;
  int32_t Cy;
  int32_t age;
}; 

#define START_X 1450
#define END_X 1550

#define START_Y 1650
#define END_Y 1750

#define STEP_N 

#define CENTER_X 1342
#define CENTER_Y 1495

CameraErrorPacket lastCameraError;

// Create servo objects
PWMServo servo1;
PWMServo servo2;

int servo1GlobalPos = 0;
int servo2GlobalPos = 0;

void setup() {
  // Attach servos to their respective pins
  servo1.attach(servoPin1, 1000, 2000);
  servo2.attach(servoPin2, 1000, 2000);
  Serial.begin(115200);
  Serial4.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_POWER_PIN, OUTPUT);

  // delay(10000);

  // // Do a full scan with the servos with 100 steps on each axis from 900 to 2100us to calibrate the camera
  // for (int i = 0; i < STEP_N; i++) {
  //   servo1GlobalPos = map(i, 0, STEP_N, START_X, END_X);
  //   servo1.write(map(servo1GlobalPos, 1000, 2000, 0, 180));
  //   delay(1000);
  //   servo2.write(map(START_Y, 1000, 2000, 0, 180));
  //   delay(1000);
  //   for (int j = 0; j < STEP_N; j++) {
  //     servo2GlobalPos = map(j, 0, STEP_N, START_Y, END_Y);
  //     // servo1.write(map(servo1GlobalPos, 1000, 2000, 0, 180));
  //     servo2.write(map(servo2GlobalPos, 1000, 2000, 0, 180));

  //     // Serial4.print(lastCameraError.Cx);
  //     // Serial4.print(",");
  //     // Serial4.print(lastCameraError.Cy);
  //     // Serial4.print(",");

  //     delay(200);

  //     Serial.print(servo1GlobalPos);
  //     Serial.print(",");
  //     Serial.println(servo2GlobalPos);

  //     // for (int k = 0; k < 50; k++) {
  //     //   delay(1);
  //     //   // while (Serial.available() > 0) {
  //     //     // rpi.decode(Serial.read());
  //     //   // }
  //     // }
  //   }
  // }
}

void loop() {

  while (Serial.available() > 0) {
    rpi.decode(Serial.read());
  }

  // servo1.write(map(1500, 1000, 2000, 0, 180));
  // servo1.write(map(1500, 1000, 2000, 0, 180));

  // while(Serial4.available() > 0) {
  //   Serial.write(Serial4.read());
  // }

  // if (millis() - lastTimeLightInView > SERVO_POWER_TIMER) {
  //   digitalWrite(SERVO_POWER_PIN, LOW);
  // }
  // else {
  //   digitalWrite(SERVO_POWER_PIN, HIGH);
  // }
  // static unsigned long lastChange = millis();
  // if (millis() - lastChange > 10000) {
  //   lastChange = millis();
  //   static int servoState = 0;
  //   servoState++;
  //   servoState = servoState%4;

  //   switch (servoState) {
  //     case 0:
  //       servo1.writeMicroseconds(900);
  //       servo2.writeMicroseconds(900);
  //       break;
  //     case 1:
  //       servo1.writeMicroseconds(2100);
  //       servo2.writeMicroseconds(900);
  //       break;
  //     case 2:
  //       servo1.writeMicroseconds(2100);
  //       servo2.writeMicroseconds(2100);
  //       break;
  //     case 3:
  //       servo1.writeMicroseconds(900);
  //       servo2.writeMicroseconds(2100);
  //       break;
  //   }
  // }
  // servo1.writeMicroseconds(1500);
  // servo2.writeMicroseconds(1500);
  // delay(1000);
}

void handleRpi(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  // Check if the packet ID is 1
  if (packetId == 0x01) {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    // Serial.println("Received Camera Error Packet");

    memcpy(&lastCameraError, dataIn, sizeof(CameraErrorPacket));

    if (lastCameraError.isVisible) {
      lastTimeLightInView = millis();
  
      int servoPos1 = modelS1(lastCameraError.Cy);
      int servoPos2 = modelS2(lastCameraError.Cx);

      servoPos1 = constrain(servoPos1, 1000, 2000);
      servoPos2 = constrain(servoPos2, 1000, 2000);

      // map_quadrilateral_to_rectangle(lastCameraError.Cx, lastCameraError.Cy, servoPosX, servoPosY);

      servo1.write(map(servoPos1, 1000, 2000, 0, 180));
      servo2.write(map(servoPos2, 1000, 2000, 0, 180));
    }
  }
}

// Define coefficients for the plane equations
float s1p00 = 1783;
float s1p10 = -0.1597;
float s1p01 = 4.126;

float s2p00 = 1757;
float s2p10 = 4.977;
float s2p01 = 0.2432;

float k1 = ((1498.0-1342.0)/-300.0);
float k2 = ((1668.0-1495.0)/-200.0);

// Define functions to calculate S1 and S2 based on CamX and CamY
float modelS1(float CamY) {
    return CENTER_X + k1*CamY;
}

float modelS2(float CamX) {
    return CENTER_Y + k2*CamX;
}
