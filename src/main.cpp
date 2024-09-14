#include <Arduino.h>
#include <PWMServo.h>
#include <capsule.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>
#include <AccelStepper.h>
#include <capsule.h>

#define CAPSULE_ID_POSITION 0x15
#define CAPSULE_ID_GUSTAVO_CUSTOM 0x21

#define X_STEP_PIN 12
#define X_DIR_PIN 11
#define HOME_SWITCH_X 7

AccelStepper stepperX(1, X_STEP_PIN, X_DIR_PIN);
void handlePacket(uint8_t, uint8_t*, uint32_t);

// const int stepIncrement = 3;  // Define el tamaño del paso para movimientos incrementales
const long int limite_safe = 44800;  // Límite seguro para el movimiento del motor

// MAC address for Teensy 1
byte mac[] = {0xDE, 0xAD, 0xBF, 0xEB, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 103); // IP address for Teensy 1
// unsigned int localPort = 8888; // Local port to listen on

EthernetUDP udp;
EthernetServer server(80);

// Define the servo pins
const int servoPin1 = 3;
const int servoPin2 = 4;

#define SERVO_POWER_PIN 5
#define SERVO_POWER_TIMER 5000

static unsigned long lastTimeLightInView = millis();

// void map_quadrilateral_to_rectangle(float src_x, float src_y, float& dest_x, float& dest_y);
void handleRpi(uint8_t packetId, uint8_t *dataIn, uint32_t len);
void handleJoystick(uint8_t packetId, uint8_t *dataIn, uint32_t len);
CapsuleStatic rpi(handleRpi);
CapsuleStatic joystick(handleJoystick);

struct CameraErrorPacket {
  char name[4];
  int32_t isVisible;
  int32_t Cx;
  int32_t Cy;
  int32_t age;
  int32_t camID;
  float Kp;
  float maxSpeed;
}; 

void homing(); 

struct dataStruct {
  int position;
};
 
dataStruct lastCmd;
CameraErrorPacket lastCameraError;

// Create servo objects
PWMServo servo1;
PWMServo servo2;

float servo1GlobalPos = 1500;
float servo2GlobalPos = 1500;

#define START_X 1125
#define END_X 1650
#define START_Y 1100
#define END_Y 2000

#define STEP_N 40

void setup() {
  // Attach servos to their respective pins
  servo1.attach(servoPin1, 1000, 2000);
  servo2.attach(servoPin2, 1000, 2000);
  Serial.begin(115200);
  Serial4.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SERVO_POWER_PIN, OUTPUT);

  Ethernet.begin(mac, ip);
  udp.begin(8888);
  server.begin();

  // Focus stepper  
  pinMode(HOME_SWITCH_X, INPUT_PULLUP);
  pinMode(LED_BUILTIN, OUTPUT);

  stepperX.setMaxSpeed(10000);
  stepperX.setAcceleration(1000);

  // delay(5000);

  // // Do a full scan with the servos with 100 steps on each axis from 900 to 2100us to calibrate the camera
  // for (int i = 0; i < STEP_N/2; i++) {

  //   servo1GlobalPos = map(i*2, 0, STEP_N, START_X, END_X);
  //   servo1.write(map(servo1GlobalPos, 1000, 2000, 0, 180));
  //   delay(500);
  //   // servo2.write(map(START_Y, 1000, 2000, 0, 180));
  //   // delay(1000);

  //   for (int j = 0; j < STEP_N; j++) {
  //     servo2GlobalPos = map(j, 0, STEP_N, START_Y, END_Y);
  //     servo2.write(map(servo2GlobalPos, 1000, 2000, 0, 180));

  //     delay(500);

  //     Serial.print(servo1GlobalPos);
  //     Serial.print(",");
  //     Serial.println(servo2GlobalPos);
  //   }

  //   servo1GlobalPos = map((i*2)+1, 0, STEP_N, START_X, END_X);
  //   servo1.write(map(servo1GlobalPos, 1000, 2000, 0, 180));
  //   delay(500);
  //   // servo2.write(map(END_Y, 1000, 2000, 0, 180));
  //   // delay(1000);

  //   for (int j = STEP_N; j > 0; j--) {
  //     servo2GlobalPos = map(j, 0, STEP_N, START_Y, END_Y);
  //     servo2.write(map(servo2GlobalPos, 1000, 2000, 0, 180));

  //     delay(500);

  //     Serial.print(servo1GlobalPos);
  //     Serial.print(",");
  //     Serial.println(servo2GlobalPos);
  //   }
  // }

  homing();
}

void loop() {

  int packetSize = udp.parsePacket();
  if (packetSize) {
    for (int i = 0; i < packetSize; i++) {
      joystick.decode(udp.read());
    }
  }

  while (Serial.available() > 0) {
    rpi.decode(Serial.read());
  }

  static unsigned long lastDataSent = millis();

  if (millis()-lastDataSent>100) {
      lastDataSent = millis();
      Serial.print(servo1GlobalPos);
      Serial.print(",");
      Serial.println(servo2GlobalPos);
  }
}

void handleRpi(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  // Check if the packet ID is 1
  switch (packetId ) {
    case 0x01:
      static bool ledState = false;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);
      // Serial.println("Received Camera Error Packet");

      memcpy(&lastCameraError, dataIn, sizeof(CameraErrorPacket));

      if (lastCameraError.isVisible) {
        lastTimeLightInView = millis();
    
        // servo1GlobalPos = map(lastCameraError.Cy, -1000, 1000, 1000, 2000);
        // servo2GlobalPos = map(lastCameraError.Cx, -1000, 1000, 1000, 2000);

        servo1GlobalPos = lastCameraError.Cx;
        servo2GlobalPos = lastCameraError.Cy;

        // map_quadrilateral_to_rectangle(lastCameraError.Cx, lastCameraError.Cy, servoPosX, servoPosY);

        servo1.write(map(servo1GlobalPos, 1000, 2000, 0, 180));
        servo2.write(map(servo2GlobalPos, 1000, 2000, 0, 180));
      }
    break;

    case CAPSULE_ID_POSITION:
      memcpy(&lastCmd, dataIn, sizeof(dataStruct));
      Serial.println("Received packet from Raspberry");
      if (lastCmd.position >= 0 && lastCmd.position <= limite_safe) {
        stepperX.moveTo(lastCmd.position);
        while (stepperX.distanceToGo() != 0) {
          stepperX.run();
        }
        // Serial.println("Movimiento completado a posición: " + String(lastCmd.position));
      } else {
        // Serial.println("Error: Posición fuera de límites");
      }
    break;
  }
}

void handleJoystick(uint8_t packetId, uint8_t *dataIn, uint32_t len) {
  // Check if the packet ID is 1
  if (packetId == 0x01) {
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
    // Serial.println("Received Camera Error Packet");

    memcpy(&lastCameraError, dataIn, sizeof(CameraErrorPacket));

    if (lastCameraError.isVisible) {
      lastTimeLightInView = millis();
  
      servo1GlobalPos+= map(lastCameraError.Cy, -1000, 1000, -5.0, 5.0);
      servo2GlobalPos+= map(lastCameraError.Cx, -1000, 1000, -5.0, 5.0);

      servo1GlobalPos = constrain(servo1GlobalPos, 1000, 2000);
      servo2GlobalPos = constrain(servo2GlobalPos, 1000, 2000);

      // map_quadrilateral_to_rectangle(lastCameraError.Cx, lastCameraError.Cy, servoPosX, servoPosY);

      servo1.write(map(servo1GlobalPos, 1000, 2000, 0, 180));
      servo2.write(map(servo2GlobalPos, 1000, 2000, 0, 180));
    }
  }
}

// // Define coefficients for the plane equations
// float s1p00 = 1783;
// float s1p10 = -0.1597;
// float s1p01 = 4.126;

// float s2p00 = 1757;
// float s2p10 = 4.977;
// float s2p01 = 0.2432;

// float k1 = ((1498.0-1342.0)/-300.0);
// float k2 = ((1668.0-1495.0)/-200.0);

// // Define functions to calculate S1 and S2 based on CamX and CamY
// float modelS1(float CamY) {
//     return CENTER_X + k1*CamY;
// }

// float modelS2(float CamX) {
//     return CENTER_Y + k2*CamX;
// }

void homing() {
  Serial.println("Starting homing");
  while (digitalRead(HOME_SWITCH_X) == HIGH) {
    Serial.println("Homing...");
    // stepperX.moveTo(stepperX.currentPosition() - 10);
    stepperX.setSpeed(-10000);
    stepperX.runSpeed();
    delay(1);
  }
  stepperX.setCurrentPosition(0);
  Serial.println("Homing completed");
}
