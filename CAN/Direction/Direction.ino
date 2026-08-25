#include <canIds.h>
#include <mcp_can.h>
#include <SPI.h>
#include "can_diagnostics.h"

// Defin pins
#define driverPUL D5  // PUL- pin
#define driverDIR D6  // DIR- pin

#define PIN_CS D7
#define PIN_SCK D8
#define PIN_MISO D9
#define PIN_MOSI D10  //Chip Select do CAN

//CAN variables
MCP_CAN CAN(PIN_CS);                       //Define o CS do CAN
volatile bool canMessageReceived = false;  // Flag for interrupt
unsigned char CANbuf[8];                   //buffer das mensagens CAN
unsigned char CANbuf2[8];                  //buffer das mensagens CAN para enviar
long unsigned int rxId;
unsigned char len = 0;
 
//Stepper Variables
unsigned int pd = 600;       // Pulse Delay period
float PulsePerRev = 400;  // Number of pulses for 1 rotation
int CurrentAngle = 0;  //Registerd angle for stepper motor
boolean setdir = LOW; // Set Direction

int CurrentStep = 0;

int SendTimer = 0;

CanDiagnosticsState canDiag(1000);

void setup() {
 
  Serial.begin(9600);
  pinMode(driverPUL, OUTPUT);
  pinMode(driverDIR, OUTPUT);
  pinMode(PIN_CS, OUTPUT);



  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  while (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("CANSPI_CS_PIN init failed, retrying...");
    delay(200);
  }
  Serial.println("CAN init ok!");
  CAN.setMode(MCP_NORMAL);
}

void loop() {

  short int newAngle = CANAngle();
  CurrentAngle = (int)newAngle;
  rotate();
  //Serial.println(CurrentAngle);
  //Sending Vesc values for ROS
  if(SendTimer <= 0){
    SendTimer = 20;
    sendCanCurrentAngle();
  }else{
    SendTimer--;
  }

  checkCanDiagnostics(CAN, canDiag, "Dir");
}

/**
* Function:     CANAngle
* Arguments:
*               
* Returns:      num                       -Int number detected from CAN communication from ID SenderCAN_ID
* Description:  Receives a int number threw CAN and returns that number if it came from ID SenderCAN_ID
**/
int CANAngle() {
  if (CAN_MSGAVAIL == CAN.checkReceive()) {
    CAN.readMsgBuf(&rxId, &len, CANbuf);  // read data,  len: data length, buf: data buf
    Serial.println(rxId);

    if (rxId == dir_msg_instance.CAN_ID) {
      int16_t steeringAngle = (CANbuf[0] << 8) | CANbuf[1];
      Serial.print("[CAN] ");
      Serial.println(steeringAngle);
      return steeringAngle;
    }
  }
  return CurrentAngle;
}

float angleToStep(int angle){
  return (float) (PulsePerRev/360 * (angle));
}

void rotate(){

  float refStep = angleToStep(CurrentAngle);
  if ((int)refStep > CurrentStep){
    //Rotate to left
    digitalWrite(driverDIR,LOW);

    //Rotate motor
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pd);

    //add one step
    CurrentStep++;
  }else if((int)refStep < CurrentStep){
    //Rotate to RIGHT
    digitalWrite(driverDIR,HIGH);

    //Rotate motor
    digitalWrite(driverPUL,HIGH);
    delayMicroseconds(pd);
    digitalWrite(driverPUL,LOW);
    delayMicroseconds(pd);

    //remove one step
    CurrentStep--;
  } else {
    Serial.printf("Current step: %d\nCurrent angle: %d\n", CurrentStep, CurrentAngle);
  }
}

byte sendCanCurrentAngle(){
  short int angle2 = (short int) (CurrentStep*360 / (PulsePerRev));
  CANbuf2[0] = (0>>8) & 0xFF;
  CANbuf2[1] = (0) & 0xFF;
  CANbuf2[2] = (angle2>>8) & 0xFF;
  CANbuf2[3] = (angle2) & 0xFF;
  byte flag = CAN.sendMsgBuf(cur_dir_msg_instance.CAN_ID, 0, 8, CANbuf2);
  Serial.printf("[CAN, SENDING] Sending: %d. Flag: %d\n", angle2, flag);
  //Serial.println(angle2);
  
  return flag;
}