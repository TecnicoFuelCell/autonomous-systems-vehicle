#pragma once
#include <Arduino.h> //required for PD# definitions

class CAN
{

public:

#define CAN0_INT 21                              // Set INT to pin 21

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];// Array to store serial string


float inpVoltage, dutyCycleNow, avgInputCurrent, avgMotorCurrent, tempFET, tempMotor;
long erpm, WattHours;
uint8_t error = -1;

void initialize(int cs_pin, int sck_pin, int miso_pin, int mosi_pin, unsigned long can_speed, byte mcp_clock);
void spin();
void get_frame(); // populates rxId and rxBuf with latest can frame 
byte can_send(uint16_t canId, uint8_t ext, uint8_t len, byte data[8]); //transmits the send commands to the sensor
void print_raw_can_data(); //output raw can data to terminal (debug)


void vesc_set_duty(float duty);
void vesc_set_current(float current);
void vesc_set_brake_current(float brake_current);
void vesc_set_handbrake(float handbrake_current);
void vesc_set_erpm(float erpm);
float process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2);
int hex2int(char buf[]);

// Contadores de erro do MCP2515 (TEC/REC). Sobem com erros no bus,
// descem em transmissões/receções bem sucedidas.
//   <96 = ok, >=96 = warning, >=128 = passive, =255 (TEC) = bus-off
uint8_t errorTX();      // devolve TEC (Transmit Error Counter)
uint8_t errorRX();      // devolve REC (Receive Error Counter)
uint8_t errorFlags();   // devolve EFLG — bits: BUS-OFF/TXEP/RXEP/TXWAR/RXWAR/RX0OVR/RX1OVR
void clearRxOverflow(); // limpa os bits latched RX0OVR/RX1OVR do EFLG. Chamar após reportar.

private:
int _cs_pin;

};
