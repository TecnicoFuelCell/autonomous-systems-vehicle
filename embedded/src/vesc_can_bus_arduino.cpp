#include <mcp_can.h>
#include <SPI.h>
#include "vesc_can_bus_arduino.h"

MCP_CAN *CAN0 = nullptr;

static const uint8_t VESC_CAN_ID = 0x0A;
static const uint8_t CAN_PACKET_SET_DUTY = 0;
static const uint8_t CAN_PACKET_SET_CURRENT = 1;
static const uint8_t CAN_PACKET_SET_CURRENT_BRAKE = 2;
static const uint8_t CAN_PACKET_SET_RPM = 3;
static const uint8_t CAN_PACKET_SET_CURRENT_HANDBRAKE = 12;

static unsigned long make_vesc_can_id(uint8_t packet_id) {
  return ((unsigned long)packet_id << 8) | VESC_CAN_ID;
}

static byte send_vesc_int32(uint8_t packet_id, int32_t set_value) {
  uint8_t buffer[4];
  buffer[0] = (set_value >> 24) & 0xFF;
  buffer[1] = (set_value >> 16) & 0xFF;
  buffer[2] = (set_value >> 8) & 0xFF;
  buffer[3] = set_value & 0xFF;
  return CAN0->sendMsgBuf(make_vesc_can_id(packet_id), 1, 4, buffer);
}

void CAN::initialize(int cs_pin, int sck_pin, int miso_pin, int mosi_pin, unsigned long can_speed, byte mcp_clock) {
  _cs_pin = cs_pin;
  
  // Inicializar SPI com pinos customizados
  pinMode(_cs_pin, OUTPUT);
  digitalWrite(_cs_pin, HIGH);
  SPI.begin(sck_pin, miso_pin, mosi_pin, _cs_pin);
  
  // Criar instância do MCP_CAN com pino CS customizado
  if (CAN0 != nullptr) delete CAN0;
  CAN0 = new MCP_CAN(_cs_pin);
  
  CAN0->begin(MCP_ANY, can_speed, mcp_clock);
  CAN0->setMode(MCP_NORMAL);
}

void CAN::spin() {
  get_frame();

  if (rxId == 0x8000090A) { //
    dutyCycleNow = process_data_frame_vesc('D', rxBuf[6], rxBuf[7]);
    avgMotorCurrent = process_data_frame_vesc('C', rxBuf[4], rxBuf[5]);
    unsigned char erpmvals[4];
    erpmvals[0] = rxBuf[3];
    erpmvals[1] = rxBuf[2];
    erpmvals[2] = rxBuf[1];
    erpmvals[3] = rxBuf[0];
    erpm = *(long *)erpmvals;

    //need to add in the rpm conversion function for 4 byte values
  }
  if (rxId == 0x80000F0A) {
    unsigned char WHvals[4];
    WHvals[0] = rxBuf[3];
    WHvals[1] = rxBuf[2];
    WHvals[2] = rxBuf[1];
    WHvals[3] = rxBuf[0];
    WattHours = *(long *)WHvals;
  }
  if (rxId == 0x8000100A) { //
    tempFET = process_data_frame_vesc('F', rxBuf[0], rxBuf[1]);
    tempMotor = process_data_frame_vesc('T', rxBuf[2], rxBuf[3]);
    avgInputCurrent = process_data_frame_vesc('I', rxBuf[4], rxBuf[5]);
  }
  if (rxId == 0x80001B0A) {
    char receivedByte[4], *p;
    sprintf(receivedByte, "%02X%02X", rxBuf[4], rxBuf[5]);
    inpVoltage = hex2int(receivedByte) * 0.1;
  }
  if (rxId == 0x80000E0A) {
    error = rxBuf[0];
  }
  // print_raw_can_data()  // uncomment to see raw can messages
}

void CAN::print_raw_can_data() {
  int len = 8;
  sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  Serial.print(msgString);
  for (byte i = 0; i < len; i++) {
    sprintf(msgString, " 0x%.2X", rxBuf[i]);
    Serial.print(msgString);
  }
  Serial.println();
}

float CAN::process_data_frame_vesc(char datatype, unsigned char byte1, unsigned char byte2) {
  char receivedByte[4], *p;
  sprintf(receivedByte, "%02X%02X", byte1, byte2);
  float output = hex2int(receivedByte);

  switch (datatype) {
    case 'D': output *= 0.001; break; //dutyCycleNow
    case 'C': output *= 0.1; break; //avgMotorCurrent
    case 'F': output *= 0.1; break; //tempFET
    case 'T': output *= 0.1; break; //tempMotor
    case 'I': output *= 0.1; break; //avgInputCurrent
    case 'V': output *= 0.1; break; //inpVoltage
  }
  return output;
}

int CAN::hex2int(char buf[])
{
  return (short) strtol(buf, NULL, 16);
}

void CAN::vesc_set_duty(float duty) {
  int32_t set_value = duty * 100000;
  send_vesc_int32(CAN_PACKET_SET_DUTY, set_value);
}

void CAN::vesc_set_current(float current) {
  int32_t set_value = current * 1000;
  send_vesc_int32(CAN_PACKET_SET_CURRENT, set_value);
}

void CAN::vesc_set_brake_current(float brake_current) {
  if (brake_current < 0) {
    brake_current = -brake_current;
  }
  int32_t set_value = brake_current * 1000;
  send_vesc_int32(CAN_PACKET_SET_CURRENT_BRAKE, set_value);
}

void CAN::vesc_set_handbrake(float handbrake_current) {
  if (handbrake_current < 0) {
    handbrake_current = -handbrake_current;
  }
  int32_t set_value = handbrake_current * 1000;
  send_vesc_int32(CAN_PACKET_SET_CURRENT_HANDBRAKE, set_value);
}

void CAN::vesc_set_erpm(float erpm) {
  int32_t set_value = erpm;
  send_vesc_int32(CAN_PACKET_SET_RPM, set_value);
}
void CAN::get_frame() {
  if (CAN0 != nullptr && CAN0->checkReceive() == CAN_MSGAVAIL) {
    CAN0->readMsgBuf(&rxId, &len, rxBuf);
  }
}

byte CAN::can_send(uint16_t canId, uint8_t ext, uint8_t len, byte data[8]) {
  return CAN0->sendMsgBuf(canId, ext, len, data);
}

uint8_t CAN::errorTX() {
  return CAN0 ? CAN0->errorCountTX() : 0;
}

uint8_t CAN::errorRX() {
  return CAN0 ? CAN0->errorCountRX() : 0;
}

uint8_t CAN::errorFlags() {
  return CAN0 ? CAN0->getError() : 0;
}

void CAN::clearRxOverflow() {
  // Limpa os bits latched RX0OVR/RX1OVR do EFLG via bit-modify SPI direto.
  if (CAN0 == nullptr) return;
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_cs_pin, LOW);
  SPI.transfer(MCP_BITMOD);                          // 0x05 — comando bit-modify
  SPI.transfer(MCP_EFLG);                            // 0x2D — registo EFLG
  SPI.transfer(MCP_EFLG_RX0OVR | MCP_EFLG_RX1OVR);   // máscara: bits a afetar
  SPI.transfer(0x00);                                // data: pôr os bits a 0
  digitalWrite(_cs_pin, HIGH);
  SPI.endTransaction();
}
