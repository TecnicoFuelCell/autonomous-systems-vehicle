#include <Wire.h>
#include <math.h>
#include "bmm150.h" // 

// ================= CAN & SPI =================
#include <SPI.h>
#include <mcp_can.h>
#include <canIds.h>
#include "can_diagnostics.h"

#define PIN_SCK D8
#define PIN_MISO D9
#define PIN_MOSI D10
#define PIN_CS D7

MCP_CAN CAN(PIN_CS);
// can diagnostic logger instance (1000 means it scans/logs second)
CanDiagnosticsState canDiag(1000);
unsigned char buf[8] = {0};
unsigned long lastCanSend = 0;

BMM150 bmm = BMM150();

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();
  
  Serial.println("\n--- Iniciar Sistema de Orientacao (BMM150) ---");

  // sensor init
  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    Serial.println("ERRO: O chip nao e um BMM150 (ou falha de comunicacao).");
    while (1); // Bloqueia o código aqui se falhar
  }
  
  Serial.println("Compasso BMM150 inicializado com SUCESSO!");

  // =================  CAN SETUP =================
  pinMode(PIN_CS, OUTPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  byte status = CAN.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ);

  if (status == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    Serial.println("CAN inicializado com SUCESSO!");
  } else {
    Serial.print("FALHA no CAN. Erro: ");
    Serial.println(status);
  }

  Serial.println("----------------------------------------------");
  delay(1000);
}

void loop() {
  // send at 100Hz (~10ms)
  if (millis() - lastCanSend > 10) {
    lastCanSend = millis();
    
    bmm.read_mag_data();

    float mag_x_float = bmm.raw_mag_data.raw_datax;
    float mag_y_float = bmm.raw_mag_data.raw_datay;
    float mag_z_float = bmm.raw_mag_data.raw_dataz;

    // =================  CAN LOGIC =================
    // convert to 16bit
    int16_t mag_x = (int16_t)mag_x_float;
    int16_t mag_y = (int16_t)mag_y_float;
    int16_t mag_z = (int16_t)mag_z_float;

    buf[0] = mag_x & 0xFF;
    buf[1] = (mag_x >> 8) & 0xFF;
    buf[2] = mag_y & 0xFF;
    buf[3] = (mag_y >> 8) & 0xFF;
    buf[4] = mag_z & 0xFF;
    buf[5] = (mag_z >> 8) & 0xFF;
    buf[6] = 0;
    buf[7] = 0;

    byte can_status = CAN.sendMsgBuf(
        imu_mag_message_instance.CAN_ID,
        0,
        imu_mag_message_instance.dlc,
        buf
    );  
    /* ================= HEADING CALCULATION ================= */
    float heading = atan2(mag_x_float, mag_z_float) * 180.0 / PI;

    // normalize for 0-360 degree interval
    if (heading < 0) {
      heading += 360.0;
    }
    // Imprimir resultados
    Serial.print("MAG X: "); Serial.print(mag_z_float);
    Serial.print("\t | Y: "); Serial.print(mag_y_float);
    Serial.print("\t | Z: "); Serial.print(mag_x_float);
    Serial.print("\t | CAN Send Status: "); Serial.print(can_status);
    Serial.print("\t | Orientacao: "); Serial.print(heading);
    Serial.println(" graus");
  }
  // 1 Hz, only prints when there's an issue
  checkCanDiagnostics(CAN, canDiag, "Magnetometer");
}