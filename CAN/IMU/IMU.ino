#include "ICM20600.h"
#include "AK09918.h" // Biblioteca comum para o Magnetómetro que acompanha o ICM20600
#include <Wire.h>
#include <SPI.h>
#include <mcp_can.h>
#include <canIds.h>
#include "can_diagnostics.h"
#include <stdlib.h>

/* --- PIN DEFINITIONS (XIAO ESP32C3) --- */
#define PIN_SCK D8
#define PIN_MISO D9
#define PIN_MOSI D10
#define PIN_CS D7

ICM20600 icm20600(true);
AK09918 ak09918; // Instância do magnetómetro

MCP_CAN CAN(PIN_CS);

// Inicializa Janela do Acelerómetro fora do loop - menos overhead por iteração
int16_t acc_x_window[10];
int16_t acc_y_window[10];
int16_t acc_z_window[10];
unsigned int acc_window_counter = 0;

unsigned char buf[8] = {0};
unsigned long lastCanSend = 0;
CanDiagnosticsState canDiag(1000);

int compare(const void* a, const void* b) {
    return (*(int*)a - *(int*)b);
}

int16_t sort_and_get_median(int16_t* array, int array_size){
  qsort(array, array_size, sizeof(int16_t), compare);
  return (array[4]+array[5])/2 ;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Wire.begin();   // Inicia o bus I2C

  // Inicializa ICM20600 (Acelerómetro + Giroscópio)
  icm20600.initialize();
  Serial.println("IMU initialized");

  // Inicializa SPI e CAN
  // Inicializa AK09918 (Magnetómetro)
  AK09918_err_type_t err = ak09918.initialize();
  ak09918.switchMode(AK09918_CONTINUOUS_100HZ); // Configura para ler a 100Hz
  if (err == AK09918_ERR_OK) {
    Serial.println("Magnetometer initialized");
  } else {
    Serial.println("Magnetometer init FAILED!");
  }

  // Inicializa SPI e CAN
  pinMode(PIN_CS, OUTPUT);
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  byte status = CAN.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ);

  if (status == CAN_OK) {
    Serial.println("CAN init OK!");
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.print("CAN init FAILED. Error: ");
    Serial.println(status);
  }
  

  Serial.println("CAN BUS OK!");
}

void loop() {
  unsigned long nowMs = millis();

  checkCanDiagnostics(CAN, canDiag, "IMU", nowMs);

  // 50Hz loop frequency (20ms)
  if (nowMs - lastCanSend > 20) {
    lastCanSend = nowMs;

    // Leituras da IMU 6-DoF
    // Leituras da IMU 6-DoF
    int16_t acc_x = icm20600.getAccelerationX();
    int16_t acc_y = icm20600.getAccelerationY();
    int16_t acc_z = icm20600.getAccelerationZ();

    if (acc_window_counter <= 9){
      acc_x_window[acc_window_counter] = acc_x;
      acc_y_window[acc_window_counter] = acc_y;
      acc_z_window[acc_window_counter] = acc_z;
      acc_window_counter ++;
    }
    else{
      acc_window_counter = 0;
      acc_x = sort_and_get_median(acc_x_window, 10);
      acc_y = sort_and_get_median(acc_y_window, 10);
      acc_z = sort_and_get_median(acc_z_window, 10);

      /* ================= ACC MESSAGE ================= */
      buf[0] = acc_x & 0xFF;
      buf[1] = (acc_x >> 8) & 0xFF;
      buf[2] = acc_y & 0xFF;
      buf[3] = (acc_y >> 8) & 0xFF;
      buf[4] = acc_z & 0xFF;
      buf[5] = (acc_z >> 8) & 0xFF;
      buf[6] = 0;
      buf[7] = 0;

      byte flag1 = CAN.sendMsgBuf(
          imu_acc_message_instance.CAN_ID,
          0,
          imu_acc_message_instance.dlc,
          buf
      );
      Serial.print("flag1: ");
      Serial.println(flag1);
    }

    int16_t gyro_x = icm20600.getGyroscopeX();
    int16_t gyro_y = icm20600.getGyroscopeY();
    int16_t gyro_z = icm20600.getGyroscopeZ();

    // Leituras do Magnetómetro
    int32_t mag_x_32, mag_y_32, mag_z_32;
    ak09918.getData(&mag_x_32, &mag_y_32, &mag_z_32);

    // Converter para 16 bits para consistência no CAN (ajusta se precisares de precisão de 32 bits)
    int16_t mag_x = (int16_t)mag_x_32;
    int16_t mag_y = (int16_t)mag_y_32;
    int16_t mag_z = (int16_t)mag_z_32;

    /* ================= GYRO MESSAGE ================= */
    buf[0] = gyro_x & 0xFF;
    buf[1] = (gyro_x >> 8) & 0xFF;
    buf[2] = gyro_y & 0xFF;
    buf[3] = (gyro_y >> 8) & 0xFF;
    buf[4] = gyro_z & 0xFF;
    buf[5] = (gyro_z >> 8) & 0xFF;
    buf[6] = 0;
    buf[7] = 0;

    byte flag2 = CAN.sendMsgBuf(
        imu_gyro_message_instance.CAN_ID,
        0,
        imu_gyro_message_instance.dlc,
        buf
    );
    Serial.print("flag2: ");
    Serial.println(flag2);

    /* ================= MAG MESSAGE ================= */
    buf[0] = mag_x & 0xFF;
    buf[1] = (mag_x >> 8) & 0xFF;
    buf[2] = mag_y & 0xFF;
    buf[3] = (mag_y >> 8) & 0xFF;
    buf[4] = mag_z & 0xFF;
    buf[5] = (mag_z >> 8) & 0xFF;
    buf[6] = 0;
    buf[7] = 0;

    // Assegura-te que 'imu_mag_message_instance' existe no teu canIds.h
    byte flag3 = CAN.sendMsgBuf(
        imu_mag_message_instance.CAN_ID,
        0,
        imu_mag_message_instance.dlc,
        buf
    );
    Serial.print("Mag CAN Flag: ");
    Serial.println(flag3);

    Serial.printf("Mag CAN ID: %d\n", imu_mag_message_instance.CAN_ID);

    // Debug local
    Serial.print("ACC: ");
    Serial.print(acc_x); Serial.print(" ");
    Serial.print(acc_y); Serial.print(" ");
    Serial.print(acc_z);

    Serial.print(" | GYRO: ");
    Serial.print(gyro_x); Serial.print(" ");
    Serial.print(gyro_y); Serial.print(" ");
    Serial.print(gyro_z);

    Serial.print(" | MAG: ");
    Serial.print(mag_x); Serial.print(" ");
    Serial.print(mag_y); Serial.print(" ");
    Serial.println(mag_z);
  }
}
