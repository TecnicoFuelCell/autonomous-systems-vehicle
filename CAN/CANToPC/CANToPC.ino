/*
 * (1) Receives data via CAN from the VESC
 * (2) Sends it to the PC via UART
 * We decouple 1 and 2 such that they can run on different cores
*/

#include <canIds.h>
#include <mcp_can.h>
#include <SPI.h>
#include "vesc_can_bus_arduino.h"
#include "can_diagnostics.h"

/** Initiate VescUart class */

//HardwareSerial VescSerial(1);

/* IMPORTANT: Commented this out.
   Let mcp_can.h define the correct register value for 500KBPS.
   Manually defining it to 16 is common in other libraries but might break this one.
*/
// #define CAN_500KBPS 16

/* CS_PIN for ESP-32 */
static const int PIN_SCK   = D8;
static const int PIN_MISO  = D9;
static const int PIN_MOSI  = D10;
static const int PIN_CS    = D7;

CAN can;

// Tracking de "last seen" do heartbeat do PCSender.
unsigned long pcsender_last_seen = 0;
const unsigned long alive_timeout = 5000;   // ms — 5x o intervalo de heartbeat

void CANReceiverTask(void *pvParameters) {
    // Monitorização TEC/REC do MCP2515 a 1 Hz. Mantido nesta task porque é
    // a única que acede ao SPI — evita conflito com CANtoPCTask.
    static CanDiagnosticsState canDiag(1000);

    for (;;) {
        // Updated API call here
        can.spin();

        // Só monotoria o heartbeat do PCSender 
        if (can.rxId == pcsender_heartbeat_msg_instance.CAN_ID) {
            pcsender_last_seen = millis();
        }

        // -------------------------------------------------
        // Monitorização CAN a 1 Hz — TEC/REC + EFLG do MCP2515.
        // Só imprime quando há problema
        // -------------------------------------------------
        checkCanDiagnostics(can, canDiag, "CANtoPC");

        if (can.rxId == cur_dir_msg_instance.CAN_ID) {
            short int angle = (can.rxBuf[2] << 8) | can.rxBuf[3];
            Serial.print("Dir: ");
            Serial.println(angle / 5);
            Serial.flush();
        }
        if (can.rxId == imu_acc_message_instance.CAN_ID) {
            int16_t acc_x = (can.rxBuf[1] << 8) | can.rxBuf[0];
            int16_t acc_y = (can.rxBuf[3] << 8) | can.rxBuf[2];
            int16_t acc_z = (can.rxBuf[5] << 8) | can.rxBuf[4];

            Serial.print("ACC:");
            Serial.print(acc_x); Serial.print(",");
            Serial.print(acc_y); Serial.print(",");
            Serial.println(acc_z);
            Serial.flush();
        }
        if (can.rxId == imu_gyro_message_instance.CAN_ID) {
            int16_t gyro_x = (can.rxBuf[1] << 8) | can.rxBuf[0];
            int16_t gyro_y = (can.rxBuf[3] << 8) | can.rxBuf[2];
            int16_t gyro_z = (can.rxBuf[5] << 8) | can.rxBuf[4];

            Serial.print("GYRO:");
            Serial.print(gyro_x); Serial.print(",");
            Serial.print(gyro_y); Serial.print(",");
            Serial.println(gyro_z);
            Serial.flush();
        }
        /* ================= MAG MESSAGE ================= */
        if (can.rxId == imu_mag_message_instance.CAN_ID) {
            int16_t mag_x = (can.rxBuf[1] << 8) | can.rxBuf[0];
            int16_t mag_y = (can.rxBuf[3] << 8) | can.rxBuf[2];
            int16_t mag_z = (can.rxBuf[5] << 8) | can.rxBuf[4];

            Serial.print("MAG:");
            Serial.print(mag_x); Serial.print(",");
            Serial.print(mag_y); Serial.print(",");
            Serial.println(mag_z);
            Serial.flush();
        }
        /* =============================================== */
        if (can.rxId == deadman_msg_instance.CAN_ID) {
            Serial.println("ALIVE");
            Serial.flush();
            //if (can.rxBuf[0] == 1){
            //    Serial.println("ALIVE");
            //    Serial.flush();
            //}
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void CANtoPCTask(void *pvParameters) {
    const unsigned long pcSendingTs    = 100;
    const unsigned long statusInterval = 1000;   // 1 Hz
    unsigned long lastSend   = millis();
    unsigned long lastStatus = millis();

    for (;;) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastSend >= pcSendingTs) {
            lastSend = currentMillis;

            // send to PC via Serial
            Serial.print("VESC:");
            Serial.printf("%f,%f,%f,%f,%ld,%f,%f\n",
                          can.tempFET,
                          can.avgMotorCurrent,
                          can.avgInputCurrent,
                          can.dutyCycleNow,
                          can.erpm,
                          can.inpVoltage,
                          can.WattHours);
            Serial.flush();
        }

        // Linha STATUS: formato- <id>=<state>
        if (currentMillis - lastStatus >= statusInterval) {
            lastStatus = currentMillis;

            bool alive = (currentMillis - pcsender_last_seen) < alive_timeout;
            Serial.print("STATUS:");
            Serial.print(pcsender_heartbeat_msg_instance.CAN_ID);
            Serial.print("=");
            Serial.println(alive ? 1 : 0);
            Serial.flush();
        }

        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);

    while(!Serial){
      delay(10);
    }

    can.initialize(PIN_CS, PIN_SCK, PIN_MISO, PIN_MOSI, CAN_250KBPS, MCP_16MHZ);

    Serial.println("[CANtoPC] CAN BUS OK!");
    Serial.println("[CANtoPC] STATUS: emission @ 1 Hz");
    Serial.println("---------- CANtoPC.INO ----------");

    xTaskCreatePinnedToCore(CANReceiverTask, "CAN Receiver", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(CANtoPCTask, "CANtoPCTask", 4096, NULL, 1, NULL, 0);
}

void loop() {
    // Empty as expected
}
