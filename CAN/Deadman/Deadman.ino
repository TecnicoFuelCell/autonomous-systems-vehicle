#include <canIds.h>
#include <mcp_can.h>
#include <SPI.h>
#include "can_diagnostics.h"

#define PIN_CS  D7
#define PIN_SCK D8
#define PIN_MISO D9
#define PIN_MOSI D10

#define LED_PIN D5
#define DEADMAN_PIN D1
#define BRAKE_PIN D2

MCP_CAN CAN(PIN_CS);
unsigned long lastDeadmanPressTime = 0;
CanDiagnosticsState canDiag(1000);

void setup() {
    pinMode(DEADMAN_PIN, INPUT);
    pinMode(BRAKE_PIN, INPUT);
    pinMode(LED_PIN, OUTPUT);
    
    digitalWrite(LED_PIN, LOW); 

    Serial.begin(115200);
    delay(500);
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
    bool brakeDigital = digitalRead(BRAKE_PIN);
    bool deadmanPressed = digitalRead(DEADMAN_PIN) == LOW;
    unsigned long nowMs = millis();
    bool safe = (nowMs - lastDeadmanPressTime <= 500) && !brakeDigital;

    // debug
    Serial.printf(
        "[DEBUG] deadman=%d brake_digital=%d last=%lu now=%lu\n",
        deadmanPressed,
        brakeDigital,
        lastDeadmanPressTime,
        nowMs,
        safe
    );

    if (deadmanPressed) {
        Serial.println("[DEBUG] Deadman Pressed");
        lastDeadmanPressTime = millis();
    }

    if (safe) {
        byte msg = 1;
        byte flag = CAN.sendMsgBuf(deadman_msg_instance.CAN_ID, 0, 1, &msg); 
        digitalWrite(LED_PIN, HIGH); 
        Serial.printf("[DEBUG] Sending alive | CAN Flag: %d | My ID: %d\n", flag, deadman_msg_instance.CAN_ID);
    } else {
        Serial.println("[DEBUG] NOT SAFE!");
        digitalWrite(LED_PIN, LOW);
    }

    checkCanDiagnostics(CAN, canDiag, "Deadman", nowMs);

    delay(50);
}