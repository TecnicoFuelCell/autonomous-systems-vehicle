#pragma once

// Diagnóstico CAN partilhado por todos os ESPs.
//
// Imprime via Serial os contadores de erro do MCP2515 (TEC, REC) e o
// registo EFLG descodificado (BUS-OFF, error-passive, warnings, RX overflow).
// Útil quando se liga uma porta Serial directamente a um ESP para perceber
// se há problema no bus CAN e qual é.
//
// Uso típico no .ino:
//     #include "can_diagnostics.h"
//     CanDiagnosticsState canDiag(1000);
//     void loop() {
//         checkCanDiagnostics(can, canDiag, "PCSender");
//     }

#include <Arduino.h>
#include "mcp_can.h"
#include "vesc_can_bus_arduino.h"
#include "mcp_can_dfs.h"

struct CanDiagnosticsState {
    unsigned long lastCheckMs;
    unsigned long intervalMs;
    uint8_t warningThreshold;

    explicit CanDiagnosticsState(unsigned long interval = 1000,
                                 uint8_t threshold = 96)
        : lastCheckMs(0), intervalMs(interval), warningThreshold(threshold) {}
};

inline void printCanDiagnosticsValues(const char* nodeName,
                                      uint8_t tec,
                                      uint8_t rec,
                                      uint8_t eflg) {
    // Contadores + EFLG em hex + lista compacta de flags ligadas.
    Serial.print("[");
    Serial.print(nodeName);
    Serial.print("] TEC=");
    Serial.print(tec);
    Serial.print(" REC=");
    Serial.print(rec);
    Serial.print(" EFLG=0x");
    if (eflg < 0x10) Serial.print('0');
    Serial.print(eflg, HEX);
    Serial.print(" [");
    if (eflg & MCP_EFLG_TXBO)   Serial.print("BUS-OFF ");
    if (eflg & MCP_EFLG_TXEP)   Serial.print("TXEP ");
    if (eflg & MCP_EFLG_RXEP)   Serial.print("RXEP ");
    if (eflg & MCP_EFLG_TXWAR)  Serial.print("TXWAR ");
    if (eflg & MCP_EFLG_RXWAR)  Serial.print("RXWAR ");
    if (eflg & MCP_EFLG_RX0OVR) Serial.print("RX0OVR ");
    if (eflg & MCP_EFLG_RX1OVR) Serial.print("RX1OVR ");
    if (eflg == 0)              Serial.print("OK");
    Serial.println("]");

    // Descrição dos flags- só quaundo há erro (senão, fica o código acima)
    if (eflg & MCP_EFLG_TXBO)
        Serial.println("  BUS-OFF: TEC>=256, no desligado do bus, no TX/RX ate reset");
    if (eflg & MCP_EFLG_TXEP)
        Serial.println("  TXEP   : TEC>=128, TX em error-passive (frames sem ACK)");
    if (eflg & MCP_EFLG_RXEP)
        Serial.println("  RXEP   : REC>=128, RX em error-passive (frames corrompidos a chegar)");
    if (eflg & MCP_EFLG_TXWAR)
        Serial.println("  TXWAR  : TEC>=96, aviso de erros de TX");
    if (eflg & MCP_EFLG_RXWAR)
        Serial.println("  RXWAR  : REC>=96, aviso de erros de RX");
    if (eflg & MCP_EFLG_RX0OVR)
        Serial.println("  RX0OVR : buffer RX0 cheio, frame(s) perdido(s)");
    if (eflg & MCP_EFLG_RX1OVR)
        Serial.println("  RX1OVR : buffer RX1 cheio, frame(s) perdido(s)");
}

inline void readCanDiagnostics(CAN& can,
                               uint8_t& tec,
                               uint8_t& rec,
                               uint8_t& eflg) {
    tec = can.errorTX();
    rec = can.errorRX();
    eflg = can.errorFlags();
}

inline void readCanDiagnostics(MCP_CAN& can,
                               uint8_t& tec,
                               uint8_t& rec,
                               uint8_t& eflg) {
    tec = can.errorCountTX();
    rec = can.errorCountRX();
    eflg = can.getError();
}

inline void printCanDiagnostics(CAN& can, const char* nodeName) {
    uint8_t tec;
    uint8_t rec;
    uint8_t eflg;
    readCanDiagnostics(can, tec, rec, eflg);
    printCanDiagnosticsValues(nodeName, tec, rec, eflg);
}

inline void printCanDiagnostics(MCP_CAN& can, const char* nodeName) {
    uint8_t tec;
    uint8_t rec;
    uint8_t eflg;
    readCanDiagnostics(can, tec, rec, eflg);
    printCanDiagnosticsValues(nodeName, tec, rec, eflg);
}

template <typename CanBus>
inline bool checkCanDiagnostics(CanBus& can,
                                CanDiagnosticsState& state,
                                const char* nodeName,
                                unsigned long nowMs = millis()) {
    if (nowMs - state.lastCheckMs < state.intervalMs) {
        return false;
    }

    state.lastCheckMs = nowMs;

    uint8_t tec;
    uint8_t rec;
    uint8_t eflg;
    readCanDiagnostics(can, tec, rec, eflg);

    if (tec >= state.warningThreshold ||
        rec >= state.warningThreshold ||
        eflg != 0) {
        printCanDiagnosticsValues(nodeName, tec, rec, eflg);
        return true;
    }

    return false;
}
