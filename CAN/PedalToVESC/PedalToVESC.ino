/*
VESC Monitor & Control — ESP32-C3 com MCP2515 @250kbps + Pedal
Função: recebe dados do VESC via CAN, faz print e permite controlo via Serial ou Pedal
Ligações: SCK=D8, MISO=D9, MOSI=D10, CS=D7; VCC1→5V, VCC→3V3, GND comum.
Pedal: A0 (3 cabos: GND, A0, 3V3) - potenciómetro lido a 12-bit (0–4095) e recalibrado
Bus: CANH↔CANH, CANL↔CANL; 120 Ω só nas pontas (≈60 Ω H-L a frio).
Parâmetros: CAN_250KBPS, MCP_16MHZ

Comandos Serial:
  d <valor>  - Define duty cycle (-1.0 a 1.0, ex: "d 0.5")
  c <valor>  - Define corrente em A (ex: "c 5.0")
  e <valor>  - Define ERPM (ex: "e 10000")
  s          - Stop motor (duty = 0)
  p          - Toggle modo pedal ON/OFF
  m <valor>  - Define corrente máxima do pedal (ex: "m 20.0")
  h          - Mostra ajuda
*/

#include <mcp_can.h>
#include "vesc_can_bus_arduino.h"
#include "canIds.h"

// Definição dos pinos (mesmos do emissor.ino / XIAO ESP32-C3)
static const int PIN_SCK   = D8;
static const int PIN_MISO  = D9;
static const int PIN_MOSI  = D10;
static const int PIN_CS    = D7;
static const int PIN_PEDAL = A0;   // XIAO ESP32-C3: A0 = GPIO 2

CAN can;

unsigned long lastSendTime = 0;
const int CAN_INTERVAL = 200;
unsigned long lastPrint = 0;
const unsigned long PRINT_INTERVAL = 200; // print a cada 200ms
String cmd = "";
unsigned char buf[2] = {0}; 

// ---------------- Pedal / ADC config (from second sketch) ----------------
const int INPUT_MIN = 1300;
const int INPUT_MAX = 3200;

bool   pedalEnabled      = true;   // modo pedal ativo por padrão
float  maxPedalCurrent   = 50.0;   // corrente máxima do pedal (A)
uint16_t pedalFiltered   = 0;      // EMA sobre valor reescalado (0-4095)
uint16_t lastAdcRaw       = 0;      // último valor ADC cru (0-4095)
uint16_t lastRescaled     = 0;      // último valor reescalado (0-4095)

unsigned long lastPedalRead = 0;
const unsigned long PEDAL_INTERVAL = 20; // lê pedal a cada 20ms (~50Hz)

// Filtro exponencial (EMA) aplicado ao valor reescalado 0–4095
#define ALPHA_NUM      1
#define ALPHA_DEN      5
// ------------------------------------------------------------------------

void printVESCData() {
  // Pequeno dump legivel de todos os valores recebidos do VESC
  Serial.println("========== VESC DATA ==========");
  Serial.printf("ERPM:              %ld\n", can.erpm);
  Serial.printf("Duty Cycle:        %.3f %%\n", can.dutyCycleNow * 100.0);
  Serial.printf("Input Voltage:     %.2f V\n", can.inpVoltage);
  Serial.printf("Input Current:     %.2f A\n", can.avgInputCurrent);
  Serial.printf("Motor Current:     %.2f A\n", can.avgMotorCurrent);
  Serial.printf("FET Temperature:   %.1f °C\n", can.tempFET);
  Serial.printf("Motor Temperature: %.1f °C\n", can.tempMotor);
  Serial.printf("Watt Hours:        %ld Wh\n", can.WattHours);
  Serial.printf("Error Code:        %u\n", can.error);
  Serial.println("===============================\n");
}

void sendPedalDataToCAN() {
  buf[0] = pedalFiltered & 0xFF;        
  buf[1] = (pedalFiltered >> 8) & 0xFF;  

  byte flag = can.can_send(pedal_msg_instance.CAN_ID, 0, 2, buf);

  if (flag == 0) {
    // Print the result to the Serial Monitor
    Serial.printf("[Pedal] Sending ADC value: %u | CAN ID: %u | CAN Flag: %d\n", 
                  pedalFiltered, pedal_msg_instance.CAN_ID, flag);
  } else {
    Serial.printf("Failed to send ADC value. Error code: %d\n", flag);
  }
}

void sendVescDataToCAN() {

  byte bufMsg1[8];
  byte bufMsg2[8];

  // -------- Message 1 (ID 64) --------
  uint16_t tempMosfet   = (uint16_t)(can.tempFET * 10.0f);          // °C ×10
  int16_t inputCurrent = (int16_t)(can.avgInputCurrent * 10.0f); // A ×10
  uint16_t inputVoltage = (uint16_t)(can.inpVoltage * 10.0f);      // V ×10
  int16_t erpm         = (int16_t)(can.erpm);                    // truncated OK

  bufMsg1[0] = (tempMosfet >> 8) & 0xFF;
  bufMsg1[1] = tempMosfet & 0xFF;
  bufMsg1[2] = (inputCurrent >> 8) & 0xFF;
  bufMsg1[3] = inputCurrent & 0xFF;
  bufMsg1[4] = (inputVoltage >> 8) & 0xFF;
  bufMsg1[5] = inputVoltage & 0xFF;
  bufMsg1[6] = (erpm >> 8) & 0xFF;
  bufMsg1[7] = erpm & 0xFF;

  byte msg1 = can.can_send(vesc_msg1_instance.CAN_ID, 0, 8, bufMsg1);

  Serial.println(msg1 == CAN_OK ?
    "Telemetry MSG1 sent: tempMosfet = " + String(tempMosfet) + ", avgInputCurrent = " + String(inputCurrent) + ", inpVoltage = " + String(inputVoltage) + ", ERPM = " + String(erpm) :
    "Telemetry MSG1 FAILED");

  // -------- Message 2 (ID 65) --------
  uint8_t error        = can.error;
  uint16_t wattHours   = (uint16_t)(can.WattHours * 10.0f);
  int16_t duty         = (int16_t)(can.dutyCycleNow * 1000.0f);  // ×1000
  int16_t motorCurrent = (int16_t)(can.avgMotorCurrent * 10.0f); // A ×10

  bufMsg2[0] = error;
  bufMsg2[1] = (wattHours >> 8) & 0xFF;
  bufMsg2[2] = wattHours & 0xFF;
  bufMsg2[3] = (duty >> 8) & 0xFF;
  bufMsg2[4] = duty & 0xFF;
  bufMsg2[5] = (motorCurrent >> 8) & 0xFF;
  bufMsg2[6] = motorCurrent & 0xFF;

  byte msg2 = can.can_send(vesc_msg2_instance.CAN_ID, 0, 7, bufMsg2);

  Serial.println(msg2 == CAN_OK ?
    "Telemetry MSG2 sent: VescError = " + String(error) + ", wattHours = " + String(wattHours) + ", dutyCycleNow = " + String(duty) + ", avgMotorCurrent = " + String(motorCurrent) :
    "Telemetry MSG2 FAILED");
}

void printHelp() {
  // Ajuda resumida que pode ser chamada a qualquer momento com 'h'
  Serial.println("\n========== COMANDOS ==========");
  Serial.println("d <valor>  - Duty cycle (-1.0 a 1.0)");
  Serial.println("             Ex: d 0.5");
  Serial.println("c <valor>  - Corrente em A (desativa pedal)");
  Serial.println("             Ex: c 5.0");
  Serial.println("e <valor>  - ERPM (desativa pedal)");
  Serial.println("             Ex: e 10000");
  Serial.println("s          - Stop motor (duty=0)");
  Serial.println("p          - Toggle modo pedal ON/OFF");
  Serial.println("m <valor>  - Corrente máxima pedal (A)");
  Serial.println("             Ex: m 30.0");
  Serial.println("h          - Mostra esta ajuda");
  Serial.printf("Status: Pedal %s | Max: %.1fA\n", pedalEnabled ? "ON" : "OFF", maxPedalCurrent);
  Serial.println("==============================\n");
}

// Lê o pedal usando 12-bit ADC, aplica clamp INPUT_MIN/INPUT_MAX e reescala para 0–4095
void readPedal() {
  unsigned long now = millis();
  // Lê no maximo a 50 Hz para nao saturar o loop principal
  if (now - lastPedalRead < PEDAL_INTERVAL) return;
  lastPedalRead = now;

  // 1) ADC 12-bit (0–4095) no XIAO ESP32-C3
  int adcValue = analogRead(PIN_PEDAL);     // 0..4095 (ja configurado para 8 bits)
  if (adcValue < 0)   adcValue = 0;
  if (adcValue > 4095) adcValue = 4095;
  lastAdcRaw = (uint16_t)adcValue;

  // 2) Clamp à janela útil (INPUT_MIN..INPUT_MAX)
  if (adcValue < INPUT_MIN) {
    adcValue = INPUT_MIN;
  } else if (adcValue > INPUT_MAX) {
    adcValue = INPUT_MAX;
  }

  // 3) Reescala para 0..4095
  int rescaled = ((long)(adcValue - INPUT_MIN) * 4095) / (INPUT_MAX - INPUT_MIN);
  if (rescaled < 0)   rescaled = 0;
  if (rescaled > 4095) rescaled = 4095;
  lastRescaled = (uint16_t)rescaled;

  // 4) Filtro EMA no valor reescalado (0..4095)
  //    Mantem a leitura suave mas responsiva (ALPHA_NUM/ALPHA_DEN define suavizacao)
  pedalFiltered = (uint16_t)(
      ((uint32_t)pedalFiltered * (ALPHA_DEN - ALPHA_NUM) + (uint32_t)rescaled * ALPHA_NUM)
      / ALPHA_DEN
  );

  if (pedalEnabled) {
    // 5) Mapear 0–4095 para 0–maxPedalCurrent
    float current = (pedalFiltered / 4095.0f) * maxPedalCurrent;
    Serial.printf("[Pedal] Vai enviar corrente %.2f A para o VESC (ADC filt=%u)\n", current, pedalFiltered);
    can.vesc_set_current(current);
  }
}

void handleSerialCommands() {
  // Parser simples "linha a linha": cada linha recebida executa um comando
  while (Serial.available()) {
    char c = Serial.read();
    // Ignora carriage return (espera apenas \n para finalizar o comando)
    if (c == '\r') continue;
    if (c == '\n') {
      cmd.trim();
      if (cmd.length() > 0) {
        char type = cmd.charAt(0);
        String valueStr = cmd.substring(1);
        valueStr.trim();
        
        // Ajuda e visao geral
        if (type == 'h' || type == 'H') {
          printHelp();
        }
        // Liga/desliga modo pedal
        else if (type == 'p' || type == 'P') {
          pedalEnabled = !pedalEnabled;
          Serial.printf("[CMD] Modo pedal: %s\n", pedalEnabled ? "ON" : "OFF");
          if (!pedalEnabled) {
            can.vesc_set_duty(0.0);
            Serial.println("[CMD] Motor parado");
          }
        }
        // Ajusta corrente maxima permitida ao pedal
        else if (type == 'm' || type == 'M') {
          float maxCurrent = valueStr.toFloat();
          if (maxCurrent > 0 && maxCurrent <= 100.0) {
            maxPedalCurrent = maxCurrent;
            Serial.printf("[CMD] Corrente máxima pedal: %.1f A\n", maxPedalCurrent);
          } else {
            Serial.println("[ERRO] Corrente deve estar entre 0.1 e 100.0 A");
          }
        }
        // Forca parada imediata
        else if (type == 's' || type == 'S') {
          pedalEnabled = false;
          can.vesc_set_duty(0.0);
          Serial.println("[CMD] Motor STOP (pedal desativado)");
        }
        // Duty cycle manual (desativa o pedal enquanto estiver ativo)
        else if (type == 'd' || type == 'D') {
          pedalEnabled = false;
          float duty = valueStr.toFloat();
          if (duty >= -1.0 && duty <= 1.0) {
            can.vesc_set_duty(duty);
            Serial.printf("[CMD] Duty cycle: %.3f (pedal desativado)\n", duty);
          } else {
            Serial.println("[ERRO] Duty deve estar entre -1.0 e 1.0");
          }
        }
        // Corrente direta manual
        else if (type == 'c' || type == 'C') {
          pedalEnabled = false;
          float current = valueStr.toFloat();
          can.vesc_set_current(current);
          Serial.printf("[CMD] Corrente: %.2f A (pedal desativado)\n", current);
        }
        // ERPM manual
        else if (type == 'e' || type == 'E') {
          pedalEnabled = false;
          float erpm = valueStr.toFloat();
          can.vesc_set_erpm(erpm);
          Serial.printf("[CMD] ERPM: %.0f (pedal desativado)\n", erpm);
        }
        // Qualquer outro caractere inicial e invalido
        else {
          Serial.println("[ERRO] Comando desconhecido. Digite 'h' para ajuda.");
        }
      }
      cmd = "";
    } else {
      cmd += c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  
  // Mensagens iniciais para facilitar diagnostico (pinagem e limites)
  Serial.println("VESC Monitor & Control + Pedal - ESP32-C3 + MCP2515 @250kbps");
  Serial.println("Inicializando CAN com pinos customizados...");
  
  // Configurar ADC para pedal (12-bit, como no código do pedal CAN)
  analogReadResolution(12);   // 0..4095
  pinMode(PIN_PEDAL, INPUT);
  
  // Inicializar biblioteca com pinos ESP32-C3 e configurações
  can.initialize(PIN_CS, PIN_SCK, PIN_MISO, PIN_MOSI, CAN_250KBPS, MCP_16MHZ);
  
  Serial.println("CAN pronto. Aguardando dados do VESC...");
  Serial.printf("Pedal configurado em A0 (max corrente: %.1fA)\n", maxPedalCurrent);
  Serial.printf("ADC 12-bit, INPUT_MIN=%d, INPUT_MAX=%d\n", INPUT_MIN, INPUT_MAX);
  printHelp();
}

void loop() {
  // Processa comandos Serial (nao bloqueia; executa rapidamente se houver dados)
  handleSerialCommands();
  
  // Lê e processa pedal (com filtragem e mapeamento para corrente)
  readPedal();
  
  // Processa mensagens CAN continuamente
  can.spin();
  
  // Print dados a cada PRINT_INTERVAL ms
  unsigned long now = millis();
  if (now - lastPrint >= PRINT_INTERVAL) {
    if (pedalEnabled) {
      float pedalPercent = (pedalFiltered / 4095.0f) * 100.0f;
      float currentCmd   = (pedalFiltered / 4095.0f) * maxPedalCurrent;
      Serial.printf("[PEDAL] ADC_raw=%3u | rescaled=%3u | filt=%3u (%.1f%%) -> %.2fA\n",
                    lastAdcRaw, lastRescaled, (unsigned int)pedalFiltered,
                    pedalPercent, currentCmd);

    } else {
      Serial.println("[PEDAL] Modo pedal DESATIVADO");
    }
    printVESCData();
    lastPrint = now;
  }

  if (now - lastSendTime >= CAN_INTERVAL) {
    sendPedalDataToCAN();
    //sendVescDataToCAN();
    lastSendTime = now;
  }
}
