#include <canIds.h>
#include <mcp_can.h>
#include <SPI.h>
#include "vesc_can_bus_arduino.h"
#include "can_diagnostics.h"

/** Initiate VescUart class */
//ComEVesc UART;

//HardwareSerial VescSerial(1); //ESP32 ligado por Serial(USB) ao jetson, e por SPI ao MCP2515.

/* IMPORTANT: Let the library define this. 
   Manually defining it to 16 can cause conflicts. */
// #define CAN_500KBPS 16

/* CS_PIN for ESP-32 */ //(define the SPI pins for the MCP2515)
static const int PIN_SCK   = D8; //int proque é o enderece numerico no hardware
static const int PIN_MISO  = D9;
static const int PIN_MOSI  = D10;
static const int PIN_CS    = D7;

//MCP_CAN CAN(SPI_CS_PIN);  
//definições d ecorrente- parâmetros do motor
#define BRAKE_CURRENT 50 //define: sempre que escrever breakcurrent, é substituido por 50 antes do compilador ver. corrente máxima de travagem
#define CURRENT 50
#define FREECURRENT 0.7

CAN can; //instanciação da classe can
//timestamp. 
unsigned long lastSend = millis();
const unsigned long sendCANInterval = 15;

// Global variables for logic
short int currentAngle = 0; // Renamed from oldAngle/newAngle for clarity
//payload buffers
unsigned char dirBuf[8] = {0};
unsigned char currentBuf[8] = {0};

bool zeroSent = false;

// Heartbeat — sinaliza ao bus que este nó está vivo.
// PROBLEMA: O PCSender só transmite ao bus quando chegam comandos pelo Serial.
// Se o PC não envia nada, o PCSender parece morto do ponto de vista dos outros nós CAN.
unsigned long lastHeartbeat = 0;
const unsigned long heartbeatInterval = 1000;   // 1 Hz
uint8_t heartbeatCounter = 0;

// Monitorização dos contadores de erro do MCP2515 (TEC/REC).
// Imprime a 1 Hz quando qualquer contador >=96 ou EFLG != 0.
CanDiagnosticsState canDiag(1000);

//uma vez após boot/reset
void setup() {
    Serial.begin(9600);  // inicializa a porta-serie com baud rate de 9600

    //delay enquanto serial nao estiver pronto 
    while(!Serial){
      delay(10);
    }

    pinMode(PIN_CS, OUTPUT);

    can.initialize(PIN_CS, PIN_SCK, PIN_MISO, PIN_MOSI, CAN_250KBPS, MCP_16MHZ);


    // FIX 2: Set mode to NORMAL to allow sending

    lastHeartbeat = millis(); //inicializa o timer c o time inicial.

    Serial.println("[PCSender]  CAN BUS OK!");
    Serial.println("[PCSender]  Heartbeat enabled @ 1 Hz");
    Serial.println("---------- PCSender.INO ----------");
}

void loop() {
    unsigned long currentTime = millis();
    
    // -------------------------------------------------
    // FIX 3: Unified Serial Parsing
    // We read ONCE. If we split reading between loop() and 
    // a helper function, messages get lost/eaten.
    // -------------------------------------------------
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim(); // Remove whitespace/newlines

        if (input.startsWith("Dir:")) {
            // Handle Steering Angle
            String numberPart = input.substring(4); // Skip "Dir:"
            int val = numberPart.toInt();

            // Constrain
            if (val > 250) val = 250;
            if (val < -250) val = -250;
            
            currentAngle = (short int)val;
            
            // Immediate feedback helps debugging
             Serial.print("Updated Angle: ");
             Serial.println(currentAngle);
             sendDirCan(currentAngle * 5);
            
        } else if (input.startsWith("L2:")) {
            // Handle BRAKE CURRENT
            float transformed_brake = extractBrakeCurrent(input);

            Serial.print("[PC Sender] Brake Current: ");
            if (transformed_brake > BRAKE_CURRENT){
                transformed_brake = BRAKE_CURRENT;
            }
            Serial.println(transformed_brake);
            
            can.vesc_set_brake_current(transformed_brake);
            
        } else if (input.startsWith("R2:")) {
            // Handle ACCEL CURRENT
            float transformed_current = extractCurrent(input);

            Serial.print("[PC Sender] Current: ");

            //if (transformed_current > CURRENT){
            //    transformed_current = CURRENT;
            //}
            Serial.println(transformed_current);

            // ID 1002 for Throttle
            if(transformed_current==0 && zeroSent==false){
                can.vesc_set_erpm(0);
                zeroSent = true;    
            }else{
                can.vesc_set_erpm(transformed_current*4);
                zeroSent = false;
            }
        } 
        // else: Unknown command, ignore safely
    }

    // -------------------------------------------------
    // Periodic CAN Sending (Steering)
    // -------------------------------------------------
    // We send the angle every 'sendCANInterval' OR if it changed?
    // Your original logic sent every 15ms.
    if (currentTime - lastSend >= sendCANInterval) {
        lastSend = currentTime;

        // Send the current known angle
        // Serial.print("Sending Angle: ");
        // Serial.println(currentAngle);
        //sendDirCan(currentAngle * 5);
    }

    // -------------------------------------------------
    // Heartbeat — anuncia "estou vivo" ao bus 1×/seg
    // -------------------------------------------------
    if (currentTime - lastHeartbeat >= heartbeatInterval) {
        lastHeartbeat = currentTime;

        unsigned char hbBuf[1] = { heartbeatCounter };
        can.can_send(pcsender_heartbeat_msg_instance.CAN_ID, 0, 1, hbBuf);
        heartbeatCounter++;   // wraps 255 -> 0 (uint8_t)
    }

    // -------------------------------------------------
    // Monitorização CAN a 1 Hz — TEC/REC + EFLG do MCP2515.
    // Só imprime quando há problema (TEC/REC >=96 ou EFLG !=0), para
    // não poluir o Serial que o Jetson lê. EFLG !=0 apanha também
    // RX overflows, que não fazem TEC/REC subir.
    // -------------------------------------------------
    checkCanDiagnostics(can, canDiag, "PCSender", currentTime);
}


/************************************************/
/*************** HELPER FUNCTIONS ***************/
/************************************************/

// Removed UARTComAngle because it caused data loss.
// Logic moved inside loop().

float extractBrakeCurrent(String input) {
    // "L2:" is 3 chars
    String numericPart = input.substring(3);
    int l2_value = numericPart.toInt();
    return (BRAKE_CURRENT * (float)l2_value) / 255.0f;
}

float extractCurrent(String input) {
    // "R2:" is 3 chars
    String numericPart = input.substring(3);
    int r2_value = numericPart.toInt();
    return r2_value;
    //return (CURRENT * (float)r2_value) / 255.0f;
}

byte sendDirCan(short int value) {
    dirBuf[0] = (value >> 8) & 0xFF;
    dirBuf[1] = (value) & 0xFF;
    
    // Ensure dir_msg_instance is defined in canIds.h
    //byte flag = CAN.sendMsgBuf(dir_msg_instance.CAN_ID, 0, 8, dirBuf);
    byte flag = can.can_send(dir_msg_instance.CAN_ID, 0, 8, dirBuf);
    Serial.println(flag);
    return flag;
}
