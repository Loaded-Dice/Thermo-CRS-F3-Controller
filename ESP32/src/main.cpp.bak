/*
state=LOOP_E flags=0x03 validMask=0x3F currentNode=0x0A timeouts=1853 crcErrors=0
10 pos=2681 raw=0x000A79 status=00 40 crc=23
08 pos=-246 raw=0xFFFF0A status=00 40 crc=42
11 pos=-174 raw=0xFFFF52 status=00 40 crc=03
09 pos=-539 raw=0xFFFDE5 status=00 40 crc=AE
12 pos=5788 raw=0x00169C status=00 40 crc=D8
0A pos=907 raw=0x00038B status=00 40 crc=C2

state=LOOP_E flags=0x07 validMask=0x3F currentNode=0x10 timeouts=1853 crcErrors=0
10 pos=2681 raw=0x000A79 status=00 40 crc=23
08 pos=-246 raw=0xFFFF0A status=00 40 crc=42
11 pos=-174 raw=0xFFFF52 status=00 40 crc=03
09 pos=-539 raw=0xFFFDE5 status=00 40 crc=AE
12 pos=5788 raw=0x00169C status=00 40 crc=D8
0A pos=907 raw=0x00038B status=00 40 crc=C2

Master Request Gripper frames
20 03 A0 C6 97 B4 66	20 A1 07 00 00 02 84	grip 20
20 03 9E C6 97 B4 58	20 A1 07 00 00 02 84	grip 21
20 03 9D C6 97 B4 5B	20 9F 07 00 00 02 BA	grip 22
20 03 9B C6 97 B4 5D	20 9F 07 00 00 02 BA	grip 23
20 03 99 C6 97 B4 5F	20 9D 07 00 00 02 B8	grip 24
20 03 97 C6 97 B4 51	20 9B 07 00 00 02 BE	grip 25
20 03 95 C6 97 B4 53	20 98 07 00 00 02 BD	grip 26
20 03 94 C6 97 B4 52	20 96 07 00 00 02 B3	grip 27
20 03 92 C6 97 B4 54	20 95 07 00 00 02 B0	grip 28
20 03 90 C6 97 B4 56	20 93 07 00 00 02 B6	grip 29
20 03 8E C6 97 B4 48	20 92 07 00 00 02 B7	grip 30

20 03 8E C6 97 B4 48	20 92 07 00 00 02 B7	grip 30
20 03 90 C6 97 B4 56	20 91 07 00 00 02 B4	grip 29
20 03 92 C6 97 B4 54	20 90 07 00 00 02 B5	grip 28
20 03 94 C6 97 B4 52	20 90 07 00 00 02 B5	grip 27
20 03 95 C6 97 B4 53	20 93 07 00 00 02 B6	grip 26
20 03 97 C6 97 B4 51	20 93 07 00 00 02 B6	grip 25

wgrip //get gripper position
20 13 00 34 0F 04 0C	|	20 6A 6C 0F 60 B0 F9 // 13 for request pos
20 13 6A 6C 0F 60 5A	|	20 6A 6C 0F 60 B0 F9 // feedback position

20 02 B0 00 F8 00 6A	|	20 0E 05 59 60 B0 A2  //Gripper fully opened
20 02 B0 01 F8 00 6B	|	20 B3 05 59 60 B0 1F  //Gripper fully closed

DIAG/CAL
10 91 0C 00 00 00 8D	10 91 07 00 40 44 82 



// ENC Reset?

08	89	0A	00	0E	00	85		08	89	00	00	00	00	81	
08	88	0A	00	0E	00	84		08	88	00	00	00	00	80	

08	89	0D	2C	01	00	A1		08	89	00	00	00	00	81	
08	88	0D	2C	01	00	A0		08	88	00	00	00	00	80	

0A	89	0D	2C	01	00	A3		0A	89	00	00	00	00	83	
0A	88	0D	2C	01	00	A2		0A	88	00	00	00	40	C2	
09	89	0A	00	0A	00	80		09	89	00	00	00	00	80	
09	88	0A	00	0A	00	81		09	88	00	00	00	00	81	


use: 
bool writeExtraFrame(const uint8_t frame[FRAME_BYTES])
and
bool readExtraResponse()

To do  : add pinout overview

use 2nd direct wire between esp & fpga
 - Esp can fully activate / deactivate FPGA


*/

#include <Arduino.h>
#include <SPI.h>
#include "fpga_spi.h"

// Power control pins
static constexpr int PIN_EN_12V_ODROID = 42;
static constexpr int PIN_EN_77V = 41; //VMOTOR
static constexpr int PIN_EN_12V_24V = 40; //VSYS
static constexpr int PIN_EN_BREAK_PWR = 39; // VBREAKS

static constexpr int PIN_FRAME_BLOCK_DONE = 14;  // IRQ from FPGA when frame block completes

// Timing constants
static constexpr uint32_t UART_BAUD = 256000;
static constexpr uint32_t POLL_INTERVAL_US = 1000;
static constexpr uint32_t FORCE_PRINT_MS = 1000;

// ====================== Master out baseline flag ======================
#define FLAG_BASELINE      0b00000100u     // = 0x04; bit 2 seems to be always active on legacy protocol

// ====================== Master Out Flags ======================
#define MASK_DRIVER        (1u << 5)       // Bit 5  → Motor Driver
#define MASK_BREAK         (1u << 1)       // Bit 1  → Break Relay

// ====================== Master Out Flags ======================
#define FLAG_DRIVER_OFF        (FLAG_BASELINE | MASK_DRIVER)     // Motor driver disabled 
#define FLAG_DRIVER_ON         FLAG_BASELINE                     // Motor driver enabled - Controls Driver Output - Diver voltage must be already supplied

#define FLAG_BREAK_ENGAGE      (FLAG_BASELINE | MASK_BREAK)      // Breaks are engaged = relay pwr off(breaks are normally closed)
#define FLAG_BREAK_DISENGAGE   FLAG_BASELINE                     // Controls switching of the relay Output - Breaks 24V must be already supplied

bool vSys = false;
//byte flags = FLAG_DRIVER_ON | FLAG_BREAK_ENGAGE;


/*
- 77V Relay off
- 24V Breaks Relay off
- 24V 12V vSys Relay on
- wait until state=LOOP_E

byte flags = 0x26 ; //== ( FLAG_DRIVER_OFF | FLAG_BREAK_ENGAGE );  --> Start condition


flags = FLAG_DRIVER_OFF | FLAG_BREAK_ENGAGE;

*/
enum FPGA_phase {IDLE, INIT_A,INIT_B,INIT_C,INIT_D,LOOP_E};
// ============================================================================
// Serial Command Parser
// ============================================================================
static void readSerial() {
    if (!Serial.available()) { return; }
    
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    // ========== Power Control Commands ==========
    if (cmd == "VMOTOR,ON" || cmd == "VMOTOR") {    digitalWrite(PIN_EN_77V, HIGH); Serial.println("VMOTOR ON (77V Motor Power)"); }
    else if (cmd == "VMOTOR,OFF") {digitalWrite(PIN_EN_77V, LOW); Serial.println("VMOTOR OFF"); }
    else if (cmd == "VSYS,ON" || cmd == "VSYS") {  digitalWrite(PIN_EN_12V_24V, HIGH); Serial.println("VSYS ON (12V/24V System Power)");  }
    else if (cmd == "VSYS,OFF") { digitalWrite(PIN_EN_12V_24V, LOW); Serial.println("VSYS OFF"); }
    else if (cmd == "VBREAKS,ON" || cmd == "VBREAKS") {digitalWrite(PIN_EN_BREAK_PWR, HIGH);  Serial.println("VBREAKS ON (24V Brake Power)");  }
    else if (cmd == "VBREAKS,OFF") { digitalWrite(PIN_EN_BREAK_PWR, LOW);Serial.println("VBREAKS OFF"); }
    else if (cmd == "ODROID,ON" || cmd == "ODROID") {        digitalWrite(PIN_EN_12V_ODROID, HIGH);        Serial.println("ODROID ON (12V)");    }
    else if (cmd == "ODROID,OFF") {        digitalWrite(PIN_EN_12V_ODROID, LOW);        Serial.println("ODROID OFF");    }
    
    // ========== Authority Commands ==========
    else if (cmd == "TAKEAUTH" || cmd == "TAKE") {
        if (takeAuthority()) { Serial.println("Authority taken - ESP controls setpoints");        } 
        else {  Serial.println("Failed to take authority"); }
    }
    else if (cmd == "RETURNAUTH" || cmd == "RETURN") {
        if (returnAuthority()) {          Serial.println("Authority returned - Robot controls setpoints");        } 
        else {            Serial.println("Failed to return authority");        }
    }
    else if (cmd == "GETAUTH" || cmd == "AUTH") {
        uint8_t state = getBuffAuthState();
        Serial.printf("Authority State: 0x%02X - ", state);
        Serial.println((state & 0x80) ? "ESP has authority" : "Robot has authority");
        Serial.printf("Buffer writeable flags: 0x%02X\n", state & 0x7F);
    }
    
    // ========== Gripper Commands ==========
    else if (cmd == "WGRIP" || cmd == "READGRIP") {
        // Read gripper position: 20 13 00 34 0F 04 0C
        uint8_t frame[7] = {0x20, 0x13, 0x00, 0x34, 0x0F, 0x04, 0x0C};
        writeExtraFrame(frame);
        delay(50);
        readExtraResponse();
    }
    else if (cmd == "GRIPOPEN" || cmd == "OPEN") {
        // Gripper fully opened: 20 02 B0 00 F8 00 6A
        uint8_t frame[7] = {0x20, 0x02, 0xB0, 0x00, 0xF8, 0x00, 0x6A};
        writeExtraFrame(frame);
        Serial.println("Gripper OPEN command sent");
        delay(50);
        readExtraResponse();
    }
    else if (cmd == "GRIPCLOSE" || cmd == "CLOSE") {
        // Gripper fully closed: 20 02 B0 01 F8 00 6B
        uint8_t frame[7] = {0x20, 0x02, 0xB0, 0x01, 0xF8, 0x00, 0x6B};
        writeExtraFrame(frame);
        Serial.println("Gripper CLOSE command sent");
        delay(50);
        readExtraResponse();
    }
    
    // ========== Status & Info Commands ==========
    else if (cmd == "STATUS" || cmd == "ST") {
        ControllerStatus status = readStatus();
        Serial.println("=== Controller Status ===");
        printStatusLine(status);
    }
    else if (cmd == "FRAMES" || cmd == "FR") {
        uint8_t frames[NODE_COUNT][FRAME_BYTES];
        readFrames(frames);
        Serial.println("=== Current Frames ===");
        printSnapshot(latestStatus, frames);
    }
    else if (cmd == "INIT") {
        if (startInitSequence()) {
            Serial.println("Init sequence started");
        } else {
            Serial.println("Init sequence rejected - use RESET first if stuck");
        }
    }
    else if (cmd == "RESET") {
        if (resetController()) {
            Serial.println("Controller reset to IDLE state");
        } else {
            Serial.println("Reset command failed");
        }
    }
    
    // ========== Custom Extra Frame ==========
    else if (cmd.startsWith("EXTRA ")) {
        // Format: EXTRA 20 03 A0 C6 97 B4 66
        uint8_t frame[7] = {0};
        int idx = 6; // Skip "EXTRA "
        int byteIdx = 0;
        
        while (idx < cmd.length() && byteIdx < 7) {
            // Skip spaces
            while (idx < cmd.length() && cmd[idx] == ' ') { idx++; }
            if (idx >= cmd.length()) { break; }
            
            // Parse hex byte
            String hexByte = "";
            while (idx < cmd.length() && cmd[idx] != ' ') {
                hexByte += cmd[idx];
                idx++;
            }
            
            if (hexByte.length() > 0) {
                frame[byteIdx++] = strtol(hexByte.c_str(), NULL, 16);
            }
        }
        
        if (byteIdx == 7) {
            writeExtraFrame(frame);
            Serial.print("Extra frame sent: ");
            for (int i = 0; i < 7; i++) {
                if (frame[i] < 0x10) Serial.print("0");
                Serial.print(frame[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
            delay(50);
            readExtraResponse();
        } else {
            Serial.println("Error: EXTRA command needs 7 hex bytes");
        }
    }
    
    // ========== Help ==========
    else if (cmd == "HELP" || cmd == "?") {
        Serial.println("\n=== Available Commands ===");
        Serial.println("Power Control:");
        Serial.println("  VMOTOR,ON / VMOTOR,OFF  - 77V Motor Power");
        Serial.println("  VSYS,ON / VSYS,OFF      - 12V/24V System Power");
        Serial.println("  VBREAKS,ON / VBREAKS,OFF - 24V Brake Power");
        Serial.println("  ODROID,ON / ODROID,OFF  - 12V Odroid Power");
        Serial.println("\nAuthority:");
        Serial.println("  TAKEAUTH / TAKE         - ESP takes control");
        Serial.println("  RETURNAUTH / RETURN     - Robot takes control");
        Serial.println("  GETAUTH / AUTH          - Show authority state");
        Serial.println("\nGripper:");
        Serial.println("  WGRIP / READGRIP        - Read gripper position");
        Serial.println("  GRIPOPEN / OPEN         - Open gripper");
        Serial.println("  GRIPCLOSE / CLOSE       - Close gripper");
        Serial.println("\nStatus:");
        Serial.println("  STATUS / ST             - Show controller status");
        Serial.println("  FRAMES / FR             - Show all frames");
        Serial.println("\nControl:");
        Serial.println("  INIT                    - Start init sequence");
        Serial.println("  RESET                   - Reset controller to IDLE");
        Serial.println("\nCustom:");
        Serial.println("  EXTRA <7 hex bytes>     - Send custom extra frame");
        Serial.println("  HELP / ?                - Show this help\n");
    }
    
    else if (cmd.length() > 0) {
        Serial.println("Unknown command. Type HELP for available commands.");
    }
}




static void pollFPGA(){
    static bool vSysLast = false;
    static uint32_t initDelayMs = 0;
    static uint32_t lastPollUs = 0;
    static uint32_t lastPrintMs = 0;
    static uint32_t lastStartRetryMs = 0;

    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastPollUs) < POLL_INTERVAL_US) { return; }
    lastPollUs = nowUs;
    const uint32_t nowMs = millis();
    vSys = digitalRead(PIN_EN_12V_24V);
    if(!vSysLast && vSys){
        initDelayMs = nowMs + 3000; 
        Serial.println(" FPGA polling started");
        Serial.println(" Power up sequence: VMOTOR -> VBREAKS -> send 'INIT' command");
    }
    vSysLast = vSys;
    if(!vSys || !vSysLast || initDelayMs == 0 || nowMs < initDelayMs) return;


    latestStatus = readStatus();

    // Auto-start disabled - user must send "INIT" command manually
    // if (!controllerRunning(latestStatus) && (uint32_t)(nowMs - lastStartRetryMs) >= 1000u) {
    //     if (startInitSequence()) { Serial.println("Init sequence started"); } 
    //     else { Serial.println("Init sequence start rejected");    }
    //     lastStartRetryMs = nowMs;
    // }



    if(latestStatus.phase != LOOP_E){
        if ((uint32_t)(nowMs - lastPrintMs) < FORCE_PRINT_MS) { return; }
        lastPrintMs = nowMs;
        printStatusLine(latestStatus);
        previousStatus = latestStatus;
    }
    else{
        readFrames(latestFrames);

        bool changed = false;// = !previousSnapshotValid || !statusEqual(latestStatus, previousStatus);
        for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT && !changed; ++nodeIndex) {
            if( !bytesEqual(latestFrames[nodeIndex], previousFrames[nodeIndex], FRAME_BYTES)){changed = true; break;}    
        }
        if (((uint32_t)(nowMs - lastPrintMs) < FORCE_PRINT_MS) || !changed) { return; }
        lastPrintMs = nowMs;
        printSnapshot(latestStatus, latestFrames);
        for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT; ++nodeIndex) {
            for (uint8_t byteIndex = 0; byteIndex < FRAME_BYTES; ++byteIndex) {
                previousFrames[nodeIndex][byteIndex] = latestFrames[nodeIndex][byteIndex];
            }
        }
    }


    //previousSnapshotValid = true;
    
    
}


void setup() {
    pinMode(PIN_EN_77V, OUTPUT);
    pinMode(PIN_EN_12V_ODROID, OUTPUT);
    pinMode(PIN_EN_12V_24V, OUTPUT);
    pinMode(PIN_EN_BREAK_PWR, OUTPUT);
    digitalWrite(PIN_EN_77V, LOW);
    digitalWrite(PIN_EN_12V_ODROID, LOW);
    digitalWrite(PIN_EN_12V_24V, LOW);
    digitalWrite(PIN_EN_BREAK_PWR, LOW);

    gpio_pulldown_en((gpio_num_t)PIN_EN_77V);
    gpio_pulldown_en((gpio_num_t)PIN_EN_12V_ODROID);
    gpio_pulldown_en((gpio_num_t)PIN_EN_12V_24V);
    gpio_pulldown_en((gpio_num_t)PIN_EN_BREAK_PWR);
    
    Serial.begin(UART_BAUD);
    // Debug - Wait for CDC & serial monitor
    delay(1000);  // Warte auf USB CDC Initialisierung
    unsigned long start = millis();
    while (!Serial && (millis() - start < 3000)) {delay(10);}
    
    Serial.println("\n\n=== ESP32-S3 FPGA Controller Starting ===");
    Serial.println("USB CDC initialized");
    Serial.println("Power control pins secured with pull-downs");
// delay(500); 
//  digitalWrite(PIN_EN_77V,HIGH);
//  delay(500); 
//   digitalWrite(PIN_EN_12V_ODROID,HIGH);
//    delay(500); 
//  digitalWrite(PIN_EN_12V_24V,HIGH);
//   delay(500); 
//  digitalWrite(PIN_EN_BREAK_PWR,HIGH);
//   delay(500); 
//    digitalWrite(PIN_EN_77V,LOW);
//    delay(500);   
//  digitalWrite(PIN_EN_12V_ODROID,LOW);
//    delay(500); 
//  digitalWrite(PIN_EN_12V_24V,LOW);
//    delay(500); 
//  digitalWrite(PIN_EN_BREAK_PWR,LOW);



     pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);
    pinMode(PIN_FRAME_BLOCK_DONE, INPUT);  // Frame block done signal from FPGA
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    Serial.println("FPGA controller poller ready");
    Serial.println("Send 'INIT' command to start initialization sequence");
    // Auto-start disabled - use serial command "INIT" to start initialization


 // Example usage of authority & position control (commented out):
    // delay(5000);  // Wait for init to complete
    // if (takeAuthority()) {
    //     Serial.println("Authority taken");
    //     int32_t newPositions[6] = {2700, -250, -180, -540, 5800, 910};
    //     for (uint8_t i = 0; i < 6; ++i) {
    //         writeNewPosition(i, newPositions[i]);
    //     }
    // }
    
    // Example usage of extra frame (commented out):
    // uint8_t extraFrame[7] = {0x20, 0x91, 0x00, 0x00, 0x00, 0x00, 0xB1};  // Reset command
    // writeExtraFrame(extraFrame);
    // delay(50);  // Wait for response
    // readExtraResponse();
}

void loop() {
    
    readSerial(); // Check for serial commands
    pollFPGA(); // poll new frames from FPGA each 1ms
}