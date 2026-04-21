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

static constexpr int PIN_SPI_CS   = 10;
static constexpr int PIN_SPI_SCLK = 12;
static constexpr int PIN_SPI_MOSI = 11;
static constexpr int PIN_SPI_MISO = 13;
static constexpr int PIN_FRAME_BLOCK_DONE = 14;  // IRQ from FPGA when frame block completes

static constexpr uint8_t CMD_READ_STATUS = 0xB0;
static constexpr uint8_t CMD_READ_FRAMES = 0xB1;
static constexpr uint8_t CMD_START_INIT  = 0xB2;
static constexpr uint8_t CMD_GET_AUTH_STATE = 0xB4;
static constexpr uint8_t CMD_TAKE_AUTHORITY = 0xB5;
static constexpr uint8_t CMD_RETURN_AUTHORITY = 0xB6;
static constexpr uint8_t CMD_WRITE_POSITION = 0xB7;
static constexpr uint8_t CMD_WRITE_EXTRA = 0xB8;
static constexpr uint8_t CMD_READ_EXTRA = 0xB9;

static constexpr uint8_t NODE_COUNT = 6;
static constexpr uint8_t FRAME_BYTES = 7;
static constexpr uint8_t STATUS_BYTES = 8;

static constexpr uint8_t NODE_IDS[NODE_COUNT] = {0x10, 0x08, 0x11, 0x09, 0x12, 0x0A};

static constexpr uint32_t UART_BAUD = 256000;
static constexpr uint32_t SPI_HZ = 4000000;
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


//byte flags = FLAG_DRIVER_ON | FLAG_BREAK_ENGAGE;

struct ControllerStatus {
    uint8_t flags;
    uint8_t phase;
    uint8_t validMask;
    uint8_t currentNodeId;
    uint16_t timeoutCount;
    uint16_t crcErrorCount;
};

static uint8_t latestFrames[NODE_COUNT][FRAME_BYTES]{};
static uint8_t previousFrames[NODE_COUNT][FRAME_BYTES]{};
static ControllerStatus latestStatus{};
static ControllerStatus previousStatus{};
static bool previousSnapshotValid = false;

SPISettings spiSettings(SPI_HZ, MSBFIRST, SPI_MODE0);

static inline void spiBeginTx() { SPI.beginTransaction(spiSettings); digitalWrite(PIN_SPI_CS, LOW); }

static inline void spiEndTx() { digitalWrite(PIN_SPI_CS, HIGH); SPI.endTransaction(); }

static void printHexByte(uint8_t value) { if (value < 0x10) { Serial.print('0'); } Serial.print(value, HEX); }

inline int32_t reinterpret_24bit_signed(uint32_t value) {  return (int32_t)((value << 8) | (value & 0x800000u ? 0xFF000000u : 0u)) >> 8; }

// Convert int32_t position to 3-byte little-endian representation
static void position_to_3bytes(int32_t position, uint8_t& byte1, uint8_t& byte2, uint8_t& byte3) {
    uint32_t unsigned_val = static_cast<uint32_t>(position) & 0x00FFFFFFu;
    byte1 = static_cast<uint8_t>(unsigned_val & 0xFFu);
    byte2 = static_cast<uint8_t>((unsigned_val >> 8) & 0xFFu);
    byte3 = static_cast<uint8_t>((unsigned_val >> 16) & 0xFFu);
}

// Calculate CRC for a 7-byte frame
static uint8_t calculate_frame_crc(const uint8_t frame[7]) { return frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]; }


static void spiReadBlock(uint8_t command, uint8_t* buffer, size_t length) {
    spiBeginTx();
    SPI.transfer(command);
    for (size_t index = 0; index < length; ++index) { buffer[index] = SPI.transfer(0x00); }
    spiEndTx();
}

static bool startInitSequence() {
    spiBeginTx();
    SPI.transfer(CMD_START_INIT);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

// Get buffer and authority state
// Returns: bit7=authority(0=Robot,1=ESP), bit6-0=writeable flags per node
static uint8_t getBuffAuthState() {
    spiBeginTx();
    SPI.transfer(CMD_GET_AUTH_STATE);
    const uint8_t state = SPI.transfer(0x00);
    spiEndTx();
    return state;
}

// Take authority from robot (ESP controls setpoints)
static bool takeAuthority() {
    spiBeginTx();
    SPI.transfer(CMD_TAKE_AUTHORITY);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

// Return authority to robot (Robot controls setpoints)
static bool returnAuthority() {
    spiBeginTx();
    SPI.transfer(CMD_RETURN_AUTHORITY);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

// Write new position for a specific node (0-5)
// frame[7] = {NodeID, CMD, Pos1, Pos2, Pos3, Flags, CRC}
static bool writeNodePosition(uint8_t nodeIndex, const uint8_t frame[7]) {
    if (nodeIndex > 5) { return false; }
    spiBeginTx();
    SPI.transfer(CMD_WRITE_POSITION);
    SPI.transfer(nodeIndex);
    for (uint8_t i = 0; i < 7; ++i) { SPI.transfer(frame[i]); }
    spiEndTx();
    return true;
}

// Helper: Create position frame from position value and flags
static void createPositionFrame(uint8_t nodeId, int32_t position, uint8_t flags, uint8_t frame[7]) {
    frame[0] = nodeId;
    frame[1] = 0x31;  // Position command
    position_to_3bytes(position, frame[2], frame[3], frame[4]);
    frame[5] = flags;
    frame[6] = calculate_frame_crc(frame);
}

// Write extra frame (NodeID 0x20 for Gripper/Reset/Custom commands)
// frame[7] = {NodeID, CMD, Data1, Data2, Data3, Flags, CRC}
static bool writeExtraFrame(const uint8_t frame[FRAME_BYTES]) {
    spiBeginTx();
    SPI.transfer(CMD_WRITE_EXTRA);
    for (uint8_t i = 0; i < FRAME_BYTES; ++i) {
        SPI.transfer(frame[i]);
    }
    spiEndTx();
    return true;
}

// Read extra frame response (7 bytes)
// Returns true if response is available, prints raw bytes as HEX
static bool readExtraResponse() {
    uint8_t response[FRAME_BYTES]{};
    spiReadBlock(CMD_READ_EXTRA, response, sizeof(response));
    
    // Print raw bytes to Serial
    Serial.print("Extra Response: ");
    for (uint8_t i = 0; i < FRAME_BYTES; ++i) {
        printHexByte(response[i]);
        Serial.print(' ');
    }
    Serial.println();
    
    return true;
}

// Write new position for a node (for now commented out - not called)
// static void writeNewPosition(uint8_t nodeIndex, int32_t position) {
//     uint8_t frame[7];
//     createPositionFrame(NODE_IDS[nodeIndex], position, 0x04, frame);  // 0x04 = motors armed
//     writeNodePosition(nodeIndex, frame);
// }

static ControllerStatus readStatus() {
    uint8_t raw[STATUS_BYTES]{};
    spiReadBlock(CMD_READ_STATUS, raw, sizeof(raw));

    ControllerStatus status{};
    status.flags = raw[0];
    status.phase = raw[1];
    status.validMask = raw[2];
    status.currentNodeId = raw[3];
    status.timeoutCount = static_cast<uint16_t>(raw[5] << 8) | raw[4];
    status.crcErrorCount = static_cast<uint16_t>(raw[7] << 8) | raw[6];
    return status;
}

static void readFrames(uint8_t frames[NODE_COUNT][FRAME_BYTES]) {
    uint8_t raw[NODE_COUNT * FRAME_BYTES]{};
    spiReadBlock(CMD_READ_FRAMES, raw, sizeof(raw));

    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT; ++nodeIndex) {
        for (uint8_t byteIndex = 0; byteIndex < FRAME_BYTES; ++byteIndex) {
            frames[nodeIndex][byteIndex] = raw[nodeIndex * FRAME_BYTES + byteIndex];
        }
    }
}

static bool bytesEqual(const uint8_t* left, const uint8_t* right, size_t length) {
    for (size_t index = 0; index < length; ++index) { if (left[index] != right[index]) { return false; } }  return true;
}

static bool statusEqual(const ControllerStatus& left, const ControllerStatus& right) {
    return left.flags == right.flags && left.phase == right.phase && left.validMask == right.validMask &&
           left.currentNodeId == right.currentNodeId && left.timeoutCount == right.timeoutCount && left.crcErrorCount == right.crcErrorCount;
}

static const char* phaseName(uint8_t phase) {
    switch (phase) {
        case 0: return "IDLE";
        case 1: return "INIT_A";
        case 2: return "INIT_B";
        case 3: return "INIT_C";
        case 4: return "INIT_D";
        case 5: return "LOOP_E";
        default: return "UNKNOWN";
    }
}

static void printStatusLine(const ControllerStatus& status) {
    Serial.printf("state= %s  flags=0x%02X  validMask=0x%02X  currentNode=0x%02X  timeouts= %i crcErrors= %i \n",
        phaseName(status.phase),status.flags,status.validMask, status.currentNodeId, status.timeoutCount, status.crcErrorCount);
}

static void printFrameSummary(uint8_t nodeIndex, const uint8_t frame[FRAME_BYTES], bool valid) {
    //printHexByte(NODE_IDS[nodeIndex]);
    if (!valid) { Serial.printf(" 0x%02X pos=------ status=-- -- crc=--\n",NODE_IDS[nodeIndex]); return; } 
    const int32_t signedPosition = reinterpret_24bit_signed(static_cast<uint32_t>(frame[1]) | (static_cast<uint32_t>(frame[2]) << 8) | (static_cast<uint32_t>(frame[3]) << 16));
    
    Serial.printf(" 0x%02X  pos= %+d raw=0x%02X%02X%02X  status=0x%02X %02X crc=0x%02X \n",NODE_IDS[nodeIndex], signedPosition, frame[1],frame[2],frame[3],frame[4],frame[5],frame[6]);
    // const uint32_t rawPositionLE = static_cast<uint32_t>(frame[1]) | (static_cast<uint32_t>(frame[2]) << 8) | (static_cast<uint32_t>(frame[3]) << 16);
    // Serial.printf(" 0x%02X  pos= %+d raw=0x%06X  status=0x%02X %02X crc=0x%02X \n",NODE_IDS[nodeIndex], signedPosition, rawPositionLE,frame[4],frame[5],frame[6]);
    //Serial.println(" pos=------ status=-- -- crc=--");
    // const uint32_t rawPositionLE = static_cast<uint32_t>(frame[1]) | (static_cast<uint32_t>(frame[2]) << 8) | (static_cast<uint32_t>(frame[3]) << 16);
    // Serial.print(" pos=");
    // Serial.print(signedPosition);
    // Serial.print(" raw=0x");
    // if (rawPositionLE < 0x10) Serial.print("00000");
    // else if (rawPositionLE < 0x100) Serial.print("0000");
    // else if (rawPositionLE < 0x1000) Serial.print("000");
    // else if (rawPositionLE < 0x10000) Serial.print("00");
    // else if (rawPositionLE < 0x100000) Serial.print('0');
    
    // Serial.print(rawPositionLE, HEX);
    // Serial.print(" status="); printHexByte(frame[4]);
    // Serial.print(' '); printHexByte(frame[5]);
    // Serial.print(" crc="); printHexByte(frame[6]);
    // Serial.println();
}

static void printSnapshot(const ControllerStatus& status, uint8_t frames[NODE_COUNT][FRAME_BYTES]) {
    printStatusLine(status);
    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT; ++nodeIndex) {
        const bool valid = ((status.validMask >> nodeIndex) & 0x01u) != 0;
        printFrameSummary(nodeIndex, frames[nodeIndex], valid);
    }
    Serial.println();
}

static bool controllerRunning(const ControllerStatus& status) {
    return (status.flags & 0x01u) != 0;
}

void setup() {
    Serial.begin(UART_BAUD);
    delay(200);

    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);
    pinMode(PIN_FRAME_BLOCK_DONE, INPUT);  // Frame block done signal from FPGA
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    Serial.println("FPGA controller poller ready");
    if (startInitSequence()) {
        Serial.println("Init sequence started");
    } else {
        Serial.println("Init sequence start rejected");
    }

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
    static uint32_t lastPollUs = 0;
    static uint32_t lastPrintMs = 0;
    static uint32_t lastStartRetryMs = 0;

    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastPollUs) < POLL_INTERVAL_US) { return; }
    lastPollUs = nowUs;

    latestStatus = readStatus();

    const uint32_t nowMs = millis();
    if (!controllerRunning(latestStatus) && (uint32_t)(nowMs - lastStartRetryMs) >= 1000u) {
        if (startInitSequence()) { Serial.println("Init sequence retriggered"); }
        lastStartRetryMs = nowMs;
    }

    readFrames(latestFrames);

    bool changed = !previousSnapshotValid || !statusEqual(latestStatus, previousStatus);
    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT && !changed; ++nodeIndex) {
        changed = !bytesEqual(latestFrames[nodeIndex], previousFrames[nodeIndex], FRAME_BYTES);
    }

    if (!changed && (uint32_t)(nowMs - lastPrintMs) < FORCE_PRINT_MS) { return; }

    printSnapshot(latestStatus, latestFrames);
    previousStatus = latestStatus;
    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT; ++nodeIndex) {
        for (uint8_t byteIndex = 0; byteIndex < FRAME_BYTES; ++byteIndex) {
            previousFrames[nodeIndex][byteIndex] = latestFrames[nodeIndex][byteIndex];
        }
    }
    previousSnapshotValid = true;
    lastPrintMs = nowMs;
}