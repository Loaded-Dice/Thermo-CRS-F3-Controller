
#include "fpga_spi.h"

// ============================================================================
// Global Variables Definition
// ============================================================================
uint8_t latestFrames[NODE_COUNT][FRAME_BYTES]{};
uint8_t previousFrames[NODE_COUNT][FRAME_BYTES]{};
ControllerStatus latestStatus{};
ControllerStatus previousStatus{};
bool previousSnapshotValid = false;
SPISettings spiSettings(SPI_HZ, MSBFIRST, SPI_MODE0);

// ============================================================================
// Internal Helper Functions
// ============================================================================
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
uint8_t calculate_frame_crc(const uint8_t frame[7]) { return frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]; }


static void spiReadBlock(uint8_t command, uint8_t* buffer, size_t length) {
    spiBeginTx();
    SPI.transfer(command);
    for (size_t index = 0; index < length; ++index) { buffer[index] = SPI.transfer(0x00); }
    spiEndTx();
}

bool startInitSequence() {
    spiBeginTx();
    SPI.transfer(CMD_START_INIT);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

bool resetController() {
    spiBeginTx();
    SPI.transfer(CMD_RESET_CTRL);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

// Get buffer and authority state
// Returns: bit7=authority(0=Robot,1=ESP), bit6-0=writeable flags per node
uint8_t getBuffAuthState() {
    spiBeginTx();
    SPI.transfer(CMD_GET_AUTH_STATE);
    const uint8_t state = SPI.transfer(0x00);
    spiEndTx();
    return state;
}

// Take authority from robot (ESP controls setpoints)
bool takeAuthority() {
    spiBeginTx();
    SPI.transfer(CMD_TAKE_AUTHORITY);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

// Return authority to robot (Robot controls setpoints)
bool returnAuthority() {
    spiBeginTx();
    SPI.transfer(CMD_RETURN_AUTHORITY);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

// Write new position for a specific node (0-5)
// frame[7] = {NodeID, CMD, Pos1, Pos2, Pos3, Flags, CRC}
bool writeNodePosition(uint8_t nodeIndex, const uint8_t frame[7]) {
    if (nodeIndex > 5) { return false; }
    spiBeginTx();
    SPI.transfer(CMD_WRITE_POSITION);
    SPI.transfer(nodeIndex);
    for (uint8_t i = 0; i < 7; ++i) { SPI.transfer(frame[i]); }
    spiEndTx();
    return true;
}

// Helper: Create position frame from position value and flags
void createPositionFrame(uint8_t nodeId, int32_t position, uint8_t flags, uint8_t frame[7]) {
    frame[0] = nodeId;
    frame[1] = 0x31;  // Position command
    position_to_3bytes(position, frame[2], frame[3], frame[4]);
    frame[5] = flags;
    frame[6] = calculate_frame_crc(frame);
}

// Write extra frame (NodeID 0x20 for Gripper/Reset/Custom commands)
// frame[7] = {NodeID, CMD, Data1, Data2, Data3, Flags, CRC}
bool writeExtraFrame(const uint8_t frame[FRAME_BYTES]) {
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
bool readExtraResponse() {
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

ControllerStatus readStatus() {
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

void readFrames(uint8_t frames[NODE_COUNT][FRAME_BYTES]) {
    uint8_t raw[NODE_COUNT * FRAME_BYTES]{};
    spiReadBlock(CMD_READ_FRAMES, raw, sizeof(raw));

    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT; ++nodeIndex) {
        for (uint8_t byteIndex = 0; byteIndex < FRAME_BYTES; ++byteIndex) {
            frames[nodeIndex][byteIndex] = raw[nodeIndex * FRAME_BYTES + byteIndex];
        }
    }
}

bool bytesEqual(const uint8_t* left, const uint8_t* right, size_t length) {
    for (size_t index = 0; index < length; ++index) { if (left[index] != right[index]) { return false; } }  return true;
}

bool statusEqual(const ControllerStatus& left, const ControllerStatus& right) {
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

void printStatusLine(const ControllerStatus& status) {
    Serial.printf("state= %s  flags=0x%02X  validMask=0x%02X  currentNode=0x%02X  timeouts= %i crcErrors= %i \n",
        phaseName(status.phase),status.flags,status.validMask, status.currentNodeId, status.timeoutCount, status.crcErrorCount);
}

static void printFrameSummary(uint8_t nodeIndex, const uint8_t frame[FRAME_BYTES], bool valid) {

    if (!valid) { Serial.printf(" 0x%02X pos=------ status=-- -- crc=--\n",NODE_IDS[nodeIndex]); return; } 
    const int32_t signedPosition = reinterpret_24bit_signed(static_cast<uint32_t>(frame[1]) | (static_cast<uint32_t>(frame[2]) << 8) | (static_cast<uint32_t>(frame[3]) << 16));
    Serial.printf(" 0x%02X  pos= %+d raw=0x%02X%02X%02X  status=0x%02X %02X crc=0x%02X \n",NODE_IDS[nodeIndex], signedPosition, frame[1],frame[2],frame[3],frame[4],frame[5],frame[6]);

}

void printSnapshot(const ControllerStatus& status, uint8_t frames[NODE_COUNT][FRAME_BYTES]) {
    printStatusLine(status);
    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT; ++nodeIndex) {
        const bool valid = ((status.validMask >> nodeIndex) & 0x01u) != 0;
        printFrameSummary(nodeIndex, frames[nodeIndex], valid);
    }
    Serial.println();
}

bool controllerRunning(const ControllerStatus& status) { return (status.flags & 0x01u) != 0; }
