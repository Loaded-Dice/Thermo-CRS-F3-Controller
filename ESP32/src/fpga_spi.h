#ifndef FPGA_SPI_H
#define FPGA_SPI_H

#include <Arduino.h>
#include <SPI.h>

// ============================================================================
// Pin Definitions
// ============================================================================
static constexpr int PIN_SPI_CS   = 10;
static constexpr int PIN_SPI_SCLK = 12;
static constexpr int PIN_SPI_MOSI = 11;
static constexpr int PIN_SPI_MISO = 13;

// ============================================================================
// SPI Commands
// ============================================================================
static constexpr uint8_t CMD_READ_STATUS      = 0xB0;
static constexpr uint8_t CMD_READ_FRAMES      = 0xB1;
static constexpr uint8_t CMD_START_INIT       = 0xB2;
static constexpr uint8_t CMD_RESET_CTRL       = 0xB3;
static constexpr uint8_t CMD_GET_AUTH_STATE   = 0xB4;
static constexpr uint8_t CMD_TAKE_AUTHORITY   = 0xB5;
static constexpr uint8_t CMD_RETURN_AUTHORITY = 0xB6;
static constexpr uint8_t CMD_WRITE_POSITION   = 0xB7;
static constexpr uint8_t CMD_WRITE_EXTRA      = 0xB8;
static constexpr uint8_t CMD_READ_EXTRA       = 0xB9;

// ============================================================================
// Protocol Constants
// ============================================================================
static constexpr uint8_t NODE_COUNT   = 6;
static constexpr uint8_t FRAME_BYTES  = 7;
static constexpr uint8_t STATUS_BYTES = 8;

static constexpr uint8_t NODE_IDS[NODE_COUNT] = {0x10, 0x08, 0x11, 0x09, 0x12, 0x0A};

static constexpr uint32_t SPI_HZ = 4000000;

// ============================================================================
// Data Structures
// ============================================================================
struct ControllerStatus {
    uint8_t flags;
    uint8_t phase;
    uint8_t validMask;
    uint8_t currentNodeId;
    uint16_t timeoutCount;
    uint16_t crcErrorCount;
};
struct FrameIn {
    uint8_t rawBytes[FRAME_BYTES];
    uint8_t nodeID;
    int32_t position;
    uint8_t status[2];
};
// ============================================================================
// Global Variables
// ============================================================================
extern uint8_t latestFrames[NODE_COUNT][FRAME_BYTES];
extern uint8_t previousFrames[NODE_COUNT][FRAME_BYTES];
extern ControllerStatus latestStatus;
extern ControllerStatus previousStatus;
extern bool previousSnapshotValid;
extern SPISettings spiSettings;

// ============================================================================
// Function Declarations
// ============================================================================

// Initialization
bool startInitSequence();
bool resetController();

// Authority Control
uint8_t getBuffAuthState();
bool takeAuthority();
bool returnAuthority();

// Position Control
bool writeNodePosition(uint8_t nodeIndex, const uint8_t frame[7]);
void createPositionFrame(uint8_t nodeId, int32_t position, uint8_t flags, uint8_t frame[7]);

// Extra Frame (Gripper/Reset/Custom)
bool writeExtraFrame(const uint8_t frame[FRAME_BYTES]);
bool readExtraResponse();

// Status & Frame Reading
ControllerStatus readStatus();
void readFrames(uint8_t frames[NODE_COUNT][FRAME_BYTES]);

// Utility Functions
bool bytesEqual(const uint8_t* left, const uint8_t* right, size_t length);
bool statusEqual(const ControllerStatus& left, const ControllerStatus& right);
bool controllerRunning(const ControllerStatus& status);

// Output Functions
void printSnapshot(const ControllerStatus& status, uint8_t frames[NODE_COUNT][FRAME_BYTES]);
void printStatusLine(const ControllerStatus& status);

// Helper Functions
uint8_t calculate_frame_crc(const uint8_t frame[7]);
int32_t reinterpret_24bit_signed(uint32_t value);

#endif // FPGA_SPI_H
