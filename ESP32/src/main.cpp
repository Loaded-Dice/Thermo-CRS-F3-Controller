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

state=LOOP_E flags=0x03 validMask=0x3F currentNode=0x10 timeouts=1853 crcErrors=0
10 pos=2681 raw=0x000A79 status=00 40 crc=23
08 pos=-246 raw=0xFFFF0A status=00 40 crc=42
11 pos=-174 raw=0xFFFF52 status=00 40 crc=03
09 pos=-539 raw=0xFFFDE5 status=00 40 crc=AE
12 pos=5788 raw=0x00169C status=00 40 crc=D8
0A pos=907 raw=0x00038B status=00 40 crc=C2

state=LOOP_E flags=0x03 validMask=0x3F currentNode=0x08 timeouts=1853 crcErrors=0
10 pos=2681 raw=0x000A79 status=00 40 crc=23
08 pos=-246 raw=0xFFFF0A status=00 40 crc=42
11 pos=-174 raw=0xFFFF52 status=00 40 crc=03
09 pos=-539 raw=0xFFFDE5 status=00 40 crc=AE
12 pos=5788 raw=0x00169C status=00 40 crc=D8
0A pos=907 raw=0x00038B status=00 40 crc=C2

state=LOOP_E flags=0x03 validMask=0x3F currentNode=0x11 timeouts=1853 crcErrors=0
10 pos=2681 raw=0x000A79 status=00 40 crc=23
08 pos=-246 raw=0xFFFF0A status=00 40 crc=42
11 pos=-174 raw=0xFFFF52 status=00 40 crc=03
09 pos=-539 raw=0xFFFDE5 status=00 40 crc=AE
12 pos=5788 raw=0x00169C status=00 40 crc=D8
0A pos=907 raw=0x00038B status=00 40 crc=C2

state=LOOP_E flags=0x03 validMask=0x3F currentNode=0x10 timeouts=1853 crcErrors=0
10 pos=2681 raw=0x000A79 status=00 40 crc=23
08 pos=-246 raw=0xFFFF0A status=00 40 crc=42
11 pos=-174 raw=0xFFFF52 status=00 40 crc=03
09 pos=-539 raw=0xFFFDE5 status=00 40 crc=AE
12 pos=5788 raw=0x00169C status=00 40 crc=D8
0A pos=907 raw=0x00038B status=00 40 crc=C2
*/



#include <Arduino.h>
#include <SPI.h>

static constexpr int PIN_SPI_CS   = 10;
static constexpr int PIN_SPI_SCLK = 12;
static constexpr int PIN_SPI_MOSI = 11;
static constexpr int PIN_SPI_MISO = 13;

static constexpr uint8_t CMD_READ_STATUS = 0xB0;
static constexpr uint8_t CMD_READ_FRAMES = 0xB1;
static constexpr uint8_t CMD_START_INIT  = 0xB2;

static constexpr uint8_t NODE_COUNT = 6;
static constexpr uint8_t FRAME_BYTES = 7;
static constexpr uint8_t STATUS_BYTES = 8;

static constexpr uint8_t NODE_IDS[NODE_COUNT] = {0x10, 0x08, 0x11, 0x09, 0x12, 0x0A};

static constexpr uint32_t UART_BAUD = 256000;
static constexpr uint32_t SPI_HZ = 1000000;
static constexpr uint32_t POLL_INTERVAL_US = 1000;
static constexpr uint32_t FORCE_PRINT_MS = 1000;

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

static inline void spiBeginTx() {
    SPI.beginTransaction(spiSettings);
    digitalWrite(PIN_SPI_CS, LOW);
}

static inline void spiEndTx() {
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
}

static void printHexByte(uint8_t value) {
    if (value < 0x10) {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

inline int32_t reinterpret_24bit_signed(uint32_t value) {
    return (int32_t)((value << 8) | (value & 0x800000u ? 0xFF000000u : 0u)) >> 8;
}

static void spiReadBlock(uint8_t command, uint8_t* buffer, size_t length) {
    spiBeginTx();
    SPI.transfer(command);
    for (size_t index = 0; index < length; ++index) {
        buffer[index] = SPI.transfer(0x00);
    }
    spiEndTx();
}

static bool startInitSequence() {
    spiBeginTx();
    SPI.transfer(CMD_START_INIT);
    const uint8_t ack = SPI.transfer(0x00);
    spiEndTx();
    return ack == 0xA5;
}

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
    for (size_t index = 0; index < length; ++index) {
        if (left[index] != right[index]) {
            return false;
        }
    }
    return true;
}

static bool statusEqual(const ControllerStatus& left, const ControllerStatus& right) {
    return left.flags == right.flags &&
           left.phase == right.phase &&
           left.validMask == right.validMask &&
           left.currentNodeId == right.currentNodeId &&
           left.timeoutCount == right.timeoutCount &&
           left.crcErrorCount == right.crcErrorCount;
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
    Serial.print("state=");
    Serial.print(phaseName(status.phase));
    Serial.print(" flags=0x");
    printHexByte(status.flags);
    Serial.print(" validMask=0x");
    printHexByte(status.validMask);
    Serial.print(" currentNode=0x");
    printHexByte(status.currentNodeId);
    Serial.print(" timeouts=");
    Serial.print(status.timeoutCount);
    Serial.print(" crcErrors=");
    Serial.println(status.crcErrorCount);
}

static void printFrameSummary(uint8_t nodeIndex, const uint8_t frame[FRAME_BYTES], bool valid) {
    printHexByte(NODE_IDS[nodeIndex]);
    if (!valid) {
        Serial.println(" pos=------ status=-- -- crc=--");
        return;
    }

    const uint32_t rawPositionLE = static_cast<uint32_t>(frame[1]) |
                                   (static_cast<uint32_t>(frame[2]) << 8) |
                                   (static_cast<uint32_t>(frame[3]) << 16);
    const int32_t signedPosition = reinterpret_24bit_signed(rawPositionLE);

    Serial.print(" pos=");
    Serial.print(signedPosition);
    Serial.print(" raw=0x");
    if (rawPositionLE < 0x10) Serial.print("00000");
    else if (rawPositionLE < 0x100) Serial.print("0000");
    else if (rawPositionLE < 0x1000) Serial.print("000");
    else if (rawPositionLE < 0x10000) Serial.print("00");
    else if (rawPositionLE < 0x100000) Serial.print('0');
    Serial.print(rawPositionLE, HEX);
    Serial.print(" status=");
    printHexByte(frame[4]);
    Serial.print(' ');
    printHexByte(frame[5]);
    Serial.print(" crc=");
    printHexByte(frame[6]);
    Serial.println();
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
    SPI.begin(PIN_SPI_SCLK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    Serial.println("FPGA controller poller ready");
    if (startInitSequence()) {
        Serial.println("Init sequence started");
    } else {
        Serial.println("Init sequence start rejected");
    }
}

void loop() {
    static uint32_t lastPollUs = 0;
    static uint32_t lastPrintMs = 0;
    static uint32_t lastStartRetryMs = 0;

    const uint32_t nowUs = micros();
    if ((uint32_t)(nowUs - lastPollUs) < POLL_INTERVAL_US) {
        return;
    }
    lastPollUs = nowUs;

    latestStatus = readStatus();

    const uint32_t nowMs = millis();
    if (!controllerRunning(latestStatus) && (uint32_t)(nowMs - lastStartRetryMs) >= 1000u) {
        if (startInitSequence()) {
            Serial.println("Init sequence retriggered");
        }
        lastStartRetryMs = nowMs;
    }

    readFrames(latestFrames);

    bool changed = !previousSnapshotValid || !statusEqual(latestStatus, previousStatus);
    for (uint8_t nodeIndex = 0; nodeIndex < NODE_COUNT && !changed; ++nodeIndex) {
        changed = !bytesEqual(latestFrames[nodeIndex], previousFrames[nodeIndex], FRAME_BYTES);
    }

    if (!changed && (uint32_t)(nowMs - lastPrintMs) < FORCE_PRINT_MS) {
        return;
    }

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