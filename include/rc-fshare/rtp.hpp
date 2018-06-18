#pragma once

#include <cstdint>
#include <vector>
#include <string>

namespace rtp {

enum MessageType : uint8_t {
    CONTROL = 0,
    ROBOT_STATUS,
    PKT_ACK,
    DEBUG_REQUEST,
    DEBUG_RESPONSE,
    PING
    // FILE,
    // FILE_CHECK
};

enum DebugVar : uint8_t {
    PID_ERROR,
    MOTOR_DUTY,
    WHEEL_VEL,
    STALL_COUNTER,
    TARGET_WHEEL_VEL,
    TEST
};

constexpr uint16_t ROBOT_PAN = 0x0001;
constexpr uint16_t BASE_PAN = 0x0002;
constexpr uint16_t BROADCAST_PAN = 0xFFFF;

constexpr uint16_t BROADCAST_ADDR = 0xFFFF;

constexpr uint8_t INVALID_ROBOT_UID = 0xFF - 1;
constexpr uint8_t ANY_ROBOT_UID = 0xFF;

struct MACInfo {
    uint8_t seqNum;
    uint8_t ackRequest;
    uint8_t framePending;
    uint16_t srcAddr;
    uint16_t destPAN;
    uint16_t destAddr;
    MACInfo() : seqNum(0), ackRequest(0), framePending(0),
        srcAddr(BROADCAST_ADDR), destPAN(BROADCAST_PAN),
        destAddr(BROADCAST_ADDR) {}
} __attribute__((packed));

struct Header {
    MessageType type;
} __attribute__((packed));

struct ControlMessage {
    // uint8_t uid;
    int16_t bodyX;
    int16_t bodyY;
    int16_t bodyW;
    int8_t dribbler;
    uint8_t kickStrength;
    unsigned shootMode : 1;    // 0 = kick, 1 = chip
    unsigned triggerMode : 2;  // 0 = off, 1 = immediate, 2 = on break beam
} __attribute__((packed));

constexpr auto VELOCITY_SCALE_FACTOR = 1000;
struct RobotStatusMessage {
    // uint8_t uid;
    uint8_t battVoltage;
    unsigned motorErrors : 5;      // 0 = good, 1 = error
    unsigned ballSenseStatus : 1;  // 0 = no-ball, 1 = has-ball
    unsigned kickStatus : 1;       // 0 = uncharged, 1 = charged
    unsigned fpgaStatus : 1;       // 0 = good, 1 = error
} __attribute__((packed));

struct DebugRequestMessage {
    DebugVar debugType;
};

struct DebugResponseMessage {
    DebugVar debugType;
    float values[5];
};

// struct FileCheckMessage {
//     uint32_t chkSum;
//     uint32_t filesize;
//     char fileName[8];
//     char fileExt[3];
// } __attribute__((packed));


class SubPacket {
public:
    rtp::Header header;
    std::vector<uint8_t> payload;
    bool empty = true;

    size_t size() const {
        return sizeof(Header) + payload.size();
    }

    void clear() {
        header = {};
        payload.clear();
        empty = true;
    }

    static size_t messageSize(MessageType type) {
        switch(type) {
            case CONTROL: return sizeof(struct ControlMessage);
            case ROBOT_STATUS: return sizeof(struct RobotStatusMessage);
            case PKT_ACK: return 0;
            case DEBUG_REQUEST: return sizeof(struct DebugRequestMessage);
            case DEBUG_RESPONSE: return sizeof(struct DebugResponseMessage);
            case PING: return 0;
            default: return 0;
        }
    }
};

class Packet {
public:
    rtp::MACInfo macInfo;
    std::vector<SubPacket> subPackets;
    bool empty = true;

    size_t size() const {
        size_t size = 0;
        for (auto i : subPackets) {
            size += i.size();
        }
        return size;
    }

    void clear() {
        subPackets.clear();
        empty = true;
    }
};

template <typename PACKET_TYPE>
void serializeToVector(const PACKET_TYPE& pkt, std::vector<uint8_t>* buf) {
    const auto data = reinterpret_cast<const uint8_t*>(&pkt);
    buf->reserve(sizeof(PACKET_TYPE));
    for (size_t i = 0; i < sizeof(PACKET_TYPE); ++i) buf->push_back(data[i]);
}

}
