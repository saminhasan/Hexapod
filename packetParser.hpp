#ifndef PACKET_PARSER_HPP
#define PACKET_PARSER_HPP
#include <common.h>
#include <messages.h>





template<typename SerialPortType>
class PacketParser {
public:
    SerialPortType& port;
    uint8_t* buffer;
    uint32_t bufferSize;
    uint32_t bufferIndex;
    void (*processPacketCallback)(const uint8_t* packet, uint32_t packetLength);

    // Use initializer list to properly initialize the reference and other members.
    PacketParser(SerialPortType& p, uint8_t* buf, uint32_t bufSize)
        : port(p), buffer(buf), bufferSize(bufSize), bufferIndex(0), processPacketCallback(nullptr)
    {
    }

    // The main parsing function, called at the end of every loop, not calling at the start not to break initialization states.
    void parse() {
        uint32_t bytesToRead = port.available();
        if (bytesToRead == 0)
            return;

        uint32_t availableSpace = bufferSize - bufferIndex;
        if (bytesToRead > availableSpace) // >=?
            bytesToRead = availableSpace;
        // was there supposed to be an else here?
        // have to take a decision here, either drop everything and send a error message in both cases.
        uint32_t bytesRead = port.readBytes(reinterpret_cast<char*>(buffer + bufferIndex), bytesToRead);
        bufferIndex += bytesRead;

        uint32_t pos = 0;
        while (bufferIndex - pos >= 16) {
            if (buffer[pos] != 0x01) {
                pos++;
                continue;
            }
            uint32_t packetLength = ((uint32_t)buffer[pos + 1]) |
                                    ((uint32_t)buffer[pos + 2] << 8) |
                                    ((uint32_t)buffer[pos + 3] << 16) |
                                    ((uint32_t)buffer[pos + 4] << 24);
            if (packetLength < 16 || packetLength > bufferSize) {
                pos++;
                continue;
            }
            if (bufferIndex - pos < packetLength)
                break; // Incomplete packet.
            if (buffer[pos + packetLength - 1] != 0x04) {
                pos++;
                continue;
            }
            uint32_t crcReported = ((uint32_t)buffer[pos + packetLength - 5]) |
                                   ((uint32_t)buffer[pos + packetLength - 4] << 8) |
                                   ((uint32_t)buffer[pos + packetLength - 3] << 16) |
                                   ((uint32_t)buffer[pos + packetLength - 2] << 24);
            uint32_t crcComputed = calcCRC32(buffer + pos + 1, packetLength - 6);
            if (crcReported != crcComputed) {
                pos++;
                logInfo("CRC Mismatch!!!");
                continue;
            }
            // Process the packet using the callback if available.
            if (processPacketCallback) {
                processPacketCallback(buffer + pos, packetLength);
            }
            else {
                // Empty function; override this in a derived class if you prefer.
                processPacket(buffer + pos, packetLength);
            }
            pos += packetLength;
        }

        // Shift unprocessed data to the beginning.
        if (pos > 0) {
            memmove(buffer, buffer + pos, bufferIndex - pos);
            bufferIndex -= pos;
        }
        if (bufferIndex >= bufferSize) // Should not happen as sender knws full protocol.
            bufferIndex = 0;
    }

    // Default processPacket function. Override this in a subclass
    void processPacket(const uint8_t* packet, uint32_t packetLength) {
        // Empty by default.
    }
};

#endif // PACKET_PARSER_HPP



/*

#ifndef PACKET_PARSER_HPP
#define PACKET_PARSER_HPP

#include <cstdint>
#include <array>
#include <functional>
#include <common.h>
#include <messages.h>

// Provide your own CRC‑32
extern uint32_t calcCRC32(const uint8_t* data, size_t len);

template<typename SerialPortType,
         size_t MAX_PACKET_SIZE = 4096    // adjust to your worst‑case
>
class PacketParser {
public:
    using PacketCallback = std::function<void(const uint8_t* packet, uint32_t len)>;

    PacketParser(SerialPortType& port)
      : port_(port), callback_(nullptr)
    {}

    void setCallback(PacketCallback cb) {
        callback_ = std::move(cb);
    }

    // call this once per loop
    void parse() {
        while (port_.available() > 0) {
            uint8_t b;
            if (port_.readBytes(reinterpret_cast<char*>(&b), 1) == 0)
                break;
            feed(b);
        }
    }

private:
    // protocol constants
    static constexpr uint8_t  SYNC_BYTE     = 0x01;
    static constexpr uint8_t  FOOTER_BYTE   = 0x04;
    static constexpr size_t   LEN_SZ        = 4;
    static constexpr size_t   CRC_SZ        = 4;
    static constexpr size_t   MIN_PKT_SZ    = 1 + LEN_SZ + CRC_SZ + 1; // sync+len+crc+footer

    enum class State { Sync, ReadLen, ReadPayload, ReadCRC, ReadFooter };
    State                state_    = State::Sync;
    uint32_t             idx_      = 0;
    uint32_t             totalLen_ = 0;
    std::array<uint8_t, MAX_PACKET_SIZE> buf_;
    SerialPortType&      port_;
    PacketCallback       callback_;

    static uint32_t decodeLE32(const uint8_t* p) {
        return uint32_t(p[0])
             | (uint32_t(p[1]) << 8)
             | (uint32_t(p[2]) << 16)
             | (uint32_t(p[3]) << 24);
    }

    void reset() {
        state_ = State::Sync;
        idx_   = 0;
    }

    void feed(uint8_t b) {
        switch (state_) {
          case State::Sync:
            if (b == SYNC_BYTE) {
                buf_[0] = b;
                idx_    = 1;
                state_  = State::ReadLen;
            }
            break;

          case State::ReadLen:
            buf_[idx_++] = b;
            if (idx_ == 1 + LEN_SZ) {
                totalLen_ = decodeLE32(buf_.data() + 1);
                if (totalLen_ < MIN_PKT_SZ || totalLen_ > MAX_PACKET_SIZE) {
                    reset();
                } else {
                    state_ = State::ReadPayload;
                }
            }
            break;

          case State::ReadPayload:
            buf_[idx_++] = b;
            if (idx_ == totalLen_ - (CRC_SZ + 1)) {
                state_ = State::ReadCRC;
            }
            break;

          case State::ReadCRC:
            buf_[idx_++] = b;
            if (idx_ == totalLen_ - 1) {
                state_ = State::ReadFooter;
            }
            break;

          case State::ReadFooter:
            buf_[idx_++] = b;
            if (b == FOOTER_BYTE) {
                // verify CRC
                size_t crcStart = totalLen_ - 1 - CRC_SZ;
                uint32_t reported = decodeLE32(buf_.data() + crcStart);
                uint32_t computed = calcCRC32(buf_.data() + 1, crcStart - 1);
                if (reported == computed && callback_) {
                    callback_(buf_.data(), totalLen_);
                }
            }
            reset();
            break;
        }
    }
};

#endif // PACKET_PARSER_HPP

*/
