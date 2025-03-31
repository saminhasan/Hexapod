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

    // The main parsing function.
    void parse() {
        uint32_t bytesToRead = port.available();
        if (bytesToRead < 1)
            return;

        uint32_t availableSpace = bufferSize - bufferIndex;
        if (bytesToRead > availableSpace)
            bytesToRead = availableSpace;

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
        if (bufferIndex >= bufferSize)
            bufferIndex = 0;
    }

    // Default processPacket function. Override this in a subclass if you want.
    void processPacket(const uint8_t* packet, uint32_t packetLength) {
        // Empty by default.
    }
};

#endif // PACKET_PARSER_HPP
