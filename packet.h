#ifndef PACKET_H
#define PACKET_H
#include <common.h>

static constexpr uint32_t PACKET_OVERHEAD = 16;           // Total overhead = 16 bytes (header, CRC, and end byte)
static constexpr uint32_t START_OFFSET   = 0;            // 1 byte: 0x01
static constexpr uint32_t LENGTH_OFFSET  = 1;            // 4 bytes: packet length
static constexpr uint32_t SEQ_OFFSET     = 5;            // 4 bytes: sequence number
static constexpr uint32_t SYSID_OFFSET   = 9;            // 1 byte: sysID
static constexpr uint32_t MSGID_OFFSET   = 10;           // 1 byte: message ID
static constexpr uint32_t PAYLOAD_OFFSET = 11;           // message payload starts here

class Packet
{
  uint32_t packetLength;
  uint32_t messageLength;
  uint8_t* rawBytes;
  static uint32_t globalSequenceNumber;  // auto-incremented sequence

public:
  static uint8_t sysID;

  // msgLen: payload length (sysID (1) + msgID (1) +  data)
  Packet(uint32_t msgLen)
  {
    messageLength = msgLen;
    packetLength = messageLength + PACKET_OVERHEAD;
    rawBytes = new uint8_t[packetLength];
    rawBytes[START_OFFSET] = 0x01;  // start byte
    // Write packetLength into bytes 1-4.
    memcpy(&rawBytes[LENGTH_OFFSET], &packetLength, sizeof(uint32_t));
    // Auto-assign the sequence number into bytes 5-8.
    memcpy(&rawBytes[SEQ_OFFSET], &globalSequenceNumber, sizeof(uint32_t));
    rawBytes[SYSID_OFFSET] = sysID;
    globalSequenceNumber++;  // increment for next packet
    // The end byte is set at the end.
    rawBytes[packetLength - 1] = 0x04;
  }

  ~Packet()
  {
    delete[] rawBytes;
  }

  void setByte(uint32_t index, uint8_t value)
  {
    if (index < packetLength)
      rawBytes[index] = value;
  }
  void setMessageID(uint8_t msgID)
  {
    rawBytes[MSGID_OFFSET] = msgID;
  }

  template<typename T>
  void setMessage(const T& msg)
  {
    if (sizeof(T) > messageLength) return;  // could assert or clamp
    memcpy(&rawBytes[PAYLOAD_OFFSET], &msg, sizeof(T));
  }


  void setCRC() {
    uint32_t crc = calcCRC32(&rawBytes[1], packetLength - 6);  // exclude start & end bytes
    memcpy(&rawBytes[packetLength - 5], &crc, sizeof(crc));
  }
  uint8_t* data()
  {
    return rawBytes;
  }

  uint32_t length() const
  {
    return packetLength;
  }

  // Prints the raw bytes in one line in "0x.. 0x.." format.
  void print()
  {
    for (uint32_t i = 0; i < packetLength; i++)
    {
      Serial.print("0x");
      if (rawBytes[i] < 0x10)
        Serial.print("0");
      Serial.print(rawBytes[i], HEX);
      if (i < packetLength - 1)
        Serial.print(" ");
    }
    Serial.println();
  }
  template<typename SerialType>
  bool send(SerialType& serialPort)
  {
    if (!serialPort) return false;
    // if (!(serialPort.availableForWrite() >= packetLength)) return false;
    size_t written = serialPort.write(rawBytes, packetLength);
    return written == packetLength;
  }
};

uint8_t Packet::sysID = 0;
uint32_t Packet::globalSequenceNumber = 1;

#endif  // PACKET_H
