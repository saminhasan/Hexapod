#ifndef MESSAGES_H
#define MESSAGES_H

#include <packet.h>

typedef enum
{
  MSG_HEARTBEAT = 0,
  MSG_REBOOT = 2,
  MSG_ESTOP = 4,
  MSG_ENABLE = 6,
  MSG_DISABLE = 8,
  MSG_CALIBRATE = 10,
  MSG_STAGE_POSITION = 12,
  MSG_MODE = 14,
  MSG_POSITION6D = 16,
  MSG_TRAJECTORY_LENGTH = 18,
  MSG_FEED_RATE = 20,
  MSG_TRAJECTORY_6D = 22,
  MSG_INFO = 24,
  MSG_ACK = 32
} MsgID_t;

class HeartBeat : public Packet
{
public:
  HeartBeat()
    : Packet(0)
  {
    setMessageID(MSG_HEARTBEAT);
    setCRC();
  }
};

class Reboot : public Packet
{
public:
  Reboot()
    : Packet(0)
  {
    setMessageID(MSG_REBOOT);
    setCRC();
  }
};

class eStop : public Packet
{
public:
  eStop()
    : Packet(0)
  {
    setMessageID(MSG_ESTOP);
    setCRC();
  }
};

class Enable : public Packet
{
public:
  Enable()
    : Packet(0)
  {
    setMessageID(MSG_ENABLE);
    setCRC();
  }
};

class Disable : public Packet
{
public:
  Disable(uint8_t axis)
    : Packet(0)
  {
    setMessageID(MSG_DISABLE);
    setCRC();
  }
};

class Calibrate : public Packet
{
public:
  Calibrate()
    : Packet(0)
  {
    setMessageID(MSG_CALIBRATE);
    setCRC();
  }
};

class stagePosition : public Packet
{
public:
  stagePosition()
    : Packet(0)
  {
    setMessageID(MSG_STAGE_POSITION);
    setCRC();
  }
};

class Mode : public Packet
{
public:
  Mode(uint8_t value = 0x00)
    : Packet(1)
  {
    setMessageID(MSG_MODE);
    setMessage<uint8_t>(value);
    setCRC();
  }
};


class pose6D : public Packet
{
public:
  pose6D(const float* values)
    : Packet(6 * sizeof(float))
  {
    setMessageID(MSG_POSITION6D);
    memcpy(&data()[11], values, 6 * sizeof(float));
    setCRC();
  }
};

class TrajectoryLength : public Packet
{
public:
  TrajectoryLength(uint32_t value = 0)
    : Packet(4)
  {
    setMessageID(MSG_TRAJECTORY_LENGTH);
    setMessage<uint32_t>(value);
    setCRC();
  }
};

class FeedRate : public Packet
{
public:
  FeedRate(uint8_t value = 100)
    : Packet(1)
  {
    setMessageID(MSG_FEED_RATE);
    setMessage<uint8_t>(value);
    setCRC();
  }
};

class Trajectory6D : public Packet
{
public:
  // 'values' is a pointer to numFloats float elements.
  Trajectory6D( float* values, uint32_t numFloats)
    : Packet(6 * sizeof(float) * numFloats)
  {
    setMessageID(MSG_TRAJECTORY_6D);
    memcpy(&data()[11], values, 6 * sizeof(float));
    setCRC();
  }
};

// Info: payload is a C-string with null terminator; total =  strlen(text) + 1 bytes.
class Info
{
  Packet* pkt;

public:
  Info(uint8_t sysID, const char* fmt, ...)
  {
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(nullptr, 0, fmt, args) + 1;
    va_end(args);

    char msg[len];
    va_start(args, fmt);
    vsnprintf(msg, len, fmt, args);
    va_end(args);
    pkt = new Packet(len);
    pkt->setMessageID(MSG_INFO);
    memcpy(&pkt->data()[11], msg, len);
    pkt->setCRC();
  }

  ~Info()
  {
    delete pkt;
  }

  uint8_t* data()
  {
    return pkt->data();
  }

  uint32_t length() const
  {
    return pkt->length();
  }

  void print()
  {
    pkt->print();
  }
  template<typename SerialType>
  bool send(SerialType& serialPort)
  {
    return pkt->send(serialPort);
  }
};

template<typename... Args>
void logInfosysID(uint8_t sysID, const char* fmt, Args... args)
{
  Info info(sysID, fmt, args...);
  info.send(Serial);
}

template<typename... Args>
void logInfo(const char* fmt, Args... args)
{
  logInfosysID(0, fmt, args...);
}

class ACK : public Packet
{
public:
  template<typename T>
  ACK(const T& payload)
    : Packet(sizeof(T))
  {
    setByte(9, 255);
    setMessageID(MSG_ACK);
    setMessage<T>(payload);
    setCRC();
  }
};
/// @brief make AckID private
struct __attribute__((packed)) AckHeartBeat
{
  uint8_t ackID = 1;
  uint32_t timestamp=0;
};


struct __attribute__((packed)) AckReboot
{
  uint8_t ackID = 2;
  uint32_t timestamp=0;
};

struct __attribute__((packed)) AckeStop
{
  uint8_t ackID = 3;
  uint32_t timestamp=0;
};

struct __attribute__((packed)) AckMotorConfig
{
  uint8_t ackID = 4;
  uint8_t axisID=0;
  uint8_t armed=(uint8_t)false;
  uint8_t calibrated=(uint8_t)false;
  uint8_t automatic=(uint8_t)false;
  // uint8_t cState;
  // uint8_t tState;
  // uint8_t can_id;
  // uint8_t prox_pin;
  // int dir;
  // uint32_t canmsg_count;

  // float angle1;
  // float angle2;
  // float offset;

  // float vLim;
  // float Kp;
  // float Kd;

  uint32_t timestamp=0;
};
struct __attribute__((packed)) AckMotorState
{
  uint8_t ackID = 5;
  uint8_t axisID=0;
  float positionSetpoint=0.0;
  float velocitySetpoint=0.0;
  float theta=0.0;
  float omega=0.0;
  uint32_t timestamp=0;
};

struct __attribute__((packed)) initPosition
{
  uint8_t ackID = 6;
  float theta6[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  uint32_t Idx = 0;
  uint32_t timestamp=0;
};

struct __attribute__((packed)) trajectoryLengthS
{
  uint8_t ackID = 7;
  uint32_t trajLen = 0;
  uint32_t timestamp=0;
};
struct __attribute__((packed)) feedRateS
{
  uint8_t ackID = 8;
  uint8_t feedRate = 0;
  uint32_t timestamp=0;
};
struct __attribute__((packed)) trajectory6D
{
  uint8_t ackID = 9;
  float rowFirst[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  float rowLast[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  uint32_t timestamp=0;
};
#endif  // MESSAGES_H
