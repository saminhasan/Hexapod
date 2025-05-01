#ifndef MOTOR_AXIS_H
#define MOTOR_AXIS_H

#include <Bounce2.h>
#include <FlexCAN_T4.h>
#include "motctrl_prot.h"
#include <messages.h>
#include <FroCtrl.h>


template<typename CANBusType>
class MotorAxis
{
public:
  typedef enum
  {
    MOTCTRL_CMD_START_MOTOR = 0x91,
    MOTCTRL_CMD_STOP_MOTOR = 0x92,
    MOTCTRL_CMD_TORQUE_CONTROL = 0x93,
    MOTCTRL_CMD_SPEED_CONTROL = 0x94,
    MOTCTRL_CMD_POSITION_CONTROL = 0x95,
    MOTCTRL_CMD_STOP_CONTROL = 0x97,
  } MOTCTRL_CMD;

  volatile bool armed = false;
  volatile bool automatic = false;
  volatile bool newConfig = false;
  volatile bool newState = false;
  volatile float theta = 0;
  volatile float omega = 0;
  volatile float torque = 0;
  volatile int8_t temperature = 0;
  volatile uint32_t canmsg_count;

  volatile bool calibrated = false;
  static constexpr float cVelocity = 1.0;  // rad/s
  static constexpr float D90 = M_PI/2;  // rad/s

  volatile uint8_t cState = 0;
  volatile uint8_t tState = 0;
  // volatile uint8_t cStateTarget = 0;

  volatile float angle1 = 0.0;
  volatile float angle2 = 0.0;
  volatile float offset = 0.0;
  volatile bool a1 = false;
  volatile bool a2 = false;
  volatile float positionSetpoint = 0.0;
  static constexpr float vMax = 32.0;
  volatile float vLim = cVelocity;
  volatile float Kp = 5.0;
  volatile float Kd = 0.5;


  AckMotorState state;
  AckMotorConfig cfg;


  Bounce bounce = Bounce();
  FeedrateGovernor frg{0, 8.0f, cVelocity}; 

  int dir;
  uint8_t can_id;
  uint8_t prox_pin;
  uint8_t axisID;
  static MotorAxis* instance;  // Declare static instance pointer

  MotorAxis(uint8_t can_id, uint8_t prox_pin, int dir, uint8_t axisID)
    : can()
  {
    this->can_id = can_id;
    this->dir = dir;
    this->prox_pin = prox_pin;
    this->axisID = axisID;
    instance = this;  // Assign the current instance to static pointer
  }
  static void pinChangeISRWrapper()
  {
    instance->pinChangeISR();
  }
  void init()
  {
    bounce.attach(prox_pin, INPUT_PULLUP);
    bounce.interval(1);  //5
    can.begin();
    can.setBaudRate(500000);
    can.setMaxMB(16);
    can.enableFIFO();
    can.enableFIFOInterrupt();
    can.mailboxStatus();
    // Serial.println("<Motor Axis Initialized>");
  }

  void tick()
  {      
    // logInfo("TICK - %u\n", axisID);
    bounce.update();
    if (!armed) return;
    if (calibrated)
      setPosition(positionSetpoint);
    else 
      calibrate();
  }

  void update()
  {

    if(newState)
    {
      state = getAckMotorState();
      ACK ack(state);
      ack.send(Serial);
      newState = false;
    }
    if(newConfig)
    {
      cfg = getAckMotorConfig();
      ACK ack(cfg);
      ack.send(Serial);
      newConfig = false;
    }
  }


  

  void calibrate()
  {     

    switch (tState)
    {
      case 1:  // droop all
        if (!bounce.read())
          // setVelocity(cVelocity * -dir);
          frg.setTarget(cVelocity * -dir);
        else
        {
          // setVelocity(0.0);
          frg.setTarget(0.0);
          // if(abs(omega)<0.001)
          cState = 1;
        }
        break;
      case 2:  // cross edge
        if (bounce.read())
          // setVelocity(cVelocity * -dir);
          frg.setTarget(cVelocity * -dir);

        else
        {
          if(!a1)
          {
            angle1 = theta;
            a1 = true;
          }
          // setVelocity(0.0);
          frg.setTarget(0.0);
          // if(abs(omega)<0.001)
          cState = 2;
        }
        break;
      case 3:  // get back to light
        if (!bounce.read())
          // setVelocity(cVelocity * dir);
          frg.setTarget(cVelocity * dir);

        else
        {
          // setVelocity(0.0);
          frg.setTarget(0.0);
          // if(abs(omega)<0.001)
          cState = 3;
        }
        break;
      case 4:  //cross edge again
        if (bounce.read())
          // setVelocity(cVelocity * dir);
          frg.setTarget(cVelocity * dir);

        else
        {
          // angle2 = theta;
          if(!a2)
          {
            angle2 = theta;
            a2 = true;
          }
          // setVelocity(0.0);
          frg.setTarget(0.0);
          // if(abs(omega)==0.0)
          cState = 4;
        }
        break;
      case 5:  // go to zero position again.
        offset = (angle1 + angle2) / 2.0;
        positionSetpoint = motorToBodyFrame(angle2);
        logInfo("angle1:%f\n", angle1);
        logInfo("angle2:%f\n", angle2);
        logInfo("offset:%f\n", offset);
        cState = 5;
        calibrated = true;
        newConfig = true;
        break;
      default:
      frg.setTarget(0.0);
        break;
    }

    frg.tock();

setVelocity(frg.getFeedrate());
  // logInfo("Target: %.2f, Smoothed: %.2f\n", target, frg.getFeedrate());

  }




  void enable()
  {
    CAN_message_t msg;
    msg.id = can_id;
    msg.len = MOTCTRL_FRAME_SIZE;
    MCReqStartMotor(msg.buf);
    can.write(msg);
  }


  void disable()
  {
    CAN_message_t msg;
    msg.id = can_id;
    msg.len = MOTCTRL_FRAME_SIZE;
    MCReqStopMotor(msg.buf);
    can.write(msg);
  }

  void setAxisID(uint8_t AxisID)
  {
    axisID = AxisID;
    newConfig = true;
    update();
  }
  void setMode(bool automatic)
  {
    if (automatic)
    {
      automatic = true;;
      Kp = 106.5;
      vLim = vMax;
    }
    else
    {
      automatic = false;
      Kp = 5.0;
      vLim = cVelocity;
    }
    newConfig = true;
  }
  void setVelocity(float radPerSec)
  {
    radPerSec = constrain(radPerSec, -vLim, vLim);
    CAN_message_t msg;
    msg.len = MOTCTRL_FRAME_SIZE;
    msg.id = can_id;
    MCReqSpeedControl(msg.buf, radPerSec * (30.0 / M_PI), 0);
    can.write(msg);
  }


  void setPosition(float rad)
  {
    rad = constrain(rad, -D90, D90);
    // rad = (rad * dir) + (dir * M_PI / 2) + offset;
    float u = Kp * (bodyToMotorFrame(rad) - theta) - (Kd * omega);
    setVelocity(u);
  }

  void setPositionSetpoint(float pos)
  {
    positionSetpoint = pos;
  }

  void CanRxHandler(const CAN_message_t& msg)
  {
    canmsg_count++;
    float p, s, t;
    int8_t tmp;
    uint8_t buf[8];
    memcpy(buf, msg.buf, msg.len);

    if (buf[1] == MOTCTRL_RES_SUCCESS)
    {
      switch (buf[0])
      {
        case MOTCTRL_CMD_SPEED_CONTROL:
          {

            MCResSpeedControl(buf, &tmp, &p, &s, &t);
            theta = p;
            omega = s;
            torque = t;
            temperature = tmp;
            newState = true;
            break;
          }
        case MOTCTRL_CMD_START_MOTOR:
          {
            armed = true;
            newConfig = true;
            // logInfo("<ENABLED>");
            break;
          }
        case MOTCTRL_CMD_STOP_MOTOR:
          {
            armed = false;
            newConfig = true;
            // logInfo("<DISABLED>");
            break;
          }
        default:
        logInfo("<UNKNOWN_CMD>");
          break;
      }
    }
    else if (buf[1] == MOTCTRL_RES_FAIL)
    {
      logInfo("<FAIL>");  // add which command failed
    }
  }


  float motorToBodyFrame(float X_motor)
  {
    return (X_motor - offset - (dir * M_PI / 2)) / dir;
  }
  float bodyToMotorFrame(float rad)
  {
    return (rad * dir) + (dir * M_PI / 2) + offset;
  }
AckMotorConfig getAckMotorConfig()
{
  cfg.axisID = axisID;
  cfg.armed = (uint8_t)armed;
  cfg.calibrated = (uint8_t)calibrated;
  cfg.automatic = automatic;
  cfg.timestamp = micros();
  return cfg;
}
AckMotorState getAckMotorState()
{
  state.axisID = axisID;
  // state.temperature = temperature;
  state.positionSetpoint = positionSetpoint;
  state.velocitySetpoint = Kp * (bodyToMotorFrame(positionSetpoint) - theta) - Kd * omega;
  state.theta = motorToBodyFrame(theta);
  state.omega = omega;
  state.current = torque;
  state.timestamp = micros();
  return state;
}



private:
  CANBusType can;
};

template<typename CANBusType>
MotorAxis<CANBusType>* MotorAxis<CANBusType>::instance = nullptr;



#endif  // MOTOR_AXIS_H
