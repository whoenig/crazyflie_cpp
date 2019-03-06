#pragma once

#include <stdint.h>
#include <fstream>

class ITransport
{
public:
  struct Ack
  {
    Ack()
      : ack(0)
      , size(0)
    {}

    uint8_t ack:1;
    uint8_t powerDet:1;
    uint8_t retry:4;
    uint8_t data[32];

    uint8_t size;
  }__attribute__((packed));

public:
  ITransport()
    : m_enableLogging(false)
  {
  }

  virtual ~ITransport() {}

  virtual void sendPacket(
    const uint8_t* data,
    uint32_t length,
    Ack& result) = 0;

  virtual void sendPacketNoAck(
    const uint8_t* data,
    uint32_t length) = 0;


  void enableLogging(
    bool enable);

protected:
  void logPacket(
    const uint8_t* data,
    uint32_t length);

  void logAck(
    const Ack& ack);

protected:
  bool m_enableLogging;
  std::ofstream m_file;
};
