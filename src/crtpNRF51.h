#pragma once

#include <crazyflieLinkCpp/Packet.hpp>

#if 0
#include <cstdint>

// Header
struct crtpNrf51Header
{
  constexpr crtpNrf51Header(
    uint8_t target,
    uint8_t cmd)
    : header(0xFF)
    , target(target)
    , cmd(cmd)
  {
  }

  uint8_t header;
  uint8_t target;
  uint8_t cmd;
} __attribute__((packed));
#endif
// RESET_INIT

class crtpNrf51ResetInitRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNrf51ResetInitRequest()
      : Packet(0xF, 0x3, 2)
  {
    setPayloadAt<uint8_t>(0, 0xFE); // target
    setPayloadAt<uint8_t>(1, 0xFF); // cmd
  }
};

struct crtpNrf51ResetInitResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 0xF &&
           p.channel() == 0x3 &&
           p.payloadSize() == 8 &&
           p.payloadAt<uint8_t>(0) == 0xFE &&
           p.payloadAt<uint8_t>(1) == 0xFF;
  }

  static uint64_t address(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint64_t>(2);
  }
};

// RESET

class crtpNrf51ResetRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNrf51ResetRequest(uint8_t bootToFirmware)
      : Packet(0xF, 0x3, 3)
  {
    setPayloadAt<uint8_t>(0, 0xFE); // target
    setPayloadAt<uint8_t>(1, 0xF0); // cmd
    setPayloadAt<uint8_t>(2, bootToFirmware); //0=boot to bootloader; otherwise: boot to firmware
  }
};

/* no response sent */

// ALLOFF

class crtpNrf51AllOffRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNrf51AllOffRequest()
      : Packet(0xF, 0x3, 2)
  {
    setPayloadAt<uint8_t>(0, 0xFE); // target
    setPayloadAt<uint8_t>(1, 0x01); // cmd
  }
};

/* no response sent */

// SYSOFF

class crtpNrf51SysOffRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNrf51SysOffRequest()
      : Packet(0xF, 0x3, 2)
  {
    setPayloadAt<uint8_t>(0, 0xFE);           // target
    setPayloadAt<uint8_t>(1, 0x02);           // cmd
  }
};

/* no response sent */

// SYSON

class crtpNrf51SysOnRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNrf51SysOnRequest()
      : Packet(0xF, 0x3, 2)
  {
    setPayloadAt<uint8_t>(0, 0xFE); // target
    setPayloadAt<uint8_t>(1, 0x03); // cmd
  }
};

/* no response sent */

// GETVBAT

class crtpNrf51GetVBatRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNrf51GetVBatRequest()
      : Packet(0xF, 0x3, 2)
  {
    setPayloadAt<uint8_t>(0, 0xFE); // target
    setPayloadAt<uint8_t>(1, 0x04); // cmd
  }
};

struct crtpNrf51GetVBatResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 0xF &&
           p.channel() == 0x3 &&
           p.payloadSize() == 6 &&
           p.payloadAt<uint8_t>(0) == 0xFE &&
           p.payloadAt<uint8_t>(1) == 0x04;
  }

  static float vbat(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<float>(2);
  }
};

#if 0
struct crtpNrf51ResetInitRequest
{
  crtpNrf51ResetInitRequest()
    : header(0xFE, 0xFF)
  {
  }

  crtpNrf51Header header;
} __attribute__((packed));

struct crtpNrf51ResetInitResponse
{
  crtpNrf51ResetInitRequest request;
  uint8_t addr[6];
} __attribute__((packed));

// RESET

struct crtpNrf51ResetRequest
{
  crtpNrf51ResetRequest(
    uint8_t bootToFirmware)
    : header(0xFE, 0xF0)
    , bootToFirmware(bootToFirmware)
  {
  }

  crtpNrf51Header header;
  uint8_t bootToFirmware; //0=boot to bootloader; otherwise: boot to firmware
} __attribute__((packed));

/* no response sent */

// ALLOFF

struct crtpNrf51AllOffRequest
{
  crtpNrf51AllOffRequest()
    : header(0xFE, 0x01)
  {
  }

  crtpNrf51Header header;
} __attribute__((packed));

/* no response sent */

// SYSOFF

struct crtpNrf51SysOffRequest
{
  crtpNrf51SysOffRequest()
    : header(0xFE, 0x02)
  {
  }

  crtpNrf51Header header;
} __attribute__((packed));

/* no response sent */

// SYSON

struct crtpNrf51SysOnRequest
{
  crtpNrf51SysOnRequest()
    : header(0xFE, 0x03)
  {
  }

  crtpNrf51Header header;
} __attribute__((packed));

/* no response sent */

// GETVBAT

struct crtpNrf51GetVBatRequest
{
  crtpNrf51GetVBatRequest()
    : header(0xFE, 0x04)
  {
  }

  crtpNrf51Header header;
} __attribute__((packed));

struct crtpNrf51GetVBatResponse
{
  crtpNrf51GetVBatRequest request;
  float vbat;
} __attribute__((packed));

//////////////////////////////////////////////////////////////////////

struct crtpNrf51SetSafelinkRequest
{

  crtpNrf51SetSafelinkRequest(
    uint8_t hasSafelink)
    : hasSafelink(hasSafelink)
  {
  }

  const uint8_t header1 = 0xFF;
  const uint8_t header2 = 0x05;
  uint8_t hasSafelink;
} __attribute__((packed));
#endif