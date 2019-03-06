#pragma once
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

// RESET_INIT

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
