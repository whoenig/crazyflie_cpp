#pragma once
#include <cstdint>

// Header
struct bootloader
{
  constexpr bootloader(
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

// GET_INFO
struct bootloaderGetInfoRequest
{
  bootloaderGetInfoRequest(
    uint8_t target)
    : header(target, 0x10)
  {
  }

  bootloader header;
} __attribute__((packed));

struct bootloaderGetInfoResponse
{
  bootloaderGetInfoRequest request;
  uint16_t pageSize;
  uint16_t nBuffPage;
  uint16_t nFlashPage;
  uint16_t flashStart;
  uint8_t reserved[12];
  uint8_t version;
} __attribute__((packed));

// GET_MAPPING

// LOAD_BUFFER

struct bootloaderLoadBufferRequest
{
  bootloaderLoadBufferRequest(
    uint8_t target,
    uint16_t page,
    uint16_t address)
    : header(target, 0x14)
    , page(page)
    , address(address)
  {
  }

  bootloader header;
  uint16_t page;
  uint16_t address;
  uint8_t data[25];
} __attribute__((packed));

// READ_BUFFER

struct bootloaderReadBufferRequest
{
  bootloaderReadBufferRequest(
    uint8_t target,
    uint16_t page,
    uint16_t address)
    : header(target, 0x15)
    , page(page)
    , address(address)
  {
  }

  bootloader header;
  uint16_t page;
  uint16_t address;
} __attribute__((packed));

struct bootloaderReadBufferResponse
{
  bootloaderReadBufferRequest request;
  uint8_t data[25];
} __attribute__((packed));

// WRITE_FLASH

struct bootloaderWriteFlashRequest
{
  bootloaderWriteFlashRequest(
    uint8_t target,
    uint16_t bufferPage,
    uint16_t flashPage,
    uint16_t nPages)
    : header(target, 0x18)
    , bufferPage(bufferPage)
    , flashPage(flashPage)
    , nPages(nPages)
  {
  }

  bootloader header;
  uint16_t bufferPage;
  uint16_t flashPage;
  uint16_t nPages;
} __attribute__((packed));

struct bootloaderWriteFlashResponse
{
  bootloader header;
  uint8_t done;
  uint8_t error;
} __attribute__((packed));

// FLASH_STATUS

struct bootloaderFlashStatusRequest
{
  bootloaderFlashStatusRequest(
    uint8_t target)
    : header(target, 0x19)
  {
  }

  bootloader header;
} __attribute__((packed));

struct bootloaderFlashStatusResponse
{
  bootloaderFlashStatusRequest request;
  uint8_t done;
  uint8_t error;
} __attribute__((packed));

// READ_FLASH

struct bootloaderReadFlashRequest
{
  bootloaderReadFlashRequest(
    uint8_t target,
    uint16_t page,
    uint16_t address)
    : header(target, 0x1C)
    , page(page)
    , address(address)
  {
  }

  bootloader header;
  uint16_t page;
  uint16_t address;
} __attribute__((packed));

struct bootloaderReadFlashResponse
{
  bootloaderReadFlashRequest request;
  uint8_t data[25];
} __attribute__((packed));

// RESET

struct bootloaderResetRequest
{
  bootloaderResetRequest(
    uint8_t bootToFirmware)
    : header(0xFE, 0xF0)
    , bootToFirmware(bootToFirmware)
  {
  }

  bootloader header;
  uint8_t bootToFirmware; //0=boot to bootloader; otherwise: boot to firmware
} __attribute__((packed));
