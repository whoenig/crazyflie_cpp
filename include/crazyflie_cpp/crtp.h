#pragma once

#include "Crazyradio.h"
#include <cstdint>

static int const CRTP_MAX_DATA_SIZE = 30;
static int const CRTP_MAXSIZE = 31;
#define CHECKSIZE(s) static_assert(sizeof(s) <= CRTP_MAXSIZE, #s " packet is too large");
#define CHECKSIZE_WITH_STATE(s, stateSize) static_assert(sizeof(s) - stateSize <= CRTP_MAXSIZE, #s " packet is too large");

static int const CRTP_MAXSIZE_RESPONSE = 32;
#define CHECKSIZE_RESPONSE(s) static_assert(sizeof(s) <= CRTP_MAXSIZE_RESPONSE, #s " packet is too large");

void quatdecompress(uint32_t comp, float q[4]);

// Header
struct crtp
{
  constexpr crtp(uint8_t port, uint8_t channel)
    : channel(channel)
    , link(3)
    , port(port)
  {
  }

  crtp(uint8_t byte)
  {
    channel = (byte >> 0) & 0x3;
    link    = (byte >> 2) & 0x3;
    port    = (byte >> 4) & 0xF;
  }

  bool operator==(const crtp& other) const {
    return channel == other.channel && port == other.port;
  }

  uint8_t channel:2;
  uint8_t link:2;
  uint8_t port:4;
} __attribute__((packed));

// Packet structure definition
typedef struct {
  uint8_t size;
  union {
    struct {
      uint8_t header;
      uint8_t data[CRTP_MAX_DATA_SIZE];
    };
    uint8_t raw[CRTP_MAX_DATA_SIZE+1];
  };
} crtpPacket_t;

struct crtpEmpty
{
  const uint8_t cmd = 0xFF;
};

// Port 0 (Console)
struct crtpConsoleResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return crtp(response.data[0]) == crtp(0, 0);
    }

    crtp header;
    char text[31];
};
CHECKSIZE_RESPONSE(crtpConsoleResponse)

// Port 2 (Parameters)

enum ParamType : uint8_t 
{
  ParamTypeUint8  = 0x00 | (0x00<<2) | (0x01<<3),
  ParamTypeInt8   = 0x00 | (0x00<<2) | (0x00<<3),
  ParamTypeUint16 = 0x01 | (0x00<<2) | (0x01<<3),
  ParamTypeInt16  = 0x01 | (0x00<<2) | (0x00<<3),
  ParamTypeUint32 = 0x02 | (0x00<<2) | (0x01<<3),
  ParamTypeInt32  = 0x02 | (0x00<<2) | (0x00<<3),
  ParamTypeFloat  = 0x02 | (0x01<<2) | (0x00<<3),
};

struct crtpParamTocGetItemResponse;
struct crtpParamTocGetItemRequest
{
  crtpParamTocGetItemRequest(
    uint8_t id)
    : header(2, 0)
    , command(0)
    , id(id)
  {
  }

  bool operator==(const crtpParamTocGetItemRequest& other) const {
    return header == other.header && command == other.command && id == other.id;
  }

  typedef crtpParamTocGetItemResponse Response;

  const crtp header;
  const uint8_t command;
  uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpParamTocGetItemRequest)

struct crtpParamTocGetItemResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 5 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 0;
  }

  crtpParamTocGetItemRequest request;
  uint8_t length:2; // one of ParamLength
  uint8_t type:1;   // one of ParamType
  uint8_t sign:1;   // one of ParamSign
  uint8_t res0:2;   // reserved
  uint8_t readonly:1;
  uint8_t group:1;  // one of ParamGroup
  char text[28]; // group, name
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamTocGetItemResponse)

struct crtpParamTocGetInfoResponse;
struct crtpParamTocGetInfoRequest
{
  crtpParamTocGetInfoRequest()
    : header(2, 0)
    , command(1)
  {
  }

  bool operator==(const crtpParamTocGetInfoRequest& other) const {
    return header == other.header && command == other.command;
  }

  typedef crtpParamTocGetInfoResponse Response;

  const crtp header;
  const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpParamTocGetInfoRequest)

struct crtpParamTocGetInfoResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 7 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 1;
  }

  crtpParamTocGetInfoRequest request;
  uint8_t numParam;
  uint32_t crc;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamTocGetInfoResponse)

struct crtpParamValueResponse;
struct crtpParamReadRequest
{
  crtpParamReadRequest(
    uint8_t id)
    : header(2, 1)
    , id(id)
  {
  }

  bool operator==(const crtpParamReadRequest& other) const {
    return header == other.header && id == other.id;
  }

  typedef crtpParamValueResponse Response;

  const crtp header;
  const uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpParamReadRequest)

template <class T>
struct crtpParamWriteRequest
{
  crtpParamWriteRequest(
    uint8_t id,
    const T& value)
    : header(2, 2)
    , id(id)
    , value(value)
    {
    }

    const crtp header;
    const uint8_t id;
    const T value;
} __attribute__((packed));
CHECKSIZE(crtpParamWriteRequest<double>) // largest kind of param

struct crtpParamValueResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 2 &&
           (crtp(response.data[0]) == crtp(2, 1) ||
            crtp(response.data[0]) == crtp(2, 2));
  }

  crtpParamReadRequest request;
  union {
    uint8_t valueUint8;
    int8_t valueInt8;
    uint16_t valueUint16;
    int16_t valueInt16;
    uint32_t valueUint32;
    int32_t valueInt32;
    float valueFloat;
  };
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamValueResponse)

// V2
struct crtpParamTocGetItemV2Response;
struct crtpParamTocGetItemV2Request
{
  crtpParamTocGetItemV2Request(
    uint16_t id)
    : header(2, 0)
    , command(2)
    , id(id)
  {
  }

  bool operator==(const crtpParamTocGetItemV2Request& other) const {
    return header == other.header && command == other.command && id == other.id;
  }

  typedef crtpParamTocGetItemResponse Response;

  const crtp header;
  const uint8_t command;
  uint16_t id;
} __attribute__((packed));
CHECKSIZE(crtpParamTocGetItemV2Request)

struct crtpParamTocGetItemV2Response
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 5 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 2;
  }

  crtpParamTocGetItemV2Request request;
  uint8_t length:2; // one of ParamLength
  uint8_t type:1;   // one of ParamType
  uint8_t sign:1;   // one of ParamSign
  uint8_t res0:2;   // reserved
  uint8_t readonly:1;
  uint8_t group:1;  // one of ParamGroup
  char text[27]; // group, name
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamTocGetItemV2Response)

struct crtpParamTocGetInfoV2Response;
struct crtpParamTocGetInfoV2Request
{
  crtpParamTocGetInfoV2Request()
    : header(2, 0)
    , command(3)
  {
  }

  bool operator==(const crtpParamTocGetInfoV2Request& other) const {
    return header == other.header && command == other.command;
  }

  typedef crtpParamTocGetInfoV2Response Response;

  const crtp header;
  const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpParamTocGetInfoV2Request)

struct crtpParamTocGetInfoV2Response
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 8 &&
           crtp(response.data[0]) == crtp(2, 0) &&
           response.data[1] == 3;
  }

  crtpParamTocGetInfoV2Request request;
  uint16_t numParam;
  uint32_t crc;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamTocGetInfoV2Response)

struct crtpParamValueV2Response;
struct crtpParamReadV2Request
{
  crtpParamReadV2Request(
    uint16_t id)
    : header(2, 1)
    , id(id)
  {
  }

  bool operator==(const crtpParamReadV2Request& other) const {
    return header == other.header && id == other.id;
  }

  typedef crtpParamValueV2Response Response;

  const crtp header;
  const uint16_t id;
} __attribute__((packed));
CHECKSIZE(crtpParamReadV2Request)

template <class T>
struct crtpParamWriteV2Request
{
  crtpParamWriteV2Request(
    uint16_t id,
    const T& value)
    : header(2, 2)
    , id(id)
    , value(value)
    {
    }

    const crtp header;
    const uint16_t id;
    const T value;
} __attribute__((packed));
CHECKSIZE(crtpParamWriteV2Request<float>) // largest kind of param

struct crtpParamValueV2Response
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 2 &&
           (crtp(response.data[0]) == crtp(2, 1) ||
            crtp(response.data[0]) == crtp(2, 2));
  }

  crtpParamReadV2Request request;
  uint8_t status; // 0 = success
  union {
    uint8_t valueUint8;
    int8_t valueInt8;
    uint16_t valueUint16;
    int16_t valueInt16;
    uint32_t valueUint32;
    int32_t valueInt32;
    float valueFloat;
  };
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamValueV2Response)

template <class T>
struct crtpParamSetByNameRequest
{
  crtpParamSetByNameRequest(
    const char* group,
    const char* name,
    const T& value);

    const crtp header;
    const uint8_t cmd = 0;
    uint8_t data[29];

  uint8_t size() const {
    return size_;
  }

  uint8_t responseSize() const {
    return responseSize_;
  }

private:
    // member state (not part of packet)
    uint8_t size_;
    uint8_t responseSize_;

private:
  crtpParamSetByNameRequest(
    const char* group,
    const char* name,
    uint8_t paramType,
    const void* value,
    uint8_t valueSize);
} __attribute__((packed));
CHECKSIZE_WITH_STATE(crtpParamSetByNameRequest<float>, 2) // largest kind of param

struct crtpParamSetByNameResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size > 2 &&
           (crtp(response.data[0]) == crtp(2, 3));
  }

  uint8_t data[32];

  uint8_t error(uint8_t responseSize) const {
    return data[responseSize];
  }

} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpParamSetByNameResponse) // largest kind of param


// Port 3 (Commander)

struct crtpSetpointRequest
{
  crtpSetpointRequest(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust)
    : header(0x03, 0)
    , roll(roll)
    , pitch(pitch)
    , yawrate(yawrate)
    , thrust(thrust)
  {
  }
  const crtp header;
  float roll;
  float pitch;
  float yawrate;
  uint16_t thrust;
}  __attribute__((packed));
CHECKSIZE(crtpSetpointRequest)

// Port 4 (Memory access)

struct crtpMemoryGetNumberRequest
{
  crtpMemoryGetNumberRequest()
    : header(0x04, 0)
    , command(1)
  {
  }
  const crtp header;
  const uint8_t command;
}  __attribute__((packed));
CHECKSIZE(crtpMemoryGetNumberRequest)

struct crtpMemoryGetNumberResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size == 3 &&
             crtp(response.data[0]) == crtp(4, 0) &&
             response.data[1] == 1;
    }

    crtpMemoryGetNumberRequest request;
    uint8_t numberOfMemories;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpMemoryGetNumberResponse)

struct crtpMemoryGetInfoRequest
{
  crtpMemoryGetInfoRequest(
    uint8_t memId)
    : header(0x04, 0)
    , command(2)
    , memId(memId)
  {
  }
  const crtp header;
  const uint8_t command;
  uint8_t memId;
}  __attribute__((packed));
CHECKSIZE(crtpMemoryGetInfoRequest)

enum crtpMemoryType : uint8_t
{
  EEPROM = 0x00,
  OW     = 0x01,
  LED12  = 0x10,
  LOCO   = 0x11,
};

struct crtpMemoryGetInfoResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 2 &&
             crtp(response.data[0]) == crtp(4, 0) &&
             response.data[1] == 2;
    }

    crtpMemoryGetInfoRequest request;
    crtpMemoryType memType;
    uint32_t memSize; // Bytes
    uint64_t memAddr; // valid for OW and EEPROM
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpMemoryGetInfoResponse)

struct crtpMemoryReadRequest
{
  crtpMemoryReadRequest(
    uint8_t memId,
    uint32_t memAddr,
    uint8_t length)
    : header(0x04, 1)
    , memId(memId)
    , memAddr(memAddr)
    , length(length)
  {
  }
  const crtp header;
  uint8_t memId;
  uint32_t memAddr;
  uint8_t length;
}  __attribute__((packed));
CHECKSIZE(crtpMemoryReadRequest)

struct crtpMemoryReadResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 2 &&
             crtp(response.data[0]) == crtp(4, 1);
    }

    crtp header;
    uint8_t memId;
    uint32_t memAddr;
    uint8_t status;
    uint8_t data[24];
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpMemoryReadResponse)

struct crtpMemoryWriteRequest
{
  crtpMemoryWriteRequest(
    uint8_t memId,
    uint32_t memAddr)
    : header(0x04, 2)
    , memId(memId)
    , memAddr(memAddr)
  {
  }
  const crtp header;
  uint8_t memId;
  uint32_t memAddr;
  uint8_t data[24];
}  __attribute__((packed));
CHECKSIZE(crtpMemoryWriteRequest)

struct crtpMemoryWriteResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 2 &&
             crtp(response.data[0]) == crtp(4, 2);
    }

    crtp header;
    uint8_t memId;
    uint32_t memAddr;
    uint8_t status;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpMemoryWriteResponse)

// Port 5 (Data logging)

struct crtpLogGetInfoResponse;
struct crtpLogGetInfoRequest
{
  crtpLogGetInfoRequest()
    : header(5, 0)
    , command(1)
    {
    }

  bool operator==(const crtpLogGetInfoRequest& other) const {
    return header == other.header && command == other.command;
  }

  typedef crtpLogGetInfoResponse Response;

  const crtp header;
  const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpLogGetInfoRequest)

struct crtpLogGetInfoResponse
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 9 &&
           crtp(response.data[0]) == crtp(5, 0) &&
           response.data[1] == 1;
  }

  crtpLogGetInfoRequest request;
  // Number of log items contained in the log table of content
  uint8_t log_len;
  // CRC values of the log TOC memory content. This is a fingerprint of the copter build that can be used to cache the TOC
  uint32_t log_crc;
  // Maximum number of log packets that can be programmed in the copter
  uint8_t log_max_packet;
  // Maximum number of operation programmable in the copter. An operation is one log variable retrieval programming
  uint8_t log_max_ops;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogGetInfoResponse)

struct crtpLogGetItemResponse;
struct crtpLogGetItemRequest
{
  crtpLogGetItemRequest(uint8_t id)
    : header(5, 0)
    , command(0)
    , id(id)
  {
  }

  bool operator==(const crtpLogGetItemRequest& other) const {
    return header == other.header && command == other.command && id == other.id;
  }

  typedef crtpLogGetItemResponse Response;

  const crtp header;
  const uint8_t command;
  uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpLogGetItemRequest)

struct crtpLogGetItemResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 5 &&
             crtp(response.data[0]) == crtp(5, 0) &&
             response.data[1] == 0;
    }

    crtpLogGetItemRequest request;
    uint8_t type;
    char text[28]; // group, name
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogGetItemResponse)

struct logBlockItem {
  uint8_t logType;
  uint8_t id;
} __attribute__((packed));

struct crtpLogCreateBlockRequest
{
  crtpLogCreateBlockRequest()
  : header(5, 1)
  , command(0)
  {
  }

  const crtp header;
  const uint8_t command;
  uint8_t id;
  logBlockItem items[14];
} __attribute__((packed));
CHECKSIZE(crtpLogCreateBlockRequest)

// struct logAppendBlockRequest
// {
//   logAppendBlockRequest()
//     : header(5, 1)
//     , command(1)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     uint8_t id;
//     logBlockItem items[16];
// } __attribute__((packed));

// struct logDeleteBlockRequest
// {
//   logDeleteBlockRequest()
//     : header(5, 1)
//     , command(2)
//     {
//     }

//     const crtp header;
//     const uint8_t command;
//     uint8_t id;
// } __attribute__((packed));

struct crtpLogStartRequest
{
  crtpLogStartRequest(
    uint8_t id,
    uint8_t period)
    : header(5, 1)
    , command(3)
    , id(id)
    , period(period)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
    uint8_t period; // in increments of 10ms
} __attribute__((packed));
CHECKSIZE(crtpLogStartRequest)

struct crtpLogStopRequest
{
  crtpLogStopRequest(
    uint8_t id)
    : header(5, 1)
    , command(4)
    , id(id)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t id;
} __attribute__((packed));
CHECKSIZE(crtpLogStopRequest)

struct crtpLogResetRequest
{
  crtpLogResetRequest()
    : header(5, 1)
    , command(5)
    {
    }

    const crtp header;
    const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpLogResetRequest)

enum crtpLogControlResult {
  crtpLogControlResultOk            = 0,
  crtpLogControlResultOutOfMemory   = 12, // ENOMEM
  crtpLogControlResultCmdNotFound   = 8,  // ENOEXEC
  crtpLogControlResultWrongBlockId  = 2,  // ENOENT
  crtpLogControlResultBlockTooLarge = 7,  // E2BIG
  crtpLogControlResultBlockExists   = 17, // EEXIST

};

struct crtpLogControlResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size == 4 &&
             crtp(response.data[0]) == crtp(5, 1);
    }

    crtp header;
    uint8_t command;
    uint8_t requestByte1;
    uint8_t result; // one of crtpLogControlResult
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogControlResponse)

struct crtpLogDataResponse
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 4 &&
             crtp(response.data[0]) == crtp(5, 2);
    }

    crtp header;
    uint8_t blockId;
    uint8_t timestampLo;
    uint16_t timestampHi;
    uint8_t data[26];
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogDataResponse)

// V2
struct crtpLogGetInfoV2Response;
struct crtpLogGetInfoV2Request
{
  crtpLogGetInfoV2Request()
    : header(5, 0)
    , command(3)
    {
    }

  bool operator==(const crtpLogGetInfoV2Request& other) const {
    return header == other.header && command == other.command;
  }

  typedef crtpLogGetInfoV2Response Response;

  const crtp header;
  const uint8_t command;
} __attribute__((packed));
CHECKSIZE(crtpLogGetInfoV2Request)

struct crtpLogGetInfoV2Response
{
  static bool match(const Crazyradio::Ack& response) {
    return response.size == 10 &&
           crtp(response.data[0]) == crtp(5, 0) &&
           response.data[1] == 3;
  }

  crtpLogGetInfoRequest request;
  // Number of log items contained in the log table of content
  uint16_t log_len;
  // CRC values of the log TOC memory content. This is a fingerprint of the copter build that can be used to cache the TOC
  uint32_t log_crc;
  // Maximum number of log packets that can be programmed in the copter
  uint8_t log_max_packet;
  // Maximum number of operation programmable in the copter. An operation is one log variable retrieval programming
  uint8_t log_max_ops;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogGetInfoV2Response)

struct crtpLogGetItemV2Response;
struct crtpLogGetItemV2Request
{
  crtpLogGetItemV2Request(uint16_t id)
    : header(5, 0)
    , command(2)
    , id(id)
  {
  }

  bool operator==(const crtpLogGetItemV2Request& other) const {
    return header == other.header && command == other.command && id == other.id;
  }

  typedef crtpLogGetItemV2Response Response;

  const crtp header;
  const uint8_t command;
  uint16_t id;
} __attribute__((packed));
CHECKSIZE(crtpLogGetItemV2Request)

struct crtpLogGetItemV2Response
{
    static bool match(const Crazyradio::Ack& response) {
      return response.size > 6 &&
             crtp(response.data[0]) == crtp(5, 0) &&
             response.data[1] == 2;
    }

    crtpLogGetItemV2Request request;
    uint8_t type;
    char text[27]; // group, name
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpLogGetItemV2Response)

struct logBlockItemV2 {
  uint8_t logType;
  uint16_t id;
} __attribute__((packed));

struct crtpLogCreateBlockV2Request
{
  crtpLogCreateBlockV2Request()
  : header(5, 1)
  , command(6)
  {
  }

  const crtp header;
  const uint8_t command;
  uint8_t id;
  logBlockItemV2 items[9];
} __attribute__((packed));
CHECKSIZE(crtpLogCreateBlockV2Request)

struct crtpLogAppendBlockV2Request
{
  crtpLogAppendBlockV2Request()
  : header(5, 1)
  , command(7)
  {
  }

  const crtp header;
  const uint8_t command;
  uint8_t id;
  logBlockItemV2 items[9];
} __attribute__((packed));
CHECKSIZE(crtpLogAppendBlockV2Request)

// Port 0x06 (External Position Update)

struct crtpExternalPositionUpdate
{
  crtpExternalPositionUpdate(
    float x,
    float y,
    float z)
    : header(0x06, 0)
    , x(x)
    , y(y)
    , z(z)
  {
  }
  const crtp header;
  float x;
  float y;
  float z;
}  __attribute__((packed));
CHECKSIZE(crtpExternalPositionUpdate)

struct crtpExternalPositionPacked
{
  crtpExternalPositionPacked()
    : header(0x06, 2)
  {
  }
  const crtp header;
  struct {
    uint8_t id;
    int16_t x; // mm
    int16_t y; // mm
    int16_t z; // mm
  } __attribute__((packed)) positions[4];
}  __attribute__((packed));
CHECKSIZE(crtpExternalPositionPacked)

struct crtpEmergencyStopRequest
{
  crtpEmergencyStopRequest()
    : header(0x06, 1)
  {
  }
  const crtp header;
  const uint8_t type = 3;
}  __attribute__((packed));
CHECKSIZE(crtpEmergencyStopRequest)

struct crtpEmergencyStopWatchdogRequest
{
  crtpEmergencyStopWatchdogRequest()
    : header(0x06, 1)
  {
  }
  const crtp header;
  const uint8_t type = 4;
}  __attribute__((packed));
CHECKSIZE(crtpEmergencyStopWatchdogRequest)

struct crtpExternalPoseUpdate
{
  crtpExternalPoseUpdate(
    float x,
    float y,
    float z,
    float qx,
    float qy,
    float qz,
    float qw)
    : header(0x06, 1)
    , x(x)
    , y(y)
    , z(z)
    , qx(qx)
    , qy(qy)
    , qz(qz)
    , qw(qw)
  {
  }
  const crtp header;
  const uint8_t type = 8;
  float x;
  float y;
  float z;
  float qx;
  float qy;
  float qz;
  float qw;
}  __attribute__((packed));
CHECKSIZE(crtpExternalPoseUpdate)

struct crtpExternalPosePacked
{
  crtpExternalPosePacked()
    : header(0x06, 1)
  {
  }
  const crtp header;
  const uint8_t type = 9;
  struct {
    uint8_t id; // last 8 bit of the Crazyflie address
    int16_t x; // mm
    int16_t y; // mm
    int16_t z; // mm
    uint32_t quat; // compressed quaternion, see quatcompress.h
  } __attribute__((packed)) poses[2];
}  __attribute__((packed));
CHECKSIZE(crtpExternalPosePacked)

struct crtpStopRequest
{
  crtpStopRequest();
  const crtp header;
  uint8_t type;
} __attribute__((packed));
CHECKSIZE(crtpStopRequest)

struct crtpHoverSetpointRequest
{
  crtpHoverSetpointRequest(
    float vx,
    float vy,
    float yawrate,
    float zDistance);
  const crtp header;
  uint8_t type;
  float vx;
  float vy;
  float yawrate;
  float zDistance;
} __attribute__((packed));
CHECKSIZE(crtpHoverSetpointRequest)

struct crtpPositionSetpointRequest
{
  crtpPositionSetpointRequest(
    float x,
    float y,
    float z,
    float yaw);
  const crtp header;
  uint8_t type;
  float x;
  float y;
  float z;
  float yaw;
} __attribute__((packed));
CHECKSIZE(crtpPositionSetpointRequest)

// Port 0x07 (Generic Setpoint)

struct crtpFullStateSetpointRequest
{
  crtpFullStateSetpointRequest(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate);
  const crtp header;
  uint8_t type;
  int16_t x;
  int16_t y;
  int16_t z;
  int16_t vx;
  int16_t vy;
  int16_t vz;
  int16_t ax;
  int16_t ay;
  int16_t az;
  int32_t quat; // compressed quaternion, xyzw
  int16_t omegax;
  int16_t omegay;
  int16_t omegaz;
} __attribute__((packed));
CHECKSIZE(crtpFullStateSetpointRequest)

// Port 0x08 (High-level Setpoints)

struct crtpCommanderHighLevelSetGroupMaskRequest
{
  crtpCommanderHighLevelSetGroupMaskRequest(
    uint8_t groupMask)
    : header(0x08, 0)
    , command(0)
    , groupMask(groupMask)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t groupMask;
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelSetGroupMaskRequest)

struct crtpCommanderHighLevelTakeoffRequest
{
  crtpCommanderHighLevelTakeoffRequest(
    uint8_t groupMask,
    float height,
    float duration)
    : header(0x08, 0)
    , command(1)
    , groupMask(groupMask)
    , height(height)
    , duration(duration)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t groupMask;        // mask for which CFs this should apply to
    float height;             // m (absolute)
    float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelTakeoffRequest)

struct crtpCommanderHighLevelLandRequest
{
  crtpCommanderHighLevelLandRequest(
    uint8_t groupMask,
    float height,
    float duration)
    : header(0x08, 0)
    , command(2)
    , groupMask(groupMask)
    , height(height)
    , duration(duration)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t groupMask;        // mask for which CFs this should apply to
    float height;             // m (absolute)
    float duration;           // s (time it should take until target height is reached)
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelLandRequest)

struct crtpCommanderHighLevelStopRequest
{
  crtpCommanderHighLevelStopRequest(
    uint8_t groupMask)
    : header(0x08, 0)
    , command(3)
    , groupMask(groupMask)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t groupMask;        // mask for which CFs this should apply to
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelStopRequest)

struct crtpCommanderHighLevelGoToRequest
{
  crtpCommanderHighLevelGoToRequest(
    uint8_t groupMask,
    bool relative,
    float x,
    float y,
    float z,
    float yaw,
    float duration)
    : header(0x08, 0)
    , command(4)
    , groupMask(groupMask)
    , relative(relative)
    , x(x)
    , y(y)
    , z(z)
    , yaw(yaw)
    , duration(duration)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t groupMask; // mask for which CFs this should apply to
    uint8_t relative;  // set to true, if position/yaw are relative to current setpoint
    float x; // m
    float y; // m
    float z; // m
    float yaw; // deg
    float duration; // sec
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelGoToRequest)

struct crtpCommanderHighLevelStartTrajectoryRequest
{
  crtpCommanderHighLevelStartTrajectoryRequest(
    uint8_t groupMask,
    bool relative,
    bool reversed,
    uint8_t trajectoryId,
    float timescale)
    : header(0x08, 0)
    , command(5)
    , groupMask(groupMask)
    , relative(relative)
    , reversed(reversed)
    , trajectoryId(trajectoryId)
    , timescale(timescale)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t groupMask; // mask for which CFs this should apply to
    uint8_t relative;  // set to true, if trajectory should be shifted to current setpoint
    uint8_t reversed;  // set to true, if trajectory should be executed in reverse
    uint8_t trajectoryId; // id of the trajectory (previously defined by COMMAND_DEFINE_TRAJECTORY)
    float timescale; // time factor; 1 = original speed; >1: slower; <1: faster
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelStartTrajectoryRequest)

enum TrajectoryLocation_e {
  TRAJECTORY_LOCATION_INVALID = 0,
  TRAJECTORY_LOCATION_MEM     = 1, // for trajectories that are uploaded dynamically
  // Future features might include trajectories on flash or uSD card
};

enum TrajectoryType_e {
  TRAJECTORY_TYPE_POLY4D = 0, // struct poly4d, see pptraj.h
  // Future types might include versions without yaw
};

struct trajectoryDescription
{
  uint8_t trajectoryLocation; // one of TrajectoryLocation_e
  uint8_t trajectoryType;     // one of TrajectoryType_e
  union
  {
    struct {
      uint32_t offset;  // offset in uploaded memory
      uint8_t n_pieces;
    } __attribute__((packed)) mem; // if trajectoryLocation is TRAJECTORY_LOCATION_MEM
  } trajectoryIdentifier;
} __attribute__((packed));

struct crtpCommanderHighLevelDefineTrajectoryRequest
{
  crtpCommanderHighLevelDefineTrajectoryRequest(
    uint8_t trajectoryId)
    : header(0x08, 0)
    , command(6)
    , trajectoryId(trajectoryId)
    {
    }

    const crtp header;
    const uint8_t command;
    uint8_t trajectoryId;
    struct trajectoryDescription description;
} __attribute__((packed));
CHECKSIZE(crtpCommanderHighLevelDefineTrajectoryRequest)

// Port 13 (Platform)

struct crtpGetProtocolVersionRequest
{
  crtpGetProtocolVersionRequest()
    : header(0x0D, 1)
    {
    }

    const crtp header;
    const uint8_t cmd = 0;
} __attribute__((packed));
CHECKSIZE(crtpGetProtocolVersionRequest)

struct crtpGetProtocolVersionResponse
{
  crtpGetProtocolVersionRequest request;
  int version;
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpGetProtocolVersionResponse)

struct crtpGetFirmwareVersionRequest
{
  crtpGetFirmwareVersionRequest()
    : header(0x0D, 1)
    {
    }

    const crtp header;
    const uint8_t cmd = 1;
} __attribute__((packed));
CHECKSIZE(crtpGetProtocolVersionRequest)

struct crtpGetFirmwareVersionResponse
{
  crtpGetFirmwareVersionRequest request;
  char version[30];
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpGetFirmwareVersionResponse)

struct crtpGetDeviceTypeNameRequest
{
  crtpGetDeviceTypeNameRequest()
    : header(0x0D, 1)
    {
    }

    const crtp header;
    const uint8_t cmd = 2;
} __attribute__((packed));
CHECKSIZE(crtpGetProtocolVersionRequest)

struct crtpGetDeviceTypeNameResponse
{
  crtpGetDeviceTypeNameRequest request;
  char name[30];
} __attribute__((packed));
CHECKSIZE_RESPONSE(crtpGetDeviceTypeNameResponse)

// The crazyflie-nrf firmware sends empty packets with the signal strength, if nothing else is in the queue
struct crtpPlatformRSSIAck
{
    static bool match(const Crazyradio::Ack& response) {
      return crtp(response.data[0]) == crtp(15, 3);
    }

    crtp header;
    uint8_t reserved;
    uint8_t rssi;
};
CHECKSIZE_RESPONSE(crtpPlatformRSSIAck)
