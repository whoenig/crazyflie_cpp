#pragma once

#include <cstdint>

#include <crazyflieLinkCpp/Packet.hpp>
#if 0
static int const CRTP_MAX_DATA_SIZE = 30;
static int const CRTP_MAXSIZE = 31;
#define CHECKSIZE(s) static_assert(sizeof(s) <= CRTP_MAXSIZE, #s " packet is too large");
#define CHECKSIZE_WITH_STATE(s, stateSize) static_assert(sizeof(s) - stateSize <= CRTP_MAXSIZE, #s " packet is too large");

static int const CRTP_MAXSIZE_RESPONSE = 32;
#define CHECKSIZE_RESPONSE(s) static_assert(sizeof(s) <= CRTP_MAXSIZE_RESPONSE, #s " packet is too large");
#endif
uint32_t quatcompress(float const q[4]);
void quatdecompress(uint32_t comp, float q[4]);
#if 0
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
#endif

// Port 0 (Console)
struct crtpConsoleResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static std::string text(const bitcraze::crazyflieLinkCpp::Packet &p);
};

//////////////////////
// Port 2 (Parameters)
//////////////////////

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

// Get basic information about a specific parameter
class crtpParamTocGetItemV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpParamTocGetItemV2Request(uint16_t id);
};

struct crtpParamTocGetItemV2Response
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static uint16_t id(const bitcraze::crazyflieLinkCpp::Packet &p);
  static ParamType type(const bitcraze::crazyflieLinkCpp::Packet &p);
  static bool readonly(const bitcraze::crazyflieLinkCpp::Packet &p);
  static std::pair<std::string, std::string> groupAndName(const bitcraze::crazyflieLinkCpp::Packet &p);
};

// Get basic information about param table of contents (TOC)
class crtpParamTocGetInfoV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpParamTocGetInfoV2Request();
};

struct crtpParamTocGetInfoV2Response
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static uint16_t numParams(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint32_t crc(const bitcraze::crazyflieLinkCpp::Packet &p);
};

// Request the current value of a parameter
class crtpParamReadV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpParamReadV2Request(uint16_t id);
};

struct crtpParamValueV2Response
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static uint16_t id(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint8_t status(const bitcraze::crazyflieLinkCpp::Packet &p);

  template <typename T>
  static T value(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<T>(3);
  }
};

// Set the current value of a parameter
template <class T>
class crtpParamSetByNameRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpParamSetByNameRequest(
      const std::string& group,
      const std::string& name,
      const T &value)
      : Packet(2, 3, 1+group.size()+1+name.size()+1+1+sizeof(T))
  {
    setPayloadAt<uint8_t>(0, 0); // command
    size_t idx = 1;
    setPayloadAtString(idx, group); // group
    idx += group.size() + 1;
    setPayloadAtString(idx, name);  // name
    idx += name.size() + 1;
    setPayloadAt<uint8_t>(idx, deductParamType(value)); // type
    setPayloadAt<T>(idx+1, value);  // value
  }

private:
  ParamType deductParamType(const uint8_t&) { return ParamTypeUint8;}
  ParamType deductParamType(const int8_t&) { return ParamTypeInt8;}
  ParamType deductParamType(const uint16_t&) { return ParamTypeUint16;}
  ParamType deductParamType(const int16_t&) { return ParamTypeInt16;}
  ParamType deductParamType(const uint32_t&) { return ParamTypeUint32;}
  ParamType deductParamType(const int32_t&) { return ParamTypeInt32;}
  ParamType deductParamType(const float&) { return ParamTypeFloat;}
};

struct crtpParamSetByNameResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 2 &&
           p.channel() == 3 &&
           p.payloadSize() >= 2 &&
           p.payloadAt<uint8_t>(0) == 0;
  }

  static std::pair<std::string, std::string> groupAndName(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    auto group = p.payloadAtString(1);
    auto name = p.payloadAtString(1 + group.length() + 1);
    return std::make_pair(group, name);
  }

  static uint8_t error(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(p.payloadSize()-1);
  }
};

template <class T>
class crtpParamWriteV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpParamWriteV2Request(
      uint16_t id,
      const T &value)
      : Packet(2, 2, 2 + sizeof(T))
  {
    setPayloadAt<uint16_t>(0, id);
    setPayloadAt<T>(2, value);
  }
};

// Port 3 (Commander)

class crtpSetpointRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpSetpointRequest(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust)
    : Packet(3, 0, 14)
  {
    setPayloadAt<float>(0, roll);
    setPayloadAt<float>(4, pitch);
    setPayloadAt<float>(8, yawrate);
    setPayloadAt<uint16_t>(12, thrust);
  }
};

// Port 4 (Memory access)

class crtpMemoryGetNumberRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpMemoryGetNumberRequest()
    : Packet(4, 0, 1)
  {
    setPayloadAt<uint8_t>(0, 1);
  }
};

struct crtpMemoryGetNumberResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 4 &&
           p.channel() == 0 &&
           p.payloadSize() == 2 &&
           p.payloadAt<uint8_t>(0) == 1;
  }

  static uint8_t numberOfMemories(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(1);
  }
};

class crtpMemoryGetInfoRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpMemoryGetInfoRequest(uint8_t id)
      : Packet(4, 0, 2)
  {
    setPayloadAt<uint8_t>(0, 2);
    setPayloadAt<uint8_t>(1, id);
  }
};

struct crtpMemoryGetInfoResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 4 &&
           p.channel() == 0 &&
           p.payloadSize() == 15 &&
           p.payloadAt<uint8_t>(0) == 2;
  }

  static uint8_t id(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(1);
  }

  static uint8_t type(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(2);
  }

  static uint32_t size(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint32_t>(3);
  }

  static uint64_t addr(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint64_t>(7);
  }
};

class crtpMemoryReadRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpMemoryReadRequest(
      uint8_t memId,
      uint32_t memAddr,
      uint8_t length)
      : Packet(4, 1, 6)
  {
    setPayloadAt<uint8_t>(0, memId);
    setPayloadAt<uint32_t>(1, memAddr);
    setPayloadAt<uint8_t>(5, length);
  }
};

struct crtpMemoryReadResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 4 &&
           p.channel() == 1 &&
           p.payloadSize() > 6;
  }

  static uint8_t id(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(0);
  }

  static uint32_t address(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint32_t>(1);
  }

  static uint8_t status(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(5);
  }

  static uint8_t dataSize(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadSize() - 6;
  }

  template <typename T>
  static T dataAt(const bitcraze::crazyflieLinkCpp::Packet &p, uint8_t idx)
  {
    return p.payloadAt<T>(6 + idx);
  }

  static const uint8_t* data(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return &p.payload()[6];
  }
};

class crtpMemoryWriteRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpMemoryWriteRequest(
      uint8_t memId,
      uint32_t memAddr)
      : Packet(4, 2, 5)
  {
    setPayloadAt<uint8_t>(0, memId);
    setPayloadAt<uint32_t>(1, memAddr);
  }

  void setDataAt(uint8_t idx, const uint8_t* data, size_t size)
  {
    setPayloadAt(5+idx, data, size);
  }

  template <typename T>
  void setDataAt(uint8_t idx, const T& data)
  {
    setPayloadAt<T>(5+idx, data);
  }

  void setDataSize(uint8_t size)
  {
    setPayloadSize(5+size);
  }
};

struct crtpMemoryWriteResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 4 &&
           p.channel() == 2 &&
           p.payloadSize() == 6;
  }

  static uint8_t id(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(0);
  }

  static uint32_t address(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint32_t>(1);
  }

  static uint8_t status(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(5);
  }
};

// Port 5 (Data logging)

// Get basic information about table of contents (TOC)
class crtpLogGetInfoV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogGetInfoV2Request();
};

struct crtpLogGetInfoV2Response
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static uint16_t numLogVariables(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint32_t crc(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint8_t numMaxLogBlocks(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint8_t numMaxOps(const bitcraze::crazyflieLinkCpp::Packet &p);
};

// Get basic information about a specific entries
class crtpLogGetItemV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogGetItemV2Request(uint16_t id);
};

struct crtpLogGetItemV2Response
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static uint16_t id(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint8_t type(const bitcraze::crazyflieLinkCpp::Packet &p); // one of LogType actually
  static std::pair<std::string, std::string> groupAndName(const bitcraze::crazyflieLinkCpp::Packet &p);
};

class crtpLogCreateBlockV2Request
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogCreateBlockV2Request(uint8_t id)
      : Packet(5, 1, 2)
  {
    setPayloadAt<uint8_t>(0, 6); // command
    setPayloadAt<uint8_t>(1, id); // logBlock Id
  }

  void add(uint8_t logType, uint16_t id)
  {
    uint8_t idx = payloadSize();
    setPayloadSize(idx + 3);
    setPayloadAt<uint8_t>(idx, logType);
    setPayloadAt<uint16_t>(idx+1, id);
  }
};

class crtpLogStartRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogStartRequest(uint8_t id, uint8_t period)
      : Packet(5, 1, 3)
  {
    setPayloadAt<uint8_t>(0, 3);  // command
    setPayloadAt<uint8_t>(1, id); // logBlock Id
    setPayloadAt<uint8_t>(2, period); // period in increments of 10ms
  }
};

class crtpLogStopRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogStopRequest(uint8_t id)
      : Packet(5, 1, 2)
  {
    setPayloadAt<uint8_t>(0, 4);      // command
    setPayloadAt<uint8_t>(1, id);     // logBlock Id
  }
};

class crtpLogResetRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLogResetRequest()
      : Packet(5, 1, 1)
  {
    setPayloadAt<uint8_t>(0, 5);  // command
  }
};

enum crtpLogControlResult : uint8_t {
  crtpLogControlResultOk            = 0,
  crtpLogControlResultOutOfMemory   = 12, // ENOMEM
  crtpLogControlResultCmdNotFound   = 8,  // ENOEXEC
  crtpLogControlResultWrongBlockId  = 2,  // ENOENT
  crtpLogControlResultBlockTooLarge = 7,  // E2BIG
  crtpLogControlResultBlockExists   = 17, // EEXIST

};

struct crtpLogControlResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 5 &&
           p.channel() == 1 &&
           p.payloadSize() == 3;
  }

  static uint8_t command(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(0);
  }

  static uint8_t requestByte1(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(1);
  }

  static crtpLogControlResult result(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<crtpLogControlResult>(2);
  }
};

struct crtpLogDataResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.port() == 5 &&
           p.channel() == 2 &&
           p.payloadSize() > 3;
  }

  static uint8_t blockId(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    return p.payloadAt<uint8_t>(0);
  }

  static uint32_t timestampMS(const bitcraze::crazyflieLinkCpp::Packet &p)
  {
    uint8_t timestampLo = p.payloadAt<uint8_t>(1);
    uint16_t timestampHi = p.payloadAt<uint16_t>(2);
    uint32_t time_in_ms = ((uint32_t)timestampHi << 8) | timestampLo;
    return time_in_ms;
  }

  template <typename T>
  static T variableAt(const bitcraze::crazyflieLinkCpp::Packet &p, uint8_t idx)
  {
    return p.payloadAt<T>(4+idx);
  }
};

#if 0
// V2
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

#endif

class crtpExternalPositionPacked
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpExternalPositionPacked()
      : Packet(0x06, 2, 0)
  {
  }

  void add(uint8_t id, int16_t x, int16_t y, int16_t z)
  {
    uint8_t idx = payloadSize();
    setPayloadSize(idx + 7);
    setPayloadAt<uint8_t>(idx, id);
    setPayloadAt<int16_t>(idx + 1, x); // mm
    setPayloadAt<int16_t>(idx + 3, y); // mm
    setPayloadAt<int16_t>(idx + 5, z); // mm
  }

  void clear()
  {
    setPayloadSize(0);
  }
};

class crtpEmergencyStopRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpEmergencyStopRequest()
      : Packet(0x06, 1, 1)
  {
    setPayloadAt<uint8_t>(0, 3); // type
  }
};

class crtpEmergencyStopWatchdogRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpEmergencyStopWatchdogRequest()
      : Packet(0x06, 1, 1)
  {
    setPayloadAt<uint8_t>(0, 4); // type
  }
};

#if 0

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

#endif

class crtpExternalPosePacked
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpExternalPosePacked()
      : Packet(0x06, 1, 1)
  {
    setPayloadAt<uint8_t>(0, 9); // type
  }

  void add(uint8_t id, int16_t x, int16_t y, int16_t z, uint32_t quat)
  {
    uint8_t idx = payloadSize();
    setPayloadSize(idx + 11);
    setPayloadAt<uint8_t>(idx, id);     // last 8 bit of the Crazyflie address
    setPayloadAt<int16_t>(idx + 1, x); // mm
    setPayloadAt<int16_t>(idx + 3, y); // mm
    setPayloadAt<int16_t>(idx + 5, z); // mm
    setPayloadAt<uint32_t>(idx + 7, quat); // compressed quaternion, see quatcompress.h
  }

  void clear()
  {
    setPayloadSize(1);
  }
};

class crtpStopRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpStopRequest()
      : Packet(0x07, 0, 1)
  {
    setPayloadAt<uint8_t>(0, 0);  // type
  }
};

class crtpHoverSetpointRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpHoverSetpointRequest(
      float vx,
      float vy,
      float yawrate,
      float zDistance)
      : Packet(0x07, 0, 17)
  {
    setPayloadAt<uint8_t>(0, 5); // type
    setPayloadAt<float>(1, vx); // m/s
    setPayloadAt<float>(5, vy); // m/s
    setPayloadAt<float>(9, yawrate); // deg/s
    setPayloadAt<float>(13, zDistance); // m
  }
};

class crtpPositionSetpointRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpPositionSetpointRequest(
      float x,
      float y,
      float z,
      float yaw)
      : Packet(0x07, 0, 17)
  {
    setPayloadAt<uint8_t>(0, 7);  // type
    setPayloadAt<float>(1, x);    // m
    setPayloadAt<float>(5, y);    // m
    setPayloadAt<float>(9, z);    // m
    setPayloadAt<float>(13, yaw); // deg
  }
};

// Port 0x07 (Generic Setpoint)

class crtpFullStateSetpointRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpFullStateSetpointRequest(
      float x, float y, float z,
      float vx, float vy, float vz,
      float ax, float ay, float az,
      float qx, float qy, float qz, float qw,
      float rollRate, float pitchRate, float yawRate)
      : Packet(0x07, 0, 29)
  {
    setPayloadAt<uint8_t>(0, 6);          // type
    setPayloadAt<int16_t>(1, x * 1000);  // mm
    setPayloadAt<int16_t>(3, y * 1000);  // mm
    setPayloadAt<int16_t>(5, z * 1000);  // mm
    setPayloadAt<int16_t>(7, vx * 1000);  // mm/s
    setPayloadAt<int16_t>(9, vy * 1000);  // mm/s
    setPayloadAt<int16_t>(11, vz * 1000); // mm/s
    setPayloadAt<int16_t>(13, ax * 1000); // mm/s^2
    setPayloadAt<int16_t>(15, ay * 1000); // mm/s^2
    setPayloadAt<int16_t>(17, az * 1000); // mm/s^2
    float q[4] = {qx, qy, qz, qw};
    setPayloadAt<int32_t>(19, quatcompress(q));
    setPayloadAt<int16_t>(23, rollRate * 1000);   // millirad/s
    setPayloadAt<int16_t>(25, pitchRate * 1000);  // millirad/s
    setPayloadAt<int16_t>(27, yawRate * 1000);    // millirad/s
  }
};

class crtpVelocityWorldSetpointRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpVelocityWorldSetpointRequest(
      float x, float y, float z, float yawRate)
      : Packet(0x07, 0, 17)
  {
    setPayloadAt<uint8_t>(0, 1);        // type
    setPayloadAt<float>(1, x);          // m
    setPayloadAt<float>(5, y);          // m
    setPayloadAt<float>(9, z);          // m
    setPayloadAt<float>(13, yawRate);   // rad
  }
};

class crtpNotifySetpointsStopRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpNotifySetpointsStopRequest(
      uint32_t remainValidMillisecs)
      : Packet(0x07, 1, 5)
  {
    setPayloadAt<uint8_t>(0, 0);      // type
    setPayloadAt<uint32_t>(1, remainValidMillisecs);
  }
};

// Port 0x08 (High-level Setpoints)

class crtpCommanderHighLevelSetGroupMaskRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelSetGroupMaskRequest(
      uint8_t groupMask)
      : Packet(8, 0, 2)
  {
    setPayloadAt<uint8_t>(0, 0);         // command
    setPayloadAt<uint8_t>(1, groupMask); // mask of this CF
  }
};

class crtpCommanderHighLevelTakeoffRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelTakeoffRequest(
      uint8_t groupMask,
      float height,
      float duration)
      : Packet(8, 0, 10)
  {
    setPayloadAt<uint8_t>(0, 1);         // command
    setPayloadAt<uint8_t>(1, groupMask); // mask for which CFs this should apply to
    setPayloadAt<float>(2, height);      // m (absolute)
    setPayloadAt<float>(6, duration);    // s (time it should take until target height is reached)
  }
};

class crtpCommanderHighLevelLandRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelLandRequest(
      uint8_t groupMask,
      float height,
      float duration)
      : Packet(8, 0, 10)
  {
    setPayloadAt<uint8_t>(0, 2);        // command
    setPayloadAt<uint8_t>(1, groupMask);// mask for which CFs this should apply to
    setPayloadAt<float>(2, height);     // m (absolute)
    setPayloadAt<float>(6, duration);   // s (time it should take until target height is reached)
  }
};

class crtpCommanderHighLevelStopRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelStopRequest(
      uint8_t groupMask)
      : Packet(8, 0, 2)
  {
    setPayloadAt<uint8_t>(0, 3);         // command
    setPayloadAt<uint8_t>(1, groupMask); // mask for which CFs this should apply to
  }
};

class crtpCommanderHighLevelGoToRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelGoToRequest(
      uint8_t groupMask,
      bool relative,
      float x,
      float y,
      float z,
      float yaw,
      float duration)
      : Packet(8, 0, 23)
  {
    setPayloadAt<uint8_t>(0, 4);            // command
    setPayloadAt<uint8_t>(1, groupMask);    // mask for which CFs this should apply to
    setPayloadAt<uint8_t>(2, relative);     // set to true, if trajectory should be shifted to current setpoint
    setPayloadAt<float>(3, x);              // m
    setPayloadAt<float>(7, y);              // m
    setPayloadAt<float>(11, z);             // m
    setPayloadAt<float>(15, yaw);           // deg
    setPayloadAt<float>(19, duration);      // sec
  }
};

class crtpCommanderHighLevelStartTrajectoryRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelStartTrajectoryRequest(
      uint8_t groupMask,
      bool relative,
      bool reversed,
      uint8_t trajectoryId,
      float timescale)
      : Packet(8, 0, 9)
  {
    setPayloadAt<uint8_t>(0, 5);  // command
    setPayloadAt<uint8_t>(1, groupMask); // mask for which CFs this should apply to
    setPayloadAt<uint8_t>(2, relative);  // set to true, if trajectory should be shifted to current setpoint
    setPayloadAt<uint8_t>(3, reversed);  // set to true, if trajectory should be executed in reverse
    setPayloadAt<uint8_t>(4, trajectoryId); // id of the trajectory (previously defined by COMMAND_DEFINE_TRAJECTORY)
    setPayloadAt<float>(5, timescale);      // time factor; 1 = original speed; >1: slower; <1: faster
  }
};

enum TrajectoryLocation_e {
  TRAJECTORY_LOCATION_INVALID = 0,
  TRAJECTORY_LOCATION_MEM     = 1, // for trajectories that are uploaded dynamically
  // Future features might include trajectories on flash or uSD card
};

enum TrajectoryType_e {
  TRAJECTORY_TYPE_POLY4D = 0, // struct poly4d, see pptraj.h
  // Future types might include versions without yaw
};

class crtpCommanderHighLevelDefineTrajectoryRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpCommanderHighLevelDefineTrajectoryRequest(
      uint8_t trajectoryId)
      : Packet(8, 0, 2)
  {
    setPayloadAt<uint8_t>(0, 6);            // command
    setPayloadAt<uint8_t>(1, trajectoryId);
  }

  void setPoly4d(uint32_t offset, uint8_t n_pieces)
  {
    setPayloadAt<uint8_t>(2, TRAJECTORY_LOCATION_MEM); // trajectoryLocation
    setPayloadAt<uint8_t>(3, TRAJECTORY_TYPE_POLY4D);  // trajectoryType
    setPayloadAt<uint32_t>(4, offset);                 // offset in uploaded memory
    setPayloadAt<uint8_t>(8, n_pieces);                // trajectoryType
    setPayloadSize(9);
  }
};

// Port 13 (Platform)
class crtpGetProtocolVersionRequest
  : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpGetProtocolVersionRequest();
};

struct crtpGetProtocolVersionResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static int version(const bitcraze::crazyflieLinkCpp::Packet &p);
};

class crtpGetFirmwareVersionRequest
  : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpGetFirmwareVersionRequest();
};

struct crtpGetFirmwareVersionResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static std::string version(const bitcraze::crazyflieLinkCpp::Packet &p);
};

class crtpGetDeviceTypeNameRequest
    : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpGetDeviceTypeNameRequest();
};

struct crtpGetDeviceTypeNameResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static std::string name(const bitcraze::crazyflieLinkCpp::Packet &p);
};

// Port 15 (Link)
class crtpLatencyMeasurementRequest
  : public bitcraze::crazyflieLinkCpp::Packet
{
public:
  crtpLatencyMeasurementRequest(
      uint32_t id,
      uint64_t timestamp)
      : Packet(15, 0, 12)
  {
    setPayloadAt<uint32_t>(0, id);
    setPayloadAt<uint64_t>(4, timestamp);
  }
};

struct crtpLatencyMeasurementResponse
{
  static bool valid(const bitcraze::crazyflieLinkCpp::Packet &p);

  static uint32_t id(const bitcraze::crazyflieLinkCpp::Packet &p);
  static uint64_t timestamp(const bitcraze::crazyflieLinkCpp::Packet &p);
};

#if 0
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
#endif