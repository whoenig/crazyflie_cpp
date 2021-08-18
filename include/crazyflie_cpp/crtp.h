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

#if 0
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
#endif
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

#if 0
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
#endif

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

struct crtpVelocityWorldSetpointRequest
{
  crtpVelocityWorldSetpointRequest(
      float x, float y, float z, float yawRate)
      : header(0X07, 0), type(1), x(x), y(y), z(z), yawRate(yawRate)
  {
  }
  const crtp header;
  uint8_t type;
  float x;
  float y;
  float z;
  float yawRate;
}__attribute__((packed));
CHECKSIZE(crtpVelocityWorldSetpointRequest);

struct crtpNotifySetpointsStopRequest
{
  crtpNotifySetpointsStopRequest(uint32_t remainValidMillisecs)
    : header(0x07, 1), type(0), remainValidMillisecs(remainValidMillisecs)
  {
  }
  const crtp header;
  uint8_t type;
  uint32_t remainValidMillisecs;
}__attribute__((packed));
CHECKSIZE(crtpNotifySetpointsStopRequest);

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
#endif

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