#pragma once

#include <cstring>
#include <sstream>
#include <functional>
#include <math.h>

#include "crtp.h"
#include <list>
#include <set>
#include <map>
#include <chrono>
#include <cassert>

#include <crazyflieLinkCpp/Connection.h>

#ifdef __GNUC__
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#endif

#ifdef _MSC_VER
#define PACK(__Declaration__) __pragma(pack(push, 1)) __Declaration__ __pragma(pack(pop))
#endif

class Logger
{
public:
  Logger() {}
  virtual ~Logger() {}

  virtual void info(const std::string& /*msg*/) {}
  virtual void warning(const std::string& /*msg*/) {}
  virtual void error(const std::string& /*msg*/) {}
};

extern Logger EmptyLogger;

class Crazyflie
{
public:
  enum ParamType {
    ParamTypeUint8  = 0x00 | (0x00<<2) | (0x01<<3),
    ParamTypeInt8   = 0x00 | (0x00<<2) | (0x00<<3),
    ParamTypeUint16 = 0x01 | (0x00<<2) | (0x01<<3),
    ParamTypeInt16  = 0x01 | (0x00<<2) | (0x00<<3),
    ParamTypeUint32 = 0x02 | (0x00<<2) | (0x01<<3),
    ParamTypeInt32  = 0x02 | (0x00<<2) | (0x00<<3),
    ParamTypeFloat  = 0x02 | (0x01<<2) | (0x00<<3),
  };

  struct ParamTocEntry {
    uint16_t id;
    ParamType type;
    bool readonly;
    // ParamLength length;
    // ParamType type;
    // ParamSign sign;
    // bool readonly;
    // ParamGroup paramGroup;
    std::string group;
    std::string name;
  };

  union ParamValue{
    uint8_t valueUint8;
    int8_t valueInt8;
    uint16_t valueUint16;
    int16_t valueInt16;
    uint32_t valueUint32;
    int32_t valueInt32;
    float valueFloat;
  };

  enum LogType {
    LogTypeUint8  = 1,
    LogTypeUint16 = 2,
    LogTypeUint32 = 3,
    LogTypeInt8   = 4,
    LogTypeInt16  = 5,
    LogTypeInt32  = 6,
    LogTypeFloat  = 7,
    LogTypeFP16   = 8,
  };

  struct LogTocEntry {
    uint16_t id;
    LogType type;
    std::string group;
    std::string name;
  };

#if 0
  enum BootloaderTarget {
    TargetSTM32 = 0xFF,
    TargetNRF51 = 0xFE,
  };
#endif
  enum MemoryType {
    MemoryTypeEEPROM  = 0x00,
    MemoryTypeOneWire = 0x01,
    MemoryTypeLED12   = 0x10,
    MemoryTypeLOCO    = 0x11,
    MemoryTypeTRAJ    = 0x12,
    MemoryTypeLOCO2   = 0x13,
    MemoryTypeLH      = 0x14,
    MemoryTypeTester  = 0x15,
    MemoryTypeUSD     = 0x16,
    MemoryTypeLEDMem  = 0x17,
    MemoryTypeApp     = 0x18,
    MemoryTypeDeckMem = 0x19,
  };

  struct MemoryTocEntry {
    uint16_t id;
    MemoryType type;
    uint32_t size;
    uint64_t addr;
  };

  PACK(struct poly4d {
    float p[4][8];
    float duration;
  });

public:
  Crazyflie(
    const std::string& link_uri,
    Logger& logger = EmptyLogger,
    std::function<void(const char*)> consoleCb = nullptr);

  static std::vector<std::string> scan(
    uint64_t address = 0xE7E7E7E7E7);

  // returns the URI that can be used for broadcast communication (or empty string if there is none)
  static std::string broadcastUriFromUnicastUri(const std::string& link_uri);

  const bitcraze::crazyflieLinkCpp::Connection::Statistics connectionStats() const
  {
    return m_connection.statistics();
  }

  bitcraze::crazyflieLinkCpp::Connection::Statistics connectionStatsDelta()
  {
    return m_connection.statisticsDelta();
  }

  // returns the URI for this Crazyflie
  std::string uri() const;

  // returns the URI that can be used for broadcast communication (or empty string if there is none)
  std::string broadcastUri() const;

  // returns the address for this URI (or -1 if there is none)
  uint64_t address() const;

  int getProtocolVersion();

  std::string getFirmwareVersion();

  std::string getDeviceTypeName();
  void logReset();

  void sendSetpoint(
    float roll,
    float pitch,
    float yawrate,
    uint16_t thrust);

  void sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate);

  void sendVelocityWorldSetpoint(
    float x, float y, float z, float yawRate);

  void sendHoverSetpoint(
    float vx,
    float vy,
    float yawrate,
    float zDistance);

  void sendPositionSetpoint(
    float x,
    float y,
    float z,
    float yaw);

  void notifySetpointsStop(uint32_t remainValidMillisecs);

  void sendStop();

  void emergencyStop();

  void emergencyStopWatchdog();

  void sendExternalPositionUpdate(
    float x,
    float y,
    float z);

  void sendExternalPoseUpdate(
    float x, float y, float z,
    float qx, float qy, float qz, float qw);

  void sendPing();
  void processAllPackets();
  void reboot();
#if 0
  // returns new address
  uint64_t rebootToBootloader();
  void rebootFromBootloader();
#endif
  void sysoff();
  void alloff();
  void syson();
  float vbat();

#if 0
  void writeFlash(
    BootloaderTarget target,
    const std::vector<uint8_t>& data);
  void readFlash(
    BootloaderTarget target,
    size_t size,
    std::vector<uint8_t>& data);

#endif
  void requestLogToc(bool forceNoCache=false);
  void requestParamToc(bool forceNoCache=false, bool requestValues=true);
  void requestParamValues();
  void requestMemoryToc();
  std::vector<ParamTocEntry>::const_iterator paramsBegin() const {
    return m_paramTocEntries.begin();
  }
  std::vector<ParamTocEntry>::const_iterator paramsEnd() const {
    return m_paramTocEntries.end();
  }

  std::vector<LogTocEntry>::const_iterator logVariablesBegin() const {
    return m_logTocEntries.begin();
  }
  std::vector<LogTocEntry>::const_iterator logVariablesEnd() const {
    return m_logTocEntries.end();
  }

  std::vector<MemoryTocEntry>::const_iterator memoriesBegin() const {
    return m_memoryTocEntries.begin();
  }
  std::vector<MemoryTocEntry>::const_iterator memoriesEnd() const {
    return m_memoryTocEntries.end();
  }

  template<class T>
  void setParam(uint16_t id, const T& value) {
    ParamValue v;
    memcpy(&v, &value, sizeof(value));
    setParam(id, v);
  }

  template<class T>
  void setParamByName(const std::string& group, const std::string& name, const T& value) {
    crtpParamSetByNameRequest<T> request(group, name, value);
    m_connection.send(request);
    using res = crtpParamSetByNameResponse;
    auto p = waitForResponse(&res::valid);

    assert( (res::groupAndName(p) == std::make_pair(group, name)) );

    uint8_t error = res::error(p);
    if (error != 0) {
      std::stringstream sstr;
      sstr << "Couldn't set parameter " << group << "." << name << "!";
      if (error == ENOENT) {
        sstr << " No such variable." << std::endl;
      } else if (error == EINVAL) {
        sstr << " Wrong type." << std::endl;
      } else if (error == EACCES) {
        sstr << " Variable is readonly." << std::endl;
      } else {
        sstr << " Error: " << (int)error << std::endl;
      }
      throw std::runtime_error(sstr.str());
    }
  }

  template<class T>
  T getParam(uint16_t id) const {
    ParamValue v = getParam(id);
    return *(reinterpret_cast<T*>(&v));
  }

  const ParamTocEntry* getParamTocEntry(
    const std::string& group,
    const std::string& name) const;

#if 0
  void setEmptyAckCallback(
    std::function<void(const crtpPlatformRSSIAck*)> cb) {
    m_emptyAckCallback = cb;
  }

  void setLinkQualityCallback(
    std::function<void(float)> cb) {
    m_linkQualityCallback = cb;
  }

  void setConsoleCallback(
    std::function<void(const char*)> cb) {
    m_consoleCallback = cb;
  }
#endif
  static size_t size(LogType t) {
    switch(t) {
      case LogTypeUint8:
      case LogTypeInt8:
        return 1;
        break;
      case LogTypeUint16:
      case LogTypeInt16:
      case LogTypeFP16:
        return 2;
        break;
      case LogTypeUint32:
      case LogTypeInt32:
      case LogTypeFloat:
        return 4;
        break;
      default:
        // assert(false);
        return 0;
    }
  }
#if 0

  void setGenericPacketCallback(
    std::function<void(const ITransport::Ack&)> cb) {
    m_genericPacketCallback = cb;
  }

  /**
  * En-queues a generic crtpPacket into a vector so that it can be transmitted later.
  * @param packet the crtpPacket to be en-queued.
  */
  void queueOutgoingPacket(const crtpPacket_t& packet) {
    m_outgoing_packets.push_back(packet);
  }

  void transmitPackets();
#endif

  // High-Level setpoints
  void setGroupMask(uint8_t groupMask);
  void takeoff(float height, float duration, uint8_t groupMask = 0);

  void land(float height, float duration, uint8_t groupMask = 0);

  void stop(uint8_t groupMask = 0);

  void goTo(float x, float y, float z, float yaw, float duration, bool relative = false, uint8_t groupMask = 0);

  void uploadTrajectory(
    uint8_t trajectoryId,
    uint32_t pieceOffset,
    const std::vector<poly4d>& pieces);

  void startTrajectory(
    uint8_t trajectoryId,
    float timescale = 1.0,
    bool reversed = false,
    bool relative = true,
    uint8_t groupMask = 0);

  // Memory subsystem
  void readUSDLogFile(
    std::vector<uint8_t>& data);

  // latency measurements
  void setLatencyCallback(
    std::function<void(uint64_t)> cb) {
    m_latencyCallback = cb;
  }
  void triggerLatencyMeasurement();

private:
  bitcraze::crazyflieLinkCpp::Packet waitForResponse(
      std::function<bool(const bitcraze::crazyflieLinkCpp::Packet&)> condition);

  void processPacket(const bitcraze::crazyflieLinkCpp::Packet& p);

#if 0
  std::vector<crtpPacket_t> m_outgoing_packets;

private:
  struct logInfo {
    uint8_t len;
    uint32_t log_crc;
    uint8_t log_max_packet;
    uint8_t log_max_ops;
  };

  /////////

  struct paramInfo {
    uint8_t len;
    uint32_t crc;
  };

  // enum ParamLength {
  //   ParamLength1Byte  = 0,
  //   ParamLength2Bytes = 1,
  //   ParamLength3Bytes = 2,
  //   ParamLength4Bytes = 3,
  // };

  // enum ParamType {
  //   ParamTypeInt   = 0,
  //   ParamTypeFloat = 1,
  // };

  // enum ParamSign {
  //   ParamSignSigned   = 0,
  //   ParamSignUnsigned = 1,
  // };

  // enum ParamGroup {
  //   ParamGroupVariable = 0,
  //   ParamGroupGroup    = 1,
  // };

#endif

private:
  const LogTocEntry* getLogTocEntry(
    const std::string& group,
    const std::string& name) const;

  uint8_t registerLogBlock(
    std::function<void(const bitcraze::crazyflieLinkCpp::Packet&, uint8_t)> cb);

  bool unregisterLogBlock(
    uint8_t id);

  void setParam(uint16_t id, const ParamValue& value);
  void addSetParam(uint16_t id, const ParamValue& value);

  const ParamValue& getParam(uint16_t id) const {
    return m_paramValues.at(id);
  }
private:
  std::vector<LogTocEntry> m_logTocEntries;
  std::map<uint8_t, std::function<void(const bitcraze::crazyflieLinkCpp::Packet&, uint8_t)>> m_logBlockCb;
  std::vector<ParamTocEntry> m_paramTocEntries;
  std::map<uint16_t, ParamValue> m_paramValues;

  std::vector<MemoryTocEntry> m_memoryTocEntries;

#if 0
  std::function<void(const crtpPlatformRSSIAck*)> m_emptyAckCallback;
  std::function<void(float)> m_linkQualityCallback;
#endif
  std::function<void(const char*)> m_consoleCallback;
#if 0
  std::function<void(const ITransport::Ack&)> m_genericPacketCallback;
#endif
  template<typename T>
  friend class LogBlock;
  friend class LogBlockGeneric;

  int m_protocolVersion;
  // logging
  Logger& m_logger;

  bitcraze::crazyflieLinkCpp::Connection m_connection;

  // latency measurements
  std::chrono::time_point<std::chrono::steady_clock> m_clock_start;
  std::function<void(uint64_t)> m_latencyCallback;
  uint32_t m_latencyCounter;

};

template<class T>
class LogBlock
{
public:
  LogBlock(
    Crazyflie* cf,
    std::list<std::pair<std::string, std::string> > variables,
    std::function<void(uint32_t, const T*)>& callback)
    : m_cf(cf)
    , m_callback(callback)
    , m_id(0)
  {
    m_id = m_cf->registerLogBlock([=](const bitcraze::crazyflieLinkCpp::Packet& p, uint8_t s) { this->handleData(p, s); });
    crtpLogCreateBlockV2Request req(m_id);
    size_t s = 0;
    for (auto&& pair : variables) {
      const Crazyflie::LogTocEntry* entry = m_cf->getLogTocEntry(pair.first, pair.second);
      if (entry) {
        s += Crazyflie::size(entry->type);
        if (s > 26) {
          std::stringstream sstr;
          sstr << "Can't configure that many variables in a single log block!"
                << " Ignoring " << pair.first << "." << pair.second << std::endl;
          throw std::runtime_error(sstr.str());
        } else {
          req.add(entry->type, entry->id);
        }
      } else {
        std::stringstream sstr;
        sstr << "Could not find " << pair.first << "." << pair.second << " in log toc!";
        throw std::runtime_error(sstr.str());
      }
    }
    m_cf->m_connection.send(req);
    using res = crtpLogControlResponse;
    auto p = m_cf->waitForResponse(&res::valid);
    auto result = res::result(p);
    if (result != crtpLogControlResultOk && result != crtpLogControlResultBlockExists)
    {
      throw std::runtime_error("Could not create log block (" + std::to_string((int)result) + ")!");
    }
  }

  ~LogBlock()
  {
    stop();
    m_cf->unregisterLogBlock(m_id);
  }

  void start(uint8_t period)
  {
    crtpLogStartRequest request(m_id, period);
    m_cf->m_connection.send(request);
    using res = crtpLogControlResponse;
    auto p = m_cf->waitForResponse(&res::valid);
    auto result = res::result(p);
    if (result != crtpLogControlResultOk)
    {
      throw std::runtime_error("Could not start log block!");
    }
  }

  void stop()
  {
    crtpLogStopRequest request(m_id);
    m_cf->m_connection.send(request);
    using res = crtpLogControlResponse;
    m_cf->waitForResponse(&res::valid);
    /* intentionally no checking of result */
  }

private:
  void handleData(const bitcraze::crazyflieLinkCpp::Packet &p, uint8_t size)
  {
    using res = crtpLogDataResponse;
    if (size == sizeof(T)) {
      uint32_t time_in_ms = res::timestampMS(p);
      const T* t = reinterpret_cast<const T*>(&p.payload()[4]);
      m_callback(time_in_ms, t);
    } else {
      std::stringstream sstr;
      sstr << "Size doesn't match! Is: " << (size_t)size << " expected: " << sizeof(T);
      throw std::runtime_error(sstr.str());
    }
  }

private:
  Crazyflie* m_cf;
  std::function<void(uint32_t, const T*)> m_callback;
  uint8_t m_id;
};
///

class LogBlockGeneric
{
public:
  LogBlockGeneric(
    Crazyflie* cf,
    const std::vector<std::string>& variables,
    void* userData,
    std::function<void(uint32_t, const std::vector<float>*, void* userData)>& callback)
    : m_cf(cf)
    , m_userData(userData)
    , m_callback(callback)
    , m_id(0)
  {
    m_id = m_cf->registerLogBlock([=](const bitcraze::crazyflieLinkCpp::Packet& p, uint8_t s) { this->handleData(p, s); });
    crtpLogCreateBlockV2Request req(m_id);
    int i = 0;
    size_t s = 0;
    for (auto&& var : variables) {
      auto pos = var.find(".");
      std::string first = var.substr(0, pos);
      std::string second = var.substr(pos+1);
      const Crazyflie::LogTocEntry* entry = m_cf->getLogTocEntry(first, second);
      if (entry) {
        s += Crazyflie::size(entry->type);
        if (s > 26) {
          std::stringstream sstr;
          sstr << "Can't configure that many variables in a single log block!"
                << " Ignoring " << first << "." << second << std::endl;
          throw std::runtime_error(sstr.str());
        } else {
          if (i < 9) {
            req.add(entry->type, entry->id);
            ++i;
            m_types.push_back(entry->type);
          } else {
            std::stringstream sstr;
            sstr << "Can only log up to 9 variables at a time!"
                  << " Ignoring " << first << "." << second << std::endl;
            throw std::runtime_error(sstr.str());
          }
        }
      }
      else {
        std::stringstream sstr;
        sstr << "Could not find " << first << "." << second << " in log toc!";
        throw std::runtime_error(sstr.str());
      }
    }
    m_cf->m_connection.send(req);
    using res = crtpLogControlResponse;
    auto p = m_cf->waitForResponse(&res::valid);
    auto result = res::result(p);
    if (result != crtpLogControlResultOk
        && result != crtpLogControlResultBlockExists) {
      throw std::runtime_error("Could not create log block (" + std::to_string((int)result) + ")!");
    }
  }

  ~LogBlockGeneric()
  {
    stop();
    m_cf->unregisterLogBlock(m_id);
  }


  void start(uint8_t period)
  {
    crtpLogStartRequest request(m_id, period);
    m_cf->m_connection.send(request);
    using res = crtpLogControlResponse;
    auto p = m_cf->waitForResponse(&res::valid);
    auto result = res::result(p);
    if (result != crtpLogControlResultOk)
    {
      throw std::runtime_error("Could not start log block!");
    }
  }

  void stop()
  {
    crtpLogStopRequest request(m_id);
    m_cf->m_connection.send(request);
    using res = crtpLogControlResponse;
    m_cf->waitForResponse(&res::valid);
    /* intentionally no checking of result */
  }

private:
  void handleData(const bitcraze::crazyflieLinkCpp::Packet &p, uint8_t /*size*/)
  {
    using res = crtpLogDataResponse;
    std::vector<float> result;
    size_t pos = 0;
    for (size_t i = 0; i < m_types.size(); ++i)
    {
      switch (m_types[i])
      {
      case Crazyflie::LogTypeUint8:
        {
          uint8_t value = res::variableAt<uint8_t>(p, pos);
          result.push_back(value);
          pos += sizeof(uint8_t);
          break;
        }
      case Crazyflie::LogTypeInt8:
        {
          int8_t value = res::variableAt<int8_t>(p, pos);
          result.push_back(value);
          pos += sizeof(int8_t);
          break;
        }
      case Crazyflie::LogTypeUint16:
        {
          uint16_t value = res::variableAt<uint16_t>(p, pos);
          result.push_back(value);
          pos += sizeof(uint16_t);
          break;
        }
      case Crazyflie::LogTypeInt16:
        {
          int16_t value = res::variableAt<int16_t>(p, pos);
          result.push_back(value);
          pos += sizeof(int16_t);
          break;
        }
      case Crazyflie::LogTypeUint32:
        {
          uint32_t value = res::variableAt<uint32_t>(p, pos);
          result.push_back(value);
          pos += sizeof(uint32_t);
          break;
        }
      case Crazyflie::LogTypeInt32:
        {
          int32_t value = res::variableAt<int32_t>(p, pos);
          result.push_back(value);
          pos += sizeof(int32_t);
          break;
        }
      case Crazyflie::LogTypeFloat:
        {
          float value = res::variableAt<float>(p, pos);
          result.push_back(value);
          pos += sizeof(float);
          break;
        }
      case Crazyflie::LogTypeFP16:
        {
          /* TODO: not yet supported */
          pos += 2;
          break;
        }
      }
    }

    uint32_t time_in_ms = res::timestampMS(p);
    m_callback(time_in_ms, &result, m_userData);
  }

private:
  Crazyflie* m_cf;
  void* m_userData;
  std::function<void(uint32_t, const std::vector<float>*, void*)> m_callback;
  uint8_t m_id;
  std::vector<Crazyflie::LogType> m_types;
};
///

class CrazyflieBroadcaster
{
public:
  CrazyflieBroadcaster(
    const std::string& link_uri);

  const std::string& uri() const
  {
    return m_connection.uri();
  }

  const bitcraze::crazyflieLinkCpp::Connection::Statistics connectionStats() const
  {
    return m_connection.statistics();
  }

  bitcraze::crazyflieLinkCpp::Connection::Statistics connectionStatsDelta()
  {
    return m_connection.statisticsDelta();
  }

  // High-Level setpoints
  void takeoff(float height, float duration, uint8_t groupMask = 0);

  void land(float height, float duration, uint8_t groupMask = 0);

  void stop(uint8_t groupMask = 0);

  // This is always in relative coordinates
  void goTo(float x, float y, float z, float yaw, float duration, uint8_t groupMask = 0);

  // This is always in relative coordinates
  void startTrajectory(
    uint8_t trajectoryId,
    float timescale = 1.0,
    bool reversed = false,
    uint8_t groupMask = 0);

  void notifySetpointsStop(uint32_t remainValidMillisecs);

  struct externalPosition
  {
    uint8_t id;
    float x;
    float y;
    float z;
  };

  void sendExternalPositions(
    const std::vector<externalPosition>& data);

  struct externalPose
  {
    uint8_t id;
    float x;
    float y;
    float z;
    float qx;
    float qy;
    float qz;
    float qw;
  };

  void sendExternalPoses(
    const std::vector<externalPose>& data);

  void emergencyStop();

  void emergencyStopWatchdog();

  template<class T>
  void setParam(
    const char* group,
    const char* name,
    const T& value)
  {
    crtpParamSetByNameRequest<T> req(group, name, value);
    m_connection.send(req);
  }

  void sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate);

private:
  bitcraze::crazyflieLinkCpp::Connection m_connection;
};
