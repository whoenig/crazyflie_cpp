#pragma once

#include <cstring>
#include <sstream>
#include <functional>
#include <math.h>

#include "Crazyradio.h"
#include "crtp.h"
#include <list>
#include <set>
#include <map>
#include <chrono>

#define ENABLE_SAFELINK 0

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

  enum BootloaderTarget {
    TargetSTM32 = 0xFF,
    TargetNRF51 = 0xFE,
  };

  enum MemoryType {
    MemoryTypeEEPROM  = 0x00,
    MemoryTypeOneWire = 0x01,
    MemoryTypeLED12   = 0x10,
    MemoryTypeLOCO    = 0x11,
    MemoryTypeTRAJ    = 0x12,
    MemoryTypeLOCO2   = 0x13,
    MemoryTypeLH      = 0x14,
    MemoryTypeTester  = 0x15,
    MemoryTypeUSD     = 0x16, // Crazyswarm experimental
  };

  struct MemoryTocEntry {
    uint16_t id;
    MemoryType type;
    uint32_t size;
    uint64_t addr;
  };

  struct poly4d {
    float p[4][8];
    float duration;
  } __attribute__((packed));

public:
  Crazyflie(
    const std::string& link_uri,
    Logger& logger = EmptyLogger,
    std::function<void(const char*)> consoleCb = nullptr);

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

  void reboot();
  // returns new address
  uint64_t rebootToBootloader();

  void rebootFromBootloader();

  void sysoff();
  void alloff();
  void syson();
  float vbat();

  void writeFlash(
    BootloaderTarget target,
    const std::vector<uint8_t>& data);
  void readFlash(
    BootloaderTarget target,
    size_t size,
    std::vector<uint8_t>& data);

  void requestLogToc(bool forceNoCache=false);

  void requestParamToc(bool forceNoCache=false);

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
  void setParamByName(const char* group, const char* name, const T& value) {
    crtpParamSetByNameRequest<T> request(group, name, value);
    // sendPacketOrTimeoutInternal(reinterpret_cast<const uint8_t*>(&request), request.size());
    startBatchRequest();
    addRequestInternal(
      reinterpret_cast<const uint8_t*>(&request), request.size(), request.responseSize() - 1);
    handleRequests();
    auto response = getRequestResult<crtpParamSetByNameResponse>(0);

    uint8_t error = response->error(request.responseSize());
    if (error != 0) {
      std::stringstream sstr;
      sstr << "Couldn't set parameter " << group << "." << name << "!";
      if (error == ENOENT) {
        sstr << "No such variable." << std::endl;
      } else if (error == EINVAL) {
        sstr << "Wrong type." << std::endl;
      } else if (error == EACCES) {
        sstr << "Variable is readonly." << std::endl;
      } else {
        sstr << " Error: " << (int)error << std::endl;
      }
      throw std::runtime_error(sstr.str());
    }
  }

  void startSetParamRequest();

  template<class T>
  void addSetParam(uint16_t id, const T& value) {
    ParamValue v;
    memcpy(&v, &value, sizeof(value));
    addSetParam(id, v);
  }

  void setRequestedParams();


  template<class T>
  T getParam(uint16_t id) const {
    ParamValue v = getParam(id);
    return *(reinterpret_cast<T*>(&v));
  }

  const ParamTocEntry* getParamTocEntry(
    const std::string& group,
    const std::string& name) const;

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

private:
  void sendPacketInternal(
    const uint8_t* data,
    uint32_t length,
    ITransport::Ack& result,
    bool useSafeLink = ENABLE_SAFELINK);

  template<typename R>
  void sendPacket(
    const R& request,
    ITransport::Ack& result,
    bool useSafeLink = ENABLE_SAFELINK)
  {
    sendPacketInternal(
      reinterpret_cast<const uint8_t*>(&request), sizeof(request), result, useSafeLink);
  }

  bool sendPacketInternal(
    const uint8_t* data,
    uint32_t length,
    bool useSafeLink = ENABLE_SAFELINK);

  template<typename R>
  void sendPacket(
    const R& request,
    bool useSafeLink = ENABLE_SAFELINK)
  {
    sendPacketInternal(
      reinterpret_cast<const uint8_t*>(&request), sizeof(request), useSafeLink);
  }

 void sendPacketOrTimeoutInternal(
   const uint8_t* data,
   uint32_t length,
   bool useSafeLink = ENABLE_SAFELINK,
   float timeout = 1.0);

  template<typename R>
  void sendPacketOrTimeout(
    const R& request,
    bool useSafeLink = ENABLE_SAFELINK)
  {
    sendPacketOrTimeoutInternal(
      reinterpret_cast<const uint8_t*>(&request), sizeof(request), useSafeLink);
  }

  void handleAck(
    const ITransport::Ack& result);

  std::vector<crtpPacket_t> m_outgoing_packets;

  void startBatchRequest();

  void addRequestInternal(
    const uint8_t* data,
    size_t numBytes,
    size_t numBytesToMatch);

  template<typename R>
  void addRequest(
    const R& request,
    size_t numBytesToMatch)
  {
    addRequestInternal(
      reinterpret_cast<const uint8_t*>(&request), sizeof(request), numBytesToMatch);
  }

  void handleRequests(
    bool crtpMode = true,
    bool useSafeLink = ENABLE_SAFELINK,
    float baseTime = 2.0,
    float timePerRequest = 0.2);


  void handleBatchAck(
    const ITransport::Ack& result,
    bool crtpMode);

  template<typename R>
  const R* getRequestResult(size_t index) const {
    return reinterpret_cast<const R*>(m_batchRequests[index].ack.data);
  }

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


private:
  const LogTocEntry* getLogTocEntry(
    const std::string& group,
    const std::string& name) const;

  uint8_t registerLogBlock(
    std::function<void(crtpLogDataResponse*, uint8_t)> cb);

  bool unregisterLogBlock(
    uint8_t id);

  void setParam(uint16_t id, const ParamValue& value);
  void addSetParam(uint16_t id, const ParamValue& value);


  const ParamValue& getParam(uint16_t id) const {
    return m_paramValues.at(id);
  }

private:
  Crazyradio* m_radio;
  ITransport* m_transport;
  int m_devId;

  uint8_t m_channel;
  uint64_t m_address;
  Crazyradio::Datarate m_datarate;

  std::vector<LogTocEntry> m_logTocEntries;
  std::map<uint8_t, std::function<void(crtpLogDataResponse*, uint8_t)> > m_logBlockCb;

  std::vector<ParamTocEntry> m_paramTocEntries;
  std::map<uint16_t, ParamValue> m_paramValues;

  std::vector<MemoryTocEntry> m_memoryTocEntries;

  std::function<void(const crtpPlatformRSSIAck*)> m_emptyAckCallback;
  std::function<void(float)> m_linkQualityCallback;
  std::function<void(const char*)> m_consoleCallback;
  std::function<void(const ITransport::Ack&)> m_genericPacketCallback;

  template<typename T>
  friend class LogBlock;
  friend class LogBlockGeneric;

  // batch system
  struct batchRequest
  {
    std::vector<uint8_t> request;
    size_t numBytesToMatch;
    ITransport::Ack ack;
    bool finished;
  };
  std::vector<batchRequest> m_batchRequests;
  size_t m_numRequestsFinished;
  size_t m_numRequestsEnqueued;

  int m_curr_up;
  int m_curr_down;

  bool m_log_use_V2;
  bool m_param_use_V2;

  int m_protocolVersion;

  // logging
  Logger& m_logger;
};

template<class T>
class LogBlock
{
public:
  LogBlock(
    Crazyflie* cf,
    std::list<std::pair<std::string, std::string> > variables,
    std::function<void(uint32_t, T*)>& callback)
    : m_cf(cf)
    , m_callback(callback)
    , m_id(0)
  {
    m_id = m_cf->registerLogBlock([=](crtpLogDataResponse* r, uint8_t s) { this->handleData(r, s);});
    if (m_cf->m_log_use_V2) {
      std::vector<logBlockItemV2> logBlockItems;
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
            logBlockItems.push_back({static_cast<uint8_t>(entry->type), entry->id});
          }
        }
        else {
          std::stringstream sstr;
          sstr << "Could not find " << pair.first << "." << pair.second << " in log toc!";
          throw std::runtime_error(sstr.str());
        }
      }

      // we can use up to 9 items per request
      size_t requests = ceil(logBlockItems.size() / 9.0f);
      size_t i = 0;
      for (size_t r = 0; r < requests; ++r) {
        size_t numElements = std::min<size_t>(logBlockItems.size() - i, 9);
        if (r == 0) {
          crtpLogCreateBlockV2Request request;
          request.id = m_id;
          memcpy(request.items, &logBlockItems[i], sizeof(logBlockItemV2) * numElements);
          m_cf->sendPacketOrTimeoutInternal(reinterpret_cast<const uint8_t*>(&request), 3 + 3*numElements);
        } else {
          crtpLogAppendBlockV2Request request;
          request.id = m_id;
          memcpy(request.items, &logBlockItems[i], sizeof(logBlockItemV2) * numElements);
          m_cf->sendPacketOrTimeoutInternal(reinterpret_cast<const uint8_t*>(&request), 3 + 3*numElements);
        }
        i += numElements;
      }
      // auto r = m_cf->getRequestResult<crtpLogControlResponse>(0);
      // if (r->result != crtpLogControlResultOk
      //     && r->result != crtpLogControlResultBlockExists) {
      //   std::stringstream sstr;
      //   sstr << "Could not create log block! Result: " << (int)r->result << " " << (int)m_id << " " << (int)r->requestByte1 << " " << (int)r->command;
      //   throw std::runtime_error(sstr.str());
      // }
    } else {
      crtpLogCreateBlockRequest request;
      request.id = m_id;
      int i = 0;
      for (auto&& pair : variables) {
        const Crazyflie::LogTocEntry* entry = m_cf->getLogTocEntry(pair.first, pair.second);
        if (entry) {
          request.items[i].logType = entry->type;
          request.items[i].id = entry->id;
          ++i;
        }
        else {
          std::stringstream sstr;
          sstr << "Could not find " << pair.first << "." << pair.second << " in log toc!";
          throw std::runtime_error(sstr.str());
        }
      }

      m_cf->startBatchRequest();
      m_cf->addRequestInternal(reinterpret_cast<const uint8_t*>(&request), 3 + 2*i, 2);
      m_cf->handleRequests();
      auto r = m_cf->getRequestResult<crtpLogControlResponse>(0);
      if (r->result != crtpLogControlResultOk
          && r->result != crtpLogControlResultBlockExists) {
        std::stringstream sstr;
        sstr << "Could not create log block! Result: " << (int)r->result << " " << (int)m_id << " " << (int)r->requestByte1 << " " << (int)r->command;
        throw std::runtime_error(sstr.str());
      }
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
    m_cf->startBatchRequest();
    m_cf->addRequest(request, 2);
    m_cf->handleRequests();
  }

  void stop()
  {
    crtpLogStopRequest request(m_id);
    m_cf->startBatchRequest();
    m_cf->addRequest(request, 2);
    m_cf->handleRequests();
  }

private:
  void handleData(crtpLogDataResponse* response, uint8_t size) {
    if (size == sizeof(T)) {
      uint32_t time_in_ms = ((uint32_t)response->timestampHi << 8) | (response->timestampLo);
      T* t = (T*)response->data;
      m_callback(time_in_ms, t);
    }
    else {
      std::stringstream sstr;
      sstr << "Size doesn't match! Is: " << (size_t)size << " expected: " << sizeof(T);
      throw std::runtime_error(sstr.str());
    }
  }

private:
  Crazyflie* m_cf;
  std::function<void(uint32_t, T*)> m_callback;
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
    std::function<void(uint32_t, std::vector<double>*, void* userData)>& callback)
    : m_cf(cf)
    , m_userData(userData)
    , m_callback(callback)
    , m_id(0)
  {
    m_id = m_cf->registerLogBlock([=](crtpLogDataResponse* r, uint8_t s) { this->handleData(r, s);});
    if (m_cf->m_log_use_V2) {
      crtpLogCreateBlockV2Request request;
      request.id = m_id;
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
              request.items[i].logType = entry->type;
              request.items[i].id = entry->id;
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
      m_cf->startBatchRequest();
      m_cf->addRequestInternal(reinterpret_cast<const uint8_t*>(&request), 3 + 3*i, 2);
      m_cf->handleRequests();
      auto r = m_cf->getRequestResult<crtpLogControlResponse>(0);
      if (r->result != crtpLogControlResultOk
          && r->result != crtpLogControlResultBlockExists) {
        throw std::runtime_error("Could not create log block!");
      }
    } else {
      crtpLogCreateBlockRequest request;
      request.id = m_id;
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
            request.items[i].logType = entry->type;
            request.items[i].id = entry->id;
            ++i;
            m_types.push_back(entry->type);
          }
        }
        else {
          std::stringstream sstr;
          sstr << "Could not find " << first << "." << second << " in log toc!";
          throw std::runtime_error(sstr.str());
        }
      }
      m_cf->startBatchRequest();
      m_cf->addRequestInternal(reinterpret_cast<const uint8_t*>(&request), 3 + 2*i, 2);
      m_cf->handleRequests();
      auto r = m_cf->getRequestResult<crtpLogControlResponse>(0);
      if (r->result != crtpLogControlResultOk
          && r->result != crtpLogControlResultBlockExists) {
        throw std::runtime_error("Could not create log block!");
      }
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
    m_cf->startBatchRequest();
    m_cf->addRequest(request, 2);
    m_cf->handleRequests();
  }

  void stop()
  {
    crtpLogStopRequest request(m_id);
    m_cf->startBatchRequest();
    m_cf->addRequest(request, 2);
    m_cf->handleRequests();
  }

private:
  void handleData(crtpLogDataResponse* response, uint8_t /*size*/) {

    std::vector<double> result;
    size_t pos = 0;
    for (size_t i = 0; i < m_types.size(); ++i)
    {
      switch (m_types[i])
      {
      case Crazyflie::LogTypeUint8:
        {
          uint8_t value;
          memcpy(&value, &response->data[pos], sizeof(uint8_t));
          result.push_back(value);
          pos += sizeof(uint8_t);
          break;
        }
      case Crazyflie::LogTypeInt8:
        {
          int8_t value;
          memcpy(&value, &response->data[pos], sizeof(int8_t));
          result.push_back(value);
          pos += sizeof(int8_t);
          break;
        }
      case Crazyflie::LogTypeUint16:
        {
          uint16_t value;
          memcpy(&value, &response->data[pos], sizeof(uint16_t));
          result.push_back(value);
          pos += sizeof(uint16_t);
          break;
        }
      case Crazyflie::LogTypeInt16:
        {
          int16_t value;
          memcpy(&value, &response->data[pos], sizeof(int16_t));
          result.push_back(value);
          pos += sizeof(int16_t);
          break;
        }
      case Crazyflie::LogTypeUint32:
        {
          uint32_t value;
          memcpy(&value, &response->data[pos], sizeof(uint32_t));
          result.push_back(value);
          pos += sizeof(uint32_t);
          break;
        }
      case Crazyflie::LogTypeInt32:
        {
          int32_t value;
          memcpy(&value, &response->data[pos], sizeof(int32_t));
          result.push_back(value);
          pos += sizeof(int32_t);
          break;
        }
      case Crazyflie::LogTypeFloat:
        {
          float value;
          memcpy(&value, &response->data[pos], sizeof(float));
          result.push_back(value);
          pos += sizeof(float);
          break;
        }
      case Crazyflie::LogTypeFP16:
        {
          double value;
          memcpy(&value, &response->data[pos], sizeof(double));
          result.push_back(value);
          pos += sizeof(double);
          break;
        }
      }
    }

    uint32_t time_in_ms = ((uint32_t)response->timestampHi << 8) | (response->timestampLo);
    m_callback(time_in_ms, &result, m_userData);
  }

private:
  Crazyflie* m_cf;
  void* m_userData;
  std::function<void(uint32_t, std::vector<double>*, void*)> m_callback;
  uint8_t m_id;
  std::vector<Crazyflie::LogType> m_types;
};

///

class CrazyflieBroadcaster
{
public:
  CrazyflieBroadcaster(
    const std::string& link_uri);

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
    crtpParamSetByNameRequest<T> request(group, name, value);
    sendPacket(reinterpret_cast<const uint8_t*>(&request), request.size());
  }

protected:
  void sendPacket(
    const uint8_t* data,
    uint32_t length);

  void send2Packets(
    const uint8_t* data,
    uint32_t length);

  // void setParam(
  //   uint8_t group,
  //   uint8_t id,
  //   Crazyflie::ParamType type,
  //   const Crazyflie::ParamValue& value);

private:
  Crazyradio* m_radio;
  int m_devId;

  uint8_t m_channel;
  uint64_t m_address;
  Crazyradio::Datarate m_datarate;
};
