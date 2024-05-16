//#include <regex>
#include <mutex>
#include <cassert>

#include "Crazyflie.h"
#include "crtp.h"
#include "crtpBootloader.h"
#include "crtpNRF51.h"

#include <fstream>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <inttypes.h>
#include <regex>

#define FIRMWARE_BUGGY

Logger EmptyLogger;


Crazyflie::Crazyflie(
  const std::string& link_uri,
  Logger& logger,
  std::function<void(const char*)> consoleCb)
  // : m_logTocEntries()
  // , m_logBlockCb()
  // , m_paramTocEntries()
  // , m_paramValues()
  // , m_emptyAckCallback(nullptr)
  // , m_linkQualityCallback(nullptr)
  : m_consoleCallback(consoleCb)
  // , m_log_use_V2(false)
  , m_protocolVersion(-1)
  , m_logger(logger)
  , m_connection(link_uri)
  , m_clock_start(std::chrono::steady_clock::now())
  , m_latencyCounter(0)
{
}

std::vector<std::string> Crazyflie::scan(
    uint64_t address)
{
  return bitcraze::crazyflieLinkCpp::Connection::scan(address);
}

std::string Crazyflie::broadcastUriFromUnicastUri(
  const std::string& link_uri)
{
  const std::regex uri_regex("radio:\\/\\/(\\d+|\\*)\\/(\\d+)\\/(250K|1M|2M)\\/([a-fA-F0-9]+)");
  std::smatch match;
  if (!std::regex_match(link_uri, match, uri_regex))
  {
    // unsupported for broadcast
    return std::string();
  }

  return "radiobroadcast://*/" + match[2].str() + "/" + match[3].str();
}

std::string Crazyflie::uri() const
{
  return m_connection.uri();
}

std::string Crazyflie::broadcastUri() const
{
  return Crazyflie::broadcastUriFromUnicastUri(uri());
}

uint64_t Crazyflie::address() const
{
  const std::regex uri_regex("radio:\\/\\/(\\d+|\\*)\\/(\\d+)\\/(250K|1M|2M)\\/([a-fA-F0-9]+)");
  std::smatch match;
  if (!std::regex_match(m_connection.uri(), match, uri_regex))
  {
    return -1;
  }
  return std::stoull(match[4].str(), nullptr, 16);
}

int Crazyflie::getProtocolVersion()
{
  crtpGetProtocolVersionRequest req;
  m_connection.send(req);
  using res = crtpGetProtocolVersionResponse;
  auto p = waitForResponse(&res::valid);
  return res::version(p);
}

std::string Crazyflie::getFirmwareVersion()
{
  crtpGetFirmwareVersionRequest req;
  m_connection.send(req);
  using res = crtpGetFirmwareVersionResponse;
  auto p = waitForResponse(&res::valid);
  return res::version(p);
}

std::string Crazyflie::getDeviceTypeName()
{
  crtpGetDeviceTypeNameRequest req;
  m_connection.send(req);
  using res = crtpGetDeviceTypeNameResponse;
  auto p = waitForResponse(&res::valid);
  return res::name(p);
}

void Crazyflie::sendArmingRequest(bool arm)
{
  crtpArmingRequest req(arm);
  m_connection.send(req);
}

void Crazyflie::logReset()
{
  crtpLogResetRequest request;
  m_connection.send(request);
  using res = crtpLogControlResponse;
  auto p = waitForResponse(&res::valid);
  auto result = res::result(p);
  if (result != crtpLogControlResultOk)
  {
    throw std::runtime_error("Could not start log block!");
  }
}

void Crazyflie::sendSetpoint(
  float roll,
  float pitch,
  float yawrate,
  uint16_t thrust)
{
  crtpSetpointRequest req(roll, pitch, yawrate, thrust);
  m_connection.send(req);
}

void Crazyflie::sendStop()
{
  crtpStopRequest req;
  m_connection.send(req);
}

void Crazyflie::emergencyStop()
{
  crtpEmergencyStopRequest req;
  m_connection.send(req);
}

void Crazyflie::emergencyStopWatchdog()
{
  crtpEmergencyStopWatchdogRequest req;
  m_connection.send(req);
}

void Crazyflie::sendPositionSetpoint(
  float x,
  float y,
  float z,
  float yaw)
{
  crtpPositionSetpointRequest req(x, y, z, yaw);
  m_connection.send(req);
}

void Crazyflie::sendHoverSetpoint(
  float vx,
  float vy,
  float yawrate,
  float zDistance)
{
  crtpHoverSetpointRequest req(vx, vy, yawrate, zDistance);
  m_connection.send(req);
}

void Crazyflie::sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate)
{
  crtpFullStateSetpointRequest req(
      x, y, z,
      vx, vy, vz,
      ax, ay, az,
      qx, qy, qz, qw,
      rollRate, pitchRate, yawRate);
  m_connection.send(req);
}

void Crazyflie::sendVelocityWorldSetpoint(
        float x, float y, float z, float yawRate)
{
  crtpVelocityWorldSetpointRequest req(
      x, y, z, yawRate);
  m_connection.send(req);
}

void Crazyflie::notifySetpointsStop(uint32_t remainValidMillisecs)
{
  crtpNotifySetpointsStopRequest req(remainValidMillisecs);
  m_connection.send(req);
}

#if 0
void Crazyflie::sendExternalPositionUpdate(
  float x,
  float y,
  float z)
{
  crtpExternalPositionUpdate position(x, y, z);
  sendPacket(position);
}

void Crazyflie::sendExternalPoseUpdate(
  float x, float y, float z,
  float qx, float qy, float qz, float qw)
{
  crtpExternalPoseUpdate pose(x, y, z, qx, qy, qz, qw);
  sendPacket(pose);
}
#endif
void Crazyflie::sendPing()
{
  auto p = m_connection.recv(1);
  if (p.valid()) {
    processPacket(p);
  }
}

void Crazyflie::processAllPackets()
{
  while (true) {
    auto p = m_connection.receive(bitcraze::crazyflieLinkCpp::Connection::TimeoutNone);
    if (p.valid()) {
      processPacket(p);
    } else {
      break;
    }
  }
}

#if 0
/**
 * Transmits any outgoing packets to the crazyflie.
 */
void Crazyflie::transmitPackets()
{
  if (!m_outgoing_packets.empty())
  {
    std::vector<crtpPacket_t>::iterator it;
    for (it = m_outgoing_packets.begin(); it != m_outgoing_packets.end(); it++)
    {
      sendPacketInternal(it->raw, it->size+1);
    }
    m_outgoing_packets.clear();
  }
}
#endif
// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  // m_connection.close();
  // bitcraze::crazyflieLinkCpp::Connection m_connectionBootloader("radio://0/80/2M/E7E7E7E7E7?safelink=0&autoping=0&ackfilter=0");

  crtpNrf51ResetInitRequest req1;
  m_connection.send(req1);
  crtpNrf51ResetRequest req2(/*bootToFirmware*/ 1);
  m_connection.send(req2);

  while (m_connection.statistics().enqueued_count > 0)
    ;

  // while(true) {
  //   std::cout << m_connectionBootloader.recv(0) << std::endl;
  // }

  // crtpNrf51ResetInitRequest req1;
  // m_connection.send(req1);

  // while(true) {
  //   std::cout << m_connection.recv(0) << std::endl;
  // }
}
#if 0
uint64_t Crazyflie::rebootToBootloader()
{
  if (m_radio) {
    crtpNrf51ResetInitRequest req;
    startBatchRequest();
    addRequest(req, 2);
    handleRequests();
    const crtpNrf51ResetInitResponse* response = getRequestResult<crtpNrf51ResetInitResponse>(0);

    uint64_t result =
        ((uint64_t)response->addr[0] << 0)
      | ((uint64_t)response->addr[1] << 8)
      | ((uint64_t)response->addr[2] << 16)
      | ((uint64_t)response->addr[3] << 24)
      | ((uint64_t)0xb1 << 32);

    crtpNrf51ResetRequest req2(/*bootToFirmware*/ 0);
    sendPacketOrTimeout(req2);

    // switch to new address
    m_address = result;
    m_channel = 0;
    m_datarate = Crazyradio::Datarate_2MPS;


    return result;
  } else {
    return -1;
  }
}

void Crazyflie::rebootFromBootloader()
{
  if (m_radio) {
    bootloaderResetRequest req(/*bootToFirmware*/ 1);
    sendPacketOrTimeout(req, /*useSafeLink*/ false);
  }
}
#endif
void Crazyflie::sysoff()
{
  // m_connection.close();
  // bitcraze::crazyflieLinkCpp::Connection m_connectionBootloader("radio://0/80/2M/E7E7E7E7E7?safelink=0&autoping=0&ackfilter=0");
  crtpNrf51SysOffRequest req;
  m_connection.send(req);

  while (m_connection.statistics().enqueued_count > 0)
    ;
  // while (true)
  // {
  //   std::cout << m_connection.statistics() << m_connection.recv(0) << std::endl;
  // }
}

void Crazyflie::alloff()
{
  crtpNrf51AllOffRequest req;
  m_connection.send(req);

  while (m_connection.statistics().enqueued_count > 0)
    ;
}

void Crazyflie::syson()
{
  crtpNrf51SysOnRequest req;
  m_connection.send(req);

  while (m_connection.statistics().enqueued_count > 0) ;
  // while (true)
  // {
  //   std::cout << m_connection.statistics() << m_connection.recv(0) << std::endl;
  // }
}

float Crazyflie::vbat()
{
  crtpNrf51GetVBatRequest req;
  m_connection.send(req);
  using res = crtpNrf51GetVBatResponse;
  auto p = waitForResponse(&res::valid);
  return res::vbat(p);
}

#if 0
void Crazyflie::writeFlash(
  BootloaderTarget target,
  const std::vector<uint8_t>& data)
{
  // Get info about the target
  bootloaderGetInfoRequest req(target);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false, /*useSafeLink*/ false);
  const bootloaderGetInfoResponse* response = getRequestResult<bootloaderGetInfoResponse>(0);
  uint16_t pageSize = response->pageSize;
  uint16_t flashStart = response->flashStart;
  uint16_t nBuffPage = response->nBuffPage;

  uint16_t numPages = ceil(data.size() / (float)pageSize);
  if (numPages + flashStart >= response->nFlashPage) {
    std::stringstream sstr;
    sstr << "Requested size too large!";
    throw std::runtime_error(sstr.str());
  }

  std::stringstream sstr;
  sstr << "pageSize: " << pageSize
            << " nBuffPage: " << nBuffPage
            << " nFlashPage: " << response->nFlashPage
            << " flashStart: " << flashStart
            << " version: " << (int)response->version
            << " numPages: " << numPages;
  m_logger.info(sstr.str());

  // write flash
  size_t offset = 0;
  uint16_t usedBuffers = 0;
  // startBatchRequest();
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    std::stringstream sstr;
    sstr << "page: " << page - flashStart + 1 << " / " << numPages;
    m_logger.info(sstr.str());
    for (uint16_t address = 0; address < pageSize; address += 25) {

      // std::cout << "request: " << page << " " << address << std::endl;
      bootloaderLoadBufferRequest req(target, usedBuffers, address);
      size_t requestedSize = std::min<size_t>(data.size() - offset, std::min<size_t>(25, pageSize - address));
      memcpy(req.data, &data[offset], requestedSize);
      // addRequest(req, 0);
      // for (size_t i = 0; i < 10; ++i)
      // std::cout << "request: " << req.page << " " << req.address << " " << requestedSize << std::endl;
      // for (size_t i = 0; i < 10; ++i) {

      // auto start = std::chrono::system_clock::now();
      // while (true) {
        sendPacketOrTimeoutInternal((uint8_t*)&req, 7 + requestedSize, false);
      //   startBatchRequest();
      //   bootloaderReadBufferRequest req2(target, usedBuffers, address);
      //   addRequest(req2, 7);
      //   handleRequests(/*crtpMode=*/false);
      //   const bootloaderReadBufferResponse* response = getRequestResult<bootloaderReadBufferResponse>(0);
      //   if (memcmp(req.data, response->data, requestedSize) == 0) {
      //     break;
      //   }
      //   auto end = std::chrono::system_clock::now();
      //   std::chrono::duration<double> elapsedSeconds = end-start;
      //   if (elapsedSeconds.count() > 1.0) {
      //     throw std::runtime_error("timeout");
      //   }
      // }
      offset += requestedSize;
      if (offset >= data.size()) {
        break;
      }
    }
    ++usedBuffers;
    if (usedBuffers == nBuffPage
        || page == numPages + flashStart - 1) {


      // startBatchRequest();
      // for (uint16_t buf = 0; buf < usedBuffers; ++buf) {
      //   for (uint16_t address = 0; address < pageSize; address += 25) {
      //     // std::cout << "request: " << page << " " << address << std::endl;
      //     bootloaderReadBufferRequest req(target, buf, address);
      //     addRequest(req, 7);
      //   }
      // }
      // handleRequests(/*crtpMode=*/false);


      // upload all the buffers now
      // std::cout << "try to upload buffers!" << std::endl;
      // handleRequests(/*crtpMode=*/false);
      // std::cout << "buffers uploaded!" << std::endl;

      // std::this_thread::sleep_for(std::chrono::milliseconds(100));

      // write flash
      bootloaderWriteFlashRequest req(target, 0, page - usedBuffers + 1, usedBuffers);
      sendPacketOrTimeoutInternal((uint8_t*)&req, sizeof(req), false);

      auto start = std::chrono::system_clock::now();

      size_t tries = 0;
      while (true) {
        ITransport::Ack ack;
        bootloaderFlashStatusRequest statReq(target);
        sendPacket(statReq, ack, false);
        if (   ack.ack
            && ack.size == 5
            && memcmp(&req, ack.data, 3) == 0) {
          if (ack.data[3] != 1 || ack.data[4] != 0) {
            throw std::runtime_error("Error during flashing!");
          }
          break;
        }

        // std::cout << page - usedBuffers + 1 << "," << usedBuffers << std::endl;
        // startBatchRequest();
        // bootloaderWriteFlashRequest req(target, 0, page - usedBuffers + 1, usedBuffers);
        // addRequest(req, 3);
        // handleRequests(/*crtpMode=*/false);
        // const bootloaderWriteFlashResponse* response = getRequestResult<bootloaderWriteFlashResponse>(0);
        // if (response->done == 1 && response->error == 0) {
        //   break;
        // }
        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        if (elapsedSeconds.count() > 0.5) {
          start = end;
          sendPacketOrTimeout(req, false);
          ++tries;
          if (tries > 5) {
            throw std::runtime_error("timeout");
          }
        }
      }

      // std::cout << "Flashed: " << (page - flashStart) / (float)numPages * 100.0 << " %" << std::endl;

      // get ready to fill more buffers
      // if (page != numPages + flashStart - 1) {
      //   startBatchRequest();
      // }
      usedBuffers = 0;
    }
  }


}

void Crazyflie::readFlash(
  BootloaderTarget target,
  size_t size,
  std::vector<uint8_t>& data)
{
  // Get info about the target
  bootloaderGetInfoRequest req(target);
  startBatchRequest();
  addRequest(req, 3);
  handleRequests(/*crtpMode=*/false, /*useSafelink*/false);
  const bootloaderGetInfoResponse* response = getRequestResult<bootloaderGetInfoResponse>(0);
  uint16_t pageSize = response->pageSize;
  uint16_t flashStart = response->flashStart;

  uint16_t numPages = ceil(size / (float)pageSize);
  if (numPages + flashStart >= response->nFlashPage) {
    std::stringstream sstr;
    sstr << "Requested size too large!";
    throw std::runtime_error(sstr.str());
  }

  std::stringstream sstr;
  sstr << "pageSize: " << pageSize
       << " nFlashPage: " << response->nFlashPage
       << " flashStart: " << flashStart
       << " version: " << (int)response->version
       << " numPages: " << numPages;
  m_logger.info(sstr.str());

  // read flash
  size_t offset = 0;
  startBatchRequest();
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      // std::cout << "request: " << page << " " << address << std::endl;
      bootloaderReadFlashRequest req(target, page, address);
      addRequest(req, 7);
      size_t requestedSize = std::min(25, pageSize - address);
      offset += requestedSize;
      if (offset > size) {
        break;
      }
    }
  }
  handleRequests(/*crtpMode=*/false, /*useSafelink*/false);

  // update output
  data.resize(size);
  size_t i = 0;
  offset = 0;
  for (uint16_t page = flashStart; page < numPages + flashStart; ++page) {
    for (uint16_t address = 0; address < pageSize; address += 25) {
      const bootloaderReadFlashResponse* response = getRequestResult<bootloaderReadFlashResponse>(i++);
      size_t requestedSize = std::min(25, pageSize - address);
      // std::cout << "offset: " << offset << " reqS: " << requestedSize;
      memcpy(&data[offset], response->data, std::min(size - offset, requestedSize));
      offset += requestedSize;
      if (offset > size) {
        break;
      }
    }
  }
}
#endif

void Crazyflie::requestLogToc(bool forceNoCache)
{
  // Lazily initialize protocol version
  if (m_protocolVersion < 0)
  {
    m_protocolVersion = getProtocolVersion();
  }
  if (m_protocolVersion < 4)
  {
    m_logger.error("Old logging interface not supported! Please update your firmware.");
    return;
  }

  // Find the number of entries in TOC
  crtpLogGetInfoV2Request req;
  m_connection.send(req);
  using res = crtpLogGetInfoV2Response;
  auto p = waitForResponse(&res::valid);
  uint16_t numLogVariables = res::numLogVariables(p);
  uint32_t crc = res::crc(p);

  m_logger.info("Log TOC: " + std::to_string(numLogVariables) + " entries with CRC " + std::to_string(crc));

  // check if it is in the cache
  std::string fileName = "log" + std::to_string(crc) + ".csv";
  std::ifstream infile(fileName);

  m_logTocEntries.clear();
  if (!forceNoCache && infile.good())
  {
    m_logger.info("Log TOC: found cache.");
    std::string line, cell;
    std::getline(infile, line); // ignore header
    while (std::getline(infile, line)) {
      std::stringstream lineStream(line);
      m_logTocEntries.resize(m_logTocEntries.size() + 1);
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().id = std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().type = (LogType)std::stoi(cell);
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().group = cell;
      std::getline(lineStream, cell, ',');
      m_logTocEntries.back().name = cell;
    }
    if (m_logTocEntries.size() != numLogVariables)
    {
      m_logger.warning("Log TOC: invalid cache.");
      m_logTocEntries.clear();
    }
  }
  if (m_logTocEntries.empty())
  {
    m_logger.info("Log TOC: not in cache");

    // Update internal structure with obtained data
    m_logTocEntries.resize(numLogVariables);

    // Request detailed information
    for (uint16_t i = 0; i < numLogVariables; ++i)
    {
      crtpLogGetItemV2Request req1(i);
      m_connection.send(req1);
#ifndef FIRMWARE_BUGGY
    }

    for (uint16_t i = 0; i < numLogVariables; ++i)
    {
#endif
      using res = crtpLogGetItemV2Response;
      auto p = waitForResponse(&res::valid);

      LogTocEntry &entry = m_logTocEntries[i];
      entry.id = i;
      entry.type = (Crazyflie::LogType)res::type(p);
      auto groupAndName = res::groupAndName(p);
      entry.group = groupAndName.first;
      entry.name = groupAndName.second;

#ifdef FIRMWARE_BUGGY
      if (res::id(p) != i) {
        m_logger.warning("Firmware bug! expected " + std::to_string(i) + " got " + std::to_string(res::id(p)));
        --i; // try again...
      }
#else
      assert(res::id(p) == i);
#endif

    }
    // Write a cache file
    {
      // Atomic file write: write in temporary file first to avoid race conditions
      std::string fileNameTemp = fileName + ".tmp";
      std::ofstream output(fileNameTemp);
      output << "id,type,group,name" << std::endl;
      for (const auto &entry : m_logTocEntries)
      {
        output << std::to_string(entry.id) << ","
               << std::to_string(entry.type) << ","
               << entry.group << ","
               << entry.name << std::endl;
      }
      // change the filename
      rename(fileNameTemp.c_str(), fileName.c_str());
    }
  }

}

void Crazyflie::requestParamToc(bool forceNoCache, bool requestValues)
{
  // Lazily initialize protocol version
  if (m_protocolVersion < 0) {
    m_protocolVersion = getProtocolVersion();
  }
  if (m_protocolVersion < 4) {
    m_logger.error("Old parameter interface not supported! Please update your firmware.");
    return;
  }

  // Find the number of parameters in TOC
  crtpParamTocGetInfoV2Request req;
  m_connection.send(req);
  using res = crtpParamTocGetInfoV2Response;
  auto p = waitForResponse(&res::valid);
  uint16_t numParams = res::numParams(p);
  uint32_t crc = res::crc(p);

  m_logger.info("Param TOC: " + std::to_string(numParams) + " entries with CRC " + std::to_string(crc));

  // check if it is in the cache
  std::string fileName = "params" + std::to_string(crc) + ".csv";
  std::ifstream infile(fileName);

  m_paramTocEntries.clear();
  if (!forceNoCache && infile.good()) {
      m_logger.info("Param TOC: found cache.");
      std::string line, cell;
      std::getline(infile, line); // ignore header
      while (std::getline(infile, line)) {
        std::stringstream lineStream(line);
        m_paramTocEntries.resize(m_paramTocEntries.size() + 1);
        std::getline(lineStream, cell, ',');
        m_paramTocEntries.back().id = std::stoi(cell);
        std::getline(lineStream, cell, ',');
        m_paramTocEntries.back().type = (ParamType)std::stoi(cell);
        std::getline(lineStream, cell, ',');
        m_paramTocEntries.back().readonly = std::stoi(cell);
        std::getline(lineStream, cell, ',');
        m_paramTocEntries.back().group = cell;
        std::getline(lineStream, cell, ',');
        m_paramTocEntries.back().name = cell;
      }
      if (m_paramTocEntries.size() != numParams) {
        m_logger.warning("Param TOC: invalid cache.");
        m_paramTocEntries.clear();
      }
  }
  if (m_paramTocEntries.empty()) {
    m_logger.info("Param TOC: not in cache");

    // Update internal structure with obtained data
    m_paramTocEntries.resize(numParams);

    // Request detailed information
    for (uint16_t i = 0; i < numParams; ++i) {
      crtpParamTocGetItemV2Request req1(i);
      m_connection.send(req1);

#ifndef FIRMWARE_BUGGY
    }

    for (uint16_t i = 0; i < numParams; ++i) {
#endif
      using res1 = crtpParamTocGetItemV2Response;
      auto p1 = waitForResponse(&res1::valid);

      ParamTocEntry &entry = m_paramTocEntries[i];
      entry.id = i;
      entry.type = (Crazyflie::ParamType) res1::type(p1);
      entry.readonly = res1::readonly(p1);
      auto groupAndName = res1::groupAndName(p1);
      entry.group = groupAndName.first;
      entry.name = groupAndName.second;

#ifdef FIRMWARE_BUGGY
      if (res1::id(p1) != i)
      {
        m_logger.warning("Firmware bug! expected " + std::to_string(i) + " got " + std::to_string(res1::id(p1)));
        --i; // try again...
      }
#else
      assert(res1::id(p1) == i);
#endif

    }
    // Write a cache file
    {
      // Atomic file write: write in temporary file first to avoid race conditions
      std::string fileNameTemp = fileName + ".tmp";
      std::ofstream output(fileNameTemp);
      output << "id,type,readonly,group,name" << std::endl;
      for (const auto& entry : m_paramTocEntries) {
        output << std::to_string(entry.id) << ","
               << std::to_string(entry.type) << ","
               << std::to_string(entry.readonly) << ","
               << entry.group << ","
               << entry.name << std::endl;
      }
      // change the filename
      rename(fileNameTemp.c_str(), fileName.c_str());
    }
  }

  // Request values
  assert(m_paramTocEntries.size() == numParams);
  
  if (requestValues) {
    requestParamValues();
  }
}

void Crazyflie::requestParamValues()
{
  size_t numParams = m_paramTocEntries.size();
  for (uint16_t i = 0; i < numParams; ++i)
  {
    crtpParamReadV2Request req2(i);
    m_connection.send(req2);

#ifndef FIRMWARE_BUGGY
  }

  for (uint16_t i = 0; i < numParams; ++i)
  {
#endif

    using res2 = crtpParamValueV2Response;
    auto p2 = waitForResponse(&res2::valid);
    assert(res2::status(p2) == 0);

    const auto &entry = m_paramTocEntries[i];
    switch (entry.type)
    {
    case ParamTypeUint8:
      m_paramValues[i].valueUint8 = res2::value<uint8_t>(p2);
      break;
    case ParamTypeInt8:
      m_paramValues[i].valueInt8 = res2::value<int8_t>(p2);
      break;
    case ParamTypeUint16:
      m_paramValues[i].valueUint16 = res2::value<uint16_t>(p2);
      break;
    case ParamTypeInt16:
      m_paramValues[i].valueInt16 = res2::value<int16_t>(p2);
      break;
    case ParamTypeUint32:
      m_paramValues[i].valueUint32 = res2::value<uint32_t>(p2);
      break;
    case ParamTypeInt32:
      m_paramValues[i].valueInt32 = res2::value<int32_t>(p2);
      break;
    case ParamTypeFloat:
      m_paramValues[i].valueFloat = res2::value<float>(p2);
      break;
    default:
      assert(false);
    }

#ifdef FIRMWARE_BUGGY
    if (res2::id(p2) != i)
    {
      m_logger.warning("Firmware bug! expected " + std::to_string(i) + " got " + std::to_string(res2::id(p2)));
      --i; // try again...
    }
#else
    assert(res2::id(p2) == i);
#endif

  }
}

void Crazyflie::requestMemoryToc()
{
  // Find the number of memories
  crtpMemoryGetNumberRequest req;
  m_connection.send(req);
  using res = crtpMemoryGetNumberResponse;
  auto p = waitForResponse(&res::valid);
  uint8_t numberOfMemories = res::numberOfMemories(p);

  m_logger.info("Memories: " + std::to_string(numberOfMemories));

  // Request detailed information
  m_memoryTocEntries.resize(numberOfMemories);

  for (uint8_t i = 0; i < numberOfMemories; ++i) {
    crtpMemoryGetInfoRequest req(i);
    m_connection.send(req);

#ifndef FIRMWARE_BUGGY
  }

  // Update internal structure with obtained data
  for (uint8_t i = 0; i < numberOfMemories; ++i) {
#endif
    using res = crtpMemoryGetInfoResponse;
    auto p = waitForResponse(&res::valid);

    MemoryTocEntry& entry = m_memoryTocEntries[i];
    entry.id = i;
    entry.type = (MemoryType)res::type(p);
    entry.size = res::size(p);
    entry.addr = res::addr(p);

#ifdef FIRMWARE_BUGGY
    if (res::id(p) != i)
    {
      m_logger.warning("Firmware bug! expected " + std::to_string(i) + " got " + std::to_string(res::id(p)));
      --i; // try again...
    }
#else
    assert(res::id(p) == i);
#endif
  }
}

void Crazyflie::setParam(uint16_t id, const ParamValue &value)
{
  bool found = false;
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      found = true;
      switch (entry.type) {
        case ParamTypeUint8:
          {
            crtpParamWriteV2Request<uint8_t> req(id, value.valueUint8);
            m_connection.send(req);
            break;
          }
        case ParamTypeInt8:
          {
            crtpParamWriteV2Request<int8_t> req(id, value.valueInt8);
            m_connection.send(req);
            break;
          }
        case ParamTypeUint16:
          {
            crtpParamWriteV2Request<uint16_t> req(id, value.valueUint16);
            m_connection.send(req);
            break;
          }
        case ParamTypeInt16:
          {
            crtpParamWriteV2Request<int16_t> req(id, value.valueInt16);
            m_connection.send(req);
            break;
          }
        case ParamTypeUint32:
          {
            crtpParamWriteV2Request<uint32_t> req(id, value.valueUint32);
            m_connection.send(req);
            break;
          }
        case ParamTypeInt32:
          {
            crtpParamWriteV2Request<int32_t> req(id, value.valueInt32);
            m_connection.send(req);
            break;
          }
        case ParamTypeFloat:
          {
            crtpParamWriteV2Request<float> req(id, value.valueFloat);
            m_connection.send(req);
            break;
          }
      }
    }
  }

  if (!found) {
    std::stringstream sstr;
    sstr << "Could not find parameter with id " << id;
    throw std::runtime_error(sstr.str());
  }

  m_paramValues[id] = value;
}

bitcraze::crazyflieLinkCpp::Packet Crazyflie::waitForResponse(
    std::function<bool(const bitcraze::crazyflieLinkCpp::Packet &)> condition)
{
  while (true) {
    auto p = m_connection.recv(0);
    processPacket(p);
    if (condition(p)) {
      return p;
    }
  }
}

void Crazyflie::processPacket(const bitcraze::crazyflieLinkCpp::Packet& p)
{
  if (crtpConsoleResponse::valid(p))
  {
    if (m_consoleCallback)
    {
      m_consoleCallback(crtpConsoleResponse::text(p).c_str());
    }
  }
  else if (crtpLogDataResponse::valid(p))
  {
    uint8_t blockId = crtpLogDataResponse::blockId(p);
    auto iter = m_logBlockCb.find(blockId);
    if (iter != m_logBlockCb.end())
    {
      iter->second(p, p.size() - 5);
    }
    else
    {
      m_logger.warning("Received unrequested data for block: " + std::to_string((int)blockId));
    }
  }
  else if (crtpLatencyMeasurementResponse::valid(p))
  {
    if (m_latencyCallback)
    {
      if (crtpLatencyMeasurementResponse::id(p) != m_latencyCounter-1) {
        m_logger.warning("Received wrong latency id: " + std::to_string(crtpLatencyMeasurementResponse::id(p)));
      } else {
        auto now = std::chrono::steady_clock::now();
        uint64_t recv_timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now - m_clock_start).count();
        uint64_t send_timestamp = crtpLatencyMeasurementResponse::timestamp(p);
        uint64_t latency_in_us = recv_timestamp - send_timestamp;
        m_latencyCallback(latency_in_us);
      }
    }
  }
}

const Crazyflie::LogTocEntry* Crazyflie::getLogTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_logTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

const Crazyflie::ParamTocEntry* Crazyflie::getParamTocEntry(
  const std::string& group,
  const std::string& name) const
{
  for (auto&& entry : m_paramTocEntries) {
    if (entry.group == group && entry.name == name) {
      return &entry;
    }
  }
  return nullptr;
}

uint8_t Crazyflie::registerLogBlock(
  std::function<void(const bitcraze::crazyflieLinkCpp::Packet&, uint8_t)> cb)
{
  for (uint8_t id = 0; id < 255; ++id) {
    if (m_logBlockCb.find(id) == m_logBlockCb.end()) {
      m_logBlockCb[id] = cb;
      return id;
    }
  }
  return 255;
}

bool Crazyflie::unregisterLogBlock(
  uint8_t id)
{
  m_logBlockCb.erase(m_logBlockCb.find(id));
  return true;
}

void Crazyflie::setGroupMask(uint8_t groupMask)
{
  crtpCommanderHighLevelSetGroupMaskRequest req(groupMask);
  m_connection.send(req);
}

void Crazyflie::takeoff(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelTakeoffRequest req(groupMask, height, duration);
  m_connection.send(req);
}

void Crazyflie::land(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelLandRequest req(groupMask, height, duration);
  m_connection.send(req);
}

void Crazyflie::stop(uint8_t groupMask)
{
  crtpCommanderHighLevelStopRequest req(groupMask);
  m_connection.send(req);
}

void Crazyflie::goTo(float x, float y, float z, float yaw, float duration, bool relative, uint8_t groupMask)
{
  crtpCommanderHighLevelGoToRequest req(groupMask, relative, x, y, z, yaw, duration);
  m_connection.send(req);
}

void Crazyflie::uploadTrajectory(
  uint8_t trajectoryId,
  uint32_t pieceOffset,
  const std::vector<poly4d>& pieces)
{
  for (const auto& entry : m_memoryTocEntries) {
    if (entry.type == MemoryTypeTRAJ) {
      // upload pieces
      size_t remainingBytes = sizeof(poly4d) * pieces.size();
      size_t numRequests = ceil(remainingBytes / 24.0f);
      for (size_t i = 0; i < numRequests; ++i) {
        crtpMemoryWriteRequest req(entry.id, pieceOffset * sizeof(poly4d) + i*24);
        size_t size = std::min<size_t>(remainingBytes, 24);
        req.setDataAt(0, reinterpret_cast<const uint8_t *>(pieces.data()) + i * 24, size);
        req.setDataSize(size);
        m_connection.send(req);

        // wait for the response
        using res = crtpMemoryWriteResponse;
        auto p = waitForResponse(&res::valid);
        if (   res::id(p) != entry.id
            || res::address(p) != pieceOffset * sizeof(poly4d) + i*24
            || res::status(p) != 0) {
            m_logger.error("uploadTrajectory: unexpected response!" + std::to_string(res::status(p)));
        }

        remainingBytes -= size;
      }
      // define trajectory
      crtpCommanderHighLevelDefineTrajectoryRequest req(trajectoryId);
      req.setPoly4d(pieceOffset * sizeof(poly4d), (uint8_t)pieces.size());
      m_connection.send(req);

      // // verify
      // remainingBytes = sizeof(poly4d) * pieces.size();
      // numRequests = ceil(remainingBytes / 24.0f);
      // for (size_t i = 0; i < numRequests; ++i) {
      //   size_t size = std::min<size_t>(remainingBytes, 24);
      //   crtpMemoryReadRequest req(entry.id, pieceOffset * sizeof(poly4d) + i*24, size);
        
      //   m_connection.send(req);
      //   using res = crtpMemoryReadResponse;
      //   auto p = waitForResponse(&res::valid);
      //   if (   res::id(p) != entry.id
      //       || res::address(p) != pieceOffset * sizeof(poly4d) + i*24
      //       || res::dataSize(p) != size
      //       || res::status(p) != 0) {
      //       m_logger.error("uploadTrajectory: unexpected response!");
      //       return;
      //   }

      //   if (memcmp(reinterpret_cast<const uint8_t *>(pieces.data()) + i * 24, res::data(p), res::dataSize(p)) != 0) {
      //       m_logger.error("uploadTrajectory: verify failed!");
      //   }

      //   remainingBytes -= size;
      // }
      // m_logger.info("upload & verify done!");

      return;
    }
  }
  throw std::runtime_error("Could not find MemoryTypeTRAJ!");
}

void Crazyflie::startTrajectory(
  uint8_t trajectoryId,
  float timescale,
  bool reversed,
  bool relative,
  uint8_t groupMask)
{
  crtpCommanderHighLevelStartTrajectoryRequest req(groupMask, relative, reversed, trajectoryId, timescale);
  m_connection.send(req);
}

void Crazyflie::readUSDLogFile(
  std::vector<uint8_t>& data)
{
  for (const auto& entry : m_memoryTocEntries) {
    if (entry.type == MemoryTypeUSD) {
      size_t remainingBytes = entry.size;
      size_t numRequests = ceil(remainingBytes / 24.0f);
      data.resize(entry.size);
      for (size_t i = 0; i < numRequests; ++i) {
        size_t size = std::min<size_t>(remainingBytes, 24);
        crtpMemoryReadRequest req(entry.id, i*24, size);
        
        m_connection.send(req);
        using res = crtpMemoryReadResponse;
        auto p = waitForResponse(&res::valid);
        if (   res::id(p) != entry.id
            || res::address(p) != i*24
            || res::dataSize(p) != size
            || res::status(p) != 0) {
            m_logger.error("readUSDLogFile: unexpected response!");
            data.clear();
            return;
        }
        memcpy(&data[i*24], res::data(p), res::dataSize(p));
        remainingBytes -= size;
      }
      return;
    }
  }
  m_logger.error("Could not find MemoryTypeUSD!");
}

void Crazyflie::triggerLatencyMeasurement()
{
  auto now = std::chrono::steady_clock::now();
  uint64_t timestamp = std::chrono::duration_cast<std::chrono::microseconds>(now - m_clock_start).count();
  crtpLatencyMeasurementRequest req(m_latencyCounter, timestamp);
  m_connection.send(req);

  ++m_latencyCounter;
}
////////////////////////////////////////////////////////////////

CrazyflieBroadcaster::CrazyflieBroadcaster(
  const std::string& link_uri)
  : m_connection(link_uri)
{
}

void CrazyflieBroadcaster::sendArmingRequest(bool arm)
{
  crtpArmingRequest req(arm);
  m_connection.send(req);
}

void CrazyflieBroadcaster::takeoff(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelTakeoffRequest req(groupMask, height, duration);
  m_connection.send(req);
}

void CrazyflieBroadcaster::land(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelLandRequest req(groupMask, height, duration);
  m_connection.send(req);
}

void CrazyflieBroadcaster::stop(uint8_t groupMask)
{
  crtpCommanderHighLevelStopRequest req(groupMask);
  m_connection.send(req);
}

// This is always in relative coordinates
void CrazyflieBroadcaster::goTo(float x, float y, float z, float yaw, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelGoToRequest req(groupMask, true, x, y, z, yaw, duration);
  m_connection.send(req);
}

// This is always in relative coordinates
// TODO: this does not support trajectories that are of a different length!
void CrazyflieBroadcaster::startTrajectory(
  uint8_t trajectoryId,
  float timescale,
  bool reversed,
  uint8_t groupMask)
{
  crtpCommanderHighLevelStartTrajectoryRequest req(groupMask, true, reversed, trajectoryId, timescale);
  m_connection.send(req);
}

void CrazyflieBroadcaster::notifySetpointsStop(uint32_t remainValidMillisecs)
{
  crtpNotifySetpointsStopRequest req(remainValidMillisecs);
  m_connection.send(req);
}

void CrazyflieBroadcaster::sendExternalPositions(
  const std::vector<externalPosition>& data)
{
  crtpExternalPositionPacked req;
  size_t j = 0;
  for (size_t i = 0; i < data.size(); ++i) {
    req.add(data[i].id, data[i].x * 1000, data[i].y * 1000, data[i].z * 1000);
    ++j;
    if (j == 4) {
      m_connection.send(req);
      req.clear();
      j = 0;
    }
  }
  if (j > 0) {
    m_connection.send(req);
  }
}

void CrazyflieBroadcaster::emergencyStop()
{
  crtpEmergencyStopRequest req;
  m_connection.send(req);
}

void CrazyflieBroadcaster::emergencyStopWatchdog()
{
  crtpEmergencyStopWatchdogRequest req;
  m_connection.send(req);
}

void CrazyflieBroadcaster::sendExternalPoses(
  const std::vector<externalPose>& data)
{
  crtpExternalPosePacked req;
  size_t j = 0;
  for (size_t i = 0; i < data.size(); ++i) {
    float q[4] = {data[i].qx, data[i].qy, data[i].qz, data[i].qw};
    uint32_t quat = quatcompress(q);
    req.add(data[i].id, data[i].x * 1000, data[i].y * 1000, data[i].z * 1000, quat);
    ++j;
    if (j == 2) {
      m_connection.send(req);
      req.clear();
      j = 0;
    }
  }
  if (j > 0) {
    m_connection.send(req);
  }
}

void CrazyflieBroadcaster::sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate)
{
  crtpFullStateSetpointRequest req(
      x, y, z,
      vx, vy, vz,
      ax, ay, az,
      qx, qy, qz, qw,
      rollRate, pitchRate, yawRate);
  m_connection.send(req);
}