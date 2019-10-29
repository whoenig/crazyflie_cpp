//#include <regex>
#include <mutex>

#include "Crazyflie.h"
#include "crtp.h"
#include "crtpBootloader.h"
#include "crtpNRF51.h"

#include "Crazyradio.h"
#include "CrazyflieUSB.h"

#include <fstream>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <cmath>
#include <inttypes.h>

const static int MAX_RADIOS = 16;
const static int MAX_USB = 4;
const static bool LOG_COMMUNICATION = 0;

Crazyradio* g_crazyradios[MAX_RADIOS];
std::mutex g_radioMutex[MAX_RADIOS];

CrazyflieUSB* g_crazyflieUSB[MAX_USB];
std::mutex g_crazyflieusbMutex[MAX_USB];

Logger EmptyLogger;


Crazyflie::Crazyflie(
  const std::string& link_uri,
  Logger& logger,
  std::function<void(const char*)> consoleCb)
  : m_radio(nullptr)
  , m_transport(nullptr)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
  , m_logTocEntries()
  , m_logBlockCb()
  , m_paramTocEntries()
  , m_paramValues()
  , m_emptyAckCallback(nullptr)
  , m_linkQualityCallback(nullptr)
  , m_consoleCallback(consoleCb)
  , m_log_use_V2(false)
  , m_param_use_V2(false)
  , m_logger(logger)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%" SCNx64,
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId] = new Crazyradio(m_devId);
        g_crazyradios[m_devId]->enableLogging(LOG_COMMUNICATION);
        // g_crazyradios[m_devId]->setAckEnable(false);
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    success = std::sscanf(link_uri.c_str(), "usb://%d",
       &m_devId) == 1;

    if (m_devId >= MAX_USB) {
      throw std::runtime_error("This version does not support that many CFs over USB. Adjust MAX_USB and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_crazyflieusbMutex[m_devId]);
      if (!g_crazyflieUSB[m_devId]) {
        g_crazyflieUSB[m_devId] = new CrazyflieUSB(m_devId);
        g_crazyflieUSB[m_devId]->enableLogging(LOG_COMMUNICATION);
      }
    }

    m_transport = g_crazyflieUSB[m_devId];
  }

  if (!success) {
    throw std::runtime_error("Uri is not valid!");
  }

  // enable safelink
  if (m_radio) {
    crtpNrf51SetSafelinkRequest request(ENABLE_SAFELINK);
    sendPacketOrTimeout(request, /*useSafeLink*/false);
  }

  m_curr_up = 0;
  m_curr_down = 0;

  m_protocolVersion = -1;

}

int Crazyflie::getProtocolVersion()
{
  crtpGetProtocolVersionRequest req;
  startBatchRequest();
  addRequest(req, 1);
  handleRequests();
  return getRequestResult<crtpGetProtocolVersionResponse>(0)->version;
}

std::string Crazyflie::getFirmwareVersion()
{
  crtpGetFirmwareVersionRequest req;
  startBatchRequest();
  addRequest(req, 1);
  handleRequests();
  return std::string(getRequestResult<crtpGetFirmwareVersionResponse>(0)->version);
}

std::string Crazyflie::getDeviceTypeName()
{
  crtpGetDeviceTypeNameRequest req;
  startBatchRequest();
  addRequest(req, 1);
  handleRequests();
  return std::string(getRequestResult<crtpGetDeviceTypeNameResponse>(0)->name);
}

void Crazyflie::logReset()
{
  crtpLogResetRequest request;
  startBatchRequest();
  addRequest(request, 1);
  handleRequests();
}

void Crazyflie::sendSetpoint(
  float roll,
  float pitch,
  float yawrate,
  uint16_t thrust)
{
  crtpSetpointRequest request(roll, pitch, yawrate, thrust);
  sendPacket(request);
}

void Crazyflie::sendStop()
{
  crtpStopRequest request;
  sendPacket(request);
}

void Crazyflie::emergencyStop()
{
  crtpEmergencyStopRequest request;
  sendPacketOrTimeout(request);
}

void Crazyflie::emergencyStopWatchdog()
{
  crtpEmergencyStopWatchdogRequest request;
  sendPacketOrTimeout(request);
}

void Crazyflie::sendPositionSetpoint(
  float x,
  float y,
  float z,
  float yaw)
{
  crtpPositionSetpointRequest request(x, y, z, yaw);
  sendPacket(request);
}

void Crazyflie::sendHoverSetpoint(
  float vx,
  float vy,
  float yawrate,
  float zDistance)
{
  crtpHoverSetpointRequest request(vx, vy, yawrate, zDistance);
  sendPacket(request);
}

void Crazyflie::sendFullStateSetpoint(
    float x, float y, float z,
    float vx, float vy, float vz,
    float ax, float ay, float az,
    float qx, float qy, float qz, float qw,
    float rollRate, float pitchRate, float yawRate)
{
  crtpFullStateSetpointRequest request(
    x, y, z,
    vx, vy, vz,
    ax, ay, az,
    qx, qy, qz, qw,
    rollRate, pitchRate, yawRate);
  sendPacket(request);
}

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

void Crazyflie::sendPing()
{
  crtpEmpty req;
  sendPacket(req);
}

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

// https://forum.bitcraze.io/viewtopic.php?f=9&t=1488
void Crazyflie::reboot()
{
  if (m_radio) {
    crtpNrf51ResetInitRequest req1;
    sendPacketOrTimeout(req1);

    crtpNrf51ResetRequest req2(/*bootToFirmware*/ 1);
    sendPacketOrTimeout(req2);
  }
}

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

void Crazyflie::sysoff()
{
  if (m_radio) {
    crtpNrf51SysOffRequest req;
    sendPacketOrTimeout(req);
  }
}

void Crazyflie::alloff()
{
  if (m_radio) {
    crtpNrf51AllOffRequest req;
    sendPacketOrTimeout(req);
  }
}

void Crazyflie::syson()
{
  if (m_radio) {
    crtpNrf51SysOnRequest req;
    sendPacketOrTimeout(req);
  }
}

float Crazyflie::vbat()
{
  if (m_radio) {
    crtpNrf51GetVBatRequest req;
    startBatchRequest();
    addRequest(req, 2);
    handleRequests();
    return getRequestResult<crtpNrf51GetVBatResponse>(0)->vbat;
  } else {
    return nan("");
  }
}

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

void Crazyflie::requestLogToc(bool forceNoCache)
{
  m_log_use_V2 = true;
  uint16_t len;
  uint32_t crc;

  // Lazily initialize protocol version
  if (m_protocolVersion < 0) {
    m_protocolVersion = getProtocolVersion();
  }

  crtpLogGetInfoV2Request infoRequest;
  startBatchRequest();
  addRequest(infoRequest, 1);
  if (m_protocolVersion >= 4) {
    handleRequests();
    len = getRequestResult<crtpLogGetInfoV2Response>(0)->log_len;
    crc = getRequestResult<crtpLogGetInfoV2Response>(0)->log_crc;
  } else {
    // std::cout << "Fall back to V1 param API" << std::endl;
    m_log_use_V2 = false;

    crtpLogGetInfoRequest infoRequest;
    startBatchRequest();
    addRequest(infoRequest, 1);
    handleRequests();
    len = getRequestResult<crtpLogGetInfoResponse>(0)->log_len;
    crc = getRequestResult<crtpLogGetInfoResponse>(0)->log_crc;
    // std::cout << len << std::endl;
  }

  // check if it is in the cache
  std::string fileName = "log" + std::to_string(crc) + ".csv";
  std::ifstream infile(fileName);

  if (forceNoCache || !infile.good()) {
    m_logger.info("Log: " + std::to_string(len));

    // Request detailed information
    startBatchRequest();
    if (m_log_use_V2) {
      for (size_t i = 0; i < len; ++i) {
        crtpLogGetItemV2Request itemRequest(i);
        addRequest(itemRequest, 2);
      }
    } else {
      for (size_t i = 0; i < len; ++i) {
        crtpLogGetItemRequest itemRequest(i);
        addRequest(itemRequest, 2);
      }
    }
    handleRequests();

    // Update internal structure with obtained data
    m_logTocEntries.resize(len);
    if (m_log_use_V2) {
      for (size_t i = 0; i < len; ++i) {
        auto response = getRequestResult<crtpLogGetItemV2Response>(i);
        LogTocEntry& entry = m_logTocEntries[i];
        entry.id = i;
        entry.type = (LogType)response->type;
        entry.group = std::string(&response->text[0]);
        entry.name = std::string(&response->text[entry.group.size() + 1]);
      }
    } else {
      for (size_t i = 0; i < len; ++i) {
        auto response = getRequestResult<crtpLogGetItemResponse>(i);
        LogTocEntry& entry = m_logTocEntries[i];
        entry.id = i;
        entry.type = (LogType)response->type;
        entry.group = std::string(&response->text[0]);
        entry.name = std::string(&response->text[entry.group.size() + 1]);
      }
    }

    // Write a cache file
    {
      // Atomic file write: write in temporary file first to avoid race conditions
      std::string fileNameTemp = fileName + ".tmp";
      std::ofstream output(fileNameTemp);
      output << "id,type,group,name" << std::endl;
      for (const auto& entry : m_logTocEntries) {
        output << std::to_string(entry.id) << ","
               << std::to_string(entry.type) << ","
               << entry.group << ","
               << entry.name << std::endl;
      }
      // change the filename
      rename(fileNameTemp.c_str(), fileName.c_str());
    }
  } else {
    m_logger.info("Found variables in cache.");
    m_logTocEntries.clear();
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
  }
}

void Crazyflie::requestParamToc(bool forceNoCache)
{
  m_param_use_V2 = true;
  uint16_t numParam;
  uint32_t crc;

  // Lazily initialize protocol version
  if (m_protocolVersion < 0) {
    m_protocolVersion = getProtocolVersion();
  }

  // Find the number of parameters in TOC
  crtpParamTocGetInfoV2Request infoRequest;
  startBatchRequest();
  // std::cout << "infoReq" << std::endl;
  addRequest(infoRequest, 1);
  if (m_protocolVersion >= 4) {
    handleRequests();
    numParam = getRequestResult<crtpParamTocGetInfoV2Response>(0)->numParam;
    crc = getRequestResult<crtpParamTocGetInfoV2Response>(0)->crc;
  } else {
    // std::cout << "Fall back to V1 param API" << std::endl;
    m_param_use_V2 = false;

    crtpParamTocGetInfoRequest infoRequest;
    startBatchRequest();
    addRequest(infoRequest, 1);
    handleRequests();
    numParam = getRequestResult<crtpParamTocGetInfoResponse>(0)->numParam;
    crc = getRequestResult<crtpParamTocGetInfoResponse>(0)->crc;
  }

  // check if it is in the cache
  std::string fileName = "params" + std::to_string(crc) + ".csv";
  std::ifstream infile(fileName);

  if (forceNoCache || !infile.good()) {
    m_logger.info("Params: " + std::to_string(numParam));

    // Request detailed information and values
    startBatchRequest();
    if (!m_param_use_V2) {
      for (uint16_t i = 0; i < numParam; ++i) {
        crtpParamTocGetItemRequest itemRequest(i);
        addRequest(itemRequest, 2);
        crtpParamReadRequest readRequest(i);
        addRequest(readRequest, 1);
      }
    } else {
      for (uint16_t i = 0; i < numParam; ++i) {
        crtpParamTocGetItemV2Request itemRequest(i);
        addRequest(itemRequest, 2);
        crtpParamReadV2Request readRequest(i);
        addRequest(readRequest, 1);
      }
    }
    handleRequests();
    // Update internal structure with obtained data
    m_paramTocEntries.resize(numParam);

    if (!m_param_use_V2) {
      for (uint16_t i = 0; i < numParam; ++i) {
        auto r = getRequestResult<crtpParamTocGetItemResponse>(i*2+0);
        auto val = getRequestResult<crtpParamValueResponse>(i*2+1);

        ParamTocEntry& entry = m_paramTocEntries[i];
        entry.id = i;
        entry.type = (ParamType)(r->length | r-> type << 2 | r->sign << 3);
        entry.readonly = r->readonly;
        entry.group = std::string(&r->text[0]);
        entry.name = std::string(&r->text[entry.group.size() + 1]);

        ParamValue v;
        std::memcpy(&v, &val->valueFloat, 4);
        m_paramValues[i] = v;
      }
    } else {
      for (uint16_t i = 0; i < numParam; ++i) {
        auto r = getRequestResult<crtpParamTocGetItemV2Response>(i*2+0);
        auto val = getRequestResult<crtpParamValueV2Response>(i*2+1);

        ParamTocEntry& entry = m_paramTocEntries[i];
        entry.id = i;
        entry.type = (ParamType)(r->length | r-> type << 2 | r->sign << 3);
        entry.readonly = r->readonly;
        entry.group = std::string(&r->text[0]);
        entry.name = std::string(&r->text[entry.group.size() + 1]);

        ParamValue v;
        std::memcpy(&v, &val->valueFloat, 4);
        m_paramValues[i] = v;
      }
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
  } else {
    m_logger.info("Found variables in cache.");
    m_paramTocEntries.clear();
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

    // Request values
    if (!m_param_use_V2) {
      startBatchRequest();
      for (size_t i = 0; i < numParam; ++i) {
        crtpParamReadRequest readRequest(i);
        addRequest(readRequest, 1);
      }
      handleRequests();
      for (size_t i = 0; i < numParam; ++i) {
        auto val = getRequestResult<crtpParamValueResponse>(i);
        ParamValue v;
        std::memcpy(&v, &val->valueFloat, 4);
        m_paramValues[i] = v;
      }
    } else {
      startBatchRequest();
      for (size_t i = 0; i < numParam; ++i) {
        crtpParamReadV2Request readRequest(i);
        addRequest(readRequest, 1);
      }
      handleRequests();
      for (size_t i = 0; i < numParam; ++i) {
        auto val = getRequestResult<crtpParamValueV2Response>(i);
        ParamValue v;
        std::memcpy(&v, &val->valueFloat, 4);
        m_paramValues[i] = v;
      }
    }
  }
}

void Crazyflie::requestMemoryToc()
{
  // Find the number of parameters in TOC
  crtpMemoryGetNumberRequest infoRequest;
  startBatchRequest();
  addRequest(infoRequest, 1);
  handleRequests();
  uint8_t len = getRequestResult<crtpMemoryGetNumberResponse>(0)->numberOfMemories;

  m_logger.info("Memories: " + std::to_string(len));

  // Request detailed information and values
  startBatchRequest();
  for (uint8_t i = 0; i < len; ++i) {
    crtpMemoryGetInfoRequest itemRequest(i);
    addRequest(itemRequest, 2);
  }
  handleRequests();

  // Update internal structure with obtained data
  m_memoryTocEntries.resize(len);
  for (uint8_t i = 0; i < len; ++i) {
    auto info = getRequestResult<crtpMemoryGetInfoResponse>(i);

    MemoryTocEntry& entry = m_memoryTocEntries[i];
    entry.id = i;
    entry.type = (MemoryType)info->memType;
    entry.size = info->memSize;
    entry.addr = info->memAddr;
  }
}

void Crazyflie::startSetParamRequest()
{
  startBatchRequest();
}

void Crazyflie::addSetParam(uint16_t id, const ParamValue& value)
{
  bool found = false;
  for (auto&& entry : m_paramTocEntries) {
    if (entry.id == id) {
      found = true;
      if (!m_param_use_V2) {
        switch (entry.type) {
          case ParamTypeUint8:
            {
              crtpParamWriteRequest<uint8_t> request(id, value.valueUint8);
              addRequest(request, 1);
              break;
            }
          case ParamTypeInt8:
            {
              crtpParamWriteRequest<int8_t> request(id, value.valueInt8);
              addRequest(request, 1);
              break;
            }
          case ParamTypeUint16:
            {
              crtpParamWriteRequest<uint16_t> request(id, value.valueUint16);
              addRequest(request, 1);
              break;
            }
          case ParamTypeInt16:
            {
              crtpParamWriteRequest<int16_t> request(id, value.valueInt16);
              addRequest(request, 1);
              break;
            }
          case ParamTypeUint32:
            {
              crtpParamWriteRequest<uint32_t> request(id, value.valueUint32);
              addRequest(request, 1);
              break;
            }
          case ParamTypeInt32:
            {
              crtpParamWriteRequest<int32_t> request(id, value.valueInt32);
              addRequest(request, 1);
              break;
            }
          case ParamTypeFloat:
            {
              crtpParamWriteRequest<float> request(id, value.valueFloat);
              addRequest(request, 1);
              break;
            }
        }
      } else {
        switch (entry.type) {
          case ParamTypeUint8:
            {
              crtpParamWriteV2Request<uint8_t> request(id, value.valueUint8);
              addRequest(request, 2);
              break;
            }
          case ParamTypeInt8:
            {
              crtpParamWriteV2Request<int8_t> request(id, value.valueInt8);
              addRequest(request, 2);
              break;
            }
          case ParamTypeUint16:
            {
              crtpParamWriteV2Request<uint16_t> request(id, value.valueUint16);
              addRequest(request, 2);
              break;
            }
          case ParamTypeInt16:
            {
              crtpParamWriteV2Request<int16_t> request(id, value.valueInt16);
              addRequest(request, 2);
              break;
            }
          case ParamTypeUint32:
            {
              crtpParamWriteV2Request<uint32_t> request(id, value.valueUint32);
              addRequest(request, 2);
              break;
            }
          case ParamTypeInt32:
            {
              crtpParamWriteV2Request<int32_t> request(id, value.valueInt32);
              addRequest(request, 2);
              break;
            }
          case ParamTypeFloat:
            {
              crtpParamWriteV2Request<float> request(id, value.valueFloat);
              addRequest(request, 2);
              break;
            }
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

void Crazyflie::setRequestedParams()
{
  handleRequests();
}

void Crazyflie::setParam(uint16_t id, const ParamValue& value)
{
  startBatchRequest();
  addSetParam(id, value);
  setRequestedParams();
}

bool Crazyflie::sendPacketInternal(
  const uint8_t* data,
  uint32_t length,
  bool useSafeLink)
{
  ITransport::Ack ack;
  sendPacketInternal(data, length, ack, useSafeLink);
  return ack.ack;
}

 void Crazyflie::sendPacketOrTimeoutInternal(
   const uint8_t* data,
   uint32_t length,
   bool useSafeLink,
   float timeout)
{
  auto start = std::chrono::system_clock::now();
  while (!sendPacketInternal(data, length, useSafeLink)) {
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    if (elapsedSeconds.count() > timeout) {
      throw std::runtime_error("timeout");
    }
  }
}

void Crazyflie::sendPacketInternal(
  const uint8_t* data,
  uint32_t length,
  ITransport::Ack& ack,
  bool useSafeLink)
{
  static uint32_t numPackets = 0;
  static uint32_t numAcks = 0;

  numPackets++;

  if (m_radio) {
    std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
    if (m_radio->getAddress() != m_address) {
      m_radio->setAddress(m_address);
    }
    if (m_radio->getChannel() != m_channel) {
      m_radio->setChannel(m_channel);
    }
    if (m_radio->getDatarate() != m_datarate) {
      m_radio->setDatarate(m_datarate);
    }
    if (!m_radio->getAckEnable()) {
      m_radio->setAckEnable(true);
    }
    // consider safelink here:
    //    Adds 1bit counter to CRTP header to guarantee that no ack (downlink)
    //    payload are lost and no uplink packet are duplicated.
    //    The caller should resend packet if not acked (ie. same as with a
    //    direct call to crazyradio.send_packet)
    if (useSafeLink) {
      std::vector<uint8_t> dataCopy(data, data + length);
      dataCopy[0] &= 0xF3;
      dataCopy[0] |= m_curr_up << 3 | m_curr_down << 2;
      m_radio->sendPacket(dataCopy.data(), length, ack);
      if (ack.ack && ack.size > 0 && (ack.data[0] & 0x04) == (m_curr_down << 2)) {
        m_curr_down = 1 - m_curr_down;
      }
      if (ack.ack) {
        m_curr_up = 1 - m_curr_up;
      }
    } else {
      m_radio->sendPacket(data, length, ack);
    }

  } else {
    std::unique_lock<std::mutex> mlock(g_crazyflieusbMutex[m_devId]);
    m_transport->sendPacket(data, length, ack);
  }
  ack.data[ack.size] = 0;
  if (ack.ack) {
    handleAck(ack);
    numAcks++;
  }
  if (numPackets == 100) {
    if (m_linkQualityCallback) {
      // We just take the ratio of sent vs. acked packets here
      // for a sliding window of 100 packets
      float linkQuality = numAcks / (float)numPackets;
      m_linkQualityCallback(linkQuality);
    }
    numPackets = 0;
    numAcks = 0;
  }
}

void Crazyflie::handleAck(
  const ITransport::Ack& result)
{
  if (crtpConsoleResponse::match(result)) {
    if (result.size > 0) {
      crtpConsoleResponse* r = (crtpConsoleResponse*)result.data;
      if (m_consoleCallback) {
        m_consoleCallback(r->text);
      }
      // std::cout << "Console CF: " << r->text << std::endl;
    }
  }
  else if (crtpLogGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogGetInfoV2Response::match(result)) {
    // handled in batch system
  }
  else if (crtpLogGetItemResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogGetItemV2Response::match(result)) {
    // handled in batch system
  }
  else if (crtpLogControlResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpLogDataResponse::match(result)) {
    crtpLogDataResponse* r = (crtpLogDataResponse*)result.data;
    auto iter = m_logBlockCb.find(r->blockId);
    if (iter != m_logBlockCb.end()) {
      iter->second(r, result.size - 5);
    }
    else {
      m_logger.warning("Received unrequested data for block: " + std::to_string((int)r->blockId));
    }
  }
  else if (crtpParamTocGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamTocGetItemResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamTocGetInfoV2Response::match(result)) {
    // handled in batch system
  }
  else if (crtpParamTocGetItemV2Response::match(result)) {
    // handled in batch system
  }
  else if (crtpParamSetByNameResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpMemoryGetNumberResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpMemoryGetInfoResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpParamValueResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpMemoryGetNumberResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpMemoryReadResponse::match(result)) {
    // handled in batch system
  }
  else if (crtpMemoryWriteResponse::match(result)) {
    // handled in batch system
  }
  else if (crtp(result.data[0]).port == 8) {
    // handled in batch system
  }
  else if (crtp(result.data[0]).port == 13) {
    // handled in batch system
  }
  else if (crtpPlatformRSSIAck::match(result)) {
    if (result.size >= 3) {
      crtpPlatformRSSIAck* r = (crtpPlatformRSSIAck*)result.data;
      if (m_emptyAckCallback) {
        m_emptyAckCallback(r);
      }
    }
  }
  else {
    crtp* header = (crtp*)result.data;
    m_logger.warning("Don't know ack: Port: " + std::to_string((int)header->port)
      + " Channel: " + std::to_string((int)header->channel)
      + " Len: " + std::to_string((int)result.size));
    // for (size_t i = 1; i < result.size; ++i) {
    //   std::cout << "    " << (int)result.data[i] << std::endl;
    // }
    if (m_genericPacketCallback) {
      m_genericPacketCallback(result);
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
  std::function<void(crtpLogDataResponse*, uint8_t)> cb)
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

// Batch system

void Crazyflie::startBatchRequest()
{
  m_batchRequests.clear();
}

void Crazyflie::addRequestInternal(
  const uint8_t* data,
  size_t numBytes,
  size_t numBytesToMatch)
{
  m_batchRequests.resize(m_batchRequests.size() + 1);
  m_batchRequests.back().request.resize(numBytes);
  memcpy(m_batchRequests.back().request.data(), data, numBytes);
  m_batchRequests.back().numBytesToMatch = numBytesToMatch;
  m_batchRequests.back().finished = false;
}

void Crazyflie::handleRequests(
  bool crtpMode,
  bool useSafeLink,
  float baseTime,
  float timePerRequest)
{
  auto start = std::chrono::system_clock::now();
  ITransport::Ack ack;
  m_numRequestsFinished = 0;
  m_numRequestsEnqueued = 0;
  bool sendPing = false;
  const size_t queueSize = 16;

  float timeout = baseTime + timePerRequest * m_batchRequests.size();

  // Workaround for https://github.com/USC-ACTLab/crazyswarm/issues/172
  // Disable safelink for now, until packets are really not dropped
  // anymore.
  if (false /*useSafeLink*/) {

    const size_t numRequests = m_batchRequests.size();
    size_t remainingRequests = numRequests;
    size_t requestIdx = 0;

    while (remainingRequests > 0) {
      remainingRequests = numRequests - m_numRequestsFinished;
      // std::cout << "rR: " << remainingRequests << " " << m_numRequestsEnqueued << std::endl;
      // enqueue up to queue size
      while(m_numRequestsEnqueued < queueSize && requestIdx < numRequests) {
        const auto& request = m_batchRequests[requestIdx++];

        do {
          sendPacketInternal(request.request.data(), request.request.size(), ack, useSafeLink);
          handleBatchAck(ack, crtpMode);

          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSeconds = end-start;
          if (elapsedSeconds.count() > timeout) {
            throw std::runtime_error("timeout");
          }
          // std::cout << "send req " << requestIdx << std::endl;
        } while (!ack.ack);
        m_numRequestsEnqueued++;
      }
      // send ping's until at least one item in queue is done
      while(m_numRequestsEnqueued == queueSize
            || (m_numRequestsFinished < numRequests && requestIdx == numRequests)) {
        crtpEmpty ping;
        sendPacket(ping, ack, useSafeLink);
        handleBatchAck(ack, crtpMode);

        auto end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = end-start;
        if (elapsedSeconds.count() > timeout) {
          throw std::runtime_error("timeout");
        }
        // std::cout << "send ping " << m_numRequestsEnqueued << std::endl;
      }
    }
  } else {
    while (true) {
      if (!crtpMode || !sendPing) {
        for (const auto& request : m_batchRequests) {
          if (!request.finished) {
            // std::cout << "sendReq" << std::endl;
            sendPacketInternal(request.request.data(), request.request.size(), ack, useSafeLink);
            handleBatchAck(ack, crtpMode);
            auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsedSeconds = end-start;
            if (elapsedSeconds.count() > timeout) {
              throw std::runtime_error("timeout");
            }
          }
        }
        if (m_radio && !useSafeLink) {
          sendPing = true;
        }
      } else {
        size_t remainingRequests = m_batchRequests.size() - m_numRequestsFinished;
        for (size_t i = 0; i < remainingRequests; ++i) {
          crtpEmpty ping;
          sendPacket(ping, ack, useSafeLink);
          handleBatchAck(ack, crtpMode);
          // if (ack.ack && crtpPlatformRSSIAck::match(ack)) {
          //   sendPing = false;
          // }

          auto end = std::chrono::system_clock::now();
          std::chrono::duration<double> elapsedSeconds = end-start;
          if (elapsedSeconds.count() > timeout) {
            throw std::runtime_error("timeout");
          }
        }

        sendPing = false;
      }
      if (m_numRequestsFinished == m_batchRequests.size()) {
        break;
      }
    }
  }
}

void Crazyflie::handleBatchAck(
  const ITransport::Ack& ack,
  bool crtpMode)
{
  if (ack.ack) {
    for (auto& request : m_batchRequests) {
      if (crtpMode) {
        if ((crtp(ack.data[0]) == crtp(request.request[0]) || ack.data[0] == request.request[0])
            && memcmp(&ack.data[1], &request.request[1], request.numBytesToMatch) == 0
            && !request.finished) {
          request.ack = ack;
          request.finished = true;
          ++m_numRequestsFinished;
          --m_numRequestsEnqueued;
          // std::cout << "gotack" <<std::endl;
          return;
        }
      } else {
        if (!request.finished
            && memcmp(&ack.data[0], &request.request[0], request.numBytesToMatch) == 0) {
          request.ack = ack;
          request.finished = true;
          ++m_numRequestsFinished;
          --m_numRequestsEnqueued;
          // std::cout << m_numRequestsFinished / (float)m_batchRequests.size() * 100.0 << " %" << std::endl;
          return;
        }
      }
    }
    // ack is (also) handled in sendPacket
  }
}

void Crazyflie::setGroupMask(uint8_t groupMask)
{
  crtpCommanderHighLevelSetGroupMaskRequest request(groupMask);
  startBatchRequest();
  addRequest(request, 2);
  handleRequests();
}

void Crazyflie::takeoff(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelTakeoffRequest req(groupMask, height, duration);
  sendPacketOrTimeout(req);
}

void Crazyflie::land(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelLandRequest req(groupMask, height, duration);
  sendPacketOrTimeout(req);
}

void Crazyflie::stop(uint8_t groupMask)
{
  crtpCommanderHighLevelStopRequest req(groupMask);
  sendPacketOrTimeout(req);
}

void Crazyflie::goTo(float x, float y, float z, float yaw, float duration, bool relative, uint8_t groupMask)
{
  crtpCommanderHighLevelGoToRequest req(groupMask, relative, x, y, z, yaw, duration);
  sendPacketOrTimeout(req);
}

void Crazyflie::uploadTrajectory(
  uint8_t trajectoryId,
  uint32_t pieceOffset,
  const std::vector<poly4d>& pieces)
{
  for (const auto& entry : m_memoryTocEntries) {
    if (entry.type == MemoryTypeTRAJ) {
      startBatchRequest();
      // upload pieces
      size_t remainingBytes = sizeof(poly4d) * pieces.size();
      size_t numRequests = ceil(remainingBytes / 24.0f);
      for (size_t i = 0; i < numRequests; ++i) {
        crtpMemoryWriteRequest req(entry.id, pieceOffset * sizeof(poly4d) + i*24);
        size_t size = std::min<size_t>(remainingBytes, 24);
        memcpy(req.data, reinterpret_cast<const uint8_t*>(pieces.data()) + i * 24, size);
        remainingBytes -= size;
        addRequestInternal(reinterpret_cast<const uint8_t*>(&req), 6 + size, 5);
      }
      // define trajectory
      crtpCommanderHighLevelDefineTrajectoryRequest req(trajectoryId);
      req.description.trajectoryLocation = TRAJECTORY_LOCATION_MEM;
      req.description.trajectoryType = TRAJECTORY_TYPE_POLY4D;
      req.description.trajectoryIdentifier.mem.offset = pieceOffset * sizeof(poly4d);
      req.description.trajectoryIdentifier.mem.n_pieces = (uint8_t)pieces.size();
      addRequest(req, 2);
      handleRequests();
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
  sendPacketOrTimeout(req);
}

void Crazyflie::readUSDLogFile(
  std::vector<uint8_t>& data)
{
  for (const auto& entry : m_memoryTocEntries) {
    if (entry.type == MemoryTypeUSD) {
      startBatchRequest();
      size_t remainingBytes = entry.size;
      size_t numRequests = ceil(remainingBytes / 24.0f);
      for (size_t i = 0; i < numRequests; ++i) {
        size_t size = std::min<size_t>(remainingBytes, 24);
        crtpMemoryReadRequest req(entry.id, i*24, size);
        remainingBytes -= size;
        addRequest(req, 5);
      }
      handleRequests();
      // put result in data vector
      data.resize(entry.size);
      remainingBytes = entry.size;
      for (size_t i = 0; i < numRequests; ++i) {
        size_t size = std::min<size_t>(remainingBytes, 24);
        const crtpMemoryReadResponse* response = getRequestResult<crtpMemoryReadResponse>(i);
        memcpy(&data[i*24], response->data, size);
        remainingBytes -= size;
      }
      return;
    }
  }
  throw std::runtime_error("Could not find MemoryTypeUSD!");
}

////////////////////////////////////////////////////////////////

CrazyflieBroadcaster::CrazyflieBroadcaster(
  const std::string& link_uri)
  : m_radio(nullptr)
  , m_devId(0)
  , m_channel(0)
  , m_address(0)
  , m_datarate(Crazyradio::Datarate_250KPS)
{
  int datarate;
  int channel;
  char datarateType;
  bool success = false;

  success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c/%" SCNx64,
     &m_devId, &channel, &datarate,
     &datarateType, &m_address) == 5;
  if (!success) {
    success = std::sscanf(link_uri.c_str(), "radio://%d/%d/%d%c",
       &m_devId, &channel, &datarate,
       &datarateType) == 4;
    m_address = 0xE7E7E7E7E7;
  }

  if (success)
  {
    m_channel = channel;
    if (datarate == 250 && datarateType == 'K') {
      m_datarate = Crazyradio::Datarate_250KPS;
    }
    else if (datarate == 1 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_1MPS;
    }
    else if (datarate == 2 && datarateType == 'M') {
      m_datarate = Crazyradio::Datarate_2MPS;
    }

    if (m_devId >= MAX_RADIOS) {
      throw std::runtime_error("This version does not support that many radios. Adjust MAX_RADIOS and recompile!");
    }

    {
      std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
      if (!g_crazyradios[m_devId]) {
        g_crazyradios[m_devId] = new Crazyradio(m_devId);
        g_crazyradios[m_devId]->enableLogging(LOG_COMMUNICATION);
        // g_crazyradios[m_devId]->setAckEnable(false);
        g_crazyradios[m_devId]->setAckEnable(true);
        g_crazyradios[m_devId]->setArc(0);
      }
    }

    m_radio = g_crazyradios[m_devId];
  }
  else {
    throw std::runtime_error("Uri is not valid!");
  }
}

void CrazyflieBroadcaster::sendPacket(
  const uint8_t* data,
  uint32_t length)
{
  std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
  if (m_radio->getAddress() != m_address) {
    m_radio->setAddress(m_address);
  }
  if (m_radio->getChannel() != m_channel) {
    m_radio->setChannel(m_channel);
  }
  if (m_radio->getDatarate() != m_datarate) {
    m_radio->setDatarate(m_datarate);
  }
  if (m_radio->getAckEnable()) {
    m_radio->setAckEnable(false);
  }
  m_radio->sendPacketNoAck(data, length);
}

void CrazyflieBroadcaster::send2Packets(
  const uint8_t* data,
  uint32_t length)
{
  std::unique_lock<std::mutex> mlock(g_radioMutex[m_devId]);
  if (m_radio->getAddress() != m_address) {
    m_radio->setAddress(m_address);
  }
  if (m_radio->getChannel() != m_channel) {
    m_radio->setChannel(m_channel);
  }
  if (m_radio->getDatarate() != m_datarate) {
    m_radio->setDatarate(m_datarate);
  }
  if (m_radio->getAckEnable()) {
    m_radio->setAckEnable(false);
  }
  m_radio->send2PacketsNoAck(data, length);
}

void CrazyflieBroadcaster::takeoff(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelTakeoffRequest req(groupMask, height, duration);
  sendPacket((uint8_t*)&req, sizeof(req));
}

void CrazyflieBroadcaster::land(float height, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelLandRequest req(groupMask, height, duration);
  sendPacket((uint8_t*)&req, sizeof(req));
}

void CrazyflieBroadcaster::stop(uint8_t groupMask)
{
  crtpCommanderHighLevelStopRequest req(groupMask);
  sendPacket((uint8_t*)&req, sizeof(req));
}

// This is always in relative coordinates
void CrazyflieBroadcaster::goTo(float x, float y, float z, float yaw, float duration, uint8_t groupMask)
{
  crtpCommanderHighLevelGoToRequest req(groupMask, true, x, y, z, yaw, duration);
  sendPacket((uint8_t*)&req, sizeof(req));
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
  sendPacket((uint8_t*)&req, sizeof(req));
}

void CrazyflieBroadcaster::sendExternalPositions(
  const std::vector<externalPosition>& data)
{
  if (data.size() == 0) {
    return;
  }

  std::vector<crtpExternalPositionPacked> requests(ceil(data.size() / 4.0));
  for (size_t i = 0; i < data.size(); ++i) {
    size_t j = i / 4;
    requests[j].positions[i%4].id = data[i].id;
    requests[j].positions[i%4].x = data[i].x * 1000;
    requests[j].positions[i%4].y = data[i].y * 1000;
    requests[j].positions[i%4].z = data[i].z * 1000;
  }
  // 1 header byte per packet; 7 bytes for each position
  size_t numBytes = requests.size() + data.size() * 7;

  size_t remainingRequests = requests.size();
  size_t i = 0;
  while (remainingRequests > 0) {
    // the crazyradio requires the two packets to be the same size
    // -> only send2packets if this is possible
    if (   remainingRequests >= 2
        && numBytes >= 2 * sizeof(crtpExternalPositionPacked)) {
      size_t size = std::min(numBytes, 2 * sizeof(crtpExternalPositionPacked));
      send2Packets(reinterpret_cast<const uint8_t*>(&requests[i]), size);
      remainingRequests -= 2;
      numBytes -= size;
      i += 2;
    } else {
      size_t size = std::min(numBytes, sizeof(crtpExternalPositionPacked));
      sendPacket(reinterpret_cast<const uint8_t*>(&requests[i]), size);
      remainingRequests -= 1;
      numBytes -= size;
      i += 1;
    }
  }
  // assert(numBytes == 0);
}

void CrazyflieBroadcaster::emergencyStop()
{
  crtpEmergencyStopRequest req;
  sendPacket((uint8_t*)&req, sizeof(req));
}

void CrazyflieBroadcaster::emergencyStopWatchdog()
{
  crtpEmergencyStopWatchdogRequest req;
  sendPacket((uint8_t*)&req, sizeof(req));
}

// assumes input quaternion is normalized. will fail if not.
static inline uint32_t quatcompress(float const q[4])
{
  // we send the values of the quaternion's smallest 3 elements.
  unsigned i_largest = 0;
  for (unsigned i = 1; i < 4; ++i) {
    if (fabsf(q[i]) > fabsf(q[i_largest])) {
      i_largest = i;
    }
  }

  // since -q represents the same rotation as q,
  // transform the quaternion so the largest element is positive.
  // this avoids having to send its sign bit.
  unsigned negate = q[i_largest] < 0;

  // 1/sqrt(2) is the largest possible value
  // of the second-largest element in a unit quaternion.

  // do compression using sign bit and 9-bit precision per element.
  uint32_t comp = i_largest;
  for (unsigned i = 0; i < 4; ++i) {
    if (i != i_largest) {
      unsigned negbit = (q[i] < 0) ^ negate;
      unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / (float)M_SQRT1_2) + 0.5f;
      comp = (comp << 10) | (negbit << 9) | mag;
    }
  }

  return comp;
}

void CrazyflieBroadcaster::sendExternalPoses(
  const std::vector<externalPose>& data)
{
  if (data.size() == 0) {
    return;
  }

  std::vector<crtpExternalPosePacked> requests(ceil(data.size() / 2.0));
  for (size_t i = 0; i < data.size(); ++i) {
    size_t j = i / 2;
    requests[j].poses[i%2].id = data[i].id;
    requests[j].poses[i%2].x = data[i].x * 1000;
    requests[j].poses[i%2].y = data[i].y * 1000;
    requests[j].poses[i%2].z = data[i].z * 1000;
    float q[4] = { data[i].qx, data[i].qy, data[i].qz, data[i].qw };
    requests[j].poses[i%2].quat = quatcompress(q);
  }
  // 2 header byte per packet; 11 bytes for each position
  size_t numBytes = requests.size() * 2 + data.size() * 11;

  size_t remainingRequests = requests.size();
  size_t i = 0;
  while (remainingRequests > 0) {
    // the crazyradio requires the two packets to be the same size
    // -> only send2packets if this is possible
    if (   remainingRequests >= 2
        && numBytes >= 2 * sizeof(crtpExternalPosePacked)) {
      size_t size = std::min(numBytes, 2 * sizeof(crtpExternalPosePacked));
      send2Packets(reinterpret_cast<const uint8_t*>(&requests[i]), size);
      remainingRequests -= 2;
      numBytes -= size;
      i += 2;
    } else {
      size_t size = std::min(numBytes, sizeof(crtpExternalPosePacked));
      sendPacket(reinterpret_cast<const uint8_t*>(&requests[i]), size);
      remainingRequests -= 1;
      numBytes -= size;
      i += 1;
    }
  }
  // assert(numBytes == 0);
}

// void CrazyflieBroadcaster::setParam(
//   uint8_t group,
//   uint8_t id,
//   Crazyflie::ParamType type,
//   const Crazyflie::ParamValue& value) {

//   switch (type) {
//     case Crazyflie::ParamTypeUint8:
//       {
//         crtpParamWriteBroadcastRequest<uint8_t> request(group, id, value.valueUint8);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//     case Crazyflie::ParamTypeInt8:
//       {
//         crtpParamWriteBroadcastRequest<int8_t> request(group, id, value.valueInt8);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//     case Crazyflie::ParamTypeUint16:
//       {
//         crtpParamWriteBroadcastRequest<uint16_t> request(group, id, value.valueUint16);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//     case Crazyflie::ParamTypeInt16:
//       {
//         crtpParamWriteBroadcastRequest<int16_t> request(group, id, value.valueInt16);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//     case Crazyflie::ParamTypeUint32:
//       {
//         crtpParamWriteBroadcastRequest<uint32_t> request(group, id, value.valueUint32);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//     case Crazyflie::ParamTypeInt32:
//       {
//         crtpParamWriteBroadcastRequest<int32_t> request(group, id, value.valueInt32);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//     case Crazyflie::ParamTypeFloat:
//       {
//         crtpParamWriteBroadcastRequest<float> request(group, id, value.valueFloat);
//         sendPacket((const uint8_t*)&request, sizeof(request));
//         break;
//       }
//   }
//   // TODO: technically we should update the internal copy of the value of each CF object
// }
