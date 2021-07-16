#include <stdint.h>
#include <math.h>
#include <cstring>
#include "crtp.h"

// Note: the quaternion compression code is copied from
// github.com/jpreiss/quatcompress

// compress a unit quaternion into 32 bits.
// assumes input quaternion is normalized. will fail if not.
uint32_t quatcompress(float const q[4])
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
	float const SMALL_MAX = 1.0 / sqrt(2);

	// do compression using sign bit and 9-bit precision per element.
	uint32_t comp = i_largest;
	for (unsigned i = 0; i < 4; ++i) {
		if (i != i_largest) {
			unsigned negbit = (q[i] < 0) ^ negate;
			unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / SMALL_MAX) + 0.5f;
			comp = (comp << 10) | (negbit << 9) | mag;
		}
	}

	return comp;
}

// This is the matching function to decompress
// decompress a quaternion from 32 bit compressed representation.
void quatdecompress(uint32_t comp, float q[4])
{
	float const SMALL_MAX = 1.0 / sqrt(2);
	unsigned const mask = (1 << 9) - 1;

	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = SMALL_MAX * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}
	q[i_largest] = sqrtf(1.0f - sum_squares);
}

// Port 0 (Console)

bool crtpConsoleResponse::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.port() == 0 &&
         p.channel() == 0 &&
         p.payloadSize() > 0;
}

std::string crtpConsoleResponse::text(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.payloadAtString(0);
}

// Port 2 (Parameters)

crtpParamTocGetItemV2Request::crtpParamTocGetItemV2Request(uint16_t id)
	: Packet(2, 0, 3)
{
	setPayloadAt<uint8_t>(0, 2);
	setPayloadAt<uint16_t>(1, id);
}

bool crtpParamTocGetItemV2Response::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.port() == 2 &&
		   p.channel() == 0 &&
		   p.payloadSize() > 3 &&
		   p.payloadAt<uint8_t>(0) == 2;
}

uint16_t crtpParamTocGetItemV2Response::id(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint16_t>(1);
}

ParamType crtpParamTocGetItemV2Response::type(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return (ParamType)(p.payloadAt<uint8_t>(3) & 0xF);
}

bool crtpParamTocGetItemV2Response::readonly(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return (p.payloadAt<uint8_t>(3) >> 6) & 0x1;
}

std::pair<std::string, std::string> crtpParamTocGetItemV2Response::groupAndName(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	auto group =  p.payloadAtString(4);
	auto name = p.payloadAtString(4+group.length()+1);
	return std::make_pair(group, name);
}


crtpParamTocGetInfoV2Request::crtpParamTocGetInfoV2Request()
	: Packet(2, 0, 1)
{
	setPayloadAt<uint8_t>(0, 3);
}

bool crtpParamTocGetInfoV2Response::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.port() == 2 &&
		   p.channel() == 0 &&
		   p.payloadSize() == 7 &&
		   p.payloadAt<uint8_t>(0) == 3;
}

uint16_t crtpParamTocGetInfoV2Response::numParams(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<int>(1);
}

uint32_t crtpParamTocGetInfoV2Response::crc(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<int>(1);
}


crtpParamReadV2Request::crtpParamReadV2Request(uint16_t id)
	: Packet(2, 1, 2)
{
	setPayloadAt<uint16_t>(0, id);
}

bool crtpParamValueV2Response::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.port() == 2 &&
		   (p.channel() == 1 || p.channel() == 2) &&
		   p.payloadSize() > 2;
}

uint16_t crtpParamValueV2Response::id(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint16_t>(0);
}

uint8_t crtpParamValueV2Response::status(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint8_t>(2);
}

// Port 5 (Logging)

crtpLogGetInfoV2Request::crtpLogGetInfoV2Request()
	: Packet(5, 0, 1)
{
	setPayloadAt<uint8_t>(0, 3);
}

bool crtpLogGetInfoV2Response::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.port() == 5 &&
		   p.channel() == 0 &&
		   p.payloadSize() == 9 &&
		   p.payloadAt<uint8_t>(0) == 3;
}

uint16_t crtpLogGetInfoV2Response::numLogVariables(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint16_t>(1);
}

uint32_t crtpLogGetInfoV2Response::crc(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint32_t>(3);
}

uint8_t crtpLogGetInfoV2Response::numMaxLogBlocks(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint8_t>(7);
}

uint8_t crtpLogGetInfoV2Response::numMaxOps(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint8_t>(8);
}

crtpLogGetItemV2Request::crtpLogGetItemV2Request(uint16_t id)
	: Packet(5, 0, 3)
{
	setPayloadAt<uint8_t>(0, 2);
	setPayloadAt<uint16_t>(1, id);
}

bool crtpLogGetItemV2Response::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.port() == 5 &&
		   p.channel() == 0 &&
		   p.payloadSize() > 3 &&
		   p.payloadAt<uint8_t>(0) == 2;
}

uint16_t crtpLogGetItemV2Response::id(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint16_t>(1);
}

uint8_t crtpLogGetItemV2Response::type(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	return p.payloadAt<uint8_t>(3);
}

std::pair<std::string, std::string> crtpLogGetItemV2Response::groupAndName(const bitcraze::crazyflieLinkCpp::Packet &p)
{
	auto group = p.payloadAtString(4);
	auto name = p.payloadAtString(4 + group.length() + 1);
	return std::make_pair(group, name);
}

// Port 13 (Platform)

crtpGetProtocolVersionRequest::crtpGetProtocolVersionRequest()
  : Packet(13, 1, 1)
{
  setPayloadAt<uint8_t>(0, 0);
}

bool crtpGetProtocolVersionResponse::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return  p.port() == 13 &&
          p.channel() == 1 &&
          p.payloadSize() == 5 &&
          p.payloadAt<uint8_t>(0) == 0;
}

int crtpGetProtocolVersionResponse::version(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.payloadAt<int>(1);
}

crtpGetFirmwareVersionRequest::crtpGetFirmwareVersionRequest()
    : Packet(13, 1, 1)
{
  setPayloadAt<uint8_t>(0, 1);
}

bool crtpGetFirmwareVersionResponse::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.port() == 13 &&
         p.channel() == 1 &&
         p.payloadSize() > 1 &&
         p.payloadAt<uint8_t>(0) == 1;
}

std::string crtpGetFirmwareVersionResponse::version(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.payloadAtString(1);
}

crtpGetDeviceTypeNameRequest::crtpGetDeviceTypeNameRequest()
    : Packet(13, 1, 1)
{
  setPayloadAt<uint8_t>(0, 2);
}

bool crtpGetDeviceTypeNameResponse::valid(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.port() == 13 &&
         p.channel() == 1 &&
         p.payloadSize() > 1 &&
         p.payloadAt<uint8_t>(0) == 2;
}

std::string crtpGetDeviceTypeNameResponse::name(const bitcraze::crazyflieLinkCpp::Packet &p)
{
  return p.payloadAtString(1);
}

#if 0
crtpFullStateSetpointRequest::crtpFullStateSetpointRequest(
  float x, float y, float z,
  float vx, float vy, float vz,
  float ax, float ay, float az,
  float qx, float qy, float qz, float qw,
  float rollRate, float pitchRate, float yawRate)
  : header(0x07, 0), type(6)
{
	float s = 1000.0;
	this->x = s * x;
	this->y = s * y;
	this->z = s * z;
	this->vx = s * vx;
	this->vy = s * vy;
	this->vz = s * vz;
	this->ax = s * ax;
	this->ay = s * ay;
	this->az = s * az;

	float q[4] = { qx, qy, qz, qw };
	this->quat = quatcompress(q);
	this->omegax = s * rollRate;
	this->omegay = s * pitchRate;
	this->omegaz = s * yawRate;
}

crtpStopRequest::crtpStopRequest()
	: header(0X07, 0), type(0)
{}

// m/s for velocity, deg/s for yawrate, and m for zDistance
crtpHoverSetpointRequest::crtpHoverSetpointRequest(
	float vx,
	float vy,
	float yawrate,
	float zDistance)
	: header(0X07, 0), type(5)
{
	this->vx = vx;
	this->vy = vy;
	this->yawrate = yawrate;
	this->zDistance = zDistance;
}

// m in position, degree in yaw
crtpPositionSetpointRequest::crtpPositionSetpointRequest(
	float x,
	float y,
	float z,
	float yaw)
	: header(0X07, 0), type(7)
{
	this->x = x;
	this->y = y;
	this->z = z;
	this->yaw = yaw;
}

template<class T>
crtpParamSetByNameRequest<T>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	uint8_t paramType,
	const void* value,
	uint8_t valueSize)
	: header(2,3)
{
	size_t groupLen = strlen(group);
	size_t nameLen = strlen(name);
	// TODO: do size checking here...

	int idx = 0;
	// first insert group (followed by \0)
	memcpy(&data[idx], group, groupLen + 1);
	idx += groupLen + 1;
	// insert name (followed by \0)
	memcpy(&data[idx], name, nameLen + 1);
	idx += nameLen + 1;
	// insert type
	data[idx] = paramType;
	idx++;
	// insert value
	memcpy(&data[idx], value, valueSize);
	idx += valueSize;

	size_ = idx + 2;
	responseSize_ = 1+groupLen+1+nameLen+1+1;
}

template <>
crtpParamSetByNameRequest<uint8_t>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const uint8_t& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeUint8, &value, sizeof(uint8_t))
{
}

template <>
crtpParamSetByNameRequest<int8_t>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const int8_t& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeInt8, &value, sizeof(int8_t))
{
}

template <>
crtpParamSetByNameRequest<uint16_t>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const uint16_t& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeUint16, &value, sizeof(uint16_t))
{
}

template <>
crtpParamSetByNameRequest<int16_t>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const int16_t& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeInt16, &value, sizeof(int16_t))
{
}

template <>
crtpParamSetByNameRequest<uint32_t>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const uint32_t& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeUint32, &value, sizeof(uint32_t))
{
}

template <>
crtpParamSetByNameRequest<int32_t>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const int32_t& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeInt32, &value, sizeof(int32_t))
{
}

template <>
crtpParamSetByNameRequest<float>::crtpParamSetByNameRequest(
	const char* group,
	const char* name,
	const float& value)
	: crtpParamSetByNameRequest(group, name, ParamTypeFloat, &value, sizeof(float))
{
}
#endif