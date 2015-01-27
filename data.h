//
// Common data definitions for both test app and Arduino.
//

#ifndef _MPU6050_DATA_H_
#define _MPU6050_DATA_H_

#include <inttypes.h>

#define MPU_REPORT_SIGNATURE 0xDEADF00D

#pragma pack(push, 1)

/*
struct MpuSensorData
{
	int16_t x_accel;
	int16_t y_accel;
	int16_t z_accel;
	int16_t temp;
	int16_t x_gyro;
	int16_t y_gyro;
	int16_t z_gyro;
};
*/

struct MpuReport
{
	uint32_t signature; // MPU_REPORT_SIGNATURE
	uint32_t size;
	uint32_t crc;
	float x_angle;
	float y_angle;
	float z_angle;
//	struct MpuSensorData data;
};

#pragma pack(pop)

// Implemented in crc32.c
uint32_t crc32(const void *buf, size_t size);

#endif

