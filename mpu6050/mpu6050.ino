//
// MPU-6050 gyro + accelerometer
// 
// Useful links:
// http://www.invensense.com/mems/gyro/documents/RM-MPU-6000A-00v4.2.pdf
// http://www.invensense.com/mems/gyro/documents/PS-MPU-6000A-00v3.4.pdf
// 

#include <Wire.h>

// Undefine to reduce image size
#define MPU_DEBUG

#ifdef MPU_DEBUG
static void _assert(unsigned long line, const char* cond)
{
  Serial.print("Assertion failed: ");
  Serial.print(cond);
  Serial.print(", line ");
  Serial.print(line, DEC);
  Serial.println();
  Serial.flush();
  exit(-1);
}
#  define MPU_ASSERT(_cond_) do { if ((int)(_cond_) == 0) _assert(__LINE__, #_cond_); } while(0);
#else
#  define MPU_ASSERT(_cond_)
#endif // MPU_DEBUG

// Chip I2C slave address
#define MPU_I2C_ADDR      0x68

// Some of the chip's registers. 
// We only define what we will use, refer to http://www.invensense.com/mems/gyro/documents/RM-MPU-6000A-00v4.2.pdf for a complete spec
#define MPU_WHO_AM_I      0x75
#define MPU_PWG_MGMT_1    0x6B
#define MPU_GYRO_CONFIG   0x1B
#define MPU_ACCEL_CONFIG  0x1C
#define MPU_SELF_TEST_X   0x0D
#define MPU_SELF_TEST_Y   0x0E
#define MPU_SELF_TEST_Z   0x0F
#define MPU_SELF_TEST_A   0x10
#define MPU_ACCEL_XOUT_H  0x3B
#define MPU_ACCEL_XOUT_L  0x3C
#define MPU_ACCEL_YOUT_H  0x3D
#define MPU_ACCEL_YOUT_L  0x3E
#define MPU_ACCEL_ZOUT_H  0x3F
#define MPU_ACCEL_ZOUT_L  0x40
#define MPU_TEMP_OUT_H    0x41
#define MPU_TEMP_OUT_L    0x42
#define MPU_GYRO_XOUT_H   0x43
#define MPU_GYRO_XOUT_L   0x44
#define MPU_GYRO_YOUT_H   0x45
#define MPU_GYRO_YOUT_L   0x46
#define MPU_GYRO_ZOUT_H   0x47
#define MPU_GYRO_ZOUT_L   0x48
#define MPU_INT_STATUS    0x3A
#define MPU_INT_ENABLE    0x38
#define MPU_INT_PIN_CFG   0x37

// Convert 16 bit big-endian value from MPU into 16-bit little-endian AVR value
#define BE16_TO_CPU(_be16_) (_be16_) = (((_be16_) & 0x00FF) << 8) | (((_be16_) & 0xFF00) >> 8)

// Sensor data report
struct MpuSensorData
{
  int16_t x_accel;
  int16_t y_accel;
  int16_t z_accel;
  int16_t temp;
  int16_t x_gyro;
  int16_t y_gyro;
  int16_t z_gyro;
} __attribute__((packed));

///////////////////////////////////////////////////////////////////////////////////////////////////////

// Precalibrated offset bases
float    g_ax_base = 0;
float    g_ay_base = 0;
float    g_az_base = 0;
float    g_gx_base = -193.83;
float    g_gy_base = 52.24;
float    g_gz_base = -197.19;

// Read single MPU register
static byte MpuReadReg(int addr)
{
  int res = 0;
  
  // Put register address on I2C bus
  Wire.beginTransmission(MPU_I2C_ADDR);
  
  res = Wire.write(addr);
  MPU_ASSERT(res == 1);
  
  res = Wire.endTransmission(false);
  MPU_ASSERT(res == 0);
  
  Wire.requestFrom(MPU_I2C_ADDR, 1, true); // true means release I2C bus after request is completed
  MPU_ASSERT(Wire.available() != 0);
  return Wire.read();
}

// Write single MPU register
static void MpuWriteReg(int addr, byte val)
{
  int res = 0;
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  
  res = Wire.write(addr);
  MPU_ASSERT(res == 1);
  
  res = Wire.write(val);
  MPU_ASSERT(res == 1);
    
  res = Wire.endTransmission(true);
  MPU_ASSERT(res == 0);
}
  
// Read a number of registers starting from addr into caller buffer
// Returns number of bytes read
static int MpuReadBuffer(int addr, int bytes, byte* data)
{
  int res = 0;
    
  MPU_ASSERT(data);
  
  if (bytes == 0)
  {
    return 0;
  }
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  
  res = Wire.write(addr);
  MPU_ASSERT(res == 1);
  
  res = Wire.endTransmission(false);
  MPU_ASSERT(res == 0);
  
  Wire.requestFrom(MPU_I2C_ADDR, bytes, true); // true means release I2C bus after request is completed
  
  int bytesRead = 0;
  while(bytesRead != bytes)
  {
    if (Wire.available() == 0) {
      break;
    }
    
    *data++ = Wire.read();
    ++bytesRead;
  }
  
  return bytesRead;
}
  
// Read MPU sensor report data
static void MpuReadSensorData(struct MpuSensorData* data)
{
  MPU_ASSERT(data != NULL);
  
  int bytesRead = 0;
  do 
  {
    bytesRead = MpuReadBuffer(MPU_ACCEL_XOUT_H, sizeof(MpuSensorData), (byte*)data);
  } 
  while (bytesRead != sizeof(MpuSensorData));
  
  // AVR is Little Endian and we read data in as Big endian
  BE16_TO_CPU(data->x_accel);
  BE16_TO_CPU(data->y_accel);
  BE16_TO_CPU(data->z_accel);
  BE16_TO_CPU(data->temp);
  BE16_TO_CPU(data->x_gyro);
  BE16_TO_CPU(data->y_gyro);
  BE16_TO_CPU(data->z_gyro);
}

// Dump sensor data into serial port
static void MpuDumpSensorData(const struct MpuSensorData* data)
{
  Serial.println("");
  Serial.print("Accl {"); Serial.print(data->x_accel); Serial.print(", "); Serial.print(data->y_accel); Serial.print(", "); Serial.print(data->z_accel); Serial.println(" }"); 
  Serial.print("Gyro {"); Serial.print(data->x_gyro); Serial.print(", "); Serial.print(data->y_gyro); Serial.print(", "); Serial.print(data->z_gyro); Serial.println(" }");
  Serial.print("Temp "); Serial.println(data->temp/340.00+36.53);
  Serial.flush();  
}

// Perform MPU self test.
static int MpuSelfTest()
{
  int res = 0;
  
  // Read constant self test reg values
  const uint8_t test_x = MpuReadReg(MPU_SELF_TEST_X);
  const uint8_t test_y = MpuReadReg(MPU_SELF_TEST_Y);
  const uint8_t test_z = MpuReadReg(MPU_SELF_TEST_Z);
  const uint8_t test_a = MpuReadReg(MPU_SELF_TEST_A);

  const uint8_t gyro_regs[3] = 
  { 
    test_x & 0x1F, 
    test_y & 0x1F, 
    test_z & 0x1F, 
  };
    
  const uint8_t accl_regs[3] = 
  { 
    ((test_x >> 3) & 0x1C) | ((test_a >> 4) & 0x03), 
    ((test_y >> 3) & 0x1C) | ((test_a >> 2) & 0x03),
    ((test_z >> 3) & 0x1C) | ((test_a >> 0) & 0x03),
  };

  MpuSensorData responce1;
  MpuSensorData responce2;
    
  //
  // Accel self-test
  //
  
  {
    // Activate gyro self-test and read responce
    uint8_t gyro_cfg = MpuReadReg(MPU_GYRO_CONFIG);
    MpuWriteReg(MPU_GYRO_CONFIG, 0xE0);
    MpuReadSensorData(&responce1);
  
    // Deactivate gyro self-test and read response
    MpuWriteReg(MPU_GYRO_CONFIG, gyro_cfg);
    MpuReadSensorData(&responce2);
  
    // For each gyro axis calc factory trim value deviation
    int16_t axis_values_1[3] = {responce1.x_gyro, responce1.y_gyro, responce1.z_gyro};
    int16_t axis_values_2[3] = {responce2.x_gyro, responce2.y_gyro, responce2.z_gyro};
    for (int i = 0; i < 3; ++i)
    {
       double ft = 0;
       if (i == 1)
       {
         // For Y axis this formula is slightly different
         ft = (gyro_regs[i] == 0 ? 0 : (-25.0 * 131.0 * pow(1.046, gyro_regs[i] - 1)));
       }
       else
       {
         ft = (gyro_regs[i] == 0 ? 0 : (25.0 * 131.0 * pow(1.046, gyro_regs[i] - 1)));
       }

       MPU_ASSERT(ft != 0);      
       double delta = ((double)(axis_values_1[i] - axis_values_2[i]) - ft) / ft;
      
       if (delta < -14.0 || delta > 14.0)
       {
         Serial.print("MPU gyro factory deviation for self test axis "); Serial.println("i"); Serial.flush();
         res = -1;
       }
    }
  }
  
  //
  // Accel self-test
  //

  {
    // Activate accel self-test and read responce
    uint8_t accel_cfg = MpuReadReg(MPU_ACCEL_CONFIG);
    MpuWriteReg(MPU_ACCEL_CONFIG, 0xF0);
    MpuReadSensorData(&responce1);
  
    // Deactivate accel self-test and read response
    MpuWriteReg(MPU_ACCEL_CONFIG, accel_cfg);
    MpuReadSensorData(&responce2);
    
    // For each accel axis calc factory trim value deviation
    int16_t axis_values_1[3] = {responce1.x_accel, responce1.y_accel, responce1.z_accel};
    int16_t axis_values_2[3] = {responce2.x_accel, responce2.y_accel, responce2.z_accel};
    for (int i = 0; i < 3; ++i)
    {
      double ft = (accl_regs[i] == 0 ? 0 : (4096.0 * 0.34 * pow(2.70, (accl_regs[i] - 1.0) / 30.0)));  
      double delta = ((double)(axis_values_1[i] - axis_values_2[i]) - ft) / ft;
      
      if (delta < -14.0 || delta > 14.0)
      {
        Serial.print("MPU accel factory deviation for self test axis "); Serial.println("i"); Serial.flush();
        res = -1;
      }
    }
  }
  
  return res;
}

// The sensor should be motionless on a horizontal surface 
//  while calibration is happening
static void MpuCalibrate() 
{
  int                   num_readings = 100;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  MpuSensorData         sensorData;
  
  Serial.println("Starting Calibration");

  // Discard the first set of values read from the IMU
  MpuReadSensorData(&sensorData);
  
  // Read and average the raw values from the IMU
  for (int i = 0; i < num_readings; i++) 
  {
    MpuReadSensorData(&sensorData);
    x_accel += sensorData.x_accel;
    y_accel += sensorData.y_accel;
    z_accel += sensorData.z_accel;
    x_gyro += sensorData.x_gyro;
    y_gyro += sensorData.y_gyro;
    z_gyro += sensorData.z_gyro;
    delay(333);
  }
  
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;
  
  // Store the raw calibration values globally
  g_ax_base = x_accel;
  g_ay_base = y_accel;
  g_az_base = z_accel;
  g_gx_base = x_gyro;
  g_gy_base = y_gyro;
  g_gz_base = z_gyro;
  
  Serial.println("Finishing Calibration");
  Serial.print("xa = "); Serial.println(g_ax_base);
  Serial.print("ya = "); Serial.println(g_ay_base);
  Serial.print("za = "); Serial.println(g_az_base);
  Serial.print("xg = "); Serial.println(g_gx_base);
  Serial.print("yg = "); Serial.println(g_gy_base);
  Serial.print("zg = "); Serial.println(g_gz_base);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

static uint32_t crc32_tab[] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3, 
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988,	0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91, 
	0x1db71064, 0x6ab020f2,	0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5, 
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172,	0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b, 
	0x35b5a8fa, 0x42b2986c,	0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423,	0xcfba9599, 0xb8bda50f, 
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924,	0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d, 
	0x76dc4190, 0x01db7106,	0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d,	0x91646c97, 0xe6635c01, 
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e,	0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457, 
	0x65b0d9c6, 0x12b7e950,	0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7,	0xa4d1c46d, 0xd3d6f4fb, 
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0,	0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9, 
	0x5005713c, 0x270241aa,	0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81,	0xb7bd5c3b, 0xc0ba6cad, 
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a,	0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683, 
	0xe3630b12, 0x94643b84,	0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb,	0x196c3671, 0x6e6b06e7, 
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc,	0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5, 
	0xd6d6a3e8, 0xa1d1937e,	0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55,	0x316e8eef, 0x4669be79, 
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236,	0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f, 
	0xc5ba3bbe, 0xb2bd0b28,	0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f,	0x72076785, 0x05005713, 
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38,	0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21, 
	0x86d3d2d4, 0xf1d4e242,	0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69,	0x616bffd3, 0x166ccf45, 
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2,	0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db, 
	0xaed16a4a, 0xd9d65adc,	0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693,	0x54de5729, 0x23d967bf, 
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94,	0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

uint32_t crc32(const char *buf, size_t len)
{
	uint32_t crc = 0;
	const char *p, *q;
	uint8_t octet;

	crc = ~crc;
	q = buf + len;
	for (p = buf; p < q; p++) {
		octet = *p; 
		crc = (crc >> 8) ^ crc32_tab[(crc & 0xff) ^ octet];
	}
	return ~crc;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

#define MPU_REPORT_SIGNATURE 0xDEADF00D

struct MpuReport
{
  uint32_t signature; // MPU_REPORT_SIGNATURE
  uint32_t size;
  uint32_t crc;
  float x_angle;
  float y_angle;
  float z_angle;
} __attribute__((packed));

static float gyroXangle = 180.0;
static float gyroYangle = 180.0;
static float gyroZangle = 180.0;
static unsigned long timer = 0;

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  byte val = 0;
  
  // Detect MPU
  val = MpuReadReg(MPU_WHO_AM_I);
  MPU_ASSERT(val == 0x68);

  // Set gyro full scale range to 250
  MpuWriteReg(MPU_GYRO_CONFIG, 0);
 
  // Set accel full scale range to 2g
  MpuWriteReg(MPU_ACCEL_CONFIG, 0); 
  
  // Start sensor
  MpuWriteReg(MPU_PWG_MGMT_1, 0);
  
  // Self-test
  val = MpuSelfTest();
  MPU_ASSERT((val == 0) && "MPU self test failed");
  
  timer = micros();
}

void loop()
{
  // Read raw report and apply offset correction
  MpuSensorData data;
  MpuReadSensorData(&data);
  data.x_accel -= g_ax_base;
  data.y_accel -= g_ay_base;
  data.z_accel -= g_az_base;
  data.x_gyro -= g_gx_base;
  data.y_gyro -= g_gy_base;
  data.z_gyro -= g_gz_base;

  // Transform into angles
  float gyroXrate = (float)data.x_gyro/131.0;
  float gyroYrate = -((float)data.y_gyro/131.0);
  float gyroZrate = (float)data.z_gyro/131.0;
  gyroXangle += gyroXrate*((float)(micros()-timer)/1000000); // Calculate gyro angle without any filter  
  gyroYangle += gyroYrate*((float)(micros()-timer)/1000000);
  gyroZangle += gyroZrate*((float)(micros()-timer)/1000000);
  
  // Send report to host
  MpuReport report;
  report.signature = MPU_REPORT_SIGNATURE;
  report.size = sizeof(report);
  report.crc = 0;
  report.x_angle = gyroXangle;
  report.y_angle = gyroYangle;
  report.z_angle = gyroZangle;
  report.crc = crc32((const char*)&report, sizeof(report));
  Serial.write((byte*)&report, sizeof(report));
 
  // Continue 
  timer = micros();
  delay(100);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

