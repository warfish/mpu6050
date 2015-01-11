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

// Convert 16 bit big-endian value from MPU into 16-bit little-endian AVR value
#define BE16_TO_CPU(_be16_) (_be16_) = (((_be16_) & 0x00FF) << 8) | (((_be16_) & 0xFF00) >> 8)

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
static void MpuReadBuffer(int addr, int bytes, byte* data)
{
  int res = 0;
    
  MPU_ASSERT(data);
  
  if (bytes == 0)
  {
    return;
  }
  
  Wire.beginTransmission(MPU_I2C_ADDR);
  
  res = Wire.write(addr);
  MPU_ASSERT(res == 1);
  
  res = Wire.endTransmission(false);
  MPU_ASSERT(res == 0);
  
  Wire.requestFrom(MPU_I2C_ADDR, bytes, true); // true means release I2C bus after request is completed
  MPU_ASSERT(Wire.available() == bytes);
  while(bytes > 0)
  {
    *data = Wire.read();
    ++data;
    --bytes;
  }
}
  
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

// Read MPU sensor report data
static void MpuReadSensorData(struct MpuSensorData* data)
{
  MPU_ASSERT(data != NULL);
  
  MpuReadBuffer(MPU_ACCEL_XOUT_H, sizeof(MpuSensorData), (byte*)data);
  
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
#ifdef MPU_DEBUG
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
    MpuWriteReg(MPU_GYRO_CONFIG, MpuReadReg(MPU_GYRO_CONFIG) & 0xE0);
    MpuReadSensorData(&responce1);
  
    // Deactivate gyro self-test and read response
    MpuWriteReg(MPU_GYRO_CONFIG, MpuReadReg(MPU_GYRO_CONFIG) & ~0xE0);
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
      
       Serial.print("Gyro self test axis "); Serial.print(i); Serial.println(":");
       Serial.print("ft = "); Serial.println(ft);
       Serial.print("delta = "); Serial.println(delta);

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
    MpuWriteReg(MPU_ACCEL_CONFIG, MpuReadReg(MPU_ACCEL_CONFIG) & 0xE0);
    MpuReadSensorData(&responce1);
  
    // Deactivate accel self-test and read response
    MpuWriteReg(MPU_ACCEL_CONFIG, MpuReadReg(MPU_ACCEL_CONFIG) & ~0xE0);
    MpuReadSensorData(&responce2);
  
    // For each accel axis calc factory trim value deviation
    int16_t axis_values_1[3] = {responce1.x_accel, responce1.y_accel, responce1.z_accel};
    int16_t axis_values_2[3] = {responce2.x_accel, responce2.y_accel, responce2.z_accel};
    for (int i = 0; i < 3; ++i)
    {
      double ft = (accl_regs[i] == 0 ? 0 : (4096.0 * 0.34 * pow(2.70, (accl_regs[i] - 1.0) / 30.0)));  
      double delta = ((double)(axis_values_1[i] - axis_values_2[i]) - ft) / ft;
      
      Serial.print("Aceel self test axis "); Serial.print(i); Serial.println(":");
      Serial.print("ft = "); Serial.println(ft);
      Serial.print("delta = "); Serial.println(delta);
      
      if (delta < -14.0 || delta > 14.0)
      {
        Serial.print("MPU accel factory deviation for self test axis "); Serial.println("i"); Serial.flush();
        res = -1;
      }
    }
  }
  
  if (res == 0)
  {
    Serial.println("MPU self test successful");
  }
  else
  {
    Serial.println("MPU self test failed");
  }
  
  Serial.flush();
  return res;
#else
  return 0;
#endif // MPU_DEBUG
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  byte val = 0;
  
  // Detect MPU
  val = MpuReadReg(MPU_WHO_AM_I);
  MPU_ASSERT(val == 0x68);

  // Start sensor
  MpuWriteReg(MPU_PWG_MGMT_1, 0);
  
  // Self-test
  val = MpuSelfTest();
  MPU_ASSERT((val == 0) && "MPU self test failed");
}

void loop()
{
  MpuSensorData data;
  MpuReadSensorData(&data);
  MpuDumpSensorData(&data);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

