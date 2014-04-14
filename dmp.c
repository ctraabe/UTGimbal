#include "dmp.h"
#include "dmp_firmware.h"

#include <string.h>

#include "boolean.h"
#include "endian.h"
#include "i2c.h"
#include "quaternion.h"
#include "timer0.h"

// Set the following to 1 or 0 to enable/disable
#define DMP_OUTPUT_ACCELEROMETER   (1)
#define DMP_OUTPUT_GYRO            (1)
#define DMP_CALIBRATED_GYRO_OUTPUT (0)

#define DMP_BANK_SIZE (256)
#define DMP_FIFO_DATA_SIZE (28)

// #define DMP_FIFO_DATA_SIZE (4 * sizeof(int32_t)
//   + DMP_OUTPUT_ACCELEROMETER * 3 * sizeof(int16_t)
//   + DMP_OUTPUT_GYRO * 3 * sizeof(int16_t))

// Quaternion from InvenSense's MotionDriver v5.1 are composed of an array of
// 32-bit fixed-point signed integers in Q1.30 form (i.e. 1L<<30 = 1.0f).
#define DMP_QUATERNION_FIXED_POINT (1L << 30)

#define DMP_EULER_ANGLES_CLEAR (0)
#define DMP_EULER_ANGLES_ROLL  (1<<0)
#define DMP_EULER_ANGLES_PITCH (1<<1)
#define DMP_EULER_ANGLES_YAW   (1<<2)

enum MPU6050MemoryAccessMode {
  READ,
  WRITE,
};

static float _dmp_quaternion[4] = {0.0}, _dmp_roll_angle = 0.0,
  _dmp_pitch_angle = 0.0, _dmp_yaw_angle = 0.0;
static int16_t _dmp_acclerometer[3] = {0}, _dmp_gyro[3] = {0};
static uint8_t _dmp_euler_angles = DMP_EULER_ANGLES_CLEAR;

static inline float q0_squared(void)
{
  static float _q0_squared = 0.0;
  if (!(_dmp_euler_angles & (DMP_EULER_ANGLES_ROLL | DMP_EULER_ANGLES_YAW)))
    _q0_squared = _dmp_quaternion[0] * _dmp_quaternion[0];
  return _q0_squared;
}

float dmp_quaternion(uint8_t index)
{
  return _dmp_quaternion[index];
}

int16_t dmp_accelerometer(uint8_t index)
{
  return _dmp_acclerometer[index];
}

int16_t dmp_gyro(uint8_t index)
{
  return _dmp_gyro[index];
}

float dmp_roll_angle(void)
{
  if (!(_dmp_euler_angles & DMP_EULER_ANGLES_ROLL)) {
    _dmp_roll_angle = RollAngleFromQuaternion(_dmp_quaternion, q0_squared());
    _dmp_euler_angles |= DMP_EULER_ANGLES_ROLL;
  }
  return _dmp_roll_angle;
}

float dmp_pitch_angle(void)
{
  if (!(_dmp_euler_angles & DMP_EULER_ANGLES_PITCH)) {
    _dmp_pitch_angle = PitchAngleFromQuaternion(_dmp_quaternion);
    _dmp_euler_angles |= DMP_EULER_ANGLES_PITCH;
  }
  return _dmp_pitch_angle;
}

float dmp_yaw_angle(void)
{
  if (!(_dmp_euler_angles & DMP_EULER_ANGLES_YAW)) {
    _dmp_yaw_angle = YawAngleFromQuaternion(_dmp_quaternion, q0_squared());
    _dmp_euler_angles |= DMP_EULER_ANGLES_YAW;
  }
  return _dmp_yaw_angle;
}

// -----------------------------------------------------------------------------
/**
 *  @brief      Read/Write to the DMP memory.
 *  This function prevents I2C reads/writes past the bank boundaries. The DMP
 *  memory is only accessible when the chip is awake.
 *  @param[in]  mem_addr    Memory location (bank << 8 | start address)
 *  @param[in]  buffer      Local source/destination.
 *  @param[in]  length      Number of bytes to read/write.
 *  @param[in]  memory_access_mode    Read or write.
 *  @return     0 if successful.
 */
int8_t MPU6050AccessDMPMemory(uint16_t memory_address, uint8_t *buffer,
  uint8_t length, enum MPU6050MemoryAccessMode memory_access_mode)
{
#define MPU6050_DMP_BANK_SEL (0x6D)
#define MPU6050_DMP_MEM_RW   (0x6F)

  // TODO: change these return values to something meaningful
  if (!buffer)
    return -1;
  // Check bank boundaries.
  if ((uint16_t)((uint8_t)memory_address) + (uint16_t)length > DMP_BANK_SIZE)
    return -1;

  // Transmit the memory address to be subsequently accessed. Note, AVR is
  // little-endian, MPU6050 is big-endian, bytes need to be swapped.
  uint8_t *tx_buffer = BigEndianArrayFromU16(memory_address);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_DMP_BANK_SEL, tx_buffer,
    2);
  I2CWaitUntilCompletion();
  free(tx_buffer);

  // Perform the memory access (read or write).
  if (memory_access_mode == READ)
    I2CRxBytesFromRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_DMP_MEM_RW, buffer,
      length);
  else
    I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_DMP_MEM_RW, buffer,
      length);
  I2CWaitUntilCompletion();

  // It is expected that the slave's last response will be that it is ready
  // to receive more data.
  if (i2c_error() != I2C_ERROR_ACK)
    return -1;

  return 0;
}

// -----------------------------------------------------------------------------
/**
 *  @brief      Load and verify DMP firmware image.
 *  @return     0 if successful.
 */
int8_t DMPLoadFirmware(void)
{
// Must divide evenly into MPU6050 bank_size (256) to avoid bank crossings.
#define DMP_UPLOAD_BLOCK_SIZE (16)
#define MPU6050_DMP_PRGM_START_H (0x70)
#define MPU6050_DMP_START_ADDRESS (0x0400)
  uint8_t block_size = DMP_UPLOAD_BLOCK_SIZE;
  uint8_t tx_buffer[DMP_UPLOAD_BLOCK_SIZE];
  uint8_t verify_buffer[DMP_UPLOAD_BLOCK_SIZE];

  for (uint16_t i = 0; i < DMP_FIRMWARE_SIZE; i += DMP_UPLOAD_BLOCK_SIZE) {
    if (i + DMP_UPLOAD_BLOCK_SIZE > DMP_FIRMWARE_SIZE)
      block_size = DMP_FIRMWARE_SIZE - i;
    for (int j = 0; j < block_size; j++)
      tx_buffer[j] = pgm_read_byte(dmp_firmware + i + j);
    if (MPU6050AccessDMPMemory(i, tx_buffer, block_size, WRITE))
      return -1;
    if (MPU6050AccessDMPMemory(i, verify_buffer, block_size, READ))
      return -1;
    if (memcmp(tx_buffer, verify_buffer, block_size))
      return -2;
  }

  /* Set program start address. */
  tx_buffer[0] = MPU6050_DMP_START_ADDRESS >> 8;
  tx_buffer[1] = MPU6050_DMP_START_ADDRESS & 0xFF;
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_DMP_PRGM_START_H,
    tx_buffer, 2);
  I2CWaitUntilCompletion();

  // It is expected that the slave's last response will be that it is ready
  // to receive more data.
  if (i2c_error() != I2C_ERROR_ACK)
    return -1;

  return 0;
}

void DMPInit(void)
{
  uint8_t tx_buffer[12];

  // Reset the MPU6050
  tx_buffer[0] = _BV(MPU6050_PWR_MGMT_1_DEVICE_RESET);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
    tx_buffer, 1);
  I2CWaitUntilCompletion();
  Timer0Delay(100);

  // Wake the MPU6050
  tx_buffer[0] = 0x00;
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
    tx_buffer, 1);
  I2CWaitUntilCompletion();

  // mpu_set_gyro_fsr(2000) [Set gyro range to +/- 2000 deg/s]
  tx_buffer[0] = _BV(MPU6050_GYRO_CONFIG_FS_SEL1)
    | _BV(MPU6050_GYRO_CONFIG_FS_SEL0);  // 2000 deg/s
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG,
    tx_buffer, 1);
  I2CWaitUntilCompletion();

  // mpu_set_accel_fsr(2) [Set accelerometer range to +/- 2 g]
  tx_buffer[0] = 0;
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG,
    tx_buffer, 1);
  I2CWaitUntilCompletion();

  // Set sample rate to 200 Hz
  tx_buffer[0] = 4;  // 1000 Hz / (1 + 4) = 200 Hz
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPRT_DIV, tx_buffer,
    1);
  I2CWaitUntilCompletion();

  // Set the digital low-pass filter to 98 Hz
  tx_buffer[0] = _BV(MPU6050_CONFIG_DLPF_CFG1);  // 98 Hz LPF
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, tx_buffer,
    1);
  I2CWaitUntilCompletion();

  // Load DMP firmware
  DMPLoadFirmware();

  // Set orientation
  tx_buffer[0] = 0x4C;
  tx_buffer[1] = 0xCD;
  tx_buffer[2] = 0x6C;
  MPU6050AccessDMPMemory(1062, tx_buffer, 3, WRITE);

  // Don't send gesture data to FIFO
  tx_buffer[0] = 0xD8;
  MPU6050AccessDMPMemory(2742, tx_buffer, 1, WRITE);

  // Enable accelerometer output to FIFO
  if (DMP_OUTPUT_ACCELEROMETER) {
    tx_buffer[0] = 0xC0;
    tx_buffer[1] = 0xC8;
    tx_buffer[2] = 0xC2;
    MPU6050AccessDMPMemory(2727, tx_buffer, 3, WRITE);
  }

  // Enable gyro output to FIFO
  if (DMP_OUTPUT_GYRO) {
    tx_buffer[0] = 0xC4;
    tx_buffer[1] = 0xCC;
    tx_buffer[2] = 0xC6;
    MPU6050AccessDMPMemory(2730, tx_buffer, 3, WRITE);
    if (!DMP_CALIBRATED_GYRO_OUTPUT) {
      tx_buffer[0] = 0xB8;
      tx_buffer[1] = 0xAA;
      tx_buffer[2] = 0xAA;
      tx_buffer[3] = 0xAA;
      tx_buffer[4] = 0xB0;
      tx_buffer[5] = 0x88;
      tx_buffer[6] = 0xC3;
      tx_buffer[7] = 0xC5;
      tx_buffer[8] = 0xC7;
      MPU6050AccessDMPMemory(1208, tx_buffer, 9, WRITE);
      tx_buffer[0] = 0xB0;
      tx_buffer[1] = 0x80;
      tx_buffer[2] = 0xB4;
      tx_buffer[3] = 0x90;
      MPU6050AccessDMPMemory(2722, tx_buffer, 4, WRITE);
    }
  }

  // Enable Quaternion and gyro calibration features
  tx_buffer[0] = 0x20;
  tx_buffer[1] = 0x28;
  tx_buffer[2] = 0x30;
  tx_buffer[3] = 0x38;
  MPU6050AccessDMPMemory(2718, tx_buffer, 4, WRITE);

  // Set DMP FIFO rate to 200 Hz
  tx_buffer[0] = 0;
  MPU6050AccessDMPMemory(535, tx_buffer, 1, WRITE);
  tx_buffer[0] = 0xFE;
  tx_buffer[1] = 0xF2;
  tx_buffer[2] = 0xAB;
  tx_buffer[3] = 0xC4;
  tx_buffer[4] = 0xAA;
  tx_buffer[5] = 0xF1;
  tx_buffer[6] = 0xDF;
  tx_buffer[7] = 0xDF;
  tx_buffer[8] = 0xBB;
  tx_buffer[9] = 0xAF;
  tx_buffer[10] = 0xDF;
  tx_buffer[11] = 0xDF;
  MPU6050AccessDMPMemory(2753, tx_buffer, 12, WRITE);

  // Reset the DMP and FIFO
  tx_buffer[0] = _BV(MPU6050_USER_CTRL_DMP_RESET)
    | _BV(MPU6050_USER_CTRL_FIFO_RESET);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, tx_buffer,
    1);
  I2CWaitUntilCompletion();
  Timer0Delay(50);

  // Start the DMP and FIFO
  tx_buffer[0] = _BV(MPU6050_USER_CTRL_DMP_EN) | _BV(MPU6050_USER_CTRL_FIFO_EN);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, tx_buffer,
    1);
  I2CWaitUntilCompletion();

  // Enable the DMP data ready interrupt
  tx_buffer[0] = _BV(MPU6050_INT_ENABLE_DMP_INT_EN);
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE,
    tx_buffer, 1);
  I2CWaitUntilCompletion();

  // Set the biases (specific to each chip)
  MPU6050SetAccelerometerBias(MPU6050_X_AXIS, -66);
  MPU6050SetAccelerometerBias(MPU6050_Y_AXIS, 219);
  MPU6050SetAccelerometerBias(MPU6050_Z_AXIS, 1062);
  MPU6050SetGyroBias(MPU6050_X_AXIS, -43);
  MPU6050SetGyroBias(MPU6050_Y_AXIS, 83);
  MPU6050SetGyroBias(MPU6050_Z_AXIS, 9);
}

enum MPU6050Error DMPReadFIFO(void)
{
  uint8_t remaining = 1;
  uint8_t rx_buffer[DMP_FIFO_DATA_SIZE];
  enum MPU6050Error error = MPU6050_ERROR_NONE;

  // TODO: Make this non-blocking
  while (remaining) {
    error = MPU6050ReadFromFIFO(rx_buffer, DMP_FIFO_DATA_SIZE, &remaining);
    I2CWaitUntilCompletion();
  }

  for (uint8_t i = 0; i < 4; i++) {
    _dmp_quaternion[i] = (float)BigEndianArrayToS32(rx_buffer + i *
      sizeof(int32_t)) / (float)DMP_QUATERNION_FIXED_POINT;
  }
  // Mark previously computed Euler angles as invalid.
  _dmp_euler_angles = DMP_EULER_ANGLES_CLEAR;

  if (DMP_OUTPUT_ACCELEROMETER) {
    for (uint8_t i = 0; i < 3; i++) {
      _dmp_acclerometer[i] = BigEndianArrayToS16(rx_buffer + 4 * sizeof(int32_t)
        + i * sizeof(int16_t));
    }
  }

  if (DMP_OUTPUT_GYRO) {
    uint8_t temp = (DMP_OUTPUT_ACCELEROMETER) ? 3 * sizeof(int16_t) : 0;
    for (uint8_t i = 0; i < 3; i++) {
      _dmp_gyro[i] = BigEndianArrayToS16(rx_buffer + 4 * sizeof(int32_t)
        + temp + i * sizeof(int16_t));
    }
  }

  return error;
}
