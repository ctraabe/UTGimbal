#include "mpu6050_dmp.h"

#include <string.h>
#include <avr/eeprom.h>

#include "dmp_firmware.h"
#include "eeprom.h"
#include "endian.h"
#include "i2c.h"
#include "quaternion.h"
#include "timer0.h"


// =============================================================================
// Private data:

#define DMP_BANK_SIZE (256)
#define DMP_FIFO_DATA_SIZE (4 * sizeof(int32_t)\
  + DMP_OUTPUT_ACCELEROMETER * 3 * sizeof(int16_t)\
  + DMP_OUTPUT_GYRO * 3 * sizeof(int16_t))

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
static int16_t _dmp_accelerometer[3] = {0}, _dmp_gyro[3] = {0};
static uint8_t _dmp_euler_angles = DMP_EULER_ANGLES_CLEAR;


// =============================================================================
// Private function declarations:

static inline float q0_squared(void);
static inline int8_t sign(int16_t);
static int8_t MPU6050AccessDMPMemory(uint16_t memory_address, uint8_t *buffer,
  uint8_t length, enum MPU6050MemoryAccessMode memory_access_mode);
static int8_t DMPLoadFirmware(void);
static void DMPLoadOffsets(void);


// =============================================================================
// Accessors

float dmp_quaternion(uint8_t index)
{
  return _dmp_quaternion[index];
}
int16_t dmp_accelerometer(uint8_t index)
{
  return _dmp_accelerometer[index];
}
int16_t dmp_gyro(uint8_t index)
{
  return _dmp_gyro[index];
}


// =============================================================================
// Pseudo-accessors:

// Computation of these quantities is expensive and therefore are only performed
// if necessary and are stored so that they may be accessed again without
// re-invoking the expensive floating-point operations.
float dmp_roll_angle(void)  // rad
{
  if (!(_dmp_euler_angles & DMP_EULER_ANGLES_ROLL)) {
    _dmp_roll_angle = RollAngleFromQuaternion(_dmp_quaternion, q0_squared());
    _dmp_euler_angles |= DMP_EULER_ANGLES_ROLL;
  }
  return _dmp_roll_angle;
}
float dmp_pitch_angle(void)  // rad
{
  if (!(_dmp_euler_angles & DMP_EULER_ANGLES_PITCH)) {
    _dmp_pitch_angle = PitchAngleFromQuaternion(_dmp_quaternion);
    _dmp_euler_angles |= DMP_EULER_ANGLES_PITCH;
  }
  return _dmp_pitch_angle;
}
float dmp_yaw_angle(void)  // rad
{
  if (!(_dmp_euler_angles & DMP_EULER_ANGLES_YAW)) {
    _dmp_yaw_angle = YawAngleFromQuaternion(_dmp_quaternion, q0_squared());
    _dmp_euler_angles |= DMP_EULER_ANGLES_YAW;
  }
  return _dmp_yaw_angle;
}


// =============================================================================
// Public functions:

void MPU6050DMPInit(void)
{
  MPU6050Init();

  uint8_t tx_buffer[12];

  // The following are consecutive writes to ascending addresses:
  tx_buffer[0] = 4;  // Sample rate: 1000 Hz / (1 + 4) = 200 Hz
  tx_buffer[1] = _BV(MPU6050_CONFIG_DLPF_CFG1);  // 98 Hz LPF
  tx_buffer[2] = _BV(MPU6050_GYRO_CONFIG_FS_SEL1)
    | _BV(MPU6050_GYRO_CONFIG_FS_SEL0);  // Gyro range: +/- 2000 deg/s
  tx_buffer[3] = 0;  // Accelerometer range +/- 2 g
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPRT_DIV, tx_buffer,
    4);
  I2CWaitUntilCompletion();

  // There seems to be a problem with using a gyro as the clock source, so the
  // internal oscillator is used instead, for now...
  tx_buffer[0] = 0;
  I2CTxBytesToRegister(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1,
    tx_buffer, 1);
  I2CWaitUntilCompletion();

  // Load DMP firmware
  DMPLoadFirmware();

  // Enable accelerometer output to FIFO (CFG_15 + 1)
  if (DMP_OUTPUT_ACCELEROMETER) {
    tx_buffer[0] = 0xC0;
    tx_buffer[1] = 0xC8;
    tx_buffer[2] = 0xC2;
    MPU6050AccessDMPMemory(2728, tx_buffer, 3, WRITE);
  }

  // Enable gyro output to FIFO (CFG_15 + 4)
  if (DMP_OUTPUT_GYRO) {
    tx_buffer[0] = 0xC4;
    tx_buffer[1] = 0xCC;
    tx_buffer[2] = 0xC6;
    MPU6050AccessDMPMemory(2731, tx_buffer, 3, WRITE);
    // Output calibrated gyro (CFG_GYRO_RAW_DATA)
    if (!DMP_CALIBRATED_GYRO_OUTPUT) {
      tx_buffer[0] = 0xB0;
      tx_buffer[1] = 0x80;
      tx_buffer[2] = 0xB4;
      tx_buffer[3] = 0x90;
      MPU6050AccessDMPMemory(2722, tx_buffer, 4, WRITE);
    }
  }

  // Disable automatic calibration of gyro when still (CFG_MOTION_BIAS)
  if (!DMP_AUTO_CALIBRATE_GYRO) {
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
  }

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
  DMPLoadOffsets();
}

// -----------------------------------------------------------------------------
enum MPU6050Error DMPReadFIFO(void)
{
  uint8_t remaining = 1;
  uint8_t rx_buffer[DMP_FIFO_DATA_SIZE];
  enum MPU6050Error error = MPU6050_ERROR_NONE;

  // TODO: Make this non-blocking
  while (remaining) {
    error = MPU6050ReadFromFIFO(rx_buffer, DMP_FIFO_DATA_SIZE, &remaining);
    if (error != MPU6050_ERROR_NONE)
      return error;
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
      _dmp_accelerometer[i] = BigEndianArrayToS16(rx_buffer + 4
        * sizeof(int32_t) + i * sizeof(int16_t));
    }
  }

  if (DMP_OUTPUT_GYRO) {
    uint8_t temp = (DMP_OUTPUT_ACCELEROMETER) ? 3 * sizeof(int16_t) : 0;
    for (uint8_t i = 0; i < 3; i++) {
      _dmp_gyro[i] = BigEndianArrayToS16(rx_buffer + 4 * sizeof(int32_t)
        + temp + i * sizeof(int16_t));
    }
  }

  return MPU6050_ERROR_NONE;
}

// -----------------------------------------------------------------------------
void DMPCalibrate(enum DMPCalibrationMode* mode)
{
  static int16_t offset = 0;
  int16_t sample = 0;

  PORTB ^= _BV(PORTB5);  // Green LED

  switch(*mode) {
    case DMP_CALIBRATE_GYRO_X:
      sample = _dmp_gyro[0];
      break;
    case DMP_CALIBRATE_GYRO_Y:
      sample = _dmp_gyro[1];
      break;
    case DMP_CALIBRATE_GYRO_Z:
      sample = _dmp_gyro[2];
      break;
    case DMP_CALIBRATE_ACCELEROMETER_X:
      sample = _dmp_accelerometer[0];
      break;
    case DMP_CALIBRATE_ACCELEROMETER_Y:
      sample = _dmp_accelerometer[1];
      break;
    case DMP_CALIBRATE_ACCELEROMETER_Z:
      sample = _dmp_accelerometer[2];
      break;
    default:
      break;
  }

  if (*mode != DMP_CALIBRATE_ACCELEROMETER_Z && sample)
    offset -= sign(sample);

  switch(*mode) {
    case DMP_CALIBRATE_START:
      *mode = DMP_CALIBRATE_GYRO_X;
      MPU6050SetGyroBias(MPU6050_X_AXIS, offset);
      break;
    case DMP_CALIBRATE_GYRO_X:
      if (sample == 0) {
        eeprom_update_word((uint16_t*)EEPROM_GYRO_X_OFFSET, offset);
        offset = 0;
        *mode = DMP_CALIBRATE_GYRO_Y;
        MPU6050SetGyroBias(MPU6050_Y_AXIS, offset);
      } else {
        MPU6050SetGyroBias(MPU6050_X_AXIS, offset);
      }
      break;
    case DMP_CALIBRATE_GYRO_Y:
      if (sample == 0) {
        eeprom_update_word((uint16_t*)EEPROM_GYRO_Y_OFFSET, offset);
        offset = 0;
        *mode = DMP_CALIBRATE_GYRO_Z;
        MPU6050SetGyroBias(MPU6050_Z_AXIS, offset);
      } else {
        MPU6050SetGyroBias(MPU6050_Y_AXIS, offset);
      }
      break;
    case DMP_CALIBRATE_GYRO_Z:
      if (sample == 0) {
        eeprom_update_word((uint16_t*)EEPROM_GYRO_Z_OFFSET, offset);
        offset = 0;
        *mode = DMP_CALIBRATE_ACCELEROMETER_X;
        MPU6050SetAccelerometerBias(MPU6050_X_AXIS, offset);
      } else {
        MPU6050SetGyroBias(MPU6050_Z_AXIS, offset);
      }
      break;
    case DMP_CALIBRATE_ACCELEROMETER_X:
      if (sample == 0) {
        eeprom_update_word((uint16_t*)EEPROM_ACCELEROMETER_X_OFFSET, offset);
        offset = 0;
        *mode = DMP_CALIBRATE_ACCELEROMETER_Y;
        MPU6050SetAccelerometerBias(MPU6050_Y_AXIS, offset);
      } else {
        MPU6050SetAccelerometerBias(MPU6050_X_AXIS, offset);
      }
      break;
    case DMP_CALIBRATE_ACCELEROMETER_Y:
      if (sample == 0) {
        eeprom_update_word((uint16_t*)EEPROM_ACCELEROMETER_Y_OFFSET, offset);
        offset = 0;
        *mode = DMP_CALIBRATE_ACCELEROMETER_Z;
        MPU6050SetAccelerometerBias(MPU6050_Z_AXIS, offset);
      } else {
        MPU6050SetAccelerometerBias(MPU6050_Y_AXIS, offset);
      }
      break;
    case DMP_CALIBRATE_ACCELEROMETER_Z:
      if (sample == 16384) {
        eeprom_update_word((uint16_t*)EEPROM_ACCELEROMETER_Z_OFFSET, offset);
        offset = 0;
        *mode = DMP_CALIBRATE_DONE;
        PORTB &= ~_BV(PORTB5);  // Turn off green LED
      } else {
        offset -= sign(sample - 16384);
        MPU6050SetAccelerometerBias(MPU6050_Z_AXIS, offset);
      }
      break;
    default:
      *mode = DMP_CALIBRATE_DONE;
      break;
  }
}


// =============================================================================
// Private functions:

// This helper function squares the scaler component of a quaternion and stores
// the result so that it can be reused without re-invoking the expensive
// floating-point multiplication.
static inline float q0_squared(void)
{
  static float _q0_squared = 0.0;
  if (!(_dmp_euler_angles & (DMP_EULER_ANGLES_ROLL | DMP_EULER_ANGLES_YAW)))
    _q0_squared = _dmp_quaternion[0] * _dmp_quaternion[0];
  return _q0_squared;
}

// -----------------------------------------------------------------------------
static inline int8_t sign(int16_t value)
{
  if (value > 0) return 1;
  else if (value < 0) return -1;
  else return 0;
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
static int8_t MPU6050AccessDMPMemory(uint16_t memory_address, uint8_t *buffer,
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

// -----------------------------------------------------------------------------
void DMPLoadOffsets()
{
  int16_t offset;

  offset = eeprom_read_word((uint16_t*)EEPROM_ACCELEROMETER_X_OFFSET);
  MPU6050SetAccelerometerBias(MPU6050_X_AXIS, offset);

  offset = eeprom_read_word((uint16_t*)EEPROM_ACCELEROMETER_Y_OFFSET);
  MPU6050SetAccelerometerBias(MPU6050_Y_AXIS, offset);

  offset = eeprom_read_word((uint16_t*)EEPROM_ACCELEROMETER_Z_OFFSET);
  MPU6050SetAccelerometerBias(MPU6050_Z_AXIS, offset);

  offset = eeprom_read_word((uint16_t*)EEPROM_GYRO_X_OFFSET);
  MPU6050SetGyroBias(MPU6050_X_AXIS, offset);

  offset = eeprom_read_word((uint16_t*)EEPROM_GYRO_Y_OFFSET);
  MPU6050SetGyroBias(MPU6050_Y_AXIS, offset);

  offset = eeprom_read_word((uint16_t*)EEPROM_GYRO_Z_OFFSET);
  MPU6050SetGyroBias(MPU6050_Z_AXIS, offset);
}
