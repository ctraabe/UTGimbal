#include "mpu6050.h"

#include <avr/io.h>

#include "i2c.h"


// ============================================================================+
// Public functions:

void InitMPU6050(uint8_t mpu6050_address)
{
  struct str_MasterTx
  {
    uint8_t address;
    uint8_t data[3];
  };

  union {
    struct str_MasterTx s;
    uint8_t bytes[sizeof(struct str_MasterTx)];
  } master_tx;

  // Turn off MPU6050 sleep enabled bit and set clock to PLL with X gyro ref.
  master_tx.s.address = MPU6050_RA_PWR_MGMT_1;
  master_tx.s.data[0] = _BV(MPU6050_PWR_MGMT_1_CLKSEL0);
  SendBytesI2C(mpu6050_address, master_tx.bytes, 2);
  while(!I2CIsIdle()) continue;  // Wait for transmission to complete

  // Turn on the 185Hz low-pass filter for the gyro and accelerometers, then
  // set the gyro and accelerometer range (consecutive destinations).
  master_tx.s.address = MPU6050_RA_CONFIG;
  master_tx.s.data[0] = _BV(MPU6050_CONFIG_DLPF_CFG0);
  // master_tx.s.data[1] = 0;  // 250 deg/s
  master_tx.s.data[1] = _BV(MPU6050_GYRO_CONFIG_FS_SEL0);  // 500 deg/s
  // master_tx.s.data[1] = _BV(MPU6050_GYRO_CONFIG_FS_SEL1);  // 1000 deg/s
  // master_tx.s.data[2] = 0;  // 2 g
  master_tx.s.data[2] = _BV(MPU6050_ACCEL_CONFIG_AFS_SEL0);  // 4 g
  // master_tx.s.data[2] = _BV(MPU6050_ACCEL_CONFIG_AFS_SEL1);  // 8 g
  SendBytesI2C(mpu6050_address, master_tx.bytes, 4);
  while(!I2CIsIdle()) continue;  // Wait for transmission to complete

  // Set the interrupt signal to latch high and clear on any read, then
  // enable the interrupt on new data ready (consecutive destinations).
  master_tx.s.address = MPU6050_RA_INT_PIN_CFG;
  master_tx.s.data[0] = _BV(MPU6050_INT_PIN_CFG_LATCH_INT_EN)
    | _BV(MPU6050_INT_PIN_CFG_INT_RD_CLEAR);
  master_tx.s.data[1] = _BV(MPU6050_INT_ENABLE_DATA_RDY_EN);
  SendBytesI2C(mpu6050_address, master_tx.bytes, 3);
  while(!I2CIsIdle()) continue;  // Wait for transmission to complete
}

// -----------------------------------------------------------------------------
void ReadMPU6050(uint8_t mpu6050_address, uint8_t *rx_destination_ptr)
{
  RequestFromAddress(mpu6050_address, MPU6050_RA_ACCEL_XOUT_H,
    rx_destination_ptr, 14);
}