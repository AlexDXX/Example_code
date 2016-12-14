#include "stm32f10x.h"

#define SPI_MAG SPI2
#define MAG_CS_HIGH GPIOB->ODR |= GPIO_ODR_ODR12;
#define MAG_CS_LOW GPIOB->ODR &= ~GPIO_ODR_ODR12;

#define WHO_AM_I_REG 0xF
#define CTRL_REG_1 0x20
#define CTRL_REG_2 0x21
#define CTRL_REG_3 0x22
#define CTRL_REG_4 0x23
#define CTRL_REG_5 0x24
#define STATUS_REG 0x27
#define OUT_X_L_REG 0x28
#define OUT_X_H_REG 0x29
#define OUT_Y_L_REG 0x2A
#define OUT_Y_H_REG 0x2B
#define OUT_Z_L_REG 0x2C
#define OUT_Z_H_REG 0x2D
#define TEMP_OUT_L_REG 0x2E
#define TEMP_OUT_H_REG 0x2F
#define INT_CFG_REG 0x30
#define INT_SRC_REG 0x31
#define INT_THS_L_REG 0x32
#define INT_THS_H_REG 0x33

#define DATA_RATE_0_625_HZ 0x00
#define DATA_RATE_1_25_HZ 0x01
#define DATA_RATE_2_5_HZ 0x02
#define DATA_RATE_5_HZ 0x03
#define DATA_RATE_10_HZ 0x04
#define DATA_RATE_20_HZ 0x05
#define DATA_RATE_40_HZ 0x06
#define DATA_RATE_80_HZ 0x07

#define LOW_POWER_MODE 0x00
#define MEDIUM_PERFOMANCE_MODE 0x01
#define HIGH_PERFOMANCE_MODE 0x02
#define ULTRA_HIGH_PERFOMANCE_MODE 0x03

#define FULL_SCALE_4_gauss 0x00
#define FULL_SCALE_8_gauss 0x01
#define FULL_SCALE_12_gauss 0x02
#define FULL_SCALE_16_gauss 0x01

typedef struct{
  
  uint16_t header;
  uint8_t Command_code;
  ///////////////
  uint16_t X_Gyro;
  uint16_t Y_Gyro;
  uint16_t Z_Gyro;
  ///////////////
  uint16_t X_Acc;
  uint16_t Y_Acc;
  uint16_t Z_Acc;
  //////////////
  uint16_t X_Mag;
  uint16_t Y_Mag;
  uint16_t Z_Mag;
  //////////////
  uint8_t checksum;
}SERIAL_PORT_msg_t;


typedef struct 
{
  uint8_t dev_ID;
}DEV_number_t;

/////////////////////////////////////
typedef struct 
{
  uint8_t SELF_TEST_EN:1;
  uint8_t FAST_ODR_EN:1;
  uint8_t OUT_DATA_RATE:3;
  uint8_t OPERATIVE_MODE_SEL:3;
  uint8_t TEMP_EN:1;
}CTRL_REG1_t;

////////////////////////////////////
typedef struct 
{
  uint8_t reserved:2;
  uint8_t SOFT_RST:1;
  uint8_t REBOOT:1;
  uint8_t rezerved2:1;
  uint8_t FULL_SCALE_CONFIG:2;
  uint8_t rezerved3:1;
}CTRL_REG2_t;


////////////////////////////////////
typedef struct 
{
  uint8_t OPERATING_MODE_SEL:2;
  uint8_t SPI_MODE_SEL:1;
  uint8_t zero:2;
  uint8_t LOW_PWR_MODE_CONFIG:1;
  uint8_t zero2:2;
}CTRL_REG3_t;

////////////////////////////////////
typedef struct 
{
  uint8_t zero1:1;
  uint8_t BIG_LITTLE_ENDIAN:1;
  uint8_t Zaxis_OPERATION_MODE:2;
  uint8_t zero2:4;
}CTRL_REG4_t;

//////////////////////////////////
typedef struct 
{
  uint8_t zero:6;
  uint8_t BLOCK_DATA_UPDATE:1;
  uint8_t FAST_READ:1;
}CTRL_REG5_t;

//////////////////////////////////
typedef struct 
{
  uint8_t XDA:1;
  uint8_t YDA:1;
  uint8_t ZDA:1;
  uint8_t ZYXDA:1;
  uint8_t XOR:1;
  uint8_t YOR:1;
  uint8_t ZOR:1;
  uint8_t XYZOR:1;
}STATUS_REG_t;

/////////////////////////////////
typedef struct 
{
  uint8_t INTERRUPT_ENABLE:1;
  uint8_t LATCH_INTERRUPT:1;
  uint8_t INTERRUPT_ACTIVE:1;
  uint8_t zero:2;
  uint8_t Z_INT_EN:1;
  uint8_t Y_INT_EN:1;
  uint8_t X_INT_EN:1;
}INT_CFG_t;
///////////////////////////////

void LIS3MDL_reg_read(uint32_t address, uint8_t* ptr);
void LIS3MDL_reg_write(uint32_t address, uint8_t* ptr);
uint8_t LIS3MDL_Read_data_XYZ(uint16_t* data_ptr);
void Magnetometer_SPI_CONFIG(void);
