#define SPI_GYRO SPI1
#define CS_HIGH GPIOA->ODR |= GPIO_ODR_ODR4;
#define CS_LOW GPIOA->ODR &= ~GPIO_ODR_ODR4;
#define GYRO_INT_RDY GPIOA -> IDR & GPIO_IDR_IDR0   
/////////////////////////Register address definition////////////////////////////
#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02
#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F
#define XG_OFFSET_H 0x13
#define XG_OFFSET_L 0x14
#define YG_OFFSET_H 0x15
#define YG_OFFSET_L 0x16
#define ZG_OFFSET_H 0x17
#define ZG_OFFSET_L 0x18
#define SMPLRT_DIV 0x19
#define CONFIG_REG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_CONFIG_2 0x1D
#define LP_ACCEL_ODR 0x1E
#define WOW_THRESHOLD 0x1F
#define FIFO_EN 0x23
///////if need paste I2C reg here//////
#define INT_PIN_CFG 0x37
#define INT_ENABLE 0x38
#define INT_STATUS 0x3A
#define ACCEL_XOUT_H 0x3B
#define ACCEL_XOUT_L 0x3C
#define ACCEL_YOUT_H 0x3D
#define ACCEL_YOUT_L 0x3E
#define ACCEL_ZOUT_H 0x3F
#define ACCEL_ZOUT_L 0x40
#define TEMP_OUT_H 0x41
#define TEMP_OUT_L 0x42
#define GYRO_XOUT_H 0x43
#define GYRO_XOUT_L 0x44
#define GYRO_YOUT_H 0x45
#define GYRO_YOUT_L 0x46
#define GYRO_ZOUT_H 0x47
#define GYRO_ZOUT_L 0x48
////////////////////////Settings parameter definitions//////////////////////////
#define GYRO_FULL_SCALE_250dps 0x00
#define GYRO_FULL_SCALE_500dps 0x01
#define GYRO_FULL_SCALE_1000dps 0x2
#define GYRO_FULL_SCALE_2000dps 0x3
////////////////////////////////
#define ACCEL_FULL_SCALE_2g 0x00
#define ACCEL_FULL_SCALE_4g 0x01
#define ACCEL_FULL_SCALE_8g 0x2
#define ACCEL_FULL_SCALE_16g 0x3
//////////////////////////
#define ACCEL_BW_460HZ 0x00
#define ACCEL_BW_184HZ 0x01
#define ACCEL_BW_92HZ 0x02
#define ACCEL_BW_41HZ 0x03
#define ACCEL_BW_20HZ 0x04
#define ACCEL_BW_10HZ 0x05
#define ACCEL_BW_5HZ 0x06
///////////////////////////
#define GYRO_BW_250HZ 0x00
#define GYRO_BW_184HZ 0x01
#define GYRO_BW_92HZ 0x02
#define GYRO_BW_41HZ 0x03
#define GYRO_BW_20HZ 0x04
#define GYRO_BW_10HZ 0x05
#define GYRO_BW_5HZ 0x06
///////////////////////////////////ACC and GYRO data///////////////////////////
typedef struct
{
  uint16_t ACC_X;
  uint16_t ACC_Y;
  uint16_t ACC_Z;
  ///////////////
  uint16_t GYRO_X;
  uint16_t GYRO_Y;
  uint16_t GYRO_Z;
  //////////////
}MPU6500_out_data_t;
/////////////////////////////Config gyroscope////////////////////////////////////
typedef struct
{
  uint8_t DLPF_CFG:3;
  //////////////
  uint8_t EXT_SYNC_SET:3;
  ///////////////
  uint8_t FIFO_mode:1;
  //////////////
  uint8_t rezerved:1;

}MPU6500_Config_t;



typedef struct
{
  uint8_t FCHOICE_Bypass:2;
  //////////////
  uint8_t reserved:1;
  ///////////////
  uint8_t GYRO_FULL_SCALE_SEL:2;
  //////////////
  uint8_t GYRO_self_test_X:1;
  uint8_t GYRO_self_test_Y:1;
  uint8_t GYRO_self_test_Z:1;
  
}MPU6500_GyroSetiings_t;

////////////////////////Config Accelerometer////////////////////////////////////
typedef struct
{
  uint8_t reserved:3;
  //////////////
  uint8_t ACC_FULL_SCALE_SEL:2;
  //////////////
  uint8_t ACC_self_test_Z:1;
  uint8_t ACC_self_test_Y:1;
  uint8_t ACC_self_test_X:1;
  
}MPU6500_ACC_Setiings_t;
////////////////////
typedef struct
{
  uint8_t A_DLPF_CFG:3;
  uint8_t ACC_FCHOICE_B:1;
  uint8_t reserved2:4;

}MPU6500_ACC_Setiings2_t;
///////////////////////
typedef struct
{
  uint8_t RAW_RDY_EN:1;
  uint8_t zero:2;
  uint8_t FSYNC_INT_EN:1;
  uint8_t FIFO_OVERFLOW_EN:1;
  uint8_t zero2:1;
  uint8_t WOW_EN:1;
  uint8_t zero3:1;
}INT_Enable_Setiings_t;
//////////////////////
typedef struct
{
  uint8_t Rezerved:1;
  uint8_t BYPASS_EN:1;
  uint8_t FSYNC_INT_MODE_EN:1;
  uint8_t ACTL_FSYNC:1;
  uint8_t INT_ANYRD_2CLEAR:1;
  uint8_t LATCH_INT_EN:1;
  uint8_t OPEN:1;
  uint8_t ACTL:1;
}INT_Pin_Config_t;
/////////////////////////
typedef struct
{
  uint8_t RAW_DATA_RDY_INT:1;
  uint8_t DMP_INT:1;
  uint8_t rezerved1:1;
  uint8_t FSYNC_INT:1;
  uint8_t FIFO_OVERFLOW_INT:1;
  uint8_t rezerved2:1;
  uint8_t WOW_INT:1;
  uint8_t rezerved3:1;
}INT_status_t;
///////////////////////Gyro offset//////////////////////////////////////////////
typedef struct
{
  uint16_t X_offset;
  uint16_t Y_offset;
  uint16_t Z_offset;
}MPU6500_GYRO_Offset_t;
///////////////////ACC offset///////////////////////////////////////////////////
typedef struct
{
  uint16_t X_offset;
  uint16_t Y_offset;
  uint16_t Z_offset;
}MPU6500_ACC_Offset_t;
////////////////////Self test data//////////////////////////////////////////////
typedef struct
{
  uint8_t GYRO_X_SelfTest;
  uint8_t GYRO_Y_SelfTest;
  uint8_t GYRO_Z_SelfTest;
  ///////
  uint8_t ACC_X_SelfTest;
  uint8_t ACC_Y_SelfTest;
  uint8_t ACC_Z_SelfTest;
}MPU6500_SelfTest_data_t;



////////////////////////////////////////////////
void GYRO_SPI_CONFIG(void);
//////////////
void READ_GYRO_Settings(uint8_t* gyro_read_ptr);
void SET_GYRO_Settings(uint8_t* gyro_set_ptr);
//////////////
void READ_ACC_Settings(uint8_t* acc_read_ptr);
void SET_ACC_Settings(uint8_t* acc_set_ptr);
//////////////
void GET_SELF_Test_data(uint8_t* st_ptr);
void SET_GYRO_Offset_data(uint8_t* offset_ptr);
void SET_ACC_Offset_data(uint8_t* offset_ptr);
//////////////
void READ_GYRO_data(uint16_t* gyro_data_ptr);
void READ_ACC_data(uint16_t* acc_data_ptr);
void READ_ACC_GYRO_data(uint16_t* data_ptr);
void READ_RAW_ACC_GYRO_data(uint16_t* data_ptr);
/////////////
void MPU6500_reg_read(uint8_t address, uint8_t* rd_ptr);
void MPU6500_reg_write(uint8_t address, uint8_t* ptr);