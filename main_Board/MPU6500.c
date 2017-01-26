#include "stm32f10x.h"
#include "MPU6500.h"
////////////////////////////////////////////////////////////////////////////////
void GYRO_SPI_CONFIG(void){
  SPI1 ->CR1 = 0;                                                 
  SPI1 ->CR1 |= SPI_CR1_MSTR                                                    //Master mode
              | SPI_CR1_BR                                                     //Set baud rate(=f pclk/256)
              | SPI_CR1_SSM
              | SPI_CR1_SSI
              | SPI_CR1_SPE; 
  
  ///////SPI pin config////////
  GPIOA ->CRL |= GPIO_CRL_MODE5;                                                //Set out mode(SCK)
  GPIOA ->CRL &= ~GPIO_CRL_CNF5;                                                //Reset CRL
  GPIOA ->CRL |= GPIO_CRL_CNF5_1;                                               //AF push-pull
  
  GPIOA ->CRL |= GPIO_CRL_MODE7;                                                //Set out mode(MOSI)
  GPIOA ->CRL &= ~GPIO_CRL_CNF7;                                                //Reset CRL
  GPIOA ->CRL |= GPIO_CRL_CNF7_1;                                               //AF push-pull
  
  GPIOA ->CRL |= GPIO_CRL_MODE4;                                                //Set out mode(CS)
  GPIOA ->CRL &= ~GPIO_CRL_CNF4;                                                //Set out push-pull mode                    

}

////////////////////////////////////////////////////////////////////////////////
void MPU6500_reg_write(uint8_t address, uint8_t* ptr){
 CS_LOW                                                                         //CS
  
  SPI_GYRO->DR = address & 0x7F;
  while((SPI_GYRO->SR & SPI_SR_TXE) == 0);
  
  SPI_GYRO->DR = *ptr; 
  while((SPI_GYRO->SR & SPI_SR_TXE) == 0);
     
  while(SPI_GYRO->SR & SPI_SR_BSY);  
// SPI_GYRO ->DR; 
 CS_HIGH                                                                        //CS
}
////////////////////////////////////////////////////////////////////////////////

void MPU6500_reg_read(uint8_t address, uint8_t* rd_ptr){
  CS_LOW 
  SPI1 -> DR = (address & 0x7F) | 0x80;
  while((SPI1->SR & SPI_SR_TXE)==0);
  while(SPI1 -> SR & SPI_SR_BSY);
  SPI1 -> DR ;
  SPI1->DR = 0;
  //////////////////////////////////////
  while((SPI1-> SR & SPI_SR_RXNE) ==0);
  *rd_ptr = SPI1->DR;

  ///////////////////////////////
  while(SPI1 -> SR & SPI_SR_BSY);
 //   SPI1->DR;
   CS_HIGH  
}

/////////////////////////////////////////////////////////////////////////////
void READ_GYRO_Settings(uint8_t* gyro_read_ptr){
  MPU6500_reg_read(0x1B,gyro_read_ptr);
}

////////////////////////////////////////////////////////////////////////////
void SET_GYRO_Settings(uint8_t* gyro_set_ptr){
  MPU6500_reg_write(0x1B,gyro_set_ptr);
}

////////////////////////////////////////////////////////////////////////////
void READ_ACC_Settings(uint8_t* acc_read_ptr){
  MPU6500_reg_read(0x1C,acc_read_ptr);
  MPU6500_reg_read(0x1D,acc_read_ptr + 8);
}

///////////////////////////////////////////////////////////////////////////
void SET_ACC_Settings(uint8_t* acc_set_ptr){
  MPU6500_reg_write(0x1C,acc_set_ptr);
  MPU6500_reg_write(0x1D,acc_set_ptr + 8);
}

///////////////////////////////////////////////////////////////////////////
void GET_SELF_Test_data(uint8_t* st_ptr){
  MPU6500_reg_read(0x00,st_ptr);
  MPU6500_reg_read(0x01,st_ptr + 8);
  MPU6500_reg_read(0x02,st_ptr + 16);
  ////////////
  MPU6500_reg_read(0x0D,st_ptr + 24);
  MPU6500_reg_read(0x0E,st_ptr + 32);
  MPU6500_reg_read(0x0F,st_ptr + 40);
}

//////////////////////////////////////////////////////////////////////////
void SET_GYRO_Offset_data(uint8_t* offset_ptr){
  MPU6500_reg_write(0x13,offset_ptr);
  MPU6500_reg_write(0x14,offset_ptr + 1);
  MPU6500_reg_write(0x15,offset_ptr + 2);
  MPU6500_reg_write(0x16,offset_ptr + 3);
  MPU6500_reg_write(0x17,offset_ptr + 4);
  MPU6500_reg_write(0x18,offset_ptr + 5);  
}
void SET_ACC_Offset_data(uint8_t* offset_ptr){
  MPU6500_reg_write(0x77,offset_ptr);
  MPU6500_reg_write(0x78,offset_ptr + 1);
  MPU6500_reg_write(0x7A,offset_ptr + 2);
  MPU6500_reg_write(0x7B,offset_ptr + 3);
  MPU6500_reg_write(0x7D,offset_ptr + 4);
  MPU6500_reg_write(0x7E,offset_ptr + 5);  
}
///////////////////////////////////////////////////////////////////////////
void READ_GYRO_data(uint16_t* gyro_data_ptr){
  uint16_t* temp_data;
  uint16_t temp_rez;
  uint8_t ADDR_reg_array[6] = {0x43,0x44,0x45,0x46,0x47,0x48};
  
  for(uint8_t reg_cnt=0; reg_cnt < 3; reg_cnt++){
    
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2],(uint8_t*)temp_data);
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2 +1 ],(uint8_t*)temp_data + 8);
  
  if( ( ( (*(uint16_t*)temp_data) & 0x8000) >> 15) > 0){
      temp_rez = ((~(*(uint16_t*)temp_data) +1) &0x7FFF)|0x8000; 
  }
  else{
    temp_rez = *(uint16_t*)temp_data;
}
 *((uint16_t*)gyro_data_ptr + reg_cnt) = temp_rez;
 
  }
}

//////////////////////////////////////////////////////////////////////////////
void READ_ACC_data(uint16_t* acc_data_ptr){
  uint16_t* temp_data;
  uint16_t temp_rez;
  uint8_t ADDR_reg_array[6] = {0x3B,0x3C,0x3D,0x3E,0x3F,0x40};
  
  for(uint8_t reg_cnt=0; reg_cnt < 3; reg_cnt++){
  ///////////////////
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2],(uint8_t*)temp_data);
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2 +1],(uint8_t*)temp_data);
  ////////////////////
  if( ( ( (*(uint16_t*)temp_data) & 0x8000) >> 15) > 0){
      temp_rez = ((~(*(uint16_t*)temp_data) +1) &0x7FFF)|0x8000; 
     }
  else{
      temp_rez = *(uint16_t*)temp_data;
      }
 *((uint16_t*)acc_data_ptr + reg_cnt) = temp_rez;
  }
}

////////////////////////////////////////////////////////////////////////////////

void READ_ACC_GYRO_data(uint16_t* data_ptr){
  uint8_t temp_data_L, temp_data_H;
  uint16_t temp_rez, temp_data;
  uint8_t ADDR_reg_array[14] = {0x43,0x44,0x45,0x46,0x47,0x48,0x3B,0x3C,0x3D,0x3E,0x3F,0x40};
  
  for(uint8_t reg_cnt=0; reg_cnt < 6; reg_cnt++){
    ///////////////////
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2] ,(uint8_t*)&temp_data_H);
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2 + 1 ],(uint8_t*)&temp_data_L);
  temp_data = temp_data_L | (((uint16_t)temp_data_H) << 8);
  ////////////////////
  if( ( (temp_data & 0x8000) >> 15) > 0){
      temp_rez = ((~temp_data + 1) &0x7FFF)|0x8000; 
     }
  else{
      temp_rez = temp_data;
      }
 *((uint16_t*)data_ptr + reg_cnt) = temp_rez;
  }
}
///////////////////////////////////////////////////////////////////////////
void READ_RAW_ACC_GYRO_data(uint16_t* data_ptr){
    uint8_t temp_data_L, temp_data_H;
  uint8_t ADDR_reg_array[14] = {0x43,0x44,0x45,0x46,0x47,0x48,0x3B,0x3C,0x3D,0x3E,0x3F,0x40};
  
  for(uint8_t reg_cnt=0; reg_cnt < 6; reg_cnt++){
    ///////////////////
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2] ,(uint8_t*)&temp_data_H);
  MPU6500_reg_read(ADDR_reg_array[reg_cnt*2 + 1 ],(uint8_t*)&temp_data_L);
  
  *((uint16_t*)data_ptr + reg_cnt)= (uint16_t)temp_data_L | (((uint16_t)temp_data_H) << 8);
  ////////////////////
  }
}