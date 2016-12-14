#include "stm32f10x.h"
#include "LIS3MDL.h"

void Magnetometer_SPI_CONFIG(void){
 

  SPI_MAG ->CR1 =0;                                          
  SPI_MAG ->CR1 |= SPI_CR1_MSTR                                                     //Master mode
              | SPI_CR1_BR                                                  //Set baud rate(=f pclk/256)
              | SPI_CR1_SSM
              | SPI_CR1_SSI;

  SPI_MAG ->CR1 |= SPI_CR1_SPE; 
  
  /////////////////////////////SPI pin config
  GPIOB ->CRH |= GPIO_CRH_MODE13;                                               //Set out mode(SCK)
  GPIOB ->CRH &= ~GPIO_CRH_CNF13;                                               //Reset CRL
  GPIOB ->CRH |= GPIO_CRH_CNF13_1;                                              //AF push-pull
  
  GPIOB ->CRH |= GPIO_CRH_MODE15;                                               //Set out mode(MOSI)
  GPIOB ->CRH &= ~GPIO_CRH_CNF15;                                               //Reset CRL
  GPIOB ->CRH |= GPIO_CRH_CNF15_1;                                              //AF push-pull
  
  GPIOB ->CRH |= GPIO_CRH_MODE12;                                               //Set out mode(CS)
  GPIOB ->CRH &= ~GPIO_CRH_CNF12;                                               //Reset CRL(push-pull)
}

///////////////////////////////Single regiser read//////////////////////////////
void LIS3MDL_reg_read(uint32_t address, uint8_t* ptr){
 MAG_CS_LOW                                                                         //CS
  SPI_MAG ->DR = (address & 0x3F) | 0x80;
  while((SPI_MAG->SR & SPI_SR_TXE)==0);
  while(SPI_MAG ->SR & SPI_SR_BSY); 
  SPI2 -> DR ;  
/////////////////////////////////////////
   SPI_MAG->DR = 0;
  while((SPI_MAG ->SR & SPI_SR_RXNE)==0);
  ////////////////////////////////////////
  *ptr = SPI_MAG -> DR;
    
  while(SPI_MAG ->SR & SPI_SR_BSY);   
 //SPI ->DR; 
 MAG_CS_HIGH                          //CS
}

/////////////////Single register write//////////////////////////////////////////
void LIS3MDL_reg_write(uint32_t address, uint8_t* ptr){
   MAG_CS_LOW                                                                         //CS
  SPI_MAG ->DR = address & 0x3F;
    while(!(SPI_MAG ->SR & SPI_SR_TXE));

    
    while(!(SPI_MAG ->SR & SPI_SR_TXE));
     SPI_MAG ->DR = *ptr;
     
 while(SPI_MAG ->SR & SPI_SR_BSY);    
 //SPI ->DR; 
 MAG_CS_HIGH           
}

//////////////Read Magnetometer data////////////////////////////////////////////
uint8_t LIS3MDL_Read_data_XYZ(uint16_t* data_ptr){
  STATUS_REG_t temp_state;
    uint8_t temp_data_L, temp_data_H;
  uint16_t temp_data,temp_rez;
  
  
  *(uint8_t*)((uint8_t*)&temp_state) =0;
  
  uint8_t read_status=2, addr = OUT_X_L_REG;
  
  LIS3MDL_reg_read(STATUS_REG,(uint8_t*)&temp_state);
  
  if(temp_state.ZYXDA == 0x1){
    for(uint8_t a_cnt=0;a_cnt < 3;a_cnt++ ){
      
    LIS3MDL_reg_read(addr + 2*a_cnt,(uint8_t*)&temp_data_L);
    LIS3MDL_reg_read(addr + 2*a_cnt + 1,(uint8_t*)&temp_data_H);
    
    temp_data = temp_data_L | (((uint16_t)temp_data_H) << 8);
    
  ////////////////////
  if( ( (temp_data & 0x8000) >> 15) > 0){
      temp_rez = ((~temp_data +1) &0x7FFF)|0x8000; 
     }
  else{
      temp_rez = temp_data;
      }
  *((uint16_t*)data_ptr + a_cnt) = temp_rez;
  ////////////////////////////
   
    }
    read_status=0;
  }
  else
    read_status=1;
  return read_status;
}
////////////////////////Write magnetometer settings////////////////////////////
void LIS3MDL_write_settings(uint8_t* settings_data ){
}
