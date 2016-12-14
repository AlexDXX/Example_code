#include "stm32f10x.h"
#include "MPU6500.h"
#include "LIS3MDL.h"
#include "stm32f10x_usart.h"

#define AVG_MAX 1
#define MAX_CNT 64000
/////////////////Function prototypes////////////////
uint8_t CALC_CHEKSUM (uint16_t length, uint8_t* ptr);
void SEND_UART_data(uint8_t* tx_buffer, uint8_t data_length);
////////////////Global variables definition////////
uint8_t receive_data[10];
uint16_t AVG_data[9];
uint8_t rx_counter1=0;
static uint8_t UART_TX_BUFFER[25] ={0xAA,0xBE,0x1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} ;
uint32_t avg_cnt =0;


SERIAL_PORT_msg_t SERIAL_PORT_msg;
 uint8_t read_status=1;
CTRL_REG1_t CTRL_REG1;
CTRL_REG2_t CTRL_REG2;
CTRL_REG3_t CTRL_REG3;
CTRL_REG4_t CTRL_REG4;

MPU6500_out_data_t MPU6500_out_data;
MPU6500_Config_t MPU6500_Config;
MPU6500_Config_t temp_read;
MPU6500_GyroSetiings_t MPU6500_GyroSetiings;
MPU6500_ACC_Setiings_t MPU6500_ACC_Setiings;
MPU6500_ACC_Setiings2_t MPU6500_ACC_Setiings2;
MPU6500_GYRO_Offset_t MPU6500_GYRO_Offset;
MPU6500_ACC_Offset_t MPU6500_ACC_Offset;
MPU6500_SelfTest_data_t MPU6500_SelfTest_data;


struct Magnetometer_RAW_data {
  uint16_t X_mag_data;
  uint16_t Y_mag_data;
  uint16_t Z_mag_data;
}Mag_data;
///////////////////////////////Main program/////////////////////////////////////
void main()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN 
               | RCC_APB2ENR_IOPAEN 
               | RCC_APB2ENR_IOPBEN
               | RCC_APB2ENR_SPI1EN;                                         //Enable peripheral clock for SPI1
  RCC->APB1ENR |=  RCC_APB1ENR_TIM2EN      
               | RCC_APB1ENR_SPI2EN;                                         //Enable peripheral clock for SPI2
  ///////////////////TIMER 1 config/////////////////////////////////////////////
  TIM2 ->DIER |= TIM_DIER_UIE;                                                  //Update interrupt enable
  TIM2 ->PSC  = 48000;                                                          //Set prescaler value
  TIM2 ->ARR  = 400;                                                            //Set preload register value
  TIM2->CR1  |= TIM_CR1_ARPE                                                    //Enable ARR register
             |  TIM_CR1_URS                                                     //Update request sourse
             |  TIM_CR1_CEN;                                                    //Enable timer
  
  //////////////////////////UART 1 CONFIG///////////////////////////////////////  
  RCC -> APB2ENR  |=  RCC_APB2ENR_USART1EN ;                                    //Enable UART Clock
  USART1 -> CR1 |= USART_CR1_UE ;                                               //Enable USART module
  USART1 ->CR1 &= ~ USART_CR1_M;                                                //Set 8 data bits 1 start bit
  USART1 ->CR2 &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);                       //Set 1 stop bit
  USART1 ->BRR  = 0x271;                                    //SET Baud rate
  USART1 ->CR1 |= USART_CR1_TE |USART_CR1_RE;                                   //Enable transmitter & receiver
  USART1 ->CR1 |= USART_CR1_RXNEIE;                                             //Rx interrupt enable                 
  //////UART1 pin config//////
  GPIOA ->CRH |= GPIO_CRH_MODE9;                                                //Set  out mode
  GPIOA ->CRH &= ~GPIO_CRH_CNF9;
  GPIOA ->CRH |= GPIO_CRH_CNF9_1;                                               //alternative function out push-pull
  
  ///////////////////UART2 config //////////////////////////////////////////////
  RCC -> APB1ENR  |=  RCC_APB1ENR_USART2EN ;                                    //Enable UART Clock
  USART2 -> CR1 |= USART_CR1_UE ;                                               //Enable USART module
  USART2 ->CR1 &= ~ USART_CR1_M;                                                //Set 8 data bits 1 start bit
  USART2 ->CR2 &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);                       //Set 1 stop bit
  //USART2 ->BRR  = (0x98 << 4)|((0x4)&0x0F );                                    //SET Baud rate
  USART2 ->BRR = 0x27;
  USART2 ->CR1 |= USART_CR1_TE |USART_CR1_RE;                                   //Enable transmitter & receiver
  USART2 ->CR1 |= USART_CR1_RXNEIE;                                             //Rx interrupt enable      
  //////UART2 pin config///////
  GPIOA ->CRL |= GPIO_CRL_MODE2;                                                //Set  out mode
  GPIOA ->CRL &= ~GPIO_CRL_CNF2;
  GPIOA ->CRL |= GPIO_CRL_CNF2_1;                                               //alternative function out push-pull
  
  ////////////////////UART3 config//////////////////////////////////////////////
  RCC -> APB1ENR  |=  RCC_APB1ENR_USART3EN ;                                    //Enable UART Clock
  USART3 -> CR1 |= USART_CR1_UE ;                                               //Enable USART module
  USART3 ->CR1 &= ~ USART_CR1_M;                                                //Set 8 data bits 1 start bit
  USART3 ->CR2 &= ~(USART_CR2_STOP_0 | USART_CR2_STOP_1);                       //Set 1 stop bit
  USART3 ->BRR  = 0x138 ;  
  USART3 ->CR1 |= USART_CR1_TE |USART_CR1_RE;                                   //Enable transmitter & receiver
  USART3 ->CR1 |= USART_CR1_RXNEIE;                                             //Rx interrupt enable          
  ///////UART3 pin config////////
  GPIOB ->CRH |= GPIO_CRH_MODE10;                                               //Set  out mode
  GPIOB ->CRH &= ~GPIO_CRH_CNF10;                                               //Reset CNF
  GPIOB ->CRH |= GPIO_CRH_CNF10_1;                                              //Alternative function out push-pull
  
  ////////////////////LED GPIO pin config///////////////////////////////////////
  GPIOB ->CRL |= GPIO_CRL_MODE7;                                                //Set out mode(LED2)
  GPIOB ->CRL &= ~GPIO_CRL_CNF7;                                                //Set out push-pull mode       
  
  GPIOB ->CRH |= GPIO_CRH_MODE8;                                                //Set out mode(LED 2)
  GPIOB ->CRH &= ~GPIO_CRH_CNF8;                                                //Reset CRL(push-pull)
  
  /////////////////Initial settings for magnetometer& gyroscope/////////////////
  GYRO_SPI_CONFIG();
  Magnetometer_SPI_CONFIG();
  ///////////////Magnetometer/////////////
  CTRL_REG1.OUT_DATA_RATE = DATA_RATE_80_HZ;
  CTRL_REG1.TEMP_EN = 0x0;
  CTRL_REG2.FULL_SCALE_CONFIG = FULL_SCALE_4_gauss; 
  CTRL_REG3.OPERATING_MODE_SEL = 0;
  CTRL_REG4.Zaxis_OPERATION_MODE = ULTRA_HIGH_PERFOMANCE_MODE;
  ////////////////////////////////////////
  LIS3MDL_reg_write(CTRL_REG_1,(uint8_t*)&CTRL_REG1);
  LIS3MDL_reg_write(CTRL_REG_2,(uint8_t*)&CTRL_REG2);
  LIS3MDL_reg_write(CTRL_REG_3,(uint8_t*)&CTRL_REG3);
  LIS3MDL_reg_write(CTRL_REG_4,(uint8_t*)&CTRL_REG4);
  ///////Gyroscope//////
  MPU6500_GyroSetiings.FCHOICE_Bypass = 0;
  MPU6500_GyroSetiings.GYRO_FULL_SCALE_SEL = GYRO_FULL_SCALE_250dps;
  MPU6500_GyroSetiings.GYRO_self_test_X= 1;
  MPU6500_GyroSetiings.GYRO_self_test_Y= 1;
  MPU6500_GyroSetiings.GYRO_self_test_Z= 1;
  MPU6500_Config.DLPF_CFG = 5;
  MPU6500_Config.EXT_SYNC_SET =0;
  MPU6500_Config.FIFO_mode =0;
  MPU6500_ACC_Setiings.ACC_FULL_SCALE_SEL = ACCEL_FULL_SCALE_4g;
  MPU6500_ACC_Setiings2.ACC_FCHOICE_B=0;
  MPU6500_ACC_Setiings2.A_DLPF_CFG = 0x5;
  ////////////////////////
  MPU6500_reg_write(CONFIG_REG, (uint8_t*)&MPU6500_Config);
  MPU6500_reg_write(GYRO_CONFIG, (uint8_t*)&MPU6500_GyroSetiings);
  MPU6500_reg_write(ACCEL_CONFIG, (uint8_t*)&MPU6500_ACC_Setiings) ; 
  MPU6500_reg_write(ACCEL_CONFIG_2, (uint8_t*)&MPU6500_ACC_Setiings2) ; 
  
  
  
 MPU6500_GyroSetiings.GYRO_self_test_X= 1;
  MPU6500_GyroSetiings.GYRO_self_test_Y= 0;
  MPU6500_GyroSetiings.GYRO_self_test_Z= 0;
  while(1){
 
    
 ///////////////////////////////////////////////////////   
    

 /////////////////////////////////////////////////////////////////////////////

    
   read_status = LIS3MDL_Read_data_XYZ((uint16_t*)&Mag_data);
  if(read_status == 0){
       READ_ACC_GYRO_data((uint16_t*)&MPU6500_out_data);
   ///////////////////////////////////////////////////
   AVG_data[0] = MPU6500_out_data.GYRO_X /131;
   AVG_data[1] = MPU6500_out_data.GYRO_Y/131 ;      
   AVG_data[2] = MPU6500_out_data.GYRO_Z/131 ;  
  ///////////////////////////////////////
   AVG_data[3]  = MPU6500_out_data.ACC_X;   
   AVG_data[4] = MPU6500_out_data.ACC_Y;  
   AVG_data[5] = MPU6500_out_data.ACC_Z;  
 ///////////////////////////////////////  
   AVG_data[6]  = Mag_data.X_mag_data ;   
   AVG_data[7] = Mag_data.Y_mag_data;
   AVG_data[8] = Mag_data.Z_mag_data;  
    read_status =2;
    
    for(uint8_t k=0; k < 9; k++){
      *(uint16_t*)(UART_TX_BUFFER + 3 + 2*k) =   AVG_data[k] / AVG_MAX;
    }
     SEND_UART_data((uint8_t*)&UART_TX_BUFFER, 22);
  // 
  }
    for(int t=0;t<100;t++){};
  
}
}
///////////////////////////////////////////////////////////////////////////////
void USART1_IRQHandler(void){
  if( (USART1 ->SR)& USART_SR_RXNE )
  {
    receive_data[rx_counter1++] = USART1->DR ;
  }
}

////////////////////////////////////////////////////////////////////////////////
uint8_t CALC_CHEKSUM (uint16_t length, uint8_t* ptr)
{
  uint8_t cheksum=0;
 for (int i= 0;i < length;i++)
   cheksum += *(ptr + i);
  
 return cheksum;
}

///////////////////////////////////////////////////////////////////////////////
void SEND_UART_data(uint8_t* tx_buffer, uint8_t data_length)
{ 
  *(uint8_t*)(tx_buffer) = 0xAA; 
  *(uint8_t*)(tx_buffer + 1) = 0xAE; 
  *(uint8_t*)(tx_buffer + 2) = 0x01; 
  *(uint8_t*)(tx_buffer + 21) = CALC_CHEKSUM (21,tx_buffer);
    for(int tx_counter=0; tx_counter < data_length;tx_counter++ ){
      USART1 ->DR = *(uint8_t*)(tx_buffer + tx_counter);                        //Load data to shift register        
    while((USART1->SR & USART_SR_TXE) == 0);                                    //Wait for data transmission
    }
 while(!(USART1->SR & USART_SR_TC)); 
}