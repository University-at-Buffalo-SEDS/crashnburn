#include "stm32g4xx_hal.h"
#include "accel.h"

static inline void accel_cs_low(void) {HAL_GPIO_WritePin(accel_CS_GPIO_Port, accel_CS_Pin, GPIO_PIN_RESET);}
static inline void accel_cs_high(void) {HAL_GPIO_WritePin(accel_CS_GPIO_Port, accel_CS_Pin, GPIO_PIN_SET);}

//write 1 byte to a register address
HAL_StatusTypeDef accel_write_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t data){
  uint8_t buffer[2] = {((reg) & ~0x80u), data};
  accel_cs_low(); //select accel chip
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, buffer, sizeof(buffer), 100);
  accel_cs_high();
  return status;
}

/* Single Byte Read from given register, must ignore dummy byte */
HAL_StatusTypeDef accel_read_reg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *data){
  if (!data) return HAL_ERROR;
  uint8_t reg_addr = reg | 0x80; //adding read bit to register address
  uint8_t tx_buffer[3] = {reg_addr, 0x00, 0x00};
  uint8_t rx_buffer[3];
  accel_cs_low();
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(hspi, tx_buffer, rx_buffer, 3, 100); 
  accel_cs_high();
  if (status == HAL_ERROR) return status;
  *data = rx_buffer[2]; //Select last byte as actual info, then store it where the parameters point to
  return status;
}

//Burst read function using auto-increment for BMI088
HAL_StatusTypeDef accel_read_buffer(SPI_HandleTypeDef *hspi, uint8_t start_reg, uint8_t *dst, uint16_t len){
  if (!dst || !len) return HAL_ERROR;
  uint8_t reg_addr = start_reg | 0x80;
  uint8_t dummy = 0x00;
  accel_cs_low();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hspi, &reg_addr, 1, HAL_MAX_DELAY);

  //DUMMY BYTE HANDLING
    if (status == HAL_OK) {
    status = HAL_SPI_TransmitReceive(hspi, &dummy, &dummy, 1, HAL_MAX_DELAY);
  }

  if (status == HAL_OK) status = HAL_SPI_Receive(hspi, dst, len, HAL_MAX_DELAY);
  accel_cs_high();
  return status;
}

//configure the accelerometer
HAL_StatusTypeDef accel_config(SPI_HandleTypeDef *hspi)
{
  accel_write_reg(hspi, accel_pwr_ctrl, accel_POWER_ON); //power on
  HAL_Delay(10)

  uint8_t data; 
  HAL_StatusTypeDef status = accel_read_reg(hspi, accel_chip_id_addr, *data);

  if (status != HAL_OK){
    return status;
  }

  if (data != 0x1E) {    //if the chip address is different than expcted, throw a fault. 
    return HAL_ERROR;
  }

  HAL_status = accel_write_reg(hspi, accel_reset_addr, 0xB6); //soft reset chip
  HAL_Delay(5);
  accel_write_reg(hspi, accel_range_addr, 0x03); //range set to ±24g
  HAL_Delay(1);
  accel_write_reg(hspi, accel_conf_addr, 0x28); //
  HAL_Delay(5);

  return HAL_OK;
}

//read the accelermoter axis data
HAL_StatusTypeDef accel_read(SPI_HandleTypeDef *hspi, accelData_t *accelData){
  HAL_StatusTypeDef status = accel_read_reg()
  uint8_t addresses[6] = {accel_x_lsb, accel_x_msb, accel_y_lsb, accel_y_msb, accel_z_lsb, accel_z_msb}
  int16_t accel_data[3];
  uint8_t lsb;
  uint16_t msb;
  for(i = 0; i < 6; i += 2 ){
    accel_read_reg(hspi, accel_x_lsb, *lsb);
    accel_read_reg(hspi, accel_x_msb, *msb);
    accel_data[] = ((msb << 8) | lsb)
  }
  return HAL_OK;
}


//USE CALLBACK INTERRUPT FOR READING DATA FROM ACCEL IN MAIN.c
//DMA IS FASTER, BYPASSES CPU TO WRITE AND READ DATA FROM MEMORY
//UART WRITE
//ADC READ
//volatile variables for interrupt routines, It means other process like a callback can change the var at anytime.
//INTERRUPTS ARE STORED OUTSIDE OF MAIN FUNCTION AND NEED SPECIAL VARS TO BE USED^