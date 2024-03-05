// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/******************************************************************************
                                   INCLUDES					    			 
******************************************************************************/
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_system.h"
#include "esp_log.h"


#include "pcf8575.h"
#include "version_ctrl.h"
/******************************************************************************
                                   GLOBAL VARIABLES					    			 
******************************************************************************/
// extern System_t xSystem;

/******************************************************************************
                                   GLOBAL FUNCTIONS					    			 
******************************************************************************/

/******************************************************************************
                                   DATA TYPE DEFINE					    			 
******************************************************************************/

/******************************************************************************
                                   PRIVATE VARIABLES					    			 
******************************************************************************/

/******************************************************************************
                                   LOCAL FUNCTIONS					    			 
******************************************************************************/

/******************************************************************************************/
/**
 * @brief 	: i2c master initialization
 * @param 	:
 * @author	:
 * @return 	:
 */
esp_err_t pcf_i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
	
    i2c_param_config(i2c_master_port, &conf);
	
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}


/******************************************************************************************/
/**
 * @brief 	: pcf8575 initialization
 * @param 	:
 * @author	:
 * @return 	:
 */
esp_err_t pcf_init(void)
{
	esp_err_t res;

//	if(pcf_i2c_init() == ESP_FAIL) {
//		ESP_LOGE("PCF", "Init I2C FAILED!!!");
//		return ESP_FAIL;
//	}
	
    //Khởi tạo pcf, cấu hình thanh ghi cho phép Input, output
    Int_t confReg;
	confReg.bytes[0] = 0x33;
	confReg.bytes[1] = 0;

	res = pcf_i2c_write(I2C_MASTER_NUM, PCF8575_I2C_ADDR, confReg.bytes, 2);

//	if(res == ESP_OK) {
//		uint8_t output[2] = {0xFF, 0xFF};
//		
//		pcf_i2c_write(I2C_MASTER_NUM, PCF8575_I2C_ADDR, output, 2);
		
//		output[0] = 0;
//		output[1] = 0;
//		pcf_i2c_write(I2C_MASTER_NUM, PCF8575_I2C_ADDR, output, 2);
//	}

	return res;
}


/**
 * @brief test code to read i2c slave device with registered interface
 * _______________________________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------|----------------------|--------------------|------|
 *
 */
esp_err_t pcf_i2c_read_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be read
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ), ACK_CHECK_EN);
    // send register we want to access
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
	
    // Send start again to read
    i2c_master_start(cmd);
    // now send device address (indicating read bit) & read data
    i2c_master_write_byte(cmd, i2c_addr | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
	
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}


/**
 * @brief test code to read esp-i2c-slave without register address
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
esp_err_t pcf_i2c_read(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	
	//send device address (indicating read) & read data
    i2c_master_write_byte(cmd, slave_addr | READ_BIT, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
	
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
    return ret;
}


/**
 * @brief Test code to write i2c slave device with registered interface
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 * ____________________________________________________________________________________
 * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------|----------------------|------|
 *
 */
esp_err_t pcf_i2c_write_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
{
	if (size == 0) {
		return ESP_FAIL;
	}

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
	
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, i2c_addr | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
	
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
    return ret;
}


/**
 * @brief Test code to write esp-i2c-slave without register address
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
esp_err_t pcf_i2c_write(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_wr, size_t size)
{
	if (size == 0) {
		return ESP_FAIL;
	}

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

	//send device address (indicating write bit)
    i2c_master_write_byte(cmd, slave_addr | WRITE_BIT, ACK_CHECK_EN);
	//send data want to write
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
	
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	
    return ret;
}


/**
 * @brief test code to operate on BH1750 sensor
 *
 * 1. set operation mode(e.g One time L-resolution mode)
 * _________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
 * --------|---------------------------|---------------------|------|
 * 2. wait more than 24 ms
 * 3. read data
 * ______________________________________________________________________________________
 * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
 * --------|---------------------------|--------------------|--------------------|------|
 */
//static esp_err_t pcf_i2c_read_di(uint8_t di_addr, uint8_t *data_h, uint8_t *data_l)
//{
//    int ret;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, di_addr << 1 | READ_BIT, ACK_CHECK_EN);
//    i2c_master_read_byte(cmd, data_h, ACK_VAL);
//    i2c_master_read_byte(cmd, data_l, NACK_VAL);
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 100 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//	
//    return ret;
//}



