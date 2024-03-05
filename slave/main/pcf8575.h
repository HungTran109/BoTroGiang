#ifndef __NODE_I2C_H__
#define __NODE_I2C_H__

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */


#define	I2C_MASTER_PORT_NUM 0				/* Port number for I2C Master device. */
#define I2C_MASTER_SCL_IO 18               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 5               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0			/*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000        	/*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0         /*!< I2C master doesn't need buffer */


#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define PCF8575_I2C_ADDR	0x40

esp_err_t pcf_i2c_init(void);
esp_err_t pcf_init(void);
esp_err_t pcf_i2c_read(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_rd, size_t size);
esp_err_t pcf_i2c_read_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size);
esp_err_t pcf_i2c_write(i2c_port_t i2c_num, uint8_t slave_addr, uint8_t *data_wr, size_t size);
esp_err_t pcf_i2c_write_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size);



#endif // __NODE_I2C_H__

