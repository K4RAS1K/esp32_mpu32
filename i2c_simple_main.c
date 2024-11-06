#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "MPU6050.h"
#include "esp_system.h"
#include <rom/ets_sys.h>


static const char *TAG = "i2c_MPU6050";

#define I2C_MASTER_SCL_IO           0  
#define I2C_MASTER_SDA_IO           21    
#define I2C_MASTER_NUM              0                          
#define I2C_MASTER_FREQ_HZ          100000                    
#define I2C_MASTER_TX_BUF_DISABLE   0                         
#define I2C_MASTER_RX_BUF_DISABLE   0                          
#define I2C_MASTER_TIMEOUT_MS       1000

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void I2C_WriteBuffer(uint8_t I2C_ADDRESS,uint8_t *aTxBuffer,uint8_t TXBUFFERSIZE){
	i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_DEFAULT_ADDRESS , aTxBuffer, TXBUFFERSIZE, 1000);
}

void I2C_ReadBuffer(uint8_t I2C_ADDRESS,uint8_t RegAddr,uint8_t *aTxBuffer,uint8_t TXBUFFERSIZE){
	I2C_WriteBuffer(I2C_ADDRESS,&RegAddr,1);	
	i2c_master_read_from_device(I2C_MASTER_NUM, (uint16_t)I2C_ADDRESS<<1, aTxBuffer, (uint16_t)TXBUFFERSIZE,(uint32_t)1000);
}
void MPU6050_GetAllData(int16_t *Data){
	uint8_t accelbuffer[14];
	
	//c 0x3B 14 след рег содержат данные
	I2C_ReadBuffer(MPU6050_ADDRESS_AD0_LOW,MPU6050_RA_ACCEL_XOUT_H,accelbuffer,14);
	
	//акселерометр 65 - 64
	for(int i = 0; i < 3; i++) {
		Data[i] =((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
	}
	//темпиратура 65 66
	//гироскоп 67 - 72
		for(int i = 4; i < 7; i++) {
		Data[i - 1] =((int16_t) ((uint16_t) accelbuffer[2 * i] << 8) + accelbuffer[2 * i + 1]);
	}
}

void MPU6050_Init(void) {
	uint8_t buffer[7];
	//инициализация i2c
	i2c_master_init();
	//вкл модуля
    buffer[0] = MPU6050_RA_PWR_MGMT_1;
    buffer[1] = 0x00;
    I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
    //конфиг гироскоскопа на 500
    buffer[0] = MPU6050_RA_GYRO_CONFIG;
    buffer[1] = 0x8;
    I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
    //конфиг аксель на 8
    buffer[0] = MPU6050_RA_ACCEL_CONFIG;
    buffer[1] = 0x10;
    I2C_WriteBuffer(MPU6050_ADDRESS_AD0_LOW,buffer,2);
}

void app_main(void)
{
	MPU6050_Init();
	while(1){
		int16_t mpu6050data[6];
		MPU6050_GetAllData(mpu6050data);
		ESP_LOGI(TAG, "0: %hd", mpu6050data[0]);
		ESP_LOGI(TAG, "1: %hd", mpu6050data[1]);
	    ESP_LOGI(TAG, "2: %hd", mpu6050data[2]);
	    ESP_LOGI(TAG, "3: %hd", mpu6050data[3]);
	    ESP_LOGI(TAG, "4: %hd", mpu6050data[4]);
	    ESP_LOGI(TAG, "5: %hd", mpu6050data[5]);
		ets_delay_us(3*1000000);	
	}
}
