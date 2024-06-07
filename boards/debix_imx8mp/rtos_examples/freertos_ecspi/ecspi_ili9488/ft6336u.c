#include "ft6336u.h"
#include <stdio.h>
#include "fsl_i2c.h"
#include "fsl_gpio.h"
#include "freertos_ecspi_ili9488.h"

	i2c_master_transfer_t i2cXfer;

gpio_pin_config_t int_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

unsigned char ft6336_write_byte(unsigned char addr,unsigned char reg,unsigned char data)
{
    status_t status;

    memset(&i2cXfer, 0, sizeof(i2cXfer));

    i2cXfer.slaveAddress   = addr;
    i2cXfer.direction      = kI2C_Write;
    i2cXfer.subaddress     = reg;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data           = &data;
    i2cXfer.dataSize       = 1;
    i2cXfer.flags          = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &i2cXfer);
	if (status != kStatus_Success)
    {
        PRINTF("I2C_MasterTransferBlocking meets error during \r\n");
    }
    return status;
}

unsigned char ft6336_read_byte(unsigned char addr,unsigned char reg)
{
    status_t status;
    unsigned char val;
	memset(&i2cXfer, 0, sizeof(i2cXfer));

    i2cXfer.slaveAddress   = addr;
    i2cXfer.direction      = kI2C_Read;
    i2cXfer.subaddress     = reg;
    i2cXfer.subaddressSize = 1;
    i2cXfer.data           = &val;
    i2cXfer.dataSize       = 1;

    i2cXfer.flags = kI2C_TransferDefaultFlag;

    status = I2C_MasterTransferBlocking(I2C_MASTER_BASEADDR, &i2cXfer);
	if (status != kStatus_Success)
    {
        PRINTF("I2C_MasterTransferBlocking meets error during \r\n");
    }
    return val;
}

unsigned char ft6336_get_td_status(void)
{
		unsigned char a;
		a=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TD_STATUS);
		return a;
}


void TP_FT6336_init(void)
{
}

unsigned char  ft6336u_read_touch_pos(ft6336u_touch_pos_t *touch_pos)
{
    //读取第一个点的数据
    uint8_t P1_XH_P = ft6336_read_byte(FT6336_ADDR,P1_XH);
    uint8_t P1_XL_P = ft6336_read_byte(FT6336_ADDR,P1_XL);
    uint8_t P1_YH_P = ft6336_read_byte(FT6336_ADDR,P1_YH);
    uint8_t P1_YL_P = ft6336_read_byte(FT6336_ADDR,P1_YL);
    //读取第二个点的数据
    uint8_t P2_XH_P = ft6336_read_byte(FT6336_ADDR,P2_XH);
    uint8_t P2_XL_P = ft6336_read_byte(FT6336_ADDR,P2_XL);
    uint8_t P2_YH_P = ft6336_read_byte(FT6336_ADDR,P2_YH);
    uint8_t P2_YL_P = ft6336_read_byte(FT6336_ADDR,P2_YL);
    //读取有效点的个数
    uint8_t tp_status = ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TD_STATUS);

    touch_pos->touch_num = tp_status;
    touch_pos->touch0_x = ((P1_XH_P & 0x0F) << 8) + P1_XL_P;
    touch_pos->touch0_y = ((P1_YH_P & 0x0F) << 8) + P1_YL_P;
    touch_pos->touch1_x = ((P2_XH_P & 0x0F) << 8) + P2_XL_P;
    touch_pos->touch1_y = ((P2_YH_P & 0x0F) << 8) + P2_YL_P;
	return 0;
}


void ft6336_get_touch1_position(unsigned int *x,unsigned int *y)
{
		unsigned char xh=0,xl=0,yh=0,yl=0;
		unsigned int tmp;
		xh=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH1_XH);
		xl=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH1_XL);
		yh=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH1_YH);
		yl=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH1_YL);
		*x=((xh&0x000F)<<8)|xl;
		*y=((yh&0x000F)<<8)|yl;

		tmp = *x;
		*x = 480 - *y;
		*y = tmp;
}

void ft6336_get_touch2_position(unsigned int *x,unsigned int *y)
{
		unsigned int xh=0,xl=0,yh=0,yl=0;
		xh=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH2_XH);
		xl=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH2_XL);
		yh=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH2_YH);
		yl=ft6336_read_byte(FT6336_ADDR,FT6336_ADDR_TOUCH2_YL);
		*x=((xh&0x000F)<<8)|xl;
		*y=((yh&0x000F)<<8)|yl;
}

