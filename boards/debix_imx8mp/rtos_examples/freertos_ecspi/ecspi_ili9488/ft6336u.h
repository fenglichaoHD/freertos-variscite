#ifndef INC_GT9XX_H_
#define INC_GT9XX_H_

#include "fsl_common.h"
#include "fsl_debug_console.h"
// 触摸IC设备地址

//FT6336U芯片相关信息
typedef struct FT6336U_IC_INFO
{
    /* data */
    uint32_t    CPIPHER;
    uint16_t    LIB_VERSION;
    uint8_t     FIRMWARE_VERSION;
    uint8_t     VENDOR_ID;
}FT6336U_IC_INFO;


typedef struct ft6336u_touch_pos
{
    /* data */
    uint8_t     touch_num;   //触摸点的数量
    //第一个触摸点的x,y值
    uint16_t    touch0_x;
    uint16_t    touch0_y;
    //第二个触摸点的x,y值
    uint16_t    touch1_x;
    uint16_t    touch1_y;

}ft6336u_touch_pos_t;

//四种功耗模式
typedef enum POWER_MODE{
    P_ACTIVE = 0x00,
    p_MONITOR,
    P_STANDBY,
    P_HIBERNATE
}POWER_MODE;

typedef enum WORK_MODE{
    INFO_MODE = 0x00,           //InFO模式
    NORMAL_MODE,                //正常模式
    RESERVE,
    FACTORY_MODE,               //工厂模式
    AUTO_CALIBRATION_MODE       //自动校准模式
}WORK_MODE;


typedef enum SPEC_GESTURE_ID{
    slide_left      = 32,  //左滑
    slide_right     = 33,  //右滑
    slide_up        = 34,  //上滑
    slide_down      = 35,  //下滑
    double_click    = 36,  //双击

    //剩下特殊手势识别查看寄存器手册
}SPEC_GESTURE;

#define FT6336_ADDR               0x38

#define FT6336_ADDR_DEVICE_MODE   0x00
#define FT6336_ADDR_TD_STATUS     0x02

#define FT6336_ADDR_TOUCH1_EVENT  0x03
#define FT6336_ADDR_TOUCH1_ID     0x05
#define FT6336_ADDR_TOUCH1_XH     0x03
#define FT6336_ADDR_TOUCH1_XL     0x04
#define FT6336_ADDR_TOUCH1_YH     0x05
#define FT6336_ADDR_TOUCH1_YL     0x06

#define FT6336_ADDR_TOUCH2_EVENT  0x09
#define FT6336_ADDR_TOUCH2_ID     0x0B
#define FT6336_ADDR_TOUCH2_XH     0x09
#define FT6336_ADDR_TOUCH2_XL     0x0A
#define FT6336_ADDR_TOUCH2_YH     0x0B
#define FT6336_ADDR_TOUCH2_YL     0x0C

#define FT6336_ADDR_FIRMARE_ID    0xA6


//第一个触摸点
#define P1_XH           0x03        //bit7-bit6为第1点触摸事件标志，bit3-bit0为第1点的X坐标高4位
#define P1_XL           0x04        //第1点的X坐标低8位
#define P1_YH           0x05        //bit7-bit6为第1点触摸事件标志，bit3-bit0为第1点的X坐标高4位
#define P1_YL           0x06        //第1点的Y坐标低8位

//第二个触摸点
#define P2_XH           0x09        //bit7-bit6为第1点触摸事件标志，bit3-bit0为第1点的X坐标高4位
#define P2_XL           0x0A        //第1点的X坐标低8位
#define P2_YH           0x0B        //bit7-bit6为第1点触摸事件标志，bit3-bit0为第1点的X坐标高4位
#define P2_YL           0x0C        //第1点的Y坐标低8位

void IIC_Init(void);
void TP_FT6336_init(void);
unsigned char ft6336_write_byte(unsigned char addr,unsigned char reg,unsigned char data);
unsigned char ft6336_read_byte(unsigned char addr,unsigned char reg);
unsigned char ft6336_get_td_status(void);
unsigned char  ft6336u_read_touch_pos(ft6336u_touch_pos_t *touch_pos);

void ft6336_get_touch1_position(unsigned int *x,unsigned int *y);
void ft6336_get_touch2_position(unsigned int *x,unsigned int *y);


#endif /* INC_GT9XX_H_ */

