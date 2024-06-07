/**
  * @brief  ILI9488 Registers
  */

/* Includes ------------------------------------------------------------------*/
#include "ili9488.h"
#include "font.h"
#include "fsl_gpio.h"  
#include "fsl_ecspi.h"
#include "freertos_ecspi_ili9488.h"

#define ABS(x)   ((x) > 0 ? (x) : -(x))


#define GPIO_PAD        GPIO1
#define LCD_CMD_DATA    11U
#define LCD_RESET       12U
#define INT_GPIO        13U


gpio_pin_config_t lcd_cmd_data_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
gpio_pin_config_t lcd_reset_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};
gpio_pin_config_t ctp_int_pin = {kGPIO_DigitalOutput, 0, kGPIO_NoIntmode};

/* Global Variables ------------------------------------------------------------------*/
volatile uint16_t LCD_HEIGHT = ILI9488_SCREEN_HEIGHT;
volatile uint16_t LCD_WIDTH	 = ILI9488_SCREEN_WIDTH;
uint16_t POINT_COLOR = 0x0000, BACK_COLOR = 0xFFFF;
static uint8_t Direction;


static void HAL_Delay(int ms)
{
     SDK_DelayAtLeastUs(ms * 1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
}

void ili9488_cmd_data(uint8_t val)
{
    GPIO_PinWrite(GPIO_PAD, LCD_CMD_DATA, val);
}


void ili9488_reset(uint8_t val)
{
    GPIO_PinWrite(GPIO_PAD, LCD_RESET, val);
}

void ILI9488_GPIO_Init(void)
{
    GPIO_PinInit(GPIO_PAD, LCD_CMD_DATA, &lcd_cmd_data_pin);
    GPIO_PinInit(GPIO_PAD, LCD_RESET, &lcd_reset_pin);
    // GPIO_PinInit(GPIO5, 9U, &lcd_cs_pin);

    GPIO_PinWrite(GPIO_PAD, LCD_CMD_DATA, 1);
    GPIO_PinWrite(GPIO_PAD, LCD_RESET, 1);
}

uint8_t ILI9488_GetDirection(void)
{
	return Direction;
}

/*Send data (char) to LCD*/
void ILI9488_SPI_Send(unsigned char SPI_Data)
{
		spi_transaction_one_byte(SPI_Data);
}

void ILI9488_SPI_Send_Datas(unsigned char *SPI_Data,int len)
{
	 spi_transaction_array(SPI_Data,  len);
}

/* Send command (char) to LCD */
void ILI9488_Write_Command(uint8_t Command)
{
	ili9488_cmd_data(0);
	ILI9488_SPI_Send(Command);
}

/* Send Data (char) to LCD */
void ILI9488_Write_Data(uint8_t Data)
{
	ili9488_cmd_data(1);
	ILI9488_SPI_Send(Data);
}

/* Set Address - Location block - to draw into */
void ILI9488_Set_Address(uint16_t X1, uint16_t Y1, uint16_t X2, uint16_t Y2)
{
	ILI9488_Write_Command(0x2A);
	ILI9488_Write_Data(X1 >> 8);
	ILI9488_Write_Data(X1);
	ILI9488_Write_Data(X2 >> 8);
	ILI9488_Write_Data(X2);

	ILI9488_Write_Command(0x2B);
	ILI9488_Write_Data(Y1 >> 8);
	ILI9488_Write_Data(Y1);
	ILI9488_Write_Data(Y2 >> 8);
	ILI9488_Write_Data(Y2);

	ILI9488_Write_Command(0x2C);
}

/*HARDWARE RESET*/
void ILI9488_Reset(void)
{
	ili9488_reset(1);
	HAL_Delay(20);
	ili9488_reset(0);
	HAL_Delay(10);
	HAL_Delay(1000);
	ili9488_reset(1);
	HAL_Delay(120);
}

/*Ser rotation of the screen - changes x0 and y0*/
void ILI9488_Set_Rotation(uint8_t Rotation)
{
	Direction = Rotation;

	ILI9488_Write_Command(0x36);
	HAL_Delay(1);

	switch(Direction)
	{
		case SCREEN_VERTICAL_1:
			ILI9488_Write_Data((1<<3));
			LCD_WIDTH = 320;
			LCD_HEIGHT = 480;
			break;
		case SCREEN_HORIZONTAL_1:
			ILI9488_Write_Data((1<<6)|(1<<5)|(1<<4)|(1<<3));
			LCD_WIDTH  = 480;
			LCD_HEIGHT = 320;
			break;
		case SCREEN_VERTICAL_2:
			ILI9488_Write_Data((1<<7)|(1<<6)|(1<<3));
			LCD_WIDTH  = 320;
			LCD_HEIGHT = 480;
			break;
		case SCREEN_HORIZONTAL_2:
			ILI9488_Write_Data((1<<7)|(1<<5)|(1<<3));
			LCD_WIDTH  = 480;
			LCD_HEIGHT = 320;
			break;
		default:
			//EXIT IF SCREEN ROTATION NOT VALID!
			break;
	}
}


uint16_t ILI9488_GetWidth(void)
{
	return LCD_WIDTH;
}

uint16_t ILI9488_GetHeight(void)
{
	return LCD_HEIGHT;
}


/*Initialize LCD display*/
void ILI9488_Init(void)
{
	
	ILI9488_GPIO_Init();

	ILI9488_Reset();

	//SOFTWARE RESET
	ILI9488_Write_Command(0xE0); 
	ILI9488_Write_Data(0x00); 
	ILI9488_Write_Data(0x04); 
	ILI9488_Write_Data(0x0E); 
	ILI9488_Write_Data(0x08); 
	ILI9488_Write_Data(0x17); 
	ILI9488_Write_Data(0x0A); 
	ILI9488_Write_Data(0x40); 
	ILI9488_Write_Data(0x79); 
	ILI9488_Write_Data(0x4D); 
	ILI9488_Write_Data(0x07); 
	ILI9488_Write_Data(0x0E); 
	ILI9488_Write_Data(0x0A); 
	ILI9488_Write_Data(0x1A); 
	ILI9488_Write_Data(0x1D); 
	ILI9488_Write_Data(0x0F);  
	
	ILI9488_Write_Command(0xE1); 
	ILI9488_Write_Data(0x00); 
	ILI9488_Write_Data(0x1B); 
	ILI9488_Write_Data(0x1F); 
	ILI9488_Write_Data(0x02); 
	ILI9488_Write_Data(0x10); 
	ILI9488_Write_Data(0x05); 
	ILI9488_Write_Data(0x32); 
	ILI9488_Write_Data(0x34); 
	ILI9488_Write_Data(0x43); 
	ILI9488_Write_Data(0x02); 
	ILI9488_Write_Data(0x0A); 
	ILI9488_Write_Data(0x09); 
	ILI9488_Write_Data(0x33); 
	ILI9488_Write_Data(0x37); 
	ILI9488_Write_Data(0x0F); 

	ILI9488_Write_Command(0xC0); 
	ILI9488_Write_Data(0x18); 
	ILI9488_Write_Data(0x16); 
	
	ILI9488_Write_Command(0xC1); 
	ILI9488_Write_Data(0x41); 

	ILI9488_Write_Command(0xC5); 
	ILI9488_Write_Data(0x00); 
	ILI9488_Write_Data(0x22); 
	ILI9488_Write_Data(0x80); 

	ILI9488_Write_Command(0x36); 
	ILI9488_Write_Data(0x08); 

	ILI9488_Write_Command(0x3A); //Interface Mode Control
	ILI9488_Write_Data(0x66);


	ILI9488_Write_Command(0XB0);  //Interface Mode Control  
	ILI9488_Write_Data(0x00); 
	ILI9488_Write_Command(0xB1);   //Frame rate 70HZ  
	ILI9488_Write_Data(0xB0); 

	ILI9488_Write_Command(0xB4); 
	ILI9488_Write_Data(0x02);   

	ILI9488_Write_Command(0xB6); //RGB/MCU Interface Control
	ILI9488_Write_Data(0x02); 
	ILI9488_Write_Data(0x22); 

	ILI9488_Write_Command(0xE9); 
	ILI9488_Write_Data(0x00);
	
	ILI9488_Write_Command(0XF7);    
	ILI9488_Write_Data(0xA9); 
	ILI9488_Write_Data(0x51); 
	ILI9488_Write_Data(0x2C); 
	ILI9488_Write_Data(0x82);

	ILI9488_Write_Command(0x11); 
	HAL_Delay(120); 
	ILI9488_Write_Command(0x29); 
	HAL_Delay(20); 

	ILI9488_Set_Rotation(SCREEN_VERTICAL_1);
}

//INTERNAL FUNCTION OF LIBRARY, USAGE NOT RECOMENDED, USE Draw_Pixel INSTEAD
/*Sends single pixel colour information to LCD*/
void ILI9488_Draw_Colour(uint16_t Colour)
{
//SENDS COLOUR
	//unsigned char TempBuffer[3] = {((Colour>>8)&0xF8), ((Colour>>3)&0xFC), (Colour<<3)};
	unsigned char TempBuffer[3];
	TempBuffer[0] = ((Colour>>8)&0xF8);
	TempBuffer[1] = (Colour>>3)&0xFC;
	TempBuffer[2] = (Colour<<3);

	ili9488_cmd_data(1);
	ILI9488_SPI_Send_Datas(TempBuffer, 3);
}

//INTERNAL FUNCTION OF LIBRARY
/*Sends block colour information to LCD*/
void ILI9488_Draw_Colour_Burst(uint16_t Colour, uint32_t Size)
{
//SENDS COLOUR
	uint32_t j;
	uint32_t Buffer_Size = 0;
	unsigned char Byte1, Byte2, Byte3; 
	unsigned char burst_buffer[BURST_MAX_SIZE];
	uint32_t Sending_Size;
	uint32_t Sending_in_Block;
	uint32_t Remainder_from_block;

	if((Size * 3) < BURST_MAX_SIZE)
	{
		Buffer_Size = Size;
	}
	else
	{
		Buffer_Size = BURST_MAX_SIZE;
	}

	ili9488_cmd_data(1);

	Byte1 = (Colour>>8)&0xF8;	//RED
	Byte2 = (Colour>>3)&0xFC;	//GREEN
	Byte3 = (Colour<<3);			//BLUE
	

	for(j = 0; j < Buffer_Size; j+=3)
		{
			burst_buffer[j] = 	Byte1;
			burst_buffer[j+1] = Byte2;
			burst_buffer[j+2] = Byte3;
		}
	
	Sending_Size = Size * 3;
	Sending_in_Block = Sending_Size / Buffer_Size;
	Remainder_from_block = Sending_Size % Buffer_Size;

	if(Sending_in_Block != 0)
	{
		for(j = 0; j < (Sending_in_Block); j++)
		{
            ILI9488_SPI_Send_Datas((unsigned char *)burst_buffer, Buffer_Size);
		}
	}

	//REMAINDER!
     ILI9488_SPI_Send_Datas((unsigned char *)burst_buffer, Remainder_from_block);

}

//FILL THE ENTIRE SCREEN WITH SELECTED COLOUR (either #define-d ones or custom 16bit)
/*Sets address (entire screen) and Sends Height*Width ammount of colour information to LCD*/
void ILI9488_Fill_Screen(uint16_t Colour)
{
	ILI9488_Set_Address(0,0,LCD_WIDTH,LCD_HEIGHT);
	ILI9488_Draw_Colour_Burst(Colour, LCD_WIDTH * LCD_HEIGHT);
}

//DRAW PIXEL AT XY POSITION WITH SELECTED COLOUR
//
//Location is dependant on screen orientation. x0 and y0 locations change with orientations.
//Using pixels to draw big simple structures is not recommended as it is really slow
//Try using either rectangles or lines if possible
//
void ILI9488_Draw_Pixel(uint16_t X,uint16_t Y,uint16_t Colour) 
{
	unsigned char Temp_Buffer[4] ;
	unsigned char Temp_Buffer1[4] ;
	unsigned char Temp_Buffer2[3] ;

	Temp_Buffer[0] =  X>>8 ;
	Temp_Buffer[1] =  X ;
	Temp_Buffer[2] = (X+1)>>8 ;
	Temp_Buffer[3] = (X+1);

	Temp_Buffer1[0] = Y>>8 ;
	Temp_Buffer1[1] = Y ;
	Temp_Buffer1[2] = (Y+1)>>8 ;
	Temp_Buffer1[3] = (Y+1) ;

	Temp_Buffer2[0] = (Colour>>8)&0xF8;
	Temp_Buffer2[1] = (Colour>>3)&0xFC;
	Temp_Buffer2[2] = Colour<<3;
 
	if((X >=LCD_WIDTH) || (Y >=LCD_HEIGHT)) return;	//OUT OF BOUNDS!

	//ADDRESS
	ili9488_cmd_data(0);
	ILI9488_SPI_Send(0x2A);
	ili9488_cmd_data(1);


	//XDATA
  //Temp_Buffer[4] = {X>>8, X, (X+1)>>8, (X+1)};
	
     ILI9488_SPI_Send_Datas((unsigned char *)Temp_Buffer, 4);

	//ADDRESS
	ili9488_cmd_data(0);
	ILI9488_SPI_Send(0x2B);
	ili9488_cmd_data(1);


	//YDATA

	//unsigned char Temp_Buffer1[4] = {Y>>8,Y, (Y+1)>>8, (Y+1)};
     ILI9488_SPI_Send_Datas((unsigned char *)Temp_Buffer1, 4);
	
	//ADDRESS
	ili9488_cmd_data(0);
	ILI9488_SPI_Send(0x2C);
	ili9488_cmd_data(1);

	//COLOUR

	//unsigned char Temp_Buffer2[3] = {(Colour>>8)&0xF8, (Colour>>3)&0xFC, Colour<<3};
    ILI9488_SPI_Send_Datas((unsigned char *)Temp_Buffer2, 3);
}

//DRAW RECTANGLE OF SET SIZE AND HEIGTH AT X and Y POSITION WITH CUSTOM COLOUR
//
//Rectangle is hollow. X and Y positions mark the upper left corner of rectangle
//As with all other draw calls x0 and y0 locations dependant on screen orientation
//

void ILI9488_Draw_Fill_Rectangle(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint16_t Colour)
{
	if((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT)) return;

	if((X + Width - 1) >= LCD_WIDTH)
	{
			Width = LCD_WIDTH - X;
	}
	if((Y + Height - 1) >= LCD_HEIGHT)
	{
			Height = LCD_HEIGHT - Y;
	}

	ILI9488_Set_Address(X, Y, X + Width - 1, Y + Height - 1);
	ILI9488_Draw_Colour_Burst(Colour, Height * Width);
}

//DRAW LINE FROM X,Y LOCATION to X+Width,Y LOCATION
void ILI9488_Draw_Horizontal_Line(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Colour)
{
	if((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT)) return;

	if((X+Width-1)>=LCD_WIDTH)
	{
			Width=LCD_WIDTH-X;
	}
	ILI9488_Set_Address(X, Y, X+Width-1, Y);
	ILI9488_Draw_Colour_Burst(Colour, Width);
}

//DRAW LINE FROM X,Y LOCATION to X,Y+Height LOCATION
void ILI9488_Draw_Vertical_Line(uint16_t X, uint16_t Y, uint16_t Height, uint16_t Colour)
{
	if((X >= LCD_WIDTH) || (Y >= LCD_HEIGHT)) return;
	if((Y+Height-1)>=LCD_HEIGHT)
	{
			Height=LCD_HEIGHT-Y;
	}
	ILI9488_Set_Address(X, Y, X, Y+Height-1);
	ILI9488_Draw_Colour_Burst(Colour, Height);
}

/* ==========================================================================================================================
 *
 */
void ILI9488_Draw_line(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,uint16_t color)
{
  uint16_t t;
  int xerr = 0, yerr = 0, delta_x, delta_y, distance;
  int incx, incy, uRow, uCol;

  delta_x = x2 - x1;
  delta_y = y2 - y1;
  uRow = x1;
  uCol = y1;

  if(delta_x > 0) incx = 1;
  else if(delta_x == 0) incx = 0;
  else
  {
    incx = -1;
    delta_x = -delta_x;
  }

  if(delta_y > 0) incy = 1;
  else if(delta_y == 0) incy=0;
  else
  {
    incy = -1; delta_y = -delta_y;
  }

  if( delta_x > delta_y) distance = delta_x;
  else distance = delta_y;

  for(t = 0; t <= distance + 1; t++ )
  {
  	ILI9488_Draw_Pixel(uRow, uCol, color);
    xerr += delta_x ;
    yerr += delta_y ;
    if(xerr > distance)
    {
      xerr -= distance;
      uRow += incx;
    }
    if(yerr > distance)
    {
      yerr -= distance;
      uCol += incy;
    }
  }
}

void ILI9488_Draw_Rectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
  ILI9488_Draw_line(x1,y1,x1,y2,color);
  ILI9488_Draw_line(x1,y2,x2,y2,color);
  ILI9488_Draw_line(x2,y2,x2,y1,color);
  ILI9488_Draw_line(x2,y1,x1,y1,color);
}

void ILI9488_Draw_Circle(uint16_t x, uint16_t y, uint16_t radian, uint16_t color)
{
  int a, b;
  int di;

  a = 0;
  b = radian;
  di = 3 - (radian << 1);

  while(a <= b)
  {
  	ILI9488_Draw_Pixel(x-b,y-a,color); //3
  	ILI9488_Draw_Pixel(x+b,y-a,color); //0
  	ILI9488_Draw_Pixel(x-a,y+b,color); //1
  	ILI9488_Draw_Pixel(x-b,y-a,color); //7
  	ILI9488_Draw_Pixel(x-a,y-b,color); //2
  	ILI9488_Draw_Pixel(x+b,y+a,color); //4
  	ILI9488_Draw_Pixel(x+a,y-b,color); //5
  	ILI9488_Draw_Pixel(x+a,y+b,color); //6
  	ILI9488_Draw_Pixel(x-b,y+a,color);
    a++;
    //Bresenham
    if(di<0)
    {
      di +=4*a+6;
    }
    else
    {
      di+=10+4*(a-b);
      b--;
    }
    ILI9488_Draw_Pixel(x+a,y+b,color);
  }
}

void ILI9488_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	/* Draw lines */
	ILI9488_Draw_line(x1, y1, x2, y2, color);
	ILI9488_Draw_line(x2, y2, x3, y3, color);
	ILI9488_Draw_line(x3, y3, x1, y1, color);
}

void ILI9488_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t color)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	} else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	} else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay){
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	} else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		ILI9488_Draw_line(x, y, x3, y3, color);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

void ILI9488_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	ILI9488_Draw_Pixel(x0, y0 + r, color);
	ILI9488_Draw_Pixel(x0, y0 - r, color);
	ILI9488_Draw_Pixel(x0 + r, y0, color);
	ILI9488_Draw_Pixel(x0 - r, y0, color);
	ILI9488_Draw_Horizontal_Line(x0 - r, y0, x0 + r, color);

	while (x < y)
	{
		if (f >= 0)
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		ILI9488_Draw_Horizontal_Line(x0 - x, y0 + y, x0 + x, color);
		ILI9488_Draw_Horizontal_Line(x0 + x, y0 - y, x0 - x, color);

		ILI9488_Draw_Horizontal_Line(x0 + y, y0 + x, x0 - y, color);
		ILI9488_Draw_Horizontal_Line(x0 + y, y0 - x, x0 - y, color);
	}
}

void ILI9488_SetTextColor(uint16_t color)
{
	POINT_COLOR = color;
}

void ILI9488_SetBackgroundColor(uint16_t color)
{
	BACK_COLOR = color;
}

void ILI9488_Putchar(uint16_t x,uint16_t y,uint8_t num,uint8_t mode)
{
  uint8_t temp;
  uint8_t pos,t;
  uint16_t colortemp = POINT_COLOR;

  if(x > (LCD_WIDTH - 8) || y > LCD_HEIGHT - 16) return;
  num=num-' ';

  if(!mode)
  {
    for(pos = 0; pos < 16; pos++)
    {
      temp = asc2_1608[(uint16_t)num*16 + pos];
      for(t = 0; t < 8; t++)
      {
        if(temp&0x01)ILI9488_Draw_Pixel(x+t,y+pos,POINT_COLOR);
        else ILI9488_Draw_Pixel(x+t,y+pos,BACK_COLOR);
        temp>>=1;
      }
    }
  }
  else
  {
    for(pos=0;pos<16;pos++)
    {
      temp=asc2_1608[(uint16_t)num*16+pos];
      for(t=0;t<8;t++)
      {
        if(temp&0x01)ILI9488_Draw_Pixel(x+t,y+pos,POINT_COLOR);
        temp>>=1;
      }
    }
  }

  POINT_COLOR=colortemp;
}

void ILI9488_Putchar14x24(uint16_t x,uint16_t y,uint8_t data,uint8_t mode)
{
  uint8_t i,j,k,temp;
  if((x>LCD_WIDTH-14)||(y>LCD_HEIGHT-24)) return;

  for(i=0;i<24/8;i++)
  {
    for(j=0;j<8;j++)
    {
      for(k=0;k<14;k++)
      {
        temp=Consolas14x24[(data-' ')*(24/8)*14+k*(24/8)+i];
        if(mode==TFT_STRING_MODE_BACKGROUND)
        {
          if(temp&(0x01<<j))
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),POINT_COLOR);
          }
          else
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),BACK_COLOR);
          }
        }
        else
        {
          if(temp&(0x01<<j))
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),POINT_COLOR);
          }
        }
      }
    }
  }
}

void ILI9488_Putchar18x32(uint16_t x,uint16_t y,uint8_t data,uint8_t mode)
{
  uint8_t i,j,k,temp;
  if((x>LCD_WIDTH-18)||(y>LCD_HEIGHT-32)) return;

  for(i=0;i<32/8;i++)
  {
    for(j=0;j<8;j++)
    {
      for(k=0;k<18;k++)
      {
        temp=Consolas18x32[(data-' ')*(32/8)*18+k*(32/8)+i];
        if(mode==TFT_STRING_MODE_BACKGROUND)
        {
          if(temp&(0x01<<j))
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),POINT_COLOR);
          }
          else
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),BACK_COLOR);
          }
        }
        else
        {
          if(temp&(0x01<<j))
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),POINT_COLOR);
          }
        }
      }
    }
  }
}

void ILI9488_Putchar26x48(uint16_t x,uint16_t y,uint8_t data,uint8_t mode)
{
  uint8_t i,j,k,temp;
  if((x>LCD_WIDTH-26)||(y>LCD_HEIGHT-48)) return;

  for(i=0;i<48/8;i++)
  {
    for(j=0;j<8;j++)
    {
      for(k=0;k<26;k++)
      {
        temp=Consolas26x48[(data-' ')*(48/8)*26+k*(48/8)+i];
        if(mode==TFT_STRING_MODE_BACKGROUND)
        {
          if(temp&(0x01<<j))
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),POINT_COLOR);
          }
          else
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),BACK_COLOR);
          }
        }
        else
        {
          if(temp&(0x01<<j))
          {
          	ILI9488_Draw_Pixel(x+k,y+(8*i+j),POINT_COLOR);
          }
        }
      }
    }
  }
}


void ILI9488_Puts8x16(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE)
{
  uint8_t i=0;
  uint8_t font_w=8;
  uint8_t font_h=16;

  while(*(string+i)!='\0')
  {

    if(*(string+i)==0)
    {
      return;
    }

    if(*(string+i)=='\n')
    {
      y += font_h;
      x = 0;
      string++;
    }

    if(x > LCD_WIDTH - font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > LCD_HEIGHT - font_h)
    {
      x = y = 0;
    }

    ILI9488_Putchar(x, y,*(string+i),TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}

void ILI9488_Puts14x24(uint16_t x, uint16_t y, uint8_t *string,uint8_t TFT_STRING_MODE)
{
  uint8_t i=0;
  uint8_t font_w=14;
  uint8_t font_h=24;

  while(*(string+i)!='\0')
  {

    if(*(string+i)==0)
    {
      return;
    }

    if(*(string+i)=='\n')
    {
      y += font_h;
      x = 0;
      string++;
    }

    if(x > LCD_WIDTH-font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > LCD_HEIGHT-font_h)
    {
      x = y = 0;
    }

    ILI9488_Putchar14x24(x, y, *(string+i), TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}

void ILI9488_Puts18x32(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE)
{
  uint8_t i=0;
  uint8_t font_w=18;
  uint8_t font_h=32;

  while(*(string+i)!='\0')
  {

    if(*(string+i)==0)
    {
      return;
    }

    if(*(string+i)=='\n')
    {
      x += font_h;
      y = 0;
      string++;
    }

    if(x > LCD_WIDTH-font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > LCD_HEIGHT-font_h)
    {
      x = y = 0;
    }

    ILI9488_Putchar18x32(x, y, *(string+i),TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}

void ILI9488_Puts26x48(uint16_t x, uint16_t y, uint8_t *string, uint8_t TFT_STRING_MODE)
{
  uint8_t i = 0;
  uint8_t font_w = 26;
  uint8_t font_h = 48;

  while(*(string + i) != '\0')
  {
    if(*(string + i) == 0)
    {
      return;
    }

    if(*(string + i) == '\n')
    {
      y += font_h;
      x = 0;
      string++;
    }

    if(x > LCD_WIDTH - font_w)
    {
      x = 0;
      y += font_h;
    }

    if(y > LCD_HEIGHT - font_h)
    {
      x = y = 0;
    }

    ILI9488_Putchar26x48(x, y, *(string+i), TFT_STRING_MODE);
    x += font_w;
    i++;
  }
}
