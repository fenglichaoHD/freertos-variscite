/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */

#include "freertos_ecspi_ili9488.h"
#include "ili9488.h"
#include "lvgl-src/demos/lv_demos.h"
#include "ft6336u.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/* Task priorities. */
#define LVGL_DEMO_PRIORITY          (configMAX_PRIORITIES - 3)
#define LV_TASK_HANDLER_PRIORITY    (configMAX_PRIORITIES - 4)
#define INIT_TASK_PRIORITY          (configMAX_PRIORITIES - 2)
#define SPI_TRANSFER_TASK_PRIORITY  (configMAX_PRIORITIES - 1)

#define MAX_LOG_LENGTH 1
#define MAX_QUEUE_ITEMS 128

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void ecspi_task(void *pvParameters);
static void draw_lvgl_ui(void *pvParameters);
static void init_task(void *pvParameters);
static void spi_init(void); 
static void i2c_init(void);                                                                       
static void lv_task_hander_task(void *pvParameters);


uint32_t masterTxData[ECSPI_TRANSFER_SIZE] = {0};
static ecspi_master_config_t masterConfig;
static ecspi_transfer_t masterXfer;
static ecspi_rtos_handle_t master_rtos_handle;


static QueueHandle_t spi_queue = NULL;

volatile uint32_t ulIdleCycleCount = 0UL;
TaskHandle_t xUITaskHandle = NULL;
TaskHandle_t xLVTaskHandle = NULL;


void spi_transaction_one_byte(uint8_t data)
{
    BaseType_t xStatus;
    uint32_t data_to_queue;

    data_to_queue = (uint32_t)data;

    xStatus = xQueueSend(spi_queue, &data_to_queue, portMAX_DELAY);
    if( xStatus != pdPASS )
    {
        PRINTF( "Could not send to the queue.\r\n" );
    }
}

void spi_transaction_array(uint8_t *data, uint16_t len)
{
    BaseType_t xStatus;
    uint32_t i;
    uint32_t data_to_queue;

    for(i = 0;i < len; i++)
    {
        data_to_queue = *(data+i);
        xStatus = xQueueSend(spi_queue, &data_to_queue, portMAX_DELAY);
        if( xStatus != pdPASS )
        {
            PRINTF( "Could not send to the queue from array call.\r\n" );
        }
    }
}



void ili9488_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_p)
{
    if(area->x2 < 0 || area->y2 < 0 || area->x1 > (LV_HOR_RES_MAX - 1) || area->y1 > (LV_VER_RES_MAX - 1)) {
        lv_disp_flush_ready(drv);
        return;
    }

    /* Truncate the area to the screen */
    int32_t act_x1 = area->x1 < 0 ? 0 : area->x1;
    int32_t act_y1 = area->y1 < 0 ? 0 : area->y1;
    int32_t act_x2 = area->x2 > LV_HOR_RES_MAX - 1 ? LV_HOR_RES_MAX - 1 : area->x2;
    int32_t act_y2 = area->y2 > LV_VER_RES_MAX - 1 ? LV_VER_RES_MAX - 1 : area->y2;

    uint8_t data[4];
    /* window horizontal */
    ili9488_cmd_data(0);
    spi_transaction_one_byte(0x2A);
    data[0] = act_x1 >> 8;
    data[1] = act_x1;
    data[2] = act_x2 >> 8;
    data[3] = act_x2;
    ili9488_cmd_data(1);
    spi_transaction_array(data, 4);

    /* window vertical */
    ili9488_cmd_data(0);
    spi_transaction_one_byte(0x2B);
    data[0] = act_y1 >> 8;
    data[1] = act_y1;
    data[2] = act_y2 >> 8;
    data[3] = act_y2;
    ili9488_cmd_data(1);
    spi_transaction_array(data, 4);

    ili9488_cmd_data(0);
    spi_transaction_one_byte(0x2C);

    // for(y = act_y1; y <= act_y2; y++) {
    //     ili9341_write_array(ILI9341_DATA_MODE, (uint8_t *)mybuf, len * 3);
    //     mybuf += w ;
    // }

	size_t size = (area->x2 - area->x1 + 1) * (area->y2 - area->y1 + 1);

	/*Convert ARGB to RGB is required (cut off A-byte)*/
	size_t i;
	lv_color32_t* tmp32 = (lv_color32_t*) color_p;
	lv_color24_t* tmp24 = (lv_color24_t*) color_p;

	for(i=0; i < size; i++) {
		tmp24[i].red = tmp32[i].ch.red;
		tmp24[i].green = tmp32[i].ch.green;
		tmp24[i].blue = tmp32[i].ch.blue;
	}
    ili9488_cmd_data(1);
    spi_transaction_array((uint8_t *)color_p, size * 3);

    lv_disp_flush_ready(drv);
}


void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
	ft6336u_touch_pos_t touch;		
	ft6336u_read_touch_pos(&touch); 

	if (!touch.touch_num) 
	{
		data->state = LV_INDEV_STATE_REL;
	}
	else
	{
		data->state = LV_INDEV_STATE_PR;
		/*Set the coordinates*/
		data->point.x = 480 - touch.touch0_y; 
		data->point.y = touch.touch0_x;
         PRINTF("touch x:%d y:%d\r\n",data->point.x,data->point.y);
	}
}

static void hal_init(void)
{
    static lv_disp_draw_buf_t draw_buf_dsc_2;
    static lv_color_t buf_2_1[320 * 10];                        /*A buffer for 10 rows*/
    static lv_color_t buf_2_2[480 * 10];                        /*An other buffer for 10 rows*/
    lv_disp_draw_buf_init(&draw_buf_dsc_2, buf_2_1, buf_2_2, 320 * 10);   /*Initialize the display buffer*/

    /*-----------------------------------
     * Register the display in LVGL
     *----------------------------------*/

    static lv_disp_drv_t disp_drv;                         /*Descriptor of a display driver*/
    lv_disp_drv_init(&disp_drv);                    /*Basic initialization*/

    /*Set up the functions to access to your display*/

    /*Set the resolution of the display*/
    disp_drv.hor_res = 320;
    disp_drv.ver_res = 480;

    /*Used to copy the buffer's content to the display*/
    disp_drv.flush_cb = ili9488_flush;


    /*Set a display buffer*/
    disp_drv.draw_buf = &draw_buf_dsc_2;

    /*Required for Example 3)*/
    //disp_drv.full_refresh = 1;

    /* Fill a memory array with a color if you have GPU.
     * Note that, in lv_conf.h you can enable GPUs that has built-in support in LVGL.
     * But if you have a different GPU you can use with this callback.*/
    //disp_drv.gpu_fill_cb = gpu_fill;

    /*Finally register the driver*/
    lv_disp_t * disp = lv_disp_drv_register(&disp_drv);

    lv_theme_t * th = lv_theme_default_init(disp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK, LV_FONT_DEFAULT);
    lv_disp_set_theme(disp, th);


	static lv_indev_drv_t indev_drv;
	lv_indev_drv_init(&indev_drv);
	indev_drv.type = LV_INDEV_TYPE_POINTER;
	indev_drv.read_cb = my_touchpad_read;
	lv_indev_drv_register(&indev_drv);


    lv_group_t * g = lv_group_create();
    lv_group_set_default(g);
}

/*!
 * @brief tick hook is executed every tick.
 */
void vApplicationTickHook(void)
{
    static uint32_t ulCount = 0;
    ulCount++;
    if (ulCount >= 2UL)
    {
        lv_tick_inc(2);   //calling every 2 milliseconds.
        ulCount = 0UL;
    }
}

static void set_temp(void * bar, int32_t temp)
{
    lv_bar_set_value(bar, temp, LV_ANIM_ON);
}


void lv_example_get_started_1(void)
{
    static lv_style_t style_indic;

    lv_style_init(&style_indic);
    lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indic, lv_palette_main(LV_PALETTE_RED));
    lv_style_set_bg_grad_color(&style_indic, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_dir(&style_indic, LV_GRAD_DIR_VER);

    lv_obj_t * bar = lv_bar_create(lv_scr_act());
    lv_obj_add_style(bar, &style_indic, LV_PART_INDICATOR);
    lv_obj_set_size(bar, 20, 200);
    lv_obj_center(bar);
    lv_bar_set_range(bar, -20, 40);

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_exec_cb(&a, set_temp);
    lv_anim_set_time(&a, 3000);
    lv_anim_set_playback_time(&a, 3000);
    lv_anim_set_var(&a, bar);
    lv_anim_set_values(&a, -20, 40);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_start(&a);


}



void lv_task_hander_task(void *pvParameters)
{
    TickType_t xLastWakeTime;
	const TickType_t xPeriod = pdMS_TO_TICKS( 5 );
    bool flag = false;
	
	xLastWakeTime = xTaskGetTickCount();  
	
	for(;;)
	{		
        lv_task_handler();
        if(!flag)
        {
            PRINTF("lv_task_hander is called for the first time.\r\n");
            flag = true;
        }
		vTaskDelayUntil( &xLastWakeTime,xPeriod );
	}
	vTaskDelete(NULL);
}


void init_task(void *pvParameters)
{
    vTaskSuspend(xUITaskHandle);  //suspend ui task untill init task finisded.
    vTaskSuspend(xLVTaskHandle);
    spi_init();
    i2c_init();
    ILI9488_Init();
    TP_FT6336_init();
    lv_init();
    hal_init();
    PRINTF("Init finised. resume xUI and XLV tasks\r\n");
    vTaskResume(xUITaskHandle);
    vTaskResume(xLVTaskHandle);
    //vTaskDelete( NULL );
    vTaskSuspend(NULL);
}


void draw_lvgl_ui(void *pvParameters)
{
    lv_example_get_started_1();
    //lv_demo_widgets();
    PRINTF("UI task finish.\r\n");
    //vTaskDelete( NULL );
    vTaskSuspend(NULL);
}


/***/
void vApplicationIdleHook( void )
{
    ulIdleCycleCount++;
    if( ulIdleCycleCount == 20000000UL)
    {
        PRINTF("idle task heart beat.\r\n");
        ulIdleCycleCount = 0UL;
    }
}

/* void vApplicationStackOverflowHook( TaskHandle_t xTask,
                                               char * pcTaskName )
{
    PRINTF("OOPS, STACK OVERFLOW\r\n");
} */


void spi_init(void)
{
    status_t status;
    
    ECSPI_MasterGetDefaultConfig(&masterConfig);

    masterConfig.baudRate_Bps   = ECSPI_TRANSFER_BAUDRATE;

    status = ECSPI_RTOS_Init(&master_rtos_handle, ECSPI_MASTER_BASEADDR, &masterConfig, ECSPI_MASTER_CLK_FREQ);

    if (status != kStatus_Success)
    {
        PRINTF("ECSPI meets error during initialization. \r\n");
        vTaskSuspend(NULL);
    }
    masterXfer.txData   = masterTxData;
    masterXfer.rxData   = NULL;
    //masterXfer.dataSize = ECSPI_TRANSFER_SIZE;
    masterXfer.channel  = ECSPI_MASTER_TRANSFER_CHANNEL;
}


void i2c_init(void)
{
    i2c_master_config_t   i2cConfig;
    CLOCK_SetRootMux(kCLOCK_RootI2c4, kCLOCK_I2cRootmuxSysPll1Div5); /* Set I2C source to SysPLL1 Div5 160MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootI2c4, 1U, 10U);                  /* Set root clock to 160MHZ / 10 = 16MHZ */

    I2C_MasterGetDefaultConfig(&i2cConfig);
    i2cConfig.baudRate_Bps = I2C_BAUDRATE;
    I2C_MasterInit(I2C_MASTER_BASEADDR, &i2cConfig, I2C_MASTER_CLK_FREQ);
}

/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    /* M7 has its local cache and enabled by default,
     * need to set smart subsystems (0x28000000 ~ 0x3FFFFFFF)
     * non-cacheable before accessing this address region */


    BOARD_InitMemory();

    /* Board specific RDC settings */
    BOARD_RdcInit();

    BOARD_InitBootPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();


    CLOCK_SetRootMux(kCLOCK_RootEcspi1, kCLOCK_EcspiRootmuxSysPll1); /* Set ECSPI1 source to SYSTEM PLL1 800MHZ */
    CLOCK_SetRootDivider(kCLOCK_RootEcspi1, 2U, 5U);                 /* Set root clock to 800MHZ / 10 = 80MHZ */
    /* Set IRQ priority for freertos_ecspi */
    NVIC_SetPriority(EXAMPLE_ECSPI_MASTER_IRQN, 2);

    PRINTF("\r\n***Debix iMX8MP LVGL Demo***\r\n");

    spi_queue = xQueueCreate(MAX_QUEUE_ITEMS, MAX_LOG_LENGTH);

    if (xTaskCreate(draw_lvgl_ui, "lvgl_demo_task", 2000, NULL, LVGL_DEMO_PRIORITY, &xUITaskHandle) !=
        pdPASS)
    {
        PRINTF("LVGL DEMO Task creation failed!.\r\n");
        while (1);
    }
     PRINTF("lvgl ui task created\r\n");
    if (xTaskCreate(lv_task_hander_task, "lvg_task_hander_task", 2000, NULL, LV_TASK_HANDLER_PRIORITY, &xLVTaskHandle) !=
        pdPASS)
    {
        PRINTF("LV TASK HANDER Task creation failed!.\r\n");
        while (1);
    }
      PRINTF("lv task hander task created\r\n");
    if (xTaskCreate(init_task, "init_task", 2000, NULL, INIT_TASK_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("INIT Task creation failed!.\r\n");
        while (1);
    }
        PRINTF("init task created\r\n");
    if (xTaskCreate(ecspi_task, "ecspi_task", 2000, NULL, SPI_TRANSFER_TASK_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("ecspi Task creation failed!.\r\n");
        while (1);
    }
        PRINTF("ecspi task created\r\n");
    vTaskStartScheduler();
    for(;;)
	{		
	}
}


/*!
 * @brief Task responsible for ecspi.
 */
static void ecspi_task(void *pvParameters)
{
    uint8_t data = 255;
    uint32_t count = 0;
    uint32_t left_count = 0;
    status_t status;
    while(1)
    {
        while(xQueueReceive(spi_queue, &data, portMAX_DELAY) == pdPASS)
        {
            masterTxData[count] = data;
            count++;
            //PRINTF("read queue-> %d: 0x%x\r\n",count, data);
            left_count = uxQueueMessagesWaiting( spi_queue );
            while(left_count)
            {
                xQueueReceive(spi_queue, &data, 0);
                masterTxData[count] = data;
                left_count--;
                count++;
                //PRINTF("read queue (left)-> %d: %d\r\n",count, data);
            }
            if(count)
            {
                break;
            }
        }

        masterXfer.dataSize = count;
        status = ECSPI_RTOS_Transfer(&master_rtos_handle, &masterXfer);
        if (status != kStatus_Success)
        {
            PRINTF("ECSPI transfer completed with error. \r\n\r\n");
            vTaskSuspend(NULL);
        }
        count = 0;
    }

    /* Deinit the ECSPI. */
    ECSPI_RTOS_Deinit(&master_rtos_handle);
    vTaskSuspend(NULL);
}

